// Data/solver-contract checks, independent of the UI. --synthetic needs no QP run.
// The default also checks the published scenario package against physics.json.
// --rebuild additionally solves all scenarios and compares deterministic data.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import {
  SENSITIVITY_POSES, SENSITIVITY_SCENARIOS, DEFAULT_SENSITIVITY_PATH,
  assembleSensitivity, buildSensitivity, sourceHashes,
} from './physics_sensitivity_build.mjs';

const args = process.argv.slice(2), ROOT = new URL('./', import.meta.url);
const REACTION_FIELDS = { bw: 'bw', forceN: 'F', compressionN: 'comp', shearN: 'shear' };

function fixture() {
  return Object.fromEntries(SENSITIVITY_POSES.map(pose => [pose, Object.fromEntries(Object.keys(SENSITIVITY_SCENARIOS).map(id => {
    const cue = id === 'cue';
    return [id, {
      muscles: { example: { a: cue ? .125 : .1, activation: cue ? .4 : .32, F: cue ? 125 : 100, Fmax: 1000 } },
      joints: { ls: { jr: { bw: cue ? 1.4 : 1.2, F: cue ? 980 : 840, comp: cue ? 900 : 800, shear: cue ? 100 : 80 } }, st_L: { st: { F: 50 } } },
      diag: { quality: { status: 'solved', converged: true, finite: true, physicalBalanceVerified: true, forceResidualN: .001, momentResidualNm: .001, iterations: 10, ms: 1234 },
        physicalBalance: { verified: true, assemblyVerified: true, maxAssemblyDifference: 1e-12, forceResidualN: .001, momentResidualNm: .001, forceToleranceN: .01, momentToleranceNm: .01 } },
      balance: { weightN: 700, forceN: [0, 700, 0], forceErrorN: [0, 0, 0], momentErrorNm: [0, 0, 0] },
    }];
  }))]));
}

const input = fixture(), copy = structuredClone(input), synthetic = assembleSensitivity(input, { synthetic: true });
assert.deepEqual(input, copy, 'assembling scenarios must not modify solver results');
assert.deepEqual(synthetic.poses.dog.muscles.example.a, { cue: .125, noCue: .1, range: [.1, .125] });
assert.deepEqual(synthetic.poses.dog.muscles.example.activation, { cue: .4, noCue: .32, range: [.32, .4] });
assert(!('st_L' in synthetic.poses.dog.joints), 'scapular proxy is not a joint reaction');
assert(!('ms' in synthetic.poses.dog.quality.cue), 'elapsed time cannot enter deterministic numeric evidence');
const missing = fixture(); delete missing.dog.noCue.muscles.example.activation;
assert.throws(() => assembleSensitivity(missing, {}), /Non-finite/);
const failed = fixture(); failed.dog.cue.diag.quality.converged = false;
assert.throws(() => assembleSensitivity(failed, {}), /Unverified/);
const falseAssembly = fixture(); falseAssembly.dog.cue.diag.physicalBalance.maxAssemblyDifference = .1;
assert.throws(() => assembleSensitivity(falseAssembly, {}), /Unverified physical force assembly/);
const missingPhysical = fixture(); delete missingPhysical.dog.cue.diag.physicalBalance;
assert.throws(() => assembleSensitivity(missingPhysical, {}), /Unverified physical force assembly/);
const falsePhysicalTolerance = fixture(); falsePhysicalTolerance.dog.cue.diag.physicalBalance.forceResidualN = 1;
assert.throws(() => assembleSensitivity(falsePhysicalTolerance, {}), /Physical equilibrium exceeds tolerance/);
const imbalance = fixture(); imbalance.dog.cue.balance.forceN[1] -= 1;
assert.throws(() => assembleSensitivity(imbalance, {}), /Inconsistent floor/);
imbalance.dog.cue.balance.forceErrorN[1] = -1;
assert.throws(() => assembleSensitivity(imbalance, {}), /mass equilibrium/);
const calls = [];
const mocked = await buildSensitivity({ generatedAtUTC: '2026-10-23T00:00:00.000Z', buildPoseFn: (pose, options) => {
  calls.push([pose, options.cues]);
  return input[pose][options.cues ? 'cue' : 'noCue'];
} });
assert.deepEqual(calls, SENSITIVITY_POSES.flatMap(pose => [[pose, true], [pose, false]]), 'each stored value needs its own explicitly selected scenario solve');
assert.deepEqual(mocked.poses, synthetic.poses);

function checkPackage(data, baseline) {
  assert.equal(data.v, 1); assert.equal(data.kind, 'scenario-range');
  assert.deepEqual(data.scope, synthetic.scope);
  assert.deepEqual(data.scenarios, SENSITIVITY_SCENARIOS);
  assert.deepEqual(Object.keys(data.poses), SENSITIVITY_POSES);
  assert.deepEqual(data.provenance.sourceSha256, sourceHashes(), 'sensitivity package is stale for the current numerical sources/baseline');
  assert(Number.isFinite(Date.parse(data.provenance.generatedAtUTC)), 'record the build time as provenance');
  let metrics = 0;
  function metric(m) {
    assert.deepEqual(Object.keys(m).sort(), ['cue', 'noCue', 'range']);
    assert(Number.isFinite(m.cue) && Number.isFinite(m.noCue));
    assert.deepEqual(m.range, [Math.min(m.cue, m.noCue), Math.max(m.cue, m.noCue)], 'a scenario range may not invent extrema');
    for (const value of [m.cue, m.noCue]) assert(value >= m.range[0] && value <= m.range[1]);
    metrics++;
  }
  for (const pose of SENSITIVITY_POSES) {
    const entry = data.poses[pose], ref = baseline[pose];
    assert(ref, 'published pose missing: ' + pose);
    assert.deepEqual(Object.keys(entry.muscles).sort(), Object.keys(ref.muscles).sort());
    assert.deepEqual(Object.keys(entry.joints).sort(), Object.entries(ref.joints).filter(([, joint]) => joint.jr).map(([id]) => id).sort(), 'all published joint reactions need scenario coverage');
    for (const [unit, fields] of Object.entries(entry.muscles)) {
      for (const m of Object.values(fields)) metric(m);
      for (const [field, original] of [['a', 'a'], ['activation', 'activation'], ['forceN', 'F'], ['maxForceN', 'Fmax']])
        assert.equal(fields[field].cue, ref.muscles[unit][original], `${pose}/${unit}/${field}: baseline differs from physics.json`);
      for (const scenario of Object.keys(data.scenarios)) {
        assert(fields.a[scenario] >= 0 && fields.a[scenario] <= 1);
        assert(fields.activation[scenario] >= 0 && fields.activation[scenario] <= 1);
        assert(fields.forceN[scenario] >= 0 && fields.maxForceN[scenario] > 0);
      }
    }
    for (const [id, fields] of Object.entries(entry.joints)) {
      const expectedFields = Object.entries(REACTION_FIELDS).filter(([, original]) => ref.joints[id].jr[original] !== undefined).map(([field]) => field).sort();
      assert.deepEqual(Object.keys(fields).sort(), expectedFields, 'all published reaction components need scenario coverage');
      for (const [field, m] of Object.entries(fields)) {
        metric(m);
        assert.equal(m.cue, ref.joints[id].jr[REACTION_FIELDS[field]], `${pose}/${id}/${field}: baseline reaction differs`);
      }
    }
    assert.deepEqual(entry.floor.cue, ref.balance, `${pose}: baseline floor balance differs`);
  }
  // Reuse the acceptance contract with the actual per-scenario diagnostics;
  // published floor groups are deliberately not used for mass balance.
  const replay = Object.fromEntries(SENSITIVITY_POSES.map(id => [id, Object.fromEntries(Object.keys(data.scenarios).map(scenario => [scenario, {
    muscles: {}, joints: {}, diag: { quality: data.poses[id].quality[scenario], physicalBalance: data.poses[id].physicalBalance[scenario] }, balance: data.poses[id].floor[scenario],
  }]))]));
  assembleSensitivity(replay, {});
  return metrics;
}

const fixtureBaseline = Object.fromEntries(SENSITIVITY_POSES.map(pose => [pose, input[pose].cue]));
checkPackage(mocked, fixtureBaseline);
const lostJoint = structuredClone(mocked); lostJoint.poses.dog.joints = {};
assert.throws(() => checkPackage(lostJoint, fixtureBaseline), /joint reactions need scenario coverage/);
const lostComponent = structuredClone(mocked); delete lostComponent.poses.dog.joints.ls.compressionN;
assert.throws(() => checkPackage(lostComponent, fixtureBaseline), /reaction components need scenario coverage/);

let actualMetrics = 0;
if (!args.includes('--synthetic')) {
  const filename = args.includes('--data') ? args[args.indexOf('--data') + 1] : DEFAULT_SENSITIVITY_PATH;
  assert(fs.existsSync(filename), 'build sensitivity.json after physics.json before checking actual data');
  const data = JSON.parse(fs.readFileSync(filename, 'utf8'));
  const baseline = JSON.parse(fs.readFileSync(new URL('physics.json', ROOT), 'utf8'));
  actualMetrics = checkPackage(data, baseline);
  if (args.includes('--rebuild')) {
    const rebuilt = await buildSensitivity({ generatedAtUTC: data.provenance.generatedAtUTC });
    assert.deepEqual(rebuilt, data, 'numerical scenarios must reproduce without elapsed-time differences');
  }
}
console.log(JSON.stringify({ synthetic: 'PASS', actualMetrics, rebuilt: args.includes('--rebuild'), test: fileURLToPath(import.meta.url) }));
