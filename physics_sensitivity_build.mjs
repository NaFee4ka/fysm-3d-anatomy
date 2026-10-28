// Two explicitly prescribed static-model scenarios, not statistical uncertainty.
// Full poses only: these values must not be attached to slider interpolation or
// to a cycle frame whose cue conditions and equilibrium were not solved here.
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
export const SENSITIVITY_POSES = Object.freeze(['stand', 'dog', 'plank', 'half', 'bow', 'fold', 'handstand']);
export const SENSITIVITY_SCENARIOS = Object.freeze({
  cue: Object.freeze({ cues: true, label: 'С заданным тонусом', definition: 'Prescribed CUE_TONE lower bounds, as in physics.json.' }),
  noCue: Object.freeze({ cues: false, label: 'Без заданного тонуса', definition: 'No prescribed CUE_TONE lower bounds; equilibrium, passive tissues and all other model assumptions remain.' }),
});
export const FORCE_BALANCE_RELATIVE_TOLERANCE = 1e-4;
const SOURCE_FILES = ['anatomy3.js', 'physics_core.mjs', 'physics_params.mjs', 'physics_build.mjs', 'poses.json', 'physics.json', 'package-lock.json', 'physics_sensitivity_build.mjs'];
export const DEFAULT_SENSITIVITY_PATH = path.join(ROOT, 'research/2025-2026/sensitivity.json');

function finite(value, name) {
  if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error('Non-finite scenario value: ' + name);
  return value;
}

function numericData(value) {
  // Execution duration is neither a physical observation nor deterministic data.
  if (Array.isArray(value)) return value.map(numericData);
  if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value)
    .filter(([key]) => !/^(ms|elapsed(?:Ms|Seconds)?|duration(?:Ms|Seconds)?)$/i.test(key))
    .map(([key, item]) => [key, numericData(item)]));
  if (typeof value === 'number') finite(value, 'diagnostic');
  return value;
}

export function scenarioMetric(cue, noCue) {
  finite(cue, 'cue'); finite(noCue, 'noCue');
  return { cue, noCue, range: [Math.min(cue, noCue), Math.max(cue, noCue)] };
}

function sameKeys(a, b, name) {
  const left = Object.keys(a).sort(), right = Object.keys(b).sort();
  if (JSON.stringify(left) !== JSON.stringify(right)) throw new Error('Scenario coverage differs: ' + name);
}

function scenarioDiagnostics(result, label) {
  const quality = numericData(result.diag && result.diag.quality);
  if (!quality || quality.status !== 'solved' || quality.converged !== true || quality.finite !== true)
    throw new Error('Unverified scenario solve: ' + label);
  const physicalBalance = numericData(result.diag?.physicalBalance);
  if (quality.physicalBalanceVerified !== true || physicalBalance?.verified !== true
      || physicalBalance.assemblyVerified !== true || !Number.isFinite(physicalBalance.maxAssemblyDifference)
      || physicalBalance.maxAssemblyDifference > 1e-7)
    throw new Error('Unverified physical force assembly: ' + label);
  for (const key of ['forceResidualN', 'momentResidualNm']) finite(physicalBalance[key], label + '/physical/' + key);
  for (const [residual, tolerance] of [['forceResidualN', 'forceToleranceN'], ['momentResidualNm', 'momentToleranceNm']]) {
    finite(physicalBalance[tolerance], label + '/physical/' + tolerance);
    if (physicalBalance[residual] < 0 || physicalBalance[tolerance] < 0 || physicalBalance[residual] > physicalBalance[tolerance])
      throw new Error('Physical equilibrium exceeds tolerance: ' + label);
  }
  for (const key of ['forceResidualN', 'momentResidualNm']) finite(quality[key], label + '/' + key);
  const balance = numericData(result.balance);
  if (!balance || !Array.isArray(balance.forceN) || balance.forceN.length !== 3
    || !Array.isArray(balance.forceErrorN) || balance.forceErrorN.length !== 3
    || !Array.isArray(balance.momentErrorNm) || balance.momentErrorNm.length !== 3)
    throw new Error('Unfiltered floor balance unavailable: ' + label);
  const weightN = finite(balance.weightN, label + '/weightN');
  if (!(weightN > 0)) throw new Error('Nonpositive body weight: ' + label);
  balance.forceN.forEach((v, i) => finite(v, label + '/floorForce/' + i));
  balance.momentErrorNm.forEach((v, i) => finite(v, label + '/floorMomentError/' + i));
  balance.forceErrorN.forEach((v, i) => {
    finite(v, label + '/floorForceError/' + i);
    const expected = balance.forceN[i] - (i === 1 ? weightN : 0);
    if (Math.abs(expected - v) > 1e-8) throw new Error('Inconsistent floor balance: ' + label);
    if (Math.abs(v) / weightN > FORCE_BALANCE_RELATIVE_TOLERANCE)
      throw new Error('Body mass equilibrium not verified: ' + label);
  });
  return { quality, physicalBalance, floor: balance };
}

export function assembleSensitivity(results, provenance) {
  const poses = {};
  for (const pose of SENSITIVITY_POSES) {
    const pair = results[pose];
    if (!pair || !pair.cue || !pair.noCue) throw new Error('Missing solved scenario: ' + pose);
    const { cue, noCue } = pair;
    const diagnostics = Object.fromEntries(Object.keys(SENSITIVITY_SCENARIOS)
      .map(id => [id, scenarioDiagnostics(pair[id], pose + '/' + id)]));
    sameKeys(cue.muscles, noCue.muscles, pose + '/muscles');
    sameKeys(cue.joints, noCue.joints, pose + '/joints');
    const muscles = {};
    for (const unit of Object.keys(cue.muscles)) {
      muscles[unit] = {};
      for (const [name, field] of [['a', 'a'], ['activation', 'activation'], ['forceN', 'F'], ['maxForceN', 'Fmax']])
        muscles[unit][name] = scenarioMetric(cue.muscles[unit][field], noCue.muscles[unit][field]);
    }
    const joints = {};
    for (const id of Object.keys(cue.joints)) {
      const a = cue.joints[id].jr, b = noCue.joints[id].jr;
      if (!a && !b) continue; // Scapular contact/patellofemoral proxies are different quantities.
      if (!a || !b) throw new Error('Joint-reaction coverage differs: ' + pose + '/' + id);
      joints[id] = {};
      for (const [name, field] of [['bw', 'bw'], ['forceN', 'F'], ['compressionN', 'comp'], ['shearN', 'shear']]) {
        if (a[field] === undefined && b[field] === undefined) continue;
        joints[id][name] = scenarioMetric(a[field], b[field]);
      }
    }
    poses[pose] = {
      muscles, joints,
      quality: Object.fromEntries(Object.keys(diagnostics).map(id => [id, diagnostics[id].quality])),
      physicalBalance: Object.fromEntries(Object.keys(diagnostics).map(id => [id, diagnostics[id].physicalBalance])),
      floor: Object.fromEntries(Object.keys(diagnostics).map(id => [id, diagnostics[id].floor])),
    };
  }
  return {
    v: 1, kind: 'scenario-range',
    scope: { poseT: 1, includesCycle: false, includesIntermediate: false, muscleSides: 'left', jointSides: 'left-and-central' },
    scenarios: SENSITIVITY_SCENARIOS,
    definitions: {
      a: 'Active muscle force divided by maximum isometric force; the existing displayed effort, not neural activation. Values retain the physics.json rounding.',
      activation: 'FMAX-weighted mean of solver activation variables within the left muscle unit; distinct from force/FMAX when force-length capacity changes.',
      range: 'Minimum and maximum of the two stored solved scenarios only. Not a confidence interval, measured inter-person variability, or proof of physiological accuracy.',
      floor: 'Complete unfiltered external floor force and its difference from body weight, before display grouping, thresholding and rounding.',
      forceBalanceRelativeTolerance: FORCE_BALANCE_RELATIVE_TOLERANCE,
    },
    limitations: [
      'Only the prescribed tonic lower bounds are changed. Geometry, strength, passive behavior, kinematics and support assumptions are held fixed.',
      'Both scenarios are mathematical static-model solutions. Neither scenario establishes a person-specific excitation pattern or measured joint force.',
      'No scenario range was computed for intermediate slider positions or any frame of the animated cycle.',
      'Muscle and joint values inherit the output precision and the anatomical limitations of physics.json. Compression and shear are included only where that model defines them.',
    ],
    provenance, poses,
  };
}

export function sourceHashes() {
  return Object.fromEntries(SOURCE_FILES.map(name => [name, createHash('sha256').update(fs.readFileSync(path.join(ROOT, name))).digest('hex')]));
}

export async function buildSensitivity({ buildPoseFn, generatedAtUTC = new Date().toISOString(), onProgress = () => {} } = {}) {
  if (!buildPoseFn) ({ buildPose: buildPoseFn } = await import('./physics_build.mjs'));
  if (typeof buildPoseFn !== 'function') throw new Error('physics_build.mjs must export buildPose before building sensitivity');
  const hashes = sourceHashes();
  const results = {};
  for (const pose of SENSITIVITY_POSES) {
    results[pose] = {};
    for (const [scenario, settings] of Object.entries(SENSITIVITY_SCENARIOS)) {
      onProgress({ pose, scenario });
      results[pose][scenario] = await buildPoseFn(pose, { cues: settings.cues });
    }
  }
  if (JSON.stringify(hashes) !== JSON.stringify(sourceHashes())) throw new Error('Numerical sources changed while solving sensitivity scenarios');
  return assembleSensitivity(results, { generatedAtUTC, sourceSha256: hashes, nodeVersion: process.version, baseline: 'physics.json with prescribed CUE_TONE' });
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2);
  const target = args.includes('--out') ? path.resolve(args[args.indexOf('--out') + 1]) : DEFAULT_SENSITIVITY_PATH;
  const data = await buildSensitivity({ onProgress: ({ pose, scenario }) => process.stderr.write('scenario ' + scenario + ': ' + pose + '\n') });
  fs.mkdirSync(path.dirname(target), { recursive: true });
  const temporary = target + '.tmp';
  fs.writeFileSync(temporary, JSON.stringify(data, null, 2) + '\n'); fs.renameSync(temporary, target);
  console.log(JSON.stringify({ file: target, poses: Object.keys(data.poses).length, scenarios: Object.keys(data.scenarios).length }));
}
