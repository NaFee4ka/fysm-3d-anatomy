// Fast release-guard and quantization tests. No QP solves or browser required.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';
import { HELD_POSES, validateCyclePhysics, packJointLoads, CYCLE_PHYSICS_SOURCE_FILES } from './cycle_quality.mjs';

const digest = raw => createHash('sha256').update(raw).digest('hex');

function fixture() {
  const quality = { status: 'solved', converged: true, finite: true, primalResidual: 0, dualResidual: 0,
    equalityViolation: 0, inequalityViolation: 0, boundViolation: 0, primalTolerance: 1e-6, dualTolerance: 1e-6,
    forceResidualN: 1e-8, momentResidualNm: 2e-8, physicalBalanceVerified: true };
  const record = { quality, aL: [.2], aR: [.3], sL: [0], sR: [0], ligL: [0], ligR: [0], com: [0, 1, 0],
    jr: Object.fromEntries(['L5S1', 'T12L1', 'hip', 'knee', 'ankle', 'gh', 'elbow', 'wrist'].map(id => [id, [.2, .3]])),
    reserves: 0, resid: 1e-8, modelLimits: { forceLengthClampedStrands: 0 },
    physicalBalance: { verified: true, assemblyVerified: true, forceResidualN: 1e-8, momentResidualNm: 2e-8,
      forceToleranceN: 1e-5, momentToleranceNm: 1e-5, maxAssemblyDifference: 1e-13 },
    floor: [{ F: [0, 700, 0], cop: [0, 0] }], grips: [] };
  const frames = Array.from({ length: 5 }, (_, i) => ({ i, T: i, seg: 'fixture', t: i }));
  const holds = HELD_POSES.map((pose, i) => ({ pose, T: i }));
  const cycle = { fps: 1, T: 4, frames, holds };
  const sourceSha256 = Object.fromEntries(CYCLE_PHYSICS_SOURCE_FILES.map((name, i) => [name, i.toString(16).padStart(64, '0')]));
  const expected = { sourceSha256, physicsSha256: 'f'.repeat(64) };
  const physics = { fps: 1, T: 4, frames: frames.map(frame => ({ ...frame, ...structuredClone(record) })), holds,
    sourceSha256: { ...sourceSha256 },
    units: ['fixture'], ligs: ['fixture-ligament'], opts: { every: 1 },
    poses: Object.fromEntries(HELD_POSES.map(id => [id, structuredClone(record)])) };
  return { cycle, physics, expected };
}

const valid = fixture();
assert.equal(validateCyclePhysics(valid.cycle, valid.physics, valid.expected).verified, true);
assert.equal(validateCyclePhysics(valid.cycle, valid.physics, valid.expected).physicsSha256, valid.expected.physicsSha256);
assert.deepEqual(validateCyclePhysics(valid.cycle, valid.physics, valid.expected).sourceSha256, valid.expected.sourceSha256);
assert.equal(validateCyclePhysics(valid.cycle, null), null);
function fails(change, pattern) { const { cycle, physics, expected } = fixture(); change(physics, cycle, expected); assert.throws(() => validateCyclePhysics(cycle, physics, expected), pattern); }
fails(p => { delete p.sourceSha256; }, /recorded source SHA-256/);
fails(p => { delete p.sourceSha256['physics_core.mjs']; }, /recorded source SHA-256/);
fails(p => { p.sourceSha256['anatomy3.js'] = 'invalid'; }, /recorded source SHA-256/);
fails(p => { p.sourceSha256['unexpected.json'] = 'e'.repeat(64); }, /recorded source SHA-256/);
fails((p, c, e) => { delete e.sourceSha256; }, /current source SHA-256/);
fails((p, c, e) => { delete e.physicsSha256; }, /raw physics SHA-256/);
for (const name of CYCLE_PHYSICS_SOURCE_FILES) fails(p => { p.sourceSha256[name] = 'd'.repeat(64); }, /stale physics source/);
fails(p => { delete p.frames[0].quality.forceResidualN; }, /quality metric/);
fails(p => { p.frames[0].quality.momentResidualNm = null; }, /quality metric/);
fails(p => { p.frames[0].quality.dualResidual = NaN; }, /quality metric/);
fails(p => { p.frames[0].quality.status = 'max_iterations'; }, /unverified/);
fails(p => { p.frames[0].quality.dualResidual = .1; }, /contradict/);
fails(p => { p.frames[0].quality.physicalBalanceVerified = false; }, /force-path equilibrium/);
fails(p => { delete p.frames[0].physicalBalance; }, /force-path equilibrium/);
fails(p => { p.frames[0].physicalBalance.assemblyVerified = false; }, /force-path equilibrium/);
fails(p => { p.frames[0].physicalBalance.maxAssemblyDifference = .01; }, /force-path assembly/);
fails(p => { p.frames[0].physicalBalance.forceResidualN = .01; }, /force-path assembly/);
fails(p => { p.frames[0].physicalBalance.momentResidualNm = .01; }, /force-path assembly/);
fails(p => { delete p.frames[0].physicalBalance.forceToleranceN; }, /force-path assembly/);
fails(p => { p.frames[0].physicalBalance.momentToleranceNm = -1; }, /force-path assembly/);
fails(p => { p.frames.splice(2, 1); }, /missing frame/);
fails(p => { p.frames.pop(); }, /missing first\/last/);
fails(p => { p.frames[1].i = 0; }, /duplicate/);
fails(p => { p.frames[1].T = 1.01; }, /stale frame/);
fails((p, c) => { c.frames[1].T += .001; p.frames[1].T += .001; }, /nonuniform kinematic/);
fails((p, c) => { c.T = p.T = 5; }, /nonuniform kinematic/);
fails(p => { delete p.poses.bow; }, /missing held-pose/);
fails(p => { p.frames[0].aL[0] = null; }, /numeric array/);
fails(p => { p.fps = 2; }, /stale duration/);
const sampled = fixture(); sampled.physics.opts.every = 2;
sampled.physics.frames = sampled.physics.frames.filter(frame => frame.i % 2 === 0);
const summary = validateCyclePhysics(sampled.cycle, sampled.physics, sampled.expected);
assert.equal(summary.verified, false); assert.equal(summary.fullCoverage, false);
assert.equal(summary.solvedFramesVerified, true); assert.equal(summary.frameCount, 3);
assert.equal(summary.publishedFrameCount, 5);
const high = packJointLoads([0, 1, 5.183, 5.165]);
assert(high.values[2] / high.scale > 5.1, 'a real reaction above 5.1 BW must not be silently clipped');
for (const [i, value] of [0, 1, 5.183, 5.165].entries()) assert(Math.abs(high.values[i] / high.scale - value) <= .5 / high.scale + 1e-12);
assert.equal(packJointLoads([0, 5.1]).scale, 50);
assert(packJointLoads([500]).values[0] <= 255);
assert.throws(() => packJointLoads([NaN]), /non-finite/);

// Synthetic guards above do not depend on whatever real artifacts happen to
// be in a checkout. Opt in once the full legacy pipeline has been regenerated.
if (process.argv.includes('--actual') || process.argv.includes('--packed')) {
const cycleRaw = fs.readFileSync(new URL('./cycle_keys/cycle.json', import.meta.url));
const physicsRaw = fs.readFileSync(new URL('./cycle_phys.json', import.meta.url));
const cycle = JSON.parse(cycleRaw), physics = JSON.parse(physicsRaw);
const actual = validateCyclePhysics(cycle, physics, {
  physicsSha256: digest(physicsRaw),
  sourceSha256: Object.fromEntries(CYCLE_PHYSICS_SOURCE_FILES.map(name => [name,
    digest(fs.readFileSync(new URL('./' + name, import.meta.url)))])),
});
assert.equal(actual.fullCoverage, true); assert.equal(actual.frameCount, cycle.frames.length);
const rawLoads = physics.frames.flatMap(frame => Object.values(frame.jr).flat());
const actualPacked = packJointLoads(rawLoads);
let maximumError = 0;
rawLoads.forEach((value, i) => { maximumError = Math.max(maximumError, Math.abs(Math.max(0, value) - actualPacked.values[i] / actualPacked.scale)); });
assert(maximumError <= .5 / actualPacked.scale + 1e-12);
let packedMaximumError = null;
if (process.argv.includes('--packed')) {
  const statsRaw = fs.readFileSync(new URL('./cycle_stats.json', import.meta.url));
  const stats = JSON.parse(statsRaw);
  assert.equal(stats.sourcePhysicsSha256, digest(physicsRaw), 'statistics must describe the exact solved physics file');
  assert.deepEqual(stats.provenance?.sourceSha256, {
    'cycle_stats.mjs': digest(fs.readFileSync(new URL('./cycle_stats.mjs', import.meta.url))),
  }, 'statistics must use the current statistics calculation');
  actual.statsSha256 = digest(statsRaw);
  actual.statsSourceSha256 = { ...stats.provenance.sourceSha256 };
  const published = JSON.parse(fs.readFileSync(new URL('./cycle_data.json', import.meta.url)));
  assert.deepEqual(published.modelQuality, actual, 'published quality summary must describe these exact solved frames');
  assert.equal(published.n, cycle.frames.length);
  const channels = { ls: ['L5S1', 0], tl: ['T12L1', 0] };
  for (const [id, key] of [['hip', 'hip'], ['kn', 'knee'], ['an', 'ankle'], ['gh', 'gh'], ['el', 'elbow'], ['wr', 'wrist']]) {
    channels[id + '_L'] = [key, 0]; channels[id + '_R'] = [key, 1];
  }
  assert.deepEqual(published.jload.ids.slice().sort(), Object.keys(channels).sort(), 'all reaction channels must survive packing');
  const bytes = Buffer.from(published.jload.v, 'base64'), scale = published.jload.scale;
  assert(Number.isFinite(scale) && scale > 0);
  assert.equal(bytes.length, published.n * published.jload.ids.length);
  packedMaximumError = 0;
  let preservedHighReaction = false, sourceHasHighReaction = false;
  for (const frame of physics.frames) published.jload.ids.forEach((id, channel) => {
    const [key, component] = channels[id], source = Math.max(0, frame.jr[key][component]);
    const decoded = bytes[frame.i * published.jload.ids.length + channel] / scale;
    packedMaximumError = Math.max(packedMaximumError, Math.abs(source - decoded));
    if (source > 5.15) sourceHasHighReaction = true;
    if (source > 5.15 && decoded > 5.1) preservedHighReaction = true;
  });
  assert(packedMaximumError <= .5 / scale + 1e-9, 'packing clipped or changed a reaction beyond its stated quantization precision');
  assert(!sourceHasHighReaction || preservedHighReaction, 'current data above the old 5.1 BW ceiling must remain above that ceiling');
  // Exercise the actual publication command, not only this test's comparison:
  // stale summary statistics must fail before any geometry/packing is written.
  const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'fysm-cycle-provenance-'));
  try {
    const variants = [
      [s => { s.sourcePhysicsSha256 = 'd'.repeat(64); }, /stale physics provenance/],
      [s => { delete s.sourcePhysicsSha256; }, /stale physics provenance/],
      [s => { s.provenance.sourceSha256['cycle_stats.mjs'] = 'd'.repeat(64); }, /stale cycle_stats.mjs provenance/],
      [s => { delete s.provenance; }, /stale cycle_stats.mjs provenance/],
    ];
    for (const [index, [change, pattern]] of variants.entries()) {
      const stale = structuredClone(stats); change(stale);
      const statsFile = path.join(temp, 'stats-' + index + '.json'), output = path.join(temp, 'rejected-' + index + '.json');
      fs.writeFileSync(statsFile, JSON.stringify(stale));
      const run = spawnSync(process.execPath, ['cycle_pack.mjs', '--stats', statsFile, '--out', output], {
        cwd: fileURLToPath(new URL('.', import.meta.url)), encoding: 'utf8', timeout: 10000,
      });
      assert.notEqual(run.status, 0, 'packing stale statistics must fail');
      assert.match(run.stderr, pattern);
      assert.equal(fs.existsSync(output), false, 'rejected statistics must not create a publication');
    }
  } finally { fs.rmSync(temp, { recursive: true, force: true }); }
}
console.log(JSON.stringify({ synthetic: 'PASS', actualFrames: actual.frameCount, fullCoverage: actual.fullCoverage,
  jointLoadScale: actualPacked.scale, maximumJointQuantizationErrorBW: maximumError, publishedMaximumJointQuantizationErrorBW: packedMaximumError }));
} else console.log(JSON.stringify({ synthetic: 'PASS', sourceManifestCases: CYCLE_PHYSICS_SOURCE_FILES.length,
  actualFrames: 'not requested; use --actual or --packed after ./cycle_run.sh' }));
