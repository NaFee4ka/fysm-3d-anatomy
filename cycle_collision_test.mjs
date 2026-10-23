// The old intersecting pose must fail; checking only knots must be insufficient.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { spawnSync } from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { motionFrames, validateMotionCollisions, geometryDigest } from './cycle_collision_check.mjs';

const read = file => JSON.parse(fs.readFileSync(new URL(file, import.meta.url)));
const cycle = read('./cycle_keys/cycle.json');
const old = read('./test_fixtures/seg3_arm_crossing.json').frame;
const oldMotion = { frames: [{ ...old, t: 0 }, { ...old, t: .1 }] };
await assert.rejects(() => validateMotionCollisions(oldMotion, { preflight: true }), /arm cores intersect/);

// A backend's collision signal at an interior point must block publication,
// even if the backend accepts both knot poses. This tests the sampling contract,
// not an anatomical claim about the translated test configuration.
const base = structuredClone(cycle.frames[0]);
const a = { ...structuredClone(base), T: 0 };
const b = { ...structuredClone(base), T: .1, rootP: [base.rootP[0] + 1, ...base.rootP.slice(1)] };
const interiorBackend = { boneParts: 1, inspect(W) {
  const x = W.pelvis.p.x - base.rootP[0];
  return { colliding: x > .4 && x < .6, pairs: [{ a: 'test', b: 'interior' }] };
} };
await validateMotionCollisions({ frames: [a, b] }, { substeps: 1, engine: interiorBackend });
await assert.rejects(() => validateMotionCollisions({ frames: [a, b] }, { engine: interiorBackend }), /bones intersect at 0\.0450/);
const malformed = structuredClone(a); malformed.local.hum_L = [NaN, 0, 0, 1];
assert.throws(() => motionFrames({ frames: [malformed, b] }), /invalid quaternion/);
assert.throws(() => motionFrames({ frames: [a, a] }), /invalid frame times/);
await assert.rejects(() => validateMotionCollisions(cycle, { substeps: 0 }), /subdivision/);

const packed = read('./cycle_data.json');
const actual = await validateMotionCollisions(packed);
assert.equal(actual.verified, true);
assert.equal(actual.states, (packed.n - 1) * 20 + 1);
assert.equal(actual.geometrySha256, packed.collisionCheck.geometrySha256);
assert.deepEqual(actual.sourceSha256, packed.collisionCheck.sourceSha256);
assert.equal(actual.boneParts, packed.collisionCheck.boneParts);
const changed = structuredClone(packed);
changed.p = Buffer.from(changed.p, 'base64').map((v, i) => i === 0 ? v ^ 1 : v).toString('base64');
assert.notEqual(geometryDigest(changed), actual.geometrySha256);

// Python-only page builds must reject a stale/omitted check rather than silently
// shipping data from a different motion or checker revision.
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'fysm-collision-build-'));
try {
  const staleSource = structuredClone(packed);
  staleSource.collisionCheck.sourceSha256['body_collision.mjs'] = '0'.repeat(64);
  for (const [label, data] of [['stale-geometry', changed], ['missing-check', { ...packed, collisionCheck: null }], ['stale-checker', staleSource]]) {
    const filename = path.join(tmp, label + '.json');
    fs.writeFileSync(filename, JSON.stringify(data));
    const result = spawnSync('python3', ['build.py', path.join(tmp, 'dist')], {
      cwd: path.dirname(fileURLToPath(import.meta.url)), encoding: 'utf8', env: { ...process.env, CYCLE_DATA: filename },
    });
    assert.notEqual(result.status, 0, label + ' must block the build');
    assert.match(result.stderr, /collision verification is missing or stale/);
  }
} finally { fs.rmSync(tmp, { recursive: true, force: true }); }
console.log(JSON.stringify({ passed: true, regressionPoseRejected: true, intermediateCollisionRejected: true,
  states: actual.states, boneParts: actual.boneParts, staleBuildsBlocked: 3 }));
