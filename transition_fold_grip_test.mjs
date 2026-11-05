// Preserve the intended anatomical grip frame while resolving actual atlas
// bone intersections. Static source data and force packages stay untouched.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { A, THREE, V, rootPlaced } from './cycle_lib.mjs';
import { createTransitionCollision } from './transition_collision.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { followFoldShins } from './transition_fold_grip.mjs';

const data = JSON.parse(fs.readFileSync('poses.json')).fold;
const original = { local: Object.fromEntries(Object.entries(data.local).map(([name, q]) =>
  [name, new THREE.Quaternion(...q).normalize()])), rootQ: new THREE.Quaternion(...data.root).normalize(), rootP: V(0, 0, 0) };
original.rootP = rootPlaced(original.local, original.rootQ);
const clone = input => ({ local: Object.fromEntries(Object.entries(input.local).map(([name, q]) => [name, q.clone()])),
  rootQ: input.rootQ.clone(), rootP: input.rootP.clone() });
const conf = clone(original);
for (const side of ['L', 'R']) conf.local['thigh_' + side].slerp(new THREE.Quaternion(), .04).normalize();
const snapshots = [JSON.stringify(original), JSON.stringify(conf)];
const bones = JSON.parse(fs.readFileSync('skull/bones_bp3d.json'));
const critical = createTransitionCollision(A, THREE, bones);
const opposite = createBodyCollision(A, THREE, bones), armFoot = createArmFootCollision(A, THREE, bones);
const world = c => A.computeWorld(c.local, c.rootQ, c.rootP);
assert(critical.inspect(world(original)).colliding, 'fixture must retain the source bone intersection');
assert(critical.inspect(world(conf)).colliding, 'moving only the legs must demonstrate the broken grip');
const result = followFoldShins({ A, THREE, original, conf });
const W = world(result.conf), before = world(original);
assert(!critical.inspect(W).colliding, 'all regional bone checks must pass');
assert(!opposite.inspect(W).colliding, 'opposite arms must remain clear');
assert(!armFoot.inspect(W).colliding, 'all arm/foot pairs must remain clear');
assert.deepEqual([JSON.stringify(original), JSON.stringify(conf)], snapshots, 'inputs must not mutate');
let maximumAttachedPointErrorM = 0, maximumRelativeOrientationError = 0;
for (const side of ['L', 'R']) {
  const hand = 'hand_' + side, shin = 'shank_' + side;
  const sourcePoint = before[hand].p.clone().sub(before[shin].p).applyQuaternion(before[shin].q.clone().invert());
  const actualPoint = W[hand].p.clone().sub(W[shin].p).applyQuaternion(W[shin].q.clone().invert());
  maximumAttachedPointErrorM = Math.max(maximumAttachedPointErrorM, sourcePoint.distanceTo(actualPoint));
  const sourceOrientation = before[shin].q.clone().invert().multiply(before[hand].q);
  const actualOrientation = W[shin].q.clone().invert().multiply(W[hand].q);
  maximumRelativeOrientationError = Math.max(maximumRelativeOrientationError, sourceOrientation.angleTo(actualOrientation));
}
assert(maximumAttachedPointErrorM <= .0002, 'grip may not be withdrawn away from the shin');
assert(maximumRelativeOrientationError < 1e-6, 'relative palm orientation must be preserved');
for (const name of Object.keys(conf.local)) if (!/^(hum|fore|ulna|hand)_[LR]$/.test(name))
  assert.deepEqual(result.conf.local[name].toArray(), conf.local[name].toArray(), 'unrelated/finger angle changed: ' + name);
const rotatedOriginal = clone(original), rotatedConf = clone(conf);
rotatedOriginal.rootQ.premultiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(.41, -.62, .2)));
rotatedConf.rootQ.copy(rotatedOriginal.rootQ); rotatedOriginal.rootP.add(V(.3, .2, -.4)); rotatedConf.rootP.copy(rotatedOriginal.rootP);
const rotated = followFoldShins({ A, THREE, original: rotatedOriginal, conf: rotatedConf });
for (const name of Object.keys(conf.local)) assert.deepEqual(rotated.conf.local[name].toArray(), result.conf.local[name].toArray(), 'placement changed the local IK branch');
console.log(JSON.stringify({ passed: true, extraWristWithdrawalM: result.adjustment.extraWristWithdrawalM,
  maximumAttachedPointErrorM, maximumRelativeOrientationError,
  allRequiredBoneGuards: true, fingerAnglesPreserved: true, canonicalIKPlacementInvariant: true }, null, 2));
