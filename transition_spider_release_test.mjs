// Actual bone meshes and the model elbow range verify this staged release.
// No source pose, original cycle, bone mesh or force data is changed.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { A, THREE, V, X, Z, qAx, setSide, blankLocal } from './cycle_lib.mjs';
import { prepareSpiderRelease } from './transition_spider_release.mjs';
import { transitionSlerp } from './transition_runtime.mjs';
import { createTransitionCollision } from './transition_collision.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { createElbowMotionGuard } from './transition_motion.mjs';

const data = JSON.parse(fs.readFileSync('practice/poses.json')).poses['struna-05'], bones = JSON.parse(fs.readFileSync('skull/bones_bp3d.json'));
const source = { local: Object.fromEntries(Object.entries(data.local).map(([name, q]) =>
  [name, new THREE.Quaternion(...q).normalize()])), rootQ: new THREE.Quaternion(...data.root).normalize(), rootP: V(0, 0, 0) };
source.rootP = new THREE.Vector3(...data.rootP);
const critical = createTransitionCollision(A, THREE, bones), opposite = createBodyCollision(A, THREE, bones);
const armFoot = createArmFootCollision(A, THREE, bones), elbows = createElbowMotionGuard(A);
const world = c => A.computeWorld(c.local, new THREE.Quaternion(), V(0, 0, 0)), cache = new Map();
let actualStates = 0, edges = 0, minimumElbowFlexDeg = Infinity, maximumElbowFlexDeg = -Infinity;
function collision(c) {
  const key = JSON.stringify(A.SEG.map(s => c.local[s.name].toArray()));
  if (cache.has(key)) return cache.get(key);
  actualStates++;
  const motion = elbows.inspect(c);
  for (const value of Object.values(motion.flexDeg)) {
    minimumElbowFlexDeg = Math.min(minimumElbowFlexDeg, value);
    maximumElbowFlexDeg = Math.max(maximumElbowFlexDeg, value);
  }
  let hit = motion.valid ? null : { scope: 'model-elbow-ROM', violations: motion.violations };
  const W = world(c);
  for (const inspect of [() => critical.inspect(W), () => opposite.inspect(W), () => armFoot.inspect(W)]) {
    if (hit) break;
    const result = inspect(); if (result.colliding) hit = { scope: result.scope, pair: result.pairs[0] };
  }
  cache.set(key, hit); return hit;
}
const mix = (a, b, u) => ({ local: Object.fromEntries(A.SEG.map(s =>
  [s.name, transitionSlerp(a.local[s.name].clone(), b.local[s.name], u)])),
  rootQ: a.rootQ.clone().slerp(b.rootQ, u).normalize(), rootP: a.rootP.clone().lerp(b.rootP, u) });
const clone = c => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, c.local[s.name].clone()])),
  rootQ: c.rootQ.clone(), rootP: c.rootP.clone() });
function clearEdge(a, b) {
  edges++;
  const samples = new Set(Array.from({ length: 101 }, (_, i) => i / 100));
  for (const u of [.000001, .00001, .0001, .001, .003, .006, .015, .025]) { samples.add(u); samples.add(1 - u); }
  for (const u of samples) if (collision(mix(a, b, u))) return false;
  return true;
}
const conf = source;
const snapshot = JSON.stringify(conf);
const openTarget = { local: blankLocal(), rootQ: new THREE.Quaternion(), rootP: V(0, 0, 0) };
for (const side of ['L', 'R']) setSide(openTarget.local, side, 'thigh', qAx(Z, 8));
for (const side of ['L', 'R']) {
  setSide(openTarget.local, side, 'hum', qAx(Z, 90)); setSide(openTarget.local, side, 'fore', qAx(X, -20));
  setSide(openTarget.local, side, 'ulna', new THREE.Quaternion()); setSide(openTarget.local, side, 'hand', new THREE.Quaternion());
  for (const s of A.SEG) if (/^f[1-5]/.test(s.name) && s.name.endsWith('_' + side)) openTarget.local[s.name].identity();
  setSide(openTarget.local, side, 'f1m', qAx(X, -35));
}
const openSource = clone(conf);
for (const s of A.SEG) if (/^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s.name)) openSource.local[s.name].copy(openTarget.local[s.name]);
assert(!clearEdge(openSource, openTarget), 'opening arms before unfolding the body must reproduce the upper-arm/shin shortcut');
const stages = prepareSpiderRelease({ A, THREE, conf, collision, clearEdge, openTarget });
assert(stages && stages.length === 2, 'the verified staged release must be available');
assert.equal(JSON.stringify(conf), snapshot, 'source endpoint must not mutate');
for (const [i, stage] of stages.entries()) {
  const reference = i ? openTarget : conf;
  assert.deepEqual(stage.rootQ.toArray(), reference.rootQ.toArray());
  assert.deepEqual(stage.rootP.toArray(), reference.rootP.toArray());
  for (const s of A.SEG) if (!/^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s.name))
    assert.deepEqual(stage.local[s.name].toArray(), reference.local[s.name].toArray(), s.name + ': body or leg changed at an arm-only stage');
  for (const side of ['L', 'R']) {
    const shoulder = A.analyzeJoint(A.JOINTS.find(j => j.id === 'gh_' + side), stage.local, world(stage));
    assert(Math.abs(shoulder.comps.find(c => c.key === 'flex').signed + 45) < 1e-7, 'arms stay 45 degrees behind their own thorax');
  }
}
// Global placement must preserve the thorax-relative shoulder route.
const rotated = clone(conf), rotatedOpen = clone(openTarget);
const global = new THREE.Quaternion().setFromEuler(new THREE.Euler(.4, -.7, .3));
rotated.rootQ.premultiply(global); rotatedOpen.rootQ.premultiply(global);
rotated.rootP.applyQuaternion(global).add(V(.3, .2, -.4)); rotatedOpen.rootP.applyQuaternion(global).add(V(.3, .2, -.4));
const transformed = prepareSpiderRelease({ A, THREE, conf: rotated, openTarget: rotatedOpen,
  collision: c => { assert(elbows.inspect(c).valid); return null; }, clearEdge: () => true });
assert(transformed && transformed.length === stages.length);
for (let i = 0; i < stages.length; i++) for (const s of A.SEG)
  assert(transformed[i].local[s.name].angleTo(stages[i].local[s.name]) < 1e-7, 'global placement changed the relative route');
assert.equal(prepareSpiderRelease({ A, THREE, conf, openTarget, collision: () => ({ blocked: true }), clearEdge }), null);
console.log(JSON.stringify({ passed: true, stages: stages.length, edges, actualStates,
  minimumElbowFlexDeg, maximumElbowFlexDeg, armStagesRetainTheirOwnBodyAndRoot: true, armsStayBehindDuringBodyUnfold: true,
  shoulderExtensionDeg: 45, normalElbowFlexionDeg: 20,
  limitation: 'Discrete atlas-bone geometry and the model elbow range; no loaded grasp, soft-tissue contact or dynamics.' }, null, 2));
