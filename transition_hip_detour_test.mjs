// Handstand's femur/pelvis route needs a real temporary hip flexion corridor.
// The source and target remain exact; bones and bounded joint regions do not change.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { A, THREE, V, X, Z, qAx, blankLocal, setSide, rootPlaced } from './cycle_lib.mjs';
import { createHipCollisionOracle, findHipFlexionDetour } from './transition_hip_detour.mjs';
import { createTransitionCollision } from './transition_collision.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { createElbowMotionGuard } from './transition_motion.mjs';
import { transitionSlerp } from './transition_runtime.mjs';
const bones = JSON.parse(fs.readFileSync('skull/bones_bp3d.json'));
const original = JSON.parse(fs.readFileSync('poses.json')).handstand;
const from = { local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...original.local[s.name]).normalize()])),
  rootQ: new THREE.Quaternion(...original.root).normalize(), rootP: V(0, 0, 0) };
from.rootP = rootPlaced(from.local, from.rootQ);
const to = { local: blankLocal(), rootQ: new THREE.Quaternion(), rootP: V(0, 0, 0) };
for (const side of ['L', 'R']) {
  from.local['thigh_' + side].premultiply(qAx(Z, side === 'L' ? .5 : -.5)).normalize();
  setSide(to.local, side, 'thigh', qAx(Z, 8));
  for (const c of [from, to]) {
    setSide(c.local, side, 'hum', qAx(Z, 90)); setSide(c.local, side, 'fore', qAx(X, -20));
    setSide(c.local, side, 'ulna', new THREE.Quaternion()); setSide(c.local, side, 'hand', new THREE.Quaternion());
    for (const s of A.SEG) if (/^f[1-5]/.test(s.name) && s.name.endsWith('_' + side)) c.local[s.name].identity();
    setSide(c.local, side, 'f1m', qAx(X, -35));
  }
}
const oracle = createHipCollisionOracle(A, THREE, bones), critical = createTransitionCollision(A, THREE, bones);
const opposite = createBodyCollision(A, THREE, bones), armFoot = createArmFootCollision(A, THREE, bones), elbows = createElbowMotionGuard(A);
const snapshots = [JSON.stringify(from), JSON.stringify(to)];
const mix = (a, b, u) => ({ local: Object.fromEntries(A.SEG.map(s => [s.name,
  transitionSlerp(a.local[s.name].clone(), b.local[s.name], u)])),
  rootQ: transitionSlerp(a.rootQ.clone(), b.rootQ, u), rootP: a.rootP.clone().lerp(b.rootP, u) });
assert(!oracle.inspect(from).colliding && !oracle.inspect(to).colliding, 'both fixed endpoints must be clear');
assert(oracle.inspect(mix(from, to, .19)).colliding, 'the original adduction/extension band must reproduce');
let fullBodyStates = 0;
const clear = c => {
  fullBodyStates++;
  if (!elbows.inspect(c).valid) return false;
  const W = A.computeWorld(c.local, new THREE.Quaternion(), V(0, 0, 0));
  return ![critical, opposite, armFoot].some(checker => checker.inspect(W, { maxPairs: 1 }).colliding);
};
const result = findHipFlexionDetour({ A, THREE, from, to, oracle, samplesPerEdge: 1000, accept: clear });
assert(result && result.adjustment.degrees === 2, 'the first bounded 2-degree flexion corridor must clear');
assert.equal(JSON.stringify(from), snapshots[0]); assert.equal(JSON.stringify(to), snapshots[1]);
for (const [i, stage] of result.stages.entries()) {
  const reference = i ? to : from;
  assert.deepEqual(stage.rootQ.toArray(), reference.rootQ.toArray());
  assert.deepEqual(stage.rootP.toArray(), reference.rootP.toArray());
  for (const s of A.SEG) if (!/^thigh_[LR]$/.test(s.name))
    assert.deepEqual(stage.local[s.name].toArray(), reference.local[s.name].toArray(), 'a non-hip local changed');
}
const chain = [from, ...result.stages, to];
for (let i = 1; i < chain.length; i++) for (let j = 0; j <= 100; j++)
  assert(clear(mix(chain[i - 1], chain[i], j / 100)), `full hard guards rejected edge ${i}:${j}`);
assert.equal(findHipFlexionDetour({ A, THREE, from, to, oracle, accept: () => false }), null);
console.log(JSON.stringify({ passed: true, temporaryHipFlexionDegrees: result.adjustment.degrees,
  intrinsicSamples: result.evaluatedSamples, fullBodyStates, sourceAndTargetPreserved: true,
  scope: 'actual hip bones plus all transition hard regions, opposite arms, all arm-foot groups and model elbows',
  limitation: 'Discrete kinematic geometry; no dynamic balance or loaded joint certificate.' }, null, 2));
