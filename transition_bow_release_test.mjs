// Actual bone meshes and the model elbow range verify this staged release.
// No source pose, original cycle, bone mesh or force data is changed.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { A, THREE, V, X, Z, qAx, setSide, rootPlaced, armIK, orientHand } from './cycle_lib.mjs';
import { repairBowGrip } from './transition_grip.mjs';
import { prepareBowRelease } from './transition_bow_release.mjs';
import { createTransitionCollision } from './transition_collision.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { createElbowMotionGuard } from './transition_motion.mjs';

const data = JSON.parse(fs.readFileSync('poses.json')).bow, bones = JSON.parse(fs.readFileSync('skull/bones_bp3d.json'));
const source = { local: Object.fromEntries(Object.entries(data.local).map(([name, q]) =>
  [name, new THREE.Quaternion(...q).normalize()])), rootQ: new THREE.Quaternion(...data.root).normalize(), rootP: V(0, 0, 0) };
source.rootP = rootPlaced(source.local, source.rootQ);
const critical = createTransitionCollision(A, THREE, bones), opposite = createBodyCollision(A, THREE, bones);
const armFoot = createArmFootCollision(A, THREE, bones), elbows = createElbowMotionGuard(A);
const world = c => A.computeWorld(c.local, c.rootQ, c.rootP), cache = new Map();
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
  [s.name, a.local[s.name].clone().slerp(b.local[s.name], u).normalize()])),
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
const conf = repairBowGrip({ A, THREE, conf: source, accept: c => !collision(c) }).conf;
const snapshot = JSON.stringify(conf);
const openTarget = clone(conf);
for (const side of ['L', 'R']) {
  setSide(openTarget.local, side, 'hum', qAx(Z, 90)); setSide(openTarget.local, side, 'fore', qAx(X, -20));
  setSide(openTarget.local, side, 'ulna', new THREE.Quaternion()); setSide(openTarget.local, side, 'hand', new THREE.Quaternion());
  for (const s of A.SEG) if (/^f[1-5]/.test(s.name) && s.name.endsWith('_' + side)) openTarget.local[s.name].identity();
  setSide(openTarget.local, side, 'f1m', qAx(X, -35));
}
assert(!clearEdge(conf, openTarget), 'fixture must reproduce the joint-slerp shortcut through a shin');
const stages = prepareBowRelease({ A, THREE, conf, collision, clearEdge, openTarget });
assert(stages && stages.length === 19, 'the staged release must omit the isolated folded-wrist palm turn');
assert(stages.slice(2).every(c => c.transitionGuide === true), 'continuous IK guides must avoid a full stop at each knot');
assert(stages.slice(0, 2).every(c => !c.transitionGuide), 'initial finger release remains a separate eased stage');
// Reproduce the actual regression with real atlas bones, not a stage-count
// assertion: a final world-palm turn at the lateral wrist location penetrates
// the forearm with the first metacarpal. Turning with the arm stays near neutral.
const lateral = stages.at(-1), turned = clone(lateral);
const lateralW = A.computeWorld(lateral.local, new THREE.Quaternion(), V(0, 0, 0));
const openW = A.computeWorld(openTarget.local, new THREE.Quaternion(), V(0, 0, 0));
for (const side of ['L', 'R']) {
  const flex = armIK(turned.local, new THREE.Quaternion(), V(0, 0, 0), side,
    lateralW['hand_' + side].p, lateralW['fore_' + side].p);
  orientHand(turned.local, new THREE.Quaternion(), V(0, 0, 0), side, openW['hand_' + side].q, flex);
}
const regression = critical.inspect(world(turned));
assert(regression.colliding && regression.pairs.some(p => /^(fore|ulna)_[LR]$/.test(p.a) && /^f1m_[LR]$/.test(p.b)
  || /^(fore|ulna)_[LR]$/.test(p.b) && /^f1m_[LR]$/.test(p.a)), 'old isolated turn must reproduce the real forearm/first-metacarpal collision');
const wrist = A.JOINTS.find(j => j.id === 'wr_L');
let maximumReleasedWristAngleDeg = 0;
for (let i = 0; i <= 1000; i++) {
  const c = mix(lateral, openTarget, i / 1000);
  assert(!collision(c), 'released hand turns together with the whole arm');
  maximumReleasedWristAngleDeg = Math.max(maximumReleasedWristAngleDeg, A.analyzeJoint(wrist, c.local, world(c)).total);
}
assert(maximumReleasedWristAngleDeg < 22, 'the repair must avoid the previous 121-degree wrist fold');
assert.equal(JSON.stringify(conf), snapshot, 'closed endpoint must not mutate');
for (const stage of stages) {
  assert.deepEqual(stage.rootQ.toArray(), conf.rootQ.toArray());
  assert.deepEqual(stage.rootP.toArray(), conf.rootP.toArray());
  for (const s of A.SEG) if (!/^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s.name))
    assert.deepEqual(stage.local[s.name].toArray(), conf.local[s.name].toArray(), s.name + ': body or leg changed');
}
// Global placement must not choose a different local IK branch.
const rotated = clone(conf), rotatedOpen = clone(openTarget);
const global = new THREE.Quaternion().setFromEuler(new THREE.Euler(.4, -.7, .3));
rotated.rootQ.premultiply(global); rotatedOpen.rootQ.copy(rotated.rootQ);
rotated.rootP.add(V(.3, .2, -.4)); rotatedOpen.rootP.copy(rotated.rootP);
const transformed = prepareBowRelease({ A, THREE, conf: rotated, openTarget: rotatedOpen,
  collision: c => { assert(elbows.inspect(c).valid); return null; }, clearEdge: () => true });
assert(transformed && transformed.length === stages.length);
for (let i = 0; i < stages.length; i++) for (const s of A.SEG)
  assert.deepEqual(transformed[i].local[s.name].toArray(), stages[i].local[s.name].toArray(), 'placement changed IK branch');
assert.equal(prepareBowRelease({ A, THREE, conf, openTarget, collision: () => ({ blocked: true }), clearEdge }), null);
console.log(JSON.stringify({ passed: true, stages: stages.length, edges, actualStates,
  minimumElbowFlexDeg, maximumElbowFlexDeg, localRootAndBodyPreserved: true,
  maximumReleasedWristAngleDeg, previousForearmFirstMetacarpalIntrusionRejected: true,
  initialUnhookDistanceM: Math.hypot(.005, .01, .02), canonicalIKPlacementInvariant: true,
  limitation: 'Discrete atlas-bone geometry and the model elbow range; no loaded grasp, soft-tissue contact or dynamics.' }, null, 2));
