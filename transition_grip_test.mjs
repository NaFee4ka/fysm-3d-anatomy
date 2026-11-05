// Actual atlas meshes, not wrist-center distances, decide whether the grip
// variant clears the shin. This does not validate skin contact or grasp force.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { A, THREE, V } from './cycle_lib.mjs';
import { createWholeBodyCollision } from './whole_body_collision.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { repairBowGrip } from './transition_grip.mjs';

const boneData = JSON.parse(fs.readFileSync('skull/bones_bp3d.json'));
const data = JSON.parse(fs.readFileSync('poses.json')).bow;
const conf = { local: Object.fromEntries(Object.entries(data.local).map(([s, q]) =>
  [s, new THREE.Quaternion(...q).normalize()])), rootQ: new THREE.Quaternion(), rootP: V(0, 0, 0) };
const snapshot = JSON.stringify(conf);
const collider = createWholeBodyCollision(A, THREE, boneData);
const opposite = createBodyCollision(A, THREE, boneData), armFoot = createArmFootCollision(A, THREE, boneData);
const arm = s => /^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s);
const leg = s => /^(thigh|shank|foot|toes)_[LR]$/.test(s);
const palm = s => /^hand_[LR]$/.test(s), distal = s => /^f[1-5][bc]_[LR]$/.test(s);
const filter = (a, b) => (arm(a.seg) && leg(b.seg)) || (leg(a.seg) && arm(b.seg))
  || ((palm(a.seg) && distal(b.seg) || distal(a.seg) && palm(b.seg)) && a.seg.at(-1) === b.seg.at(-1));
const world = c => A.computeWorld(c.local, c.rootQ, c.rootP);
const inspect = c => collider.inspect(world(c), {
  pairFilter: filter, pairFilterLabel: 'all-arms-all-legs-own-distal-fingers-palms', maxPairs: Infinity,
});
const original = inspect(conf);
assert(original.colliding, 'fixture must retain the actual original grip collision');
assert(original.pairs.some(p => /^f1[ab]_L$/.test(p.a) && p.b === 'shank_L'
  || p.a === 'shank_L' && /^f1[ab]_L$/.test(p.b)), 'fixture must include a thumb/shin collision');
const accept = c => !inspect(c).colliding && !opposite.inspect(world(c)).colliding && !armFoot.inspect(world(c)).colliding;
const result = repairBowGrip({ A, THREE, conf, accept });
assert(result, 'bounded grip correction must pass actual mesh checks');
assert.equal(JSON.stringify(conf), snapshot, 'source pose must not mutate');
const before = world(conf), after = world(result.conf);
let maximumWristErrorM = 0, maximumElbowErrorM = 0;
for (const side of ['L', 'R']) {
  maximumWristErrorM = Math.max(maximumWristErrorM, before['hand_' + side].p.distanceTo(after['hand_' + side].p));
  maximumElbowErrorM = Math.max(maximumElbowErrorM, before['fore_' + side].p.distanceTo(after['fore_' + side].p));
  for (const [name, q] of Object.entries(conf.local)) if (name !== 'hand_' + side)
    if (!/^hand_[LR]$/.test(name)) assert.deepEqual(result.conf.local[name].toArray(), q.toArray(), name + ': unrelated angle changed');
  const joint = A.JOINTS.find(j => j.id === 'wr_' + side);
  const wrist = A.analyzeJoint(joint, result.conf.local, after).comps;
  for (const component of wrist) assert(component.value <= component.rom, side + ': wrist outside model range');
}
assert.equal(maximumWristErrorM, 0); assert.equal(maximumElbowErrorM, 0);
for (const perturbationDegrees of [-.25, .25]) {
  const local = Object.fromEntries(Object.entries(result.conf.local).map(([name, q]) => [name, q.clone()]));
  const delta = new THREE.Quaternion().setFromAxisAngle(V(1, 0, 0), perturbationDegrees * Math.PI / 180);
  local.hand_L.multiply(delta); local.hand_R.multiply(A.mirrorQ(delta));
  assert(accept({ ...result.conf, local }), 'small orientation perturbation must retain bone clearance');
}
const quantized = { ...result.conf, local: Object.fromEntries(Object.entries(result.conf.local).map(([name, q]) =>
  [name, new THREE.Quaternion(...q.toArray().map(n => Math.round(n * 1e8) / 1e8)).normalize()])) };
assert(accept(quantized), 'coarser-than-production quaternion rounding must retain bone clearance');
assert.equal(repairBowGrip({ A, THREE, conf, accept: () => false }), null, 'a failed geometry guard must reject the variant');
// A different global placement cannot change which hand and leg bones intersect.
const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(.57, -1.03, .29));
const placed = { local: conf.local, rootQ: rotation, rootP: V(.4, -.2, .7) };
const placedResult = repairBowGrip({ A, THREE, conf: placed, accept });
assert(placedResult, 'repair must be covariant under a global rigid placement');
console.log(JSON.stringify({ passed: true, originalIntersectingPairs: original.pairs.length,
  repairedIntersectingPairs: inspect(result.conf).pairs.length, degrees: result.adjustment.degrees,
  maximumWristErrorM, maximumElbowErrorM, fingerAnglesPreserved: true,
  checks: ['bilateral-arm-leg-atlas-bones', 'own-distal-fingers-palms', 'opposite-arms', 'all-arm-foot-pairs',
    'rigid-placement', 'orientation-perturbations', 'rounded-quaternions'] }, null, 2));
