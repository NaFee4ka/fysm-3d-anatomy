import assert from 'node:assert/strict';
import { A, THREE } from './cycle_lib.mjs';
import { transitionSlerp } from './transition_runtime.mjs';
const quaternions = [new THREE.Quaternion(.173, .397, -.243, .867).normalize(),
  ...[15, 73, 127, 180].map(deg => A.qAx(new THREE.Vector3(.2, .7, .3).normalize(), deg))];
let states = 0;
for (const original of quaternions) for (const sign of [1, -1]) {
  const other = new THREE.Quaternion(...original.toArray().map(v => v * sign));
  for (let i = 0; i <= 100; i++) {
    const actual = transitionSlerp(original.clone(), other, i / 100).toArray();
    original.toArray().forEach((value, k) => assert(Object.is(actual[k], value), 'A constant or antipodal joint must stay bitwise constant'));
    states++;
  }
}
const start = new THREE.Quaternion(), end = A.qAx(new THREE.Vector3(1, 0, 0), 90);
const half = transitionSlerp(start.clone(), end, .5);
assert(Math.abs(half.angleTo(A.qAx(new THREE.Vector3(1, 0, 0), 45))) < 1e-7, 'A changing joint still follows the quaternion arc');
assert(start.equals(new THREE.Quaternion()), 'Only the requested target clone is mutable');
for (const fraction of [0, 1]) {
  const a = quaternions[0], b = quaternions[1], expected = fraction ? b : a;
  const actual = transitionSlerp(a.clone(), b, fraction).toArray();
  expected.toArray().forEach((value, k) => assert(Object.is(actual[k], value), 'Interpolation endpoints stay bitwise exact'));
}
console.log('PASS stable transition interpolation:', states, 'constant/antipodal states and an independent 45-degree arc.');
