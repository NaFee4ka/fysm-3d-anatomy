// Analytic geometry and independent convex-optimality checks. No model solve or
// renderer is needed: node collision_geometry_test.mjs
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { segmentClosest, capsuleClearance } from './collision_geometry.mjs';

const V = (x, y, z) => new THREE.Vector3(x, y, z);
const near = (a, b, tol = 1e-12) => assert(Math.abs(a - b) <= tol, `${a} != ${b}`);
const tests = [
  { name: 'orthogonal crossing', p: [V(-1, 0, 0), V(1, 0, 0), V(0, -1, 0), V(0, 1, 0)], d: 0 },
  { name: 'skew interior', p: [V(-1, 0, 0), V(1, 0, 0), V(0, -1, .6), V(0, 1, .6)], d: .6 },
  { name: 'parallel overlap', p: [V(0, 0, 0), V(2, 0, 0), V(.5, .4, 0), V(1.5, .4, 0)], d: .4 },
  { name: 'antiparallel overlap', p: [V(0, 0, 0), V(2, 0, 0), V(1.5, 0, 0), V(.5, 0, 0)], d: 0 },
  { name: 'disjoint collinear', p: [V(0, 0, 0), V(1, 0, 0), V(1.2, 0, 0), V(2, 0, 0)], d: .2 },
  { name: 'nearest endpoint pair', p: [V(0, 0, 0), V(1, 0, 0), V(2, 1, 0), V(2, 2, 0)], d: Math.SQRT2 },
  { name: 'endpoint against interior', p: [V(0, 0, 0), V(1, 0, 0), V(2, -1, 0), V(2, 1, 0)], d: 1 },
  { name: 'two point segments', p: [V(1, 1, 1), V(1, 1, 1), V(1, 4, 5), V(1, 4, 5)], d: 5 },
  { name: 'one point segment', p: [V(0, 1, 0), V(0, 1, 0), V(-1, 0, 0), V(1, 0, 0)], d: 1 },
  { name: 'near parallel crossing', p: [V(0, 0, 0), V(1, 0, 0), V(0, -5e-11, 0), V(1, 5e-11, 0)], d: 0 },
  { name: 'near parallel skew', p: [V(0, 0, 0), V(1, 0, 0), V(0, -5e-11, .2), V(1, 5e-11, .2)], d: .2 },
];
let analyticChecks = 0;
const q = new THREE.Quaternion().setFromAxisAngle(V(1, 2, 3).normalize(), .73);
for (const test of tests) {
  const before = test.p.map(p => p.toArray());
  near(segmentClosest(...test.p).distance, test.d);
  assert.deepEqual(test.p.map(p => p.toArray()), before, 'inputs must not be mutated');
  const swapped = [test.p[2], test.p[3], test.p[0], test.p[1]];
  near(segmentClosest(...swapped).distance, test.d);
  const reverse = [test.p[1], test.p[0], test.p[3], test.p[2]];
  near(segmentClosest(...reverse).distance, test.d);
  const rigid = test.p.map(p => p.clone().applyQuaternion(q).add(V(.3, -.9, .8)));
  near(segmentClosest(...rigid).distance, test.d, 2e-12);
  for (const scale of [1e-6, 1e6]) near(segmentClosest(...test.p.map(p => p.clone().multiplyScalar(scale))).distance, test.d * scale, Math.max(1e-15, scale * 1e-12));
  analyticChecks += 6;
}

// Regression: every pair of joint centres can be well apart while the bodies
// between them intersect. Checking equal coordinates alone misses this case.
const cross = tests[0].p;
for (const a of cross.slice(0, 2)) for (const b of cross.slice(2)) assert(a.distanceTo(b) > 1);
const capsule = (a, b, radiusM) => ({ a, b, radiusM });
const ca = capsule(cross[0], cross[1], .04), cb = capsule(cross[2], cross[3], .05);
near(capsuleClearance(ca, cb).clearanceM, -.09);
near(capsuleClearance(ca, { ...cb, radiusM: .08 }).clearanceM, -.12);
near(capsuleClearance(ca, cb).distanceM, 0);
near(capsuleClearance(cb, ca).clearanceM, -.09);
const touching = capsule(V(-1, .09, 0), V(1, .09, 0), .05);
near(capsuleClearance(ca, touching).clearanceM, 0);

// Recorded from published main 9f0231a, seg3 at local t=2.9 s, before the arm
// trajectory fix. Every endpoint pair has a >40 mm gap, yet the forearm axes
// cross. A deliberately thin 20 mm bone-core capsule already detects it.
const oldForearms = [
  V(-.04877392315153961, .3202603551990725, .2623602769029232),
  V(.04430184628730259, .3532597770801033, .027017780419208742),
  V(.04877392315153961, .3202603551990725, .2623602769029232),
  V(-.04430184628730259, .3532597770801033, .027017780419208742),
];
for (const a of oldForearms.slice(0, 2)) for (const b of oldForearms.slice(2)) assert(a.distanceTo(b) > .04);
near(segmentClosest(...oldForearms).distance, 0);
near(capsuleClearance(capsule(...oldForearms.slice(0, 2), .02), capsule(...oldForearms.slice(2), .02)).clearanceM, -.04);

// The squared distance is a convex quadratic. Independent first-order signs
// on its box constraints certify a global minimum, including boundary minima.
let seed = 0x7a5e44;
const rand = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 2 ** 32; };
const point = () => V(2 * rand() - 1, 2 * rand() - 1, 2 * rand() - 1);
function verifyOptimality(p, result) {
  const u = p[1].clone().sub(p[0]), v = p[3].clone().sub(p[2]);
  const s = result.a.clone().sub(p[0]).dot(u) / u.lengthSq();
  const t = result.b.clone().sub(p[2]).dot(v) / v.lengthSq();
  assert(s >= -1e-12 && s <= 1 + 1e-12 && t >= -1e-12 && t <= 1 + 1e-12);
  const delta = result.a.clone().sub(result.b);
  for (const [parameter, gradient] of [[s, u.dot(delta)], [t, -v.dot(delta)]]) {
    if (parameter <= 1e-10) assert(gradient >= -1e-10);
    else if (parameter >= 1 - 1e-10) assert(gradient <= 1e-10);
    else assert(Math.abs(gradient) < 1e-10);
  }
  near(result.a.distanceTo(result.b), result.distance);
}
for (let i = 0; i < 1000; i++) {
  const p = [point(), point(), point(), point()];
  const result = segmentClosest(...p);
  verifyOptimality(p, result);
  near(segmentClosest(p[2], p[3], p[0], p[1]).distance, result.distance);
  // A sparse sampled surface provides a separate attainable upper bound.
  for (let a = 0; a <= 4; a++) for (let b = 0; b <= 4; b++)
    assert(result.distance <= p[0].clone().lerp(p[1], a / 4).distanceTo(p[2].clone().lerp(p[3], b / 4)) + 1e-12);
}
for (const bad of [NaN, Infinity, -Infinity]) {
  assert.throws(() => segmentClosest(V(bad, 0, 0), ...cross.slice(1)), /finite Vector3/);
  assert.throws(() => capsuleClearance({ ...ca, radiusM: bad }, cb), /radius/);
}
assert.throws(() => capsuleClearance({ ...ca, radiusM: -.01 }, cb), /radius/);
assert.throws(() => capsuleClearance({ a: ca.a, b: ca.b }, cb), /radius/);
assert.throws(() => segmentClosest(null, ...cross.slice(1)), /finite Vector3/);
assert.throws(() => segmentClosest(V(1e308, 0, 0), V(-1e308, 0, 0), ...cross.slice(2)), /finite coordinate range/);
console.log(JSON.stringify({ analyticChecks, convexOptimalityChecks: 1000, sampledUpperBoundChecks: 25000,
  jointCentresMissCollisionRegression: true, recordedSeg3CrossingRegression: true, invalidInputsRejected: true, pass: true }));
