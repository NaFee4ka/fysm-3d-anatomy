// Independent constitutive/contact/mechanical-assembly probes. No data writes.
// node physics_constitutive_test.mjs
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { activeForceLengthFactor, fiberLengthRatio, forceBallFacets, makePhysics } from './physics_core.mjs';
import { MUSCLE } from './physics_params.mjs';

const V = (...xyz) => new THREE.Vector3(...xyz);
const close = (actual, expected, tolerance = 1e-10) => assert(Math.abs(actual - expected) <= tolerance,
  `${actual} differs from independently expected ${expected}`);
const report = {};

// The supplied fibre fraction is used literally. Under a fixed-length tendon,
// 5% shortening of a unit with 10% fibres halves the fibre length; it must not
// become 83.3% through an undocumented replacement of phi by .3.
close(fiberLengthRatio(.95, 1, .1), .5);
close(fiberLengthRatio(1.02, 1, .2), 1.1);
close(fiberLengthRatio(2, 2, .13), 1);
assert(fiberLengthRatio(.5, 1, .1) < 0);
for (const length of [-10, -1, 0]) assert.equal(activeForceLengthFactor(length), 0);
close(activeForceLengthFactor(1), 1);
for (const delta of [.1, .2, .5, .8])
  close(activeForceLengthFactor(1 - delta), activeForceLengthFactor(1 + delta));
for (const length of [.1, .3, .4, .8, 1.3, 1.7, 2, 3]) {
  close(activeForceLengthFactor(length), Math.exp(-((length - 1) ** 2) / .45));
  const h = 1e-6, derivative = (activeForceLengthFactor(length + h) - activeForceLengthFactor(length - h)) / (2 * h);
  close(derivative, -2 * (length - 1) / .45 * Math.exp(-((length - 1) ** 2) / .45), 2e-9);
}
assert(activeForceLengthFactor(.2) < activeForceLengthFactor(.4));
assert(activeForceLengthFactor(2) < activeForceLengthFactor(1.7));
assert(activeForceLengthFactor(1000) < 1e-100);
for (const args of [[1, 0, .2], [1, 1, 0], [1, 1, 1.1], [-1, 1, .2], [NaN, 1, .2]])
  assert.throws(() => fiberLengthRatio(...args), /Invalid muscle/);
assert.throws(() => activeForceLengthFactor(Infinity), /Nonfinite/);
report.fibre = { originalPhiUsed: true, unclippedGaussian: true, nonpositiveLengthSlack: true };

// Independent vertex/ray descriptions of the force-radius polytope. Every
// admitted force must remain inside the named sphere, including oblique rays.
const phi = (1 + Math.sqrt(5)) / 2, radius = 137, facets = forceBallFacets(radius);
assert.equal(facets.length, 20);
const vertices = [];
for (const a of [-1, 1]) for (const b of [-1, 1])
  vertices.push(V(0, a, b * phi), V(a, b * phi, 0), V(a * phi, 0, b));
for (const vertex of vertices) {
  vertex.normalize().multiplyScalar(radius);
  close(vertex.length(), radius);
  assert(facets.every(face => face.normal.dot(vertex) <= face.bound + 1e-10));
  assert.equal(facets.filter(face => Math.abs(face.normal.dot(vertex) - face.bound) < 1e-9).length, 5);
}
let maxRadius = 0, minRadius = Infinity;
for (let i = 0; i < 2000; i++) {
  const z = 1 - 2 * (i + .5) / 2000, longitude = i * Math.PI * (3 - Math.sqrt(5));
  const direction = V(Math.sqrt(1 - z * z) * Math.cos(longitude), Math.sqrt(1 - z * z) * Math.sin(longitude), z);
  const bound = Math.min(...facets.filter(face => face.normal.dot(direction) > 0)
    .map(face => face.bound / face.normal.dot(direction)));
  assert(bound <= radius + 1e-10);
  minRadius = Math.min(minRadius, bound); maxRadius = Math.max(maxRadius, bound);
}
assert.throws(() => forceBallFacets(-1), /Invalid/);
report.forceCapacity = { faces: facets.length, independentlyCheckedVertices: vertices.length,
  rayChecks: 2000, minCapacityRatio: minRadius / radius, maxCapacityRatio: maxRadius / radius };

const A = createAnatomy(THREE), physics = makePhysics(A), W = physics.W_STAND;
const parametersBefore = JSON.stringify(MUSCLE);
for (const mass of [0, -1, Infinity, NaN]) assert.throws(() => makePhysics(A, { mass }), /Body mass/);

// A common point on two bodies cannot supply a net whole-body wrench. For a
// local elbow axis through c, P=c+(0,.2,.1) gives exactly -.1*Fy+.2*Fz.
const elbow = physics.dofIndex.el_L_x, axis = V(1, 0, 0), center = V(.4, .7, -.3);
const contact = center.clone().add(V(0, .2, .1)), force = V(4, -7, 11);
const displacement = physics.pairedDisplacement(elbow, 'fore_L', 'pelvis', contact, axis, center);
close(displacement.dot(force), -.1 * force.y + .2 * force.z);
close(physics.pairedDisplacement(elbow, 'pelvis', 'fore_L', contact, axis, center).dot(force.clone().negate()), displacement.dot(force));
const placement = new THREE.Quaternion().setFromAxisAngle(V(1, 2, 3).normalize(), .73), shift = V(-.3, .6, .2);
const rotated = physics.pairedDisplacement(elbow, 'fore_L', 'pelvis', contact.clone().applyQuaternion(placement).add(shift),
  axis.clone().applyQuaternion(placement), center.clone().applyQuaternion(placement).add(shift));
close(rotated.dot(force.clone().applyQuaternion(placement)), displacement.dot(force));
for (let k = 0; k < 6; k++) {
  const d = physics.DOF[k], ax = d.axis(W), c = d.center ? d.center(W) : null;
  assert(physics.pairedDisplacement(k, 'hand_L', 'foot_R', contact, ax, c).length() < 1e-14);
}
report.actionReaction = { globalWrenchCancelled: true, analyticElbowTorque: true, rigidFrameInvariant: true };

const floor = physics.contactsOf(W).map(c => ({ ...c, P: c.P.clone() }));
assert.throws(() => physics.solve(A.STAND, { contacts: null }), /override must be an array/);
assert.throws(() => physics.solve(A.STAND, { contacts: [] }), /external floor support/);
for (const patch of [{ id: '', seg: 'foot_L', P: V() }, { id: 'x', seg: 'unknown', P: V() },
  { id: 'x', seg: 'constructor', P: V() },
  { id: 'x', seg: 'foot_L', P: V(0, .01, 0) }, { id: 'x', seg: 'foot_L', P: [0, 0] },
  { id: 'x', seg: 'foot_L', P: V(), k: 0 }, { id: 'x', seg: 'foot_L', P: V(), mu: -1 },
  { id: 'x', seg: 'foot_L', P: V(), h: -1 }, { id: 'x', seg: 'foot_L', P: V(), h: .1 }])
  assert.throws(() => physics.solve(A.STAND, { contacts: [patch] }), /contact/);
assert.throws(() => physics.solve(A.STAND, { contacts: [floor[0], floor[0]] }), /unique id/);
const invalidContacts = [
  { id: 'x', segA: 'hand_L', segB: 'hand_L', P: V(), kind: 'bilateral' },
  { id: 'x', segA: 'constructor', segB: 'hand_R', P: V(), kind: 'bilateral' },
  { id: 'x', segA: 'hand_L', segB: 'hand_R', P: V(), kind: 'unilateral', normal: V() },
  { id: 'x', segA: 'hand_L', segB: 'hand_R', P: V(), kind: 'unilateral', normal: V(0, 1, 0), mu: -1 },
  { id: 'x', segA: 'hand_L', segB: 'hand_R', P: V(), kind: 'bilateral', maxForceN: -1 },
  { id: 'x', segA: 'hand_L', segB: 'hand_R', P: V(), kind: 'bilateral', maxNormalN: 1 },
];
for (const internal of invalidContacts)
  assert.throws(() => physics.solve(A.STAND, { internalContacts: [internal] }), /contact/);
assert.throws(() => physics.solve(A.STAND, { tone: { rectAbd: -1 } }), /activation bounds/);

// Integration: an explicit zero capacity stays zero (the former || .8 silently
// replaced zero). Two same-side contacts need distinct identities for warm data.
const midpoint = W.hand_L.p.clone().lerp(W.hand_R.p, .5);
const bilateral = { id: 'lock', segA: 'hand_L', segB: 'hand_R', P: midpoint, kind: 'bilateral', maxForceN: 0, side: 'L' };
const unilateral = { id: 'surface', segA: 'fore_L', segB: 'thigh_L', P: midpoint, kind: 'unilateral', normal: V(0, 1, 0), mu: .5, maxNormalN: 0, side: 'L' };
const solved = physics.solve(A.STAND, { contacts: floor, internalContacts: [bilateral, unilateral], iters: 4000 });
assert.equal(solved.quality.status, 'solved'); assert.equal(solved.physicalBalance.verified, true);
assert.equal(solved.internalContactF.length, 2);
assert(solved.internalContactF.every(c => c.F.length() < .01));
assert(solved.warm.x.has('internal:lock:0') && solved.warm.x.has('internal:surface:0'));
assert.equal(solved.contactQuality.floorSource, 'explicit-surface-patches');
assert.equal(solved.gripF.length, 0);
const sum = solved.contactF.reduce((s, c) => s.add(c.F), V());
close(sum.y, physics.BW, .003); assert(Math.hypot(sum.x, sum.z) < .003);
const netMoment = solved.contactF.reduce((s, c) => s.add(c.P.clone().cross(c.F)), V())
  .add(solved.com.clone().cross(V(0, -physics.BW, 0)));
assert(netMoment.length() < .003);
assert(solved.physicalBalance.maxAssemblyDifference < 1e-8);
const zeroLegacyInput = { hand: 'hand_L', shank: 'hand_R', P: midpoint.clone(), side: 'L', max: 0 };
const zeroLegacySnapshot = JSON.stringify(zeroLegacyInput);
const zeroLegacy = physics.solve(A.STAND, { grips: [zeroLegacyInput], iters: 4000 });
assert.equal(zeroLegacy.gripF.length, 1); assert.equal(zeroLegacy.internalContactF.length, 0);
assert(zeroLegacy.gripF[0].F.length() < .01);
assert.equal(JSON.stringify(zeroLegacyInput), zeroLegacySnapshot);

// Synthetic contact-contract probe, not a claim that the standing hand touches
// the pelvis. This makes the new cone carry a nonzero force and checks its
// independently projected normal/tangential force and named capacity in SI.
const normal = V(.2, 1, .1).normalize();
const brace = physics.solve(A.STAND, { internalContacts: [{ id: 'synthetic-brace', segA: 'hand_L',
  segB: 'pelvis', P: W.hand_L.p, kind: 'unilateral', normal, mu: .5, maxNormalN: 7, maxForceN: 9 }], iters: 4000 });
const braceForce = brace.internalContactF[0].F, compression = braceForce.dot(normal);
const tangential = braceForce.clone().addScaledVector(normal, -compression).length();
assert(braceForce.length() > 1e-5);
assert(compression >= -.003 && compression <= 7.003);
assert(tangential <= .5 * compression + .003);
assert(braceForce.length() <= 9.003);
assert(brace.physicalBalance.verified);

const heavier = makePhysics(A, { mass: 140 }).solve(A.STAND, { contacts: floor, iters: 4000 });
const sumHeavy = heavier.contactF.reduce((s, c) => s.add(c.F), V());
close(sumHeavy.y / sum.y, 2, 1e-5);
assert(heavier.physicalBalance.verified);
report.integration = { zeroContactCapacitiesPreserved: true, distinctSameSideWarmIds: true,
  legacyZeroCapPreserved: true, nonzeroCompressionN: compression, nonzeroTangentialForceN: tangential,
  externalForceScale: sumHeavy.y / sum.y, forceResidualN: solved.physicalBalance.forceResidualN,
  momentResidualNm: solved.physicalBalance.momentResidualNm, assemblyDifference: solved.physicalBalance.maxAssemblyDifference };

// At 145 degrees the source biceps line changes to an extensor lever. It must
// never retain tension with a fictitiously zeroed single QP moment component.
const local = Object.fromEntries(Object.entries(A.STAND.local).map(([id, q]) => [id, q.clone()]));
local.fore_L = A.qAx(A.AX.X, -145); local.fore_R = A.mirrorQ(local.fore_L);
const flexed = physics.solve({ local, rootQ: A.STAND.rootQ }, { tone: { bicLong: .2 }, iters: 5000 });
const index = flexed.INST.findIndex(s => s.unit === 'bicLong' && s.side === 'L');
assert(flexed.muscleQuality[index].quarantined);
assert(flexed.R[index][elbow] > .001);
assert.equal(flexed.force[index], 0); assert.equal(flexed.FP[index], 0);
assert.equal(flexed.FE[index], 0); assert.equal(flexed.act[index], 0);
assert(flexed.physicalBalance.verified);
// Independently reconstruct point-force moments; do not multiply the exported R.
for (const j of [index, flexed.INST.findIndex(s => s.unit === 'tricLong' && s.side === 'L')]) {
  const origin = flexed.W.fore_L.p, ax = V(1, 0, 0).applyQuaternion(flexed.W.hum_L.q), path = flexed.paths[j];
  let moment = 0;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i], unit = b.p.clone().sub(a.p).normalize();
    if (physics.DOF[elbow].moved.has(a.seg)) moment += a.p.clone().sub(origin).cross(unit).dot(ax);
    if (physics.DOF[elbow].moved.has(b.seg)) moment -= b.p.clone().sub(origin).cross(unit).dot(ax);
  }
  close(flexed.R[j][elbow], moment, 1e-12);
}
assert.equal(JSON.stringify(MUSCLE), parametersBefore, 'constitutive evaluation must not retune source parameters');
report.pathConsistency = { wrongActionStrandQuarantined: true, wholeActiveAndPassiveForceDisabled: true,
  actualForceMomentVerified: true, maxLengthDerivativeDifferenceM: flexed.pathQuality.maxDerivativeMismatchM };
console.log(JSON.stringify(report, null, 2));
