// Numerical verification of constitutive equations and contact constraints.
// No poses, parameters, published data, or browser assets are regenerated.
// node physics_verification_test.mjs
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { COULOMB_FACETS, frictionFacets, spineJointEnergy, spineJointTorque, makePhysics } from './physics_core.mjs';
import { SPINE_PASSIVE } from './physics_params.mjs';
import { createAnatomy } from './anatomy3.js';

const D = Math.PI / 180;
const quaternion = degrees => new THREE.Quaternion().setFromEuler(new THREE.Euler(...degrees.map(x => x * D), 'XYZ'));
const report = {};

// Independently intersect rays in force space with the linear constraints, then
// compare every boundary point to the circular Coulomb law. Rotating the force
// direction relative to world axes must never lose more than the polygon error.
let minRatio = 1, maxRatio = 0, directionChecks = 0;
for (const mu of [.2, .5, 1, 1.4]) {
  const facets = frictionFacets(mu);
  assert.equal(facets.length, 16); assert.equal(COULOMB_FACETS, 16);
  for (const f of facets) assert(Math.abs(Math.hypot(f.x, f.z) - 1) < 1e-14);
  for (let k = 0; k < 1440; k++) {
    const yaw = k * 2 * Math.PI / 1440, x = Math.cos(yaw), z = Math.sin(yaw);
    const radius = Math.min(...facets.map(f => { const dot = f.x * x + f.z * z; return dot > 1e-12 ? -f.normal / dot : Infinity; }));
    const ratio = radius / mu; minRatio = Math.min(minRatio, ratio); maxRatio = Math.max(maxRatio, ratio); directionChecks++;
    assert(radius <= mu + 1e-12, 'friction approximation admitted force outside the Coulomb circle');
    assert(ratio >= Math.cos(Math.PI / 16) - 1e-12, 'world yaw introduced excess directional friction loss');
    assert(facets.every(f => f.x * radius * x + f.z * radius * z + f.normal < 1e-12));
  }
}
assert(Math.abs(maxRatio - 1) < 1e-12);
assert(Math.abs(minRatio - Math.cos(Math.PI / 16)) < 1e-12);
report.friction = { facets: COULOMB_FACETS, directionChecks, minCircularCapacityRatio: minRatio, maxCircularCapacityRatio: maxRatio };

function oldEulerMoment(parameter, angle, axis) {
  const limit = axis === 0 ? (angle >= 0 ? parameter.flex : parameter.ext) : parameter.side;
  const u = (Math.abs(angle) - .6 * limit) / (.4 * limit);
  return -parameter.k[axis] * angle - (u > 0 ? Math.sign(angle) * parameter.Mlim * Math.expm1(4 * u) / Math.expm1(4) : 0);
}

// Pure sagittal motion has an analytic spring/stop torque. The conservative
// implementation must retain the old stiffness, onset, range and end moment.
let sagittalChecks = 0, maxSagittalError = 0;
for (const parameter of Object.values(SPINE_PASSIVE)) {
  for (const angle of [-parameter.ext, -.6 * parameter.ext, -1, 0, 1, .6 * parameter.flex, parameter.flex]) {
    const q = quaternion([angle, 0, 0]), torque = spineJointTorque(q, parameter);
    const error = Math.abs(torque[0] - oldEulerMoment(parameter, angle, 0));
    maxSagittalError = Math.max(maxSagittalError, error); sagittalChecks++;
    assert(error < 1e-5, `sagittal spring law changed at ${angle} degrees (${error} N*m)`);
    assert(Math.abs(torque[1]) < 1e-7 && Math.abs(torque[2]) < 1e-7);
    assert(spineJointEnergy(q, parameter) >= 0);
  }
}

// Independent analytic Euler-rate Jacobian: R = Rx Ry Rz, so parent-frame
// angular velocity has columns ex, Rx*ey, Rx*Ry*ez. Physical moment is J^-T Q,
// not the vector of Euler generalized moments Q used by the previous code.
let mixedChecks = 0, maxMixedError = 0;
for (const parameter of Object.values(SPINE_PASSIVE)) for (const angles of [[10, 5, 7], [-8, -4, 3], [24, 11, -10]]) {
  const [x, y] = angles.map(a => a * D), Q = angles.map((angle, axis) => oldEulerMoment(parameter, angle, axis));
  const rhs = (Q[2] - Math.sin(y) * Q[0]) / Math.cos(y);
  const analytic = [Q[0], Math.cos(x) * Q[1] - Math.sin(x) * rhs, Math.sin(x) * Q[1] + Math.cos(x) * rhs];
  const numerical = spineJointTorque(quaternion(angles), parameter);
  for (let a = 0; a < 3; a++) {
    const error = Math.abs(numerical[a] - analytic[a]); maxMixedError = Math.max(maxMixedError, error); mixedChecks++;
    assert(error < 1e-5 * Math.max(1, Math.abs(analytic[a])), 'mixed-axis spinal torque does not match the analytic energy gradient');
  }
}

function loopWork(steps) {
  const corners = [[22, 10, 10], [24, 10, 10], [24, 11, 10], [22, 11, 10], [22, 10, 10]];
  let work = 0;
  for (let edge = 0; edge < 4; edge++) {
    const a = quaternion(corners[edge]), b = quaternion(corners[edge + 1]);
    for (let k = 0; k < steps; k++) {
      const start = a.clone().slerp(b, k / steps), end = a.clone().slerp(b, (k + 1) / steps);
      const delta = end.clone().multiply(start.clone().invert()).normalize();
      if (delta.w < 0) delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
      const angle = 2 * Math.atan2(Math.hypot(delta.x, delta.y, delta.z), delta.w);
      const axis = new THREE.Vector3(delta.x, delta.y, delta.z).normalize();
      const moment = new THREE.Vector3().fromArray(spineJointTorque(start.clone().slerp(end, .5), SPINE_PASSIVE.ls));
      work += moment.dot(axis) * angle;
    }
  }
  return work;
}
const coarseWork = loopWork(16), fineWork = loopWork(32);
assert(Math.abs(fineWork) < 1e-6, 'passive spinal springs supplied net work around a closed rotation loop');
assert(Math.abs(fineWork) < Math.abs(coarseWork) * .3 + 1e-9, 'closed-loop work does not converge to zero as quadrature is refined');
report.spine = { sagittalChecks, maxSagittalErrorNm: maxSagittalError, mixedChecks, maxMixedErrorNm: maxMixedError, coarseClosedLoopWorkJ: coarseWork, fineClosedLoopWorkJ: fineWork };

// Integration contract: an ordinary anatomical solve must verify its returned
// result; deliberately exhausting its budget must fail before a build can save
// unverified values. Failure and success both retain separate N / N*m residuals.
const A = createAnatomy(THREE), physics = makePhysics(A);
const result = physics.solve(A.STAND, { iters: 3000 });
assert.equal(result.quality.status, 'solved');
assert.equal(result.quality.converged, true);
assert.equal(result.maxRes, Math.max(result.quality.forceResidualN, result.quality.momentResidualNm));
assert(Number.isFinite(result.quality.forceResidualN) && Number.isFinite(result.quality.momentResidualNm));
assert.throws(() => physics.solve(A.STAND, { iters: 1, retries: 0 }), error => {
  assert.equal(error.quality.converged, false);
  assert(Number.isFinite(error.quality.forceResidualN) && Number.isFinite(error.quality.momentResidualNm));
  assert.match(error.message, /force residual .* N, moment residual .* N\*m/);
  return true;
});
report.integration = { verifiedStanding: true, failedSolveBlocked: true, forceResidualN: result.quality.forceResidualN, momentResidualNm: result.quality.momentResidualNm };
console.log(JSON.stringify(report, null, 2));
