// External-input component verification; no QP solve, body data or browser build.
// node physics_reference_geometry_test.mjs
// Published coordinates: Gonçalves et al., DOI 10.1007/s11044-025-10096-8.
// Analytic expectations below are derived independently from those coordinates.
// This does not reproduce the authors' Hill model, optimizer or force results.
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';

const reference = JSON.parse(readFileSync(new URL('./research/2025-2026/benchmark-geometry.json', import.meta.url), 'utf8'));
assert.equal(reference.source.license, 'CC-BY-4.0');
assert.equal(reference.source.doi, '10.1007/s11044-025-10096-8');
const A = createAnatomy(THREE), physics = makePhysics(A); // Initialization only; solve() is never called.
const V = xyz => new THREE.Vector3(...xyz), D = Math.PI / 180;
const published = reference.published, mapping = reference.adaptation.bodyToFYSM;
const joint = published.joints.J1, axisLocal = V(joint.axisGlobalAtNeutral);
const jointWorld = V(published.bodies.body1.initialCenterM).add(V(joint.body1LocalM));
const dof = physics.dofIndex.el_L_x;
assert(physics.DOF[dof].moved.has(mapping.body2));
assert(!physics.DOF[dof].moved.has(mapping.body1));
const sourceSnapshot = JSON.stringify(A.strandInstances());
const segmentOrigins = Object.fromEntries(Object.entries(A.S).map(([name, s]) => [name, s.J.toArray()]));

function instance(caseId, muscle) {
  return {
    id: `external_${caseId}_${muscle.id}`, kind: 'muscle', unit: 'external_geometry_probe',
    pts: muscle.points.map(p => ({ seg: mapping[p.body], p: V(p.localM).add(A.S[mapping[p.body]].J) })),
    wraps: [],
  };
}

// Use published center-of-mass frames, adapted through A.pt's existing rest-J
// convention. The benchmark joint center is passed explicitly to the production
// rotW/dpdq helpers: W.fore_L.p is a CoM here, not the anatomical elbow origin.
function world(angle, placement) {
  const localQ = new THREE.Quaternion().setFromAxisAngle(axisLocal, angle);
  const center2 = V(joint.body2LocalM).negate().applyQuaternion(localQ).add(jointWorld);
  const worldPoint = p => p.applyQuaternion(placement.q).add(placement.shift);
  return {
    [mapping.body1]: { p: worldPoint(V(published.bodies.body1.initialCenterM)), q: placement.q.clone() },
    [mapping.body2]: { p: worldPoint(center2), q: placement.q.clone().multiply(localQ) },
  };
}

// Scalar trigonometry, independent of A.pt, strandPath and THREE's quaternion
// path transforms: L = fixed length + |Ry(q) r - a|, R = -dL/dq.
function analytic(muscle, angle) {
  const fixed = muscle.points.filter(p => p.body === 'body1').map(p => p.localM);
  assert.equal(fixed.length, muscle.points.length - 1);
  const insertion = muscle.points.at(-1).localM;
  const r = insertion.map((x, k) => x - joint.body2LocalM[k]);
  const a = fixed.at(-1).map((x, k) => x - joint.body1LocalM[k]);
  const cos = Math.cos(angle), sin = Math.sin(angle);
  const b = [cos * r[0] + sin * r[2], r[1], -sin * r[0] + cos * r[2]];
  const delta = b.map((x, k) => x - a[k]), terminalLength = Math.hypot(...delta);
  let length = terminalLength;
  for (let i = 1; i < fixed.length; i++) length += Math.hypot(...fixed[i].map((x, k) => x - fixed[i - 1][k]));
  const derivative = (delta[0] * b[2] - delta[2] * b[0]) / terminalLength;
  const points = fixed.map(p => p.map((x, k) => x + published.bodies.body1.initialCenterM[k]));
  points.push(b.map((x, k) => x + jointWorld.getComponent(k)));
  return { length, momentArm: -derivative, points };
}

const frames = [
  { q: new THREE.Quaternion(), shift: V([0, 0, 0]) },
  { q: new THREE.Quaternion().setFromAxisAngle(V([1, 2, 3]).normalize(), .73), shift: V([.31, -.22, .48]) },
];
const report = {
  classification: reference.scope.classification,
  sourceDOI: reference.source.doi,
  publishedInputCases: ['case1', 'case2'], independentAnalyticSamples: 0,
  configurations: 0, virtualWorkChecks: 0, jointClosureChecks: 0,
  maxPointErrorM: 0, maxLengthErrorM: 0, maxMomentArmErrorM: 0,
  maxProductionStepMomentArmErrorM: 0, maxVirtualWorkErrorNm: 0,
  maxPullErrorN: 0, maxForceBalanceErrorN: 0,
};

// Verify the committed scalar reference values without relying on FYSM paths.
// l_ref_MT is deliberately not substituted for a measured geometric length.
for (const samples of reference.analyticReference.samples) {
  const muscle = published.cases.find(c => c.id === samples.case).muscles.find(m => m.id === samples.muscle);
  for (const sample of samples.values) {
    const expected = analytic(muscle, sample.angleDegrees * D);
    assert(Math.abs(expected.length - sample.lengthM) < 2e-14, 'independent scalar length reference changed');
    assert(Math.abs(expected.momentArm - sample.signedMomentArmM) < 2e-14, 'independent scalar moment-arm reference changed');
    report.independentAnalyticSamples++;
  }
}

for (const c of published.cases) for (const muscle of c.muscles) {
  const inst = instance(c.id, muscle), instSnapshot = JSON.stringify(inst);
  const neutral = analytic(muscle, 0);
  if (c.id === 'case2') assert(Math.abs(neutral.length - muscle.referenceLengthParameterM) > .01,
    'the reference contraction length must not masquerade as the Case-2 geometric length');
  for (const placement of frames) for (let degrees = -30; degrees <= 30; degrees += .5) {
    const angle = degrees * D, W = world(angle, placement), before = JSON.stringify(W);
    const axis = axisLocal.clone().applyQuaternion(placement.q);
    const center = jointWorld.clone().applyQuaternion(placement.q).add(placement.shift);
    const expected = analytic(muscle, angle), path = A.strandPath(W, inst), owned = A.ownedStrandPath(W, inst);
    report.configurations++;
    assert.equal(path.length, muscle.points.length, 'the published via-point path gained or lost nodes');
    assert.equal(owned.length, path.length);
    for (let i = 0; i < path.length; i++) {
      const point = V(expected.points[i]).applyQuaternion(placement.q).add(placement.shift);
      const error = point.distanceTo(path[i]); report.maxPointErrorM = Math.max(report.maxPointErrorM, error);
      assert(error < 2e-14, 'production path differs from published local coordinates and analytic rigid transforms');
      assert.equal(owned[i].seg, mapping[muscle.points[i].body], 'via-point force owner differs from the published body');
      assert(owned[i].p.distanceTo(path[i]) < 1e-15, 'owned and length-only production traces disagree');
    }
    const length = A.pathLength(path), lengthError = Math.abs(length - expected.length);
    report.maxLengthErrorM = Math.max(report.maxLengthErrorM, lengthError);
    assert(lengthError < 2e-14, 'production path length disagrees with the external-input analytic reference');

    const j1 = A.pt(W, mapping.body1, A.S[mapping.body1].J.clone().add(V(joint.body1LocalM)));
    const j2 = A.pt(W, mapping.body2, A.S[mapping.body2].J.clone().add(V(joint.body2LocalM)));
    assert(j1.distanceTo(center) < 2e-14 && j2.distanceTo(center) < 2e-14, 'published revolute joint opened');
    report.jointClosureChecks++;

    for (const h of [1e-6, .3 * D]) {
      const plus = physics.rotW(W, dof, h, axis, center), minus = physics.rotW(W, dof, -h, axis, center);
      const momentArm = -(A.pathLength(A.strandPath(plus, inst)) - A.pathLength(A.strandPath(minus, inst))) / (2 * h);
      const error = Math.abs(momentArm - expected.momentArm);
      if (h === 1e-6) {
        report.maxMomentArmErrorM = Math.max(report.maxMomentArmErrorM, error);
        assert(error < 8e-10, 'production length derivative disagrees with the analytic signed moment arm');
      } else {
        report.maxProductionStepMomentArmErrorM = Math.max(report.maxProductionStepMomentArmErrorM, error);
        assert(error < 1.5e-6, 'the solver-size finite-difference step has excessive geometric truncation error');
      }
    }

    // Unit tension is a selected component probe, not an imported author force.
    // Assemble the tension forces on every node from the actual owned trace,
    // then use the production virtual-displacement and reaction-force helpers.
    const forces = owned.map(() => V([0, 0, 0]));
    for (let i = 1; i < owned.length; i++) {
      const pull = owned[i].p.clone().sub(owned[i - 1].p).normalize();
      forces[i - 1].add(pull); forces[i].sub(pull);
    }
    let work = 0;
    for (let i = 0; i < owned.length; i++) {
      const velocity = physics.dpdq(dof, owned[i].seg, owned[i].p, axis, center);
      if (velocity) work += forces[i].dot(velocity);
    }
    const workError = Math.abs(work - expected.momentArm);
    report.maxVirtualWorkErrorNm = Math.max(report.maxVirtualWorkErrorNm, workError);
    report.virtualWorkChecks++;
    assert(workError < 2e-14, 'unit-tension virtual work disagrees with -dL/dq');
    const distalPull = physics.pullOn(owned, physics.DOF[dof].moved, 1);
    const expectedPull = path.at(-2).clone().sub(path.at(-1)).normalize();
    const pullError = distalPull.distanceTo(expectedPull); report.maxPullErrorN = Math.max(report.maxPullErrorN, pullError);
    assert(pullError < 2e-14, 'production reaction helper applied an internal via-point force to the wrong body');
    const proximalPull = physics.pullOn(owned, new Set([mapping.body1]), 1);
    const balance = distalPull.clone().add(proximalPull).length();
    report.maxForceBalanceErrorN = Math.max(report.maxForceBalanceErrorN, balance);
    assert(balance < 2e-14, 'fixed and moving bodies do not receive equal/opposite total tension forces');
    assert(JSON.stringify(W) === before, 'geometry verification mutated a supplied world frame');
  }
  assert.equal(JSON.stringify(inst), instSnapshot, 'production path evaluation mutated a published-input instance');
}

// Table 3 instantaneous velocity consistency only: do not equate the printed
// angular rate to a mathematically exact 30-degree/2-second sinusoid.
const neutralWorld = world(0, frames[0]);
for (const body of ['body1', 'body2']) {
  assert.deepEqual(published.bodies[body].initialQuaternionXYZW, [0, 0, 0, 1]);
  assert(neutralWorld[mapping[body]].p.distanceTo(V(published.bodies[body].initialCenterM)) < 1e-15,
    'Table-2 joint geometry does not reproduce the Table-3 initial body center');
  assert(Math.abs(Math.abs(joint[body + 'LocalM'][2]) - published.bodies[body].lengthM / 2) < 1e-15,
    'the published joint is not at the end of its Table-1 body');
}
const groundJoint = A.pt(neutralWorld, mapping.body1,
  A.S[mapping.body1].J.clone().add(V(published.joints.J0.body1LocalM)));
assert(groundJoint.length() < 1e-15, 'the published proximal fixed joint is not at the global origin');
const velocity = physics.dpdq(dof, mapping.body2, neutralWorld[mapping.body2].p, axisLocal, jointWorld)
  .multiplyScalar(published.motion.body2InitialAngularVelocityRadS[1]);
const velocityError = velocity.distanceTo(V(published.motion.body2InitialCenterVelocityMS));
const printedRoundingBound = (1 + .5) * .5 * 10 ** -published.motion.velocitySourceRoundingDecimalPlaces;
assert(velocityError <= printedRoundingBound + 1e-15, 'published instantaneous center/angular velocities are inconsistent with the joint geometry');
report.printedInitialVelocityErrorMS = velocityError;
report.printedInitialVelocityRoundingBoundMS = printedRoundingBound;

assert.equal(JSON.stringify(A.strandInstances()), sourceSnapshot, 'external component checks altered the anatomical attachments');
for (const [name, origin] of Object.entries(segmentOrigins)) assert.deepEqual(A.S[name].J.toArray(), origin);
report.wholeBodySolveRun = false;
report.authorsForceOrActivationResultsReproduced = false;
report.yogaValidation = false;
console.log(JSON.stringify(report, null, 2));
