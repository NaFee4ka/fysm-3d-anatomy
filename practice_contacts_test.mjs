// Independent geometry/quadrature and mechanical-contact invariants.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { clippedFloorTriangle, createPracticeContacts, convexHull, supportMargin,
  glutealPadGeometry, glutealPadQuadrature, FINGERTIP_SOFT_OFFSET_M } from './practice/contacts.mjs';
import { outsideCanonicalFootprint, CONTACT_REFERENCE_AREA_M2 } from './practice/contacts.mjs';
import { PRACTICE_SUPPORT_INTENT, matchesPracticeSupport, practiceSurfaceRegion } from './practice/contacts.mjs';
import { decodeGroundBones } from './practice/ground_fit.mjs';
import { isVerifiedPracticeSensitivity, practiceInputProvenance, practiceCheckpointPath,
  PRACTICE_PHYSICS_SOURCES } from './practice_physics_build.mjs';

const V = (...p) => new THREE.Vector3(...p);
const near = (a, b, tolerance = 1e-11) => assert(Math.abs(a - b) <= tolerance, `${a} differs from ${b}`);
const vectorNear = (a, b, tolerance = 1e-11) => assert(a.distanceTo(b) <= tolerance, `${a.toArray()} differs from ${b.toArray()}`);
const band = .004;
const a = V(0, -.008, 0), b = V(1, .008, 0), c = V(0, .008, 1);
const clipped = clippedFloorTriangle(a, b, c, THREE, band);
near(clipped.areaM2, .25);
near(clipped.point.x, 13 / 48);
near(clipped.point.z, 13 / 48);
near(clipped.point.y, 1 / 1500);
const ab = a.clone().lerp(b, .5), bc = b.clone().lerp(c, .5), ca = c.clone().lerp(a, .5);
const pieces = [[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]
  .map(points => clippedFloorTriangle(...points, THREE, band)).filter(Boolean);
near(pieces.reduce((sum, p) => sum + p.areaM2, 0), clipped.areaM2);
const firstMoment = pieces.reduce((sum, p) => sum.addScaledVector(p.point, p.areaM2), V(0, 0, 0));
vectorNear(firstMoment, clipped.point.clone().multiplyScalar(clipped.areaM2));
assert.equal(clippedFloorTriangle(V(0, band + 1e-8, 0), V(1, band + 1e-8, 0), V(0, band + 1e-8, 1), THREE), null);
assert.equal(clippedFloorTriangle(V(0, -band - 1e-8, 0), V(1, -band - 1e-8, 0), V(0, -band - 1e-8, 1), THREE), null);
near(clippedFloorTriangle(V(0, band, 0), V(1, band, 0), V(0, band, 1), THREE).areaM2, .5);
const yaw = new THREE.Quaternion().setFromAxisAngle(V(0, 1, 0), .73), shift = V(.9, 0, -.2);
const moved = clippedFloorTriangle(...[a, b, c].map(p => p.clone().applyQuaternion(yaw).add(shift)), THREE, band);
near(moved.areaM2, clipped.areaM2);
vectorNear(moved.point, clipped.point.clone().applyQuaternion(yaw).add(shift));
// Two overlapping local reference cells must be subtracted as a union, not
// twice. The independent oracle is a1m×0.5m rectangle inside a2m² triangle.
const whole = [V(0, 0, 0), V(2, 0, 0), V(0, 0, 2)];
const cells = [[[0, 0], [.75, 0], [.75, .5], [0, .5]],
  [[.25, 0], [1, 0], [1, .5], [.25, .5]]];
const outside = points => {
  let pieces = [clippedFloorTriangle(...points, THREE)];
  for (const cell of cells) pieces = pieces.flatMap(piece => outsideCanonicalFootprint(piece, cell, THREE));
  return pieces;
};
const remaining = outside(whole);
near(remaining.reduce((sum, p) => sum + p.areaM2, 0), 1.5);
vectorNear(remaining.reduce((sum, p) => sum.addScaledVector(p.point, p.areaM2), V(0, 0, 0)), V(13 / 12, 0, 29 / 24));
const mid = [whole[0].clone().lerp(whole[1], .5), whole[1].clone().lerp(whole[2], .5), whole[2].clone().lerp(whole[0], .5)];
const subtracted = [[whole[0], mid[0], mid[2]], [mid[0], whole[1], mid[1]],
  [mid[2], mid[1], whole[2]], mid].flatMap(outside);
near(subtracted.reduce((sum, p) => sum + p.areaM2, 0), 1.5);
vectorNear(subtracted.reduce((sum, p) => sum.addScaledVector(p.point, p.areaM2), V(0, 0, 0)), V(13 / 12, 0, 29 / 24));
const poly = convexHull([[0, 0], [1, 0], [1, 1], [0, 1], [.5, .5], [0, 0]]);
near(supportMargin(poly, [.5, .5]), .5); near(supportMargin(poly, [1.2, .5]), -.2);
assert.equal(supportMargin([[0, 0], [1, 0]], [.5, 0]), null);

const A = createAnatomy(THREE), PH = makePhysics(A), root = A.S.pelvis.J.clone();
const W = A.computeWorld(A.STAND.local, A.STAND.rootQ, root);
const mermaidFingerIntent = PRACTICE_SUPPORT_INTENT['detali-08'].find(group => group.side === 'L'
  && group.regions.includes('пальцы рук'));
assert.equal(practiceSurfaceRegion('thenar', 'f1m_L', W.f1m_L.p, W), 'ладони');
assert.equal(matchesPracticeSupport(mermaidFingerIntent, { side: 'L', region: 'пальцы рук',
  source: { kind: 'muscle-surface-quadrature' } }), false);
assert.equal(matchesPracticeSupport(mermaidFingerIntent, { side: 'L', region: 'пальцы рук',
  source: { kind: 'distal-phalanx-skin-proxy' } }), true);
const P = V(.2, .8, .15), F = V(35, 80, -12), segA = 'hand_L', segB = 'foot_L';
const pointOn = (world, seg, original) => {
  const local = original.clone().sub(W[seg].p).applyQuaternion(W[seg].q.clone().invert()).add(A.S[seg].J);
  return A.pt(world, seg, local);
};
let virtualWorkCases = 0;
for (let k = 0; k < PH.DOF.length; k++) {
  const dof = PH.DOF[k], axis = dof.axis(W), centre = dof.center?.(W) || null;
  const va = PH.dpdq(k, segA, P, axis, centre) || V(0, 0, 0);
  const vb = PH.dpdq(k, segB, P, axis, centre) || V(0, 0, 0);
  const work = va.clone().sub(vb).dot(F), delta = 1e-6;
  const plus = PH.rotW(W, k, delta, axis, centre), minus = PH.rotW(W, k, -delta, axis, centre);
  const differential = pointOn(plus, segA, P).sub(pointOn(minus, segA, P))
    .sub(pointOn(plus, segB, P).sub(pointOn(minus, segB, P))).multiplyScalar(1 / (2 * delta));
  near(work, differential.dot(F), 2e-8);
  if (dof.joint === 'root') near(work, 0, 1e-12);
  virtualWorkCases++;
}
vectorNear(F.clone().add(F.clone().negate()), V(0, 0, 0));
vectorNear(P.clone().cross(F).add(P.clone().cross(F.clone().negate())), V(0, 0, 0));
// Applying the same force pair at different points would create an external
// couple. This known nonzero example guards the reason for one common P.
assert(V(.1, 0, 0).cross(F).length() > 1);

function packageTriangle(points, indices) {
  const lo = [0, 1, -.10], hi = [.10, 1, 0];
  const positions = Uint16Array.from(points.flatMap(point => point.map((value, axis) =>
    hi[axis] === lo[axis] ? 0 : Math.round((value - lo[axis]) / (hi[axis] - lo[axis]) * 65535))));
  for (let i = positions.length - 1; i >= 3; i--) positions[i] = (positions[i] - positions[i - 3]) & 65535;
  const triangles = Uint16Array.from(indices);
  for (let i = triangles.length - 1; i > 0; i--) triangles[i] = (triangles[i] - triangles[i - 1]) & 65535;
  return { v: 1, parts: [{ unit: 'gluteMax', side: 'L', n: points.length, m: indices.length / 3, lo, hi }],
    data: gzipSync(Buffer.concat([Buffer.from(positions.buffer), Buffer.from(triangles.buffer)])).toString('base64') };
}
const triangle = [[0, 1, -.10], [.10, 1, -.10], [0, 1, 0]];
const split = triangle.concat([[.05, 1, -.10], [.05, 1, -.05], [0, 1, -.05]]);
const packageA = packageTriangle(triangle, [0, 1, 2]);
const packageB = packageTriangle(split, [0, 3, 5, 3, 1, 4, 5, 4, 2, 3, 4, 5]);
const flatWorld = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.clone().sub(V(0, 1, 0)));
const totals = [];
for (const data of [packageA, packageB]) {
  const contact = await createPracticeContacts(A, THREE, data);
  try {
    const result = contact.buildFloor(flatWorld);
    const samples = result.contacts.filter(c => c.source.kind === 'muscle-surface-quadrature');
    assert(samples.length, 'the nonpenetrating rendered floor triangle must create actual surface quadrature');
    totals.push({ area: samples.reduce((s, c) => s + c.areaM2, 0), stiffness: samples.reduce((s, c) => s + c.k, 0),
      moment: samples.reduce((s, c) => s.addScaledVector(c.P, c.areaM2), V(0, 0, 0)) });
  } finally { contact.dispose(); }
}
near(totals[0].area, totals[1].area, 2e-7);
near(totals[0].stiffness, totals[1].stiffness, 1e-3);
vectorNear(totals[0].moment, totals[1].moment, 2e-8);
// A deliberately penetrating *whole* rendered unit must never yield a floor
// support merely by clipping the stripe where its triangles cross the plane.
const penetrating = structuredClone(packageA);
penetrating.parts[0].lo[1] = .90; penetrating.parts[0].hi[1] = 1.10;
const raw = new Uint16Array([0, 0, 0, 65535, 65535, 0, 0, 65535, 65535]);
for (let i = raw.length - 1; i >= 3; i--) raw[i] = (raw[i] - raw[i - 3]) & 65535;
const tri = new Uint16Array([0, 1, 1]);
penetrating.data = gzipSync(Buffer.concat([Buffer.from(raw.buffer), Buffer.from(tri.buffer)])).toString('base64');
const factory = await createPracticeContacts(A, THREE, penetrating);
try {
  const result = factory.buildFloor(flatWorld);
  assert(result.diagnostics.maxSurfacePenetrationM > .09);
  assert.equal(result.diagnostics.surfaceGeometryValid, false);
  assert(result.diagnostics.penetratingSurfaceRegions[0].excludedFromFloorQuadrature);
  assert.equal(result.contacts.filter(c => c.source.kind === 'muscle-surface-quadrature').length, 0);
} finally { factory.dispose(); }
const shallowFactory = await createPracticeContacts(A, THREE, packageA);
try {
  const shallowWorld = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.clone().sub(V(0, 1.002, 0)));
  const result = shallowFactory.buildFloor(shallowWorld);
  assert.equal(result.diagnostics.surfaceGeometryValid, true);
  near(result.diagnostics.maxSurfacePenetrationM, .002, 1e-7);
  near(result.diagnostics.surfaceMinimum.heightM, -.002, 1e-7);
  assert(result.diagnostics.belowFloorVertices > 0);
} finally { shallowFactory.dispose(); }
// The absent gluteal soft-tissue hypothesis has fixed geometric inputs. Its
// positive disk quadrature integrates area, first moments and second moments,
// independently of force optimization and independently of the model COM.
const gluteIndex = A.LANDMARKS.findIndex(l => l.region === 'ягодицы' && l.side === 'L');
const pads = [1, .5].map(scale => glutealPadGeometry(A, THREE, gluteIndex, W, scale));
for (const pad of pads) {
  near(pad.curvatureRadiusM, .04); near(pad.maximumCompressionM, .012);
  near(pad.contactRadiusM ** 2, (2 * .04 * .012 - .012 ** 2) * pad.areaScale);
  const points = glutealPadQuadrature(pad, THREE), cx = pad.center.x, cz = pad.center.z;
  assert(points.every(p => p.areaM2 > 0 && p.point.y === 0));
  near(points.reduce((s, p) => s + p.areaM2, 0), pad.areaM2);
  near(points.reduce((s, p) => s + p.areaM2 * (p.point.x - cx), 0), 0);
  near(points.reduce((s, p) => s + p.areaM2 * (p.point.z - cz), 0), 0);
  for (const axis of ['x', 'z']) near(points.reduce((s, p) => s + p.areaM2 * (p.point[axis] - pad.center[axis]) ** 2, 0),
    pad.areaM2 * pad.contactRadiusM ** 2 / 4);
  near(points.reduce((s, p) => s + p.areaM2 * (p.point.x - cx) * (p.point.z - cz), 0), 0);
}
near(pads[1].areaM2, pads[0].areaM2 / 2);
assert.throws(() => glutealPadGeometry(A, THREE, gluteIndex, W, .8));

// The finger proxy follows actual distal atlas vertices, not a displaced generic
// fingertip landmark. Its full geometry query includes airborne points, while
// force identification accepts only points within the original contact band.
const boneData = JSON.parse(fs.readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url)));
const decodedBones = decodeGroundBones(boneData, A);
const fingerFactory = await createPracticeContacts(A, THREE, packageA, { boneData });
try {
  const proxies = fingerFactory.groundProxies(W, { id: 'detali-08' });
  assert.equal(proxies.length, 10);
  assert.equal(fingerFactory.groundProxies(W, { id: 'detali-07' }).length, 0);
  for (const proxy of proxies) {
    const source = proxy.source, bone = decodedBones[source.bonePartIndex];
    const local = new THREE.Vector3().fromArray(bone.positions, source.boneVertex * 3);
    const actual = local.applyQuaternion(W[proxy.seg].q).add(W[proxy.seg].p);
    vectorNear(actual, V(...source.boneWorldPoint));
    vectorNear(proxy.point, actual.clone().add(V(0, -FINGERTIP_SOFT_OFFSET_M, 0)));
    assert(proxy.point.y > band, 'standing finger geometry must remain airborne');
  }
  const standingContacts = fingerFactory.buildFloor(W, { id: 'detali-08' });
  assert.equal(standingContacts.contacts.filter(c => c.source.kind === 'distal-phalanx-skin-proxy').length, 0);
  assert.equal(standingContacts.diagnostics.inferredPads.length, 0);
  const markerMinimum = Math.min(...A.LANDMARKS.map(l => A.pt(W, l.seg, l.pc).y));
  const canonicalWorld = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.clone().sub(V(0, markerMinimum, 0)));
  const canonicalContacts = fingerFactory.buildFloor(canonicalWorld);
  const referenceGroups = new Map();
  for (const cell of canonicalContacts.diagnostics.canonicalPressureCells) {
    referenceGroups.set(cell.family, (referenceGroups.get(cell.family) || 0) + cell.areaM2);
    near(Math.abs(cell.footprint.reduce((s, p, i, a) => s + p[0] * a[(i + 1) % a.length][1]
      - a[(i + 1) % a.length][0] * p[1], 0)) / 2, cell.areaM2);
  }
  assert(referenceGroups.size >= 2);
  for (const area of referenceGroups.values()) near(area, CONTACT_REFERENCE_AREA_M2);
  const bottom = Math.min(...proxies.map(p => p.point.y));
  const lowered = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.clone().sub(V(0, bottom, 0)));
  const touching = fingerFactory.buildFloor(lowered, { id: 'detali-08' });
  const fingerContacts = touching.contacts.filter(c => c.source.kind === 'distal-phalanx-skin-proxy');
  assert(fingerContacts.length > 0);
  assert(fingerContacts.every(c => c.source.boneWorldPoint[1] >= -1e-6
    && c.source.offsetM === .002 && c.h <= band && c.P.y === 0));
} finally { fingerFactory.dispose(); }
const validSensitivity = { kind: 'two-deterministic-geometric-area-hypotheses', verified: true,
  isConfidenceInterval: false, hypotheses: [1, .5].map(areaScale => ({ areaScale,
    contactAreaM2: .005 * areaScale, contactGeometryVerified: true, physicalBalance: { verified: true },
    quality: { status: 'solved', converged: true, finite: true } })) };
assert.equal(isVerifiedPracticeSensitivity(validSensitivity), true);
assert.equal(isVerifiedPracticeSensitivity(undefined), false, 'omitting required sensitivity cannot verify a full bundle');
for (const mutate of [s => { s.hypotheses.pop(); }, s => { s.hypotheses[1].areaScale = 1; },
  s => { s.isConfidenceInterval = true; }, s => { s.hypotheses[1].physicalBalance.verified = false; },
  s => { s.hypotheses[1].quality.converged = false; }, s => { s.hypotheses[1].contactGeometryVerified = false; }]) {
  const invalid = structuredClone(validSensitivity); mutate(invalid);
  assert.equal(isVerifiedPracticeSensitivity(invalid), false);
}
assert.equal(practiceInputProvenance({ onlyIds: ['struna-01'], iters: 20000 }).repositoryBound, true);
for (const key of ['anatomy', 'physics', 'poseData', 'manifest', 'contacts']) {
  const provenance = practiceInputProvenance({ [key]: {} });
  assert.equal(provenance.repositoryBound, false);
  assert.deepEqual(provenance.callerOverrides, [key]);
}
assert.equal(practiceCheckpointPath('/tmp/fysm-practice-checkpoint-test.json'), '/tmp/fysm-practice-checkpoint-test.json');
for (const name of [...PRACTICE_PHYSICS_SOURCES, 'practice/physics.json', 'practice/physics-check.json',
  'practice/check.json', 'physics.json', 'poses.json', 'cycle.json', 'dist/index.html'])
  assert.throws(() => practiceCheckpointPath(new URL('./' + name, import.meta.url).pathname), /production file|physics source/);
console.log(JSON.stringify({ passed: true, triangleSubdivisionAreaAndFirstMoment: true,
  floorFrameInvariance: true, virtualWorkCases, commonPointActionReaction: true,
  densityInvariantSurfaceAreaAndStiffness: true, deepPenetrationStripeRejected: true,
  geometricSoftPadMoments: true, distalBoneSkinProxyAndAirborneExclusion: true,
  requiredSensitivityPublicationGuard: true, repositoryInputProvenanceGuard: true }));
