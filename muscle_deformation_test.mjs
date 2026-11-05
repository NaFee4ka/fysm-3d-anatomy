// Analytic rigid/half-turn tests and a GPU-fixture exporter for the actual DQS.
// This verifies a display deformation, not a constitutive model of muscle.
import assert from 'node:assert/strict';
import { gzipSync } from 'node:zlib';
import * as THREE from 'three';
import { blendMuscleDualQuaternion, transformMuscleDualQuaternion,
  deformMuscleVertex, muscleDualQuaternionGLSL, createMuscleSurfaces } from './muscle_surface_render.js';

const V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);
const Q = (...values) => new THREE.Quaternion(...values);
function palette(transforms) {
  const real = new Float32Array(transforms.length * 4), dual = new Float32Array(real.length);
  transforms.forEach(({ q, t }, index) => {
    q.toArray(real, index * 4);
    const d = Q(t.x, t.y, t.z, 0).multiply(q);
    for (let k = 0; k < 4; k++) dual[index * 4 + k] = d.toArray()[k] / 2;
  });
  return { real, dual };
}
const transform = (point, ids, weights, p) => transformMuscleDualQuaternion(point,
  blendMuscleDualQuaternion(ids, weights, p.real, p.dual), V());
let seed = 180431;
const random = () => ((seed = Math.imul(seed, 1664525) + 1013904223 | 0) >>> 0) / 4294967296;
const vector = (size = 1) => V((random() - .5) * size, (random() - .5) * size, (random() - .5) * size);
const rotation = () => Q().setFromEuler(new THREE.Euler((random() - .5) * 6,
  (random() - .5) * 6, (random() - .5) * 6));
let maxRigidError = 0, maxAntipodalError = 0, maxCovariantError = 0, maxHybridCovariantError = 0;

// A single rigid support must reproduce the independently evaluated rotation
// and translation, including large translations and arbitrary bone slots.
for (let test = 0; test < 128; test++) {
  const q = rotation(), t = vector(8), point = vector(3), p = palette([{ q, t }]);
  const actual = transform(point, [0, 0, 0, 0], [1, 0, 0, 0], p);
  const expected = point.clone().applyQuaternion(q).add(t);
  maxRigidError = Math.max(maxRigidError, actual.distanceTo(expected));
}
assert(maxRigidError < 1e-6, 'DQS changed a single rigid support');

// At a half-turn ordinary LBS collapses the middle ring of a twisted cylinder
// onto its axis. DQS must retain the independently known circular cross section.
const identity = { q: Q(), t: V() }, half = { q: Q().setFromAxisAngle(V(0, 1, 0), Math.PI), t: V() };
const twist = palette([identity, half]);
const ring = [], linearRing = [], hybridRing = [];
for (let k = 0; k < 32; k++) {
  const a = k * 2 * Math.PI / 32, point = V(.05 * Math.cos(a), .5, .05 * Math.sin(a));
  const actual = transform(point, [0, 1, 0, 0], [.5, .5, 0, 0], twist);
  const hybrid = deformMuscleVertex(point, V(0, 1, 0), [0, 1, 0, 0], [.5, .5, 0, 0], twist.real, twist.dual, 0, V());
  const expected = point.clone().applyAxisAngle(V(0, 1, 0), Math.PI / 2);
  assert(actual.distanceTo(expected) < 1e-9);
  const linear = point.clone().multiplyScalar(.5).addScaledVector(point.clone().applyQuaternion(half.q), .5);
  assert(hybrid.distanceTo(linear) < 1e-9, 'the half-turn continuity plateau must equal the analytic LBS limit');
  assert(Math.abs(Math.hypot(actual.x, actual.z) - .05) < 1e-10);
  const reordered = transform(point, [1, 0, 0, 0], [.5, .5, 0, 0], twist);
  assert(reordered.distanceTo(actual) < 1e-10, 'half-turn tie changed when the two slots were reordered');
  ring.push(actual);
  linearRing.push(linear); hybridRing.push(hybrid);
}
const area = points => Math.abs(points.reduce((sum, p, k) => {
  const next = points[(k + 1) % points.length]; return sum + p.x * next.z - next.x * p.z;
}, 0)) / 2;
const expectedArea = 32 * .05 ** 2 * Math.sin(2 * Math.PI / 32) / 2;
assert(Math.abs(area(ring) - expectedArea) < 1e-12);
assert(area(linearRing) < 1e-20, 'the comparison fixture must actually reproduce LBS collapse');
assert(area(hybridRing) < 1e-20, 'balanced half-turn hybrid collapse is a declared geometric limitation');
// Quantized weights arbitrarily close to balanced do not preserve thickness
// either: the analytic linear area ratio is (w0-w1)^2 at a half-turn.
const quantizedWeight = 32767 / 65535;
const quantizedRing = Array.from({ length: 32 }, (_, k) => {
  const point = V(.05 * Math.cos(k * 2 * Math.PI / 32), .5, .05 * Math.sin(k * 2 * Math.PI / 32));
  return deformMuscleVertex(point, V(0, 1, 0), [0, 1, 0, 0],
    [quantizedWeight, 1 - quantizedWeight, 0, 0], twist.real, twist.dual, 0, V());
});
const quantizedAreaRatio = area(quantizedRing) / expectedArea;
assert(Math.abs(quantizedAreaRatio - (2 * quantizedWeight - 1) ** 2) < 1e-15,
  'near-balanced quantized half-turn must expose its analytic loss of thickness');
const hingePalette = palette([identity, { q: Q().setFromAxisAngle(V(1, 0, 0), Math.PI), t: V() }]);
const hingePoint = V(.05, .5, 0);
const hingeDQS = transform(hingePoint, [0, 1, 0, 0], [.5, .5, 0, 0], hingePalette);
const hingeHybrid = deformMuscleVertex(hingePoint, V(0, 1, 0), [0, 1, 0, 0], [.5, .5, 0, 0],
  hingePalette.real, hingePalette.dual, 0, V());
assert(hingeHybrid.distanceTo(V(.05, 0, 0)) < 1e-9,
  'a half-turn hinge must equal its continuous LBS limit, including loss of cross-section');
assert(hingeDQS.distanceTo(V(.05, 0, 0)) > .49, 'raw DQS hinge comparison must remain independent');
for (let sample = 0; sample < 32; sample++) {
  const commonQ = rotation(), commonT = vector(2), point = vector();
  const movedPalette = palette([identity, half].map(({ q, t }) => ({
    q: commonQ.clone().multiply(q), t: t.clone().applyQuaternion(commonQ).add(commonT),
  })));
  for (const weight of [.499999, .5, .500001]) {
    const expected = transform(point, [0, 1, 0, 0], [weight, 1 - weight, 0, 0], twist)
      .applyQuaternion(commonQ).add(commonT);
    const actual = transform(point, [0, 1, 0, 0], [weight, 1 - weight, 0, 0], movedPalette);
    assert(actual.distanceTo(expected) < 2e-6, 'a half-turn tie lost whole-body rotation covariance');
  }
}

const fixtureTransforms = Array.from({ length: 4 }, () => ({ q: rotation(), t: vector(.8) }));
const fixturePalette = palette(fixtureTransforms), cases = [];
for (let test = 0; test < 128; test++) {
  const point = vector(3), ids = [0, 1, 2, 3], rawWeights = [random(), random(), random(), random()];
  const sum = rawWeights.reduce((a, b) => a + b, 0);
  const weights = Float32Array.from(rawWeights, weight => weight / sum);
  const base = transform(point, ids, weights, fixturePalette);
  const localAxis = vector().normalize();
  const hybridBase = deformMuscleVertex(point, localAxis, ids, weights, fixturePalette.real, fixturePalette.dual, 0, V());
  const dq = blendMuscleDualQuaternion(ids, weights, fixturePalette.real, fixturePalette.dual);
  assert(Math.abs(Math.hypot(...dq.slice(0, 4)) - 1) < 1e-12);
  assert(Math.abs(dq[0] * dq[4] + dq[1] * dq[5] + dq[2] * dq[6] + dq[3] * dq[7]) < 1e-12);
  // A quaternion and its negative describe exactly the same rigid transform.
  for (let signs = 0; signs < 16; signs++) {
    const p = { real: fixturePalette.real.slice(), dual: fixturePalette.dual.slice() };
    for (let bone = 0; bone < 4; bone++) if (signs & 1 << bone)
      for (let k = 0; k < 4; k++) { p.real[bone * 4 + k] *= -1; p.dual[bone * 4 + k] *= -1; }
    maxAntipodalError = Math.max(maxAntipodalError, base.distanceTo(transform(point, ids, weights, p)));
    assert(hybridBase.distanceTo(deformMuscleVertex(point, localAxis, ids, weights, p.real, p.dual, 0, V())) < 1e-12,
      'bend-aware skinning changed when a quaternion sign was negated');
  }
  // A common world rotation/translation must act outside the deformation.
  const commonQ = rotation(), commonT = vector(2);
  const commonPalette = palette(fixtureTransforms.map(({ q, t }) => ({
    q: commonQ.clone().multiply(q), t: t.clone().applyQuaternion(commonQ).add(commonT),
  })));
  const moved = transform(point, ids, weights, commonPalette);
  maxCovariantError = Math.max(maxCovariantError, moved.distanceTo(base.clone().applyQuaternion(commonQ).add(commonT)));
  const hybridMoved = deformMuscleVertex(point, localAxis, ids, weights, commonPalette.real, commonPalette.dual, 0, V());
  maxHybridCovariantError = Math.max(maxHybridCovariantError, hybridMoved.distanceTo(hybridBase.clone().applyQuaternion(commonQ).add(commonT)));
  const radial = vector(.03), belly = Math.fround(random()), scale = Math.fround(.92 + random() * .16);
  const corrected = point.clone().addScaledVector(radial, (scale - 1) * belly);
  cases.push({ point: point.toArray(), ids, weights: [...weights], axis: localAxis.toArray(), radial: [...radial.toArray(), belly], scale,
    expected: deformMuscleVertex(corrected, localAxis, ids, weights, fixturePalette.real, fixturePalette.dual, 0, V()).toArray() });
}
assert(maxAntipodalError < 1e-12);
assert(maxCovariantError < 2e-6, 'DQS lost whole-body rigid covariance');
assert(maxHybridCovariantError < 2e-6, 'bend-aware skinning lost whole-body rigid covariance');

// Exercise the public SkinnedMesh vertex query and actual triangle raycaster,
// not only the numeric helper. At 150 degrees this strip is outside the new
// half-turn window, so the original analytical DQS/raycast behavior remains.
const segments = [{ name: 'root', parent: null, J: V() },
  { name: 'a', parent: 'root', J: V() }, { name: 'b', parent: 'root', J: V(0, 1, 0) }];
const restWorld = { root: { p: V(), q: Q() }, a: { p: V(), q: Q() }, b: { p: V(0, 1, 0), q: Q() } };
const inst = { id: 'analytic-strip', unit: 'strip', side: 'L', kind: 'muscle',
  pts: [{ seg: 'a', p: V() }, { seg: 'b', p: V(0, 1, 0) }] };
const model = { SEG: segments, S: Object.fromEntries(segments.map(s => [s.name, s])), UNITS: { strip: {} },
  STAND: { local: {}, rootQ: Q() }, computeWorld: () => restWorld, strandInstances: () => [inst],
  pt: (W, seg, p) => p.clone().sub(model.S[seg].J).applyQuaternion(W[seg].q).add(W[seg].p),
  strandPath: W => inst.pts.map(p => model.pt(W, p.seg, p.p)),
  pathLength: points => points[0].distanceTo(points[1]) };
const source = [0, 0, 0, 0, 1, 0, .04, .5, .02], lo = [0, 0, 0], hi = [.04, 1, .02];
const packedPositions = Uint16Array.from(source.map((n, k) => Math.round(n / hi[k % 3] * 65535)));
for (let k = packedPositions.length - 1; k >= 3; k--) packedPositions[k] = (packedPositions[k] - packedPositions[k - 3]) & 65535;
const packedIndices = new Uint16Array([0, 1, 1]);
const strip = await createMuscleSurfaces(model, THREE, { v: 1, parts: [{ unit: 'strip', side: 'L',
  n: 3, m: 1, lo, hi }], data: gzipSync(Buffer.concat([Buffer.from(packedPositions.buffer), Buffer.from(packedIndices.buffer)])).toString('base64') });
let raycastErrorM = null, endpointErrorM = null;
try {
  const mesh = strip.pickables[0], restBuffer = mesh.geometry.attributes.position.array.slice();
  const stripAngle = 5 * Math.PI / 6;
  const W = { ...restWorld, b: { p: V(0, 1, 0), q: Q().setFromAxisAngle(V(0, 1, 0), stripAngle) } };
  const paths = new Map([[inst.id, [V(), V(0, 1.5, 0)]]]);
  strip.update(W, paths);
  const radius = mesh.muscleDeformation.scale.value;
  const a = mesh.getVertexPosition(0, V()), b = mesh.getVertexPosition(1, V()), c = mesh.getVertexPosition(2, V());
  endpointErrorM = Math.max(a.distanceTo(V()), b.distanceTo(V(0, 1, 0)));
  assert(endpointErrorM < 1e-7, 'transverse correction moved an exact attachment');
  const weights = mesh.geometry.attributes.skinWeight.array, ids = mesh.geometry.attributes.skinIndex.array;
  let wa = 0, wb = 0;
  for (let k = 0; k < 4; k++) {
    const bone = mesh.skeleton.bones[ids[8 + k]].name;
    if (bone === 'a') wa += weights[8 + k]; else if (bone === 'b') wb += weights[8 + k];
  }
  const restMiddle = V().fromBufferAttribute(mesh.geometry.attributes.position, 2);
  const angle = 2 * Math.atan2(wb * Math.sin(stripAngle / 2), wa + wb * Math.cos(stripAngle / 2));
  const expectedMiddle = restMiddle.clone(); expectedMiddle.x *= radius; expectedMiddle.z *= radius;
  expectedMiddle.applyAxisAngle(V(0, 1, 0), angle);
  assert(c.distanceTo(expectedMiddle) < 1e-7, 'public CPU mesh query diverged from the analytic 150-degree deformation');
  assert(Math.hypot(c.x, c.z) > .04, 'public mesh query silently fell back to collapsed LBS');
  assert.deepEqual(mesh.geometry.attributes.position.array, restBuffer, 'deformation changed the supplied atlas/rest vertices');
  for (const bone of mesh.skeleton.bones) assert.deepEqual(bone.scale.toArray(), [1, 1, 1]);
  const center = a.clone().add(b).add(expectedMiddle).divideScalar(3);
  const normal = b.clone().sub(a).cross(expectedMiddle.clone().sub(a)).normalize();
  const raycaster = new THREE.Raycaster(center.clone().addScaledVector(normal, .2), normal.clone().negate());
  mesh.updateWorldMatrix(true, false);
  const hits = raycaster.intersectObject(mesh, false);
  assert(hits.length > 0, 'raycaster missed the visibly deformed triangle');
  raycastErrorM = hits[0].point.distanceTo(center);
  assert(raycastErrorM < 1e-7, 'raycast point disagrees with the deformed surface');
} finally { strip.dispose(); }

// Include exact half turns in the GPU cross-check instead of testing only
// conveniently small rotations. Four slots use the same two real transforms.
const halfFixture = palette([identity, half, identity, half]);
const halfCases = [];
for (let k = 0; k < 32; k++) {
  const point = V(.05 * Math.cos(k), .5, .05 * Math.sin(k));
  for (const ids of [[0, 1, 2, 3], [1, 0, 2, 3]]) {
    const weights = [.5, .5, 0, 0];
    halfCases.push({ point: point.toArray(), ids, weights, axis: [0, 1, 0], radial: [0, 0, 0, 0], scale: 1,
      expected: deformMuscleVertex(point, V(0, 1, 0), ids, weights, halfFixture.real, halfFixture.dual, 0, V()).toArray() });
  }
}
const fixture = { glsl: muscleDualQuaternionGLSL(4), groups: [
  { real: [...fixturePalette.real], dual: [...fixturePalette.dual], cases },
  { real: [...halfFixture.real], dual: [...halfFixture.dual], cases: halfCases },
] };
// A half-turn sign choice is discontinuous in raw DQS. Check both signs of
// the actual normalized quaternion dot, the former tie and new plateau edges,
// zero weights and small positive weights. Weights are fixed atlas inputs;
// no continuity claim is made for changing a weight through zero.
let halfTurnSweepCases = 0, maxHalfTurnPairErrorM = 0;
for (const center of [0, 1e-6, 1e-5, .05]) {
  for (const sign of [-1, 1]) for (const secondary of [0, 1e-8, .03317625820636749, .49999237048905165, .5]) {
    const positions = [];
    for (const step of [-1e-9, 1e-9]) {
      const dot = sign * center + step;
      const q = Q(0, Math.sqrt(1 - dot * dot), 0, dot);
      const transforms = [identity, { q, t: V(.02, 0, -.03) }, identity, identity];
      const p = palette(transforms), weights = Float32Array.from([1 - secondary, secondary, 0, 0]);
      const ids = [0, 1, 2, 3], point = V(.05, .5, .03), axis = V(0, 1, 0);
      const actual = deformMuscleVertex(point, axis, ids, weights, p.real, p.dual, 0, V());
      positions.push(actual);
      if (secondary === 0) assert(actual.distanceTo(point) < 1e-12, 'zero-weight half-turn must be ignored');
      if (Math.abs(dot) < 9e-6 && secondary > 0) {
        const linear = point.clone().multiplyScalar(weights[0])
          .addScaledVector(point.clone().applyQuaternion(q).add(transforms[1].t), weights[1])
          .divideScalar(weights[0] + weights[1]);
        assert(actual.distanceTo(linear) < 1e-8, 'half-turn plateau is not the independently evaluated LBS limit');
      }
      const commonQ = Q().setFromAxisAngle(V(1, 2, 3).normalize(), 1.37), commonT = V(.8, -.3, .6);
      const movedPalette = palette(transforms.map(({ q, t }) => ({ q: commonQ.clone().multiply(q),
        t: t.clone().applyQuaternion(commonQ).add(commonT) })));
      const moved = deformMuscleVertex(point, axis, ids, weights, movedPalette.real, movedPalette.dual, 0, V());
      assert(moved.distanceTo(actual.clone().applyQuaternion(commonQ).add(commonT)) < 2e-6,
        'half-turn fallback lost common-world rigid covariance');
      const negated = { real: p.real.slice(), dual: p.dual.slice() };
      for (let k = 4; k < 8; k++) { negated.real[k] *= -1; negated.dual[k] *= -1; }
      assert(actual.distanceTo(deformMuscleVertex(point, axis, ids, weights,
        negated.real, negated.dual, 0, V())) < 1e-12, 'half-turn fallback depends on quaternion sign');
      const test = { point: point.toArray(), ids, weights: [...weights], axis: axis.toArray(),
        radial: [0, 0, 0, 0], scale: 1, expected: actual.toArray(), normal: [1, 0, 0] };
      // At the rank-deficient plateau shading uses the stable reference-bone
      // convention. A physical normal is undefined on a collapsed surface.
      if (Math.abs(dot) < 9e-6 || secondary === 0) test.expectedNormal = [1, 0, 0];
      fixture.groups.push({ real: [...p.real], dual: [...p.dual], cases: [test] });
      fixture.groups.push({ real: [...negated.real], dual: [...negated.dual], cases: [test] });
      const movedTest = { ...test, expected: moved.toArray() };
      if (test.expectedNormal) movedTest.expectedNormal = V(...test.expectedNormal).applyQuaternion(commonQ).toArray();
      fixture.groups.push({ real: [...movedPalette.real], dual: [...movedPalette.dual], cases: [movedTest] });
      halfTurnSweepCases++;
    }
    maxHalfTurnPairErrorM = Math.max(maxHalfTurnPairErrorM, positions[0].distanceTo(positions[1]));
  }
}
assert(maxHalfTurnPairErrorM < 1e-7, 'infinitesimal pose changes still produce a half-turn position jump');
const gpuCasesPrepared = fixture.groups.reduce((n, group) => n + group.cases.length, 0);
if (process.argv.includes('--gpu-fixture')) console.log(JSON.stringify(fixture));
else console.log(JSON.stringify({ passed: true, rigidCases: 128, antipodalCases: 128 * 16,
  covarianceCases: 128, halfTurnRingVertices: ring.length, rawDQSCrossSectionRatio: area(ring) / expectedArea,
  hybridHalfTurnCrossSectionRatio: area(hybridRing) / expectedArea,
  quantizedHalfTurnCrossSectionRatio: quantizedAreaRatio,
  halfTurnCovarianceCases: 96, fullHingeLinearBlend: 1,
  linearBlendCrossSectionRatio: area(linearRing) / expectedArea,
  maxRigidErrorM: maxRigidError, maxAntipodalErrorM: maxAntipodalError, maxCovariantErrorM: maxCovariantError,
  maxHybridCovariantErrorM: maxHybridCovariantError,
  endpointErrorM, raycastErrorM,
  halfTurnSweepCases, maxHalfTurnPairErrorM, gpuCasesPrepared }));
