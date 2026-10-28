// Analytic geometry checks: no QP, pose generation, or visual self-assertion.
import assert from 'node:assert/strict';
import { gzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createPracticeGroundFitter, decodeGroundBones, groundedRootPosition, optimizeGroundTilt } from './practice/ground_fit.mjs';

const V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z), I = () => new THREE.Quaternion();
const A = { SEG: [{ name: 'pelvis' }, { name: 'hand_L' }], S: { pelvis: {}, hand_L: {} },
  UNITS: { outer: { deep: false }, hiddenDeep: { deep: true } },
  LANDMARKS: [{ seg: 'hand_L', pc: V(0, .08, 0), region: 'ладони', side: 'L' }],
  computeWorld: (local, rootQ, rootP) => ({ pelvis: { q: rootQ, p: rootP }, hand_L: { q: rootQ, p: rootP } }),
  pt: (W, seg, point) => point.clone().applyQuaternion(W[seg].q).add(W[seg].p),
  comOf: W => W.pelvis.p.clone().add(V(0, 1, 0)) };
let W;
function mesh(unit, coords) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(coords.flat(), 3));
  geometry.setAttribute('skinIndex', new THREE.Uint16BufferAttribute(coords.flatMap(() => [0, 0, 0, 0]), 4));
  geometry.setAttribute('skinWeight', new THREE.Float32BufferAttribute(coords.flatMap(() => [1, 0, 0, 0]), 4));
  return { userData: { unit, side: 'L' }, geometry, skeleton: { bones: [{ name: 'hand_L' }] },
    getVertexPosition: (i, out) => out.fromArray(coords[i]).applyQuaternion(W.hand_L.q).add(W.hand_L.p),
    localToWorld: p => p };
}
const coords = [[-.2, -.03, -.1], [.2, .01, -.1], [0, .02, .2]];
const outer = mesh('outer', coords), deep = mesh('hiddenDeep', [[0, -10, 0]]);
const surfaces = { pickables: [outer, deep], update: world => { W = world; } };
const bones = [{ id: 'analytic', seg: 'pelvis', positions: new Float64Array([0, -.05, 0, .2, .1, .1]) }];
const fitter = createPracticeGroundFitter({ A, THREE, surfaces, boneParts: bones });
const conf = { id: 'struna-05', local: { pelvis: I(), hand_L: I() }, rootQ: I(), rootP: V(.7, -.2, 1.4) };
const snapshot = JSON.stringify(coords), original = conf.rootP.clone();
const { conf: fitted, groundDiagnostics: d } = fitter.fit(conf);
assert(Math.abs(fitted.rootP.y - .05) < 1e-13); // 50 mm bone extremum, not marker or deep muscle.
assert.equal(fitted.rootP.x, .7); assert.equal(fitted.rootP.z, 1.4);
assert(conf.rootP.equals(original)); assert.notEqual(fitted.local.hand_L, conf.local.hand_L);
assert.equal(JSON.stringify(coords), snapshot); assert.equal(d.after.counts.muscle, 3);
assert.equal(d.after.minimum.kind, 'bone'); assert(Math.abs(d.after.minimum.heightM) < 1e-13);
assert(d.nonpenetrating); assert.equal(d.primarySupportsComplete, false);
assert(d.unresolvedPrimarySupports.some(p => p.side === 'L' && Math.abs(p.gapM - .02) < 1e-13));
assert(d.unresolvedPrimarySupports.some(p => p.side === 'R' && p.minimum === null));
assert.equal(d.localKinematicsChanged, false);
const twice = fitter.fit(fitted);
assert(twice.conf.rootP.distanceTo(fitted.rootP) < 1e-13);
// A 90-degree rigid roll makes the x=-0.2 triangle vertex the exact extremum.
const rolled = { ...conf, rootQ: new THREE.Quaternion().setFromAxisAngle(V(0, 0, 1), Math.PI / 2), rootP: V() };
assert(Math.abs(fitter.fit(rolled).conf.rootP.y - .2) < 1e-12);
assert.equal(groundedRootPosition(V(1, 2, 3), -.123, .001).y, 2.124);
assert.throws(() => groundedRootPosition(V(), NaN));
assert.throws(() => groundedRootPosition(V(), 0, -.001));
assert.throws(() => fitter.fit({ ...conf, rootQ: new THREE.Quaternion(0, 0, 0, 2) }));
// The delta-coded atlas uses modulo-65536, and its local joint offset is used once.
const bytes = Buffer.alloc(12);
[65535, 0, 32768, 1, 65535, 32768].forEach((n, i) => bytes.writeUInt16LE(n, i * 2));
const boneData = { parts: [{ name: 'delta', seg: 'pelvis', n: 2, m: 0, lo: [-1, 2, 4], hi: [1, 3, 8] }],
  data: gzipSync(bytes).toString('base64') };
const decoded = decodeGroundBones(boneData, A)[0].positions;
assert.deepEqual([...decoded.slice(0, 2)], [1, 2]);
assert.equal(decoded[3], -1); assert.equal(decoded[4], 3); assert.equal(decoded[5], 4);
assert.throws(() => decodeGroundBones({ ...boneData, parts: [{ ...boneData.parts[0], n: 3 }] }, A));
assert.throws(() => decodeGroundBones({ ...boneData, parts: [{ ...boneData.parts[0], seg: '__proto__' }] }, A));
// Two support points with a 20-mm height difference over a 2-m baseline
// define the independent plane angle atan(.02/2), without any limb change.
const plane = optimizeGroundTilt([
  { point: [0, 0, -1], kind: 'marker', region: 'ладони', side: 'L' },
  { point: [0, .02, 1], kind: 'marker', region: 'ладони', side: 'R' },
  { point: [0, .3, 0], kind: 'bone' },
], [{ regions: ['ладони'], side: 'L' }, { regions: ['ладони'], side: 'R' }], THREE,
{ maxPitchDeg: 2, maxRollDeg: 0, targetGapM: 0 });
assert(Math.abs(plane.pitchDeg - Math.atan(.01) * 180 / Math.PI) < .025);
assert(Math.max(...plane.gapsM) < .0006);
assert.equal(plane.rollDeg, 0);
assert.throws(() => optimizeGroundTilt([], [], THREE));
assert.throws(() => optimizeGroundTilt([{ point: [0, 0, 0] }], [], THREE, { maxPitchDeg: 21 }));
const rectangle = [-.2, .2].flatMap(x => [-.3, .3].map(z => ({ kind: 'marker', region: 'таз', point: [x, 0, z] })));
const balanceOptions = { maxPitchDeg: 0, maxRollDeg: 0, balance: true, com: [.05, 1, 0] };
assert(Math.abs(optimizeGroundTilt(rectangle, [], THREE, balanceOptions).geometryMarginM - .15) < 1e-12);
assert(Math.abs(optimizeGroundTilt(rectangle, [], THREE, { ...balanceOptions, com: [.25, 1, 0] }).geometryMarginM + .05) < 1e-12);
assert.throws(() => optimizeGroundTilt(rectangle, [], THREE, { balance: true }));
// Exercise the complete root transform around a non-origin COM, independently
// transforming every original point, then check a second settled fit.
const Aplane = { ...A, SEG: [{ name: 'pelvis' }, { name: 'hand_L' }, { name: 'hand_R' }],
  S: { pelvis: {}, hand_L: {}, hand_R: {} },
  LANDMARKS: [{ seg: 'hand_L', pc: V(0, 0, -1), region: 'ладони', side: 'L' },
    { seg: 'hand_R', pc: V(0, .02, 1), region: 'ладони', side: 'R' }],
  computeWorld: (local, rootQ, rootP) => Object.fromEntries(['pelvis', 'hand_L', 'hand_R'].map(k => [k, { q: rootQ, p: rootP }])),
  comOf: W => V(0, 1, 0).applyQuaternion(W.pelvis.q).add(W.pelvis.p) };
const planeSurfaces = { pickables: [], update: () => {} };
const planeFitter = createPracticeGroundFitter({ A: Aplane, THREE, surfaces: planeSurfaces,
  boneParts: [{ id: 'body', seg: 'pelvis', positions: new Float64Array([0, .3, 0]) }] });
const initial = { id: 'struna-05', local: { pelvis: I(), hand_L: I(), hand_R: I() }, rootQ: I(), rootP: V(.4, .7, -1.2) };
const frozen = planeFitter.measure(initial, { cloud: true });
const settled = planeFitter.fit(initial, { rigidTilt: { maxPitchDeg: 2, maxRollDeg: 0 } });
assert(settled.groundDiagnostics.primarySupportsComplete);
assert(settled.conf.rootQ.angleTo(initial.rootQ) > .008);
const delta = settled.conf.rootQ.clone().multiply(initial.rootQ.clone().invert());
const independentHeights = frozen.points.map(p => V(...p.point).sub(initial.rootP).applyQuaternion(delta).add(settled.conf.rootP).y);
assert(Math.abs(Math.min(...independentHeights) - settled.groundDiagnostics.after.minimum.heightM) < 1e-12);
for (const key of Object.keys(initial.local)) assert(settled.conf.local[key].equals(initial.local[key]));
assert(planeFitter.fit(settled.conf, { rigidTilt: { maxPitchDeg: 2, maxRollDeg: 0 } }).conf.rootQ.angleTo(settled.conf.rootQ) < 1e-7);
const tinyBound = optimizeGroundTilt([
  { point: [0, 0, -1], kind: 'marker', region: 'ладони', side: 'L' },
  { point: [0, .02, 1], kind: 'marker', region: 'ладони', side: 'R' },
], [{ regions: ['ладони'], side: 'L' }, { regions: ['ладони'], side: 'R' }], THREE,
{ maxPitchDeg: .03, maxRollDeg: 0, targetGapM: 0 });
assert.equal(tinyBound.pitchDeg, .03); assert(tinyBound.boundReached);
assert.throws(() => planeFitter.fit({ ...initial, id: 'misspelled-source-id' }));
console.log('Practice ground fit: analytic floor/plane/balance, non-origin rigid fit, unchanged skeleton, missing supports, idempotence, delta decoding, invalid inputs PASS.');
