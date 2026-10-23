// Independent analytic fixtures and rendered-bone regression for self-collision.
// No physics solve or browser. This tests the explicitly scoped opposite arms.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gzipSync, gunzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createBodyCollision, triangleIntersect } from './body_collision.mjs';
import { createMuscleSurfaces } from './muscle_surface_render.js';

const A = createAnatomy(THREE), V = (...v) => new THREE.Vector3(...v);
const triangle = values => values.map(x => V(...x));
const flat = triangle([[0, 0, 0], [2, 0, 0], [0, 2, 0]]);
const fixtures = [
  [flat, triangle([[.2, .2, -1], [.2, .2, 1], [1.5, .2, 0]]), true, 'transverse intersection'],
  [flat, triangle([[3, 0, 0], [4, 0, 0], [3, 1, 0]]), false, 'disjoint coplanar'],
  [flat, triangle([[.2, .2, 0], [.4, .2, 0], [.2, .4, 0]]), true, 'coplanar containment'],
  [flat, flat.map(p => p.clone()), true, 'identical coordinates'],
  [flat, triangle([[2, 0, 0], [3, 0, 1], [3, 1, 1]]), true, 'shared vertex'],
  [flat, triangle([[0, 0, 0], [2, 0, 0], [1, -1, 1]]), true, 'shared edge'],
  [flat, flat.map(p => p.clone().add(V(0, 0, .001))), false, 'parallel distinct planes'],
  [flat, triangle([[.5, .5, 0], [.5, .5, 0], [.5, .5, 0]]), true, 'collapsed point inside'],
  [flat, triangle([[3, 3, 0], [3, 3, 0], [3, 3, 0]]), false, 'collapsed point outside'],
  [flat, triangle([[.5, .5, -1], [.5, .5, 1], [.5, .5, 1]]), true, 'degenerate piercing line'],
  [triangle([[0, 0, 0], [2, 0, 0], [1, 0, 0]]), triangle([[1, 0, 0], [3, 0, 0], [2, 0, 0]]), true, 'collinear overlap'],
  [triangle([[0, 0, 0], [2, 0, 0], [1, 0, 0]]), triangle([[0, 1, 0], [2, 1, 0], [1, 1, 0]]), false, 'parallel degenerate lines'],
];
const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(.51, -.83, 1.17));
const move = p => p.clone().applyQuaternion(rotation).add(V(7.3, -2.1, 4.9));
for (const [a, b, expected, name] of fixtures) {
  assert.equal(triangleIntersect(a, b), expected, name);
  assert.equal(triangleIntersect(b, a), expected, name + ' symmetry');
  assert.equal(triangleIntersect(a.slice().reverse(), b), expected, name + ' reverse winding');
  assert.equal(triangleIntersect(a.map(move), b.map(move)), expected, name + ' rigid invariance');
}
assert.throws(() => triangleIntersect([], flat), /Invalid/);
assert.throws(() => triangleIntersect(flat, flat, -1), /Invalid/);

const cube = half => ({
  vertices: [[-half, -half, -half], [half, -half, -half], [half, half, -half], [-half, half, -half],
    [-half, -half, half], [half, -half, half], [half, half, half], [-half, half, half]],
  indices: [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
    3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5],
});
function packageMeshes(meshes) {
  const parts = [], chunks = [];
  for (const [i, mesh] of meshes.entries()) {
    const lo = [0, 1, 2].map(k => Math.min(...mesh.vertices.map(v => v[k])));
    const hi = [0, 1, 2].map(k => Math.max(...mesh.vertices.map(v => v[k])));
    const q = Uint16Array.from(mesh.vertices.flatMap(v => v.map((x, k) => Math.round((x - lo[k]) / (hi[k] - lo[k]) * 65535))));
    for (let k = q.length - 1; k >= 3; k--) q[k] = (q[k] - q[k - 3]) & 65535;
    const ind = Uint16Array.from(mesh.indices);
    for (let k = ind.length - 1; k > 0; k--) ind[k] = (ind[k] - ind[k - 1]) & 65535;
    chunks.push(Buffer.from(q.buffer), Buffer.from(ind.buffer));
    parts.push({ seg: i ? 'hum_R' : 'hum_L', name: 'analytic cube', n: mesh.vertices.length, m: mesh.indices.length / 3, lo, hi });
  }
  return { parts, data: gzipSync(Buffer.concat(chunks)).toString('base64') };
}
const identityWorld = (rightX = 0) => ({
  hum_L: { p: V(0, 0, 0), q: new THREE.Quaternion() },
  hum_R: { p: V(rightX, 0, 0), q: new THREE.Quaternion() },
});
const identical = createBodyCollision(A, THREE, packageMeshes([cube(1), cube(1)]));
assert(identical.inspect(identityWorld()).colliding, 'coincident segment coordinates must be caught');
assert(!identical.inspect(identityWorld(3)).colliding, 'disjoint solid volumes');
const crossing = identical.inspect(identityWorld(1));
assert(crossing.pairs.some(p => p.kind === 'surface'), 'overlapping cube faces');
const nested = createBodyCollision(A, THREE, packageMeshes([cube(2), cube(.5)]));
const containment = nested.inspect(identityWorld());
assert(containment.colliding && containment.pairs[0].kind === 'containment', 'full containment without intersecting surfaces');
const movedWorld = Object.fromEntries(Object.entries(identityWorld()).map(([s, w]) => [s, { p: move(w.p), q: rotation.clone() }]));
assert(nested.inspect(movedWorld).colliding, 'solid containment after a common rigid transform');

const data = JSON.parse(fs.readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url)));
// Losslessly select the two original humerus chunks, without mesh rebuilding.
const raw = gunzipSync(Buffer.from(data.data, 'base64')), humerusParts = [], humerusChunks = [];
let offset = 0;
for (const p of data.parts) {
  const bytes = 6 * (p.n + p.m);
  if (/^hum_[LR]$/.test(p.seg)) { humerusParts.push(p); humerusChunks.push(raw.subarray(offset, offset + bytes)); }
  offset += bytes;
}
const humerus = createBodyCollision(A, THREE, { parts: humerusParts, data: gzipSync(Buffer.concat(humerusChunks)).toString('base64') });
// Literal world transforms of the faulty seg3 at 2.9 s, from main 9f0231a.
// This is an implementation regression on these same frozen bone surfaces.
const oldHumerusWorld = {
  hum_L: { p: V(.16411180476846954, .2520106622430725, .49910249922027966), q: new THREE.Quaternion(.6859281099327338, .6004489885325158, .32264450675833023, .2546844369667296) },
  hum_R: { p: V(-.16411180476846954, .2520106622430725, .49910249922027966), q: new THREE.Quaternion(.6859281099327338, -.6004489885325158, -.32264450675833023, .2546844369667296) },
};
const old = humerus.inspect(oldHumerusWorld);
assert(old.colliding && old.pairs.some(p => p.a === 'hum_L' && p.b === 'hum_R'), 'the actual former arm crossing must fail');

const collider = createBodyCollision(A, THREE, data);
assert.equal(collider.boneParts, 60, 'all original arm/hand/finger bone parts, excluding the clavicles/scapulae, are checked');
const frameWorld = (a, b, t) => {
  const local = {};
  for (const s of A.SEG) local[s.name] = new THREE.Quaternion(...a.local[s.name]).normalize()
    .slerp(new THREE.Quaternion(...b.local[s.name]).normalize(), t);
  const root = new THREE.Quaternion(...a.root).normalize().slerp(new THREE.Quaternion(...b.root).normalize(), t);
  return A.computeWorld(local, root, V(...a.rootP).lerp(V(...b.rootP), t));
};
let states = 0, candidatePairs = 0, triangleTests = 0;
const started = performance.now();
for (let i = 1; i <= 5; i++) {
  const segment = JSON.parse(fs.readFileSync(new URL(`./cycle_keys/seg${i}.json`, import.meta.url)));
  for (let k = 0; k < segment.frames.length - 1; k++) for (let part = 0; part < 20; part++) {
    const report = collider.inspect(frameWorld(segment.frames[k], segment.frames[k + 1], part / 20));
    assert(!report.colliding, JSON.stringify({ segment: i, time: segment.frames[k].t + part / 200, pairs: report.pairs }));
    states++; candidatePairs += report.candidatePairs; triangleTests += report.triangleTests;
  }
  const end = segment.frames.at(-1), report = collider.inspect(frameWorld(end, end, 0));
  assert(!report.colliding, 'segment endpoint ' + i); states++;
}
const muscleUnits = new Set(['deltAnt', 'deltMid', 'deltPost', 'tricLong', 'tricLat', 'tricMed', 'anconeus',
  'bicLong', 'bicShort', 'coracobr', 'brachialis', 'brachiorad', 'pronTeres', 'pronQuad', 'supinator',
  'wristFlex', 'palmaris', 'fingerFlex', 'wristExt', 'apl', 'thenar', 'hypothenar', 'interossei']);
const muscleData = JSON.parse(fs.readFileSync(new URL('./muscles/muscles_bp3d.json', import.meta.url)));
const surfaces = await createMuscleSurfaces(A, THREE, muscleData, {
  instances: A.strandInstances().filter(s => s.kind === 'muscle' && muscleUnits.has(s.unit)),
});
assert(surfaces && surfaces.pickables.length === 2 * muscleUnits.size, 'all scoped muscle surfaces on both sides must load');
const armSegment = JSON.parse(fs.readFileSync(new URL('./cycle_keys/seg3.json', import.meta.url)));
let skinStates = 0, skinVertices = 0, minimumLeftM = Infinity, maximumRightM = -Infinity;
const point = V();
try {
  function checkSkin(W, time) {
    surfaces.update(W);
    const inverse = W.pelvis.q.clone().normalize().invert();
    for (const mesh of surfaces.pickables) for (let k = 0; k < mesh.geometry.attributes.position.count; k++) {
      mesh.getVertexPosition(k, point); point.sub(W.pelvis.p).applyQuaternion(inverse);
      assert(Number.isFinite(point.x), 'finite deformed arm vertices');
      if (mesh.userData.side === 'L') {
        minimumLeftM = Math.min(minimumLeftM, point.x);
        assert(point.x >= -1e-6, `left arm crosses the symmetry plane at ${time}s: ${mesh.userData.unit}/${k}`);
      } else {
        maximumRightM = Math.max(maximumRightM, point.x);
        assert(point.x <= 1e-6, `right arm crosses the symmetry plane at ${time}s: ${mesh.userData.unit}/${k}`);
      }
      skinVertices++;
    }
    skinStates++;
  }
  // This is a conservative separating-plane proof for this symmetric transfer,
  // using every skinned vertex, rather than only joint centres or sparse tips.
  for (let k = 0; k < armSegment.frames.length - 1; k++) for (let part = 0; part < 20; part++)
    checkSkin(frameWorld(armSegment.frames[k], armSegment.frames[k + 1], part / 20), armSegment.frames[k].t + part / 200);
  const end = armSegment.frames.at(-1); checkSkin(frameWorld(end, end, 0), end.t);
} finally { surfaces.dispose(); }
console.log(JSON.stringify({ analyticTriangleChecks: fixtures.length * 4, solidFixtures: 5, formerActualCrossingRejected: true,
  boneParts: collider.boneParts, segmentCount: 5, states, subdivisions: 20, candidatePairs, triangleTests,
  skin: { units: muscleUnits.size, surfaces: surfaces.pickables.length, states: skinStates, deformedVertices: skinVertices,
    minimumLeftM, maximumRightM, separationLowerBoundM: minimumLeftM - maximumRightM },
  elapsedSeconds: +((performance.now() - started) / 1000).toFixed(2), scope: collider.scope }));
