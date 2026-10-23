// Geometry checks: node connective_surface_check.mjs (no browser or regeneration).
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createConnectiveSurfaces } from './connective_surface_render.js';

const A = createAnatomy(THREE), R = createConnectiveSurfaces(A, THREE);
const data = JSON.parse(readFileSync('poses.json', 'utf8'));
const paths = A.strandInstances().filter(s => s.kind === 'lig' || s.kind === 'cap');
for (const inst of paths) assert(R.covered.has(`${inst.kind}_${inst.unit}_${inst.side}`), inst.id);
assert.equal(R.parts.filter(p => p.kind === 'cap').length, 12);
for (const cap of R.parts.filter(p => p.kind === 'cap')) {
  assert.equal(cap.closed, cap.unit !== 'kn', `${cap.unit}: retain the anterior knee opening`);
}
// The three centerline ligaments must not produce stacked L/R surfaces.
for (const unit of ['nuchal', 'all', 'ssl']) {
  assert.equal(R.parts.filter(p => p.kind === 'lig' && p.unit === unit).length, 1, unit);
}

let maxAttachmentError = 0, maxSeamError = 0;
function checkGeometry(label, W) {
  const before = paths.map(s => A.pathLength(A.strandPath(W, s)));
  R.update(W);
  const after = paths.map(s => A.pathLength(A.strandPath(W, s)));
  assert.deepEqual(after, before, `${label}: rendering must not mutate mechanical paths`);
  for (const part of R.parts) {
    const g = part.mesh.geometry;
    for (const attr of ['position', 'normal']) {
      assert(g.attributes[attr].array.every(Number.isFinite), `${label}/${part.mesh.name}: finite ${attr}`);
    }
    assert(Number.isFinite(part.strain), `${label}/${part.mesh.name}: finite length change`);
    assert(g.boundingSphere.radius > 0 && Number.isFinite(g.boundingSphere.radius), `${label}: valid bounds`);
    if (part.kind !== 'cap') continue;
    const pos = g.attributes.position, rows = part.paths[0].length;
    for (let f = 0; f < part.insts.length; f++) for (const row of [0, rows - 1]) {
      const actual = new THREE.Vector3().fromBufferAttribute(pos, row * part.columns + f * 4);
      const error = actual.distanceTo(part.paths[f][row]);
      maxAttachmentError = Math.max(maxAttachmentError, error);
      assert(error < 5e-7, `${label}/${part.mesh.name}: capsule attachment moved`);
    }
    if (part.closed) for (let row = 0; row < rows; row++) {
      const a = new THREE.Vector3().fromBufferAttribute(pos, row * part.columns);
      const b = new THREE.Vector3().fromBufferAttribute(pos, (row + 1) * part.columns - 1);
      maxSeamError = Math.max(maxSeamError, a.distanceTo(b));
      assert(a.distanceTo(b) < 1e-7, `${label}/${part.mesh.name}: capsule seam opened`);
    }
  }
}

checkGeometry('stand', A.computeWorld(A.STAND.local, A.STAND.rootQ, new THREE.Vector3()));
for (const pose of A.POSES) {
  const conf = A.poseFromData(data[pose.id]);
  for (const t of [0.25, 0.5, 1]) {
    const c = A.blendConf(A.STAND, conf, t);
    checkGeometry(`${pose.id}/${t}`, A.placeW(c.local, c.rootQ));
  }
}
const cycle = JSON.parse(readFileSync('cycle_keys/cycle.json', 'utf8'));
for (const index of [0, 42, 76, 106, 140, 155, 184, 205, 235, 261]) {
  const f = cycle.frames[index];
  const local = Object.fromEntries(Object.entries(f.local).map(([s, q]) => [s, new THREE.Quaternion(...q)]));
  checkGeometry(`cycle/${index}`, A.computeWorld(local, new THREE.Quaternion(...f.root), new THREE.Vector3(...f.rootP)));
}

// The sheet must rotate/translate with the whole body; a world-fixed width or
// wrinkle direction would pass individual poses but fail this covariance check.
const W0 = A.computeWorld(A.STAND.local, A.STAND.rootQ, new THREE.Vector3());
R.update(W0);
const original = R.parts.map(p => p.mesh.geometry.attributes.position.array.slice());
// The neutral band width is tangent to its joint's radial plane, not to a plane
// determined by the arbitrary world origin. Check that anatomical orientation
// before testing covariance: a consistently wrong axis would pass covariance.
let radialChecks = 0;
for (const part of R.parts) if (part.kind === 'lig' && part.unit !== 'nuchal') {
  for (let f = 0; f < part.insts.length; f++) {
    const inst = part.insts[f], pts = A.strandPath(W0, inst);
    const tangent = pts.at(-1).clone().sub(pts[0]).normalize();
    const anchor = W0[inst.pts[0].seg];
    const radial = pts[0].clone().lerp(pts.at(-1), .5).sub(anchor.p);
    if (new THREE.Vector3().crossVectors(radial, tangent).lengthSq() < 1e-10) continue;
    const width = part.widthAxes[f].clone().applyQuaternion(anchor.q);
    assert(Math.abs(width.dot(tangent)) < 1e-10, `${part.mesh.name}: width must be orthogonal to path`);
    assert(Math.abs(width.dot(radial)) < 1e-10, `${part.mesh.name}: width must be tangent to joint radial plane`);
    radialChecks++;
  }
}
const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 2, 3).normalize(), 0.73);
const shift = new THREE.Vector3(0.31, -0.22, 0.48);

// Also construct the tissue renderer in a different initial world frame. This
// catches rest-world versus model-coordinate mistakes during binding; merely
// rotating an already-created renderer would not expose them.
const relocatedA = {
  ...A, STAND: { local: A.STAND.local, rootQ: q },
  computeWorld: (...args) => {
    const W = A.computeWorld(...args);
    for (const w of Object.values(W)) w.p.add(shift);
    return W;
  },
};
const relocated = createConnectiveSurfaces(relocatedA, THREE);
let maxInitialFrameError = 0;
assert.equal(relocated.parts.length, R.parts.length);
for (let p = 0; p < R.parts.length; p++) {
  const originalPart = R.parts[p], movedPart = relocated.parts[p];
  assert.equal(movedPart.mesh.name, originalPart.mesh.name);
  if (originalPart.widthAxes) for (let f = 0; f < originalPart.widthAxes.length; f++) {
    assert(originalPart.widthAxes[f].distanceTo(movedPart.widthAxes[f]) < 1e-10,
      `${originalPart.mesh.name}: initial world placement changed local band axis`);
  }
  const current = movedPart.mesh.geometry.attributes.position.array;
  for (let i = 0; i < current.length; i += 3) {
    const expected = new THREE.Vector3().fromArray(original[p], i).applyQuaternion(q).add(shift);
    const error = expected.distanceTo(new THREE.Vector3().fromArray(current, i));
    maxInitialFrameError = Math.max(maxInitialFrameError, error);
    assert(error < 8e-7, `${originalPart.mesh.name}: initial world placement changed tissue shape`);
  }
}
relocated.dispose();

R.update(A.computeWorld(A.STAND.local, q, shift));
let maxRigidError = 0;
for (let p = 0; p < R.parts.length; p++) {
  const current = R.parts[p].mesh.geometry.attributes.position.array;
  for (let i = 0; i < current.length; i += 3) {
    const expected = new THREE.Vector3().fromArray(original[p], i).applyQuaternion(q).add(shift);
    const error = expected.distanceTo(new THREE.Vector3().fromArray(current, i));
    maxRigidError = Math.max(maxRigidError, error);
    assert(error < 8e-7, `rigid transform changed local tissue shape: ${R.parts[p].mesh.name}`);
  }
}

console.log(JSON.stringify({
  parts: R.parts.length, triangles: R.parts.reduce((s, p) => s + p.mesh.geometry.index.count / 3, 0),
  poses: A.POSES.length, cycleFrames: 10, radialChecks, maxAttachmentError, maxSeamError, maxRigidError, maxInitialFrameError,
}));
R.dispose();
