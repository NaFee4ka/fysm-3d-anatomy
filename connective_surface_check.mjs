// Geometry checks: node connective_surface_check.mjs (no browser or regeneration).
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createConnectiveSurfaces } from './connective_surface_render.js';

const A = createAnatomy(THREE);
const fitted = process.argv.includes('--fitted');
const opts = fitted ? { capsuleRims: JSON.parse(readFileSync('joints/capsule-rims.json', 'utf8')).capsuleRims } : {};
const R = createConnectiveSurfaces(A, THREE, opts);
const data = JSON.parse(readFileSync('poses.json', 'utf8'));
const paths = A.strandInstances().filter(s => s.kind === 'lig' || s.kind === 'cap');
for (const inst of paths) assert(R.covered.has(`${inst.kind}_${inst.unit}_${inst.side}`), inst.id);
assert.equal(R.parts.filter(p => p.kind === 'cap').length, 12);
for (const cap of R.parts.filter(p => p.kind === 'cap')) {
  assert.equal(cap.closed, cap.unit !== 'kn', `${cap.unit}: retain the inherited knee sector`);
  assert.equal(cap.displayFit, fitted ? 'bone-rim' : 'source-fallback', `${cap.mesh.name}: display source status`);
}
const invalid = createConnectiveSurfaces(A, THREE, { capsuleRims: { hip_L: { prox: { seg: 'absent', positions: [] } } } });
assert.equal(invalid.parts.find(p => p.mesh.name === 'cap_hip_L').displayFit, 'source-fallback');
invalid.dispose();
// The three centerline ligaments must not produce stacked L/R surfaces.
for (const unit of ['nuchal', 'all', 'ssl']) {
  assert.equal(R.parts.filter(p => p.kind === 'lig' && p.unit === unit).length, 1, unit);
}

// A displayed ligament must be a solid envelope, including its insertion ends.
// Independent named bundles remain separate solids; only the three explicitly
// fan-shaped structures gain a continuous web between the source fibres.
let closedLigamentSolids = 0, fanParts = 0;
for (const part of R.parts.filter(p => p.kind === 'lig')) {
  const index = part.mesh.geometry.index.array, vertices = part.mesh.geometry.attributes.position.count;
  const edges = new Map(), adjacency = Array.from({ length: vertices }, () => []);
  for (let i = 0; i < index.length; i += 3) for (let j = 0; j < 3; j++) {
    const a = index[i + j], b = index[i + (j + 1) % 3];
    assert.notEqual(a, b, `${part.mesh.name}: repeated triangle vertex`);
    const key = a < b ? `${a}/${b}` : `${b}/${a}`;
    const edge = edges.get(key) || { count: 0, winding: 0 };
    edge.count++; edge.winding += a < b ? 1 : -1; edges.set(key, edge);
    adjacency[a].push(b); adjacency[b].push(a);
  }
  for (const edge of edges.values()) {
    assert.equal(edge.count, 2, `${part.mesh.name}: open/nonmanifold ligament edge`);
    assert.equal(edge.winding, 0, `${part.mesh.name}: inconsistent face winding`);
  }
  const visited = new Set(); let components = 0;
  for (let vertex = 0; vertex < vertices; vertex++) if (!visited.has(vertex)) {
    components++; const queue = [vertex]; visited.add(vertex);
    while (queue.length) for (const next of adjacency[queue.pop()]) if (!visited.has(next)) {
      visited.add(next); queue.push(next);
    }
  }
  const fan = ['iliofem', 'deltoidLig', 'plantar'].includes(part.unit);
  assert.equal(components, fan ? 1 : part.insts.length, `${part.mesh.name}: unexpected tissue connectivity`);
  assert.equal(!!part.fanMid, fan, `${part.mesh.name}: wrong fan morphology`);
  closedLigamentSolids += components; if (fan) fanParts++;
}
assert.equal(fanParts, 6);

let maxAttachmentError = 0, maxSeamError = 0, maxAddedRadialDeficit = 0, maxRadialRoundingRatio = 0, crossingChecks = 0;
const allStates = [];
const inverseFrames = new Map();
const ray = new THREE.Ray(), direction = new THREE.Vector3(), hit = new THREE.Vector3();
function edgeCrossesTriangle(a, b, triangle) {
  direction.subVectors(b, a); const length = direction.length();
  if (!length) return false;
  ray.origin.copy(a); ray.direction.copy(direction.divideScalar(length));
  if (!ray.intersectTriangle(...triangle.points, false, hit)) return false;
  const distance = hit.distanceTo(a);
  return distance > 1e-7 && distance < length - 1e-7;
}
function checkLocalCrossings(part, label) {
  const g = part.mesh.geometry, index = g.index.array, pos = g.attributes.position, triangles = [];
  for (let i = 0; i < index.length; i += 3) {
    const ids = [index[i], index[i + 1], index[i + 2]];
    const points = ids.map(id => new THREE.Vector3().fromBufferAttribute(pos, id));
    triangles.push({ ids, points, box: new THREE.Box3().setFromPoints(points) });
  }
  for (let i = 0; i < triangles.length; i++) for (let j = i + 1; j < triangles.length; j++) {
    const a = triangles[i], b = triangles[j];
    if (a.ids.some(id => b.ids.includes(id)) || !a.box.intersectsBox(b.box)) continue;
    for (let k = 0; k < 3; k++) assert(
      !edgeCrossesTriangle(a.points[k], a.points[(k + 1) % 3], b)
      && !edgeCrossesTriangle(b.points[k], b.points[(k + 1) % 3], a),
      `${label}/${part.mesh.name}: nonadjacent surface triangles intersect`);
  }
  crossingChecks++;
}
function normalizedRadius(p, part, W) {
  const wrap = part.wrap, w = W[wrap.seg];
  let inverse = inverseFrames.get(wrap.seg);
  if (!inverse) {
    inverse = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(w.q)).invert();
    inverseFrames.set(wrap.seg, inverse);
  }
  return p.clone().sub(w.p).applyMatrix3(inverse)
    .add(A.S[wrap.seg].J).sub(wrap.c).divide(wrap.r).length();
}
function radialFloat32Bound(p, part) {
  // A Float32 vertex has at most half an ulp of error per world component.
  // Propagate that component box through the exact inverse wrapping transform;
  // the norm of the resulting box corner bounds a change in normalized radius.
  const epsilon = 2 ** -24 / (1 - 2 ** -24), subnormal = 2 ** -149;
  const x = Math.abs(p.x) * epsilon + subnormal, y = Math.abs(p.y) * epsilon + subnormal, z = Math.abs(p.z) * epsilon + subnormal;
  const m = inverseFrames.get(part.wrap.seg).elements, r = part.wrap.r;
  return Math.hypot(
    (Math.abs(m[0]) * x + Math.abs(m[3]) * y + Math.abs(m[6]) * z) / r.x,
    (Math.abs(m[1]) * x + Math.abs(m[4]) * y + Math.abs(m[7]) * z) / r.y,
    (Math.abs(m[2]) * x + Math.abs(m[5]) * y + Math.abs(m[8]) * z) / r.z,
  ) + 1e-10;
}
function checkGeometry(label, W) {
  allStates.push(label);
  inverseFrames.clear();
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
    const actualLengths = part.insts.map(inst => A.pathLength(A.strandPath(W, inst)));
    for (let f = 0; f < part.insts.length; f++)
      assert(Math.abs(part.lengths[f] - actualLengths[f]) < 1e-12, `${label}/${part.mesh.name}: numerical path length changed`);
    const expectedStrain = Math.max(...actualLengths.map((length, f) => length / part.restLengths[f] - 1));
    assert(Math.abs(part.strain - expectedStrain) < 1e-12, `${label}/${part.mesh.name}: display fit altered numerical length change`);
    assert(g.boundingSphere.radius > 0 && Number.isFinite(g.boundingSphere.radius), `${label}: valid bounds`);
    // These are the actual locations of previously observed folds: an angular
    // source route at the shoulder and a V-shaped ankle attachment rim. Verify
    // the envelope itself, since manifold topology alone misses intersections.
    if ((label === 'stand' || label.endsWith('/1')) && (part.fanMid || part.unit === 'coracohum'))
      checkLocalCrossings(part, label);
    const pos = g.attributes.position, rows = part.paths[0].length;
    for (const { inst, end, vertex } of part.attachmentVertices || []) {
      const error = new THREE.Vector3().fromBufferAttribute(pos, vertex).distanceTo(part.paths[inst][end]);
      maxAttachmentError = Math.max(maxAttachmentError, error);
      assert(error < 5e-7, `${label}/${part.mesh.name}: ligament attachment moved`);
    }
    if (part.kind !== 'cap') continue;
    assert(part.displayPaths.flat().every(v => [v.x, v.y, v.z].every(Number.isFinite)), `${label}: finite display paths`);
    if (fitted) {
      assert.equal(part.displayColumnPaths.length, part.columns, `${label}: every display column has a routed path`);
      assert(part.displayColumnPaths.flat().every(v => [v.x, v.y, v.z].every(Number.isFinite)), `${label}: finite dense display paths`);
      for (const end of ['prox', 'dist']) {
        const row = end === 'prox' ? 0 : rows - 1, rim = part.displayRims[end], w = W[rim.seg];
        assert.equal(rim.densePositions.length, part.columns);
        for (let j = 0; j < part.columns; j++) {
          const expected = rim.densePositions[j].clone().sub(A.S[rim.seg].J).applyQuaternion(w.q).add(w.p);
          const actual = new THREE.Vector3().fromBufferAttribute(pos, row * part.columns + j);
          const error = actual.distanceTo(expected);
          maxAttachmentError = Math.max(maxAttachmentError, error);
          assert(error < 5e-7, `${label}/${part.mesh.name}: dense rim sample left its owning bone`);
        }
      }
    }
    for (let f = 0; f < part.insts.length; f++) for (let row = 0; row < rows; row++) {
      const actual = new THREE.Vector3().fromBufferAttribute(pos, row * part.columns + f * 4);
      const error = actual.distanceTo(part.displayPaths[f][row]);
      maxAttachmentError = Math.max(maxAttachmentError, error);
      assert(error < 5e-7, `${label}/${part.mesh.name}: display capsule path knot moved`);
      if (part.displayRims && (row === 0 || row === rows - 1)) {
        const rim = row === 0 ? part.displayRims.prox : part.displayRims.dist;
        const w = W[rim.seg], expected = rim.positions[f].clone().sub(A.S[rim.seg].J).applyQuaternion(w.q).add(w.p);
        assert(expected.distanceTo(part.displayPaths[f][row]) < 1e-10, `${label}/${part.mesh.name}: display rim left its owning bone`);
      }
    }
    // The interpolation must not introduce a new shortcut through the wrapping
    // ellipsoid. Source routes themselves are simplified and are not certified
    // collision-free against the segmented bone geometry.
    for (let row = 0; row < rows; row++) for (let j = 0; j < part.columns; j++) {
      const x = j / 4, f = Math.min(part.intervals - 1, Math.floor(x)), t = x - f;
      const next = part.closed ? (f + 1) % part.paths.length : f + 1;
      const expected = part.displayColumnPaths ? normalizedRadius(part.displayColumnPaths[j][row], part, W)
        : normalizedRadius(part.displayPaths[f][row], part, W) * (1 - t)
          + normalizedRadius(part.displayPaths[next][row], part, W) * t;
      const actualPoint = new THREE.Vector3().fromBufferAttribute(pos, row * part.columns + j);
      const actual = normalizedRadius(actualPoint, part, W);
      const deficit = expected - actual;
      maxAddedRadialDeficit = Math.max(maxAddedRadialDeficit, deficit);
      const bound = radialFloat32Bound(actualPoint, part);
      maxRadialRoundingRatio = Math.max(maxRadialRoundingRatio, deficit / bound);
      assert(deficit <= bound, `${label}/${part.mesh.name}: displayed membrane moved inward beyond Float32 rounding bound`);
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
const cycleIndices = [...new Set([...Array.from({ length: Math.ceil(cycle.frames.length / 8) }, (_, i) => i * 8), cycle.frames.length - 1])];
for (const index of cycleIndices) {
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
for (const part of R.parts) if (part.kind === 'lig' && part.widthAxes && part.unit !== 'nuchal') {
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
const relocated = createConnectiveSurfaces(relocatedA, THREE, opts);
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
  mode: fitted ? 'bone-rim' : 'source-fallback',
  parts: R.parts.length, triangles: R.parts.reduce((s, p) => s + p.mesh.geometry.index.count / 3, 0),
  poses: A.POSES.length, cycleFrames: cycleIndices.length, states: allStates.length,
  closedLigamentSolids, fanParts, crossingChecks, radialChecks, maxAttachmentError, maxSeamError,
  maxAddedRadialDeficit, maxRadialRoundingRatio, maxRigidError, maxInitialFrameError,
}));
R.dispose();
