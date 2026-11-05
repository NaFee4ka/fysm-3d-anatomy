// Genuine source identity, frozen-bone registration and display motion tests.
// No pressure, cartilage contact, clinical accuracy or disk mechanics claimed.
// node spine_surfaces_test.mjs [--require-transitions]
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, readFileSync } from 'node:fs';
import { gunzipSync, gzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createSpineSurfaces } from './spine_surface_render.js';
import { createTransitions } from './transition_runtime.mjs';

const A = createAnatomy(THREE), root = A.SEG.find(seg => !seg.parent);
const load = path => JSON.parse(readFileSync(new URL(path, import.meta.url), 'utf8'));
const data = load('./joints/spine-discs.json'), report = load('./joints/spine-discs-report.json');
const original = JSON.stringify(data), boneBytes = readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url));
const hash = path => createHash('sha256').update(readFileSync(new URL(path, import.meta.url))).digest('hex');
assert.equal(report.dataSha256, hash('./joints/spine-discs.json'));
for (const [path, sha] of Object.entries(data.sourceHashes)) assert.equal(hash('./' + path), sha, 'stale disk dependency: ' + path);
assert.deepEqual(report.sourceHashes, data.sourceHashes);
const sourceAudit = load('./joints/source-audit.json').intervertebralDisks.parts;
assert.equal(data.parts.length, 23); assert.equal(data.parts.filter(p => p.levelAssignment === 'inferred').length, 1);
for (const source of sourceAudit) {
  const part = data.parts.find(p => p.level === source.level);
  assert(part, 'missing real source disk: ' + source.level);
  assert.deepEqual(part.sourceIds, [source.elementId]);
  assert.equal(part.levelAssignment, source.levelAssignment.includes('inferred') ? 'inferred' : 'metadata');
  assert(part.registration.minimumTriangleNormalCosine > 0, 'registered source triangle inverted');
  assert(part.registration.maxQuantizationErrorM < 2e-6);
}
assert(!data.parts.some(p => /C1|S2/.test(p.level)), 'source pack invented an extra disk level');
const raw = gunzipSync(Buffer.from(data.data, 'base64'));
assert.equal(raw.length, report.rawBytes);
let offset = 0;
const decoded = new Map();
for (const meta of data.parts) {
  const positions = new Float64Array(meta.n * 3), weights = new Float64Array(meta.n), indices = new Uint16Array(meta.m * 3), acc = [0, 0, 0];
  for (let i = 0; i < positions.length; i++, offset += 2) {
    const axis = i % 3; acc[axis] = (acc[axis] + raw.readUInt16LE(offset)) & 65535;
    positions[i] = meta.lo[axis] + (meta.hi[axis] - meta.lo[axis]) * acc[axis] / 65535;
  }
  let index = 0;
  for (let i = 0; i < indices.length; i++, offset += 2) { index = (index + raw.readUInt16LE(offset)) & 65535; indices[i] = index; }
  for (let i = 0; i < weights.length; i++, offset += 2) weights[i] = raw.readUInt16LE(offset) / 65535;
  assert.equal(Math.min(...weights), 0); assert.equal(Math.max(...weights), 1);
  decoded.set(meta.id, { positions, indices, weights });
}
assert.equal(offset, raw.length);
const surfaces = await createSpineSurfaces(THREE, A, data);
assert.equal(surfaces.parts.length, 23); assert.equal(surfaces.covered.size, 23);
assert.deepEqual(surfaces.excludedLegacy, new Set(['neckB|Межпозвонковый диск C2–C1']));
const oldDisks = new Set(A.BONES.filter(b => b.mat === 'disc').map(b => b.seg + '|' + b.name));
assert.deepEqual(new Set([...surfaces.covered, ...surfaces.excludedLegacy]), oldDisks, 'schematic replacement must be exact');
assert.equal(new Set(surfaces.parts.map(p => p.mat)).size, 23, 'each source disk owns its mutable material');
const stats = { endpoints: 0, transitionStates: 0, vertices: 0, raycasts: 0, maxBoneBindingErrorM: 0, maxCoincidentSeamGapM: 0, minFloorM: Infinity, rejectedFrames: 0 };
const snapshots = new Map(surfaces.parts.map(p => [p.id, { rest: p.restPositions.slice(), weights: p.weights.slice(), indices: p.mesh.geometry.index.array.slice() }]));
const p0 = new THREE.Vector3(), p1 = new THREE.Vector3(), actual = new THREE.Vector3();
function worldFrom(record) {
  const local = Object.fromEntries(A.SEG.map(seg => {
    const q = record.local[seg.name]; return [seg.name, q?.isQuaternion ? q : new THREE.Quaternion(...(q || [0, 0, 0, 1]))];
  }));
  const q = record.rootQ || record.root;
  return A.computeWorld(local, q.isQuaternion ? q : new THREE.Quaternion(...q), record.rootP?.isVector3 ? record.rootP : new THREE.Vector3(...record.rootP));
}
function check(name, W, full = true) {
  const before = JSON.stringify(W); surfaces.update(W); assert.equal(JSON.stringify(W), before, name + ': renderer altered the ready skeleton');
  for (const part of surfaces.parts) {
    const g = part.mesh.geometry, attr = g.attributes.position, normals = g.attributes.normal, source = decoded.get(part.id), snapshot = snapshots.get(part.id);
    const bounds = surfaces.getBounds(part.id); assert(bounds && !bounds.isEmpty());
    assert.deepEqual(g.index.array, source.indices, 'source triangles changed');
    const lower = new THREE.Matrix4().compose(W[part.inferior.seg].p, W[part.inferior.seg].q, new THREE.Vector3(1, 1, 1));
    const upper = new THREE.Matrix4().compose(W[part.superior.seg].p, W[part.superior.seg].q, new THREE.Vector3(1, 1, 1));
    const coincident = new Map();
    const count = full ? part.n : 3;
    for (let sample = 0; sample < count; sample++) {
      const i = full ? sample : Math.floor(sample * (part.n - 1) / 2);
      p0.fromArray(source.positions, 3 * i).sub(A.S[part.inferior.seg].J).applyMatrix4(lower);
      p1.fromArray(source.positions, 3 * i).sub(A.S[part.superior.seg].J).applyMatrix4(upper);
      p0.multiplyScalar(1 - source.weights[i]).addScaledVector(p1, source.weights[i]);
      actual.fromBufferAttribute(attr, i);
      const error = p0.distanceTo(actual); stats.maxBoneBindingErrorM = Math.max(stats.maxBoneBindingErrorM, error); stats.vertices++;
      assert(error < 2e-7, name + '/' + part.level + ': surface detached from its two bone anchors');
      assert(bounds.clone().expandByScalar(1e-7).containsPoint(actual), 'culling/focus bounds omit a disk vertex');
      stats.minFloorM = Math.min(stats.minFloorM, actual.y);
      assert(actual.y >= -2e-6, name + '/' + part.level + ': new disk penetrates the displayed floor');
      const length = p1.fromBufferAttribute(normals, i).length();
      assert(Number.isFinite(length) && Math.abs(length - 1) < 1e-5, name + '/' + part.level + ': normal collapsed during motion');
      const key = source.positions.slice(3 * i, 3 * i + 3).join(',');
      if (coincident.has(key)) {
        const gap = actual.distanceTo(coincident.get(key)); stats.maxCoincidentSeamGapM = Math.max(stats.maxCoincidentSeamGapM, gap);
        assert(gap < 1e-10, 'retained OBJ seam opened');
      } else coincident.set(key, actual.clone());
    }
    // Raycast a large actual triangle, so the CPU picking surface must match
    // the very buffer sent to the GPU rather than an undeformed atlas proxy.
    let best = -1, area = -1, triangle;
    for (let i = 0; i < g.index.count; i += 3) {
      const t = [0, 1, 2].map(d => new THREE.Vector3().fromBufferAttribute(attr, g.index.getX(i + d)));
      const a = t[1].clone().sub(t[0]).cross(t[2].clone().sub(t[0])).lengthSq();
      if (a > area) { area = a; best = i / 3; triangle = t; }
    }
    assert(area > 0);
    const center = triangle[0].clone().add(triangle[1]).add(triangle[2]).divideScalar(3);
    const normal = triangle[1].clone().sub(triangle[0]).cross(triangle[2].clone().sub(triangle[0])).normalize();
    const ray = new THREE.Raycaster(center.clone().addScaledVector(normal, .002), normal.clone().negate(), 0, .004);
    const hit = ray.intersectObject(part.mesh).find(h => h.faceIndex === best);
    assert(hit && hit.point.distanceTo(center) < 1e-7, name + '/' + part.level + ': posed disk could not be picked');
    assert.equal(hit.object.userData.kind, 'spinalDisc'); assert.equal(hit.object.userData.id, part.id); stats.raycasts++;
    assert.deepEqual(part.restPositions, snapshot.rest); assert.deepEqual(part.weights, snapshot.weights); assert.deepEqual(g.index.array, snapshot.indices);
  }
}
try {
  const rest = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.J.clone()); check('stand', rest);
  for (const [id, record] of Object.entries(load('./poses.json'))) if (A.POSES.some(p => p.id === id)) {
    check('main/' + id, A.poseAt(A.poseFromData(record), 1).W); stats.endpoints++;
  }
  for (const [id, record] of Object.entries(load('./practice/poses.json').poses)) { check('practice/' + id, worldFrom(record)); stats.endpoints++; }
  assert.equal(stats.endpoints, 30);
  const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(.3, -.2, .4)), shift = new THREE.Vector3(.1, 1, -.2);
  const initial = surfaces.parts.map(part => part.mesh.geometry.attributes.position.array.slice());
  surfaces.update(rest);
  const initialRest = surfaces.parts.map(part => part.mesh.geometry.attributes.position.array.slice());
  const rigidW = Object.fromEntries(Object.entries(rest).map(([id, w]) => [id, { p: w.p.clone().applyQuaternion(rotation).add(shift), q: rotation.clone().multiply(w.q) }]));
  surfaces.update(rigidW);
  surfaces.parts.forEach((part, index) => {
    for (let i = 0; i < part.n; i++) {
      p0.fromArray(initialRest[index], 3 * i).applyQuaternion(rotation).add(shift);
      p1.fromBufferAttribute(part.mesh.geometry.attributes.position, i);
      assert(p0.distanceTo(p1) < 3e-7, 'two-anchor skinning is not rigid-frame covariant');
    }
  });
  const lastDriver = surfaces.parts.at(-1).superior.seg;
  for (const damage of [W => { delete W[lastDriver]; }, W => { W[lastDriver].p.y = NaN; }, W => { W[lastDriver].q.x = Infinity; }, W => { W[lastDriver].q.set(0, 0, 0, 0); }]) {
    const W = Object.fromEntries(Object.entries(rest).map(([id, w]) => [id, { p: w.p.clone(), q: w.q.clone() }]));
    const previous = surfaces.parts.map(p => p.mesh.geometry.attributes.position.array.slice()); damage(W);
    assert.throws(() => surfaces.update(W), /Invalid spine disk transform/);
    surfaces.parts.forEach((p, i) => assert.deepEqual(p.mesh.geometry.attributes.position.array, previous[i])); stats.rejectedFrames++;
  }
  check('recovery', rest);
  const trackPath = new URL('./transitions/data.json', import.meta.url);
  if (existsSync(trackPath)) {
    const packet = JSON.parse(readFileSync(trackPath, 'utf8'));
    const runtime = createTransitions(THREE, A, packet);
    assert(runtime.valid, 'a verified transition packet must be runtime-valid');
    const tracks = packet.tracks;
    for (const [id, track] of Object.entries(tracks)) {
      // Every knot and every interval midpoint use the actual page sampler,
      // including the nonlinear floor envelope between saved configurations.
      for (let i = 0; i < track.frames.length; i++) {
        const knot = runtime.sampleTrack(id, track.frames[i].t);
        check(id + '/' + i, A.computeWorld(knot.local, knot.rootQ, knot.rootP), false); stats.transitionStates++;
      if (i + 1 < track.frames.length) {
          const a = track.frames[i], b = track.frames[i + 1];
          const mid = runtime.sampleTrack(id, (a.t + b.t) / 2);
          check(id + '/' + i + '.5', A.computeWorld(mid.local, mid.rootQ, mid.rootP), false); stats.transitionStates++;
        }
      }
    }
  } else assert(!process.argv.includes('--require-transitions'), 'transition package is required');
  surfaces.setVisibility(false); assert.equal(surfaces.getBounds(), null); assert(surfaces.getBounds(null, false));
  surfaces.setVisibility(true); surfaces.group.visible = false; assert.equal(surfaces.getBounds(), null); surfaces.group.visible = true;
  assert.equal(JSON.stringify(data), original); assert.deepEqual(readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url)), boneBytes);
  console.log(JSON.stringify({ ...stats, parts: 23, sourceVertices: report.vertices, sourceTriangles: report.triangles, status: 'PASS', scope: 'source identity and geometric display; no disk-pressure/contact/volume-law validation' }, null, 2));
} finally { surfaces.dispose(); }
const duplicate = structuredClone(data); duplicate.parts[1].id = duplicate.parts[0].id;
await assert.rejects(createSpineSurfaces(THREE, A, duplicate), /manifest/);
const truncated = structuredClone(data); truncated.data = gzipSync(raw.subarray(0, -2)).toString('base64');
await assert.rejects(createSpineSurfaces(THREE, A, truncated), /payload length/);
const badIndex = structuredClone(data), badRaw = Buffer.from(raw); badRaw.writeUInt16LE(data.parts[0].n, data.parts[0].n * 6);
badIndex.data = gzipSync(badRaw).toString('base64');
await assert.rejects(createSpineSurfaces(THREE, A, badIndex), /index is outside/);
