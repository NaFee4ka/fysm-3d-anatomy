// Whole-page joint display motion and failure isolation. This is a renderer
// check, not measured cartilage shape, contact pressure or physiological ROM.
// node body_joint_motion_test.mjs [--require-transitions]
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createJointSurfaces } from './joint_surface_render.js';
import { createTransitions } from './transition_runtime.mjs';

const A = createAnatomy(THREE), root = A.SEG.find(seg => !seg.parent);
const load = path => JSON.parse(readFileSync(new URL(path, import.meta.url), 'utf8'));
const data = load('./joints/joints.json'), bonesBefore = readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url));
const manifestBefore = JSON.stringify(data), surfaces = await createJointSurfaces(A, THREE, data);
const raw = gunzipSync(Buffer.from(data.data, 'base64'));
const decoded = new Map(); let offset = 0;
// Independent scalar unpacker; the source uses rest-world metres, unlike the
// finished bone atlas, whose coordinates are already local to their segment.
for (const meta of data.parts) {
  const values = new Float64Array(meta.n * 3), acc = [0, 0, 0];
  for (let i = 0; i < values.length; i++, offset += 2) {
    const axis = i % 3;
    acc[axis] = (acc[axis] + raw.readUInt16LE(offset)) & 65535;
    values[i] = meta.lo[axis] + (meta.hi[axis] - meta.lo[axis]) * acc[axis] / 65535;
  }
  offset += meta.m * 6; decoded.set(meta.id, values);
}
assert.equal(offset, raw.length);
const stats = { endpoints: 0, transitionStates: 0, vertices: 0, raycasts: 0, rejectedFrames: 0, maxAttachmentErrorM: 0 };
const v = new THREE.Vector3(), expected = new THREE.Vector3();
const snapshots = new Map(surfaces.parts.map(part => {
  const geometry = part.mesh.geometry;
  return [part.id, { positions: geometry.attributes.position.array.slice(), normals: geometry.attributes.normal.array.slice(), indices: geometry.index.array.slice(), largestTriangle: largestTriangle(geometry) }];
}));
function largestTriangle(g) {
  const p = g.attributes.position, a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  let best = 0, area = -1;
  for (let i = 0; i < g.index.count; i += 3) {
    a.fromBufferAttribute(p, g.index.getX(i)); b.fromBufferAttribute(p, g.index.getX(i + 1)); c.fromBufferAttribute(p, g.index.getX(i + 2));
    const current = b.sub(a).cross(c.sub(a)).lengthSq();
    if (current > area) { area = current; best = i / 3; }
  }
  assert(area > 0); return best;
}
function worldFromRecord(record) {
  const local = Object.fromEntries(A.SEG.map(seg => {
    const q = record.local[seg.name] || [0, 0, 0, 1];
    return [seg.name, q.isQuaternion ? q.clone() : new THREE.Quaternion(...q)];
  }));
  const q = record.rootQ || record.root;
  const rq = q?.isQuaternion ? q : new THREE.Quaternion(...q);
  const rp = record.rootP?.isVector3 ? record.rootP : record.rootP ? new THREE.Vector3(...record.rootP) : root.J.clone();
  return A.computeWorld(local, rq, rp);
}
function check(name, W, fullVertices = true) {
  const skeletonBefore = JSON.stringify(W);
  surfaces.update(W);
  assert.equal(JSON.stringify(W), skeletonBefore, name + ': renderer mutated the skeleton');
  const jointBounds = new Map(A.JOINTS.map(j => [j.id, surfaces.getBounds(j.id)]));
  for (const part of surfaces.parts) {
    const mesh = part.mesh, g = mesh.geometry, snapshot = snapshots.get(part.id), source = decoded.get(part.id);
    assert(mesh.matrixWorld.elements.every(Number.isFinite), name + '/' + part.id + ': invalid posed matrix');
    const bounds = jointBounds.get(part.joint);
    assert(bounds && !bounds.isEmpty());
    const normalMatrix = new THREE.Matrix3().getNormalMatrix(mesh.matrixWorld);
    const samples = fullVertices ? g.attributes.position.count : 3;
    for (let i = 0; i < samples; i++) {
      const k = fullVertices ? i : Math.floor(i * (g.attributes.position.count - 1) / 2);
      v.fromBufferAttribute(g.attributes.position, k).applyMatrix4(mesh.matrixWorld);
      expected.set(source[k * 3], source[k * 3 + 1], source[k * 3 + 2]);
      expected.sub(A.S[part.seg].J).applyQuaternion(W[part.seg].q).add(W[part.seg].p);
      const error = v.distanceTo(expected);
      stats.maxAttachmentErrorM = Math.max(stats.maxAttachmentErrorM, error); stats.vertices++;
      assert(error < 3e-7, name + '/' + part.id + ': surface detached from bone');
      assert(bounds.clone().expandByScalar(1e-7).containsPoint(v), name + '/' + part.id + ': bounds omit the posed surface');
      v.fromBufferAttribute(g.attributes.normal, k).applyMatrix3(normalMatrix).normalize();
      assert(v.toArray().every(Number.isFinite) && Math.abs(v.length() - 1) < 1e-12, name + '/' + part.id + ': invalid posed normal');
    }
    const i = snapshot.largestTriangle * 3;
    const triangle = [0, 1, 2].map(d => new THREE.Vector3().fromBufferAttribute(g.attributes.position, g.index.getX(i + d)).applyMatrix4(mesh.matrixWorld));
    const center = triangle[0].clone().add(triangle[1]).add(triangle[2]).divideScalar(3);
    const normal = triangle[1].clone().sub(triangle[0]).cross(triangle[2].clone().sub(triangle[0])).normalize();
    const ray = new THREE.Raycaster(center.clone().addScaledVector(normal, .002), normal.clone().negate(), 0, .004);
    const hit = ray.intersectObject(mesh).find(hit => hit.faceIndex === snapshot.largestTriangle);
    assert(hit && hit.point.distanceTo(center) < 1e-7, name + '/' + part.id + ': visible surface cannot be selected at its posed location');
    assert.equal(hit.object.userData.joint, part.joint); stats.raycasts++;
    assert.deepEqual(g.attributes.position.array, snapshot.positions);
    assert.deepEqual(g.attributes.normal.array, snapshot.normals);
    assert.deepEqual(g.index.array, snapshot.indices);
  }
}
try {
  for (const [id, record] of Object.entries(load('./poses.json'))) {
    if (id === 'stand' || !A.POSES.some(pose => pose.id === id)) continue;
    const pose = A.poseFromData(record);
    check('main/' + id, A.poseAt(pose, 1).W); stats.endpoints++;
  }
  const practice = load('./practice/poses.json');
  for (const [id, record] of Object.entries(practice.poses)) {
    check('practice/' + id, worldFromRecord(record)); stats.endpoints++;
  }
  assert.equal(stats.endpoints, 30, 'all six main and all 24 FYSM endpoints are required');
  const rest = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.J.clone());
  check('stand', rest);
  // Every driven joint must reject a damaged later segment before moving ANY
  // earlier surface. A missing shank_R used to move 28 of the 42 parts first.
  const lastSegment = surfaces.parts.at(-1).seg;
  const copyWorld = () => Object.fromEntries(Object.entries(rest).map(([id, w]) => [id, { p: w.p.clone().add(new THREE.Vector3(.12, .3, -.2)), q: w.q.clone() }]));
  const damage = [
    W => { delete W[lastSegment]; },
    W => { W[lastSegment].p.x = NaN; },
    W => { W[lastSegment].p.y = Infinity; },
    W => { W[lastSegment].q.z = NaN; },
    W => { W[lastSegment].q.w = Infinity; },
    W => { W[lastSegment].q.set(0, 0, 0, 0); },
  ];
  for (const mutate of damage) {
    const before = surfaces.parts.map(part => ({ local: part.mesh.matrix.toArray(), world: part.mesh.matrixWorld.toArray() }));
    const W = copyWorld(); mutate(W);
    assert.throws(() => surfaces.update(W), /Invalid joint surface transform/);
    surfaces.parts.forEach((part, i) => {
      assert.deepEqual(part.mesh.matrix.toArray(), before[i].local);
      assert.deepEqual(part.mesh.matrixWorld.toArray(), before[i].world);
    }); stats.rejectedFrames++;
  }
  assert.throws(() => surfaces.update(null), /Invalid joint surface transform/); stats.rejectedFrames++;
  check('recovery after rejected frames', rest);
  // Use the actual page sampler, including normalized quaternions and the
  // audited nonlinear floor lift, at every knot and interval midpoint.
  const transitionPath = new URL('./transitions/data.json', import.meta.url);
  if (existsSync(transitionPath)) {
    const transitionData = JSON.parse(readFileSync(transitionPath, 'utf8'));
    const runtime = createTransitions(THREE, A, transitionData);
    assert(runtime.valid, 'a verified transition packet must be runtime-valid');
    assert(Object.keys(transitionData.tracks || {}).length > 0, 'transition package has no tracks');
    for (const [id, track] of Object.entries(transitionData.tracks)) {
      assert(track.frames.length >= 2, id + ': track requires multiple motion knots');
      for (let i = 0; i < track.frames.length; i++) {
        const a = track.frames[i];
        const knot = runtime.sampleTrack(id, a.t);
        check(id + '/' + i, A.computeWorld(knot.local, knot.rootQ, knot.rootP), false); stats.transitionStates++;
        if (i + 1 < track.frames.length) {
          const b = track.frames[i + 1];
          const mid = runtime.sampleTrack(id, (a.t + b.t) / 2);
          check(id + '/' + i + '.5', A.computeWorld(mid.local, mid.rootQ, mid.rootP), false); stats.transitionStates++;
        }
      }
    }
  } else assert(!process.argv.includes('--require-transitions'), 'build transitions/data.json before the required motion check');
  assert.equal(JSON.stringify(data), manifestBefore);
  assert.deepEqual(readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url)), bonesBefore, 'ready skeleton changed');
  console.log(JSON.stringify({ ...stats, parts: surfaces.parts.length, status: 'PASS', scope: 'bone-following joint display, normals, picking, bounds and atomic invalid-frame rejection; no tissue-contact validation' }, null, 2));
} finally { surfaces.dispose(); }
