// Rigid geometry, picking and integration checks without browser/physics rebuild.
// node joint_surfaces_test.mjs --synthetic
// node joint_surfaces_test.mjs --actual [--data path/to/joints.json]
// These checks establish consistency, not measured joint contact mechanics.
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { gzipSync, gunzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createJointSurfaces } from './joint_surface_render.js';

const A = createAnatomy(THREE), args = process.argv.slice(2);
const poses = JSON.parse(readFileSync(new URL('./poses.json', import.meta.url), 'utf8'));
const cycle = JSON.parse(readFileSync(new URL('./cycle_keys/cycle.json', import.meta.url), 'utf8'));
const root = A.SEG.find(s => !s.parent);
const restW = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.J.clone());
const p = new THREE.Vector3(), expected = new THREE.Vector3();
const summaries = [];

function pack(input) {
  const parts = [], blobs = [];
  for (const { positions, indices, ...meta } of input) {
    const lo = [0, 1, 2].map(a => Math.min(...positions.filter((_, k) => k % 3 === a)));
    const hi = [0, 1, 2].map(a => Math.max(...positions.filter((_, k) => k % 3 === a)));
    const q = Uint16Array.from(positions.map((x, k) => Math.round((x - lo[k % 3]) / (hi[k % 3] - lo[k % 3] || 1) * 65535)));
    const ind = Uint16Array.from(indices);
    for (let k = q.length - 1; k >= 3; k--) q[k] = (q[k] - q[k - 3]) & 65535;
    for (let k = ind.length - 1; k; k--) ind[k] = (ind[k] - ind[k - 1]) & 65535;
    blobs.push(Buffer.from(q.buffer), Buffer.from(ind.buffer));
    parts.push({ ...meta, n: positions.length / 3, m: indices.length / 3, lo, hi });
  }
  return { v: 1, src: 'synthetic geometry fixture', license: 'test fixture', parts, data: gzipSync(Buffer.concat(blobs)).toString('base64') };
}

function synthetic() {
  const triangle = (seg, dy = 0) => [
    A.S[seg].J.clone().add(new THREE.Vector3(-.01, dy, 0)),
    A.S[seg].J.clone().add(new THREE.Vector3(.01, dy, 0)),
    A.S[seg].J.clone().add(new THREE.Vector3(0, dy + .01, .004)),
  ].flatMap(v => v.toArray());
  const crescent = [], crescentIndices = [], center = A.S.shank_R.J;
  // An open 250-degree C with a wedge-shaped section. A renderer must not fill
  // the central hole or bridge the two horn ends while unpacking the indices.
  for (let i = 0; i <= 16; i++) {
    const angle = (55 + 250 * i / 16) * Math.PI / 180;
    for (const [radius, height] of [[.014, -.0015], [.008, -.0015], [.014, .0025]]) crescent.push(center.x + radius * Math.cos(angle), center.y + height, center.z + radius * Math.sin(angle));
    if (i < 16) for (let j = 0; j < 3; j++) {
      const a = i * 3 + j, b = i * 3 + (j + 1) % 3;
      crescentIndices.push(a, a + 3, b, b, a + 3, b + 3);
    }
  }
  crescentIndices.push(0, 1, 2, 48, 50, 49);
  const common = { source: 'reconstructed', method: 'synthetic fixture', boneRefs: ['fixture'], latin: 'fixture', side: 'L' };
  return pack([
    { ...common, id: 'fixture-humeral', joint: 'gh_L', seg: 'hum_L', type: 'cartilage', name: 'Fixture humeral patch', replaces: ['Хрящ головки плечевой кости'], positions: triangle('hum_L'), indices: [0, 1, 2] },
    { ...common, id: 'fixture-glenoid', joint: 'gh_L', seg: 'scap_L', type: 'labrum', name: 'Fixture glenoid rim', replaces: ['Суставная губа'], positions: triangle('scap_L', -.01), indices: [0, 1, 2] },
    { ...common, side: 'R', id: 'fixture-meniscus', joint: 'kn_R', seg: 'shank_R', type: 'meniscus', name: 'Fixture meniscus crescent', replaces: ['Медиальный мениск'], positions: crescent, indices: crescentIndices },
  ]);
}

function unpack(data) {
  const raw = gunzipSync(Buffer.from(data.data, 'base64'));
  assert.equal(raw.length, data.parts.reduce((sum, m) => sum + (m.n + m.m) * 6, 0));
  const result = new Map(); let offset = 0;
  for (const meta of data.parts) {
    const acc = [0, 0, 0], positions = new Float64Array(meta.n * 3), indices = new Uint16Array(meta.m * 3);
    for (let k = 0; k < positions.length; k++, offset += 2) {
      const a = k % 3; acc[a] = (acc[a] + raw.readUInt16LE(offset)) & 65535;
      positions[k] = meta.lo[a] + (meta.hi[a] - meta.lo[a]) * acc[a] / 65535;
    }
    let index = 0;
    for (let k = 0; k < indices.length; k++, offset += 2) { index = (index + raw.readUInt16LE(offset)) & 65535; indices[k] = index; }
    result.set(meta.id, { positions, indices });
  }
  return result;
}

async function runSuite(label, data) {
  const original = JSON.stringify(data), decoded = unpack(data);
  const surfaces = await createJointSurfaces(A, THREE, data);
  let transformedVertices = 0, maxError = 0, states = 0;
  try {
    assert.equal(surfaces.parts.length, data.parts.length);
    assert.equal(surfaces.pickables.length, data.parts.length);
    assert.equal(new Set(surfaces.pickables).size, data.parts.length, 'one mesh for each unique part');
    assert.deepEqual(surfaces.covered, new Set(data.parts.flatMap(m => m.replaces.map(name => m.seg + '|' + name))));
    assert.equal(new Set(surfaces.parts.map(part => part.mat)).size, data.parts.length, 'joint glow must not share a mutable material');
    const snapshots = new Map();
    for (const part of surfaces.parts) {
      assert.equal(part.mesh.userData.kind, 'cart'); assert.equal(part.mesh.userData.joint, part.joint);
      assert.equal(part.mesh.userData.source, part.source); assert.equal(part.mesh.userData.method, part.method);
      assert.equal(part.mesh.matrixAutoUpdate, false);
      const g = part.mesh.geometry;
      assert.deepEqual(g.index.array, decoded.get(part.id).indices, 'runtime must preserve open rims/holes and source winding');
      for (const attr of ['position', 'normal']) assert(g.attributes[attr].array.every(Number.isFinite));
      for (let k = 0; k < g.attributes.normal.count; k++) {
        const length = p.fromBufferAttribute(g.attributes.normal, k).length();
        assert(Math.abs(length - 1) < 1e-5, `${part.id}: runtime vertex ${k} has a non-unit normal (${length})`);
      }
      assert(g.boundingSphere.radius > 0 && Number.isFinite(g.boundingSphere.radius));
      snapshots.set(part.id, { positions: g.attributes.position.array.slice(), normals: g.attributes.normal.array.slice(), positionVersion: g.attributes.position.version, normalVersion: g.attributes.normal.version });
    }
    function checkState(name, W) {
      const before = JSON.stringify(W); surfaces.update(W); states++;
      assert.equal(JSON.stringify(W), before, `${name}: joint renderer changed the skeleton`);
      for (const part of surfaces.parts) {
        const mesh = part.mesh, g = mesh.geometry, source = decoded.get(part.id).positions;
        const bounds = surfaces.getBounds(part.joint), snapshot = snapshots.get(part.id);
        assert(bounds && !bounds.isEmpty());
        assert(mesh.matrixWorld.elements.every(Number.isFinite));
        for (let k = 0; k < g.attributes.position.count; k++) {
          p.fromBufferAttribute(g.attributes.position, k).applyMatrix4(mesh.matrixWorld);
          expected.copy(A.pt(W, part.seg, new THREE.Vector3().fromArray(source, k * 3)));
          const error = p.distanceTo(expected); maxError = Math.max(maxError, error); transformedVertices++;
          assert(error < 3e-7, `${name}/${part.id}: cartilage separated from its own bone (${error} m)`);
          assert(bounds.clone().expandByScalar(1e-8).containsPoint(p), `${name}/${part.id}: posed bounds exclude surface`);
        }
        assert.deepEqual(g.attributes.position.array, snapshot.positions, 'pose update rewrote static vertices');
        assert.deepEqual(g.attributes.normal.array, snapshot.normals, 'pose update recomputed static normals');
        assert.equal(g.attributes.position.version, snapshot.positionVersion);
        assert.equal(g.attributes.normal.version, snapshot.normalVersion);
      }
    }
    checkState('rest', restW);
    const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(.6, -.4, .3)), shift = new THREE.Vector3(.2, .3, -.7);
    const rigidW = Object.fromEntries(Object.entries(restW).map(([id, w]) => [id, { p: w.p.clone().applyQuaternion(rotation).add(shift), q: rotation.clone().multiply(w.q) }]));
    checkState('root rotation and translation', rigidW);
    for (const pose of A.POSES) for (const t of [0, .5, 1]) checkState(`${pose.id}/${t}`, A.poseAt(A.poseFromData(poses[pose.id]), t).W);
    const cycleIndices = [0, 42, 76, 106, 140, 155, 184, 205, 235, 261];
    for (const index of cycleIndices) {
      const frame = cycle.frames[index];
      const local = Object.fromEntries(Object.entries(frame.local).map(([seg, q]) => [seg, new THREE.Quaternion(...q)]));
      checkState(`cycle/${index}`, A.computeWorld(local, new THREE.Quaternion(...frame.root), new THREE.Vector3(...frame.rootP)));
    }
    surfaces.update(restW);
    // The same meshes used for visible cartilage must remain raycastable with
    // the existing kind:'cart', joint:<id> selection convention.
    const mesh = surfaces.pickables[0], g = mesh.geometry;
    const tri = [0, 1, 2].map(k => new THREE.Vector3().fromBufferAttribute(g.attributes.position, g.index.getX(k)).applyMatrix4(mesh.matrixWorld));
    const center = tri.reduce((sum, v) => sum.add(v), new THREE.Vector3()).divideScalar(3);
    const normal = tri[1].clone().sub(tri[0]).cross(tri[2].clone().sub(tri[0])).normalize();
    const ray = new THREE.Raycaster(center.clone().addScaledVector(normal, .02), normal.clone().negate(), 0, .04);
    assert(ray.intersectObject(mesh).some(hit => hit.object.userData.joint === mesh.userData.joint), 'visible joint surface could not be picked');
    if (label === 'synthetic') {
      const crescent = surfaces.parts.find(part => part.id === 'fixture-meniscus').mesh;
      for (const dx of [0, .014]) {
        const ray = new THREE.Raycaster(A.S.shank_R.J.clone().add(new THREE.Vector3(dx, .02, 0)), new THREE.Vector3(0, -1, 0), 0, .04);
        assert.equal(ray.intersectObject(crescent).length, 0, dx ? 'open crescent horns were bridged' : 'crescent central hole was filled');
      }
    }
    surfaces.setVisibility(part => part.joint === 'gh_L');
    for (const part of surfaces.parts) assert.equal(part.mesh.visible, part.joint === 'gh_L');
    surfaces.setVisibility(false); assert.equal(surfaces.getBounds(), null);
    assert(surfaces.getBounds(null, false), 'hidden surfaces retain optional bounds');
    surfaces.setVisibility(true);
    surfaces.setAppearance(part => ({ emissive: part.joint === 'gh_L' ? 0xffc04a : 0, emissiveIntensity: part.joint === 'gh_L' ? .8 : 0, opacity: part.joint === 'gh_L' ? 1 : .25 }));
    for (const part of surfaces.parts) {
      assert.equal(part.mat.emissiveIntensity, part.joint === 'gh_L' ? .8 : 0);
      assert.equal(part.mat.opacity, part.joint === 'gh_L' ? 1 : .25);
      assert.equal(part.mat.transparent, part.joint !== 'gh_L'); assert.equal(part.mat.depthWrite, part.joint === 'gh_L');
    }
    surfaces.setAppearance(part => { part.mat.emissiveIntensity = 0; });
    assert(surfaces.parts.every(part => part.mat.emissiveIntensity === 0), 'mutation callbacks must support integration');
    assert.equal(JSON.stringify(data), original, 'renderer changed the packed source manifest');
    summaries.push({ suite: label, parts: data.parts.length, triangles: data.parts.reduce((sum, m) => sum + m.m, 0), jointIds: new Set(data.parts.map(m => m.joint)).size, states, transformedVertices, maxRigidAttachmentError: maxError });
  } finally { surfaces.dispose(); assert.equal(surfaces.group.children.length, 0); }
}

const fixture = synthetic();
if (!args.includes('--actual')) {
  await runSuite('synthetic', fixture);
  const duplicate = structuredClone(fixture); duplicate.parts[1].id = duplicate.parts[0].id;
  await assert.rejects(createJointSurfaces(A, THREE, duplicate), /identity/);
  const truncated = structuredClone(fixture); truncated.data = gzipSync(gunzipSync(Buffer.from(fixture.data, 'base64')).subarray(0, -2)).toString('base64');
  await assert.rejects(createJointSurfaces(A, THREE, truncated), /payload size/);
  const badIndex = structuredClone(fixture), raw = gunzipSync(Buffer.from(fixture.data, 'base64'));
  raw.writeUInt16LE(badIndex.parts[0].n, badIndex.parts[0].n * 6);
  badIndex.data = gzipSync(raw).toString('base64');
  await assert.rejects(createJointSurfaces(A, THREE, badIndex), /index is outside/);
}
const actualPath = args.includes('--data') ? args[args.indexOf('--data') + 1] : new URL('./joints/joints.json', import.meta.url);
if (!args.includes('--synthetic') && (args.includes('--actual') || existsSync(actualPath))) {
  assert(existsSync(actualPath), 'build joints/joints.json before running --actual');
  await runSuite('actual', JSON.parse(readFileSync(actualPath, 'utf8')));
}
console.log(JSON.stringify(summaries, null, 2));
