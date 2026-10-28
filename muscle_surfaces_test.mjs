// CPU geometry/packing checks, without a browser or physics regeneration.
// node muscle_surfaces_test.mjs                 synthetic + actual data if present
// node muscle_surfaces_test.mjs --synthetic     fast, no atlas data required
// node muscle_surfaces_test.mjs --actual        require the final atlas package
// Add --all-vertices to skin every vertex instead of every 17th actual vertex.
// Source and licence: muscles/README.md (BodyParts3D / DBCLS, CC BY 4.0).
// These checks establish geometry consistency, not anatomical/clinical accuracy.
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { gzipSync, gunzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createMuscleSurfaces } from './muscle_surface_render.js';

const A = createAnatomy(THREE);
const root = A.SEG.find(s => !s.parent);
const restW = A.computeWorld(A.STAND.local, A.STAND.rootQ, root.J.clone());
const instances = A.strandInstances().filter(s => s.kind === 'muscle');
const poses = JSON.parse(readFileSync(new URL('./poses.json', import.meta.url), 'utf8'));
const cycle = JSON.parse(readFileSync(new URL('./cycle_keys/cycle.json', import.meta.url), 'utf8'));
const args = new Set(process.argv.slice(2));
const point = new THREE.Vector3(), target = new THREE.Vector3();
const partKey = p => p.unit + '_' + p.side;
const summaries = [];

function packSynthetic(splitFaces = false) {
  const inst = instances.find(s => s.unit === 'bicLong' && s.side === 'L');
  const curve = new THREE.CatmullRomCurve3(A.strandPath(restW, inst), false, 'centripetal');
  const centers = curve.getSpacedPoints(9);
  let positions = [], indices = [];
  for (let i = 0; i < 10; i++) for (let j = 0; j < 8; j++) {
    positions.push(centers[i].x + 0.006 * Math.cos(j * Math.PI / 4), centers[i].y, centers[i].z + 0.006 * Math.sin(j * Math.PI / 4));
    if (i < 9) {
      const a = i * 8 + j, b = i * 8 + (j + 1) % 8;
      indices.push(a, b, a + 8, b, b + 8, a + 8);
    }
  }
  if (splitFaces) {
    positions = indices.flatMap(index => positions.slice(index * 3, index * 3 + 3));
    indices = Array.from({ length: positions.length / 3 }, (_, k) => k);
  }
  return { original: positions, data: packArrays(inst.unit, inst.side, positions, indices) };
}

function packArrays(unit, side, positions, indices) {
  const lo = [0, 1, 2].map(axis => Math.min(...positions.filter((_, k) => k % 3 === axis)));
  const hi = [0, 1, 2].map(axis => Math.max(...positions.filter((_, k) => k % 3 === axis)));
  const q = Uint16Array.from(positions.map((v, k) => Math.round((v - lo[k % 3]) / (hi[k % 3] - lo[k % 3]) * 65535)));
  for (let k = q.length - 1; k >= 3; k--) q[k] = (q[k] - q[k - 3]) & 65535;
  const ind = Uint16Array.from(indices);
  for (let k = ind.length - 1; k > 0; k--) ind[k] = (ind[k] - ind[k - 1]) & 65535;
  return { v: 1, src: 'synthetic test fixture', license: 'test fixture', parts: [{ unit, side, source: 'reconstructed', ids: [], n: positions.length / 3, m: indices.length / 3, lo, hi }], data: gzipSync(Buffer.concat([Buffer.from(q.buffer), Buffer.from(ind.buffer)])).toString('base64') };
}

function inspectPackage(data, actual) {
  assert.equal(data.v, 1);
  assert(Array.isArray(data.parts) && data.parts.length > 0);
  if (actual) { assert.match(data.src, /BodyParts3D/i); assert.match(data.license, /CC BY\s*4\.0/i); }
  const raw = gunzipSync(Buffer.from(data.data, 'base64'));
  const expectedLength = data.parts.reduce((sum, p) => sum + 6 * (p.n + p.m), 0);
  assert.equal(raw.length, expectedLength, 'manifest byte counts must match the gzip payload exactly');
  const groups = new Map(), real = new Set(), reconstructed = new Set(), usedIds = new Set();
  let offset = 0;
  for (const part of data.parts) {
    assert(A.UNITS[part.unit], `unknown model unit ${part.unit}`);
    assert(['L', 'R'].includes(part.side));
    assert(Number.isInteger(part.n) && part.n > 0 && part.n < 65536);
    assert(Number.isInteger(part.m) && part.m > 0);
    assert.equal(part.lo.length, 3); assert.equal(part.hi.length, 3);
    assert(part.lo.concat(part.hi).every(Number.isFinite));
    for (let axis = 0; axis < 3; axis++) assert(part.hi[axis] >= part.lo[axis]);
    const centerX = (part.lo[0] + part.hi[0]) / 2;
    assert(centerX * (part.side === 'L' ? 1 : -1) >= -0.04, `${partKey(part)}/${part.ids}: atlas geometry is on the opposite side of the rest rig`);
    assert(['bp3d', 'bodyparts3d', 'reconstructed'].includes(part.source), 'each part must state its provenance');
    assert(Array.isArray(part.ids));
    if (part.source === 'reconstructed') {
      reconstructed.add(part.unit);
      assert.equal(part.ids.length, 0, 'a reconstruction must not claim an atlas element');
    } else {
      real.add(part.unit); assert(part.ids.length > 0, 'an atlas surface needs source element ids');
      for (const id of part.ids) {
        assert.match(id, /^FJ\d+M?$/);
        const key = id + '_' + part.side;
        assert(!usedIds.has(key), `duplicate source volume ${key}`); usedIds.add(key);
      }
    }
    const positions = new Float32Array(part.n * 3), acc = [0, 0, 0];
    for (let v = 0; v < part.n; v++) for (let axis = 0; axis < 3; axis++) {
      acc[axis] = (acc[axis] + raw.readUInt16LE(offset)) & 65535; offset += 2;
      positions[v * 3 + axis] = part.lo[axis] + (part.hi[axis] - part.lo[axis]) * acc[axis] / 65535;
    }
    let index = 0;
    for (let k = 0; k < part.m * 3; k++) {
      index = (index + raw.readUInt16LE(offset)) & 65535; offset += 2;
      assert(index < part.n, `${partKey(part)}: triangle index outside its source part`);
    }
    const key = partKey(part);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(positions);
  }
  assert.equal(offset, raw.length);
  for (const unit of real) assert(!reconstructed.has(unit), `${unit}: conflicting source labels`);
  if (actual) {
    const expectedCoverage = new Set(instances.map(partKey));
    assert.deepEqual(new Set(groups.keys()), expectedCoverage, 'final surfaces must cover every model unit on both sides');
    const mapping = JSON.parse(readFileSync(new URL('./muscles/source-map.json', import.meta.url), 'utf8'));
    const missing = Object.entries(mapping.units).filter(([, entry]) => entry.status === 'missing').map(([unit]) => unit);
    assert.deepEqual(reconstructed, new Set(missing), 'only unmapped atlas units may be called reconstructions');
    const reportURL = new URL('./muscles/coverage.json', import.meta.url);
    if (existsSync(reportURL)) {
      const report = JSON.parse(readFileSync(reportURL, 'utf8'));
      assert.equal(report.sourceUnits, real.size); assert.equal(report.reconstructedUnits, reconstructed.size);
      assert.equal(report.parts, data.parts.length); assert.equal(report.triangles, data.parts.reduce((sum, p) => sum + p.m, 0));
    }
  }
  return { groups, real: real.size, reconstructed: reconstructed.size };
}

function eachVertex(mesh, stride, visit) {
  const count = mesh.geometry.attributes.position.count;
  for (let k = 0; k < count; k += stride) visit(k);
  if ((count - 1) % stride) visit(count - 1);
}

async function runSuite(label, data, stride) {
  const manifest = inspectPackage(data, label === 'actual');
  const surfaces = await createMuscleSurfaces(A, THREE, data, { instances });
  assert(surfaces, `${label}: renderer must load`);
  try {
    assert.deepEqual(surfaces.covered, new Set(manifest.groups.keys()));
    assert.equal(surfaces.pickables.length, surfaces.covered.size, 'one skin per unit/side avoids duplicate rendering');
    let maxRestError = 0, maxRigidError = 0, skinnedSamples = 0, maxProximalAttachmentError = 0, maxSourceSeamGap = 0;
    const proximalSamples = [], sourceSeams = [];
    for (const mesh of surfaces.pickables) {
      assert(mesh.isSkinnedMesh); assert.equal(mesh.userData.kind, 'muscle');
      const partPositions = manifest.groups.get(partKey(mesh.userData));
      let vertex = 0;
      for (const positions of partPositions) {
        const coincident = new Map();
        for (let k = 0; k < positions.length; k += 3, vertex++) {
          target.fromArray(positions, k); mesh.getVertexPosition(vertex, point);
          maxRestError = Math.max(maxRestError, target.distanceTo(point));
          const key = positions[k] + '|' + positions[k + 1] + '|' + positions[k + 2];
          if (coincident.has(key)) sourceSeams.push({ mesh, a: coincident.get(key), b: vertex });
          else coincident.set(key, vertex);
        }
      }
      assert.equal(vertex, mesh.geometry.attributes.position.count);
      for (const name of ['position', 'normal', 'skinWeight']) assert(mesh.geometry.attributes[name].array.every(Number.isFinite));
      assert.equal(mesh.userData.skinningMethod, 'bend-aware-dual-quaternion');
      const belly = mesh.geometry.attributes.muscleRadial;
      assert(belly && belly.itemSize === 4 && belly.array.every(Number.isFinite));
      for (let k = 0; k < belly.count; k++) {
        assert(belly.getW(k) >= 0 && belly.getW(k) <= 1, 'belly correction must fade to a bounded nonnegative attachment mask');
        assert(Math.hypot(belly.getX(k), belly.getY(k), belly.getZ(k)) <= .07500001,
          'transverse display displacement must remain bounded');
      }
      const weights = mesh.geometry.attributes.skinWeight, boneIds = mesh.geometry.attributes.skinIndex;
      for (let v = 0; v < vertex; v++) {
        let sum = 0;
        for (let j = 0; j < 4; j++) { const k = v * 4 + j; assert(boneIds.array[k] < mesh.skeleton.bones.length); assert(weights.array[k] >= 0); sum += weights.array[k]; }
        assert(Math.abs(sum - 1) < 1e-6, 'skin weights must form a partition of unity');
      }
      if (label === 'actual' && mesh.userData.unit === 'fingerFlex') {
        const attr = mesh.geometry.attributes.position;
        let proximal = 0, distal = 0;
        for (let k = 0; k < attr.count; k++) {
          if (attr.getY(k) > attr.getY(proximal)) proximal = k;
          if (attr.getY(k) < attr.getY(distal)) distal = k;
        }
        const influence = (vertex, matches) => {
          let sum = 0;
          for (let j = 0; j < 4; j++) {
            const offset = vertex * 4 + j, bone = mesh.skeleton.bones[boneIds.array[offset]];
            if (matches(bone.name)) sum += weights.array[offset];
          }
          return sum;
        };
        const fore = 'fore_' + mesh.userData.side;
        // The sparse proximal FDS/FDP tendon must retain the forearm, even when
        // dense distal finger geometry competes for a four-segment skin palette.
        assert(influence(proximal, name => name === fore) > 0.5, 'proximal finger-flexor tendon was incorrectly attached to the hand/fingers');
        assert(influence(distal, name => /^f[2-5][abc]_/.test(name)) > 0.5, 'distal finger-flexor tendon lost its finger attachment');
        proximalSamples.push({ mesh, vertex: proximal, rest: new THREE.Vector3().fromBufferAttribute(attr, proximal), seg: fore });
      }
    }
    assert(maxRestError < 8e-7, `${label}: binding changed the supplied rest shape (${maxRestError} m)`);
    const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.6, -0.4, 0.3));
    const shift = new THREE.Vector3(0.2, 0.3, -0.7);
    const rigidW = Object.fromEntries(Object.entries(restW).map(([id, w]) => [id, { p: w.p.clone().applyQuaternion(rotation).add(shift), q: rotation.clone().multiply(w.q) }]));
    surfaces.update(rigidW);
    for (const mesh of surfaces.pickables) eachVertex(mesh, stride, k => {
      mesh.getVertexPosition(k, point);
      target.fromBufferAttribute(mesh.geometry.attributes.position, k).applyQuaternion(rotation).add(shift);
      maxRigidError = Math.max(maxRigidError, target.distanceTo(point));
    });
    assert(maxRigidError < 2e-6, `${label}: visual skin failed rigid transformation (${maxRigidError} m)`);

    function checkState(name, W) {
      const worldBefore = JSON.stringify(W);
      const cache = new Map();
      for (const inst of instances) if (surfaces.covered.has(partKey(inst))) {
        const path = A.strandPath(W, inst);
        for (const p of path) Object.freeze(p);
        cache.set(inst.id, Object.freeze(path));
      }
      surfaces.update(W, cache);
      assert.equal(JSON.stringify(W), worldBefore, `${name}: renderer changed the rig`);
      for (const mesh of surfaces.pickables) {
        const bounds = surfaces.getBounds(mesh.userData.unit, mesh.userData.side);
        assert(bounds && !bounds.isEmpty());
        assert(Number.isFinite(mesh.boundingSphere.radius) && mesh.boundingSphere.radius > 0);
        for (const bone of mesh.skeleton.bones) {
          assert(bone.matrixWorld.elements.every(Number.isFinite));
          assert.deepEqual(bone.scale.toArray(), [1, 1, 1], 'belly correction must not scale anatomical support transforms');
        }
        eachVertex(mesh, stride, k => {
          mesh.getVertexPosition(k, point); skinnedSamples++;
          assert(Number.isFinite(point.x) && Number.isFinite(point.y) && Number.isFinite(point.z), `${name}/${mesh.userData.unit}: nonfinite deformed vertex`);
          assert(bounds.containsPoint(point), `${name}/${mesh.userData.unit}: picking bounds exclude the skin`);
          assert(mesh.boundingSphere.containsPoint(point), `${name}: raycast sphere excludes the skin`);
        });
      }
      for (const sample of proximalSamples) {
        sample.mesh.getVertexPosition(sample.vertex, point);
        target.copy(A.pt(W, sample.seg, sample.rest));
        const error = point.distanceTo(target);
        maxProximalAttachmentError = Math.max(maxProximalAttachmentError, error);
        assert(error < 0.002, `${name}: a proximal finger-flexor tendon flew away from its forearm attachment (${error} m)`);
      }
      for (const seam of sourceSeams) {
        const gap = seam.mesh.getVertexPosition(seam.a, point).distanceTo(seam.mesh.getVertexPosition(seam.b, target));
        maxSourceSeamGap = Math.max(maxSourceSeamGap, gap);
        assert(gap < 2e-6, `${name}/${seam.mesh.userData.unit}: an exact source seam pulled apart (${gap} m)`);
      }
    }
    for (const pose of A.POSES) {
      const conf = A.poseFromData(poses[pose.id]);
      for (const t of [0, 0.5, 1]) checkState(`${pose.id}/${t}`, A.poseAt(conf, t).W);
    }
    const cycleIndices = [0, 42, 76, 106, 140, 155, 184, 205, 235, 261];
    let asymmetricFrames = 0;
    for (const index of cycleIndices) {
      const frame = cycle.frames[index]; assert(frame, `missing cycle sample ${index}`);
      const local = Object.fromEntries(Object.entries(frame.local).map(([id, q]) => [id, new THREE.Quaternion(...q)]));
      if (A.SEG.some(s => s.name.endsWith('_L') && A.mirrorQ(local[s.name]).angleTo(local[s.name.replace(/_L$/, '_R')]) > 1e-3)) asymmetricFrames++;
      checkState(`cycle/${index}`, A.computeWorld(local, new THREE.Quaternion(...frame.root), new THREE.Vector3(...frame.rootP)));
    }
    assert(asymmetricFrames > 0, 'cycle samples must exercise asymmetric motion');
    let practicePoses = 0;
    const practiceFile = new URL('./practice/poses.json', import.meta.url);
    if (existsSync(practiceFile)) {
      for (const [id, pose] of Object.entries(JSON.parse(readFileSync(practiceFile, 'utf8')).poses)) {
        const local = Object.fromEntries(Object.entries(pose.local).map(([name, q]) => [name, new THREE.Quaternion(...q).normalize()]));
        checkState('practice/' + id, A.computeWorld(local, new THREE.Quaternion(...pose.root).normalize(), new THREE.Vector3(...pose.rootP)));
        practicePoses++;
      }
      assert.equal(practicePoses, 24, 'the complete practice reconstruction must be exercised');
    }
    surfaces.setVisibility(unit => !A.UNITS[unit].deep);
    for (const mesh of surfaces.pickables) assert.equal(mesh.visible, !A.UNITS[mesh.userData.unit].deep);
    surfaces.setVisibility(() => false); assert(surfaces.pickables.every(mesh => !mesh.visible));
    surfaces.setVisibility(() => true); assert(surfaces.pickables.every(mesh => mesh.visible));
    const offColors = surfaces.pickables.map(mesh => mesh.geometry.attributes.color.array.slice());
    surfaces.setAppearance(() => ({ active: true, emissive: new THREE.Color('green'), intensity: 0.7, opacity: 0.2 }));
    let greenChanged = false;
    for (let m = 0; m < surfaces.pickables.length; m++) {
      const mesh = surfaces.pickables[m];
      assert.equal(mesh.material.opacity, 0.2); assert.equal(mesh.material.transparent, true);
      assert.equal(mesh.material.depthWrite, false); assert.equal(mesh.material.emissiveIntensity, 0.7);
      const colors = mesh.geometry.attributes.color.array;
      for (let k = 1; k < colors.length; k += 3) if (colors[k] > offColors[m][k]) greenChanged = true;
    }
    assert(greenChanged, 'working muscle bellies must turn green');
    surfaces.setAppearance(() => ({ active: false, opacity: 1 }));
    for (let m = 0; m < surfaces.pickables.length; m++) {
      const mesh = surfaces.pickables[m];
      assert.deepEqual(mesh.geometry.attributes.color.array, offColors[m]);
      assert.equal(mesh.material.transparent, false); assert.equal(mesh.material.depthWrite, true);
    }
    summaries.push({ suite: label, units: manifest.real + manifest.reconstructed, bodyPartsUnits: manifest.real, reconstructedUnits: manifest.reconstructed, meshes: surfaces.pickables.length, triangles: data.parts.reduce((sum, p) => sum + p.m, 0), poseSliderStates: A.POSES.length * 3, cycleFrames: cycleIndices.length, practicePoses, asymmetricFrames, vertexStride: stride, skinnedSamples, maxRestError, maxRigidError, maxProximalAttachmentError, exactSourceSeams: sourceSeams.length, maxSourceSeamGap });
  } finally { surfaces.dispose(); }
}

const fixture = packSynthetic();
if (!args.has('--actual')) {
  await runSuite('synthetic', fixture.data, 1);
  // Identical source geometry with every face split into separate OBJ vertices
  // must keep its seams closed through the entire pose/round suite.
  await runSuite('synthetic-split-faces', packSynthetic(true).data, 1);
  // A wide muscle has more than four anatomical supports. Four GPU influences
  // per vertex must not mean dropping a spinal or pelvic support for the whole
  // connected part.
  const lat = instances.filter(s => s.unit === 'lat' && s.side === 'L');
  const anchors = lat.map(s => s.pts[0]).concat(lat[0].pts.at(-1));
  const anchorPoints = anchors.map(p => A.pt(restW, p.seg, p.p));
  const center = anchorPoints.reduce((sum, p) => sum.add(p), new THREE.Vector3()).divideScalar(anchorPoints.length);
  const widePositions = [center, ...anchorPoints].flatMap(p => p.toArray()), wideIndices = [];
  for (let k = 1; k < anchorPoints.length; k++) wideIndices.push(0, k, k + 1);
  const wide = await createMuscleSurfaces(A, THREE, packArrays('lat', 'L', widePositions, wideIndices));
  try {
    const mesh = wide.pickables[0], ids = mesh.geometry.attributes.skinIndex, weights = mesh.geometry.attributes.skinWeight;
    for (let k = 0; k < anchors.length; k++) {
      let own = 0;
      for (let j = 0; j < 4; j++) if (mesh.skeleton.bones[ids.getComponent(k + 1, j)].name === anchors[k].seg) own += weights.getComponent(k + 1, j);
      assert(own > .99, `wide fan lost its ${anchors[k].seg} attachment to a four-slot part palette`);
    }
  } finally { wide.dispose(); }
  // Exact coincident front/back faces of a thin lamina must retain opposing,
  // finite normals, rather than being averaged to a dark zero vector.
  const a = center.clone(), b = center.clone().add(new THREE.Vector3(.01, 0, 0)), c = center.clone().add(new THREE.Vector3(0, .01, 0));
  const lamina = await createMuscleSurfaces(A, THREE, packArrays('lat', 'L', [a, b, c, a, b, c].flatMap(p => p.toArray()), [0, 1, 2, 3, 5, 4]));
  try {
    const normal = lamina.pickables[0].geometry.attributes.normal;
    for (let k = 0; k < 3; k++) {
      const front = new THREE.Vector3().fromBufferAttribute(normal, k), back = new THREE.Vector3().fromBufferAttribute(normal, k + 3);
      assert(front.length() > .99 && back.length() > .99 && front.dot(back) < -.99, 'opposing thin-lamina normals were cancelled');
    }
  } finally { lamina.dispose(); }
  const decoded = inspectPackage(fixture.data, false).groups.get('bicLong_L')[0];
  let quantisationError = 0;
  for (let k = 0; k < decoded.length; k++) quantisationError = Math.max(quantisationError, Math.abs(decoded[k] - fixture.original[k]));
  assert(quantisationError < 1e-5, 'test fixture packing error must stay below 10 micrometres');
  await assert.rejects(() => createMuscleSurfaces(A, THREE, { ...fixture.data, parts: [{ ...fixture.data.parts[0], n: 79 }] }), /length/);
  const corrupt = gunzipSync(Buffer.from(fixture.data.data, 'base64'));
  corrupt.writeUInt16LE(65535, fixture.data.parts[0].n * 6);
  await assert.rejects(() => createMuscleSurfaces(A, THREE, { ...fixture.data, data: gzipSync(corrupt).toString('base64') }), /triangle/);
  const originalDecompressor = globalThis.DecompressionStream;
  try { globalThis.DecompressionStream = undefined; assert.equal(await createMuscleSurfaces(A, THREE, fixture.data), null); }
  finally { globalThis.DecompressionStream = originalDecompressor; }
}
if (!args.has('--synthetic')) {
  const url = new URL('./muscles/muscles_bp3d.json', import.meta.url);
  if (existsSync(url)) await runSuite('actual', JSON.parse(readFileSync(url, 'utf8')), args.has('--all-vertices') ? 1 : 17);
  else { assert(!args.has('--actual'), 'actual atlas package is required but missing'); summaries.push({ suite: 'actual', skipped: 'muscles/muscles_bp3d.json is not available; no actual-geometry claim' }); }
}
console.log(JSON.stringify({ passed: summaries }, null, 2));
