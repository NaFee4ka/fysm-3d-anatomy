// Genuine BodyParts3D disk shapes registered to the finished generic skeleton.
// The two neighboring bone transforms drive a geometric display approximation.
// This is not a pressure/volume/contact law and does not change force estimates.
export async function createSpineSurfaces(THREE, A, data, opts = {}) {
  if (!data || data.schemaVersion !== 1 || !Array.isArray(data.parts) || data.parts.length !== 23
      || data.encoding !== 'u16-position-deltas,u16-index-deltas,u16-superior-weights;gzip;base64') {
    throw new Error('Unsupported spine disk surface package');
  }
  if (typeof DecompressionStream === 'undefined') throw new Error('Spine disks require gzip decompression');
  const ids = new Set(), drivers = new Set();
  let expectedBytes = 0;
  for (const part of data.parts) {
    if (!part.id || ids.has(part.id) || !part.level || !part.name || !part.sourceIds?.length
        || !['metadata', 'inferred'].includes(part.levelAssignment)
        || !A.S[part.inferior?.seg] || !A.S[part.superior?.seg]
        || !Number.isInteger(part.n) || part.n < 3 || part.n > 65535 || !Number.isInteger(part.m) || part.m < 1
        || !Array.isArray(part.lo) || !Array.isArray(part.hi) || part.lo.length !== 3 || part.hi.length !== 3
        || !part.lo.concat(part.hi).every(Number.isFinite) || part.lo.some((v, i) => v >= part.hi[i])
        || !Array.isArray(part.replaces) || !part.replaces.every(item => A.S[item.seg] && typeof item.name === 'string')) {
      throw new Error('Invalid spine disk surface manifest: ' + part.id);
    }
    ids.add(part.id); drivers.add(part.inferior.seg); drivers.add(part.superior.seg);
    expectedBytes += 8 * part.n + 6 * part.m;
  }
  if (!Number.isSafeInteger(expectedBytes) || expectedBytes > 64 * 1024 * 1024) throw new Error('Invalid spine disk payload budget');
  const packed = Uint8Array.from(atob(data.data), c => c.charCodeAt(0));
  const buffer = await new Response(new Blob([packed]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  if (buffer.byteLength !== expectedBytes) throw new Error('Spine disk payload length differs from its manifest');
  const group = new THREE.Group(); group.name = 'spine-disk-surfaces';
  const parts = [], pickables = [], covered = new Set(), excludedLegacy = new Set();
  const view = new DataView(buffer), lowerPoint = new THREE.Vector3(), upperPoint = new THREE.Vector3();
  const matrices = new Map([...drivers].map(seg => [seg, new THREE.Matrix4()]));
  const scale = new THREE.Vector3(1, 1, 1), localBox = new THREE.Box3();
  let cursor = 0;
  const u16 = () => { const value = view.getUint16(cursor, true); cursor += 2; return value; };
  try {
    for (const meta of data.parts) {
      const positions = new Float32Array(meta.n * 3), restPositions = new Float64Array(meta.n * 3), indices = new Uint16Array(meta.m * 3);
      const weights = new Float64Array(meta.n), acc = [0, 0, 0];
      for (let i = 0; i < restPositions.length; i++) {
        const axis = i % 3; acc[axis] = (acc[axis] + u16()) & 65535;
        restPositions[i] = meta.lo[axis] + (meta.hi[axis] - meta.lo[axis]) * acc[axis] / 65535;
        positions[i] = restPositions[i];
      }
      let previous = 0;
      for (let i = 0; i < indices.length; i++) {
        previous = (previous + u16()) & 65535;
        if (previous >= meta.n) throw new Error('Spine disk index is outside its part: ' + meta.id);
        indices[i] = previous;
      }
      for (let i = 0; i < weights.length; i++) weights[i] = u16() / 65535;
      const geometry = new THREE.BufferGeometry();
      const attribute = new THREE.BufferAttribute(positions, 3); attribute.setUsage(THREE.DynamicDrawUsage);
      geometry.setAttribute('position', attribute); geometry.setIndex(new THREE.BufferAttribute(indices, 1));
      geometry.computeVertexNormals(); geometry.computeBoundingBox(); geometry.computeBoundingSphere();
      const mat = opts.materialFactory ? opts.materialFactory(meta) : new THREE.MeshPhysicalMaterial({
        color: 0xb9d5ce, roughness: .44, metalness: 0, clearcoat: .18, clearcoatRoughness: .4,
        side: THREE.DoubleSide, emissive: 0xffc04a, emissiveIntensity: 0,
      });
      const mesh = new THREE.Mesh(geometry, mat); mesh.name = meta.id;
      mesh.userData = { kind: 'spinalDisc', id: meta.id, name: meta.name, latin: meta.latin,
        level: meta.level, source: meta.source, sourceIds: meta.sourceIds.slice(), levelAssignment: meta.levelAssignment };
      const part = { ...meta, kind: 'spinalDisc', mesh, mat, restPositions, weights, baseOpacity: mat.opacity };
      parts.push(part); pickables.push(mesh); group.add(mesh);
      for (const item of meta.replaces) covered.add(item.seg + '|' + item.name);
    }
    for (const item of data.excludedLegacy || []) {
      if (!A.S[item.seg] || typeof item.name !== 'string' || typeof item.reason !== 'string') throw new Error('Invalid excluded schematic disk');
      excludedLegacy.add(item.seg + '|' + item.name);
    }
  } catch (error) {
    for (const part of parts) { part.mesh.geometry.dispose(); part.mat.dispose(); }
    group.clear(); throw error;
  }
  function update(W) {
    // Preflight the entire frame first; no partial-body update on a bad input.
    for (const seg of drivers) {
      const w = W?.[seg], p = w?.p, q = w?.q;
      const lengthSq = q ? q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w : NaN;
      if (!p || !q || ![p.x, p.y, p.z, q.x, q.y, q.z, q.w].every(Number.isFinite)
          || !Number.isFinite(lengthSq) || lengthSq < 1e-12) throw new Error('Invalid spine disk transform: ' + seg);
    }
    for (const seg of drivers) matrices.get(seg).compose(W[seg].p, W[seg].q, scale);
    for (const part of parts) {
      const lower = matrices.get(part.inferior.seg), upper = matrices.get(part.superior.seg);
      const lowerJoint = A.S[part.inferior.seg].J, upperJoint = A.S[part.superior.seg].J;
      const attribute = part.mesh.geometry.attributes.position;
      for (let i = 0; i < part.n; i++) {
        lowerPoint.fromArray(part.restPositions, i * 3).sub(lowerJoint).applyMatrix4(lower);
        upperPoint.fromArray(part.restPositions, i * 3).sub(upperJoint).applyMatrix4(upper);
        lowerPoint.lerp(upperPoint, part.weights[i]);
        attribute.setXYZ(i, lowerPoint.x, lowerPoint.y, lowerPoint.z);
      }
      attribute.needsUpdate = true;
      // CPU geometry is also uploaded directly to the GPU. Picking, normals,
      // culling and focus bounds therefore use exactly the displayed shape.
      const g = part.mesh.geometry;
      g.computeVertexNormals(); g.computeBoundingBox(); g.computeBoundingSphere();
    }
    group.updateMatrixWorld(true);
  }
  function setVisibility(value) {
    for (const part of parts) part.mesh.visible = typeof value === 'function' ? !!value(part) : !!value;
  }
  function getBounds(id = null, visibleOnly = true) {
    const bounds = new THREE.Box3(); group.updateWorldMatrix(true, true);
    for (const part of parts) {
      if (id && part.id !== id && part.level !== id) continue;
      let visible = true;
      for (let object = part.mesh; object; object = object.parent) if (!object.visible) { visible = false; break; }
      if (!visibleOnly || visible) bounds.union(localBox.copy(part.mesh.geometry.boundingBox).applyMatrix4(part.mesh.matrixWorld));
    }
    return bounds.isEmpty() ? null : bounds;
  }
  function dispose() {
    for (const part of parts) { part.mesh.geometry.dispose(); part.mat.dispose(); }
    group.clear();
  }
  const root = A.SEG.find(seg => !seg.parent);
  update(A.computeWorld(A.STAND.local, A.STAND.rootQ, root.J.clone()));
  return { group, parts, pickables, covered, excludedLegacy, update, setVisibility, getBounds, dispose };
}
