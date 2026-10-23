// Joint display meshes fitted to the unchanged anatomical bones at build time.
// Packed positions are rest-world metres. Each part follows its named bone
// rigidly; these surfaces do not compute cartilage compression, meniscal motion,
// contact pressure, or any force used by the model's static optimisation.
export async function createJointSurfaces(A, THREE, data, opts = {}) {
  if (!data || data.v !== 1 || !Array.isArray(data.parts) || !data.parts.length) throw new Error('Unsupported joint surface package');
  if (typeof DecompressionStream === 'undefined') throw new Error('Joint surfaces require gzip decompression');
  const ids = new Set(), joints = new Set(A.JOINTS.map(j => j.id));
  let expectedBytes = 0;
  for (const p of data.parts) {
    if (!p.id || ids.has(p.id) || !A.S[p.seg] || !joints.has(p.joint)) throw new Error('Invalid joint surface identity: ' + p.id);
    ids.add(p.id);
    if (!Number.isInteger(p.n) || p.n < 3 || p.n > 65535 || !Number.isInteger(p.m) || p.m < 1) throw new Error('Invalid joint surface counts: ' + p.id);
    if (!Array.isArray(p.lo) || !Array.isArray(p.hi) || p.lo.length !== 3 || p.hi.length !== 3 || !p.lo.concat(p.hi).every(Number.isFinite) || p.lo.some((x, a) => x > p.hi[a])) throw new Error('Invalid joint surface bounds: ' + p.id);
    if (!p.name || !p.type || !p.source || !Array.isArray(p.replaces) || !p.replaces.every(name => typeof name === 'string')) throw new Error('Missing joint surface provenance: ' + p.id);
    expectedBytes += (p.n + p.m) * 6;
  }
  const packed = Uint8Array.from(atob(data.data), c => c.charCodeAt(0));
  const raw = await new Response(new Blob([packed]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  if (raw.byteLength !== expectedBytes) throw new Error('Joint surface payload size does not match its manifest');

  const group = new THREE.Group(); group.name = 'joint-surfaces';
  const parts = [], pickables = [], covered = new Set();
  const scale = new THREE.Vector3(1, 1, 1), box = new THREE.Box3();
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const defaultMaterial = () => new THREE.MeshPhysicalMaterial({
    color: 0xcbd8d8, roughness: 0.36, metalness: 0,
    clearcoat: 0.35, clearcoatRoughness: 0.3,
    side: THREE.DoubleSide, transparent: false, opacity: 1,
    emissive: 0xffc04a, emissiveIntensity: 0,
  });
  let offset = 0;
  try {
    for (const meta of data.parts) {
      const dq = new Uint16Array(raw, offset, meta.n * 3); offset += meta.n * 6;
      const di = new Uint16Array(raw, offset, meta.m * 3); offset += meta.m * 6;
      const positions = new Float32Array(meta.n * 3), indices = new Uint16Array(meta.m * 3);
      const acc = [0, 0, 0], J = A.S[meta.seg].J.toArray();
      for (let i = 0; i < meta.n; i++) for (let a = 0; a < 3; a++) {
        acc[a] = (acc[a] + dq[i * 3 + a]) & 65535;
        positions[i * 3 + a] = meta.lo[a] + (meta.hi[a] - meta.lo[a]) * acc[a] / 65535 - J[a];
      }
      let index = 0;
      for (let k = 0; k < indices.length; k++) {
        index = (index + di[k]) & 65535;
        if (index >= meta.n) throw new Error('Joint surface index is outside its part: ' + meta.id);
        indices[k] = index;
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geometry.setIndex(new THREE.BufferAttribute(indices, 1));
      geometry.computeVertexNormals(); geometry.computeBoundingBox(); geometry.computeBoundingSphere();
      const mat = opts.materialFactory ? opts.materialFactory(meta) : defaultMaterial();
      const mesh = new THREE.Mesh(geometry, mat);
      mesh.name = meta.id; mesh.matrixAutoUpdate = false;
      mesh.userData = {
        kind: 'cart', id: meta.id, name: meta.name, latin: meta.latin || '',
        joint: meta.joint, jointid: meta.joint, seg: meta.seg, side: meta.side || '',
        type: meta.type, source: meta.source, method: meta.method || '',
        boneRefs: meta.boneRefs || [],
      };
      const part = { ...meta, key: meta.id, kind: 'cart', mesh, mat, baseOpacity: mat.opacity };
      parts.push(part); pickables.push(mesh); group.add(mesh);
      for (const name of meta.replaces) covered.add(meta.seg + '|' + name);
    }
  } catch (error) {
    for (const part of parts) { part.mesh.geometry.dispose(); part.mat.dispose(); }
    throw error;
  }

  function update(W) {
    // Only one rigid matrix per part changes. Static vertex/normal buffers and
    // their local bounds remain untouched throughout poses and the round.
    for (const part of parts) {
      const w = W[part.seg];
      part.mesh.matrix.compose(w.p, w.q, scale);
      part.mesh.matrixWorldNeedsUpdate = true;
    }
    group.updateMatrixWorld(true);
  }
  function setVisibility(value) {
    for (const part of parts) part.mesh.visible = typeof value === 'function' ? !!value(part) : !!value;
  }
  function setAppearance(fn) {
    for (const part of parts) {
      const style = fn(part);
      if (!style) continue;
      if (style.color !== undefined) part.mat.color.set(style.color);
      if (style.emissive !== undefined) part.mat.emissive.set(style.emissive);
      if (style.emissiveIntensity !== undefined) part.mat.emissiveIntensity = style.emissiveIntensity;
      if (style.opacity !== undefined) {
        const opacity = clamp(style.opacity, 0, 1), transparent = opacity < 0.999;
        if (part.mat.transparent !== transparent) { part.mat.transparent = transparent; part.mat.needsUpdate = true; }
        part.mat.opacity = opacity; part.mat.depthWrite = !transparent;
      }
    }
  }
  function getBounds(joint = null, visibleOnly = true) {
    const bounds = new THREE.Box3();
    group.updateWorldMatrix(true, true);
    for (const part of parts) if ((!joint || part.joint === joint) && (!visibleOnly || part.mesh.visible)) {
      bounds.union(box.copy(part.mesh.geometry.boundingBox).applyMatrix4(part.mesh.matrixWorld));
    }
    return bounds.isEmpty() ? null : bounds;
  }
  function dispose() {
    for (const part of parts) { part.mesh.geometry.dispose(); part.mat.dispose(); }
    group.clear();
  }
  const root = A.SEG.find(s => !s.parent);
  update(A.computeWorld(A.STAND.local, A.STAND.rootQ, root.J.clone()));
  return { group, parts, pickables, covered, update, setVisibility, setAppearance, getBounds, dispose };
}
