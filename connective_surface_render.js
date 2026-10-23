// Display surfaces for the model's ligament paths and capsule attachment rings.
// These are geometric reconstructions, not segmented BodyParts3D tissue meshes or
// a constitutive tissue simulation. They never change attachments, path lengths,
// moments, or forces in createAnatomy / the static solver.
export function createConnectiveSurfaces(A, THREE) {
  const group = new THREE.Group(); group.name = 'connective-surfaces';
  const parts = [], pickables = [], covered = new Set();
  const W0 = A.computeWorld(A.STAND.local, A.STAND.rootQ, new THREE.Vector3());
  const V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);
  const keyOf = (kind, unit, side) => kind + '_' + unit + '_' + side;
  const N = 20, RAD = 8, CAP_SUB = 4;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const ALL = A.strandInstances().filter(s => s.kind === 'lig' || s.kind === 'cap');
  const groups = new Map();
  for (const inst of ALL) {
    const key = keyOf(inst.kind, inst.unit, inst.side);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(inst);
  }

  // A deterministic texture, aligned with the longitudinal collagen bundles.
  // Node-based geometry checks do not need a DOM or a texture.
  const fibers = (() => {
    if (typeof document === 'undefined') return null;
    const c = document.createElement('canvas'); c.width = 96; c.height = 128;
    const ctx = c.getContext('2d');
    if (!ctx) return null;
    ctx.fillStyle = '#f1e3ce'; ctx.fillRect(0, 0, c.width, c.height);
    for (let x = 0; x < c.width; x += 3) {
      const v = 202 + Math.round(15 * Math.sin(x * 1.7));
      ctx.strokeStyle = `rgb(${v + 12},${v + 6},${v})`;
      ctx.lineWidth = x % 9 === 0 ? 0.8 : 0.35;
      ctx.beginPath();
      for (let y = 0; y <= c.height; y += 4) {
        const xx = x + 0.65 * Math.sin(y * Math.PI / 32 + x * 0.3);
        if (!y) ctx.moveTo(xx, y); else ctx.lineTo(xx, y);
      }
      ctx.stroke();
    }
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  })();

  function resample(pts, out) {
    // Linear arc-length sampling keeps the rendered sheet on the same routed
    // paths. Spline overshoot could cut through a wrapping obstacle.
    const lengths = [0];
    for (let i = 1; i < pts.length; i++) lengths.push(lengths[i - 1] + pts[i].distanceTo(pts[i - 1]));
    const total = lengths[lengths.length - 1];
    let j = 1;
    for (let i = 0; i <= N; i++) {
      const d = total * i / N;
      while (j < lengths.length - 1 && lengths[j] < d) j++;
      const a = Math.max(0, j - 1), b = Math.min(j, pts.length - 1);
      const t = lengths[b] > lengths[a] ? (d - lengths[a]) / (lengths[b] - lengths[a]) : 0;
      out[i].copy(pts[a]).lerp(pts[b], t);
    }
    return total;
  }

  function geometry(count, uv, index) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(count * 3), 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uv), 2));
    g.setIndex(index);
    return g;
  }

  function addPart(insts, g, extra = {}) {
    const inst = insts[0], cap = inst.kind === 'cap';
    const mat = new THREE.MeshStandardMaterial({
      color: cap ? 0xe5dccb : 0xf0e3cf, map: fibers,
      roughness: cap ? 0.58 : 0.46, metalness: 0,
      side: THREE.DoubleSide, transparent: cap, opacity: cap ? 0.32 : 1,
      depthWrite: !cap, emissive: 0x000000, emissiveIntensity: 0,
    });
    const mesh = new THREE.Mesh(g, mat);
    mesh.name = keyOf(inst.kind, inst.unit, inst.side);
    mesh.userData = { kind: inst.kind, unit: inst.unit, side: inst.side };
    mesh.frustumCulled = false;
    if (cap) mesh.renderOrder = 1;
    const part = {
      kind: inst.kind, unit: inst.unit, side: inst.side, insts, mesh, mat,
      strain: 0, center: [], paths: insts.map(() => Array.from({ length: N + 1 }, () => V())),
      lengths: [], restLengths: insts.map(s => A.pathLength(A.strandPath(W0, s))),
      ...extra,
    };
    parts.push(part); pickables.push(mesh); group.add(mesh);
    covered.add(mesh.name);
    return part;
  }

  const BAND = new Set(['mcl', 'patTend', 'itb', 'plantar', 'coracoacr', 'nuchal', 'all', 'ssl', 'sacrotub', 'inguinal']);
  const CORD = new Set(['lcl', 'cfl', 'ptfl']);
  function ligament(insts) {
    const uv = [], index = [];
    for (let f = 0; f < insts.length; f++) {
      const offset = f * (N + 1) * RAD;
      for (let i = 0; i <= N; i++) for (let j = 0; j < RAD; j++) uv.push(j / RAD, i / N * 2);
      for (let i = 0; i < N; i++) for (let j = 0; j < RAD; j++) {
        const a = offset + i * RAD + j, b = offset + i * RAD + (j + 1) % RAD;
        const c = a + RAD, d = b + RAD;
        index.push(a, b, c, b, d, c);
      }
    }
    const widthAxes = insts.map(inst => {
      const pts = A.strandPath(W0, inst), tangent = pts[pts.length - 1].clone().sub(pts[0]).normalize();
      const anchor = W0[inst.pts[0].seg];
      // The traced points are world coordinates in W0, which need not place the
      // root at its rest-model J. Mixing them with A.S[seg].J changes band width
      // orientation when the body is translated, even before the first pose.
      const radial = pts[0].clone().lerp(pts[pts.length - 1], 0.5).sub(anchor.p);
      const width = V().crossVectors(radial, tangent);
      if (inst.unit === 'nuchal') width.set(0, 0, 1).applyQuaternion(anchor.q);
      if (width.lengthSq() < 1e-10) width.set(1, 0, 0).applyQuaternion(anchor.q);
      width.addScaledVector(tangent, -width.dot(tangent));
      if (width.lengthSq() < 1e-10) {
        width.set(0, 0, 1).applyQuaternion(anchor.q);
        width.addScaledVector(tangent, -width.dot(tangent));
      }
      // updateLigament rotates this stored direction with the origin segment.
      return width.normalize().applyQuaternion(anchor.q.clone().invert());
    });
    return addPart(insts, geometry(insts.length * (N + 1) * RAD, uv, index), { widthAxes });
  }

  function capsule(insts) {
    // Ring samples are already in angular order in anatomy3.capsule(). Preserve
    // the deliberately open anterior knee sector instead of sealing the patella.
    const centers = insts.map(s => A.strandPath(W0, s)[0]);
    const gaps = centers.map((p, i) => p.distanceTo(centers[(i + 1) % centers.length]));
    const sorted = [...gaps].sort((a, b) => a - b), median = sorted[Math.floor(sorted.length / 2)];
    const closed = gaps[gaps.length - 1] <= median * 1.6;
    const intervals = closed ? insts.length : insts.length - 1;
    const columns = intervals * CAP_SUB + 1, uv = [], index = [];
    for (let i = 0; i <= N; i++) for (let j = 0; j < columns; j++) uv.push(j / (columns - 1) * 3, i / N);
    for (let i = 0; i < N; i++) for (let j = 0; j < columns - 1; j++) {
      const a = i * columns + j, b = a + 1, c = a + columns, d = c + 1;
      index.push(a, b, c, b, d, c);
    }
    return addPart(insts, geometry((N + 1) * columns, uv, index), { closed, columns, intervals });
  }

  for (const [key, insts] of groups) {
    // Midline ligament paths are shared by L/R. Rendering both would stack two
    // coincident surfaces; retain coverage for both original sets of strands.
    if (insts[0].kind === 'lig' && insts[0].side === 'R') {
      const left = groups.get(keyOf('lig', insts[0].unit, 'L'));
      if (left && left.length === insts.length && left.every((s, k) => {
        const l = A.strandPath(W0, s), r = A.strandPath(W0, insts[k]);
        return l.length === r.length && l.every((p, j) => p.distanceToSquared(r[j]) < 1e-16);
      })) { covered.add(key); continue; }
    }
    if (insts[0].kind === 'cap') capsule(insts); else ligament(insts);
  }

  const tangent = V(), oldT = V(), width = V(), normal = V(), q = new THREE.Quaternion();
  const rowCenter = V(), tmp = V();
  function updateLigament(part, W) {
    const pos = part.mesh.geometry.attributes.position.array;
    for (let f = 0; f < part.insts.length; f++) {
      const inst = part.insts[f], path = part.paths[f];
      const broad = BAND.has(inst.unit), cord = CORD.has(inst.unit);
      const widthRatio = broad ? 2.1 : cord ? 1.05 : 1.55;
      const thickRatio = broad ? 0.24 : cord ? 0.70 : 0.34;
      const scale = clamp(Math.sqrt(part.restLengths[f] / Math.max(part.lengths[f], 1e-8)), 0.7, 1.2);
      width.copy(part.widthAxes[f]).applyQuaternion(W[inst.pts[0].seg].q);
      for (let i = 0; i <= N; i++) {
        tangent.subVectors(path[Math.min(N, i + 1)], path[Math.max(0, i - 1)]);
        if (tangent.lengthSq() < 1e-14) tangent.copy(i ? oldT : V(0, 1, 0));
        tangent.normalize();
        if (i) { q.setFromUnitVectors(oldT, tangent); width.applyQuaternion(q); }
        width.addScaledVector(tangent, -width.dot(tangent));
        if (width.lengthSq() < 1e-12) width.crossVectors(tangent, Math.abs(tangent.x) < 0.8 ? V(1, 0, 0) : V(0, 0, 1));
        width.normalize(); normal.crossVectors(tangent, width).normalize();
        const u = i / N;
        // Ligament insertions spread into the periosteum; the middle is narrower.
        const taper = 0.78 + 0.22 * Math.pow(Math.abs(2 * u - 1), 1.4);
        const rw = inst.r * widthRatio * taper * scale;
        const rt = inst.r * thickRatio * scale;
        for (let j = 0; j < RAD; j++) {
          const angle = j / RAD * Math.PI * 2;
          const k = (f * (N + 1) * RAD + i * RAD + j) * 3;
          const a = Math.cos(angle) * rw, b = Math.sin(angle) * rt;
          pos[k] = path[i].x + width.x * a + normal.x * b;
          pos[k + 1] = path[i].y + width.y * a + normal.y * b;
          pos[k + 2] = path[i].z + width.z * a + normal.z * b;
        }
        oldT.copy(tangent);
      }
    }
  }

  function updateCapsule(part) {
    const pos = part.mesh.geometry.attributes.position.array, count = part.paths.length;
    const sample = (k, i) => part.paths[part.closed ? (k + count) % count : clamp(k, 0, count - 1)][i];
    for (let i = 0; i <= N; i++) {
      rowCenter.set(0, 0, 0);
      for (const p of part.paths) rowCenter.add(p[i]);
      rowCenter.divideScalar(count);
      const u = i / N, envelope = Math.pow(Math.sin(Math.PI * u), 2);
      for (let j = 0; j < part.columns; j++) {
        const x = j / CAP_SUB, f = Math.min(part.intervals - 1, Math.floor(x));
        const t = x - f, t2 = t * t, t3 = t2 * t;
        const a = sample(f - 1, i), b = sample(f, i), c = sample(f + 1, i), d = sample(f + 2, i);
        // Smooth around the joint, while passing exactly through each existing
        // fibre and its attachment at the two rims.
        tmp.copy(b).multiplyScalar(2).addScaledVector(a, -t + 2 * t2 - t3)
          .addScaledVector(b, -5 * t2 + 3 * t3).addScaledVector(c, t + 4 * t2 - 3 * t3)
          .addScaledVector(d, -t2 + t3).multiplyScalar(0.5);
        const k0 = f % count, k1 = (f + 1) % count;
        const l0 = part.restLengths[k0] * (1 - t) + part.restLengths[k1] * t;
        const l = part.lengths[k0] * (1 - t) + part.lengths[k1] * t;
        const slack = clamp((l0 - l) * 0.08, 0, 0.0015);
        normal.subVectors(tmp, rowCenter);
        const phase = part.closed ? Math.PI * 2 * x / count : x * 0.65;
        if (normal.lengthSq() > 1e-14) tmp.addScaledVector(normal.normalize(), slack * envelope * Math.sin(u * Math.PI * 12 + phase));
        const k = (i * part.columns + j) * 3;
        pos[k] = tmp.x; pos[k + 1] = tmp.y; pos[k + 2] = tmp.z;
      }
    }
  }

  function update(W, pathCache) {
    for (const part of parts) {
      part.strain = -Infinity; part.center.length = 0;
      for (let f = 0; f < part.insts.length; f++) {
        const inst = part.insts[f];
        const pts = pathCache && pathCache.get(inst.id) || A.strandPath(W, inst);
        part.lengths[f] = resample(pts, part.paths[f]);
        part.strain = Math.max(part.strain, part.lengths[f] / part.restLengths[f] - 1);
        part.center.push(...part.paths[f]);
      }
      if (part.kind === 'lig') updateLigament(part, W); else updateCapsule(part);
      const g = part.mesh.geometry;
      g.attributes.position.needsUpdate = true;
      g.computeVertexNormals();
      if (part.kind === 'cap' && part.closed) {
        // The UV seam has duplicate vertices. Share their normals so a continuous
        // capsule does not acquire a visible longitudinal lighting seam.
        const normals = g.attributes.normal.array;
        for (let i = 0; i <= N; i++) {
          const a = i * part.columns * 3, b = (i * part.columns + part.columns - 1) * 3;
          normal.set(normals[a] + normals[b], normals[a + 1] + normals[b + 1], normals[a + 2] + normals[b + 2]).normalize();
          normals[a] = normals[b] = normal.x;
          normals[a + 1] = normals[b + 1] = normal.y;
          normals[a + 2] = normals[b + 2] = normal.z;
        }
        g.attributes.normal.needsUpdate = true;
      }
      g.computeBoundingBox(); g.computeBoundingSphere();
    }
  }

  function setVisibility(fn) {
    for (const part of parts) part.mesh.visible = !!fn(part);
  }
  function setAppearance(fn) {
    for (const part of parts) {
      const v = fn(part);
      if (!v) continue;
      if (v.color !== undefined) part.mat.color.set(v.color);
      if (v.emissive !== undefined) part.mat.emissive.set(v.emissive);
      if (v.emissiveIntensity !== undefined) part.mat.emissiveIntensity = v.emissiveIntensity;
      if (v.opacity !== undefined) {
        const opacity = clamp(v.opacity, 0, 1), transparent = part.kind === 'cap' || opacity < 0.999;
        if (part.mat.transparent !== transparent) { part.mat.transparent = transparent; part.mat.needsUpdate = true; }
        part.mat.opacity = opacity; part.mat.depthWrite = part.kind !== 'cap' && !transparent;
      }
    }
  }
  function getBounds(kind, unit, side) {
    const box = new THREE.Box3();
    for (const part of parts) if (part.kind === kind && part.unit === unit && (!side || part.side === side)) {
      box.union(part.mesh.geometry.boundingBox);
    }
    return box.isEmpty() ? null : box;
  }
  function dispose() {
    for (const part of parts) { part.mesh.geometry.dispose(); part.mat.dispose(); }
    if (fibers) fibers.dispose();
    group.clear();
  }
  update(W0);
  return { group, parts, covered, pickables, update, setVisibility, setAppearance, getBounds, dispose };
}
