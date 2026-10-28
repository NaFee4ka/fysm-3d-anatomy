// Display surfaces for the model's ligament paths and capsule attachment rings.
// These are geometric reconstructions, not segmented BodyParts3D tissue meshes or
// a constitutive tissue simulation. They never change attachments, path lengths,
// moments, or forces in createAnatomy / the static solver.
export function resampleRoutedPath(points, out) {
  // Uniform arc-length samples can omit an original corner, joining two
  // neighbouring route segments by a chord through a wrapping obstacle.
  // Reserve one interval per original segment, then apportion the spare
  // intervals by length. Every routed corner and both attachments survive.
  if (!Array.isArray(points) || !points.length || !Array.isArray(out) || out.length < 2
      || !points.every(p => p && [p.x, p.y, p.z].every(Number.isFinite)))
    throw new Error('Invalid routed connective path');
  const unique = [points[0]];
  for (let i = 1; i < points.length; i++)
    if (points[i].distanceToSquared(unique.at(-1)) > 0) unique.push(points[i]);
  if (unique.length === 1) { for (const p of out) p.copy(unique[0]); return 0; }
  const lengths = unique.slice(1).map((p, i) => p.distanceTo(unique[i]));
  const total = lengths.reduce((sum, length) => sum + length, 0);
  const budget = out.length - 1, count = lengths.length;
  if (count > budget) throw new Error('Connective path has more routed segments than display intervals');
  const spare = budget - count;
  const quotas = lengths.map(length => spare * length / total);
  const intervals = quotas.map(quota => 1 + Math.floor(quota));
  const remainder = budget - intervals.reduce((sum, value) => sum + value, 0);
  const order = quotas.map((quota, i) => ({ i, fraction: quota - Math.floor(quota) }))
    .sort((a, b) => b.fraction - a.fraction || a.i - b.i);
  for (let i = 0; i < remainder; i++) intervals[order[i].i]++;
  let index = 0;
  out[index++].copy(unique[0]);
  for (let segment = 0; segment < count; segment++) {
    const n = intervals[segment];
    for (let step = 1; step <= n; step++, index++) {
      if (step === n) out[index].copy(unique[segment + 1]);
      else out[index].copy(unique[segment]).lerp(unique[segment + 1], step / n);
    }
  }
  return total;
}

export function createConnectiveSurfaces(A, THREE, opts = {}) {
  const group = new THREE.Group(); group.name = 'connective-surfaces';
  const parts = [], pickables = [], covered = new Set();
  const W0 = A.computeWorld(A.STAND.local, A.STAND.rootQ, new THREE.Vector3());
  const V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);
  const keyOf = (kind, unit, side) => kind + '_' + unit + '_' + side;
  const N = 24, RAD = 12, CAP_SUB = 4, FAN_RAD = 24;
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
    const c = document.createElement('canvas'); c.width = 128; c.height = 256;
    const ctx = c.getContext('2d');
    if (!ctx) return null;
    ctx.fillStyle = '#eee5d6'; ctx.fillRect(0, 0, c.width, c.height);
    for (let x = 0; x < c.width; x += 4) {
      const v = 220 + Math.round(4 * Math.sin(x * 1.7));
      ctx.strokeStyle = `rgb(${v + 8},${v + 4},${v})`;
      ctx.lineWidth = x % 12 === 0 ? 0.65 : 0.35;
      ctx.beginPath();
      for (let y = 0; y <= c.height; y += 4) {
        const xx = x + 0.45 * Math.sin(y * Math.PI / 64 + x * 0.3);
        if (!y) ctx.moveTo(xx, y); else ctx.lineTo(xx, y);
      }
      ctx.stroke();
    }
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.minFilter = THREE.LinearMipmapLinearFilter;
    tex.magFilter = THREE.LinearFilter;
    tex.anisotropy = 4;
    return tex;
  })();

  function resample(pts, out) {
    return resampleRoutedPath(pts, out);
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
      roughness: cap ? 0.66 : 0.56, metalness: 0,
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
  const FAN = new Set(['iliofem', 'deltoidLig', 'plantar']);
  function ligament(insts) {
    const uv = [], index = [];
    const stride = (N + 1) * RAD + 2, attachmentVertices = [];
    for (let f = 0; f < insts.length; f++) {
      const offset = f * stride;
      for (let i = 0; i <= N; i++) for (let j = 0; j < RAD; j++) uv.push(j / RAD, i / N * 2);
      for (let i = 0; i < N; i++) for (let j = 0; j < RAD; j++) {
        const a = offset + i * RAD + j, b = offset + i * RAD + (j + 1) % RAD;
        const c = a + RAD, d = b + RAD;
        index.push(a, b, c, b, d, c);
      }
      const a = offset + (N + 1) * RAD, b = a + 1;
      uv.push(0.5, 0, 0.5, 2);
      for (let j = 0; j < RAD; j++) {
        const next = (j + 1) % RAD;
        index.push(a, offset + next, offset + j);
        index.push(b, offset + N * RAD + j, offset + N * RAD + next);
      }
      attachmentVertices.push({ inst: f, end: 0, vertex: a }, { inst: f, end: N, vertex: b });
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
    return addPart(insts, geometry(insts.length * stride, uv, index), {
      widthAxes, bandStride: stride, attachmentVertices,
      morphology: CORD.has(insts[0].unit) ? 'cord' : 'band', closedEnds: true,
    });
  }

  function ligamentFan(insts) {
    const uv = [], index = [], around = FAN_RAD, half = around / 2;
    const mid = [[], []];
    for (let i = 0; i <= N; i++) for (let j = 0; j < around; j++) uv.push(j / around, i / N * 2);
    for (let i = 0; i < N; i++) for (let j = 0; j < around; j++) {
      const a = i * around + j, b = i * around + (j + 1) % around;
      const c = a + around, d = b + around;
      index.push(a, c, b, b, c, d);
    }
    for (const end of [0, 1]) {
      const row = end ? N : 0, offset = row * around;
      for (let j = 0; j <= half; j++) {
        if (!j || j === half) mid[end].push(offset + j);
        else { mid[end].push(uv.length / 2); uv.push(j / half, end * 2); }
      }
      const face = (a, b, c) => {
        if (a === b || b === c || a === c) return;
        if (!end) index.push(a, b, c); else index.push(a, c, b);
      };
      for (let j = 0; j < half; j++) {
        const a = offset + j, b = offset + j + 1;
        const c = offset + (around - j) % around, d = offset + (around - j - 1) % around;
        face(mid[end][j], a, b); face(mid[end][j], b, mid[end][j + 1]);
        face(mid[end][j], mid[end][j + 1], c); face(mid[end][j + 1], d, c);
      }
    }
    const axis = insts[0].pts[0].seg;
    const a = A.strandPath(W0, insts[0]), b = A.strandPath(W0, insts.at(-1));
    const t = a.at(-1).clone().sub(a[0]).normalize();
    const width = b[Math.floor(b.length / 2)].clone().sub(a[Math.floor(a.length / 2)]);
    width.addScaledVector(t, -width.dot(t));
    if (width.lengthSq() < 1e-12) width.set(1, 0, 0).applyQuaternion(W0[axis].q);
    width.normalize().applyQuaternion(W0[axis].q.clone().invert());
    const attachmentVertices = [];
    for (const end of [0, 1]) {
      const row = end ? N : 0, points = insts.map(s => { const p = A.strandPath(W0, s); return end ? p.at(-1) : p[0]; });
      const common = points.every(p => p.distanceToSquared(points[0]) < 1e-16);
      for (let f = 0; f < insts.length; f++) {
        const j = common ? half / 2 : Math.round(Math.acos(1 - 2 * f / (insts.length - 1)) / Math.PI * half);
        attachmentVertices.push({ inst: f, end: row, vertex: mid[end][j] });
      }
    }
    return addPart(insts, geometry(uv.length / 2, uv, index), {
      morphology: insts[0].unit === 'iliofem' ? 'bifid-fan' : 'fan', closedEnds: true,
      fanMid: mid, fanAxis: width, fanAxisSeg: axis, attachmentVertices,
      fanCenters: Array.from({ length: N + 1 }, () => Array.from({ length: half + 1 }, () => V())),
    });
  }

  function capsule(insts) {
    // Ring samples are already in angular order in anatomy3.capsule(). Preserve
    // its open knee sector. In the inherited coordinates that gap is medial,
    // not an anatomical anterior patellar window; do not relabel or rotate the
    // physical attachment data to conceal that limitation.
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
    const sourceWrap = insts[0].wraps[0];
    const fitted = capsuleFit(insts, columns);
    return addPart(insts, geometry((N + 1) * columns, uv, index), {
      closed, columns, intervals, sourceWrap, wrap: fitted.wrap || sourceWrap, morphology: 'capsule',
      sourceOpening: closed ? null : 'inherited medial sector',
      displayFit: fitted.valid ? 'bone-rim' : 'source-fallback', displayFitReason: fitted.reason,
      displayInsts: fitted.insts || insts, displayRims: fitted.rims || null,
      displayPaths: insts.map(() => Array.from({ length: N + 1 }, () => V())),
      displayLengths: [],
      displayColumnInsts: fitted.columnInsts || null,
      displayColumnPaths: fitted.columnInsts ? Array.from({ length: columns }, () => Array.from({ length: N + 1 }, () => V())) : null,
      displayColumnLengths: [],
    });
  }

  function capsuleFit(insts, columns) {
    const source = opts && opts.capsuleRims && opts.capsuleRims[insts[0].unit + '_' + insts[0].side];
    if (!source) return { valid: false, reason: 'Bone-fitted display rims unavailable' };
    const finitePoint = p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
    const rimValid = r => r && A.S[r.seg] && Array.isArray(r.positions)
      && r.positions.length === insts.length && r.positions.every(finitePoint);
    if (!rimValid(source.prox) || !rimValid(source.dist))
      return { valid: false, reason: 'Invalid bone-fitted display rim coordinates' };
    if (!['prox', 'dist'].every(end => Array.isArray(source[end].densePositions)
      && source[end].densePositions.length === columns && source[end].densePositions.every(finitePoint)))
      return { valid: false, reason: 'Dense bone-fitted display rims unavailable or invalid' };
    if (source.sourceIds && (!Array.isArray(source.sourceIds) || source.sourceIds.length !== insts.length
      || !source.sourceIds.every((id, i) => id === insts[i].id)))
      return { valid: false, reason: 'Bone-fitted display rim source order differs' };
    const sw = source.wrap;
    if (sw && (!A.S[sw.seg] || !finitePoint(sw.c) || !finitePoint(sw.r)
      || sw.r.some(r => r < 0.002 || r > 0.15)))
      return { valid: false, reason: 'Invalid bone-fitted display wrapping envelope' };
    const wrap = sw ? { seg: sw.seg, c: V(...sw.c), r: V(...sw.r), depth: 2 } : insts[0].wraps[0];
    const rims = Object.fromEntries(['prox', 'dist'].map(end => [end, {
      seg: source[end].seg, positions: source[end].positions.map(p => V(...p)),
      densePositions: source[end].densePositions.map(p => V(...p)),
    }]));
    const displayInsts = insts.map((inst, f) => ({ ...inst,
      pts: [{ seg: rims.prox.seg, p: rims.prox.positions[f] }, { seg: rims.dist.seg, p: rims.dist.positions[f] }],
      wraps: [wrap],
    }));
    const columnInsts = Array.from({ length: columns }, (_, j) => ({ ...insts[Math.min(insts.length - 1, Math.floor(j / CAP_SUB))],
      pts: [{ seg: rims.prox.seg, p: rims.prox.densePositions[j] }, { seg: rims.dist.seg, p: rims.dist.densePositions[j] }],
      wraps: [wrap],
    }));
    return { valid: true, reason: 'Reconstructed display rims fitted to bounded ready-bone regions',
      insts: displayInsts, columnInsts, rims, wrap };
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
    if (insts[0].kind === 'cap') capsule(insts);
    else if (FAN.has(insts[0].unit)) ligamentFan(insts);
    else ligament(insts);
  }

  const tangent = V(), oldT = V(), width = V(), normal = V(), q = new THREE.Quaternion();
  const bendA = V(), bendB = V();
  const tmp = V(), an = V(), bn = V(), direction = V(), local = V(), recessDir = V();
  const wrapFrames = new Map(), wrapMatrix = new THREE.Matrix4();
  const arcQ = new THREE.Quaternion(), blendQ = new THREE.Quaternion();
  function normalizedPoint(p, wrap, W, out) {
    const w = W[wrap.seg];
    let inverse = wrapFrames.get(wrap.seg);
    if (!inverse) {
      // Saved cycle quaternions are rounded. Conjugation is an exact inverse
      // only for a unit quaternion; invert the same linear transform used by
      // Vector3.applyQuaternion without altering W or the numerical model.
      inverse = new THREE.Matrix3().setFromMatrix4(wrapMatrix.makeRotationFromQuaternion(w.q)).invert();
      wrapFrames.set(wrap.seg, inverse);
    }
    return out.copy(p).sub(w.p).applyMatrix3(inverse).add(A.S[wrap.seg].J).sub(wrap.c).divide(wrap.r);
  }
  function fromNormalized(p, wrap, W, out) {
    const w = W[wrap.seg];
    return out.copy(p).multiply(wrap.r).add(wrap.c).sub(A.S[wrap.seg].J).applyQuaternion(w.q).add(w.p);
  }
  function angularPoint(a, b, t, wrap, W, out) {
    if (t <= 1e-12) return out.copy(a);
    if (t >= 1 - 1e-12) return out.copy(b);
    normalizedPoint(a, wrap, W, an); normalizedPoint(b, wrap, W, bn);
    const ra = an.length(), rb = bn.length();
    if (ra < 1e-9 || rb < 1e-9) return out.copy(a).lerp(b, t);
    an.divideScalar(ra); bn.divideScalar(rb);
    // Interpolate on the wrapping ellipsoid, not through it. setFromUnitVectors
    // uses a deterministic joint-local axis even for antipodal directions; it
    // therefore stays covariant when the whole body rotates in world space.
    arcQ.setFromUnitVectors(an, bn);
    blendQ.identity().slerp(arcQ, t);
    direction.copy(an).applyQuaternion(blendQ).multiplyScalar(ra + (rb - ra) * t);
    return fromNormalized(direction, wrap, W, out);
  }
  function updateLigament(part, W) {
    const pos = part.mesh.geometry.attributes.position.array;
    for (let f = 0; f < part.insts.length; f++) {
      const inst = part.insts[f], path = part.paths[f];
      const broad = BAND.has(inst.unit), cord = CORD.has(inst.unit);
      const widthRatio = broad ? 2.1 : cord ? 1.05 : 1.55;
      const thickRatio = broad ? 0.24 : cord ? 0.70 : 0.34;
      const scale = clamp(Math.sqrt(part.restLengths[f] / Math.max(part.lengths[f], 1e-8)), 0.7, 1.2);
      // A coarse physical route can make a sharp corner at an obstacle or an
      // insertion. A fixed-radius sweep then intersects itself. Restrict only
      // the displayed section near that corner; the routed centreline and the
      // mechanics stay unchanged.
      const clearance = new Array(N + 1).fill(Infinity);
      for (let corner = 1; corner < N; corner++) {
        bendA.subVectors(path[corner], path[corner - 1]);
        bendB.subVectors(path[corner + 1], path[corner]);
        const la = bendA.length(), lb = bendB.length();
        if (la < 1e-9 || lb < 1e-9) continue;
        const turn = Math.acos(clamp(bendA.dot(bendB) / (la * lb), -1, 1));
        if (turn < 0.5) continue;
        const radius = Math.min(la, lb) * 0.5 / Math.tan(turn * 0.5);
        for (let i = 0; i <= N; i++) clearance[i] = Math.min(clearance[i],
          Math.max(inst.r * 0.08, radius * 0.25 + path[i].distanceTo(path[corner]) * 0.30));
      }
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
        const rw = Math.min(inst.r * widthRatio * taper * scale, clearance[i]);
        const rt = Math.min(inst.r * thickRatio * scale, clearance[i]);
        for (let j = 0; j < RAD; j++) {
          const angle = j / RAD * Math.PI * 2;
          const k = (f * part.bandStride + i * RAD + j) * 3;
          const a = Math.cos(angle) * rw, b = Math.sin(angle) * rt;
          pos[k] = path[i].x + width.x * a + normal.x * b;
          pos[k + 1] = path[i].y + width.y * a + normal.y * b;
          pos[k + 2] = path[i].z + width.z * a + normal.z * b;
        }
        oldT.copy(tangent);
      }
      const a = (f * part.bandStride + (N + 1) * RAD) * 3, b = a + 3;
      path[0].toArray(pos, a); path[N].toArray(pos, b);
    }
  }

  function acrossPaths(part, row, v, W, out) {
    const f = v * (part.paths.length - 1), a = Math.min(part.paths.length - 2, Math.floor(f)), t = f - a;
    const p = part.paths[a][row], q = part.paths[a + 1][row];
    const wrap = part.insts[0].wraps[0];
    return wrap ? angularPoint(p, q, t, wrap, W, out) : out.copy(p).lerp(q, t);
  }
  function updateFan(part, W) {
    const pos = part.mesh.geometry.attributes.position.array, half = FAN_RAD / 2;
    const radius = part.insts.reduce((s, x) => s + x.r, 0) / part.insts.length;
    const scale = clamp(Math.sqrt(part.restLengths.reduce((a, b) => a + b, 0) / part.lengths.reduce((a, b) => a + b, 0)), 0.75, 1.15);
    const at = (i, j) => part.fanCenters[clamp(i, 0, N)][clamp(j, 0, half)];
    for (let i = 0; i <= N; i++) {
      tangent.subVectors(part.paths[0][Math.min(N, i + 1)], part.paths[0][Math.max(0, i - 1)]).normalize();
      width.subVectors(part.paths.at(-1)[i], part.paths[0][i]);
      const span = width.length();
      width.addScaledVector(tangent, -width.dot(tangent));
      if (width.lengthSq() < 1e-12) width.copy(part.fanAxis).applyQuaternion(W[part.fanAxisSeg].q).addScaledVector(tangent, -width.dot(tangent));
      width.normalize();
      // A common proximal origin has a finite attachment footprint instead of a
      // singular zero-width mesh. Its midpoint stays on the original anchor.
      const margin = Math.max(0, radius * 1.5 - span) * 0.5;
      for (let j = 0; j <= half; j++) {
        const v = (1 - Math.cos(j / half * Math.PI)) * 0.5;
        acrossPaths(part, i, v, W, part.fanCenters[i][j]).addScaledVector(width, (2 * v - 1) * margin);
      }
    }
    for (let i = 0; i <= N; i++) for (let j = 0; j < FAN_RAD; j++) {
      const k = j <= half ? j : FAN_RAD - j, v = (1 - Math.cos(k / half * Math.PI)) * 0.5;
      tangent.subVectors(at(i + 2, k), at(i - 2, k));
      width.subVectors(at(i, k + 2), at(i, k - 2));
      normal.crossVectors(tangent, width);
      if (normal.lengthSq() < 1e-14) {
        width.copy(part.fanAxis).applyQuaternion(W[part.fanAxisSeg].q);
        normal.crossVectors(tangent, width);
      }
      normal.normalize();
      const u = i / N, attach = 0.24 + 0.76 * Math.pow(Math.sin(Math.PI * u), 0.55);
      let thickness = radius * (part.unit === 'plantar' ? 0.20 : 0.34) * attach * scale;
      if (part.unit === 'iliofem') {
        const bifurcation = clamp((u - 0.20) / 0.55, 0, 1);
        thickness *= 1 - 0.65 * bifurcation * Math.pow(Math.sin(Math.PI * v), 2);
      }
      const offset = thickness * Math.sin(j / FAN_RAD * Math.PI * 2);
      tmp.copy(at(i, k)).addScaledVector(normal, offset).toArray(pos, (i * FAN_RAD + j) * 3);
    }
    for (const end of [0, 1]) for (let j = 1; j < half; j++) {
      const row = end ? N : 0, a = (row * FAN_RAD + j) * 3, b = (row * FAN_RAD + FAN_RAD - j) * 3;
      const k = part.fanMid[end][j] * 3;
      pos[k] = (pos[a] + pos[b]) * 0.5; pos[k + 1] = (pos[a + 1] + pos[b + 1]) * 0.5; pos[k + 2] = (pos[a + 2] + pos[b + 2]) * 0.5;
    }
  }

  function updateCapsule(part, W) {
    const pos = part.mesh.geometry.attributes.position.array, count = part.displayPaths.length;
    const sample = (k, i) => part.displayPaths[part.closed ? (k + count) % count : clamp(k, 0, count - 1)][i];
    let armDown = 0;
    if (part.unit === 'gh') {
      const proximal = part.insts[0].pts[0].seg;
      recessDir.set(0, -1, 0).applyQuaternion(W[proximal].q);
      local.subVectors(W['fore_' + part.side].p, W['hum_' + part.side].p).normalize();
      armDown = clamp((local.dot(recessDir) + 0.2) / 1.2, 0, 1);
    }
    for (let i = 0; i <= N; i++) {
      const u = i / N, envelope = Math.pow(Math.sin(Math.PI * u), 2);
      for (let j = 0; j < part.columns; j++) {
        const x = j / CAP_SUB, f = Math.min(part.intervals - 1, Math.floor(x));
        const t = x - f;
        const b = sample(f, i), c = sample(f + 1, i);
        if (part.displayColumnPaths) tmp.copy(part.displayColumnPaths[j][i]);
        else angularPoint(b, c, t, part.wrap, W, tmp);
        const k0 = f % count, k1 = (f + 1) % count;
        const l0 = part.restLengths[k0] * (1 - t) + part.restLengths[k1] * t;
        const l = part.lengths[k0] * (1 - t) + part.lengths[k1] * t;
        const slack = clamp((l0 - l) * 0.06, 0, 0.0012);
        normalizedPoint(tmp, part.wrap, W, local);
        const w = W[part.wrap.seg];
        // Outward folds stay between the original source fibres. They do not
        // drag the interpolated membrane deeper into the joint or move a knot.
        normal.copy(local).multiply(part.wrap.r).applyQuaternion(w.q).normalize();
        const phase = part.closed ? Math.PI * 2 * x / count : x * 0.65;
        const between = Math.pow(Math.sin(Math.PI * t), 2);
        let fold = slack * envelope * between * (0.5 + 0.5 * Math.sin(u * Math.PI * 8 + phase));
        if (part.unit === 'gh') {
          // A small inferior reserve pocket in adduction opens out as the arm
          // rises. This is a display estimate between existing reinforcement
          // fibres, not a new attachment or a capsular force calculation.
          const inferior = clamp((normal.dot(recessDir) - 0.15) / 0.85, 0, 1);
          fold += 0.004 * armDown * inferior * inferior * envelope * between;
        }
        tmp.addScaledVector(normal, fold);
        const k = (i * part.columns + j) * 3;
        pos[k] = tmp.x; pos[k + 1] = tmp.y; pos[k + 2] = tmp.z;
      }
    }
  }

  function update(W, pathCache) {
    wrapFrames.clear();
    for (const part of parts) {
      part.strain = -Infinity; part.center.length = 0;
      for (let f = 0; f < part.insts.length; f++) {
        const inst = part.insts[f];
        const pts = pathCache && pathCache.get(inst.id) || A.strandPath(W, inst);
        part.lengths[f] = resample(pts, part.paths[f]);
        part.strain = Math.max(part.strain, part.lengths[f] / part.restLengths[f] - 1);
        part.center.push(...part.paths[f]);
      }
      if (part.kind === 'cap') {
        if (part.displayColumnPaths) for (let j = 0; j < part.columns; j++)
          part.displayColumnLengths[j] = resample(A.strandPath(W, part.displayColumnInsts[j]), part.displayColumnPaths[j]);
        for (let f = 0; f < part.insts.length; f++) {
          if (part.displayColumnPaths) {
            part.displayColumnPaths[f * CAP_SUB].forEach((p, i) => part.displayPaths[f][i].copy(p));
            part.displayLengths[f] = part.displayColumnLengths[f * CAP_SUB];
          }
          else if (part.displayFit === 'bone-rim')
            part.displayLengths[f] = resample(A.strandPath(W, part.displayInsts[f]), part.displayPaths[f]);
          else {
            part.paths[f].forEach((p, i) => part.displayPaths[f][i].copy(p));
            part.displayLengths[f] = part.lengths[f];
          }
        }
        updateCapsule(part, W);
      }
      else if (part.fanMid) updateFan(part, W);
      else updateLigament(part, W);
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
