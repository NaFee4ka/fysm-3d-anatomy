// three.js view of fascia3.js: translucent fascial sheets, thin bands (like the ligaments), Myers lines as glowing paths.
//
//   import { createFasciaRender } from './fascia_render.js';
//   const FR = createFasciaRender(F, A, THREE, { xray: true });   // F = createFascia(A, THREE)
//   model.add(FR.group);
//   // whenever the pose changes (W = world transforms of the body):
//   const m = F.withPhysics(F.measure(W), phys);                 // phys optional (engagement for 'effort' mode)
//   FR.update(W, m, { mode: state.lightMode === 'effort' ? 'effort' : 'length', light: state.light, time: now });
//   FR.setLayer('sheets' | 'bands' | 'lines', on);  FR.showLines(['SBL', 'SFL'] | null = all);  FR.focus({ line: 'SBL' } | { el: 'tlf' } | null);
//   FR.pickables → meshes with userData { kind: 'fascia', id, side } or { kind: 'line', id, side } for the raycaster
//
// Colours: fascia is pale silver-blue; stretch glows warm (orange), shortening cool (blue) — the same language as the
// muscles' «Длина» mode; in «Усилие» mode lines glow by engagement ((Tact + Tpas) / ΣFmax) in their own colour.
// Routed anatomical fascia is a polyline. Interpolation is confined to each
// original segment: smoothing a wrapped corner can cut through its obstacle.
export function sampleFasciaRoute(points, minimumCount = 2) {
  if (!Array.isArray(points) || !points.length || !Number.isInteger(minimumCount) || minimumCount < 2
      || !points.every(p => p && [p.x, p.y, p.z].every(Number.isFinite)))
    throw new Error('Invalid routed fascia path');
  const unique = [points[0]];
  for (let i = 1; i < points.length; i++)
    if (points[i].distanceToSquared(unique.at(-1)) > 0) unique.push(points[i]);
  if (unique.length === 1) return Array.from({ length: minimumCount }, () => unique[0].clone());
  const lengths = unique.slice(1).map((p, i) => p.distanceTo(unique[i]));
  const total = lengths.reduce((sum, x) => sum + x, 0);
  const count = Math.max(minimumCount, unique.length), spare = count - unique.length;
  const quotas = lengths.map(x => spare * x / total), intervals = quotas.map(x => 1 + Math.floor(x));
  const remaining = count - 1 - intervals.reduce((sum, x) => sum + x, 0);
  const order = quotas.map((x, i) => ({ i, fraction: x - Math.floor(x) }))
    .sort((a, b) => b.fraction - a.fraction || a.i - b.i);
  for (let i = 0; i < remaining; i++) intervals[order[i].i]++;
  const sampled = [unique[0].clone()];
  for (let i = 0; i < lengths.length; i++) for (let step = 1; step <= intervals[i]; step++)
    sampled.push(step === intervals[i] ? unique[i + 1].clone() : unique[i].clone().lerp(unique[i + 1], step / intervals[i]));
  return sampled;
}

// Sheets share the union of their columns' normalised arc-length corners. At
// every original column, every routed corner is then a true surface vertex.
export function sampleFasciaSheet(columns, minimumRows = 14, minimumColumns = 10) {
  if (!Array.isArray(columns) || !columns.length || minimumRows < 2 || minimumColumns < 2)
    throw new Error('Invalid fascia sheet');
  const routes = columns.map(points => sampleFasciaRoute(points));
  const cumulative = routes.map(points => {
    const lengths = [0];
    for (let i = 1; i < points.length; i++) lengths.push(lengths.at(-1) + points[i].distanceTo(points[i - 1]));
    const total = lengths.at(-1);
    return lengths.map(x => total > 0 ? x / total : 0);
  });
  const knots = values => {
    const out = [];
    for (const value of values.sort((a, b) => a - b))
      if (!out.length || value - out.at(-1) > 1e-12) out.push(value);
    return out;
  };
  const rows = knots([...Array.from({ length: minimumRows }, (_, i) => i / (minimumRows - 1)), ...cumulative.flat()]);
  const across = knots([...Array.from({ length: minimumColumns }, (_, i) => i / (minimumColumns - 1)),
    ...routes.map((_, i) => routes.length > 1 ? i / (routes.length - 1) : 0)]);
  const points = rows.map(v => {
    const along = routes.map((route, column) => {
      const parameters = cumulative[column];
      if (v === 0) return route[0].clone();
      if (v === 1) return route.at(-1).clone();
      let i = 1;
      while (i < parameters.length - 1 && parameters[i] < v) i++;
      if (v === parameters[i]) return route[i].clone();
      const den = parameters[i] - parameters[i - 1];
      return route[i - 1].clone().lerp(route[i], den > 0 ? Math.max(0, Math.min(1, (v - parameters[i - 1]) / den)) : 0);
    });
    return across.map(u => {
      if (routes.length === 1) return along[0].clone();
      const f = u * (routes.length - 1), i = Math.min(routes.length - 2, Math.floor(f));
      if (Number.isInteger(f)) return along[f].clone();
      return along[i].clone().lerp(along[i + 1], f - i);
    });
  });
  return { points, rows, across };
}

export function createFasciaRender(F, A, THREE, opts = {}) {
  const group = new THREE.Group(); group.name = 'fascia';
  const FASCIA = new THREE.Color(0xd5e4ea), WARM = new THREE.Color(0xff8a2a), COOL = new THREE.Color(0x36b9ff);
  const xray = opts.xray !== false;
  const glowK = (s, ref) => Math.pow(Math.min(1, Math.abs(s) / ref), 0.8);

  // faint fibre texture for sheets
  const tex = (() => {
    if (typeof document === 'undefined') return null;
    const c = document.createElement('canvas'); c.width = 64; c.height = 8;
    const g = c.getContext('2d');
    for (let x = 0; x < 64; x++) { const v = 222 + Math.round(Math.sin(x * 1.3) * 14 + (x % 5 === 0 ? -22 : 0)); g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(x, 0, 1, 8); }
    const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(3, 1); if (THREE.SRGBColorSpace) t.colorSpace = THREE.SRGBColorSpace;
    return t;
  })();

  // ---------- tube along a path (bands and lines) ----------
  const NRAD = 8;
  class Tube {
    constructor(r, flat, mat, nseg) {
      this.r = r; this.flat = flat; this.mat = mat;
      this.mesh = new THREE.Mesh(new THREE.BufferGeometry(), mat); this.mesh.frustumCulled = false;
      this.resize(nseg || 28);
    }
    resize(NSEG) {
      this.nseg = NSEG;
      const nV = (NSEG + 1) * NRAD;
      this.pos = new Float32Array(nV * 3); this.nor = new Float32Array(nV * 3);
      const idx = [];
      for (let i = 0; i < NSEG; i++) for (let j = 0; j < NRAD; j++) { const a = i * NRAD + j, b = i * NRAD + (j + 1) % NRAD, c = (i + 1) * NRAD + j, d = (i + 1) * NRAD + (j + 1) % NRAD; idx.push(a, b, c, b, d, c); }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
      g.setAttribute('normal', new THREE.BufferAttribute(this.nor, 3));
      g.setIndex(idx);
      this.mesh.geometry.dispose(); this.mesh.geometry = this.geo = g;
    }
    update(pts, referenceNormal = null, routed = false, referenceBinormal = null) {
      if (pts.length < 2) return;
      let P;
      if (routed) {
        P = sampleFasciaRoute(pts, this.nseg + 1);
        if (P.length !== this.nseg + 1) this.resize(P.length - 1);
      } else {
        const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal'); curve.arcLengthDivisions = this.nseg * 3;
        P = curve.getSpacedPoints(this.nseg);
      }
      const NSEG = this.nseg;
      const T = [], N = new THREE.Vector3(), B = new THREE.Vector3(), q = new THREE.Quaternion();
      for (let i = 0; i <= NSEG; i++) { const a = P[Math.max(0, i - 1)], b = P[Math.min(NSEG, i + 1)]; const t = b.clone().sub(a);
        if (t.lengthSq() < 1e-14) {
          if (i) t.copy(T[i - 1]);
          else if (referenceNormal && referenceBinormal) t.crossVectors(referenceNormal, referenceBinormal);
          else t.set(0, 1, 0);
        }
        T.push(t.normalize()); }
      if (referenceNormal && referenceBinormal) {
        const restTangent = new THREE.Vector3().crossVectors(referenceNormal, referenceBinormal).normalize();
        N.copy(referenceNormal);
        if (restTangent.dot(T[0]) > -1 + 1e-9) {
          q.setFromUnitVectors(restTangent, T[0]); N.applyQuaternion(q);
        }
        N.addScaledVector(T[0], -N.dot(T[0]));
      } else if (referenceNormal) N.copy(referenceNormal).addScaledVector(T[0], -referenceNormal.dot(T[0]));
      if (N.lengthSq() < 1e-14) {
        const ax = Math.abs(T[0].x) < 0.6 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
        N.crossVectors(T[0], ax);
      }
      N.normalize();
      for (let i = 0; i <= NSEG; i++) {
        if (i > 0) {
          // At a reversal use the previous section normal as the half-turn
          // axis. A world-axis fallback would rotate a flat band during a
          // rigid whole-body rotation.
          if (T[i - 1].dot(T[i]) > -1 + 1e-9) {
            q.setFromUnitVectors(T[i - 1], T[i]); N.applyQuaternion(q);
          }
          N.addScaledVector(T[i], -N.dot(T[i])).normalize();
        }
        B.crossVectors(T[i], N);
        const taper = Math.min(1, 0.35 + Math.min(i, NSEG - i) / 4);
        const rN = this.r * taper, rB = this.r * this.flat * taper;
        for (let j = 0; j < NRAD; j++) {
          const th = j / NRAD * Math.PI * 2, cs = Math.cos(th), sn = Math.sin(th), k = (i * NRAD + j) * 3;
          this.pos[k] = P[i].x + N.x * cs * rN + B.x * sn * rB; this.pos[k + 1] = P[i].y + N.y * cs * rN + B.y * sn * rB; this.pos[k + 2] = P[i].z + N.z * cs * rN + B.z * sn * rB;
          const nx = N.x * cs * rB + B.x * sn * rN, ny = N.y * cs * rB + B.y * sn * rN, nz = N.z * cs * rB + B.z * sn * rN, il = 1 / (Math.hypot(nx, ny, nz) || 1);
          this.nor[k] = nx * il; this.nor[k + 1] = ny * il; this.nor[k + 2] = nz * il;
        }
      }
      this.geo.attributes.position.needsUpdate = true; this.geo.attributes.normal.needsUpdate = true; this.geo.computeBoundingSphere();
      this.center = P;
    }
  }

  // a line is drawn through stations joined by straight gaps: resample by arc length and relax the corners (display only)
  function smoothPath(pts, n, iters) {
    const L = [0]; for (let i = 1; i < pts.length; i++) L.push(L[i - 1] + pts[i].distanceTo(pts[i - 1]));
    const tot = L[L.length - 1]; if (tot < 1e-6) return pts;
    const out = []; let j = 1;
    for (let k = 0; k < n; k++) { const d = tot * k / (n - 1); while (j < L.length - 1 && L[j] < d) j++; const u = (d - L[j - 1]) / ((L[j] - L[j - 1]) || 1); out.push(pts[j - 1].clone().lerp(pts[j], Math.max(0, Math.min(1, u)))); }
    for (let it = 0; it < iters; it++) { const prev = out.map(p => p.clone()); for (let k = 1; k < n - 1; k++) out[k].copy(prev[k - 1]).add(prev[k + 1]).multiplyScalar(0.25).addScaledVector(prev[k], 0.5); }
    return out;
  }

  // ---------- sheet surface from a grid of rig points ----------
  class Sheet {
    constructor(cols, mat) {
      this.cols = cols; this.mat = mat;
      this.geo = new THREE.BufferGeometry();
      this.mesh = new THREE.Mesh(this.geo, mat); this.mesh.frustumCulled = false; this.mesh.renderOrder = 2;
    }
    update(W) {
      const sampled = sampleFasciaSheet(this.cols.map(ci => A.strandPath(W, ci)));
      const NU = sampled.across.length, NV = sampled.rows.length;
      if (this.nu !== NU || this.nv !== NV) {
        const g = new THREE.BufferGeometry(), uv = new Float32Array(NU * NV * 2), idx = [];
        this.pos = new Float32Array(NU * NV * 3);
        for (let v = 0; v < NV; v++) for (let u = 0; u < NU; u++) {
          const k = v * NU + u; uv[k * 2] = sampled.across[u]; uv[k * 2 + 1] = sampled.rows[v];
        }
        for (let v = 0; v < NV - 1; v++) for (let u = 0; u < NU - 1; u++) {
          const a = v * NU + u, b = a + 1, c = a + NU, d = c + 1; idx.push(a, c, b, b, c, d);
        }
        g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
        g.setAttribute('uv', new THREE.BufferAttribute(uv, 2).setUsage(THREE.DynamicDrawUsage)); g.setIndex(idx);
        this.geo.dispose(); this.mesh.geometry = this.geo = g; this.nu = NU; this.nv = NV;
      }
      for (let v = 0; v < NV; v++) for (let u = 0; u < NU; u++) {
        sampled.points[v][u].toArray(this.pos, (v * NU + u) * 3);
        this.geo.attributes.uv.setXY(v * NU + u, sampled.across[u], sampled.rows[v]);
      }
      this.sampled = sampled;
      this.geo.attributes.position.needsUpdate = true; this.geo.attributes.uv.needsUpdate = true;
      // Accumulate in double precision before storing Float32 normals. Rows
      // from different source columns can be close: deriving their normals
      // from rounded position buffers made small faces visibly unstable.
      const sums = new Float64Array(NU * NV * 3), a = new THREE.Vector3(), b = new THREE.Vector3(), cross = new THREE.Vector3();
      const addFace = (i, j, k) => {
        const p = sampled.points[Math.floor(i / NU)][i % NU], q = sampled.points[Math.floor(j / NU)][j % NU], r = sampled.points[Math.floor(k / NU)][k % NU];
        cross.crossVectors(a.subVectors(q, p), b.subVectors(r, p));
        for (const vertex of [i, j, k]) { sums[vertex * 3] += cross.x; sums[vertex * 3 + 1] += cross.y; sums[vertex * 3 + 2] += cross.z; }
      };
      for (let v = 0; v < NV - 1; v++) for (let u = 0; u < NU - 1; u++) {
        const i = v * NU + u; addFace(i, i + NU, i + 1); addFace(i + 1, i + NU, i + NU + 1);
      }
      if (!this.geo.attributes.normal) this.geo.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(NU * NV * 3), 3).setUsage(THREE.DynamicDrawUsage));
      for (let i = 0; i < NU * NV; i++) {
        a.fromArray(sums, i * 3).normalize(); this.geo.attributes.normal.setXYZ(i, a.x, a.y, a.z);
      }
      this.geo.attributes.normal.needsUpdate = true; this.geo.computeBoundingSphere();
    }
  }

  // ---------- build ----------
  const sheets = [], bands = [], lines = [];
  const restW = A.computeWorld(A.STAND.local, A.STAND.rootQ, new THREE.Vector3());
  function routeReference(path, seg) {
    let tangent = path[1].clone().sub(path[0]);
    if (tangent.lengthSq() < 1e-14) tangent.set(0, 1, 0);
    tangent.normalize();
    const axis = Math.abs(tangent.x) < 0.6 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
    const normal = new THREE.Vector3().crossVectors(tangent, axis).normalize();
    const binormal = new THREE.Vector3().crossVectors(tangent, normal).normalize();
    const inverse = restW[seg].q.clone().invert();
    return { seg, normal: normal.applyQuaternion(inverse), binormal: binormal.applyQuaternion(inverse) };
  }
  const pickables = [];
  const sheetMat = () => new THREE.MeshStandardMaterial({ color: FASCIA, map: tex, roughness: 0.42, metalness: 0, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false, emissive: 0x000000, emissiveIntensity: 0 });
  const bandMat = () => new THREE.MeshStandardMaterial({ color: FASCIA, roughness: 0.32, metalness: 0, transparent: true, opacity: 0.92, emissive: 0x000000, emissiveIntensity: 0 });
  for (const e of F.ELEMENTS) for (const side of ['L', 'R']) {
    if (e.link) continue;                                    // a ligament the anatomy model already draws
    if (e.gridCols) for (const cols of e.gridCols[side]) {
      if (side === 'R' && e.central) continue;
      const s = new Sheet(cols, sheetMat());
      s.mesh.userData = { kind: 'fascia', id: e.id, side }; s.el = e; s.side = side;
      sheets.push(s); group.add(s.mesh); pickables.push(s.mesh);
    }
    for (const inst of e.inst[side]) {
      if (side === 'R' && inst.central) continue;          // midline strands are drawn once
      // on a sheet the fibres are drawn thin, they show the direction of the fibres
      const nseg = Math.max(6, Math.min(24, Math.ceil((inst.L0 || 0.1) / 0.012)));     // resolution by length: most bands are short
      const t = new Tube(e.gridInst ? Math.min(0.0012, inst.r || 0.002) : (inst.r || 0.002), e.gridInst ? 1 : (inst.flat ?? 0.42), bandMat(), nseg);
      if (e.gridInst) t.mat.opacity = 0.5;
      t.mesh.userData = { kind: 'fascia', id: e.id, side }; t.el = e; t.side = side; t.inst = inst; t.onSheet = !!e.gridInst;
      t.reference = routeReference(A.strandPath(restW, inst), inst.pts[0].seg);
      bands.push(t); group.add(t.mesh); pickables.push(t.mesh);
    }
  }
  for (const l of F.LINES) for (const side of ['L', 'R']) {
    const col = new THREE.Color(l.color);
    const mat = new THREE.MeshStandardMaterial({ color: col, roughness: 0.4, metalness: 0, emissive: col, emissiveIntensity: 0.35, transparent: true, opacity: xray ? 0.9 : 1, depthTest: !xray, depthWrite: !xray });
    const t = new Tube(opts.lineR || 0.0034, 1, mat, 64);
    t.mesh.renderOrder = xray ? 24 : 3;
    t.mesh.userData = { kind: 'line', id: l.id, side }; t.line = l; t.side = side;
    // Even a round eight-sided tube has visible facets. Carry its section with
    // an anatomical station rather than reselecting a world-axis frame.
    const station = F.resolveLine(l, side).stations[0];
    t.reference = routeReference(F.linePath(restW, l.id, side), station?.seg || 'pelvis');
    lines.push(t); group.add(t.mesh); pickables.push(t.mesh);
  }

  // bony stations of the lines (Myers' «станции»): small markers, shown with the line in focus or with the 'stations' layer
  const stationGeo = new THREE.SphereGeometry(1, 12, 9);
  const stations = [];
  for (const t of lines) {
    const res = F.resolveLine(t.line, t.side);
    const mat = new THREE.MeshBasicMaterial({ color: new THREE.Color(t.line.color).lerp(new THREE.Color(0xffffff), 0.45), transparent: true, opacity: 0.95, depthTest: !xray, depthWrite: false });
    for (const st of res.stations) {
      const mk = new THREE.Mesh(stationGeo, mat); mk.scale.setScalar(opts.stationR || 0.0065); mk.renderOrder = 26;
      mk.userData = { kind: 'station', line: t.line.id, side: t.side, ru: st.ru };
      stations.push({ mk, st, line: t.line.id, side: t.side }); group.add(mk);
    }
  }

  // ---------- state ----------
  const layer = { sheets: true, bands: true, lines: true, stations: false };
  let shown = null, focus = null;
  function vis() {
    for (const s of sheets) s.mesh.visible = layer.sheets;
    for (const b of bands) b.mesh.visible = layer.bands;
    for (const t of lines) t.mesh.visible = layer.lines && (!shown || shown.includes(t.line.id));
    for (const s of stations) s.mk.visible = layer.lines && (!shown || shown.includes(s.line)) && (layer.stations || (focus && focus.line === s.line));
  }
  vis();
  function setOpacity(mat, o) { if (mat.opacity !== o) { mat.opacity = o; mat.needsUpdate = true; } }

  function update(W, m, o = {}) {
    const mode = o.mode || 'length', light = o.light !== false;
    const pulse = o.time != null && !o.reduceMotion ? 0.8 + 0.2 * Math.sin(o.time * 0.0042) : 1;
    for (const s of sheets) {
      if (!layer.sheets) break;                                // hidden layers cost nothing
      s.update(W);
      const st = m && m.el[s.el.id + '_' + s.side] ? m.el[s.el.id + '_' + s.side].strain : 0;
      s.mat.emissive.copy(st >= 0 ? WARM : COOL);
      s.mat.emissiveIntensity = light && mode === 'length' ? 0.9 * glowK(st, 0.15) : 0;
      const own = focus && focus.el === s.el.id;
      setOpacity(s.mat, focus ? (own ? 0.62 : 0.12) : 0.4);
    }
    for (const b of bands) {
      if (!layer.bands) break;
      b.update(A.strandPath(W, b.inst), b.reference.normal.clone().applyQuaternion(W[b.reference.seg].q), true,
        b.reference.binormal.clone().applyQuaternion(W[b.reference.seg].q));
      const st = m && m.el[b.el.id + '_' + b.side] ? m.el[b.el.id + '_' + b.side].strain : 0;
      b.mat.emissive.copy(st >= 0 ? WARM : COOL);
      b.mat.emissiveIntensity = light && mode === 'length' && !b.el.pathProxy ? 0.8 * glowK(st, 0.15) : 0;
      const own = focus && focus.el === b.el.id;
      setOpacity(b.mat, focus ? (own ? 1 : 0.2) : (b.onSheet ? 0.5 : 0.92));
    }
    for (const t of lines) {
      if (!t.mesh.visible) continue;
      const key = t.line.id + '_' + t.side;
      const lm = m && m.lines[key];
      const pts = lm && lm.path ? lm.path : F.linePath(W, t.line.id, t.side);
      t.update(smoothPath(pts, 80, 3), t.reference.normal.clone().applyQuaternion(W[t.reference.seg].q), false,
        t.reference.binormal.clone().applyQuaternion(W[t.reference.seg].q));
      let k = 0;
      if (lm) k = mode === 'effort' ? glowK(lm.engage || 0, 0.25) : glowK(Math.max(0, lm.strain), 0.1);
      t.mat.emissiveIntensity = light ? (0.3 + 1.4 * k) * (k > 0.15 ? pulse : 1) : 0.15;
      const own = focus && focus.line === t.line.id;
      setOpacity(t.mat, focus && (focus.line || focus.el) ? (own ? 1 : 0.12) : (xray ? 0.9 : 1));
    }
    updateStations(W);
  }
  function updateStations(W) { for (const s of stations) if (s.mk.visible) s.mk.position.copy(A.pt(W, s.st.seg, s.st.p)); }
  return {
    group, sheets, bands, lines, stations, pickables,
    update,
    setLayer(name, on) { layer[name] = !!on; vis(); },
    showLines(ids) { shown = ids && ids.length ? ids.slice() : null; vis(); },
    focus(f) { focus = f || null; vis(); },
    dispose() { group.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); }); if (tex) tex.dispose(); },
  };
}
