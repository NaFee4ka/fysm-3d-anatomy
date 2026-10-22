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
      const NSEG = this.nseg = nseg || 28;
      this.r = r; this.flat = flat;
      const nV = (NSEG + 1) * NRAD;
      this.pos = new Float32Array(nV * 3); this.nor = new Float32Array(nV * 3);
      const idx = [];
      for (let i = 0; i < NSEG; i++) for (let j = 0; j < NRAD; j++) { const a = i * NRAD + j, b = i * NRAD + (j + 1) % NRAD, c = (i + 1) * NRAD + j, d = (i + 1) * NRAD + (j + 1) % NRAD; idx.push(a, b, c, b, d, c); }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
      g.setAttribute('normal', new THREE.BufferAttribute(this.nor, 3));
      g.setIndex(idx);
      this.geo = g; this.mat = mat;
      this.mesh = new THREE.Mesh(g, mat); this.mesh.frustumCulled = false;
    }
    update(pts) {
      if (pts.length < 2) return;
      const NSEG = this.nseg;
      const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal'); curve.arcLengthDivisions = NSEG * 3;
      const P = curve.getSpacedPoints(NSEG);
      const T = [], N = new THREE.Vector3(), B = new THREE.Vector3(), q = new THREE.Quaternion();
      for (let i = 0; i <= NSEG; i++) { const a = P[Math.max(0, i - 1)], b = P[Math.min(NSEG, i + 1)]; const t = b.clone().sub(a); if (t.lengthSq() < 1e-14) t.set(0, 1, 0); T.push(t.normalize()); }
      const ax = Math.abs(T[0].x) < 0.6 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
      N.crossVectors(T[0], ax).normalize();
      for (let i = 0; i <= NSEG; i++) {
        if (i > 0) { q.setFromUnitVectors(T[i - 1], T[i]); N.applyQuaternion(q); N.addScaledVector(T[i], -N.dot(T[i])).normalize(); }
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
  const NU = 10, NV = 14;
  class Sheet {
    // cols: the sheet's columns as strand instances {pts, wraps}; each column follows its wrapped path, rows interpolate across
    constructor(cols, mat) {
      this.cols = cols;
      const g = new THREE.BufferGeometry();
      this.pos = new Float32Array(NU * NV * 3);
      const uv = new Float32Array(NU * NV * 2), idx = [];
      for (let v = 0; v < NV; v++) for (let u = 0; u < NU; u++) { const k = v * NU + u; uv[k * 2] = u / (NU - 1); uv[k * 2 + 1] = v / (NV - 1); }
      for (let v = 0; v < NV - 1; v++) for (let u = 0; u < NU - 1; u++) { const a = v * NU + u, b = a + 1, c = a + NU, d = c + 1; idx.push(a, c, b, b, c, d); }
      g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
      g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
      g.setIndex(idx);
      this.geo = g; this.mat = mat; this.mesh = new THREE.Mesh(g, mat); this.mesh.frustumCulled = false; this.mesh.renderOrder = 2;
    }
    update(W) {
      const colS = this.cols.map(ci => resample(A.strandPath(W, ci), NV));
      for (let v = 0; v < NV; v++) {
        const across = colS.map(c => c[v]);
        const S = across.length > 2 ? new THREE.CatmullRomCurve3(across, false, 'centripetal').getPoints(NU - 1) : Array.from({ length: NU }, (_, u) => across[0].clone().lerp(across[across.length - 1], u / (NU - 1)));
        for (let u = 0; u < NU; u++) { const k = (v * NU + u) * 3; this.pos[k] = S[u].x; this.pos[k + 1] = S[u].y; this.pos[k + 2] = S[u].z; }
      }
      this.geo.attributes.position.needsUpdate = true; this.geo.computeVertexNormals(); this.geo.computeBoundingSphere();
    }
  }
  // n points evenly by arc length along a polyline (smooth if it has more than two points)
  function resample(pts, n) {
    let P = pts;
    if (pts.length > 2) { const c = new THREE.CatmullRomCurve3(pts, false, 'centripetal'); c.arcLengthDivisions = n * 2; P = c.getSpacedPoints(n * 2); }
    const L = [0]; for (let i = 1; i < P.length; i++) L.push(L[i - 1] + P[i].distanceTo(P[i - 1]));
    const tot = L[L.length - 1] || 1, out = []; let j = 1;
    for (let k = 0; k < n; k++) { const d = tot * k / (n - 1); while (j < L.length - 1 && L[j] < d) j++; const u = (d - L[j - 1]) / ((L[j] - L[j - 1]) || 1); out.push(P[j - 1].clone().lerp(P[j], Math.max(0, Math.min(1, u)))); }
    return out;
  }

  // ---------- build ----------
  const sheets = [], bands = [], lines = [];
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
      bands.push(t); group.add(t.mesh); pickables.push(t.mesh);
    }
  }
  for (const l of F.LINES) for (const side of ['L', 'R']) {
    const col = new THREE.Color(l.color);
    const mat = new THREE.MeshStandardMaterial({ color: col, roughness: 0.4, metalness: 0, emissive: col, emissiveIntensity: 0.35, transparent: true, opacity: xray ? 0.9 : 1, depthTest: !xray, depthWrite: !xray });
    const t = new Tube(opts.lineR || 0.0034, 1, mat, 64);
    t.mesh.renderOrder = xray ? 24 : 3;
    t.mesh.userData = { kind: 'line', id: l.id, side }; t.line = l; t.side = side;
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
      b.update(A.strandPath(W, b.inst));
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
      t.update(smoothPath(pts, 80, 3));
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
