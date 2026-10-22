export function createAnatomy(THREE) {
  const D = Math.PI / 180;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const IDQ = new THREE.Quaternion();
  const AX = { X: V(1, 0, 0), Y: V(0, 1, 0), Z: V(0, 0, 1) };
  const qAx = (ax, deg) => new THREE.Quaternion().setFromAxisAngle(ax.clone().normalize(), deg * D);
  const mirrorQ = q => new THREE.Quaternion(q.x, -q.y, -q.z, q.w);

  // ---------- segments (neutral standing pose, metres; +X = body left, +Y up, +Z front) ----------
  const SEG = [], S = {};
  const CENTRAL = new Set(['pelvis', 'lumA', 'lumB', 'thA', 'thB', 'thC', 'neckA', 'neckB', 'head']);
  function seg(name, parent, J, side) {
    const s = { name, parent, J: V(J[0], J[1], J[2]), side: side || 'C' };
    SEG.push(s); S[name] = s;
  }
  seg('pelvis', null, [0, 0.930, 0]);
  seg('lumA', 'pelvis', [0, 1.020, -0.058]);
  seg('lumB', 'lumA', [0, 1.095, -0.036]);
  seg('thA', 'lumB', [0, 1.200, -0.046]);
  seg('thB', 'thA', [0, 1.287, -0.070]);
  seg('thC', 'thB', [0, 1.372, -0.082]);
  seg('neckA', 'thC', [0, 1.458, -0.068]);
  seg('neckB', 'neckA', [0, 1.522, -0.046]);
  seg('head', 'neckB', [0, 1.595, -0.036]);
  const MC = [[[0.214, 0.785, 0.022], [0.214, 0.72, 0.026], [0.214, 0.636, 0.028]], [[0.214, 0.785, 0.007], [0.214, 0.715, 0.008], [0.212, 0.623, 0.009]], [[0.213, 0.785, -0.008], [0.212, 0.72, -0.01], [0.21, 0.632, -0.013]], [[0.211, 0.782, -0.021], [0.209, 0.728, -0.026], [0.206, 0.66, -0.031]]];
  const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
  const PIP_T = 0.46, DIP_T = 0.77;
  const FINGER_SEGS = [];
  MC.forEach(([b0, b1, b2], i) => {
    const n = i + 2;
    FINGER_SEGS.push(['f' + n + 'a', 'hand', b1], ['f' + n + 'b', 'f' + n + 'a', lerp3(b1, b2, PIP_T)], ['f' + n + 'c', 'f' + n + 'b', lerp3(b1, b2, DIP_T)]);
  });
  const HAND_SEGS = new Set(['hand', 'f1m', 'f1a', 'f1b'].concat(FINGER_SEGS.map(f => f[0])));
  const LIMB = [
    ['clav', 'thC', [0.025, 1.385, 0.062]],
    ['scap', 'clav', [0.170, 1.428, -0.012]],
    ['hum', 'scap', [0.185, 1.395, 0.000]],
    ['fore', 'hum', [0.205, 1.070, 0.000]],
    ['ulna', 'fore', [0.205, 1.070, 0.000]],
    ['hand', 'fore', [0.212, 0.815, 0.008]],
    ['f1m', 'hand', [0.208, 0.792, 0.030]],
    ['f1a', 'f1m', [0.204, 0.755, 0.048]],
    ['f1b', 'f1a', [0.202, 0.7335, 0.053]],
    ...FINGER_SEGS,
    ['thigh', 'pelvis', [0.090, 0.930, 0.000]],
    ['shank', 'thigh', [0.097, 0.518, -0.006]],
    ['foot', 'shank', [0.095, 0.072, -0.010]],
    ['toes', 'foot', [0.098, 0.016, 0.140]],
  ];
  const sideName = (n, side) => (CENTRAL.has(n) ? n : n + '_' + side);
  const clamp01 = v => Math.max(0, Math.min(1, v));
  // the arm sits 3 cm behind the trunk's mid-plane; scapula and clavicle blend into it
  function armZ(base, x, z) {
    if (base === 'hum' || base === 'fore' || base === 'ulna' || HAND_SEGS.has(base)) return z - 0.03;
    if (base === 'scap') return z - 0.03 * clamp01((x - 0.09) / 0.06);
    if (base === 'clav') return z - 0.03 * clamp01((x - 0.06) / 0.11);
    return z;
  }
  const baseOf = n => n.replace(/_[LR]$/, '');
  const AZ = (base, p) => [p[0], p[1], armZ(base, p[0], p[2])];
  for (const side of ['L', 'R']) for (const [n, p, J0] of LIMB) {
    const J = AZ(n, J0);
    seg(n + '_' + side, sideName(p, side), side === 'L' ? J : [-J[0], J[1], J[2]], side);
  }

  // ---------- forward kinematics ----------
  function computeWorld(local, rootQ, rootP) {
    const W = {};
    for (const s of SEG) {
      const lq = local[s.name] || IDQ;
      if (!s.parent) {
        W[s.name] = { q: rootQ.clone().multiply(lq), p: rootP.clone() };
      } else {
        const P = W[s.parent], par = S[s.parent];
        const off = s.J.clone().sub(par.J).applyQuaternion(P.q);
        W[s.name] = { q: P.q.clone().multiply(lq), p: P.p.clone().add(off) };
      }
    }
    return W;
  }
  function pt(W, segName, p) {
    const w = W[segName];
    return p.clone().sub(S[segName].J).applyQuaternion(w.q).add(w.p);
  }

  // ---------- spine sampling ----------
  const spineCurve = new THREE.CatmullRomCurve3([
    V(0, 1.020, -0.058), S.lumB.J, S.thA.J, S.thB.J, S.thC.J, S.neckA.J, S.neckB.J, S.head.J,
  ], false, 'centripetal');
  const spineSamples = spineCurve.getSpacedPoints(600);
  function spineAtY(y) {
    for (let i = 1; i < spineSamples.length; i++) {
      const a = spineSamples[i - 1], b = spineSamples[i];
      if (y >= a.y && y <= b.y) {
        const t = (y - a.y) / (b.y - a.y || 1);
        const p = a.clone().lerp(b, t);
        const tan = b.clone().sub(a).normalize();
        return { p, tan };
      }
    }
    return { p: spineSamples[spineSamples.length - 1].clone(), tan: V(0, 1, 0) };
  }
  function segForY(y) {
    if (y < 1.095) return 'lumA';
    if (y < 1.200) return 'lumB';
    if (y < 1.287) return 'thA';
    if (y < 1.372) return 'thB';
    if (y < 1.458) return 'thC';
    if (y < 1.522) return 'neckA';
    return 'neckB';
  }
  const VERT = [];
  for (let k = 0; k < 5; k++) VERT.push({ name: 'L' + (5 - k), y: 1.038 + 0.036 * k, R: 0.024 - 0.0005 * k, h: 0.027, kind: 'L' });
  for (let k = 0; k < 12; k++) VERT.push({ name: 'Th' + (12 - k), y: 1.2108 + 0.0215 * k, R: 0.021 - 0.00072 * k, h: 0.0165, kind: 'T', n: 12 - k });
  for (let k = 0; k < 7; k++) VERT.push({ name: 'C' + (7 - k), y: 1.4678 + 0.0196 * k, R: 0.0125 - 0.0003 * k, h: 0.0125, kind: 'C', n: 7 - k });
  for (const v of VERT) { const s = spineAtY(v.y); v.c = s.p; v.tan = s.tan; v.seg = segForY(v.y); }
  const vertByName = {}; for (const v of VERT) vertByName[v.name] = v;

  // ---------- bones ----------
  // kinds: ell {c,r}, lathe {a,b,prof}, tube {pts,r}, plate {pts,hub,th}, torus {c,R,r,n}
  const BONES = [];
  function bone(def) { BONES.push(def); }
  const mir = p => [-p[0], p[1], p[2]];
  function boneLR(def) {
    for (const side of ['L', 'R']) {
      const d = JSON.parse(JSON.stringify(def));
      const bs = def.seg;
      for (const k of ['c', 'a', 'b', 'hub']) if (d[k]) d[k] = AZ(bs, d[k]);
      if (d.pts) d.pts = d.pts.map(p => AZ(bs, p));
      d.seg = sideName(def.seg, side); d.side = side;
      const m = side === 'R';
      if (d.c) d.c = m ? mir(d.c) : d.c;
      if (d.a) d.a = m ? mir(d.a) : d.a;
      if (d.b) d.b = m ? mir(d.b) : d.b;
      if (d.hub) d.hub = m ? mir(d.hub) : d.hub;
      if (d.n) d.n = m ? mir(d.n) : d.n;
      if (d.pts) d.pts = d.pts.map(p => (m ? mir(p) : p));
      if (m && d.pts && d.kind === 'plate') d.pts.reverse();
      bone(d);
    }
  }
  const B = (k, o) => Object.assign({ kind: k }, o);

  // skull
  bone(B('ell', { seg: 'head', c: [0, 1.672, 0.008], r: [0.072, 0.080, 0.094], name: 'Череп', latin: 'cranium', group: 'skull' }));
  bone(B('ell', { seg: 'head', c: [0, 1.622, 0.062], r: [0.052, 0.042, 0.040], name: 'Череп', latin: 'cranium', group: 'skull' }));
  bone(B('tube', { seg: 'head', pts: [[0.05, 1.62, 0.008], [0.048, 1.585, 0.03], [0.03, 1.565, 0.07], [0, 1.56, 0.088], [-0.03, 1.565, 0.07], [-0.048, 1.585, 0.03], [-0.05, 1.62, 0.008]], r: 0.0085, name: 'Нижняя челюсть', latin: 'mandibula', group: 'skull' }));
  bone(B('ell', { seg: 'head', c: [0.031, 1.655, 0.088], r: [0.017, 0.015, 0.012], mat: 'dark', name: 'Череп', latin: 'cranium', group: 'skull' }));
  bone(B('ell', { seg: 'head', c: [-0.031, 1.655, 0.088], r: [0.017, 0.015, 0.012], mat: 'dark', name: 'Череп', latin: 'cranium', group: 'skull' }));
  bone(B('ell', { seg: 'head', c: [0, 1.624, 0.099], r: [0.011, 0.014, 0.007], mat: 'dark', name: 'Череп', latin: 'cranium', group: 'skull' }));
  for (const sx of [1, -1]) {
    bone(B('tube', { seg: 'head', pts: [[sx * 0.064, 1.64, 0.0], [sx * 0.066, 1.642, 0.03], [sx * 0.06, 1.645, 0.06], [sx * 0.048, 1.64, 0.08]], r: 0.0045, name: 'Скуловая дуга', latin: 'arcus zygomaticus', group: 'skull' }));
    bone(B('ell', { seg: 'head', c: [sx * 0.062, 1.6, -0.02], r: [0.01, 0.014, 0.011], name: 'Сосцевидный отросток', latin: 'processus mastoideus', group: 'skull' }));
    bone(B('ell', { seg: 'head', c: [sx * 0.012, 1.603, -0.03], r: [0.007, 0.004, 0.009], mat: 'cart', joint: 'ao', name: 'Затылочный мыщелок', latin: 'condylus occipitalis', group: 'skull' }));
  }
  bone(B('tube', { seg: 'head', pts: [[0.018, 1.556, 0.02], [0.012, 1.553, 0.036], [0, 1.552, 0.042], [-0.012, 1.553, 0.036], [-0.018, 1.556, 0.02]], r: 0.0032, name: 'Подъязычная кость', latin: 'os hyoideum', group: 'skull' }));

  // vertebrae + processes + discs
  for (const v of VERT) {
    const half = v.tan.clone().multiplyScalar(v.h / 2);
    const a = v.c.clone().sub(half), b = v.c.clone().add(half);
    const nm = v.kind === 'L' ? 'Поясничный позвонок ' + v.name : v.kind === 'T' ? 'Грудной позвонок ' + v.name : 'Шейный позвонок ' + v.name;
    const lat = v.kind === 'L' ? 'vertebra lumbalis' : v.kind === 'T' ? 'vertebra thoracica' : 'vertebra cervicalis';
    bone(B('lathe', { seg: v.seg, a: a.toArray(), b: b.toArray(), prof: [[0, v.R], [0.2, v.R * 0.9], [0.5, v.R * 0.84], [0.8, v.R * 0.9], [1, v.R]], name: nm, latin: lat, group: 'spine' }));
    // spinous process
    let dir, len;
    if (v.kind === 'L') { dir = V(0, -0.15, -1); len = 0.045; }
    else if (v.kind === 'T') {
      if (v.n <= 4) { dir = V(0, -0.5, -0.87); len = 0.046; }
      else if (v.n <= 9) { dir = V(0, -0.78, -0.62); len = 0.05; }
      else { dir = V(0, -0.4, -0.92); len = 0.045; }
    } else { dir = V(0, -0.3, -0.95); len = v.n === 7 ? 0.046 : v.n === 2 ? 0.032 : v.n === 1 ? 0.012 : 0.026; }
    dir.normalize();
    const s0 = v.c.clone().add(V(0, 0, -v.R * 0.7));
    const s1 = s0.clone().add(dir.clone().multiplyScalar(len));
    bone(B('lathe', { seg: v.seg, a: s0.toArray(), b: s1.toArray(), prof: [[0, 0.0075], [0.35, 0.0058], [1, 0.0036]], name: nm, latin: lat, group: 'spine' }));
    // laminae (vertebral arch) and facet joints
    for (const sx of [1, -1]) {
      const l0 = v.c.clone().add(V(sx * v.R * 0.75, 0, -v.R * 0.45));
      const l1 = v.c.clone().add(V(sx * 0.012, 0, -v.R - 0.009));
      bone(B('tube', { seg: v.seg, pts: [l0.toArray(), l1.toArray(), s0.clone().add(V(0, 0, -0.004)).toArray()], r: 0.0036, name: nm, latin: lat, group: 'spine' }));
      const fz = v.kind === 'L' ? -v.R - 0.014 : -v.R - 0.012, fx = sx * (v.kind === 'L' ? 0.017 : 0.014);
      bone(B('ell', { seg: v.seg, c: v.c.clone().add(V(fx, v.h / 2 + 0.002, fz)).toArray(), r: [0.0035, 0.0045, 0.0025], mat: 'cart', joint: 'facet', vert: v.name, name: 'Дугоотростчатый сустав', latin: 'articulatio zygapophysialis', group: 'spine' }));
    }
    // transverse processes
    for (const sx of [1, -1]) {
      let t0, t1, rr;
      if (v.kind === 'L') { t0 = V(sx * v.R * 0.6, 0, -v.R * 0.6); t1 = V(sx * 0.046, 0.002, -v.R * 0.9); rr = 0.0042; }
      else if (v.kind === 'T') { t0 = V(sx * v.R * 0.6, 0.003, -v.R * 0.8); t1 = V(sx * 0.036, 0.006, -v.R - 0.016); rr = 0.0048; }
      else { t0 = V(sx * v.R * 0.7, 0, -v.R * 0.2); t1 = V(sx * 0.028, 0, -v.R * 0.25); rr = 0.0042; }
      bone(B('lathe', { seg: v.seg, a: v.c.clone().add(t0).toArray(), b: v.c.clone().add(t1).toArray(), prof: [[0, rr], [1, rr * 0.75]], name: nm, latin: lat, group: 'spine' }));
    }
  }
  // sacrum + coccyx
  bone(B('plate', {
    seg: 'pelvis', pts: [[-0.048, 1.03, -0.064], [0.048, 1.03, -0.064], [0.045, 1.0, -0.08], [0.03, 0.965, -0.093], [0.012, 0.93, -0.105], [0, 0.918, -0.108], [-0.012, 0.93, -0.105], [-0.03, 0.965, -0.093], [-0.045, 1.0, -0.08]],
    hub: [0, 0.985, -0.072], th: 0.026, name: 'Крестец', latin: 'os sacrum', group: 'pelvis',
  }));
  // pelvis halves
  boneLR(B('plate', {
    seg: 'pelvis', pts: [[0.042, 1.035, -0.078], [0.07, 1.068, -0.084], [0.105, 1.083, -0.06], [0.135, 1.078, -0.025], [0.142, 1.06, 0.015], [0.125, 1.035, 0.058], [0.112, 1.005, 0.06], [0.106, 0.985, 0.058], [0.103, 0.965, 0.035], [0.095, 0.955, -0.005], [0.08, 0.955, -0.04], [0.06, 0.975, -0.07], [0.045, 0.995, -0.085]],
    hub: [0.082, 1.02, -0.008], th: 0.009, name: 'Подвздошная кость', latin: 'os ilium', group: 'pelvis',
  }));
  boneLR(B('tube', { seg: 'pelvis', pts: [[0.045, 1.04, -0.085], [0.07, 1.07, -0.086], [0.105, 1.085, -0.06], [0.135, 1.08, -0.025], [0.143, 1.062, 0.015], [0.126, 1.037, 0.06]], r: 0.0068, name: 'Гребень подвздошной кости', latin: 'crista iliaca', group: 'pelvis' }));
  boneLR(B('torus', { seg: 'pelvis', c: [0.088, 0.93, 0.0], R: 0.026, r: 0.006, n: [0.75, -0.3, 0.45], name: 'Вертлужная впадина', latin: 'acetabulum', group: 'pelvis' }));
  boneLR(B('torus', { seg: 'pelvis', c: [0.095, 0.927, 0.004], R: 0.0255, r: 0.0032, n: [0.75, -0.3, 0.45], mat: 'cart', joint: 'hip', name: 'Вертлужная губа', latin: 'labrum acetabulare', group: 'pelvis' }));
  boneLR(B('ell', { seg: 'pelvis', c: [0.125, 1.035, 0.058], r: [0.009, 0.009, 0.008], name: 'Передняя верхняя подвздошная ость', latin: 'spina iliaca anterior superior', group: 'pelvis' }));
  boneLR(B('tube', { seg: 'pelvis', pts: [[0.085, 0.915, -0.02], [0.075, 0.885, -0.04], [0.066, 0.862, -0.055]], r: 0.011, name: 'Седалищная кость', latin: 'os ischii', group: 'pelvis' }));
  boneLR(B('ell', { seg: 'pelvis', c: [0.064, 0.858, -0.058], r: [0.012, 0.016, 0.014], name: 'Седалищный бугор', latin: 'tuber ischiadicum', group: 'pelvis' }));
  boneLR(B('tube', { seg: 'pelvis', pts: [[0.088, 0.94, 0.028], [0.06, 0.925, 0.055], [0.03, 0.905, 0.068], [0.012, 0.895, 0.07]], r: 0.008, name: 'Лобковая кость', latin: 'os pubis', group: 'pelvis' }));
  boneLR(B('tube', { seg: 'pelvis', pts: [[0.012, 0.876, 0.066], [0.035, 0.862, 0.04], [0.055, 0.855, 0.0], [0.062, 0.856, -0.04]], r: 0.0065, name: 'Лобковая кость', latin: 'os pubis', group: 'pelvis' }));
  bone(B('ell', { seg: 'pelvis', c: [0, 0.886, 0.07], r: [0.012, 0.02, 0.01], mat: 'cart', name: 'Лобковый симфиз', latin: 'symphysis pubica', group: 'pelvis' }));

  // ribs, cartilage, sternum
  const RIB_W = [0.050, 0.075, 0.095, 0.110, 0.122, 0.130, 0.136, 0.139, 0.139, 0.136, 0.128, 0.112];
  const RIB_DROP = [0.052, 0.066, 0.076, 0.082, 0.087, 0.091, 0.095, 0.095, 0.09, 0.08, 0.06, 0.04];
  for (let i = 1; i <= 12; i++) {
    const v = vertByName['Th' + i];
    const w = RIB_W[i - 1], drop = RIB_DROP[i - 1];
    const zb = v.c.z - 0.034;
    const zf = i <= 7 ? 0.064 + (0.098 - 0.064) * (i - 1) / 6 : [0.095, 0.085, 0.075, 0.06, 0.05][i - 8];
    const zc = (zb + zf) / 2, d = (zf - zb) / 2;
    const th0 = Math.asin(Math.min(0.95, 0.034 / w));
    const th1 = i <= 7 ? Math.PI - Math.asin(Math.min(0.95, 0.022 / w)) : [0.72, 0.66, 0.6, 0.5, 0.42][i - 8] * Math.PI;
    const cartFrom = i <= 7 ? 0.74 : i <= 10 ? 0.78 : 2;
    const ribPts = [], cartPts = [];
    const head = [0.014, v.c.y + 0.004, v.c.z - 0.004];
    ribPts.push(head);
    const N = 14;
    for (let k = 0; k <= N; k++) {
      const u = k / N, th = th0 + (th1 - th0) * u;
      const p = [w * Math.sin(th), v.c.y + 0.003 - drop * Math.pow(u, 1.45), zc - d * Math.cos(th)];
      if (u <= cartFrom + 1e-6) ribPts.push(p);
      if (u >= cartFrom - 1e-6) cartPts.push(p);
    }
    if (i <= 7) { const last = cartPts[cartPts.length - 1]; cartPts.push([0.012, last[1] - 0.004, last[2] + 0.004]); }
    const nm = 'Ребро ' + i;
    boneLR(B('tube', { seg: v.seg, pts: ribPts, r: i === 1 ? 0.0062 : 0.0052, name: nm, latin: 'costa ' + ['I', 'II', 'III', 'IV', 'V', 'VI', 'VII', 'VIII', 'IX', 'X', 'XI', 'XII'][i - 1], group: 'ribs' }));
    if (cartPts.length > 1) boneLR(B('tube', { seg: v.seg, pts: cartPts, r: 0.0048, mat: 'cart', name: 'Рёберный хрящ', latin: 'cartilago costalis', group: 'ribs' }));
  }
  bone(B('plate', {
    seg: 'thC', pts: [[-0.028, 1.404, 0.063], [0.028, 1.404, 0.063], [0.022, 1.375, 0.069], [0.016, 1.35, 0.075], [-0.016, 1.35, 0.075], [-0.022, 1.375, 0.069]],
    hub: [0, 1.377, 0.072], th: 0.012, name: 'Грудина, рукоятка', latin: 'manubrium sterni', group: 'ribs',
  }));
  bone(B('plate', {
    seg: 'thB', pts: [[-0.017, 1.348, 0.076], [0.017, 1.348, 0.076], [0.02, 1.29, 0.088], [0.019, 1.235, 0.098], [0.009, 1.21, 0.099], [0, 1.192, 0.094], [-0.009, 1.21, 0.099], [-0.019, 1.235, 0.098], [-0.02, 1.29, 0.088]],
    hub: [0, 1.28, 0.088], th: 0.011, name: 'Грудина', latin: 'sternum', group: 'ribs',
  }));

  bone(B('lathe', { seg: 'thA', a: [0, 1.195, 0.094], b: [0, 1.158, 0.088], prof: [[0, 0.007], [1, 0.0035]], mat: 'cart', name: 'Мечевидный отросток', latin: 'processus xiphoideus', group: 'ribs' }));
  // shoulder girdle
  boneLR(B('tube', { seg: 'clav', pts: [[0.025, 1.385, 0.062], [0.07, 1.393, 0.068], [0.115, 1.408, 0.045], [0.15, 1.42, 0.012], [0.17, 1.428, -0.012]], r: 0.0066, name: 'Ключица', latin: 'clavicula', group: 'arm' }));
  boneLR(B('plate', {
    seg: 'scap', pts: [[0.077, 1.43, -0.092], [0.105, 1.425, -0.082], [0.132, 1.415, -0.063], [0.148, 1.41, -0.042], [0.158, 1.398, -0.026], [0.158, 1.378, -0.03], [0.145, 1.34, -0.055], [0.125, 1.295, -0.08], [0.1, 1.258, -0.1], [0.091, 1.25, -0.104], [0.083, 1.29, -0.107], [0.079, 1.34, -0.108], [0.077, 1.385, -0.104]],
    hub: [0.112, 1.35, -0.086], th: 0.006, name: 'Лопатка', latin: 'scapula', group: 'arm',
  }));
  boneLR(B('tube', { seg: 'scap', pts: [[0.078, 1.388, -0.11], [0.11, 1.398, -0.106], [0.14, 1.408, -0.09], [0.162, 1.418, -0.062], [0.178, 1.428, -0.036]], r: 0.0056, name: 'Ость лопатки', latin: 'spina scapulae', group: 'arm' }));
  boneLR(B('ell', { seg: 'scap', c: [0.182, 1.432, -0.016], r: [0.016, 0.0062, 0.022], name: 'Акромион', latin: 'acromion', group: 'arm' }));
  boneLR(B('tube', { seg: 'scap', pts: [[0.148, 1.41, -0.038], [0.152, 1.418, -0.005], [0.156, 1.408, 0.03]], r: 0.0056, name: 'Клювовидный отросток', latin: 'processus coracoideus', group: 'arm' }));
  boneLR(B('ell', { seg: 'scap', c: [0.16, 1.388, -0.012], r: [0.006, 0.019, 0.013], mat: 'cart', joint: 'gh', name: 'Суставная впадина лопатки', latin: 'cavitas glenoidalis', group: 'arm' }));
  boneLR(B('torus', { seg: 'scap', c: [0.163, 1.388, -0.012], R: 0.0175, r: 0.0026, n: [1, 0.05, 0.3], mat: 'cart', joint: 'gh', name: 'Суставная губа', latin: 'labrum glenoidale', group: 'arm' }));
  // humerus
  boneLR(B('ell', { seg: 'hum', c: [0.185, 1.395, 0.0], r: [0.024, 0.024, 0.024], name: 'Плечевая кость', latin: 'humerus', group: 'arm' }));
  boneLR(B('ell', { seg: 'hum', c: [0.205, 1.398, 0.01], r: [0.011, 0.014, 0.012], name: 'Плечевая кость', latin: 'humerus', group: 'arm' }));
  boneLR(B('lathe', { seg: 'hum', a: [0.19, 1.372, 0.002], b: [0.205, 1.095, 0.0], prof: [[0, 0.017], [0.15, 0.013], [0.6, 0.012], [0.88, 0.014], [1, 0.02]], name: 'Плечевая кость', latin: 'humerus', group: 'arm' }));
  boneLR(B('ell', { seg: 'hum', c: [0.178, 1.077, -0.006], r: [0.011, 0.009, 0.009], name: 'Плечевая кость', latin: 'humerus', group: 'arm' }));
  boneLR(B('ell', { seg: 'hum', c: [0.231, 1.077, -0.002], r: [0.009, 0.009, 0.009], name: 'Плечевая кость', latin: 'humerus', group: 'arm' }));
  boneLR(B('ell', { seg: 'hum', c: [0.205, 1.07, 0.004], r: [0.026, 0.011, 0.014], name: 'Плечевая кость', latin: 'humerus', group: 'arm' }));
  // forearm
  boneLR(B('lathe', { seg: 'fore', a: [0.214, 1.06, 0.012], b: [0.214, 0.83, 0.026], prof: [[0, 0.0095], [0.08, 0.0075], [0.6, 0.009], [0.92, 0.014], [1, 0.016]], name: 'Лучевая кость', latin: 'radius', group: 'arm' }));
  boneLR(B('lathe', { seg: 'ulna', a: [0.203, 1.092, -0.03], b: [0.212, 0.826, -0.01], prof: [[0, 0.012], [0.06, 0.014], [0.15, 0.011], [0.8, 0.0068], [1, 0.0085]], name: 'Локтевая кость', latin: 'ulna', group: 'arm' }));
  boneLR(B('ell', { seg: 'fore', c: [0.214, 1.061, 0.012], r: [0.0105, 0.004, 0.0105], name: 'Головка лучевой кости', latin: 'caput radii', group: 'arm' }));
  // hand
  const CARPALS = [
    ['Ладьевидная кость', 'os scaphoideum', [0.214, 0.806, 0.019], [0.0075, 0.008, 0.0095]],
    ['Полулунная кость', 'os lunatum', [0.214, 0.807, 0.004], [0.0072, 0.0078, 0.0075]],
    ['Трёхгранная кость', 'os triquetrum', [0.213, 0.805, -0.011], [0.0068, 0.0072, 0.0072]],
    ['Гороховидная кость', 'os pisiforme', [0.204, 0.801, -0.016], [0.0045, 0.0052, 0.0048]],
    ['Кость-трапеция', 'os trapezium', [0.212, 0.79, 0.024], [0.0072, 0.0078, 0.0075]],
    ['Трапециевидная кость', 'os trapezoideum', [0.214, 0.789, 0.011], [0.006, 0.0068, 0.0062]],
    ['Головчатая кость', 'os capitatum', [0.214, 0.788, -0.002], [0.0072, 0.0092, 0.0075]],
    ['Крючковидная кость', 'os hamatum', [0.212, 0.788, -0.016], [0.0068, 0.0082, 0.0078]],
  ];
  for (const [nm, lt, c, r] of CARPALS) boneLR(B('ell', { seg: 'hand', c, r, name: nm, latin: lt, group: 'hand' }));
  MC.forEach(([b0, b1, b2], fi) => {
    const fn = 'f' + (fi + 2);
    boneLR(B('lathe', { seg: 'hand', a: b0, b: b1, prof: [[0, 0.0055], [0.5, 0.0045], [1, 0.0066]], name: 'Пястные кости', latin: 'ossa metacarpi', group: 'hand' }));
    const p1 = lerp3(b1, b2, 0.45), p2 = lerp3(b1, b2, 0.76);
    boneLR(B('lathe', { seg: fn + 'a', a: lerp3(b1, b2, 0.02), b: p1, prof: [[0, 0.0062], [0.5, 0.0052], [1, 0.0056]], name: 'Проксимальная фаланга', latin: 'phalanx proximalis', group: 'hand' }));
    boneLR(B('lathe', { seg: fn + 'b', a: lerp3(b1, b2, 0.47), b: p2, prof: [[0, 0.0054], [0.5, 0.0046], [1, 0.005]], name: 'Средняя фаланга', latin: 'phalanx media', group: 'hand' }));
    boneLR(B('lathe', { seg: fn + 'c', a: lerp3(b1, b2, 0.78), b: b2, prof: [[0, 0.0048], [0.6, 0.004], [1, 0.0032]], name: 'Дистальная фаланга', latin: 'phalanx distalis', group: 'hand' }));
  });
  boneLR(B('lathe', { seg: 'f1m', a: [0.208, 0.792, 0.03], b: [0.204, 0.755, 0.048], prof: [[0, 0.0066], [0.5, 0.0052], [1, 0.0068]], name: 'Первая пястная кость', latin: 'os metacarpale I', group: 'hand' }));
  boneLR(B('lathe', { seg: 'f1a', a: [0.204, 0.755, 0.048], b: [0.2025, 0.7345, 0.0528], prof: [[0, 0.0064], [0.5, 0.0054], [1, 0.0058]], name: 'Проксимальная фаланга большого пальца', latin: 'phalanx proximalis pollicis', group: 'hand' }));
  boneLR(B('lathe', { seg: 'f1b', a: [0.2018, 0.7325, 0.0534], b: [0.2, 0.712, 0.058], prof: [[0, 0.0056], [0.6, 0.0046], [1, 0.0036]], name: 'Дистальная фаланга большого пальца', latin: 'phalanx distalis pollicis', group: 'hand' }));
  // femur
  boneLR(B('ell', { seg: 'thigh', c: [0.09, 0.93, 0.0], r: [0.023, 0.023, 0.023], name: 'Бедренная кость', latin: 'femur', group: 'leg' }));
  boneLR(B('tube', { seg: 'thigh', pts: [[0.09, 0.93, 0.0], [0.118, 0.92, -0.004], [0.14, 0.905, -0.008]], r: 0.012, name: 'Бедренная кость', latin: 'femur', group: 'leg' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.148, 0.912, -0.012], r: [0.014, 0.02, 0.017], name: 'Большой вертел', latin: 'trochanter major', group: 'leg' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.108, 0.862, -0.014], r: [0.008, 0.009, 0.008], name: 'Малый вертел', latin: 'trochanter minor', group: 'leg' }));
  boneLR(B('lathe', { seg: 'thigh', a: [0.14, 0.90, -0.008], b: [0.097, 0.545, -0.002], prof: [[0, 0.019], [0.1, 0.015], [0.5, 0.0125], [0.85, 0.015], [1, 0.022]], name: 'Бедренная кость', latin: 'femur', group: 'leg' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.077, 0.518, -0.006], r: [0.019, 0.022, 0.027], name: 'Бедренная кость', latin: 'femur', group: 'leg' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.117, 0.518, -0.006], r: [0.019, 0.022, 0.027], name: 'Бедренная кость', latin: 'femur', group: 'leg' }));
  boneLR(B('ell', { seg: 'shank', c: [0.098, 0.528, 0.033], r: [0.02, 0.023, 0.009], name: 'Надколенник', latin: 'patella', group: 'leg' }));
  // tibia, fibula
  boneLR(B('ell', { seg: 'shank', c: [0.097, 0.488, 0.0], r: [0.036, 0.012, 0.028], name: 'Большеберцовая кость', latin: 'tibia', group: 'leg' }));
  boneLR(B('lathe', { seg: 'shank', a: [0.097, 0.485, 0.0], b: [0.09, 0.085, -0.004], prof: [[0, 0.03], [0.1, 0.022], [0.25, 0.016], [0.7, 0.012], [0.92, 0.015], [1, 0.02]], name: 'Большеберцовая кость', latin: 'tibia', group: 'leg' }));
  boneLR(B('ell', { seg: 'shank', c: [0.098, 0.448, 0.028], r: [0.008, 0.012, 0.008], name: 'Большеберцовая кость', latin: 'tibia', group: 'leg' }));
  boneLR(B('ell', { seg: 'shank', c: [0.076, 0.072, 0.0], r: [0.009, 0.014, 0.01], name: 'Медиальная лодыжка', latin: 'malleolus medialis', group: 'leg' }));
  boneLR(B('lathe', { seg: 'shank', a: [0.13, 0.465, -0.022], b: [0.118, 0.07, -0.016], prof: [[0, 0.011], [0.08, 0.007], [0.5, 0.0055], [0.92, 0.008], [1, 0.011]], name: 'Малоберцовая кость', latin: 'fibula', group: 'leg' }));
  // foot
  boneLR(B('ell', { seg: 'foot', c: [0.095, 0.058, -0.004], r: [0.018, 0.015, 0.026], name: 'Таранная кость', latin: 'talus', group: 'foot' }));
  boneLR(B('ell', { seg: 'foot', c: [0.094, 0.033, -0.035], r: [0.02, 0.024, 0.038], name: 'Пяточная кость', latin: 'calcaneus', group: 'foot' }));
  boneLR(B('ell', { seg: 'foot', c: [0.082, 0.046, 0.028], r: [0.013, 0.012, 0.009], name: 'Ладьевидная кость стопы', latin: 'os naviculare', group: 'foot' }));
  boneLR(B('ell', { seg: 'foot', c: [0.115, 0.03, 0.03], r: [0.012, 0.011, 0.015], name: 'Кубовидная кость', latin: 'os cuboideum', group: 'foot' }));
  boneLR(B('ell', { seg: 'foot', c: [0.078, 0.042, 0.052], r: [0.0085, 0.011, 0.012], name: 'Медиальная клиновидная кость', latin: 'os cuneiforme mediale', group: 'foot' }));
  boneLR(B('ell', { seg: 'foot', c: [0.092, 0.041, 0.05], r: [0.0068, 0.0095, 0.011], name: 'Промежуточная клиновидная кость', latin: 'os cuneiforme intermedium', group: 'foot' }));
  boneLR(B('ell', { seg: 'foot', c: [0.104, 0.038, 0.048], r: [0.0068, 0.009, 0.011], name: 'Латеральная клиновидная кость', latin: 'os cuneiforme laterale', group: 'foot' }));
  const MT = [[[0.078, 0.034, 0.07], [0.074, 0.02, 0.135], [0.072, 0.012, 0.196], 0.009], [[0.088, 0.036, 0.07], [0.087, 0.018, 0.142], [0.087, 0.009, 0.192], 0.0068], [[0.098, 0.035, 0.068], [0.1, 0.017, 0.137], [0.101, 0.009, 0.183], 0.0064], [[0.108, 0.033, 0.064], [0.113, 0.016, 0.128], [0.115, 0.008, 0.171], 0.006], [[0.118, 0.03, 0.058], [0.126, 0.015, 0.118], [0.129, 0.008, 0.155], 0.006]];
  MT.forEach(([b0, b1, b2, r], ti) => {
    boneLR(B('lathe', { seg: 'foot', a: b0, b: b1, prof: [[0, r], [0.5, r * 0.72], [1, r * 1.05]], name: 'Плюсневые кости', latin: 'ossa metatarsi', group: 'foot' }));
    const cut = ti === 0 ? 0.6 : 0.5;
    boneLR(B('lathe', { seg: 'toes', a: lerp3(b1, b2, 0.03), b: lerp3(b1, b2, cut), prof: [[0, r * 0.95], [0.5, r * 0.78], [1, r * 0.8]], name: 'Проксимальная фаланга стопы', latin: 'phalanx proximalis', group: 'foot' }));
    boneLR(B('lathe', { seg: 'toes', a: lerp3(b1, b2, cut + 0.02), b: b2, prof: [[0, r * 0.78], [0.5, r * 0.66], [1, r * 0.5]], name: 'Дистальная фаланга стопы', latin: 'phalanx distalis', group: 'foot' }));
  });


  // ---------- articular cartilage, menisci, discs (joint layer) ----------
  // articular surfaces glow with their joint's angle; `joint` is the joint id without side
  boneLR(B('ell', { seg: 'hum', c: [0.185, 1.395, 0.0], r: [0.0252, 0.0252, 0.0252], mat: 'cart', joint: 'gh', name: 'Хрящ головки плечевой кости', latin: 'cartilago capitis humeri', group: 'arm' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.09, 0.93, 0.0], r: [0.0245, 0.0245, 0.0245], mat: 'cart', joint: 'hip', name: 'Хрящ головки бедренной кости', latin: 'cartilago capitis femoris', group: 'leg' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.077, 0.518, -0.006], r: [0.0205, 0.0235, 0.0285], mat: 'cart', joint: 'kn', name: 'Хрящ мыщелков бедра', latin: 'cartilago condylorum femoris', group: 'leg' }));
  boneLR(B('ell', { seg: 'thigh', c: [0.117, 0.518, -0.006], r: [0.0205, 0.0235, 0.0285], mat: 'cart', joint: 'kn', name: 'Хрящ мыщелков бедра', latin: 'cartilago condylorum femoris', group: 'leg' }));
  boneLR(B('ell', { seg: 'shank', c: [0.097, 0.4995, 0.0], r: [0.037, 0.0035, 0.029], mat: 'cart', joint: 'kn', name: 'Хрящ плато большеберцовой кости', latin: 'cartilago tibiae', group: 'leg' }));
  boneLR(B('torus', { seg: 'shank', c: [0.079, 0.5035, -0.001], R: 0.0145, r: 0.0038, n: [0, 1, 0], arc: 250, rot: 145, mat: 'cart', joint: 'kn', name: 'Медиальный мениск', latin: 'meniscus medialis', group: 'leg' }));
  boneLR(B('torus', { seg: 'shank', c: [0.115, 0.5035, -0.001], R: 0.0125, r: 0.0038, n: [0, 1, 0], arc: 300, rot: 30, mat: 'cart', joint: 'kn', name: 'Латеральный мениск', latin: 'meniscus lateralis', group: 'leg' }));
  boneLR(B('ell', { seg: 'shank', c: [0.098, 0.528, 0.025], r: [0.019, 0.022, 0.003], mat: 'cart', joint: 'kn', name: 'Хрящ надколенника', latin: 'cartilago patellae', group: 'leg' }));
  boneLR(B('ell', { seg: 'hum', c: [0.205, 1.07, 0.004], r: [0.027, 0.0125, 0.0155], mat: 'cart', joint: 'el', name: 'Хрящ блока и головки мыщелка', latin: 'cartilago trochleae', group: 'arm' }));
  boneLR(B('ell', { seg: 'fore', c: [0.214, 1.062, 0.012], r: [0.011, 0.0025, 0.011], mat: 'cart', joint: 'el', name: 'Хрящ головки лучевой кости', latin: 'cartilago capitis radii', group: 'arm' }));
  boneLR(B('ell', { seg: 'fore', c: [0.2135, 0.825, 0.012], r: [0.013, 0.003, 0.024], mat: 'cart', joint: 'wr', name: 'Хрящ лучезапястного сустава', latin: 'cartilago radiocarpalis', group: 'hand' }));
  boneLR(B('ell', { seg: 'foot', c: [0.095, 0.066, -0.004], r: [0.019, 0.012, 0.0225], mat: 'cart', joint: 'an', name: 'Хрящ блока таранной кости', latin: 'cartilago trochleae tali', group: 'foot' }));
  for (const [b0, b1, b2, r] of MT) boneLR(B('ell', { seg: 'foot', c: b1, r: [r * 1.15, r * 1.15, r * 1.15], mat: 'cart', joint: 'mtp', name: 'Хрящ головки плюсневой кости', latin: 'cartilago capitis metatarsalis', group: 'foot' }));
  boneLR(B('ell', { seg: 'clav', c: [0.025, 1.385, 0.062], r: [0.006, 0.009, 0.008], mat: 'cart', joint: 'sc', name: 'Диск грудино-ключичного сустава', latin: 'discus articularis', group: 'arm' }));
  boneLR(B('ell', { seg: 'scap', c: [0.172, 1.429, -0.013], r: [0.004, 0.006, 0.007], mat: 'cart', joint: 'ac', name: 'Акромиально-ключичный сустав', latin: 'articulatio acromioclavicularis', group: 'arm' }));
  // intervertebral discs
  {
    const chain = [{ c: V(0, 1.02, -0.058), tan: V(0, 1, 0.25).normalize(), h: 0, R: 0.024, seg: 'pelvis' }].concat(VERT);
    for (let k = 0; k < chain.length - 1; k++) {
      const lo = chain[k], hi = chain[k + 1];
      const a = lo.c.clone().addScaledVector(lo.tan, lo.h / 2), b = hi.c.clone().addScaledVector(hi.tan, -hi.h / 2);
      if (a.distanceTo(b) < 0.002) continue;
      const R = Math.min(lo.R, hi.R) * 0.96, seg = lo.seg === 'pelvis' ? 'lumA' : lo.seg;
      bone(B('lathe', { seg, a: a.toArray(), b: b.toArray(), prof: [[0, R * 0.92], [0.5, R], [1, R * 0.92]], mat: 'disc', joint: 'disc', vert: hi.name, name: 'Межпозвонковый диск ' + (k === 0 ? 'L5–S1' : lo.name + '–' + hi.name), latin: 'discus intervertebralis', group: 'spine' }));
    }
  }

  // ---------- strands: muscles (fans), ligaments, capsules ----------
  function trunkSeg(y) { return y < 1.02 ? 'pelvis' : y >= 1.595 ? 'head' : segForY(y); }
  // raw point: [seg|'auto', x, y, z]
  function resolveRaw(r) {
    const base = r[0] === 'auto' ? trunkSeg(r[2]) : r[0];
    return { base, p: V(r[1], r[2], armZ(base, r[1], r[3])) };
  }
  function samplePoly(raw, t) {
    const P = raw.map(resolveRaw);
    if (P.length === 1) return P[0];
    const segLen = [];
    let total = 0;
    for (let i = 1; i < P.length; i++) { const l = P[i].p.distanceTo(P[i - 1].p); segLen.push(l); total += l; }
    let d = t * total;
    for (let i = 0; i < segLen.length; i++) {
      if (d <= segLen[i] || i === segLen.length - 1) {
        const u = segLen[i] > 0 ? Math.max(0, Math.min(1, d / segLen[i])) : 0;
        const a = P[i], b = P[i + 1];
        const p = a.p.clone().lerp(b.p, u);
        const base = (raw[i][0] === 'auto' && raw[i + 1][0] === 'auto') ? trunkSeg(p.y) : (u < 0.5 ? a.base : b.base);
        return { base, p };
      }
      d -= segLen[i];
    }
    return P[P.length - 1];
  }
  const STR = [];
  // single strand (muscle head or ligament)
  function strand(kind, unit, pts, o) { STR.push(Object.assign({ kind, unit, pts: pts.map(resolveRaw), n: 1 }, o)); }
  // fan: origin polyline o, insertion polyline i, optional via polylines, n fascicles
  function fan(unit, o) {
    const n = o.n || 3;
    for (let k = 0; k < n; k++) {
      const t = n === 1 ? 0.5 : k / (n - 1);
      const pts = [samplePoly(o.o, o.oMap ? o.oMap(t) : t)];
      for (const v of (o.via || [])) pts.push(samplePoly(v, o.viaMap ? o.viaMap(t) : t));
      pts.push(samplePoly(o.i, o.iMap ? o.iMap(t) : t));
      const tn = n === 1 ? 0.5 : (k + (k === n - 1 ? -1 : 1)) / (n - 1);
      const nbO = n === 1 ? null : samplePoly(o.o, o.oMap ? o.oMap(tn) : tn);
      const nbI = n === 1 ? null : samplePoly(o.i, o.iMap ? o.iMap(tn) : tn);
      STR.push({ kind: 'muscle', unit, pts, n, idx: k, r: o.r, flat: o.flat ?? 0.42, belly: o.belly ?? 0.5, dips: o.dips, wraps: o.wraps || [], label: o.labels ? o.labels[k] : null, flatRef: nbO, flatRefIns: nbI, sheet: n > 1 && o.sheet !== false });
    }
  }
  const mus = (unit, pts, o) => strand('muscle', unit, pts, Object.assign({ wraps: [], flat: 1, belly: 0.5 }, o || {}));
  const lig = (unit, pts, o) => strand('lig', unit, pts, Object.assign({ wraps: [], flat: 0.6, r: 0.0028 }, o || {}));

  // wrapping obstacles: [[seg,x,y,z], radius|[rx,ry,rz], depth]
  const GH_WRAP = [['hum', 0.185, 1.395, 0.0], 0.032];
  const GH_BIG = [['hum', 0.185, 1.395, 0.0], 0.038];
  const HIP_WRAP = [['thigh', 0.09, 0.93, 0.0], 0.038];
  const HIP_BIG = [['thigh', 0.1, 0.925, 0.0], 0.062];
  const KNEE_WRAP = [['thigh', 0.097, 0.52, -0.008], [0.034, 0.03, 0.032]];
  const ELBOW_WRAP = [['hum', 0.205, 1.07, 0.0], [0.026, 0.02, 0.022]];
  const WRIST_WRAP = [['hand', 0.213, 0.8, 0.006], 0.019];
  const ANKLE_WRAP = [['foot', 0.095, 0.058, -0.012], [0.024, 0.024, 0.032]];
  const THORAX = [['thB', 0, 1.30, -0.005], [0.15, 0.2, 0.115], 3];
  const ABDOMEN = [['lumA', 0, 1.08, 0.0], [0.145, 0.16, 0.12], 3];

  // ----- shoulder girdle and arm -----
  fan('trapUp', { o: [['head', 0, 1.6, -0.085], ['auto', 0, 1.49, -0.11]], i: [['clav', 0.125, 1.415, 0.025], ['scap', 0.18, 1.434, -0.016]], n: 3, r: 0.005, labels: ['затылочные', 'шейные', 'к акромиону'] });
  fan('trapMid', { o: [['auto', 0, 1.47, -0.115], ['auto', 0, 1.33, -0.128]], i: [['scap', 0.165, 1.422, -0.055], ['scap', 0.105, 1.395, -0.108]], n: 3, r: 0.0055 });
  fan('trapLow', { o: [['auto', 0, 1.31, -0.128], ['auto', 0, 1.2, -0.12]], i: [['scap', 0.1, 1.392, -0.11], ['scap', 0.085, 1.386, -0.11]], n: 3, r: 0.0045 });
  fan('lat', { o: [['auto', 0, 1.27, -0.128], ['auto', 0, 1.05, -0.112], ['pelvis', 0.1, 1.075, -0.065]], via: [[['auto', 0.1, 1.255, -0.118], ['auto', 0.135, 1.18, -0.08], ['auto', 0.14, 1.15, -0.05]], [['scap', 0.125, 1.285, -0.098], ['scap', 0.133, 1.278, -0.09]]], i: [['hum', 0.188, 1.35, 0.016], ['hum', 0.189, 1.343, 0.019]], n: 5, r: 0.0065, flat: 0.3, wraps: [THORAX, GH_WRAP], labels: ['верхние', 'верхние', 'средние', 'нижние', 'нижние'] });
  fan('pecMaj', { o: [['clav', 0.04, 1.393, 0.07], ['auto', 0.014, 1.345, 0.09], ['auto', 0.016, 1.225, 0.1], ['auto', 0.06, 1.2, 0.1]], i: [['hum', 0.204, 1.35, 0.024], ['hum', 0.204, 1.332, 0.028]], iMap: t => 1 - t, n: 5, r: 0.0105, flat: 0.32, wraps: [[['hum', 0.185, 1.395, 0.0], 0.035]], labels: ['ключичные', 'грудинные', 'грудинные', 'грудинные', 'брюшные'] });
  fan('pecMin', { sheet: false, o: [['auto', 0.07, 1.33, 0.09], ['auto', 0.08, 1.25, 0.098]], i: [['scap', 0.156, 1.408, 0.03], ['scap', 0.154, 1.406, 0.028]], n: 3, r: 0.005 });
  fan('serratus', { sheet: false, o: [['auto', 0.085, 1.4, 0.02], ['auto', 0.14, 1.31, 0.045], ['auto', 0.136, 1.19, 0.055]], i: [['scap', 0.079, 1.425, -0.088], ['scap', 0.084, 1.3, -0.106], ['scap', 0.092, 1.256, -0.098]], iMap: t => Math.min(1, t * 1.25), n: 7, r: 0.0055, flat: 0.45, wraps: [THORAX], labels: ['1-е ребро', '2-е ребро', '3-е ребро', '4-е ребро', '5-е ребро', '7-е ребро', '9-е ребро'] });
  fan('rhomb', { o: [['auto', 0, 1.45, -0.125], ['auto', 0, 1.33, -0.13]], i: [['scap', 0.079, 1.39, -0.108], ['scap', 0.088, 1.27, -0.106]], n: 4, r: 0.0055 });
  fan('levScap', { sheet: false, o: [['auto', 0.03, 1.585, -0.04], ['auto', 0.034, 1.52, -0.048]], i: [['scap', 0.078, 1.43, -0.09], ['scap', 0.078, 1.42, -0.092]], n: 2, r: 0.0075 });
  const DELT_INS = [['hum', 0.212, 1.272, 0.0], ['hum', 0.213, 1.268, 0.004]];
  fan('deltAnt', { o: [['clav', 0.11, 1.41, 0.045], ['clav', 0.15, 1.422, 0.025]], i: DELT_INS, n: 2, r: 0.012, wraps: [GH_BIG] });
  fan('deltMid', { o: [['scap', 0.168, 1.436, -0.03], ['scap', 0.194, 1.434, -0.004]], i: DELT_INS, n: 3, r: 0.013, wraps: [GH_BIG] });
  fan('deltPost', { o: [['scap', 0.1, 1.396, -0.108], ['scap', 0.15, 1.412, -0.082]], i: DELT_INS, n: 2, r: 0.011, wraps: [GH_BIG] });
  mus('supra', [['scap', 0.1, 1.415, -0.098], ['scap', 0.15, 1.428, -0.05], ['hum', 0.203, 1.418, 0.006]], { r: 0.012 });
  fan('infra', { o: [['scap', 0.092, 1.36, -0.112], ['scap', 0.108, 1.29, -0.11]], via: [[['scap', 0.15, 1.375, -0.075], ['scap', 0.155, 1.345, -0.07]]], i: [['hum', 0.205, 1.4, -0.012], ['hum', 0.206, 1.394, -0.016]], n: 3, r: 0.0085, flat: 0.4 });
  mus('teresMin', [['scap', 0.128, 1.3, -0.088], ['scap', 0.162, 1.35, -0.058], ['hum', 0.2, 1.385, -0.02]], { r: 0.0085, wraps: [GH_WRAP] });
  fan('subscap', { o: [['scap', 0.095, 1.37, -0.085], ['scap', 0.1, 1.28, -0.088]], via: [[['scap', 0.145, 1.39, -0.04], ['scap', 0.148, 1.345, -0.045]]], i: [['hum', 0.19, 1.388, 0.02], ['hum', 0.19, 1.38, 0.022]], n: 3, r: 0.009, flat: 0.4 });
  mus('teresMaj', [['scap', 0.096, 1.262, -0.108], ['scap', 0.135, 1.3, -0.075], ['hum', 0.186, 1.345, 0.012]], { r: 0.013, wraps: [GH_WRAP] });
  mus('tricLong', [['scap', 0.158, 1.373, -0.025], ['hum', 0.19, 1.26, -0.032], ['hum', 0.203, 1.13, -0.032], ['hum', 0.204, 1.1, -0.034], ['ulna', 0.203, 1.093, -0.032]], { r: 0.017 });
  mus('tricLat', [['hum', 0.205, 1.34, -0.016], ['hum', 0.217, 1.2, -0.03], ['hum', 0.207, 1.1, -0.034], ['ulna', 0.205, 1.093, -0.032]], { r: 0.0155 });
  mus('tricMed', [['hum', 0.198, 1.2, -0.026], ['hum', 0.2, 1.12, -0.03], ['hum', 0.202, 1.1, -0.034], ['ulna', 0.203, 1.093, -0.032]], { r: 0.013 });
  mus('bicLong', [['scap', 0.163, 1.413, -0.006], ['hum', 0.198, 1.415, 0.018], ['hum', 0.196, 1.34, 0.024], ['hum', 0.205, 1.18, 0.032], ['fore', 0.212, 1.037, 0.016]], { r: 0.0145 });
  mus('bicShort', [['scap', 0.156, 1.405, 0.032], ['hum', 0.19, 1.3, 0.034], ['hum', 0.202, 1.18, 0.032], ['fore', 0.212, 1.037, 0.016]], { r: 0.0135 });
  mus('coracobr', [['scap', 0.155, 1.405, 0.028], ['hum', 0.188, 1.3, 0.016], ['hum', 0.19, 1.225, 0.0]], { r: 0.009 });
  mus('brachialis', [['hum', 0.206, 1.21, 0.018], ['hum', 0.208, 1.11, 0.022], ['ulna', 0.203, 1.052, -0.012]], { r: 0.017 });
  mus('brachiorad', [['hum', 0.228, 1.135, 0.0], ['fore', 0.232, 1.0, 0.012], ['fore', 0.222, 0.85, 0.026], ['fore', 0.216, 0.836, 0.03]], { r: 0.012, belly: 0.3 });
  mus('pronTeres', [['hum', 0.18, 1.077, -0.004], ['fore', 0.2, 1.03, 0.012], ['fore', 0.224, 0.98, 0.02]], { r: 0.009 });
  fan('wristFlex', { sheet: false, o: [['hum', 0.18, 1.079, -0.002], ['hum', 0.178, 1.075, -0.008]], via: [[['fore', 0.197, 0.98, 0.012], ['fore', 0.195, 0.975, -0.012]], [['fore', 0.198, 0.86, 0.016], ['fore', 0.197, 0.855, -0.012]], [['fore', 0.198, 0.823, 0.018], ['fore', 0.2, 0.823, -0.014]], [['hand', 0.198, 0.807, 0.018], ['hand', 0.2, 0.807, -0.014]]], i: [['hand', 0.199, 0.76, 0.02], ['hand', 0.198, 0.76, -0.016]], n: 3, r: 0.011, belly: 0.3, labels: ['лучевой сгибатель', 'сгибатели пальцев', 'локтевой сгибатель'] });
  MC.forEach(([b0, b1, b2], i) => {
    const fn = 'f' + (i + 2);
    const z0 = [0.012, 0.004, -0.004, -0.012][i];
    const P = t => lerp3(b1, b2, t);
    const a = P(0.25), b = P(0.62), c = P(0.86);
    // carpal tunnel: the tendons cross the wrist axis 1.3 cm on the palm side, bundled near its middle
    const zc = 0.008 + z0 * 0.4;
    mus('fingerFlex', [['fore', 0.196, 0.93, z0 * 0.6], ['fore', 0.197, 0.86, z0], ['fore', 0.199, 0.823, zc], ['hand', 0.199, 0.807, zc], ['hand', 0.2, 0.79, z0 * 0.8], ['hand', 0.205, b1[1] + 0.012, b1[2]], [fn + 'a', a[0] - 0.007, a[1], a[2]], [fn + 'b', b[0] - 0.007, b[1], b[2]], [fn + 'c', c[0] - 0.006, c[1], c[2]]], { r: 0.0045, belly: 0.12, label: ['к указательному', 'к среднему', 'к безымянному', 'к мизинцу'][i] });
  });

  fan('wristExt', { sheet: false, o: [['hum', 0.232, 1.08, 0.004], ['hum', 0.232, 1.076, -0.006]], via: [[['fore', 0.23, 0.98, 0.012], ['fore', 0.228, 0.975, -0.012]], [['fore', 0.227, 0.86, 0.016], ['fore', 0.225, 0.855, -0.012]], [['fore', 0.225, 0.823, 0.018], ['fore', 0.225, 0.823, -0.014]], [['hand', 0.225, 0.807, 0.018], ['hand', 0.225, 0.807, -0.014]]], i: [['hand', 0.226, 0.775, 0.02], ['hand', 0.225, 0.775, -0.016]], n: 3, r: 0.0095, belly: 0.3, labels: ['лучевые разгибатели', 'разгибатель пальцев', 'локтевой разгибатель'] });

  // ----- neck and trunk -----
  fan('scm', { sheet: false, o: [['head', 0.06, 1.602, -0.02], ['head', 0.058, 1.6, -0.024]], i: [['auto', 0.016, 1.405, 0.062], ['clav', 0.045, 1.395, 0.068]], n: 2, r: 0.009 });
  fan('scalene', { sheet: false, o: [['auto', 0.028, 1.56, -0.04], ['auto', 0.03, 1.5, -0.048]], i: [['auto', 0.055, 1.385, 0.03], ['auto', 0.06, 1.37, 0.0]], n: 3, r: 0.0065 });
  fan('splenius', { sheet: false, o: [['auto', 0, 1.46, -0.12], ['auto', 0, 1.4, -0.128]], i: [['head', 0.055, 1.605, -0.03], ['head', 0.04, 1.6, -0.06]], n: 2, r: 0.0075 });
  fan('erector', { sheet: false, o: [['pelvis', 0.018, 0.985, -0.108], ['pelvis', 0.07, 1.05, -0.095]], via: [[['auto', 0.025, 1.12, -0.092], ['auto', 0.072, 1.13, -0.088]], [['auto', 0.028, 1.26, -0.114], ['auto', 0.076, 1.25, -0.11]], [['auto', 0.028, 1.36, -0.122], ['auto', 0.074, 1.35, -0.118]]], i: [['auto', 0.022, 1.47, -0.105], ['auto', 0.068, 1.42, -0.11]], n: 4, r: 0.014, flat: 0.55, belly: 0.4, labels: ['остистая мышца', 'длиннейшая', 'длиннейшая', 'подвздошно-рёберная'] });
  fan('ql', { o: [['pelvis', 0.075, 1.065, -0.085], ['pelvis', 0.09, 1.06, -0.075]], i: [['auto', 0.055, 1.22, -0.1], ['auto', 0.07, 1.205, -0.095]], n: 2, r: 0.009 });
  fan('rectAbd', { o: [['pelvis', 0.01, 0.895, 0.074], ['pelvis', 0.04, 0.898, 0.072]], via: [[['pelvis', 0.014, 0.975, 0.1], ['pelvis', 0.05, 0.975, 0.098]], [['auto', 0.016, 1.06, 0.114], ['auto', 0.06, 1.06, 0.112]], [['auto', 0.018, 1.15, 0.118], ['auto', 0.066, 1.15, 0.116]]], i: [['auto', 0.022, 1.245, 0.108], ['auto', 0.078, 1.3, 0.102]], n: 3, r: 0.0065, flat: 0.5, dips: [0.4, 0.58, 0.76] });
  fan('extObl', { o: [['auto', 0.112, 1.315, 0.088], ['auto', 0.14, 1.2, 0.035], ['auto', 0.118, 1.13, -0.03]], i: [['auto', 0.03, 1.07, 0.11], ['auto', 0.06, 1.0, 0.1], ['pelvis', 0.11, 1.06, 0.03]], n: 4, r: 0.0045, flat: 0.3, wraps: [ABDOMEN] });
  fan('psoas', { sheet: false, o: [['auto', 0.025, 1.2, -0.03], ['auto', 0.03, 1.08, -0.02]], via: [[['pelvis', 0.055, 1.01, 0.01], ['pelvis', 0.06, 1.0, 0.012]], [['pelvis', 0.078, 0.945, 0.054], ['pelvis', 0.08, 0.94, 0.055]]], i: [['thigh', 0.106, 0.865, -0.006], ['thigh', 0.107, 0.862, -0.004]], n: 2, r: 0.015, belly: 0.4, wraps: [HIP_WRAP] });
  mus('iliacus', [['pelvis', 0.1, 1.025, 0.005], ['pelvis', 0.088, 0.96, 0.048], ['thigh', 0.106, 0.865, -0.006]], { r: 0.014, wraps: [HIP_WRAP] });

  // ----- hip and thigh -----
  fan('gluteMax', { o: [['pelvis', 0.06, 1.052, -0.105], ['pelvis', 0.025, 0.97, -0.115], ['pelvis', 0.012, 0.925, -0.112]], i: [['thigh', 0.152, 0.86, -0.02], ['thigh', 0.142, 0.815, -0.032]], n: 4, r: 0.016, flat: 0.5, wraps: [HIP_BIG], labels: ['верхние, в подвздошно-большеберцовый тракт', 'верхние', 'нижние', 'нижние, к бедру'] });
  fan('gluteMed', { o: [['pelvis', 0.09, 1.065, -0.065], ['pelvis', 0.135, 1.07, -0.015], ['pelvis', 0.13, 1.045, 0.04]], i: [['thigh', 0.15, 0.922, -0.016], ['thigh', 0.152, 0.918, -0.006]], n: 3, r: 0.011, flat: 0.5, labels: ['задние', 'средние', 'передние'] });
  fan('gluteMin', { o: [['pelvis', 0.1, 1.01, -0.03], ['pelvis', 0.115, 1.005, 0.03]], i: [['thigh', 0.148, 0.916, 0.002], ['thigh', 0.147, 0.914, 0.006]], n: 2, r: 0.0075 });
  mus('piriformis', [['pelvis', 0.03, 0.975, -0.085], ['pelvis', 0.095, 0.95, -0.05], ['thigh', 0.146, 0.925, -0.012]], { r: 0.009 });
  mus('tfl', [['pelvis', 0.125, 1.03, 0.055], ['thigh', 0.15, 0.95, 0.04], ['thigh', 0.152, 0.87, 0.02]], { r: 0.011 });
  mus('sartorius', [['pelvis', 0.124, 1.032, 0.06], ['thigh', 0.105, 0.82, 0.062], ['thigh', 0.082, 0.65, 0.032], ['thigh', 0.072, 0.545, 0.004], ['shank', 0.08, 0.452, 0.02]], { r: 0.0095, flat: 0.5, wraps: [KNEE_WRAP] });
  mus('pectineus', [['pelvis', 0.06, 0.912, 0.052], ['thigh', 0.108, 0.845, -0.01]], { r: 0.0095 });
  mus('addLong', [['pelvis', 0.03, 0.892, 0.066], ['thigh', 0.09, 0.73, -0.016]], { r: 0.013 });
  fan('addMag', { sheet: false, o: [['pelvis', 0.04, 0.872, 0.03], ['pelvis', 0.062, 0.862, -0.05]], i: [['thigh', 0.098, 0.8, -0.03], ['thigh', 0.072, 0.54, -0.004]], n: 3, r: 0.013, flat: 0.5, labels: ['приводящая часть', 'приводящая часть', 'седалищная часть'] });
  mus('gracilis', [['pelvis', 0.025, 0.88, 0.052], ['thigh', 0.068, 0.62, -0.01], ['shank', 0.078, 0.445, 0.014]], { r: 0.0085, wraps: [KNEE_WRAP] });
  mus('bicFemL', [['pelvis', 0.066, 0.858, -0.064], ['thigh', 0.1, 0.78, -0.062], ['thigh', 0.125, 0.62, -0.055], ['thigh', 0.135, 0.535, -0.042], ['shank', 0.131, 0.465, -0.024]], { r: 0.018, wraps: [HIP_BIG, KNEE_WRAP] });
  mus('bicFemS', [['thigh', 0.115, 0.69, -0.03], ['thigh', 0.133, 0.56, -0.04], ['shank', 0.131, 0.465, -0.024]], { r: 0.012, wraps: [KNEE_WRAP] });
  mus('semiT', [['pelvis', 0.062, 0.857, -0.062], ['thigh', 0.082, 0.76, -0.062], ['thigh', 0.074, 0.6, -0.052], ['thigh', 0.068, 0.525, -0.042], ['shank', 0.072, 0.47, -0.015], ['shank', 0.082, 0.437, 0.022]], { r: 0.016, belly: 0.4, wraps: [HIP_BIG, KNEE_WRAP] });
  mus('semiM', [['pelvis', 0.068, 0.862, -0.058], ['thigh', 0.09, 0.76, -0.05], ['thigh', 0.083, 0.61, -0.048], ['thigh', 0.078, 0.53, -0.042], ['shank', 0.077, 0.478, -0.024]], { r: 0.02, wraps: [HIP_BIG, KNEE_WRAP] });
  const TIB_TUB = ['shank', 0.098, 0.448, 0.03];
  mus('rectFem', [['pelvis', 0.103, 0.985, 0.06], ['thigh', 0.112, 0.72, 0.065], ['thigh', 0.1, 0.58, 0.058], ['shank', 0.098, 0.54, 0.045], TIB_TUB], { r: 0.018, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.045]] });
  mus('vastLat', [['thigh', 0.152, 0.87, 0.002], ['thigh', 0.158, 0.72, 0.022], ['thigh', 0.128, 0.58, 0.045], ['shank', 0.106, 0.54, 0.044], TIB_TUB], { r: 0.03 });
  mus('vastMed', [['thigh', 0.098, 0.8, 0.02], ['thigh', 0.078, 0.63, 0.038], ['thigh', 0.083, 0.565, 0.05], ['shank', 0.092, 0.54, 0.044], TIB_TUB], { r: 0.025, belly: 0.62 });
  mus('vastInt', [['thigh', 0.118, 0.84, 0.016], ['thigh', 0.112, 0.68, 0.04], ['thigh', 0.1, 0.56, 0.05], ['shank', 0.098, 0.54, 0.042], TIB_TUB], { r: 0.022 });

  // ----- lower leg and foot -----
  mus('gastroM', [['thigh', 0.078, 0.535, -0.032], ['shank', 0.077, 0.45, -0.062], ['shank', 0.083, 0.32, -0.064], ['shank', 0.09, 0.18, -0.045], ['shank', 0.093, 0.09, -0.05], ['foot', 0.094, 0.04, -0.07]], { r: 0.021, belly: 0.28, wraps: [KNEE_WRAP] });
  mus('gastroL', [['thigh', 0.118, 0.535, -0.032], ['shank', 0.12, 0.45, -0.058], ['shank', 0.112, 0.32, -0.06], ['shank', 0.098, 0.18, -0.045], ['shank', 0.095, 0.09, -0.05], ['foot', 0.095, 0.04, -0.07]], { r: 0.017, belly: 0.28, wraps: [KNEE_WRAP] });
  mus('soleus', [['shank', 0.112, 0.4, -0.032], ['shank', 0.102, 0.29, -0.05], ['shank', 0.096, 0.16, -0.042], ['shank', 0.095, 0.085, -0.046], ['foot', 0.094, 0.042, -0.068]], { r: 0.022, belly: 0.35 });
  mus('tibAnt', [['shank', 0.118, 0.43, 0.022], ['shank', 0.108, 0.25, 0.03], ['shank', 0.092, 0.1, 0.035], ['foot', 0.082, 0.06, 0.05], ['foot', 0.078, 0.04, 0.062]], { r: 0.014, belly: 0.35 });
  fan('edl', { sheet: false, o: [['shank', 0.122, 0.44, 0.014], ['shank', 0.124, 0.43, 0.01]], via: [[['shank', 0.112, 0.25, 0.028], ['shank', 0.114, 0.25, 0.026]], [['shank', 0.102, 0.095, 0.036], ['shank', 0.106, 0.095, 0.034]], [['foot', 0.095, 0.045, 0.085], ['foot', 0.115, 0.04, 0.075]]], i: [['toes', 0.088, 0.016, 0.185], ['toes', 0.126, 0.014, 0.15]], n: 3, r: 0.0055, flat: 0.8, belly: 0.25 });
  mus('ehl', [['shank', 0.112, 0.3, 0.012], ['shank', 0.095, 0.1, 0.034], ['foot', 0.08, 0.05, 0.075], ['toes', 0.074, 0.018, 0.19]], { r: 0.0055, belly: 0.3 });
  mus('fibL', [['shank', 0.13, 0.455, -0.02], ['shank', 0.126, 0.3, -0.016], ['shank', 0.122, 0.11, -0.022], ['foot', 0.122, 0.055, -0.03], ['foot', 0.118, 0.02, 0.035], ['foot', 0.08, 0.026, 0.068]], { r: 0.0095, belly: 0.3, wraps: [ANKLE_WRAP] });
  mus('fibB', [['shank', 0.126, 0.26, -0.02], ['shank', 0.121, 0.11, -0.024], ['foot', 0.122, 0.055, -0.03], ['foot', 0.125, 0.03, 0.05]], { r: 0.007, belly: 0.3 });
  mus('tibPost', [['shank', 0.105, 0.36, -0.03], ['shank', 0.09, 0.15, -0.03], ['shank', 0.08, 0.08, -0.016], ['foot', 0.078, 0.046, 0.024]], { r: 0.009, belly: 0.3 });
  mus('fhl', [['shank', 0.116, 0.26, -0.034], ['shank', 0.1, 0.1, -0.04], ['foot', 0.088, 0.045, -0.032], ['foot', 0.078, 0.014, 0.07], ['toes', 0.074, 0.01, 0.185]], { r: 0.0085, belly: 0.3, wraps: [ANKLE_WRAP] });
  fan('fdl', { sheet: false, o: [['shank', 0.092, 0.32, -0.03], ['shank', 0.09, 0.31, -0.028]], via: [[['shank', 0.082, 0.085, -0.018], ['shank', 0.084, 0.085, -0.02]], [['foot', 0.082, 0.03, 0.0], ['foot', 0.09, 0.028, -0.004]], [['foot', 0.09, 0.012, 0.08], ['foot', 0.12, 0.012, 0.07]]], i: [['toes', 0.088, 0.008, 0.18], ['toes', 0.126, 0.008, 0.145]], n: 3, r: 0.005, flat: 0.8, belly: 0.25, wraps: [ANKLE_WRAP] });

  // ----- deep abdominal wall -----
  const ABD_IN = [['lumA', 0, 1.08, 0.0], [0.138, 0.155, 0.112], 3];
  const ABD_TA = [['lumA', 0, 1.08, 0.0], [0.13, 0.15, 0.104], 3];
  fan('intObl', { o: [['pelvis', 0.128, 1.072, -0.04], ['pelvis', 0.141, 1.062, 0.012], ['pelvis', 0.12, 1.03, 0.058]], i: [['auto', 0.112, 1.215, 0.058], ['auto', 0.06, 1.165, 0.106], ['auto', 0.012, 1.07, 0.11]], n: 4, r: 0.0042, flat: 0.3, wraps: [ABD_IN] });
  fan('transAbd', { o: [['auto', 0.1, 1.205, -0.06], ['pelvis', 0.125, 1.06, -0.03]], i: [['auto', 0.008, 1.2, 0.1], ['pelvis', 0.008, 0.96, 0.098]], n: 5, r: 0.003, flat: 0.3, wraps: [ABD_TA] });
  // ----- deep back and neck -----
  {
    const lam = (name, dz) => { const v = VERT.find(x => x.name === name); return ['auto', 0.016, v.y, v.c.z - v.R - (dz || 0.012)]; };
    mus('multifidus', [['pelvis', 0.02, 0.985, -0.1]].concat(['L5', 'L4', 'L3', 'L2', 'L1'].map(n => lam(n, 0.014))), { r: 0.0075, belly: 0.4, label: 'поясничная часть' });
    mus('multifidus', ['Th12', 'Th10', 'Th8', 'Th6', 'Th4'].map(n => lam(n)), { r: 0.0048, label: 'грудная часть' });
    mus('multifidus', ['Th2', 'C7', 'C5', 'C3', 'C2'].map(n => lam(n, 0.008)), { r: 0.0045, label: 'шейная часть' });
    const fr = name => { const v = VERT.find(x => x.name === name); return ['auto', 0.008, v.y, v.c.z + v.R + 0.003]; };
    mus('longusColli', ['Th3', 'Th1', 'C6', 'C4', 'C2', 'C1'].map(fr), { r: 0.0042 });
  }
  mus('semispinalis', [['auto', 0.03, 1.33, -0.098], ['auto', 0.026, 1.43, -0.098], ['auto', 0.022, 1.52, -0.075], ['head', 0.024, 1.612, -0.075]], { r: 0.0068 });
  mus('subclavius', [['auto', 0.032, 1.372, 0.06], ['clav', 0.085, 1.384, 0.06], ['clav', 0.118, 1.398, 0.038]], { r: 0.0045 });
  // ----- deep hip rotators and adductors -----
  const HIP_SM = [['thigh', 0.09, 0.93, 0.0], 0.03];
  mus('obtInt', [['pelvis', 0.048, 0.89, 0.012], ['pelvis', 0.07, 0.896, -0.058], ['thigh', 0.141, 0.914, -0.016]], { r: 0.0058, wraps: [HIP_SM] });
  mus('gemelli', [['pelvis', 0.072, 0.905, -0.062], ['thigh', 0.142, 0.918, -0.016]], { r: 0.0042, wraps: [HIP_SM], label: 'верхняя близнецовая' });
  mus('gemelli', [['pelvis', 0.066, 0.874, -0.06], ['thigh', 0.142, 0.911, -0.018]], { r: 0.0045, wraps: [HIP_SM], label: 'нижняя близнецовая' });
  mus('quadFem', [['pelvis', 0.062, 0.882, -0.05], ['thigh', 0.136, 0.892, -0.026]], { r: 0.0075, flat: 0.6, wraps: [HIP_SM] });
  mus('obtExt', [['pelvis', 0.052, 0.876, 0.03], ['pelvis', 0.08, 0.88, -0.01], ['thigh', 0.135, 0.908, -0.022]], { r: 0.006, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.028]] });
  mus('addBrev', [['pelvis', 0.032, 0.878, 0.056], ['thigh', 0.098, 0.82, -0.018], ['thigh', 0.1, 0.76, -0.022]], { r: 0.0095 });
  // ----- knee and calf -----
  mus('popliteus', [['thigh', 0.13, 0.512, -0.012], ['shank', 0.11, 0.47, -0.034], ['shank', 0.082, 0.425, -0.03]], { r: 0.0055, flat: 0.6 });
  mus('plantaris', [['thigh', 0.122, 0.556, -0.032], ['shank', 0.112, 0.46, -0.05], ['shank', 0.095, 0.3, -0.052], ['shank', 0.09, 0.12, -0.05], ['foot', 0.09, 0.042, -0.07]], { r: 0.004, belly: 0.12, wraps: [KNEE_WRAP] });
  // ----- forearm -----
  mus('supinator', [['ulna', 0.218, 1.035, -0.022], ['fore', 0.226, 1.01, 0.0], ['fore', 0.222, 0.985, 0.02]], { r: 0.006, flat: 0.6 });
  mus('anconeus', [['hum', 0.228, 1.074, -0.014], ['ulna', 0.212, 1.035, -0.036]], { r: 0.005, flat: 0.6 });
  mus('pronQuad', [['ulna', 0.205, 0.86, -0.012], ['fore', 0.203, 0.861, 0.006], ['fore', 0.207, 0.862, 0.024]], { r: 0.0045, flat: 0.5 });
  mus('apl', [['ulna', 0.222, 0.97, -0.004], ['fore', 0.226, 0.9, 0.018], ['fore', 0.222, 0.84, 0.03], ['fore', 0.213, 0.823, 0.026], ['hand', 0.213, 0.807, 0.027], ['f1m', 0.211, 0.789, 0.034]], { r: 0.0045, belly: 0.3 });
  mus('palmaris', [['hum', 0.179, 1.076, -0.002], ['fore', 0.194, 0.96, 0.004], ['fore', 0.197, 0.86, 0.008], ['fore', 0.197, 0.823, 0.008], ['hand', 0.197, 0.807, 0.008], ['hand', 0.2, 0.77, 0.004]], { r: 0.0045, belly: 0.25 });
  // ----- hand -----
  mus('thenar', [['hand', 0.204, 0.795, 0.02], ['f1m', 0.205, 0.775, 0.042], ['f1a', 0.202, 0.752, 0.05]], { r: 0.0068, flat: 0.7 });
  mus('hypothenar', [['hand', 0.204, 0.8, -0.018], ['hand', 0.205, 0.765, -0.028], ['f5a', 0.206, 0.722, -0.03]], { r: 0.006, flat: 0.7 });
  mus('interossei', [['hand', 0.212, 0.765, 0.035], ['f2a', 0.214, 0.714, 0.031]], { r: 0.0034, label: 'первая, к указательному' });
  mus('interossei', [['hand', 0.217, 0.755, 0.016], ['f3a', 0.216, 0.708, 0.013]], { r: 0.0032, label: 'вторая, к среднему' });
  mus('interossei', [['hand', 0.217, 0.758, -0.001], ['f3a', 0.216, 0.708, 0.003]], { r: 0.0032, label: 'третья, к среднему' });
  mus('interossei', [['hand', 0.215, 0.76, -0.018], ['f4a', 0.214, 0.714, -0.015]], { r: 0.0032, label: 'четвёртая, к безымянному' });
  // ----- sole of the foot -----
  mus('abdHall', [['foot', 0.08, 0.02, -0.05], ['foot', 0.074, 0.02, 0.04], ['foot', 0.072, 0.016, 0.12], ['toes', 0.07, 0.013, 0.15]], { r: 0.0065, belly: 0.4 });
  fan('fdb', { sheet: false, o: [['foot', 0.092, 0.017, -0.045], ['foot', 0.096, 0.017, -0.044]], via: [[['foot', 0.088, 0.017, 0.06], ['foot', 0.112, 0.017, 0.055]]], i: [['toes', 0.088, 0.008, 0.17], ['toes', 0.124, 0.008, 0.14]], n: 3, r: 0.0042, flat: 0.8, belly: 0.35 });
  mus('adm', [['foot', 0.108, 0.017, -0.045], ['foot', 0.122, 0.017, 0.04], ['toes', 0.128, 0.01, 0.128]], { r: 0.0055, belly: 0.4 });

  // ----- ligaments -----
  lig('iliofem', [['pelvis', 0.108, 0.998, 0.058], ['thigh', 0.14, 0.9, 0.022]], { r: 0.0038, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.03]], label: 'латеральный пучок' });
  lig('iliofem', [['pelvis', 0.108, 0.998, 0.058], ['thigh', 0.112, 0.882, 0.026]], { r: 0.0038, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.03]], label: 'медиальный пучок' });
  lig('pubofem', [['pelvis', 0.062, 0.918, 0.05], ['thigh', 0.106, 0.878, 0.014]], { r: 0.003, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.03]] });
  lig('ischiofem', [['pelvis', 0.078, 0.9, -0.04], ['thigh', 0.14, 0.92, -0.028]], { r: 0.003, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.03]] });
  lig('acl', [['thigh', 0.104, 0.522, -0.016], ['shank', 0.094, 0.492, 0.012]], { r: 0.0028 });
  lig('pcl', [['thigh', 0.089, 0.515, 0.012], ['shank', 0.098, 0.482, -0.024]], { r: 0.0028 });
  lig('mcl', [['thigh', 0.062, 0.526, -0.006], ['shank', 0.07, 0.455, -0.004]], { r: 0.0032, flat: 0.4 });
  lig('lcl', [['thigh', 0.131, 0.527, -0.008], ['shank', 0.131, 0.468, -0.022]], { r: 0.0026 });
  lig('patTend', [['shank', 0.098, 0.508, 0.036], ['shank', 0.098, 0.452, 0.03]], { r: 0.004, flat: 0.4 });
  lig('itb', [['thigh', 0.152, 0.868, 0.02], ['thigh', 0.142, 0.7, 0.002], ['thigh', 0.128, 0.56, 0.014], ['shank', 0.118, 0.468, 0.018]], { r: 0.0038, flat: 0.35, wraps: [KNEE_WRAP] });
  lig('atfl', [['shank', 0.12, 0.076, -0.01], ['foot', 0.11, 0.06, 0.016]], { r: 0.0024 });
  lig('cfl', [['shank', 0.12, 0.072, -0.016], ['foot', 0.112, 0.034, -0.026]], { r: 0.0024 });
  lig('ptfl', [['shank', 0.119, 0.074, -0.02], ['foot', 0.108, 0.05, -0.03]], { r: 0.0022 });
  lig('deltoidLig', [['shank', 0.076, 0.072, 0.002], ['foot', 0.08, 0.046, 0.03]], { r: 0.0026, label: 'передний пучок' });
  lig('deltoidLig', [['shank', 0.076, 0.07, -0.002], ['foot', 0.08, 0.042, -0.012]], { r: 0.0026, label: 'средний пучок' });
  lig('deltoidLig', [['shank', 0.076, 0.072, -0.006], ['foot', 0.082, 0.05, -0.026]], { r: 0.0026, label: 'задний пучок' });
  lig('plantar', [['foot', 0.094, 0.012, -0.062], ['foot', 0.078, 0.01, 0.06], ['foot', 0.074, 0.01, 0.13]], { r: 0.0024, flat: 0.5 });
  lig('plantar', [['foot', 0.094, 0.012, -0.062], ['foot', 0.1, 0.01, 0.062], ['foot', 0.1, 0.01, 0.134]], { r: 0.0024, flat: 0.5 });
  lig('plantar', [['foot', 0.094, 0.012, -0.062], ['foot', 0.118, 0.01, 0.055], ['foot', 0.126, 0.01, 0.116]], { r: 0.0024, flat: 0.5 });
  lig('coracoacr', [['scap', 0.154, 1.41, 0.028], ['scap', 0.178, 1.432, -0.006]], { r: 0.0026, flat: 0.4 });
  lig('coracohum', [['scap', 0.152, 1.412, 0.022], ['hum', 0.2, 1.412, 0.004]], { r: 0.0026, wraps: [[['hum', 0.185, 1.395, 0.0], 0.027]] });
  lig('ighl', [['scap', 0.158, 1.372, 0.004], ['hum', 0.19, 1.372, 0.016]], { r: 0.0026, wraps: [[['hum', 0.185, 1.395, 0.0], 0.027]], label: 'передний пучок' });
  lig('ighl', [['scap', 0.158, 1.372, -0.028], ['hum', 0.19, 1.372, -0.024]], { r: 0.0026, wraps: [[['hum', 0.185, 1.395, 0.0], 0.027]], label: 'задний пучок' });
  lig('ccl', [['scap', 0.15, 1.41, -0.028], ['clav', 0.142, 1.419, -0.002]], { r: 0.0024, label: 'конусовидная' });
  lig('ccl', [['scap', 0.153, 1.413, -0.016], ['clav', 0.156, 1.424, 0.004]], { r: 0.0024, label: 'трапециевидная' });
  lig('costoclav', [['clav', 0.038, 1.384, 0.063], ['auto', 0.042, 1.372, 0.062]], { r: 0.0024 });
  lig('ucl', [['hum', 0.178, 1.072, -0.006], ['ulna', 0.2, 1.058, -0.018]], { r: 0.0026 });
  lig('rcl', [['hum', 0.231, 1.072, -0.002], ['ulna', 0.222, 1.05, 0.012]], { r: 0.0024 });
  lig('nuchal', [['head', 0, 1.598, -0.09], ['auto', 0, 1.55, -0.1], ['auto', 0, 1.505, -0.108], ['auto', 0, 1.47, -0.118]], { r: 0.0026, flat: 0.4 });
  {
    const ALL = [], SSL = [];
    for (const v of VERT) { ALL.push(['auto', 0, v.y, v.c.z + v.R + 0.001]); }
    ALL.unshift(['pelvis', 0, 1.0, -0.038]);
    const sp = VERT.map(v => ['auto', 0, v.y - 0.004, v.c.z - v.R - (v.kind === 'L' ? 0.052 : v.kind === 'T' ? 0.05 : 0.03)]);
    lig('all', ALL, { r: 0.0028, flat: 0.4 });
    lig('ssl', [['pelvis', 0, 1.0, -0.1]].concat(sp.slice(0, 17)), { r: 0.0026, flat: 0.4 });
  }
  lig('sacrotub', [['pelvis', 0.024, 0.95, -0.1], ['pelvis', 0.062, 0.862, -0.06]], { r: 0.003, flat: 0.4 });
  lig('inguinal', [['pelvis', 0.125, 1.035, 0.058], ['pelvis', 0.022, 0.9, 0.072]], { r: 0.0026, flat: 0.4 });
  lig('iliolum', [['auto', 0.046, 1.042, -0.08], ['pelvis', 0.075, 1.065, -0.082]], { r: 0.0026 });

  // ----- joint capsules: fibres from a proximal ring to a distal ring, wrapping the joint -----
  function capsule(joint, o) {
    const basis = n => { const N = V(...n).normalize(); const a = Math.abs(N.y) < 0.9 ? V(0, 1, 0) : V(0, 0, 1); const u = a.clone().cross(N).normalize(); const v = N.clone().cross(u).normalize(); return [u, v]; };
    const [u1, v1] = basis(o.n1), [u2, v2] = basis(o.n2);
    const c1 = V(...o.c1), c2 = V(...o.c2);
    const n = o.n || 8;
    for (let k = 0; k < n; k++) {
      const th = (o.from || 0) * D + ((o.to || 360) - (o.from || 0)) * D * (k + 0.5) / n;
      const p1 = c1.clone().addScaledVector(u1, o.r1 * Math.cos(th)).addScaledVector(v1, o.r1 * Math.sin(th));
      const p2 = c2.clone().addScaledVector(u2, o.r2 * Math.cos(th)).addScaledVector(v2, o.r2 * Math.sin(th));
      strand('cap', joint, [[o.s1, p1.x, p1.y, p1.z], [o.s2, p2.x, p2.y, p2.z]], { r: 0.0022, flat: 0.5, wraps: o.wraps || [] });
    }
  }
  capsule('hip', { s1: 'pelvis', c1: [0.092, 0.929, 0.002], n1: [0.75, -0.3, 0.45], r1: 0.029, s2: 'thigh', c2: [0.128, 0.912, -0.004], n2: [0.88, -0.44, -0.14], r2: 0.024, n: 9, wraps: [[['thigh', 0.09, 0.93, 0.0], 0.0275]] });
  capsule('gh', { s1: 'scap', c1: [0.162, 1.388, -0.012], n1: [1, 0.05, 0.3], r1: 0.021, s2: 'hum', c2: [0.192, 1.366, 0.003], n2: [0.08, -1, 0.02], r2: 0.022, n: 9, wraps: [[['hum', 0.185, 1.395, 0.0], 0.0265]] });
  capsule('kn', { s1: 'thigh', c1: [0.097, 0.552, -0.006], n1: [0, 1, 0], r1: 0.031, s2: 'shank', c2: [0.097, 0.482, -0.002], n2: [0, 1, 0], r2: 0.034, n: 8, from: 40, to: 320, wraps: [KNEE_WRAP] });
  capsule('el', { s1: 'hum', c1: [0.205, 1.098, 0.0], n1: [0, 1, 0], r1: 0.023, s2: 'ulna', c2: [0.208, 1.044, -0.004], n2: [0, 1, 0], r2: 0.021, n: 8, wraps: [[['hum', 0.205, 1.07, 0.0], 0.021]] });
  capsule('an', { s1: 'shank', c1: [0.097, 0.098, -0.006], n1: [0, 1, 0], r1: 0.024, s2: 'foot', c2: [0.095, 0.048, -0.004], n2: [0, 1, 0], r2: 0.027, n: 8, wraps: [[['foot', 0.095, 0.064, -0.006], 0.021]] });
  capsule('wr', { s1: 'fore', c1: [0.213, 0.842, 0.008], n1: [0, 1, 0], r1: 0.02, s2: 'hand', c2: [0.213, 0.786, 0.006], n2: [0, 1, 0], r2: 0.02, n: 8, wraps: [[['hand', 0.213, 0.806, 0.006], 0.019]] });
  const UNITS = {
    // shoulder girdle
    trapUp: { short: 'Трапеция, верх', name: 'Трапециевидная, верхняя часть', latin: 'm. trapezius, pars descendens', zone: 'shoulder', fn: 'Поднимает лопатку и вместе с нижней частью и передней зубчатой разворачивает её вверх. Наклоняет голову назад.' },
    trapMid: { short: 'Трапеция, середина', name: 'Трапециевидная, средняя часть', latin: 'm. trapezius, pars transversa', zone: 'shoulder', fn: 'Сводит лопатки к позвоночнику.' },
    trapLow: { short: 'Трапеция, низ', name: 'Трапециевидная, нижняя часть', latin: 'm. trapezius, pars ascendens', zone: 'shoulder', fn: 'Опускает лопатку и помогает развернуть её вверх. Это то самое «плечи от ушей».' },
    lat: { short: 'Широчайшая', name: 'Широчайшая мышца спины', latin: 'm. latissimus dorsi', zone: 'shoulder', fn: 'Разгибает, приводит и вращает плечо внутрь. Ограничивает подъём рук над головой.' },
    pecMaj: { short: 'Большая грудная', name: 'Большая грудная мышца', latin: 'm. pectoralis major', zone: 'shoulder', fn: 'Приводит плечо и вращает его внутрь. Ключичная часть сгибает руку, грудинная опускает поднятую руку.' },
    pecMin: { deep: true, short: 'Малая грудная', name: 'Малая грудная мышца', latin: 'm. pectoralis minor', zone: 'shoulder', fn: 'Наклоняет лопатку вперёд и тянет её вниз. Короткая мешает раскрыть плечи.' },
    subclavius: { deep: true, short: 'Подключичная', name: 'Подключичная мышца', latin: 'm. subclavius', zone: 'shoulder', fn: 'Прижимает ключицу к грудине и опускает её.' },
    serratus: { short: 'Передняя зубчатая', name: 'Передняя зубчатая мышца', latin: 'm. serratus anterior', zone: 'shoulder', fn: 'Прижимает лопатку к рёбрам, уводит её вперёд и разворачивает вверх. Держит опору на руках.' },
    rhomb: { deep: true, short: 'Ромбовидные', name: 'Ромбовидные мышцы', latin: 'mm. rhomboidei', zone: 'shoulder', fn: 'Сводят и поднимают лопатки, поворачивают лопатку вниз.' },
    levScap: { deep: true, short: 'Поднимающая лопатку', name: 'Мышца, поднимающая лопатку', latin: 'm. levator scapulae', zone: 'shoulder', fn: 'Поднимает лопатку и поворачивает её вниз.' },
    deltAnt: { short: 'Дельта, передняя', name: 'Дельтовидная, передняя часть', latin: 'm. deltoideus, pars clavicularis', zone: 'shoulder', fn: 'Сгибает плечо и вращает его внутрь.' },
    deltMid: { short: 'Дельта, средняя', name: 'Дельтовидная, средняя часть', latin: 'm. deltoideus, pars acromialis', zone: 'shoulder', fn: 'Отводит плечо.' },
    deltPost: { short: 'Дельта, задняя', name: 'Дельтовидная, задняя часть', latin: 'm. deltoideus, pars spinalis', zone: 'shoulder', fn: 'Разгибает плечо и вращает его наружу.' },
    supra: { deep: true, short: 'Надостная', name: 'Надостная мышца', latin: 'm. supraspinatus', zone: 'shoulder', fn: 'Начинает отведение плеча и центрирует головку в суставе. Её сухожилие проходит под акромионом.' },
    infra: { short: 'Подостная', name: 'Подостная мышца', latin: 'm. infraspinatus', zone: 'shoulder', fn: 'Вращает плечо наружу и прижимает головку плеча к впадине.' },
    teresMin: { deep: true, short: 'Малая круглая', name: 'Малая круглая мышца', latin: 'm. teres minor', zone: 'shoulder', fn: 'Вращает плечо наружу вместе с подостной.' },
    subscap: { deep: true, short: 'Подлопаточная', name: 'Подлопаточная мышца', latin: 'm. subscapularis', zone: 'shoulder', fn: 'Вращает плечо внутрь и держит головку плеча спереди.' },
    teresMaj: { short: 'Большая круглая', name: 'Большая круглая мышца', latin: 'm. teres major', zone: 'shoulder', fn: 'Разгибает, приводит и вращает плечо внутрь, работает вместе с широчайшей.' },
    // arm
    tricLong: { short: 'Трицепс, длинная', name: 'Трёхглавая, длинная головка', latin: 'm. triceps brachii, caput longum', zone: 'arm', fn: 'Разгибает локоть. Начинается на лопатке, поэтому ещё и разгибает плечо.' },
    tricLat: { short: 'Трицепс, латеральная', name: 'Трёхглавая, латеральная головка', latin: 'm. triceps brachii, caput laterale', zone: 'arm', fn: 'Разгибает локоть.' },
    tricMed: { deep: true, short: 'Трицепс, медиальная', name: 'Трёхглавая, медиальная головка', latin: 'm. triceps brachii, caput mediale', zone: 'arm', fn: 'Разгибает локоть при любом положении плеча.' },
    anconeus: { short: 'Локтевая', name: 'Локтевая мышца', latin: 'm. anconeus', zone: 'arm', fn: 'Помогает трицепсу разгибать локоть и стабилизирует сустав.' },
    bicLong: { short: 'Бицепс, длинная', name: 'Двуглавая, длинная головка', latin: 'm. biceps brachii, caput longum', zone: 'arm', fn: 'Сгибает локоть и поворачивает ладонь вверх. Сухожилие проходит над головкой плеча.' },
    bicShort: { short: 'Бицепс, короткая', name: 'Двуглавая, короткая головка', latin: 'm. biceps brachii, caput breve', zone: 'arm', fn: 'Сгибает локоть и плечо, поворачивает ладонь вверх.' },
    coracobr: { deep: true, short: 'Клювовидно-плечевая', name: 'Клювовидно-плечевая мышца', latin: 'm. coracobrachialis', zone: 'arm', fn: 'Сгибает и приводит плечо.' },
    brachialis: { deep: true, short: 'Плечевая', name: 'Плечевая мышца', latin: 'm. brachialis', zone: 'arm', fn: 'Главный сгибатель локтя при любом положении предплечья.' },
    brachiorad: { short: 'Плечелучевая', name: 'Плечелучевая мышца', latin: 'm. brachioradialis', zone: 'arm', fn: 'Сгибает локоть в среднем положении предплечья.' },
    pronTeres: { short: 'Круглый пронатор', name: 'Круглый пронатор', latin: 'm. pronator teres', zone: 'arm', fn: 'Поворачивает ладонь вниз и помогает сгибать локоть.' },
    pronQuad: { deep: true, short: 'Квадратный пронатор', name: 'Квадратный пронатор', latin: 'm. pronator quadratus', zone: 'arm', fn: 'Поворачивает ладонь вниз и скрепляет лучевую и локтевую кости у запястья.' },
    supinator: { deep: true, short: 'Супинатор', name: 'Мышца-супинатор', latin: 'm. supinator', zone: 'arm', fn: 'Поворачивает ладонь вверх.' },
    wristFlex: { short: 'Сгибатели запястья', name: 'Сгибатели запястья', latin: 'mm. flexores carpi', zone: 'arm', fn: 'Сгибают запястье и отводят кисть в стороны.' },
    palmaris: { short: 'Длинная ладонная', name: 'Длинная ладонная мышца', latin: 'm. palmaris longus', zone: 'arm', fn: 'Натягивает ладонный апоневроз и слабо сгибает запястье. Есть не у всех.' },
    fingerFlex: { deep: true, short: 'Сгибатели пальцев', name: 'Сгибатели пальцев', latin: 'mm. flexores digitorum', zone: 'arm', fn: 'Сгибают пальцы и помогают сгибать запястье. Это главный хват.' },
    wristExt: { short: 'Разгибатели запястья', name: 'Разгибатели запястья и пальцев', latin: 'mm. extensores carpi et digitorum', zone: 'arm', fn: 'Разгибают запястье и пальцы.' },
    apl: { deep: true, short: 'Отводящая большой палец', name: 'Длинная мышца, отводящая большой палец кисти', latin: 'm. abductor pollicis longus', zone: 'arm', fn: 'Отводит большой палец и отводит кисть в сторону лучевой кости.' },
    thenar: { short: 'Возвышение большого пальца', name: 'Мышцы возвышения большого пальца', latin: 'mm. thenaris', zone: 'arm', fn: 'Отводят, сгибают и противопоставляют большой палец.' },
    hypothenar: { short: 'Возвышение мизинца', name: 'Мышцы возвышения мизинца', latin: 'mm. hypothenaris', zone: 'arm', fn: 'Отводят и сгибают мизинец.' },
    interossei: { deep: true, short: 'Межкостные кисти', name: 'Тыльные межкостные мышцы кисти', latin: 'mm. interossei dorsales', zone: 'arm', fn: 'Разводят пальцы веером и сгибают их в основании.' },
    // neck and trunk
    scm: { short: 'Грудино-ключично-сосцевидная', name: 'Грудино-ключично-сосцевидная мышца', latin: 'm. sternocleidomastoideus', zone: 'trunk', fn: 'Наклоняет и поворачивает голову, при опоре поднимает грудину на вдохе.' },
    scalene: { deep: true, short: 'Лестничные', name: 'Лестничные мышцы', latin: 'mm. scaleni', zone: 'trunk', fn: 'Наклоняют шею и поднимают первые рёбра на вдохе.' },
    longusColli: { deep: true, short: 'Длинная шеи', name: 'Длинная мышца шеи', latin: 'm. longus colli', zone: 'trunk', fn: 'Сгибает шею спереди и держит её от переразгибания.' },
    splenius: { deep: true, short: 'Ременная', name: 'Ременная мышца головы', latin: 'm. splenius capitis', zone: 'trunk', fn: 'Разгибает и поворачивает голову и шею.' },
    semispinalis: { deep: true, short: 'Полуостистая головы', name: 'Полуостистая мышца головы', latin: 'm. semispinalis capitis', zone: 'trunk', fn: 'Разгибает голову и верх шеи.' },
    erector: { deep: true, short: 'Разгибатели спины', name: 'Мышца, выпрямляющая позвоночник', latin: 'm. erector spinae', zone: 'trunk', fn: 'Разгибает позвоночник и держит его против силы тяжести.' },
    multifidus: { deep: true, short: 'Многораздельная', name: 'Многораздельная мышца', latin: 'mm. multifidi', zone: 'trunk', fn: 'Держит каждый позвонок по отдельности и тонко стабилизирует позвоночник.' },
    ql: { deep: true, short: 'Квадратная поясницы', name: 'Квадратная мышца поясницы', latin: 'm. quadratus lumborum', zone: 'trunk', fn: 'Наклоняет поясницу в сторону, держит 12-е ребро и таз.' },
    rectAbd: { short: 'Прямая живота', name: 'Прямая мышца живота', latin: 'm. rectus abdominis', zone: 'trunk', fn: 'Сгибает позвоночник, тянет рёбра к тазу и подкручивает таз.' },
    extObl: { short: 'Наружная косая', name: 'Наружная косая мышца живота', latin: 'm. obliquus externus abdominis', zone: 'trunk', fn: 'Сгибает и поворачивает корпус, держит нижние рёбра.' },
    intObl: { deep: true, short: 'Внутренняя косая', name: 'Внутренняя косая мышца живота', latin: 'm. obliquus internus abdominis', zone: 'trunk', fn: 'Сгибает и поворачивает корпус, вместе с поперечной стягивает живот.' },
    transAbd: { deep: true, short: 'Поперечная живота', name: 'Поперечная мышца живота', latin: 'm. transversus abdominis', zone: 'trunk', fn: 'Стягивает живот как пояс и стабилизирует поясницу перед любым движением.' },
    psoas: { deep: true, short: 'Поясничная', name: 'Большая поясничная мышца', latin: 'm. psoas major', zone: 'trunk', fn: 'Сгибает бедро и тянет поясницу вперёд, наклоняя таз.' },
    iliacus: { deep: true, short: 'Подвздошная', name: 'Подвздошная мышца', latin: 'm. iliacus', zone: 'trunk', fn: 'Сгибает бедро и наклоняет таз вперёд.' },
    // hip and thigh
    gluteMax: { short: 'Большая ягодичная', name: 'Большая ягодичная мышца', latin: 'm. gluteus maximus', zone: 'leg', fn: 'Разгибает бедро, вращает его наружу и подкручивает таз.' },
    gluteMed: { short: 'Средняя ягодичная', name: 'Средняя ягодичная мышца', latin: 'm. gluteus medius', zone: 'leg', fn: 'Отводит бедро и держит таз ровно. Передние волокна вращают бедро внутрь, задние наружу.' },
    gluteMin: { deep: true, short: 'Малая ягодичная', name: 'Малая ягодичная мышца', latin: 'm. gluteus minimus', zone: 'leg', fn: 'Отводит бедро, вращает его внутрь и стабилизирует сустав.' },
    piriformis: { deep: true, short: 'Грушевидная', name: 'Грушевидная мышца', latin: 'm. piriformis', zone: 'leg', fn: 'Вращает бедро наружу, а при глубоком сгибании бедра отводит его.' },
    obtInt: { deep: true, short: 'Внутренняя запирательная', name: 'Внутренняя запирательная мышца', latin: 'm. obturator internus', zone: 'leg', fn: 'Вращает бедро наружу и прижимает его головку к впадине.' },
    gemelli: { deep: true, short: 'Близнецовые', name: 'Близнецовые мышцы', latin: 'mm. gemelli', zone: 'leg', fn: 'Помогают внутренней запирательной вращать бедро наружу.' },
    quadFem: { deep: true, short: 'Квадратная бедра', name: 'Квадратная мышца бедра', latin: 'm. quadratus femoris', zone: 'leg', fn: 'Вращает бедро наружу и приводит его.' },
    obtExt: { deep: true, short: 'Наружная запирательная', name: 'Наружная запирательная мышца', latin: 'm. obturator externus', zone: 'leg', fn: 'Вращает бедро наружу и держит головку бедра снизу.' },
    tfl: { short: 'Напрягатель широкой фасции', name: 'Напрягатель широкой фасции', latin: 'm. tensor fasciae latae', zone: 'leg', fn: 'Сгибает, отводит и вращает бедро внутрь, натягивает подвздошно-большеберцовый тракт.' },
    sartorius: { short: 'Портняжная', name: 'Портняжная мышца', latin: 'm. sartorius', zone: 'leg', fn: 'Сгибает, отводит и вращает бедро наружу, сгибает колено.' },
    pectineus: { short: 'Гребенчатая', name: 'Гребенчатая мышца', latin: 'm. pectineus', zone: 'leg', fn: 'Сгибает и приводит бедро.' },
    addLong: { short: 'Длинная приводящая', name: 'Длинная приводящая мышца', latin: 'm. adductor longus', zone: 'leg', fn: 'Приводит бедро и помогает его сгибать.' },
    addBrev: { deep: true, short: 'Короткая приводящая', name: 'Короткая приводящая мышца', latin: 'm. adductor brevis', zone: 'leg', fn: 'Приводит и сгибает бедро.' },
    addMag: { deep: true, short: 'Большая приводящая', name: 'Большая приводящая мышца', latin: 'm. adductor magnus', zone: 'leg', fn: 'Приводит бедро. Седалищная часть разгибает бедро, как мышца задней поверхности бедра.' },
    gracilis: { short: 'Тонкая', name: 'Тонкая мышца', latin: 'm. gracilis', zone: 'leg', fn: 'Приводит бедро и сгибает колено.' },
    bicFemL: { short: 'Двуглавая бедра, длинная', name: 'Двуглавая мышца бедра, длинная головка', latin: 'm. biceps femoris, caput longum', zone: 'leg', fn: 'Разгибает бедро, сгибает колено и наклоняет таз назад.' },
    bicFemS: { deep: true, short: 'Двуглавая бедра, короткая', name: 'Двуглавая мышца бедра, короткая головка', latin: 'm. biceps femoris, caput breve', zone: 'leg', fn: 'Сгибает колено и вращает голень наружу. Через тазобедренный сустав не идёт.' },
    semiT: { short: 'Полусухожильная', name: 'Полусухожильная мышца', latin: 'm. semitendinosus', zone: 'leg', fn: 'Разгибает бедро, сгибает колено и вращает голень внутрь.' },
    semiM: { deep: true, short: 'Полуперепончатая', name: 'Полуперепончатая мышца', latin: 'm. semimembranosus', zone: 'leg', fn: 'Разгибает бедро, сгибает колено и вращает голень внутрь.' },
    rectFem: { short: 'Прямая бедра', name: 'Прямая мышца бедра', latin: 'm. rectus femoris', zone: 'leg', fn: 'Разгибает колено и сгибает бедро.' },
    vastLat: { short: 'Широкая латеральная', name: 'Латеральная широкая мышца бедра', latin: 'm. vastus lateralis', zone: 'leg', fn: 'Разгибает колено.' },
    vastMed: { short: 'Широкая медиальная', name: 'Медиальная широкая мышца бедра', latin: 'm. vastus medialis', zone: 'leg', fn: 'Разгибает колено и держит надколенник по центру.' },
    vastInt: { deep: true, short: 'Широкая промежуточная', name: 'Промежуточная широкая мышца бедра', latin: 'm. vastus intermedius', zone: 'leg', fn: 'Разгибает колено.' },
    // lower leg and foot
    popliteus: { deep: true, short: 'Подколенная', name: 'Подколенная мышца', latin: 'm. popliteus', zone: 'leg', fn: 'Отпирает выпрямленное колено, поворачивая бедро наружу относительно голени.' },
    gastroM: { short: 'Икроножная, медиальная', name: 'Икроножная мышца, медиальная головка', latin: 'm. gastrocnemius, caput mediale', zone: 'leg', fn: 'Поднимает пятку и помогает сгибать колено. Идёт через колено и голеностоп.' },
    gastroL: { short: 'Икроножная, латеральная', name: 'Икроножная мышца, латеральная головка', latin: 'm. gastrocnemius, caput laterale', zone: 'leg', fn: 'Поднимает пятку и помогает сгибать колено. Идёт через колено и голеностоп.' },
    plantaris: { deep: true, short: 'Подошвенная', name: 'Подошвенная мышца', latin: 'm. plantaris', zone: 'leg', fn: 'Слабо помогает икроножной. Длинное тонкое сухожилие идёт к пятке.' },
    soleus: { deep: true, short: 'Камбаловидная', name: 'Камбаловидная мышца', latin: 'm. soleus', zone: 'leg', fn: 'Поднимает пятку при любом положении колена и держит голень над стопой.' },
    tibAnt: { short: 'Передняя большеберцовая', name: 'Передняя большеберцовая мышца', latin: 'm. tibialis anterior', zone: 'leg', fn: 'Тянет стопу на себя и поднимает её внутренний край.' },
    edl: { short: 'Разгибатель пальцев стопы', name: 'Длинный разгибатель пальцев стопы', latin: 'm. extensor digitorum longus', zone: 'leg', fn: 'Разгибает пальцы стопы и тянет стопу на себя.' },
    ehl: { deep: true, short: 'Разгибатель большого пальца', name: 'Длинный разгибатель большого пальца', latin: 'm. extensor hallucis longus', zone: 'leg', fn: 'Разгибает большой палец стопы и тянет стопу на себя.' },
    fibL: { short: 'Длинная малоберцовая', name: 'Длинная малоберцовая мышца', latin: 'm. fibularis longus', zone: 'leg', fn: 'Опускает первую плюсневую кость, поворачивает стопу наружу и держит поперечный свод.' },
    fibB: { deep: true, short: 'Короткая малоберцовая', name: 'Короткая малоберцовая мышца', latin: 'm. fibularis brevis', zone: 'leg', fn: 'Поворачивает стопу наружу и помогает подошвенному сгибанию.' },
    tibPost: { deep: true, short: 'Задняя большеберцовая', name: 'Задняя большеберцовая мышца', latin: 'm. tibialis posterior', zone: 'leg', fn: 'Держит внутренний свод стопы и поворачивает стопу внутрь.' },
    fhl: { deep: true, short: 'Сгибатель большого пальца', name: 'Длинный сгибатель большого пальца', latin: 'm. flexor hallucis longus', zone: 'leg', fn: 'Сгибает большой палец стопы и прижимает его к полу.' },
    fdl: { deep: true, short: 'Сгибатель пальцев стопы', name: 'Длинный сгибатель пальцев стопы', latin: 'm. flexor digitorum longus', zone: 'leg', fn: 'Сгибает пальцы стопы.' },
    abdHall: { short: 'Отводящая большой палец стопы', name: 'Мышца, отводящая большой палец стопы', latin: 'm. abductor hallucis', zone: 'leg', fn: 'Отводит большой палец стопы и держит внутренний свод.' },
    fdb: { short: 'Короткий сгибатель пальцев', name: 'Короткий сгибатель пальцев стопы', latin: 'm. flexor digitorum brevis', zone: 'leg', fn: 'Сгибает пальцы стопы и прижимает их к полу.' },
    adm: { short: 'Отводящая мизинец стопы', name: 'Мышца, отводящая мизинец стопы', latin: 'm. abductor digiti minimi pedis', zone: 'leg', fn: 'Отводит мизинец стопы и держит наружный край стопы.' },
  };
  const LIGS = {
    iliofem: { name: 'Подвздошно-бедренная связка', latin: 'lig. iliofemorale', joint: 'hip', role: 'Самая прочная связка тела. Натягивается при разгибании бедра, при сгибании расслабляется.' },
    pubofem: { name: 'Лобково-бедренная связка', latin: 'lig. pubofemorale', joint: 'hip', role: 'Ограничивает отведение и разгибание бедра.' },
    ischiofem: { name: 'Седалищно-бедренная связка', latin: 'lig. ischiofemorale', joint: 'hip', role: 'Ограничивает внутреннюю ротацию и натягивается при сгибании бедра.' },
    acl: { name: 'Передняя крестообразная связка', latin: 'lig. cruciatum anterius', joint: 'kn', role: 'Не даёт голени сместиться вперёд. Перегружается при переразгибании колена.' },
    pcl: { name: 'Задняя крестообразная связка', latin: 'lig. cruciatum posterius', joint: 'kn', role: 'Не даёт голени сместиться назад.' },
    mcl: { name: 'Медиальная коллатеральная связка', latin: 'lig. collaterale tibiale', joint: 'kn', role: 'Стабилизирует колено с внутренней стороны, натянута при прямом колене.' },
    lcl: { name: 'Латеральная коллатеральная связка', latin: 'lig. collaterale fibulare', joint: 'kn', role: 'Стабилизирует колено снаружи.' },
    patTend: { name: 'Связка надколенника', latin: 'lig. patellae', joint: 'kn', role: 'Продолжение сухожилия четырёхглавой мышцы: через неё квадрицепс разгибает колено.' },
    itb: { name: 'Подвздошно-большеберцовый тракт', latin: 'tractus iliotibialis', joint: 'hip', role: 'Плотная полоса фасции от таза до голени. В неё вплетаются напрягатель широкой фасции и верхняя часть большой ягодичной.' },
    atfl: { name: 'Передняя таранно-малоберцовая связка', latin: 'lig. talofibulare anterius', joint: 'an', role: 'Самая уязвимая связка голеностопа. Натягивается при подошвенном сгибании.' },
    cfl: { name: 'Пяточно-малоберцовая связка', latin: 'lig. calcaneofibulare', joint: 'an', role: 'Натягивается при тыльном сгибании и повороте стопы внутрь.' },
    ptfl: { name: 'Задняя таранно-малоберцовая связка', latin: 'lig. talofibulare posterius', joint: 'an', role: 'Ограничивает тыльное сгибание.' },
    deltoidLig: { name: 'Дельтовидная связка', latin: 'lig. deltoideum', joint: 'an', role: 'Веер связок с внутренней стороны голеностопа, держит стопу от заваливания внутрь.' },
    plantar: { name: 'Подошвенный апоневроз', latin: 'aponeurosis plantaris', joint: 'an', role: 'Натягивается, когда пальцы стопы разогнуты, и держит продольный свод.' },
    coracoacr: { name: 'Клювовидно-акромиальная связка', latin: 'lig. coracoacromiale', joint: 'gh', role: 'Образует «крышу» над плечевым суставом. Под ней проходит сухожилие надостной.' },
    coracohum: { name: 'Клювовидно-плечевая связка', latin: 'lig. coracohumerale', joint: 'gh', role: 'Укрепляет верх капсулы и натягивается при наружной ротации.' },
    ighl: { name: 'Нижняя плечесуставная связка', latin: 'lig. glenohumerale inferius', joint: 'gh', role: 'Главный стабилизатор плеча при поднятой руке: держит головку плеча снизу.' },
    ccl: { name: 'Клювовидно-ключичная связка', latin: 'lig. coracoclaviculare', joint: 'ac', role: 'Подвешивает лопатку на ключице. Через неё разворот лопатки передаётся ключице.' },
    costoclav: { name: 'Рёберно-ключичная связка', latin: 'lig. costoclaviculare', joint: 'sc', role: 'Ограничивает подъём ключицы.' },
    ucl: { name: 'Локтевая коллатеральная связка', latin: 'lig. collaterale ulnare', joint: 'el', role: 'Стабилизирует локоть с внутренней стороны.' },
    rcl: { name: 'Лучевая коллатеральная связка', latin: 'lig. collaterale radiale', joint: 'el', role: 'Стабилизирует локоть снаружи.' },
    nuchal: { name: 'Выйная связка', latin: 'lig. nuchae', joint: 'ct', role: 'Натягивается при сгибании шеи и помогает держать голову.' },
    all: { name: 'Передняя продольная связка', latin: 'lig. longitudinale anterius', joint: 'spine', role: 'Идёт по передней поверхности позвонков и натягивается при разгибании позвоночника.' },
    ssl: { name: 'Надостистая связка', latin: 'lig. supraspinale', joint: 'spine', role: 'Идёт по верхушкам остистых отростков и натягивается при сгибании позвоночника.' },
    sacrotub: { name: 'Крестцово-бугорная связка', latin: 'lig. sacrotuberale', joint: 'hip', role: 'Связывает крестец и седалищный бугор. Через неё натяжение задней поверхности бедра передаётся на таз.' },
    inguinal: { name: 'Паховая связка', latin: 'lig. inguinale', joint: 'hip', role: 'Нижний край апоневроза наружной косой мышцы.' },
    iliolum: { name: 'Подвздошно-поясничная связка', latin: 'lig. iliolumbale', joint: 'ls', role: 'Связывает L5 с тазом и стабилизирует пояснично-крестцовый переход.' },
  };

  const CAP_NAMES = { hip: 'Капсула тазобедренного сустава', gh: 'Капсула плечевого сустава', kn: 'Капсула коленного сустава', el: 'Капсула локтевого сустава', an: 'Капсула голеностопа', wr: 'Капсула лучезапястного сустава' };

  function strandInstances() {
    const out = [];
    const mirP = (base, p, side) => ({ seg: sideName(base, side), p: V(side === 'R' ? -p.x : p.x, p.y, p.z) });
    for (const side of ['L', 'R']) {
      STR.forEach((m, idx) => {
        const wraps = (m.wraps || []).map(([[n, x, y, z], r, depth]) => ({ seg: sideName(n, side), c: V(side === 'R' ? -x : x, y, armZ(n, x, z)), r: Array.isArray(r) ? V(r[0], r[1], r[2]) : V(r, r, r), depth: depth || 1 }));
        out.push({
          id: m.kind + '_' + m.unit + '_' + idx + '_' + side, kind: m.kind, unit: m.unit, side, idx: m.idx || 0, n: m.n,
          r: m.r, flat: m.flat ?? 1, belly: m.belly ?? 0.5, dips: m.dips || null, label: m.label || null,
          pts: m.pts.map(q => mirP(q.base, q.p, side)),
          flatRef: m.flatRef ? mirP(m.flatRef.base, m.flatRef.p, side) : null,
          flatRefIns: m.flatRefIns ? mirP(m.flatRefIns.base, m.flatRefIns.p, side) : null,
          sheet: !!m.sheet,
          wraps,
        });
      });
    }
    return out;
  }

  // ---------- joints ----------
  const JOINTS = [];
  function jnt(o) { JOINTS.push(o); }
  jnt({ id: 'ls', seg: 'lumA', at: [0, 1.020, -0.058], r: 0.016, name: 'Пояснично-крестцовый сегмент L5–S1', type: 'spine', region: 'lumbar' });
  jnt({ id: 'l34', seg: 'lumB', at: S.lumB.J.toArray(), r: 0.016, name: 'Поясница, L3–L4', type: 'spine', region: 'lumbar' });
  jnt({ id: 'tl', seg: 'thA', at: S.thA.J.toArray(), r: 0.015, name: 'Грудопоясничный переход Th12–L1', type: 'spine', region: 'thoracic' });
  jnt({ id: 't89', seg: 'thB', at: S.thB.J.toArray(), r: 0.014, name: 'Грудной отдел, Th8–Th9', type: 'spine', region: 'thoracic' });
  jnt({ id: 't45', seg: 'thC', at: S.thC.J.toArray(), r: 0.013, name: 'Грудной отдел, Th4–Th5', type: 'spine', region: 'thoracic' });
  jnt({ id: 'ct', seg: 'neckA', at: S.neckA.J.toArray(), r: 0.012, name: 'Шейно-грудной переход C7–Th1', type: 'spine', region: 'cervical' });
  jnt({ id: 'c45', seg: 'neckB', at: S.neckB.J.toArray(), r: 0.011, name: 'Шея, C4–C5', type: 'spine', region: 'cervical' });
  jnt({ id: 'ao', seg: 'head', at: S.head.J.toArray(), r: 0.011, name: 'Атлантозатылочный сустав', type: 'spine', region: 'cervical' });
  const JL = [
    { id: 'sc', seg: 'clav', r: 0.009, name: 'Грудино-ключичный сустав', type: 'sc' },
    { id: 'ac', seg: 'scap', r: 0.008, name: 'Акромиально-ключичный сустав', type: 'ac' },
    { id: 'st', seg: 'scap', at: [0.11, 1.34, -0.084], r: 0.03, name: 'Лопатка на грудной клетке', type: 'st' },
    { id: 'gh', seg: 'hum', r: 0.028, name: 'Плечевой сустав', type: 'gh' },
    { id: 'el', seg: 'fore', r: 0.018, name: 'Локоть и предплечье', type: 'elbow' },
    { id: 'wr', seg: 'hand', r: 0.015, name: 'Лучезапястный сустав', type: 'wrist' },
    { id: 'hip', seg: 'thigh', r: 0.028, name: 'Тазобедренный сустав', type: 'hip' },
    { id: 'kn', seg: 'shank', r: 0.024, name: 'Коленный сустав', type: 'knee' },
    { id: 'an', seg: 'foot', r: 0.018, name: 'Голеностоп', type: 'ankle' },
    { id: 'mtp', seg: 'toes', r: 0.012, name: 'Плюснефаланговые суставы', type: 'toes' },
  ];
  for (const side of ['L', 'R']) for (const j of JL) {
    const sn = sideName(j.seg, side);
    const at0 = j.at ? AZ(j.seg, j.at) : null;
    const at = at0 ? (side === 'R' ? mir(at0) : at0) : S[sn].J.toArray();
    jnt(Object.assign({}, j, { base: j.id, id: j.id + '_' + side, seg: sn, at, side }));
  }
  for (const j of JOINTS) if (!j.base) j.base = j.id;
  const REGION = { lumbar: { ids: ['ls', 'l34'], name: 'Поясничный отдел', ext: 30, flex: 60 }, thoracic: { ids: ['tl', 't89', 't45'], name: 'Грудной отдел', ext: 25, flex: 45 }, cervical: { ids: ['ct', 'c45', 'ao'], name: 'Шейный отдел', ext: 70, flex: 50 } };
  // ---------- scapular frame (used by the pose builder and the joint analysis) ----------
  const sSup = V(0.077, 1.43, -0.092), sInf = V(...AZ('scap', [0.091, 1.25, -0.104])), sGlen = V(...AZ('scap', [0.16, 1.388, -0.012]));
  const nScap = sSup.clone().sub(sInf).cross(sGlen.clone().sub(sInf)).normalize();
  if (nScap.z > 0) nScap.negate();
  nScap.y = 0; nScap.normalize();
  const uScap = V(-nScap.z, 0, nScap.x).normalize();
  const SCAP_UP = nScap.clone().negate();
  const SCAP_POST = uScap.clone().negate();
  const SCAP_PIVOT = V(0.12, 1.395, -0.085);
  function scapulaPose(up, post, prot, pivot, clavRot, elev) {
    const PV = pivot || SCAP_PIVOT;
    const Rup = qAx(SCAP_UP, up).multiply(qAx(SCAP_POST, post));
    const Rprot = qAx(AX.Y, -prot);
    const Rst = Rprot.clone().multiply(Rup);
    const c = V(0, 0, -0.01);
    const lift = V(0, elev || 0, 0);
    const tr = X => X.clone().sub(PV).applyQuaternion(Rup).add(PV).sub(c).applyQuaternion(Rprot).add(c).add(lift);
    const SC = S.clav_L.J, AC0 = S.scap_L.J;
    const axis0 = AC0.clone().sub(SC).normalize();
    const Rc0 = new THREE.Quaternion().setFromUnitVectors(axis0, tr(AC0).sub(SC).normalize());
    const Rc = Rc0.clone().multiply(qAx(axis0, -(clavRot || 0)));
    return { clav: Rc, scap: Rc.clone().invert().multiply(Rst), Rst, tr };
  }

  // ---------- skin landmarks: floor contact, framing, cue checks ----------
  const LANDMARKS = [];
  // soft tissue: how far a landmark can sink in under body weight (along the segment's z axis)
  const SOFT = { 'живот': -0.018, 'нижние рёбра': -0.006, 'грудь': -0.008, 'бёдра': -0.008, 'таз': -0.004, 'голени': -0.002, 'ягодицы': 0.012, 'икры': 0.008, 'задняя поверхность бёдер': 0.008 };
  function lmk(region, raw, tag) {
    const r = resolveRaw(raw);
    const central = Math.abs(r.p.x) < 1e-6;
    const dz = SOFT[region] || 0;
    for (const side of ['L', 'R']) {
      if (side === 'R' && central) continue;
      const p = V(side === 'R' ? -r.p.x : r.p.x, r.p.y, r.p.z);
      LANDMARKS.push({ region, tag: tag || null, seg: sideName(r.base, side), side: central ? 'C' : side, p, pc: p.clone().add(V(0, 0, dz)), soft: Math.abs(dz) });
    }
  }
  lmk('пятки', ['foot', 0.094, 0.005, -0.045], 'heel'); lmk('пятки', ['foot', 0.1, 0.012, -0.064]);
  lmk('подушечки стоп', ['foot', 0.082, 0.006, 0.135], 'ball'); lmk('подушечки стоп', ['foot', 0.124, 0.007, 0.114]);
  lmk('наружный край стопы', ['foot', 0.118, 0.01, 0.03]);
  lmk('пальцы стоп', ['toes', 0.074, 0.004, 0.19], 'toeTip'); lmk('пальцы стоп', ['toes', 0.1, 0.004, 0.178], 'toePad'); lmk('пальцы стоп', ['toes', 0.126, 0.004, 0.152]);
  lmk('пальцы стоп', ['toes', 0.064, 0.012, 0.188], 'hallux');
  lmk('тыл стоп', ['toes', 0.076, 0.022, 0.19], 'toeTop'); lmk('тыл стоп', ['toes', 0.1, 0.02, 0.17]);
  lmk('тыл стоп', ['foot', 0.088, 0.048, 0.1], 'instep'); lmk('тыл стоп', ['foot', 0.092, 0.066, 0.05]); lmk('тыл стоп', ['foot', 0.096, 0.085, 0.02]);
  for (const [y, z] of [[0.12, 0.035], [0.2, 0.036], [0.3, 0.038], [0.4, 0.042]]) lmk('голени', ['shank', 0.1, y, z]);
  for (const [y, z] of [[0.12, -0.05], [0.3, -0.085], [0.42, -0.07]]) lmk('икры', ['shank', 0.1, y, z]);
  lmk('колени', ['shank', 0.098, 0.528, 0.052], 'patella'); lmk('колени', ['shank', 0.098, 0.455, 0.048]);
  for (const [y, z] of [[0.6, 0.085], [0.7, 0.092], [0.8, 0.095], [0.88, 0.09]]) lmk('бёдра', ['thigh', 0.11, y, z]);
  for (const [y, z] of [[0.6, -0.085], [0.75, -0.095]]) lmk('задняя поверхность бёдер', ['thigh', 0.1, y, z]);
  lmk('ягодицы', ['pelvis', 0.07, 0.9, -0.14]); lmk('ягодицы', ['pelvis', 0.07, 0.98, -0.14]);
  lmk('таз', ['pelvis', 0, 0.875, 0.098], 'pubis'); lmk('таз', ['pelvis', 0.04, 0.9, 0.105]);
  lmk('таз', ['pelvis', 0.125, 1.035, 0.08], 'asis');
  for (const [y, z] of [[0.95, 0.11], [1.0, 0.116], [1.05, 0.119], [1.1, 0.12], [1.15, 0.12], [1.2, 0.122], [1.25, 0.12], [1.3, 0.118], [1.35, 0.11], [1.4, 0.085]]) {
    const reg = y < 1.18 ? 'живот' : y < 1.27 ? 'нижние рёбра' : 'грудь';
    const tag = y === 1.4 ? 'chestTop' : null;
    lmk(reg, ['auto', 0, y, z], tag); lmk(reg, ['auto', 0.07, y, z - 0.012]);
  }
  for (const y of [1.05, 1.2, 1.35]) lmk('спина', ['auto', 0, y, y < 1.2 ? -0.12 : -0.15]);
  lmk('голова', ['head', 0, 1.755, 0.0], 'crown'); lmk('голова', ['head', 0, 1.7, 0.095]); lmk('голова', ['head', 0, 1.625, 0.112]); lmk('голова', ['head', 0, 1.56, 0.088], 'chin');
  lmk('голова', ['head', 0, 1.69, -0.095]); lmk('голова', ['head', 0, 1.6, -0.09]);
  lmk('ладони', ['hand', 0.197, 0.795, 0.006], 'palmHeel'); lmk('ладони', ['hand', 0.196, 0.75, 0.008], 'palm'); lmk('ладони', ['hand', 0.198, 0.735, -0.022]); lmk('ладони', ['hand', 0.198, 0.74, 0.03]); lmk('ладони', ['hand', 0.198, 0.79, -0.018]); lmk('ладони', ['hand', 0.198, 0.785, 0.026]);
  MC.forEach(([b0, b1, b2], i) => {
    const n = 'f' + (i + 2);
    const tip = lerp3(b1, b2, 0.97), mid = lerp3(b1, b2, 0.3);
    lmk('пальцы рук', [n + 'c', tip[0] - 0.007, tip[1], tip[2]], 'tip' + (i + 2));
    lmk('пальцы рук', [n + 'a', mid[0] - 0.008, mid[1], mid[2]]);
    lmk('кулаки', [n + 'a', b1[0] + 0.008, b1[1], b1[2]], 'knuckle' + (i + 2));
  });
  lmk('пальцы рук', ['f1b', 0.197, 0.714, 0.058], 'tip1');
  lmk('кисти', ['hand', 0.208, 0.78, -0.034]); lmk('кисти', ['hand', 0.21, 0.73, -0.036], 'handEdge');
  lmk('предплечья', ['ulna', 0.205, 1.045, -0.05], 'foreElbow'); lmk('предплечья', ['ulna', 0.208, 0.95, -0.04]); lmk('предплечья', ['ulna', 0.21, 0.865, -0.028], 'foreWrist');
  lmk('предплечья', ['fore', 0.2, 0.95, 0.01]); lmk('предплечья', ['fore', 0.226, 0.95, 0.01]);
  lmk('локти', ['ulna', 0.203, 1.095, -0.045], 'olecranon');
  lmk('плечи', ['hum', 0.22, 1.33, 0.0]); lmk('плечи', ['hum', 0.21, 1.2, 0.03]); lmk('плечи', ['scap', 0.19, 1.445, -0.015]);
  const lmkTag = (tag, side) => LANDMARKS.find(l => l.tag === tag && (l.side === side || l.side === 'C'));
  const lmkRegion = reg => LANDMARKS.filter(l => l.region === reg);

  // ---------- segment masses (fraction of body mass, adult male) ----------
  const MASS = [];
  const massPt = (seg, p, m) => MASS.push({ seg, p, m });
  massPt('head', V(0, 1.665, 0.01), 0.0694);
  massPt('thB', V(0, 1.31, 0.0), 0.1596);
  massPt('lumB', V(0, 1.12, 0.02), 0.1633);
  massPt('pelvis', V(0, 0.975, 0.0), 0.1117);
  for (const side of ['L', 'R']) {
    const s = n => sideName(n, side);
    const mid = (a, b, t) => S[s(a)].J.clone().lerp(S[s(b)].J, t);
    massPt(s('hum'), mid('hum', 'fore', 0.45), 0.0271);
    massPt(s('fore'), mid('fore', 'hand', 0.43), 0.0162);
    massPt(s('hand'), mid('hand', 'f3a', 0.5), 0.0061);
    massPt(s('thigh'), mid('thigh', 'shank', 0.42), 0.1416);
    massPt(s('shank'), mid('shank', 'foot', 0.44), 0.0433);
    massPt(s('foot'), V(side === 'R' ? -0.093 : 0.093, 0.035, 0.045), 0.0137);
  }
  function comOf(W) { const c = V(0, 0, 0); for (const m of MASS) c.addScaledVector(pt(W, m.seg, m.p), m.m); return c; }

  // ---------- placing a configuration on the floor ----------
  function placeW(L, rootQ) {
    const W0 = computeWorld(L, rootQ, V(0, 0, 0));
    let minY = Infinity, zmin = Infinity, zmax = -Infinity;
    for (const l of LANDMARKS) { const p = pt(W0, l.seg, l.pc); if (p.y < minY) minY = p.y; if (p.z < zmin) zmin = p.z; if (p.z > zmax) zmax = p.z; }
    return computeWorld(L, rootQ, V(0, -minY, -(zmin + zmax) / 2));
  }
  function poseFromData(d) {
    const local = {};
    for (const s of SEG) { const q = d.local[s.name]; local[s.name] = q ? new THREE.Quaternion(q[0], q[1], q[2], q[3]) : IDQ.clone(); }
    return { local, rootQ: new THREE.Quaternion(d.root[0], d.root[1], d.root[2], d.root[3]) };
  }
  function blendConf(a, b, u) {
    const L = {};
    for (const s of SEG) L[s.name] = a.local[s.name].clone().slerp(b.local[s.name], u);
    return { local: L, rootQ: a.rootQ.clone().slerp(b.rootQ, u) };
  }
  const STAND = (() => { const L = {}; for (const s of SEG) L[s.name] = IDQ.clone(); return { local: L, rootQ: IDQ.clone() }; })();
  function poseAt(conf, t) { const c = blendConf(STAND, conf, t); return { local: c.local, rootQ: c.rootQ, W: placeW(c.local, c.rootQ) }; }

  // ---------- paths with wrapping ----------
  const _qi = new THREE.Quaternion();
  function wrapSeg(W, w, a, b, out, depth) {
    const ww = W[w.seg]; _qi.copy(ww.q).invert();
    const J = S[w.seg].J;
    const toN = P => P.clone().sub(ww.p).applyQuaternion(_qi).add(J).sub(w.c).divide(w.r);
    const an = toN(a), bn = toN(b), d = bn.clone().sub(an);
    const dd = d.lengthSq();
    let t = dd > 1e-12 ? -an.dot(d) / dd : 0;
    t = Math.max(0.02, Math.min(0.98, t));
    const qn = an.clone().addScaledVector(d, t);
    const L = qn.length();
    if (L >= 1 || L < 1e-6) { out.push(b); return; }
    qn.multiplyScalar(1.0 / L);
    const q = qn.multiply(w.r).add(w.c).sub(J).applyQuaternion(ww.q).add(ww.p);
    if (depth > 0) { wrapSeg(W, w, a, q, out, depth - 1); wrapSeg(W, w, q, b, out, depth - 1); }
    else { out.push(q); out.push(b); }
  }
  function strandPath(W, m) {
    let pts = m.pts.map(q => pt(W, q.seg, q.p));
    for (const w of m.wraps) {
      const out = [pts[0]];
      for (let i = 1; i < pts.length; i++) wrapSeg(W, w, pts[i - 1], pts[i], out, w.depth);
      pts = out;
    }
    return pts;
  }
  function pathLength(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += pts[i].distanceTo(pts[i - 1]); return L; }

  // ---------- joint analysis: clinical components ----------
  // [key, label if positive, norm if positive, label if negative, norm if negative, note]
  const ROM = {
    hip: [['flex', 'Сгибание', 120, 'Разгибание', 20, 'с прямым коленом у большинства сгибание 80–90°'], ['abd', 'Отведение', 45, 'Приведение', 30], ['er', 'Наружная ротация', 45, 'Внутренняя ротация', 45]],
    knee: [['flex', 'Сгибание', 140, 'Переразгибание', 5]],
    gh: [['elev', 'Подъём руки', 180], ['flex', 'Сгибание', 180, 'Разгибание', 60, 'больше 180° значит, что рука ушла за линию уха'], ['abd', 'Отведение', 180, 'Приведение', 45], ['hflex', 'Горизонтальное приведение', 130, 'Горизонтальное отведение', 45, 'рука уходит вперёд от линии плеч'], ['er', 'Наружная ротация', 90, 'Внутренняя ротация', 70]],
    st: [['up', 'Разворот вверх', 60, 'Разворот вниз', 20], ['post', 'Наклон назад', 30, 'Наклон вперёд', 20], ['prot', 'Смещение вперёд', 30, 'Сведение', 25]],
    elbow: [['flex', 'Сгибание', 150, 'Переразгибание', 5], ['pron', 'Пронация', 90, 'Супинация', 90, 'от среднего положения предплечья']],
    wrist: [['ext', 'Разгибание', 70, 'Сгибание', 80], ['dev', 'Лучевое отведение', 20, 'Локтевое отведение', 35]],
    ankle: [['dorsi', 'Тыльное сгибание', 20, 'Подошвенное сгибание', 50]],
    toes: [['ext', 'Разгибание пальцев', 70, 'Сгибание пальцев', 40]],
    sc: [['elev', 'Подъём ключицы', 45, 'Опускание ключицы', 10], ['prot', 'Ключица вперёд', 30, 'Ключица назад', 30], ['rot', 'Поворот ключицы назад', 50, 'Поворот ключицы вперёд', 10]],
    ac: [['rot', 'Поворот лопатки относительно ключицы', 30]],
  };
  const angle = (a, b) => a.angleTo(b) / D;
  const twistAbout = (q, ax) => { const v = V(q.x, q.y, q.z); const w = ax.clone().multiplyScalar(v.dot(ax)); const tq = new THREE.Quaternion(w.x, w.y, w.z, q.w).normalize(); let a = 2 * Math.atan2(tq.x * ax.x + tq.y * ax.y + tq.z * ax.z, tq.w) / D; if (a > 180) a -= 360; if (a < -180) a += 360; return a; };
  const swingOf = (q, axis) => { const d = axis.clone().applyQuaternion(q); const qs = new THREE.Quaternion().setFromUnitVectors(axis, d); return { d, twist: qs.clone().invert().multiply(q) }; };
  const scapM = new THREE.Matrix4().makeBasis(uScap, nScap, V(0, 1, 0));
  const scapQ = new THREE.Quaternion().setFromRotationMatrix(scapM), scapQi = scapQ.clone().invert();
  function quatAngleDeg(q) { return 2 * Math.acos(Math.min(1, Math.abs(q.w))) / D; }
  function analyzeJoint(j, local, W, hint) {
    const side = j.side || 'L';
    const Lq = n => { let q = local[sideName(n, side)].clone(); if (side === 'R') q = mirrorQ(q); return q; };
    const out = { id: j.id, type: j.type, comps: [], total: 0 };
    const push = (key, v) => {
      const r = ROM[j.type].find(x => x[0] === key);
      const neg = v < 0 && r[3];
      out.comps.push({ key, label: neg ? r[3] : r[1], value: Math.abs(v), signed: v, rom: neg ? r[4] : r[2], note: (neg ? null : r[5]) || null });
    };
    if (j.type === 'spine') {
      const e = new THREE.Euler().setFromQuaternion(local[j.seg], 'XYZ').x / D;
      const R = REGION[j.region], n = R.ids.length;
      out.comps.push({ key: 'ext', label: e < -0.2 ? 'Разгибание' : e > 0.2 ? 'Сгибание' : 'Нейтраль', value: Math.abs(e), signed: e, rom: (e <= 0 ? R.ext : R.flex) / n });
      out.primary = Math.abs(e); out.total = Math.abs(e); out.signed = e;
      return out;
    }
    if (['hip', 'knee', 'elbow', 'wrist', 'ankle', 'toes'].includes(j.type)) {
      const q = Lq(j.seg.replace(/_[LR]$/, ''));
      out.total = quatAngleDeg(q);
      const { d, twist } = swingOf(q, V(0, -1, 0));
      const fwd = Math.atan2(d.z, -d.y) / D;
      const lat = Math.asin(Math.max(-1, Math.min(1, d.x))) / D;
      const tw = twistAbout(twist, V(0, 1, 0));
      if (j.type === 'hip') { push('flex', fwd); push('abd', lat); push('er', tw); out.primary = fwd; }
      else if (j.type === 'knee') { push('flex', -fwd); out.primary = -fwd; }
      else if (j.type === 'elbow') { push('flex', fwd); push('pron', -tw); out.primary = fwd; }
      else if (j.type === 'wrist') {
        // the hand bent back past a right angle (palms flat on the floor under vertical forearms): asin alone folds 95° back to 85°
        let ext = lat, dev = fwd;
        if (Math.abs(fwd) > 90) { dev = fwd - Math.sign(fwd) * 180; ext = d.x >= 0 ? 180 - lat : -180 - lat; }
        push('ext', ext); push('dev', dev); out.primary = ext;
      }
      else if (j.type === 'ankle') { push('dorsi', fwd); out.primary = fwd; }
      else if (j.type === 'toes') { push('ext', fwd); out.primary = fwd; }
      return out;
    }
    if (j.type === 'gh') {
      const thQ = W.thC.q.clone().invert().multiply(W[j.seg].q);
      const q = side === 'R' ? mirrorQ(thQ) : thQ;
      out.total = quatAngleDeg(q);
      const d = V(0, -1, 0).applyQuaternion(q);
      const elev = angle(d, V(0, -1, 0));
      // arm raised close to the plane of the shoulders: abduction first, then the horizontal swing;
      // otherwise flexion/extension first. The full pose decides, the slider keeps that choice (hint)
      const plane = Math.atan2(d.z, d.x) / D;
      const mode = hint || (elev > 30 && elev < 150 && Math.abs(plane) < 45 ? 'abd' : 'flex');
      out.mode = mode;
      push('elev', elev);
      if (mode === 'abd') {
        let abd = Math.atan2(d.x, -d.y) / D; if (abd < -90) abd += 360;
        const rem = qAx(AX.Z, -abd).multiply(q);
        const { d: d2, twist } = swingOf(rem, V(0, -1, 0));
        push('abd', abd); push('hflex', Math.asin(Math.max(-1, Math.min(1, d2.z))) / D); push('er', twistAbout(twist, V(0, 1, 0)));
      } else {
        let flex = Math.atan2(d.z, -d.y) / D; if (flex < -90) flex += 360;
        const rem = qAx(AX.X, flex).multiply(q);
        const { d: d2, twist } = swingOf(rem, V(0, -1, 0));
        push('flex', flex); push('abd', Math.asin(Math.max(-1, Math.min(1, d2.x))) / D); push('er', twistAbout(twist, V(0, 1, 0)));
      }
      out.primary = elev;
      return out;
    }
    if (j.type === 'st') {
      const q = Lq('clav').multiply(Lq('scap'));
      out.total = quatAngleDeg(q);
      const qf = scapQi.clone().multiply(q).multiply(scapQ);
      const e = new THREE.Euler().setFromQuaternion(qf, 'ZYX');
      push('up', -e.y / D); push('post', -e.x / D); push('prot', -e.z / D);
      out.primary = Math.abs(e.y / D);
      return out;
    }
    if (j.type === 'sc') {
      const q = Lq('clav'); out.total = quatAngleDeg(q);
      const ax = S.scap_L.J.clone().sub(S.clav_L.J).normalize();
      const { d, twist } = swingOf(q, ax);
      push('elev', Math.asin(Math.max(-1, Math.min(1, d.y))) / D - Math.asin(ax.y) / D);
      push('prot', -(Math.atan2(d.z, d.x) - Math.atan2(ax.z, ax.x)) / D);
      push('rot', -twistAbout(twist, ax));
      out.primary = Math.abs(out.comps[0].signed); return out;
    }
    if (j.type === 'ac') { const q = Lq('scap'); out.total = quatAngleDeg(q); push('rot', out.total); out.primary = out.total; return out; }
    return out;
  }
  function analyzeRegions(local) {
    const out = {};
    for (const key in REGION) {
      const R = REGION[key];
      let sum = 0;
      for (const id of R.ids) { const j = JOINTS.find(x => x.id === id); sum += new THREE.Euler().setFromQuaternion(local[j.seg], 'XYZ').x / D; }
      out[key] = { name: R.name, signed: sum, rom: sum < 0 ? R.ext : R.flex };
    }
    return out;
  }

  // ---------- support and cue measurements ----------
  const ARM_REG = new Set(['ладони', 'пальцы рук', 'кисти', 'предплечья', 'локти', 'кулаки']);
  const FOOT_REG = new Set(['пятки', 'подушечки стоп', 'пальцы стоп', 'тыл стоп', 'наружный край стопы']);
  const wp = (W, l) => pt(W, l.seg, l.p);
  function support(W) {
    const cs = [];
    for (const l of LANDMARKS) { const p = pt(W, l.seg, l.pc); if (p.y < 0.004) cs.push({ l, p }); }
    const com = comOf(W);
    const arms = cs.filter(c => ARM_REG.has(c.l.region)), feet = cs.filter(c => FOOT_REG.has(c.l.region));
    const regions = [];
    for (const c of cs) if (!regions.includes(c.l.region)) regions.push(c.l.region);
    let split = null;
    if (arms.length && feet.length) {
      const za = arms.reduce((s, c) => s + c.p.z, 0) / arms.length, zf = feet.reduce((s, c) => s + c.p.z, 0) / feet.length;
      const k = Math.max(0, Math.min(1, (com.z - zf) / (za - zf)));
      split = { arms: k, feet: 1 - k };
    }
    const cz = cs.length ? cs.reduce((s, c) => s + c.p.z, 0) / cs.length : com.z;
    return { regions, split, com, contactZ: cz, contacts: cs };
  }
  const minH = (W, reg, side) => { let m = Infinity; for (const l of lmkRegion(reg)) if (!side || l.side === side || l.side === 'C') m = Math.min(m, pt(W, l.seg, l.pc).y); return Math.max(0, m); };
  const tagP = (W, tag, side) => wp(W, lmkTag(tag, side || 'L'));
  const flatAngle = (a, b) => { const u = V(a.x, 0, a.z).normalize(), v = V(b.x, 0, b.z).normalize(); return angle(u, v); };
  function measure(id, local, W) {
    const J = jid => analyzeJoint(JOINTS.find(j => j.id === jid), local, W);
    const C = (res, key) => res.comps.find(c => c.key === key).signed;
    const reg = analyzeRegions(local);
    const sup = support(W);
    const m = { support: sup, regions: reg };
    const segDir = (a, b) => W[b].p.clone().sub(W[a].p);
    if (id === 'dog') {
      m.lumbar = -reg.lumbar.signed;
      m.hipIR = -C(J('hip_L'), 'er');
      m.heelGap = tagP(W, 'heel', 'L').distanceTo(tagP(W, 'heel', 'R'));
      m.hipWidth = W.thigh_L.p.distanceTo(W.thigh_R.p);
      m.retract = -C(J('st_L'), 'prot');
      m.clavElev = C(J('sc_L'), 'elev');
      const thumb = tagP(W, 'tip1').sub(W.f1m_L.p), little = tagP(W, 'tip5').sub(W.f5a_L.p);
      m.spread = angle(thumb, little);
      m.midDev = flatAngle(tagP(W, 'tip3').sub(W.f3a_L.p), segDir('fore_L', 'hand_L'));
      m.heelH = minH(W, 'пятки', 'L');
      m.knee = C(J('kn_L'), 'flex');
    } else if (id === 'plank') {
      m.knee = C(J('kn_L'), 'flex');
      const d = W.fore_L.p.clone().sub(W.hum_L.p); m.elbowUnder = Math.hypot(d.x, d.z);
      m.lumbar = reg.lumbar.signed;
      m.hipExt = -C(J('hip_L'), 'flex');
      m.neck = reg.cervical.signed;
      const a = W.hum_L.p.clone().add(W.hum_R.p).multiplyScalar(0.5), b = W.foot_L.p.clone().add(W.foot_R.p).multiplyScalar(0.5);
      const h = W.thigh_L.p.clone().add(W.thigh_R.p).multiplyScalar(0.5);
      const ab = b.clone().sub(a), t = h.clone().sub(a).dot(ab) / ab.lengthSq();
      const proj = a.clone().addScaledVector(ab, t);
      m.line = (h.y - proj.y);
      m.toeGap = tagP(W, 'hallux', 'L').distanceTo(tagP(W, 'hallux', 'R'));
      m.foreAngle = flatAngle(segDir('fore_L', 'hand_L'), segDir('fore_R', 'hand_R'));
    } else if (id === 'half') {
      m.plantar = -C(J('an_L'), 'dorsi');
      m.feet = minH(W, 'тыл стоп', 'L');
      m.knee = minH(W, 'колени', 'L');
      m.ribs = minH(W, 'нижние рёбра');
      m.chest = tagP(W, 'chestTop', 'C').y;
      m.wrist = C(J('wr_L'), 'ext');
      m.neck = reg.cervical.signed;
      m.clavElev = C(J('sc_L'), 'elev');
      m.thoracic = -reg.thoracic.signed; m.lumbar = -reg.lumbar.signed;
      // elbows behind the line of the shoulders, in the frame of the upper chest
      const back = V(0, 0, -1).applyQuaternion(W.thC.q);
      m.elbowBack = W.fore_L.p.clone().sub(W.hum_L.p).dot(back);
      m.elbowGap = W.fore_L.p.distanceTo(W.fore_R.p);
      { const u = W.fore_L.p.clone().sub(W.hum_L.p).normalize(); m.armLevel = Math.asin(u.y) / D; }
      m.retract = -C(J('st_L'), 'prot');
      m.handH = minH(W, 'кисти', 'L');
    } else if (id === 'bow') {
      m.ghER = C(J('gh_L'), 'er');
      m.sup = -C(J('el_L'), 'pron');
      m.dorsi = C(J('an_L'), 'dorsi');
      m.neck = reg.cervical.signed;
      m.retract = -C(J('st_L'), 'prot');
      m.thoracic = -reg.thoracic.signed; m.lumbar = -reg.lumbar.signed;
      m.kneeGap = W.shank_L.p.distanceTo(W.shank_R.p);
      m.hipWidth = W.thigh_L.p.distanceTo(W.thigh_R.p);
      m.knee = C(J('kn_L'), 'flex');
      m.feetH = W.foot_L.p.y;
      const zs = sup.contacts.map(c => c.p.z);
      const zlo = Math.min(...zs), zhi = Math.max(...zs);
      m.supportSpan = zhi - zlo;
      m.balance = sup.com.z < zlo ? sup.com.z - zlo : sup.com.z > zhi ? sup.com.z - zhi : 0;
    } else if (id === 'fold') {
      m.lumbar = reg.lumbar.signed; m.thoracic = reg.thoracic.signed;
      m.hipFlex = C(J('hip_L'), 'flex');
      { const up = V(0, 1, 0).applyQuaternion(W.pelvis.q); m.pelvisTilt = Math.atan2(up.z, up.y) / D; }
      m.elbowKnee = W.fore_L.p.distanceTo(W.shank_L.p);
      m.neck = reg.cervical.signed;
      m.toeGap = tagP(W, 'hallux', 'L').distanceTo(tagP(W, 'hallux', 'R'));
      m.heelGap = tagP(W, 'heel', 'L').distanceTo(tagP(W, 'heel', 'R'));
      const edge = side => { const s = side === 'R' ? -1 : 1; return pt(W, 'foot_' + side, V(0.13 * s, 0.015, 0.118)).sub(pt(W, 'foot_' + side, V(0.112 * s, 0.02, -0.05))); };
      m.edges = flatAngle(edge('L'), edge('R'));
      const heel = tagP(W, 'heel', 'L'), toe = tagP(W, 'toeTip', 'L');
      m.comPct = (sup.com.z - heel.z) / (toe.z - heel.z);
    } else if (id === 'handstand') {
      const mid = (a, b) => a.clone().add(b).multiplyScalar(0.5);
      m.elbow = C(J('el_L'), 'flex');
      m.armRaise = C(J('gh_L'), 'elev');
      m.scapUp = C(J('st_L'), 'up');
      m.clavElev = C(J('sc_L'), 'elev');
      m.wrist = C(J('wr_L'), 'ext');
      m.lumbar = reg.lumbar.signed; m.thoracic = reg.thoracic.signed;
      m.neck = reg.cervical.signed;
      m.plantar = -C(J('an_L'), 'dorsi');
      m.toeGap = tagP(W, 'hallux', 'L').distanceTo(tagP(W, 'hallux', 'R'));
      { const thumb = tagP(W, 'tip1').sub(W.f1m_L.p), little = tagP(W, 'tip5').sub(W.f5a_L.p); m.spread = angle(thumb, little); }
      m.handGap = W.hand_L.p.distanceTo(W.hand_R.p);
      m.shoulderW = W.hum_L.p.distanceTo(W.hum_R.p);
      const wr = mid(W.hand_L.p, W.hand_R.p), an = mid(W.foot_L.p, W.foot_R.p), hp = mid(W.thigh_L.p, W.thigh_R.p), sh = mid(W.hum_L.p, W.hum_R.p);
      const ab = an.clone().sub(wr);
      m.lean = Math.atan2(Math.hypot(ab.x, ab.z), ab.y) / D;
      // the direction the fingers point along the floor: the hands are the whole base of support
      const fdir = tagP(W, 'tip3', 'L').sub(W.hand_L.p); fdir.y = 0; fdir.normalize();
      m.shoulderAhead = sh.clone().sub(wr).dot(fdir);
      m.hipAhead = hp.clone().sub(sh).dot(fdir);
      const ss = sup.contacts.map(c => c.p.dot(fdir)), lo = Math.min(...ss), hi = Math.max(...ss), cm = sup.com.dot(fdir);
      m.handLen = hi - lo; m.comFromHeel = cm - lo; m.balance = Math.min(cm - lo, hi - cm);
    }
    return m;
  }

  // ---------- poses: cues, working muscles, per-pose notes ----------
  const fd = v => Math.round(Math.abs(v)) + '°';
  const fcm = v => { const c = Math.abs(v) * 100; return (c < 10 ? c.toFixed(1).replace('.', ',') : String(Math.round(c))) + '\u00a0см'; };
  const neckText = v => (Math.abs(v) < 2 ? 'шея ровно по линии позвоночника' : v < 0 ? 'шея разогнута на ' + fd(v) : 'шея согнута на ' + fd(v));
  const clavText = (v, ref) => 'ключица ' + (v < 0 ? 'опущена на ' : 'поднята на ') + fd(v) + (ref == null ? '' : Math.abs(ref) < 1.5 ? ', без этого действия была бы на уровне стойки' : ' вместо ' + fd(ref) + ' без этого действия');
  const onFloor = (h, what) => (h < 0.004 ? what + ' на полу' : what + ' в ' + fcm(h) + ' от пола');
  const notes = (list, text) => Object.fromEntries(list.map(u => [u, text]));
  const POSES = [
    {
      id: 'dog', num: 1, name: 'Горка', sanskrit: 'Адхо мукха шванасана', view: [-1, 0.36, 0.55],
      stepsTitle: 'Вход в позу',
      steps: [
        { t: 'Из планки на прямых руках, ноги вместе, максимально прогибаем поясницу' },
        { t: 'Входим в положение, сохраняя прогиб в пояснице и ноги вместе', m: m => 'поясница прогнута на ' + fd(m.lumbar) },
        { t: 'Сводим лопатки, опускаем плечи к тазу', m: m => 'лопатки сведены на ' + fd(m.retract) + ', ' + clavText(m.clavElev, m.clavElevNoCue) },
        { t: 'Ставим ноги на ширину таза, пятки смотрят в разные стороны', m: m => 'между пятками ' + fcm(m.heelGap) + ' при ширине таза ' + fcm(m.hipWidth) + ', бёдра повёрнуты внутрь на ' + fd(m.hipIR) },
      ],
      cuesTitle: 'Нюансы',
      cues: [
        { t: 'Пальцы на руках растопырены максимально', m: m => 'между большим пальцем и мизинцем ' + fd(m.spread) },
        { t: 'Средний палец смотрит в продолжение запястья', m: m => 'отклонение от линии предплечья ' + fd(m.midDev) },
        { t: 'Если пятки не касаются пола, колени подсогнуты. Если коснулись, выпрямляем ноги', m: m => 'пятки в ' + fcm(m.heelH) + ' от пола, колени согнуты на ' + fd(m.knee) },
      ],
      supportLabel: 'Руки',
      active: ['serratus', 'trapLow', 'trapMid', 'rhomb', 'deltAnt', 'infra', 'teresMin', 'tricLong', 'tricLat', 'tricMed', 'anconeus', 'wristFlex', 'fingerFlex', 'interossei', 'thenar', 'hypothenar', 'erector', 'multifidus', 'psoas', 'iliacus', 'tfl', 'gluteMin', 'vastLat', 'vastMed', 'vastInt', 'rectFem'],
      notes: Object.assign({
        serratus: 'Толкает пол и держит лопатку на рёбрах. При сведении лопаток работает вместе с ромбовидными, чтобы лопатки не отходили от рёбер.',
        trapLow: 'Опускает плечи к тазу и помогает развернуть лопатку вверх.',
        trapMid: 'Сводит лопатки.',
        rhomb: 'Сводят лопатки. Работают против передней зубчатой, и вместе они прижимают лопатку к рёбрам.',
        lat: 'Главный ограничитель подъёма рук: чем она короче, тем труднее держать прогиб и раскрыть плечи.',
        teresMaj: 'Тянется вместе с широчайшей и тоже ограничивает подъём рук.',
        pecMaj: 'Грудинные и брюшные волокна удлиняются, когда руки уходят за голову.',
        erector: 'Держит прогиб в пояснице, набранный ещё в планке.',
        multifidus: 'Держит каждый поясничный позвонок в прогибе.',
        psoas: 'Укорочена: бедро глубоко согнуто. В таком положении у неё почти нет рычага, поэтому наклонять таз вперёд приходится другим мышцам.',
        iliacus: 'Наклоняет таз вперёд против натяжения задней поверхности бедра. Она укорочена, и силы у неё здесь немного.',
        sartorius: 'При глубоком сгибании бедра у неё лучший рычаг среди сгибателей: вместе с подвздошной наклоняет таз вперёд против натяжения задней поверхности бедра.',
        tfl: 'Бедро повёрнуто внутрь, и пятки разведены, но держать это положение ей почти не нужно.',
        gluteMin: 'Немного помогает держать бедро повёрнутым внутрь.',
        deltAnt: 'Работает сильнее всех. Опора давит руку за голову, а передняя дельта её удерживает. Чем ниже грудь к полу, тем больше нагрузка.',
        infra: 'Прижимает головку плеча к лопатке и помогает передней дельте.',
        teresMin: 'Прижимает головку плеча к лопатке и помогает передней дельте.',
        tricMed: 'Держит локоть прямым: опора давит руку в сгибание.',
        tricLat: 'Держит локоть прямым: опора давит руку в сгибание.',
        anconeus: 'Помогает трёхглавой держать локоть прямым.',
        quadFem: 'Короткая и лежит далеко от оси сгибания: при глубоком сгибании бедра её концы расходятся на несколько сантиметров, поэтому в процентах она удлиняется сильнее длинных мышц бедра.',
        interossei: 'Разводят пальцы веером.',
        thenar: 'Отводят большой палец.',
        hypothenar: 'Отводят мизинец.',
        fingerFlex: 'Прижимают подушечки пальцев к полу и помогают держать запястье.',
        wristFlex: 'Тянутся при разгибании запястья и прижимают основание ладони.',
        tricLong: 'Выпрямляет локоть и тянется в плечевом суставе, потому что начинается на лопатке.',
        gluteMax: 'Сильно удлинена: бедро глубоко согнуто.',
      }, notes(['vastLat', 'vastMed', 'vastInt', 'rectFem'], 'Натяжение задней поверхности бедра сгибает колени, а четырёхглавая с небольшим усилием держит их почти прямыми. Когда пятки опустятся, она выпрямит ноги.'),
        notes(['bicFemL', 'semiT', 'semiM'], 'Задняя поверхность бедра натянута и тянет таз назад. Подсогнутые колени снимают часть натяжения.'),
        notes(['gastroM', 'gastroL', 'soleus'], 'Пятки над полом, поэтому икры частично натянуты, а частично работают, как в полупальцах, и держат пятку на весу.')),
      joints: {
        gh: 'Рука ушла за ухо. Лопатки сведены и опущены, поэтому большая часть подъёма руки идёт в самом плечевом суставе.',
        st: 'Лопатка развёрнута вверх, но сведена и опущена по подсказке.',
        sc: 'Ключица поднимается меньше обычного, потому что плечи опущены к тазу.',
        elbow: 'Локоть прямой. Предплечье развёрнуто ладонью вниз: лучевая кость обвивает локтевую, сама локтевая кость не вращается.',
        wrist: 'Разгибание запястья равно наклону предплечья к полу: ладонь лежит плоско, средний палец по линии предплечья.',
        hip: 'Глубокое сгибание: таз наклонён вперёд, чтобы сохранить прогиб в пояснице. Внутренняя ротация разводит пятки в стороны.',
        knee: 'Колени подсогнуты, потому что пятки не достают пола.',
        ankle: 'Тыльное сгибание до натяжения икр, пятки над полом.',
        toes: 'Пальцы стоп разогнуты, вес на подушечках.',
        spine: 'Прогиб в пояснице сохраняется из планки, грудной отдел тоже разогнут.',
      },
    },
    {
      id: 'plank', num: 2, name: 'Планка на предплечьях', sanskrit: 'Макара адхо мукха шванасана', view: [-1, 0.32, 0.5],
      cuesTitle: 'Как делать',
      cues: [
        { t: 'Ноги идеально прямые', m: m => 'сгибание в колене ' + fd(m.knee) },
        { t: 'Колени в тонусе' },
        { t: 'Большие пальцы ног вместе', m: m => 'зазор между большими пальцами ' + fcm(m.toeGap) },
        { t: 'Локоть под плечевым суставом', m: m => 'смещение локтя от плеча ' + fcm(m.elbowUnder) },
        { t: 'Предплечья параллельны', m: m => 'угол между предплечьями ' + fd(m.foreAngle) },
        { t: 'Копчик подкручен', m: m => 'поясничный прогиб уменьшен на ' + fd(m.lumbar) + ', бедро разогнуто на ' + fd(m.hipExt) },
        { t: 'Шея в продолжение позвоночника', m: m => neckText(m.neck) },
      ],
      supportLabel: 'Предплечья',
      extra: m => 'Таз ' + (m.line >= 0 ? 'выше' : 'ниже') + ' прямой от плеч до щиколоток на ' + fcm(m.line),
      active: ['rectAbd', 'extObl', 'intObl', 'transAbd', 'gluteMax', 'rectFem', 'vastLat', 'vastMed', 'vastInt', 'serratus', 'deltAnt', 'coracobr', 'pecMaj', 'splenius', 'semispinalis', 'psoas', 'iliacus', 'tibAnt', 'fingerFlex'],
      notes: Object.assign({
        rectAbd: 'Подкручивает копчик и не даёт пояснице провиснуть.',
        gluteMax: 'Подкручивает таз вместе с прямой мышцей живота.',
        rectFem: 'Держит колено прямым и не даёт тазу провиснуть.',
        serratus: 'Толкает пол предплечьями и не даёт груди провалиться между лопатками.',
        deltAnt: 'Почти не работает: локоть стоит под плечом, и опора проходит почти через плечевой сустав.',
        coracobr: 'Почти не работает: опора проходит почти через плечевой сустав.',
        pecMaj: 'Почти не работает: предплечья параллельны, и опора проходит почти через плечевой сустав.',
        tibAnt: 'Почти не нужна: стопа стоит на пальцах, и голеностоп держат икры.',
        fingerFlex: 'Сжимают кулаки.',
        tfl: 'Не даёт тазу провиснуть: в планке вес тела разгибает бедро, и сгибатели бедра держат его на линии.',
        sartorius: 'Помогает сгибателям бедра не дать тазу провиснуть.',
        pectineus: 'Помогает сгибателям бедра не дать тазу провиснуть.',
        addLong: 'Помогает сгибателям бедра не дать тазу провиснуть.',
      }, notes(['extObl', 'intObl', 'transAbd'], 'Вместе с другими мышцами живота держит живот плоским, а тело в одну линию.'),
        notes(['vastLat', 'vastMed', 'vastInt'], 'Колени в тонусе: четырёхглавая держит ноги идеально прямыми.'),
        notes(['splenius', 'semispinalis'], 'Держит голову в продолжение позвоночника.'),
        notes(['psoas', 'iliacus'], 'Держит ноги на одной линии с корпусом.')),
      joints: {
        gh: 'Плечо согнуто примерно на 90°, локоть ровно под плечом, предплечья параллельны.',
        st: 'Лопатки чуть уходят вперёд: передняя зубчатая толкает пол.',
        elbow: 'Локоть согнут на 90°, предплечье лежит на полу ребром ладони.',
        wrist: 'Запястье в нейтрали, кулаки сжаты.',
        hip: 'Копчик подкручен: бедро немного разогнуто относительно таза. Ноги сведены так, что большие пальцы касаются.',
        knee: 'Колени прямые и в тонусе.',
        ankle: 'Стопы почти вертикальны, голеностоп в лёгком тыльном сгибании.',
        toes: 'Пальцы стоп разогнуты: вес на подушечках пальцев.',
        spine: 'Поясничный прогиб уменьшен, шея в продолжение позвоночника.',
      },
    },
    {
      id: 'half', num: 3, name: 'Полупрогиб', sanskrit: 'Вариант шалабхасаны', view: [-1, 0.3, 0.62],
      cuesTitle: 'Как делать',
      cues: [
        { t: 'Носки от себя, на полу', m: m => 'подошвенное сгибание ' + fd(m.plantar) + ', ' + onFloor(m.feet, 'тыл стопы') },
        { t: 'Колени пола не касаются', m: m => 'колени в ' + fcm(m.knee) + ' от пола' },
        { t: 'Самые нижние рёбра прижаты, остальные нет', m: m => onFloor(m.ribs, 'нижние рёбра') + ', верх груди в ' + fcm(m.chest) + ' от пола' },
        { t: 'Кисти тянем на себя', m: m => 'запястье разогнуто на ' + fd(m.wrist) },
        { t: 'Шея в продолжение позвоночника', m: m => neckText(m.neck) },
        { t: 'Плечи к тазу', m: m => clavText(m.clavElev, m.clavElevNoCue) },
        { t: 'Локти сводим за спиной', m: m => 'локти на ' + fcm(m.elbowBack) + ' за линией плеч, на высоте плеч, лопатки сведены на ' + fd(m.retract) },
        { t: 'Ягодицы в тонусе' },
        { t: 'Пресс включён' },
      ],
      active: ['erector', 'multifidus', 'semispinalis', 'splenius', 'trapLow', 'trapMid', 'rhomb', 'deltPost', 'deltMid', 'infra', 'teresMin', 'gluteMax', 'bicFemL', 'semiT', 'semiM', 'vastLat', 'vastMed', 'vastInt', 'rectAbd', 'transAbd', 'intObl', 'extObl', 'wristExt', 'gastroM', 'gastroL', 'soleus'],
      notes: Object.assign({
        erector: 'Поднимает грудь. Разгибание идёт в груди, нижние рёбра остаются на полу.',
        multifidus: 'Держит позвонки поясницы и груди в разгибании.',
        trapLow: 'Опускает плечи к тазу.',
        gluteMax: 'Ягодицы в тонусе: разгибают бедро и не дают тазу наклониться вперёд.',
        rectAbd: 'Пресс включён: держит нижние рёбра у пола.',
        wristExt: 'Тянут кисти на себя.',
        psoas: 'Работает против ягодичных: вместе с прямой мышцей бедра прижимает стопы к полу.',
        iliacus: 'Работает против ягодичных: вместе с прямой мышцей бедра прижимает стопы к полу.',
        tfl: 'Работает против ягодичной мышцы: та разгибает бедро и разворачивает его наружу, а эта сгибает и разворачивает внутрь, поэтому ноги остаются на полу ровно.',
      }, notes(['semispinalis', 'splenius'], 'Держит голову в продолжение позвоночника.'),
        { trapMid: 'Сводит лопатки, и локти уходят за спину.', rhomb: 'Сводят лопатки, и локти уходят за спину.', deltPost: 'Отводит локти назад за линию плеч и держит плечо развёрнутым наружу.', deltMid: 'Держит руки на высоте плеч: без неё локти опустились бы к полу.', supra: 'Помогает средней дельте держать руку на высоте плеча.' },
        notes(['infra', 'teresMin'], 'Плечо развёрнуто наружу, предплечья смотрят вперёд. Разворот держит задняя дельта, поэтому этой мышце почти нечего делать.'),
        notes(['bicFemL', 'semiT', 'semiM'], 'Почти не нужна: бедро разгибают ягодицы, а колено прямым держит четырёхглавая.'),
        notes(['vastLat', 'vastMed', 'vastInt'], 'Четырёхглавая выпрямляет колени и подтягивает надколенники, поэтому колени не касаются пола.'),
        notes(['transAbd', 'intObl', 'extObl'], 'Пресс включён: не даёт пояснице провалиться в прогиб.'),
        notes(['gastroM', 'gastroL', 'soleus'], 'Тянет носки от себя.')),
      joints: {
        gh: 'Плечо параллельно полу: рука отведена в сторону и назад, локоть за линией плеч на их высоте.',
        st: 'Лопатки сведены и опущены к тазу, поэтому локти уходят назад.',
        elbow: 'Локоть сильно согнут, предплечье тоже параллельно полу, кисть сбоку от плеча.',
        wrist: 'Кисти тянем на себя: запястье разогнуто, ладони смотрят вперёд и вниз.',
        hip: 'Бёдра лежат на полу почти без сгибания. Колени над полом, потому что напряжённые мышцы бедра подтягивают надколенники.',
        knee: 'Колени прямые.',
        ankle: 'Подошвенное сгибание: носки вытянуты, тыл стопы на полу.',
        toes: 'Пальцы стоп вытянуты по линии стопы.',
        spine: 'Разгибание в груди и пояснице, нижние рёбра на полу, шея в продолжение позвоночника.',
      },
    },
    {
      id: 'bow', num: 4, name: 'Корзинка', sanskrit: 'Дханурасана', view: [-1, 0.38, 0.62],
      cuesTitle: 'Как делать',
      cues: [
        { t: 'Руки берутся за голени изнутри', m: m => 'ладони на внутренней стороне голеней, предплечья развёрнуты ладонью наружу на ' + fd(m.sup) },
        { t: 'Локоть смотрит внутрь', m: m => (m.ghER >= 0 ? 'наружная' : 'внутренняя') + ' ротация плеча ' + fd(m.ghER) },
        { t: 'Носки на себя', m: m => 'тыльное сгибание ' + fd(m.dorsi) },
        { t: 'Шея расслаблена', m: m => neckText(m.neck) },
        { t: 'Плечи назад, грудь вперёд', m: m => 'лопатки сведены на ' + fd(m.retract) + ', грудной отдел разогнут на ' + fd(m.thoracic) },
        { t: 'Ноги толкаем вовнутрь', m: m => 'между коленями ' + fcm(m.kneeGap) + ' при ширине таза ' + fcm(m.hipWidth) },
        { t: 'Пятки толкаем вверх и назад', m: m => 'колено согнуто на ' + fd(m.knee) + ', стопы на высоте ' + fcm(m.feetH) },
      ],
      extra: m => (Math.abs(m.balance) < 0.003 ? 'Центр масс над площадкой опоры длиной ' + fcm(m.supportSpan) : 'Центр масс вне опоры на ' + fcm(m.balance)),
      active: ['rectFem', 'vastLat', 'vastMed', 'vastInt', 'gluteMax', 'bicFemL', 'semiT', 'semiM', 'addLong', 'addBrev', 'addMag', 'gracilis', 'tibAnt', 'edl', 'ehl', 'erector', 'multifidus', 'trapMid', 'rhomb', 'trapLow', 'fingerFlex', 'wristFlex', 'infra', 'teresMin', 'supinator'],
      notes: Object.assign({
        rectFem: 'Сильно растянута: бедро разогнуто, колено согнуто. Сама почти не работает, потому что сгибала бы бедро, но её натяжение помогает разгибать колено.',
        deltMid: 'Держит вес руки: прямая рука висит между плечом и голенью.',
        gluteMed: 'Отводит бедро против приводящих: ноги толкают вовнутрь, и тазобедренный сустав держат обе группы.',
      }, notes(['vastLat', 'vastMed', 'vastInt'], 'Четырёхглавая толкает пятки вверх и назад в руки. Именно это поднимает грудь.'),
        notes(['gluteMax', 'bicFemL', 'semiT', 'semiM'], 'Почти не работает: колени поднимает тяга рук, когда четырёхглавая толкает голени назад.'),
        notes(['addLong', 'addBrev', 'addMag', 'gracilis'], 'Ноги толкаем вовнутрь: голени сжимают кисти, и приводящие мышцы работают против рук.'),
        notes(['tibAnt', 'edl', 'ehl'], 'Держит носки на себя.'),
        notes(['erector', 'multifidus'], 'Держит прогиб.'),
        { trapMid: 'Отводит плечи назад и раскрывает грудь.', trapLow: 'Почти не работает: плечи назад и вниз тянут натянутые руки.', rhomb: 'Отводят плечи назад и раскрывают грудь.' },
        notes(['fingerFlex', 'wristFlex'], 'Держат хват.'),
        notes(['pecMaj', 'deltAnt', 'bicLong', 'coracobr'], 'Сильно растянута: руки отведены назад.'),
        notes(['rectAbd', 'psoas', 'iliacus'], 'Растянута прогибом и разгибанием бедра.'),
        notes(['infra', 'teresMin'], 'Плечо развёрнуто наружу почти до предела, но держать это не нужно: руки натянуты, как тросы.'),
        { supinator: 'Предплечье развёрнуто ладонью наружу почти до предела. Держать его так почти не нужно: голень сама прижимает ладонь.' }),
      joints: {
        gh: 'Руки прямые и отведены назад к голеням. Хват изнутри требует наружной ротации плеча почти до предела, поэтому локти смотрят внутрь.',
        st: 'Лопатки сведены и опущены: плечи назад, грудь вперёд.',
        elbow: 'Локти прямые: руки работают как тросы между плечами и голенями. Предплечье развёрнуто ладонью наружу почти до предела супинации. Вместе с ротацией плеча это около 180°.',
        wrist: 'Ладони прижаты к внутренней стороне голеней, запястье почти прямое.',
        hip: 'Бедро разогнуто, колени над полом.',
        knee: 'Колено согнуто ровно настолько, чтобы прямые руки дотягивались до голеней.',
        ankle: 'Тыльное сгибание: носки на себя.',
        toes: 'Пальцы стоп чуть разогнуты.',
        spine: 'Глубокое разгибание всего позвоночника, шея расслаблена.',
      },
    },
    {
      id: 'fold', num: 5, name: 'Наклон', sanskrit: 'Уттанасана', view: [-1, 0.16, 0.75],
      cuesTitle: 'Как делать',
      cues: [
        { t: 'Спина прямая', m: m => 'сгибание поясницы ' + fd(m.lumbar) + ', груди ' + fd(m.thoracic) },
        { t: 'Прокручиваем поясницу', m: m => 'таз наклонён вперёд на ' + fd(m.pelvisTilt) + ' от стойки' },
        { t: 'Локти прижаты к коленям', m: m => (m.elbowKnee < 0.09 ? 'локоть касается колена снаружи, между центрами суставов ' : 'между центрами локтя и колена ') + fcm(m.elbowKnee) },
        { t: 'Шея в продолжение позвоночника', m: m => neckText(m.neck) },
        { t: 'Большие пальцы вместе, пятки слегка врозь, так чтобы боковые части стоп были параллельны', m: m => 'зазор между большими пальцами ' + fcm(m.toeGap) + ', между пятками ' + fcm(m.heelGap) + ', угол между краями стоп ' + fd(m.edges) },
        { t: 'Вес в центре стопы', m: m => 'центр масс проецируется на ' + Math.round(m.comPct * 100) + '\u00a0% длины стопы от пятки' },
      ],
      active: ['erector', 'multifidus', 'psoas', 'iliacus', 'rectFem', 'vastLat', 'vastMed', 'vastInt', 'bicLong', 'bicShort', 'brachialis', 'fingerFlex', 'tibAnt', 'fibL', 'tibPost', 'abdHall', 'fdb', 'fhl', 'tfl', 'gluteMin', 'addLong', 'splenius', 'semispinalis', 'trapLow', 'trapMid'],
      notes: Object.assign({
        gluteMax: 'Сильно удлинена глубоким наклоном.',
        fingerFlex: 'Держат хват за голени над щиколотками.',
        quadFem: 'Короткая и лежит далеко от оси сгибания: при глубоком сгибании бедра её концы расходятся на несколько сантиметров, поэтому в процентах она удлиняется сильнее длинных мышц бедра.',
        tibAnt: 'Почти не нужна: центр масс впереди голеностопа, и голень держат икры.',
        supra: 'Держит руку в плечевом суставе: руки свисают к полу.',
        splenius: 'Держит шею в продолжение позвоночника.',
      }, notes(['bicFemL', 'semiT', 'semiM'], 'Задняя поверхность бедра даёт главное растяжение позы и своим натяжением держит большую часть веса корпуса. Чем она короче, тем сильнее округляется спина.'),
        notes(['erector', 'multifidus'], 'Держит спину прямой против силы тяжести.'),
        notes(['psoas', 'iliacus'], 'Тянет корпус к ногам и наклоняет таз вперёд, но в глубоком наклоне она укорочена, и силы у неё немного.'),
        notes(['rectFem', 'vastLat', 'vastMed', 'vastInt'], 'Почти не нужна: вес корпуса сам выпрямляет колени. От переразгибания их держат икроножные и задняя поверхность бедра.'),
        notes(['bicLong', 'bicShort', 'brachialis'], 'Почти не работает: руки лишь обхватывают голени. Если тянуть корпус руками к ногам, работа перейдёт сюда.'),
        notes(['gastroM', 'gastroL'], 'Держит голень над стопой и не даёт колену переразогнуться: вес корпуса давит колени назад.'),
        notes(['soleus', 'tibPost', 'fhl'], 'Помогает икроножной держать голень над стопой.'),
        notes(['addLong', 'addBrev', 'gracilis'], 'В глубоком наклоне приводящие становятся разгибателями бедра и помогают задней поверхности держать корпус. Длинная приводящая ещё и сводит большие пальцы стоп.'),
        notes(['tfl', 'gluteMin'], 'Сводит большие пальцы стоп: вращает бедро внутрь.'),
        notes(['trapLow', 'trapMid'], 'Почти не работает: в наклоне плечи от ушей держат большая грудная и широчайшая.')),
      joints: {
        gh: 'Руки опущены вдоль ног к щиколоткам.',
        st: 'Лопатки держатся от ушей.',
        elbow: 'Локти согнуты и прижаты к ногам у коленей.',
        wrist: 'Ладони обхватывают голени сзади и снаружи над щиколотками, запястье прямое.',
        hip: 'Глубокое сгибание с прямыми коленями: наклон идёт из тазобедренных суставов.',
        knee: 'Колени прямые. Вес корпуса давит их назад, в переразгибание, и колени держат икроножные мышцы и натянутая задняя поверхность бедра.',
        ankle: 'Голень наклонена так, чтобы центр масс был над серединой стопы.',
        toes: 'Стопы плоско на полу.',
        spine: 'Спина почти прямая, шея в продолжение позвоночника.',
      },
    },
    {
      id: 'handstand', num: 6, name: 'Стойка на руках', short: 'На руках', sanskrit: 'Адхо мукха врикшасана', view: [1, 0.25, -0.55],
      stepsTitle: 'Вход в позу',
      steps: [
        { t: 'Из наклона ставим ладони на пол на ширине плеч, пальцы широко', m: m => 'между центрами кистей ' + fcm(m.handGap) + ' при ширине плеч ' + fcm(m.shoulderW) },
        { t: 'Переносим вес на руки: плечи над ладонями, локти прямые', m: m => 'в стойке плечи на ' + fcm(m.shoulderAhead) + ' впереди запястий' },
        { t: 'Выталкиваем плечи вверх и поднимаем ноги вместе, пока таз не окажется над плечами', m: m => 'таз ' + (m.hipAhead >= 0 ? 'впереди' : 'позади') + ' плеч на ' + fcm(m.hipAhead) },
      ],
      cuesTitle: 'Как делать',
      cues: [
        { t: 'Ладони плоско на полу, пальцы широко', m: m => 'угол между большим пальцем и мизинцем ' + fd(m.spread) + ', разгибание в запястье ' + fd(m.wrist) + ' при норме до 70°' },
        { t: 'Руки прямые, локти не сгибаем', m: m => (m.elbow < 3 ? 'локти выпрямлены' : 'локоть согнут на ' + fd(m.elbow)) },
        { t: 'Выталкиваем плечи вверх, к ушам', m: m => 'лопатка повёрнута вверх на ' + fd(m.scapUp) + ', ' + clavText(m.clavElev) },
        { t: 'Руки вплотную к ушам', m: m => 'рука поднята на ' + fd(m.armRaise) + ' относительно корпуса из 180° возможных у самого уха' },
        { t: 'Рёбра втянуты, спина ровная', m: m => 'поясница ' + (Math.abs(m.lumbar) < 1 ? 'в нейтрали' : (m.lumbar < 0 ? 'разогнута на ' : 'согнута на ') + fd(m.lumbar)) + ', грудной отдел ' + (Math.abs(m.thoracic) < 1 ? 'в нейтрали' : (m.thoracic < 0 ? 'разогнут на ' : 'согнут на ') + fd(m.thoracic)) },
        { t: 'Тело одной линией от запястий до пальцев ног', m: m => 'линия от запястий до лодыжек отклонена от вертикали на ' + fd(m.lean) },
        { t: 'Ноги вместе, носки вытянуты', m: m => 'подошвенное сгибание ' + fd(m.plantar) + ', зазор между большими пальцами ' + fcm(m.toeGap) },
        { t: 'Шея в продолжение позвоночника, взгляд между ладонями', m: m => neckText(m.neck) },
        { t: 'Баланс держим пальцами и основанием ладони: уходим вперёд — давим пальцами, назад — основанием ладони', m: m => 'центр масс на ' + fcm(m.comFromHeel) + ' впереди основания ладони' },
      ],
      extra: m => 'Опора — только кисти длиной ' + fcm(m.handLen) + ': центр масс на ' + fcm(m.balance) + ' от ближайшего края опоры',
      active: ['fingerFlex', 'wristFlex', 'palmaris', 'bicShort', 'bicLong', 'levScap', 'trapMid', 'trapUp', 'erector', 'deltPost', 'rectAbd', 'extObl', 'scm', 'rhomb', 'serratus', 'splenius', 'multifidus', 'coracobr', 'plantaris', 'fhl'],
      notes: Object.assign({
        fingerFlex: 'Прижимают пальцы к полу и принимают около 13 % веса тела. Это главный рычаг баланса: если тело уходит вперёд, его возвращают пальцами. В расчёте это одна из самых нагруженных мышц позы.',
        wristFlex: 'Вес тела давит на запястье в ещё большее разгибание, и сгибатели вместе с натянутыми связками ладони не дают ему провалиться. Работают почти на три четверти силы: предплечья в стойке устают первыми.',
        palmaris: 'Маленькая мышца, у которой немного силы: она выбирает запас почти полностью, но тянет всего на 2–3 кг. Помогает сгибателям запястья.',
        levScap: 'Тянет лопатку вверх, к уху: плечи выталкиваются к ушам, и между ушами и плечами нет зазора. Работает примерно наполовину.',
        trapMid: 'Держит лопатки от скольжения вперёд по грудной клетке: нагрузка от рук давит на них в эту сторону.',
        deltPost: 'Помогает удерживать руку над головой против давления веса тела на плечо.',
        deltMid: 'Рука поднята почти до уха, и средняя часть сильно укорочена: она работает слабо.',
        scm: 'Голова висит вниз между руками. Эта мышца вместе с ременной держит её на одной линии с позвоночником.',
        splenius: 'Голова висит вниз между руками. Мышца вместе с грудино-ключично-сосцевидной держит её на одной линии с позвоночником.',
        coracobr: 'Помогает двуглавой держать плечо.',
        plantaris: 'Держит вытянутый носок вместе с длинными сгибателями пальцев стопы. Нагрузка небольшая: вес стопы около килограмма.',
        fhl: 'Держит вытянутый носок и большой палец стопы.',
        fdl: 'Держит вытянутый носок.',
        tibAnt: 'Растянута вытянутым носком, но сама не включается.',
        tibPost: 'Помогает держать вытянутую стопу, при этом ноги сведены.',
        lat: 'Поднятая рука сильно растягивает её. Сама она почти не работает, зато пассивно тянет таз и нижние рёбра к рукам и вносит заметный вклад в нагрузку на позвоночник. Это натяжение гасят пресс и разгибатели спины.',
        pecMaj: 'Поднятая рука сильно растягивает её, и натяжение пассивно тянет грудную клетку к рукам. Вносит вклад в нагрузку на позвоночник, как и широчайшая.',
        teresMaj: 'Растянута поднятой рукой сильнее всех мышц позы.',
        tricLong: 'Растянута поднятой рукой, но сама не включается: прямой локоть держит не трицепс.',
      }, notes(['bicShort', 'bicLong'], 'Расчёт держит ими прямой локоть и плечо. В отличие от Горки, вес тела не складывает локоть, а слегка выгибает назад, и локоть упирается в кость только при переразгибании. В жизни запас прочности у локтя больше, поэтому реальная работа двуглавой, скорее всего, меньше расчётной.'),
        notes(['tricLat', 'tricMed'], 'Почти не работают: в отличие от Горки, вес тела не сгибает локоть, а слегка выгибает его назад.'),
        notes(['trapUp', 'serratus'], 'Поворачивают лопатку вверх и держат плечо у уха. Поворот лопатки сильно укорачивает мышцу, поэтому ей не нужно много силы.'),
        notes(['rhomb'], 'Растянута поворотом лопатки вверх. Держит лопатку у грудной клетки вместе со средней частью трапеции.'),
        notes(['erector'], 'Держит спину ровной и несёт около 60\u00a0кг. Сюда входит и компенсация натяжения растянутых широчайшей и большой грудной мышц.'),
        notes(['multifidus'], 'Помогает мышце, выпрямляющей позвоночник, держать спину ровной.'),
        notes(['rectAbd', 'extObl', 'intObl', 'transAbd'], 'Подтягивают грудную клетку к тазу: рёбра втянуты, поясница не прогибается под поднятыми руками.'),
        notes(['gastroM', 'gastroL', 'soleus'], 'Почти не работают: стопа в воздухе, ноги не несут нагрузки. Вытянутый носок укорачивает их.')),
      joints: {
        gh: 'Рука поднята почти до уха (около 171° относительно корпуса) и развёрнута наружу на 45°. Для такого подъёма нужна гибкость широчайшей, грудных и задней стороны плеча.',
        st: 'Лопатки повёрнуты вверх на 47° и прижаты к рёбрам: плечи выталкиваются к ушам. Из-за этого руки поднимаются выше, чем один плечевой сустав позволил бы сам.',
        sc: 'Ключицы подняты примерно на 21°: плечи у ушей.',
        elbow: 'Локти выпрямлены. Предплечье повёрнуто так, чтобы ладонь легла плоско, пальцами вперёд.',
        wrist: 'Запястье разогнуто до 97° при норме модели 70°: ладонь должна лечь плоско под вертикальным предплечьем, иначе в стойке не встать. Не у всех запястий хватает такого диапазона, и его нужно готовить заранее. Нагрузку принимают сгибатели пальцев и запястья и натянутые связки ладони.',
        hip: 'Бедро чуть разогнуто, ноги вместе: линия от таза к стопам продолжает линию рук.',
        knee: 'Колени прямые, ноги вытянуты вверх.',
        ankle: 'Подошвенное сгибание около 48°: носки вытянуты вверх.',
        toes: 'Пальцы стоп немного разогнуты, большие пальцы касаются друг друга.',
        spine: 'Позвоночник почти нейтрален: поясница ровная, грудной отдел чуть округлён, шея в продолжение линии. Расчёт даёт заметно более высокую нагрузку на диск L5–S1, чем в других позах. Причина: поднятые руки сильно растягивают широчайшую и большую грудную, их пассивное натяжение тянет корпус к рукам, и спину держат разгибатели и пресс. Это оценка модели: при более гибком плечевом поясе натяжение меньше.',
      },
    },
  ];

  return { strandPath, pathLength, strandInstances, analyzeJoint, analyzeRegions, ROM, REGION, LIGS, CAP_NAMES, UNITS, JOINTS, POSES, measure, support, comOf, LANDMARKS, MASS, placeW, poseAt, blendConf, poseFromData, STAND, scapulaPose, nScap, uScap, THREE, D, V, SEG, S, VERT, BONES, computeWorld, pt, quatAngleDeg, sideName, qAx, AX, MT, MC, lerp3, AZ, armZ, mirrorQ, IDQ, resolveRaw, lmkTag, lmkRegion, CENTRAL };
}
