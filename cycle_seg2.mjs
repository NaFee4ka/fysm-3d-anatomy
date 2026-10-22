// seg2 of the round: Планка на предплечьях → Полупрогиб («лёг на живот, вытянул рёбра, свёл лопатки»).
//  1. «Опускаюсь на живот»: the pelvis comes down to the mat while toes and forearms stay put (a sphinx on the forearms),
//     then the knees soften and the feet turn onto their tops.
//  2. «Вытягиваю рёбра вперёд»: the forearms slide forward along the mat, the hands open palms down, the chest lowers
//     and the whole front of the body lengthens.
//  3. «Свожу лопатки — полупрогиб»: the arms leave the floor, the elbows are drawn out and back to shoulder height (W),
//     the shoulder blades come together and down, the chest lifts into the half backbend on the lower ribs.
// Each frame is solved: joint parameters follow a schedule, and a small least-squares solve places the trunk so that
// planted points stay where they are, nothing goes through the floor, and the arms reach their targets.
// usage: node cycle_seg2.mjs [-v]   → cycle_keys/seg2.json
import fs from 'fs';
import { A, V, THREE, computeWorld, pt, LANDMARKS, comOf, qAx, X, Y, Z, D, SEG, IDQ, mirrorQ, armIK, orientHand, scapulaPose, LH, LF, ease } from './cycle_lib.mjs';

const anchors = JSON.parse(fs.readFileSync('cycle_keys/anchors.json', 'utf8'));
const Q = a => new THREE.Quaternion(...a);
const confOf = a => { const L = {}; for (const [s, v] of Object.entries(a.local)) L[s] = Q(v); return { L, rootQ: Q(a.root), rootP: V(...a.rootP) }; };
const PL = confOf(anchors.plank), HF = confOf(anchors.half);
const WPL = computeWorld(PL.L, PL.rootQ, PL.rootP), WHF = computeWorld(HF.L, HF.rootQ, HF.rootP);
const xAng = q => 2 * Math.atan2(q.x, q.w) / D;
const lerp = (a, b, u) => a + (b - a) * u;
const s01 = (u, a, b) => ease(Math.max(0, Math.min(1, (u - a) / (b - a))));
const bump = (u, a, b) => (u <= a || u >= b ? 0 : Math.sin(Math.PI * (u - a) / (b - a)));
const bump2 = (u, a, b) => (u <= a || u >= b ? 0 : 0.5 * (1 - Math.cos(2 * Math.PI * (u - a) / (b - a))));

// ---------- fingers and hands (as in poses_build) ----------
function fingers(left, o) {
  const spread = o.spread || [0, 0, 0, 0, 0];
  const fl = o.flex || [[0, 0, 0]];
  for (let i = 0; i < 4; i++) {
    const n = 'f' + (i + 2);
    const ab = spread[i + 1] || 0;
    const abQ = i <= 1 ? qAx(X, -ab) : qAx(X, ab);
    const f = fl[i] || fl[0];
    left[n + 'a'] = abQ.multiply(qAx(Z, -f[0]));
    left[n + 'b'] = qAx(Z, -f[1]);
    left[n + 'c'] = qAx(Z, -f[2]);
  }
  const t = o.thumb || {};
  left.f1m = qAx(X, -(t.abd || 0)).multiply(qAx(Z, -(t.flex || 0)));
  left.f1a = qAx(Z, -(t.mcp || 0));
  left.f1b = qAx(Z, -(t.ip || 0));
}
const FIST = { spread: [0, 0, 0, 0, 0], flex: [[85, 100, 60], [85, 100, 60], [85, 100, 60], [85, 100, 60]], thumb: { abd: -5, flex: 40, mcp: 35, ip: 25 } };
const SOFT = { spread: [0, 4, 0, 3, 6], flex: [[15, 22, 12], [15, 22, 12], [15, 22, 12], [15, 22, 12]], thumb: { abd: 18, flex: 5, mcp: 5, ip: 5 } };
// palm resting on the mat, fingers long and slightly apart
const FLAT = { spread: [0, 8, 0, 6, 12], flex: [[7.5, 0, 0], [5.7, 0, 0], [4.8, 0, 0], [2.8, 0, 0]], thumb: { abd: 30, flex: 0, mcp: 0, ip: 0 } };
const lerpHand = (a, b, u) => ({
  spread: a.spread.map((v, i) => lerp(v, b.spread[i], u)),
  flex: a.flex.map((f, i) => f.map((v, k) => lerp(v, b.flex[i][k], u))),
  thumb: Object.fromEntries(Object.keys(a.thumb).map(k => [k, lerp(a.thumb[k], b.thumb[k], u)])),
});
function handBasis(fingersDir, palmNormal) {
  const g = fingersDir.clone().normalize();
  const n = palmNormal.clone(); n.addScaledVector(g, -n.dot(g)); n.normalize();
  const Xw = n.clone().negate(), Yw = g.clone().negate(), Zw = Xw.clone().cross(Yw).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(Xw, Yw, Zw));
}

// ---------- anchor parameters ----------
const SPINE = ['lumA', 'lumB', 'thA', 'thB', 'thC', 'neckA', 'neckB', 'head'];
const thighXZ = q => { const e = new THREE.Euler().setFromQuaternion(q, 'XZY'); return [e.x / D, e.z / D]; };
function paramsOf(c) {
  const [hx, hz] = thighXZ(c.L.thigh_L);
  return { spine: SPINE.map(s => xAng(c.L[s])), hipExt: hx, hipAdd: hz, knee: xAng(c.L.shank_L), foot: xAng(c.L.foot_L), toes: xAng(c.L.toes_L) };
}
const P0 = { ...paramsOf(PL), scap: [18, 4, 10, 8, 0], hand: FIST };
const P4 = { ...paramsOf(HF), scap: [20, 8, -20, 8, -0.012], hand: SOFT };
const TWIST4 = HF.L.ulna_L.clone().invert(), SWING4 = HF.L.hand_L.clone();
const E0 = WPL.fore_L.p.clone(), T0 = WPL.hand_L.p.clone();
const E4 = WHF.fore_L.p.clone(), T4 = WHF.hand_L.p.clone();
const D4 = E4.clone().sub(WHF.hum_L.p).normalize(), F4 = T4.clone().sub(E4).normalize();
const HANDQ0 = WPL.hand_L.q.clone();
const REST_Y = 0.0015;
const DEBUG = { hand: false };
const ARM_LM = {
  fore: LANDMARKS.map((l, i) => [i, l]).filter(([i, l]) => l.side === 'L' && (l.region === 'предплечья' || l.region === 'локти')).map(([i]) => i),
  hand: LANDMARKS.map((l, i) => [i, l]).filter(([i, l]) => l.side === 'L' && ['ладони', 'пальцы рук', 'кулаки', 'кисти'].includes(l.region)).map(([i]) => i),
};

function slerpDir(a, b, u) {
  const q = new THREE.Quaternion().setFromUnitVectors(a, b);
  return a.clone().applyQuaternion(new THREE.Quaternion().slerp(q, u)).normalize();
}

// ---------- configuration from parameters ----------
// arm modes: planted {E, T} — elbow and wrist held; reach {xE, yE, dyT, hu} — forearm on the mat in front of the shoulder,
// the elbow where the upper arm meets the mat, the hand turning palm down (hu 0..1); free {u, uf, lift, liftE, d3, f3, twist0, swing0, tw, sw}
function build(v) {
  const L = {}; for (const s of SEG) L[s.name] = new THREE.Quaternion();
  SPINE.forEach((s, i) => { L[s] = qAx(X, v.spine[i]); });
  const left = {};
  left.thigh = qAx(X, v.hipExt).multiply(qAx(Z, v.hipAdd));
  left.shank = qAx(X, v.knee);
  left.foot = qAx(X, v.foot);
  left.toes = qAx(X, v.toes ?? 0);
  const sp = scapulaPose(v.scap[0], v.scap[1], v.scap[2], null, v.scap[3], v.scap[4]);
  left.clav = sp.clav; left.scap = sp.scap;
  left.hum = PL.L.hum_L.clone(); left.fore = PL.L.fore_L.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  fingers(left, v.hand);
  for (const k in left) { L[k + '_L'] = left[k].clone(); L[k + '_R'] = mirrorQ(left[k]); }
  const rootQ = qAx(X, v.phi), rootP = V(0, v.y, v.z);
  if (v.toesW) {
    const W = computeWorld(L, rootQ, rootP);
    const tl = W.foot_L.q.clone().invert().multiply(v.toesW);
    L.toes_L = tl; L.toes_R = mirrorQ(tl);
  }
  const a = v.arm;
  let Et, Tt;
  if (a.mode === 'planted') { Et = a.E; Tt = a.T; }
  else if (a.mode === 'reach') {
    const W = computeWorld(L, rootQ, rootP);
    const S = W.hum_L.p;
    const dy = S.y - a.yE, dx = S.x - a.xE;
    const h = Math.sqrt(Math.max(0, LH * LH - dy * dy - dx * dx));
    Et = V(a.xE, a.yE, S.z + h - (a.dz0 || 0) * (1 - (a.kz ?? 1)));
    Tt = Et.clone().add(V(0, a.dyT, 1).normalize().multiplyScalar(LF));
  } else {
    const W = computeWorld(L, rootQ, rootP);
    const S = W.hum_L.p;
    let d = slerpDir(a.d3, D4, a.u);
    if (a.liftE) d = d.add(V(0, a.liftE, 0)).normalize();
    let f = slerpDir(a.f3, F4, a.uf);
    if (a.lift) f = f.add(V(0, a.lift, 0)).normalize();
    Et = S.clone().addScaledVector(d, LH); Tt = Et.clone().addScaledVector(f, LF);
  }
  let flexQ = armIK(L, rootQ, rootP, 'L', Tt, Et, null);
  if (a.mode === 'reach') {
    // hand from the fist of the plank to a palm flat on the mat in line with the forearm; forearm and hand are let down
    // (or lifted) until their lowest points rest on the mat
    const orient = () => {
      const W = computeWorld(L, rootQ, rootP);
      const fd = W.hand_L.p.clone().sub(W.fore_L.p); fd.y = 0; fd.normalize();
      orientHand(L, rootQ, rootP, 'L', HANDQ0.clone().slerp(handBasis(fd, V(0, -1, 0)), a.hu), flexQ);
    };
    for (let it = 0; it < 5; it++) {
      orient();
      const W2 = computeWorld(L, rootQ, rootP);
      let mf = Infinity, mh = Infinity;
      for (const k of ARM_LM.fore) mf = Math.min(mf, pt(W2, LANDMARKS[k].seg, LANDMARKS[k].pc).y);
      for (const k of ARM_LM.hand) mh = Math.min(mh, pt(W2, LANDMARKS[k].seg, LANDMARKS[k].pc).y);
      const dE = REST_Y - mf, dT = REST_Y + (a.lift || 0) - mh;
      if (DEBUG.hand) console.log('    rest it', it, 'mf', (mf * 1000).toFixed(1), 'mh', (mh * 1000).toFixed(1), 'Et', Et.toArray().map(q => q.toFixed(3)).join(','), 'Tt', Tt.toArray().map(q => q.toFixed(3)).join(','), 'wrist', W2.hand_L.p.toArray().map(q => q.toFixed(3)).join(','));
      if (Math.abs(dE) < 2e-4 && Math.abs(dT) < 2e-4) break;
      const S = W2.hum_L.p;
      const yE = Et.y + dE, dy = S.y - yE, dx = S.x - a.xE;
      const h = Math.sqrt(Math.max(0, LH * LH - dy * dy - dx * dx));
      // only the height is corrected: near a vertical upper arm a tiny height change would throw the elbow far forward
      Et = V(a.xE, yE, Et.z);
      Tt = V(Tt.x, Tt.y + dT, Et.z + Math.sqrt(Math.max(0, LF * LF - (Tt.y + dT - yE) ** 2 - (Tt.x - a.xE) ** 2)));
      flexQ = armIK(L, rootQ, rootP, 'L', Tt, Et, null);
      if (it === 4) orient();
    }
    // the first moments of the slide start from the planted elbow and wrist (no jump off the planted position)
    if (a.k != null && a.k < 1) {
      Et = a.E0.clone().lerp(Et, a.k); Tt = a.T0.clone().lerp(Tt, a.k);
      flexQ = armIK(L, rootQ, rootP, 'L', Tt, Et, null);
      orient();
      // keep the blended forearm and hand resting on the mat (no dip through it while the slide starts)
      for (let it = 0; it < 4; it++) {
        const W3 = computeWorld(L, rootQ, rootP);
        let mf = Infinity, mh = Infinity;
        for (const k of ARM_LM.fore) mf = Math.min(mf, pt(W3, LANDMARKS[k].seg, LANDMARKS[k].pc).y);
        for (const k of ARM_LM.hand) mh = Math.min(mh, pt(W3, LANDMARKS[k].seg, LANDMARKS[k].pc).y);
        const dE = REST_Y - mf, dT = REST_Y + (a.lift || 0) - mh;
        if (Math.abs(dE) < 2e-4 && Math.abs(dT) < 2e-4) break;
        if (DEBUG.hand) console.log('    blend it', it, 'mf', (mf * 1000).toFixed(1), 'mh', (mh * 1000).toFixed(1), 'Et', Et.toArray().map(q => q.toFixed(3)).join(','), 'el', W3.fore_L.p.toArray().map(q => q.toFixed(3)).join(','), 'sh', W3.hum_L.p.toArray().map(q => q.toFixed(3)).join(','));
        Et = Et.clone().add(V(0, dE, 0)); Tt = Tt.clone().add(V(0, dT, 0));
        flexQ = armIK(L, rootQ, rootP, 'L', Tt, Et, null);
        orient();
      }
    }
  } else {
    const tw = a.mode === 'free' ? a.twist0.clone().slerp(TWIST4, a.tw) : IDQ.clone();
    const sw = a.mode === 'free' ? a.swing0.clone().slerp(SWING4, a.sw) : IDQ.clone();
    L.fore_L = flexQ.clone().multiply(tw); L.ulna_L = tw.clone().invert(); L.hand_L = sw;
  }
  for (const s of ['hum', 'fore', 'ulna', 'hand']) L[s + '_R'] = mirrorQ(L[s + '_L']);
  return { L, rootQ, rootP, Et, Tt };
}

// ---------- least squares: Levenberg–Marquardt with a numerical Jacobian ----------
const sumsq = r => { let c = 0; for (const v of r) c += v * v; return c; };
function linSolve(M, b) {
  const n = b.length, a = M.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    const d = a[c][c] || 1e-12;
    for (let r = 0; r < n; r++) { if (r === c) continue; const f = a[r][c] / d; for (let k = c; k <= n; k++) a[r][k] -= f * a[c][k]; }
  }
  return a.map((row, i) => row[n] / (row[i] || 1e-12));
}
function solveLM(x0, f, steps, iters = 60) {
  let x = x0.slice(), r = f(x), c = sumsq(r), lam = 1e-3;
  const n = x.length;
  for (let it = 0; it < iters; it++) {
    const J = [];
    for (let k = 0; k < n; k++) { const xp = x.slice(); xp[k] += steps[k]; const rp = f(xp); J.push(rp.map((v, i) => (v - r[i]) / steps[k])); }
    const M = [], g = [];
    for (let i = 0; i < n; i++) { M.push([]); let gi = 0; for (let m = 0; m < r.length; m++) gi += J[i][m] * r[m]; g.push(gi); for (let j = 0; j < n; j++) { let s = 0; for (let m = 0; m < r.length; m++) s += J[i][m] * J[j][m]; M[i].push(s); } }
    let ok = false;
    for (let tr = 0; tr < 10; tr++) {
      const Ml = M.map((row, i) => row.map((v, j) => (i === j ? v * (1 + lam) + 1e-12 : v)));
      const dx = linSolve(Ml, g.map(v => -v));
      const xn = x.map((v, i) => v + dx[i]);
      const rn = f(xn), cn = sumsq(rn);
      if (cn < c) { ok = c - cn > 1e-14 * (1 + c); x = xn; r = rn; c = cn; lam = Math.max(lam / 4, 1e-9); break; }
      lam *= 6;
    }
    if (!ok) break;
  }
  return { x, cost: c, r };
}

// ---------- the motion ----------
const FPS = 10;
const TUNE = JSON.parse(process.env.SEG2_TUNE || '{}');
const tv = (k, d) => (TUNE[k] ?? d);
const T_LOW = 1.6, T_FEET = tv('tFeet', 2.9), T_REACH = tv('tReach', 4.9), DUR = tv('dur', 6.4);
const SPH_SPINE = tv('sph', [-8, -8, -4, -3, -3, 3, 2, 0]);        // on the forearms with the pelvis down: the lower back takes most of the extension
const PRONE_SPINE = tv('prone', [-2, -2, -2, -3, -3, -7, -5, -3]); // lying long: chest on the mat, neck lengthened so the face clears the mat
const SPH_SCAP = [17, 3, 6, 8, 0], PRONE_SCAP = tv('proneScap', [36, 10, 2, 16, 0.002]);
const idx = pred => LANDMARKS.map((l, i) => [i, l]).filter(([i, l]) => pred(l)).map(([i]) => i);
const LM_TOES = [10, 12, 14];            // toe tip, toe pad, toe underside (left)
const LM_PUB = 63;                        // pelvis on the floor (left)
const LM_THIGH = [46, 48, 50, 52];        // front of the thigh (left)
const LM_TOP = [18, 20];                  // top of the toes (left)
const LM_RIBS = [82, 85];                 // lower ribs (centre)
const LM_CHEST = [88, 91];                // chest (centre)
const HAND_REGIONS = new Set(['ладони', 'пальцы рук', 'кулаки', 'кисти', 'предплечья', 'локти', 'плечи']);
const CHECK = idx(l => l.side !== 'R');
const PEN_W = tv('penW', 3000), HOVER = 0.0065;

function landmarksOf(W) { return LANDMARKS.map(l => pt(W, l.seg, l.pc)); }
// x = [phi, y, z, s, hipExt, foot]
function vOf(x, sp) {
  const s = x[3];
  const spine = sp.spine.map((a, i) => (i < 5 ? a * s : a));
  return { phi: x[0], y: x[1], z: x[2], spine, hipExt: x[4], hipAdd: sp.hipAdd, knee: sp.knee, foot: x[5], toes: sp.toes, toesW: sp.toesW, scap: sp.scap, hand: sp.hand, arm: sp.arm };
}
function residualGroups(x, sp) {
  const v = vOf(x, sp);
  const c = build(v);
  const W = computeWorld(c.L, c.rootQ, c.rootP);
  const P = landmarksOf(W);
  const G = { fix: [], arm: [], floor: [], soft: [], reg: [] };
  const fixed = new Set();
  for (const f of sp.fix) {
    fixed.add(f.i);
    if (f.p) G.fix.push((P[f.i].x - f.p.x) * f.w, (P[f.i].y - f.p.y) * f.w, (P[f.i].z - f.p.z) * f.w);
    else { G.fix.push((P[f.i].y - f.y) * f.w); if (f.z != null) G.fix.push((P[f.i].z - f.z) * (f.wz ?? f.w)); }
  }
  if (sp.arm.mode === 'planted') {
    const e = W.fore_L.p.clone().sub(sp.arm.E), t = W.hand_L.p.clone().sub(sp.arm.T);
    G.arm.push(e.x * 150, e.y * 150, e.z * 150, t.x * 150, t.y * 150, t.z * 150);
  }
  if (sp.arm.mode === 'reach') {
    // the sliding forearm must not dip through the mat (the trunk adapts instead)
    let mf = Infinity; for (const k of ARM_LM.fore) mf = Math.min(mf, P[k].y);
    G.floor.push(Math.min(0, mf - 0.0005) * tv('wForeDip', 3000));
  }
  for (const k of CHECK) {
    if (fixed.has(k)) continue;
    const l = LANDMARKS[k];
    if (HAND_REGIONS.has(l.region)) continue;
    const thr = sp.mayTouch.has(l.region) ? 0 : HOVER;
    G.floor.push(Math.min(0, P[k].y - thr) * PEN_W);
  }
  for (const s of sp.soft || []) G.soft.push((s.f(W, P) - s.target) * s.w);
  for (const g of sp.reg) G.reg.push((x[g.k] - g.target) * g.w);
  return G;
}
const residuals = (x, sp) => { const G = residualGroups(x, sp); return [...G.fix, ...G.arm, ...G.floor, ...G.soft, ...G.reg]; };
const STEPS = [0.01, 1e-5, 1e-5, 1e-4, 0.01, 0.01];

const phi0 = xAng(PL.rootQ), phi4 = xAng(HF.rootQ);
const P_PL = landmarksOf(WPL), P_HF = landmarksOf(WHF);

const R = (k, target, w) => ({ k, target, w });
// the shoulder comes down so that the elbow (at the far end of a near-vertical upper arm) slides forward smoothly:
// the horizontal reach of the upper arm, not the shoulder height, follows the easing
function shoulderY(st, u) {
  const dx = st.shXE, yE = st.yE0, LHx = Math.sqrt(LH * LH - dx * dx);
  const h0 = Math.sqrt(Math.max(0, LHx * LHx - (st.shY - yE) ** 2)), h1 = Math.sqrt(Math.max(0, LHx * LHx - (tv('shProne', 0.17) - yE) ** 2));
  const h = lerp(h0, h1, u);
  return yE + Math.sqrt(Math.max(0, LHx * LHx - h * h));
}   // k: 0 phi, 1 y, 2 z, 3 spine scale, 4 hip extension, 5 ankle
function specAt(t, st) {
  if (t <= T_LOW + 1e-9) {
    // the pelvis comes down to where it lies in the half; toes and forearms stay put, the shoulders settle a little
    // behind the elbows (sphinx)
    const u = ease(t / T_LOW);
    return {
      spine: SPINE.map((s, k) => lerp(P0.spine[k], SPH_SPINE[k], u)), hipAdd: P0.hipAdd, knee: 0, toes: null, toesW: WPL.toes_L.q,
      scap: P0.scap.map((a, k) => lerp(a, SPH_SCAP[k], u)), hand: FIST, arm: { mode: 'planted', E: E0, T: T0 },
      fix: [...LM_TOES.map(k => ({ i: k, p: P_PL[k], w: 300 })), { i: LM_PUB, y: lerp(P_PL[LM_PUB].y, 0.0010, u), z: lerp(P_PL[LM_PUB].z, P_HF[LM_PUB].z, u), w: 400, wz: tv('wzA', 60) }],
      mayTouch: new Set(['бёдра', 'колени', 'таз', 'живот', 'голени']),
      soft: [{ f: W => W.hum_L.p.z, target: lerp(E0.z, E0.z - tv('shBack', 0.05), u), w: tv('wShA', 30) }],
      reg: [R(0, lerp(phi0, tv('phiA', 84), u), tv('wPhiA', 0.3)), R(3, 1, tv('wsA', 10)), R(4, lerp(P0.hipExt, 8, u), 0.1), R(5, P0.foot, 0.5)],
    };
  }
  if (t <= T_FEET + 1e-9) {
    // feet onto their tops: the knees soften to free the toes, the feet point, the legs lie long again
    const w = (t - T_LOW) / (T_FEET - T_LOW);
    const sp = {
      spine: SPH_SPINE.slice(), hipAdd: lerp(P0.hipAdd, -0.7, s01(w, 0.1, 0.9)), knee: tv('kneeU', 18) * bump2(w, 0, 0.85), toes: lerp(st.toesA, P4.toes, s01(w, 0.05, 0.95)), toesW: null,
      scap: SPH_SCAP.slice(), hand: FIST, arm: { mode: 'planted', E: E0, T: T0 },
      fix: [{ i: LM_PUB, p: st.pubA, w: 300 }, ...st.thighA.map(k => ({ i: k, p: st.thighP[k], w: 200 }))],
      mayTouch: new Set(['бёдра', 'колени', 'таз', 'живот', 'голени']),
      soft: [],
      reg: [R(0, st.xA[0], 3), R(3, st.xA[3], 10), R(4, lerp(st.xA[4], tv('hipU', 8.7), s01(w, 0.12, 0.92)), 6), R(5, lerp(st.xA[5], P4.foot, s01(w, 0.05, 0.95)), 3)],
    };
    if (w > 0.45) sp.mayTouch.add('тыл стоп');
    if (w > 0.8) {
      const k = s01(w, 0.8, 1);
      sp.soft.push({ f: (W, P) => Math.min(...LM_TOP.map(i => P[i].y)), target: 0.0012, w: 150 * k });
      sp.soft.push({ f: (W, P) => Math.min(...LM_THIGH.map(i => P[i].y)), target: 0.0016, w: 80 * k });
    }
    return sp;
  }
  if (t <= T_REACH + 1e-9) {
    // «Вытягиваю рёбра вперёд»: forearms slide forward, palms turn down, the chest comes down long
    const lin = (t - T_FEET) / (T_REACH - T_FEET), u = ease(lin), u2 = u * u;
    return {
      spine: SPINE.map((s, k) => lerp(SPH_SPINE[k], PRONE_SPINE[k], u2)), hipAdd: lerp(-0.7, 0, u), knee: 0, toes: P4.toes, toesW: null,
      scap: SPH_SCAP.map((a, k) => lerp(a, PRONE_SCAP[k], u)), hand: lerpHand(FIST, FLAT, s01(lin, 0.05, 0.9)),
      arm: { mode: 'reach', xE: E0.x, yE: st.yE0, dyT: 0, hu: s01(lin, 0.08, 0.8), k: null, dz0: st.dz0, kz: s01(lin, 0, tv('kzEnd', 0.4)), lift: tv('handLift', 0.03) * bump(lin, 0.04, 0.97), E0, T0 },
      fix: [{ i: LM_PUB, y: lerp(st.pubA.y, 0.0012, u), z: st.pubA.z, w: tv('wPubB', 1500) }, ...st.topU.map(k => ({ i: k, p: st.topP[k], w: 150 }))],
      mayTouch: new Set(['бёдра', 'колени', 'таз', 'живот', 'голени', 'тыл стоп', 'нижние рёбра', 'грудь']),
      soft: [{ f: W => W.hum_L.p.y, target: shoulderY(st, u), w: tv('wShB', 80) }, { f: (W, P) => Math.min(...LM_RIBS.map(k => P[k].y)), target: 0.001, w: 150 * s01(u, 0.55, 1) }],
      reg: [R(0, lerp(st.xU[0], tv('phiB', 87), u2), tv('wPhiB', 0.5)), R(3, lerp(st.xU[3], 1, u2), 10), R(4, lerp(st.xU[4], tv('hipB', 2), u2), 0.3), R(5, P4.foot, 0.5)],
    };
  }
  // «Свожу лопатки — полупрогиб»: arms up, elbows drawn out and back to shoulder height, the chest lifts on the lower ribs
  const u = (t - T_REACH) / (DUR - T_REACH), ue = ease(u);
  const wA = u * u * u;
  return {
    spine: SPINE.map((s, k) => lerp(PRONE_SPINE[k], P4.spine[k], ue)), hipAdd: lerp(0, P4.hipAdd, ue), knee: 0, toes: P4.toes, toesW: null,
    scap: PRONE_SCAP.map((a, k) => lerp(a, P4.scap[k], s01(u, 0, 0.85))),
    hand: lerpHand(FLAT, SOFT, s01(u, 0.1, 0.7)),
    arm: { mode: 'free', u: s01(u, 0, 0.92), uf: s01(u, 0.05, 0.95), lift: tv('liftF', 0.25) * bump2(u, 0, 0.8), liftE: tv('liftE', 0.25) * bump2(u, 0, 0.85), d3: st.d3, f3: st.f3, twist0: st.twistB, swing0: st.swingB, tw: s01(u, 0.05, 0.85), sw: s01(u, 0.15, 0.95) },
    fix: [{ i: LM_PUB, y: lerp(st.pubB.y, P_HF[LM_PUB].y, ue), z: lerp(st.pubB.z, P_HF[LM_PUB].z, ue), w: tv('wPubC', 1500) },
      ...LM_TOP.map(k => ({ i: k, y: lerp(st.topB[k].y, P_HF[k].y, ue), z: lerp(st.topB[k].z, P_HF[k].z, ue), w: 120 }))],
    mayTouch: new Set(['бёдра', 'колени', 'таз', 'живот', 'голени', 'тыл стоп', 'нижние рёбра', 'грудь']),
    soft: [{ f: (W, P) => Math.min(...LM_RIBS.map(k => P[k].y)), target: 0.0008, w: 150 }],
    reg: [R(0, lerp(st.xB[0], phi4, ue), 1 + 4 * wA), R(1, lerp(st.xB[1], HF.rootP.y, ue), 600 * wA), R(2, lerp(st.xB[2], HF.rootP.z, ue), 600 * wA),
      R(3, lerp(st.xB[3], 1, ue), 10), R(4, lerp(st.xB[4], P4.hipExt, ue), 1 + 3 * wA), R(5, lerp(st.xB[5], P4.foot, ue), 1 + 3 * wA)],
  };
}

function hull(P) {
  const Qp = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (Qp.length < 3) return Qp;
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Qp) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Qp.length - 1; i >= 0; i--) { const p = Qp[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
function margin(poly, p) {
  if (poly.length < 3) return -1;
  let inside = true, dmin = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const ex = b[0] - a[0], ez = b[1] - a[1];
    if (ex * (p[1] - a[1]) - ez * (p[0] - a[0]) < 0) inside = false;
    const L2 = ex * ex + ez * ez, tt = Math.max(0, Math.min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ez) / (L2 || 1)));
    dmin = Math.min(dmin, Math.hypot(p[0] - a[0] - tt * ex, p[1] - a[1] - tt * ez));
  }
  return inside ? dmin : -dmin;
}
function diag(conf) {
  const W = computeWorld(conf.L, conf.rootQ, conf.rootP);
  const P = landmarksOf(W);
  const inC = P.map(p => p.y < 0.004);
  const com = comOf(W);
  const m = margin(hull(P.filter((p, k) => inC[k]).map(p => [p.x, p.z])), [com.x, com.z]);
  const noArms = margin(hull(P.filter((p, k) => inC[k] && !HAND_REGIONS.has(LANDMARKS[k].region)).map(p => [p.x, p.z])), [com.x, com.z]);
  let minY = Infinity, minL = '', minArm = Infinity, minArmL = '';
  P.forEach((p, k) => {
    const nm = LANDMARKS[k].region + ':' + (LANDMARKS[k].tag || '') + ':' + LANDMARKS[k].side;
    if (p.y < minY) { minY = p.y; minL = nm; }
    if (HAND_REGIONS.has(LANDMARKS[k].region) && p.y < minArm) { minArm = p.y; minArmL = nm; }
  });
  const hover = P.filter(p => p.y >= 0.004 && p.y < 0.006).length;
  return { W, P, com, margin: m, marginNoArms: noArms, minY, minL, minArm, minArmL, hover };
}

function run(verbose) {
  const frames = [];
  const n = Math.round(DUR * FPS);
  let x = [phi0, PL.rootP.y, PL.rootP.z, 1, P0.hipExt, P0.foot];
  const st = { yE0: E0.y };
  const report = [];
  for (let i = 0; i <= n; i++) {
    const t = +(i / FPS).toFixed(2);
    const sp = specAt(t, st);
    let conf;
    if (i === 0) conf = { L: PL.L, rootQ: PL.rootQ, rootP: PL.rootP };
    else if (i === n) conf = { L: HF.L, rootQ: HF.rootQ, rootP: HF.rootP };
    else {
      const res = solveLM(x, xx => residuals(xx, sp), STEPS, 60);
      x = res.x;
      DEBUG.hand = process.env.DBG_T && Math.abs(t - +process.env.DBG_T) < 1e-9;
      conf = build(vOf(x, sp));
      DEBUG.hand = false;
      const G = residualGroups(x, sp);
      report.push({ i, t, cost: res.cost, groups: Object.fromEntries(Object.entries(G).map(([k, r]) => [k, +sumsq(r).toExponential(1)])) });
    }
    const d = diag(conf);
    const W = d.W, P = d.P;
    if (Math.abs(t - T_LOW) < 1e-9) {
      st.pubA = P[LM_PUB].clone(); st.thighP = P.map(p => p.clone());
      const low = LM_THIGH.map(k => [k, P[k].y]).sort((a, b) => a[1] - b[1]);
      st.thighA = low.filter(([k, y]) => y < 0.004).map(([k]) => k);
      st.xA = x.slice(); st.toesA = xAng(conf.L.toes_L);
    }
    if (Math.abs(t - T_FEET) < 1e-9) {
      st.xU = x.slice(); st.topP = P.map(p => p.clone()); st.shY = W.hum_L.p.y; st.shXE = W.hum_L.p.x - E0.x;
      st.topU = LM_TOP.filter(k => P[k].y < 0.004); if (!st.topU.length) st.topU = [LM_TOP[0]];
      st.yE0 = W.fore_L.p.y;
      { const S = W.hum_L.p, h = Math.sqrt(Math.max(0, LH * LH - (S.y - st.yE0) ** 2 - (S.x - E0.x) ** 2)); st.dz0 = S.z + h - W.fore_L.p.z; if (process.env.DBG_DZ) console.log('dz0', st.dz0, 'S', S.toArray(), 'h', h); }
    }
    if (Math.abs(t - T_REACH) < 1e-9) {
      st.pubB = P[LM_PUB].clone(); st.topB = P.map(p => p.clone()); st.xB = x.slice();
      st.d3 = W.fore_L.p.clone().sub(W.hum_L.p).normalize(); st.f3 = W.hand_L.p.clone().sub(W.fore_L.p).normalize();
      st.twistB = conf.L.ulna_L.clone().invert(); st.swingB = conf.L.hand_L.clone();
    }
    frames.push({ t, conf, x: x.slice(), d });
  }
  // the last frames ease into the exact anchor (removes the small difference left by the solve)
  const SNAP = 0.5;
  for (const fr of frames) {
    if (fr.t <= DUR - SNAP || fr.t >= DUR) continue;
    const m = ease((fr.t - (DUR - SNAP)) / SNAP);
    const L = {}; for (const s of SEG) L[s.name] = fr.conf.L[s.name].clone().slerp(HF.L[s.name], m);
    fr.conf = { L, rootQ: fr.conf.rootQ.clone().slerp(HF.rootQ, m), rootP: fr.conf.rootP.clone().lerp(HF.rootP, m) };
    fr.d = diag(fr.conf);
  }
  if (verbose) for (const fr of frames) {
    const r = report.find(q => q.i === Math.round(fr.t * FPS));
    const W = fr.d.W;
    console.log(String(fr.t).padStart(4), 'x', fr.x.map(v => v.toFixed(3)).join(' '), '| sh', W.hum_L.p.y.toFixed(3), W.hum_L.p.z.toFixed(3), 'el', W.fore_L.p.y.toFixed(3), W.fore_L.p.z.toFixed(3), 'wr', W.hand_L.p.y.toFixed(3), W.hand_L.p.z.toFixed(3),
      '| thigh', (Math.min(...LM_THIGH.map(k => fr.d.P[k].y)) * 1000).toFixed(1), 'top', (Math.min(...LM_TOP.map(k => fr.d.P[k].y)) * 1000).toFixed(1), 'pub', (fr.d.P[LM_PUB].y * 1000).toFixed(1), 'chest', (Math.min(...LM_CHEST.map(k => fr.d.P[k].y)) * 1000).toFixed(1), 'ribs', (Math.min(...LM_RIBS.map(k => fr.d.P[k].y)) * 1000).toFixed(1), '| minY', (fr.d.minY * 1000).toFixed(1), fr.d.minL, 'arm', (fr.d.minArm * 1000).toFixed(1), fr.d.minArmL, 'hov', fr.d.hover, '| marg', (fr.d.margin * 100).toFixed(1), 'noArms', (fr.d.marginNoArms * 100).toFixed(1), r ? '| ' + JSON.stringify(r.groups) : '');
  }
  return { frames, report };
}

function save(frames) {
  const r6 = v => +v.toFixed(6);
  const out = {
    id: 'seg2', from: 'plank', to: 'half', fps: FPS, duration: DUR,
    phases: [{ t0: 0, t1: T_FEET, label: 'Опускаюсь на живот' }, { t0: T_FEET, t1: T_REACH, label: 'Вытягиваю рёбра вперёд' }, { t0: T_REACH, t1: DUR, label: 'Свожу лопатки — полупрогиб' }],
    frames: frames.map(fr => {
      const o = { t: fr.t, local: Object.fromEntries(SEG.map(s => [s.name, fr.conf.L[s.name].toArray().map(r6)])), root: fr.conf.rootQ.toArray().map(r6), rootP: fr.conf.rootP.toArray().map(r6), grip: [] };
      if (fr.t > 0 && fr.t <= T_LOW) o.note = 'таз опускается, предплечья и пальцы стоп стоят на месте';
      else if (fr.t > T_LOW && fr.t <= T_FEET) o.note = 'стопы переворачиваются на тыл: колени чуть сгибаются';
      else if (fr.t > T_FEET && fr.t <= T_REACH) o.note = 'скольжение: предплечья и ладони едут вперёд по коврику, грудь ложится';
      else if (fr.t > T_REACH && fr.t < DUR) o.note = 'руки отрываются от пола, локти уходят в стороны и назад на высоту плеч';
      return o;
    }),
  };
  fs.writeFileSync(process.env.SEG2_OUT || 'cycle_keys/seg2.json', JSON.stringify(out));
  return out;
}

export { build, P0, P4, PL, HF, WPL, WHF, E0, T0, E4, T4, D4, F4, TWIST4, SWING4, FIST, SOFT, FLAT, lerpHand, paramsOf, fingers };

if (process.argv[2] === 'test') {
  const angDeg = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(a.dot(b)))) / D;
  const cmp = (c, ref, name) => {
    let worst = 0, ws = null;
    for (const s of SEG) { const a = angDeg(c.L[s.name], ref.L[s.name]); if (a > worst) { worst = a; ws = s.name; } }
    const W = computeWorld(c.L, c.rootQ, c.rootP), Wr = computeWorld(ref.L, ref.rootQ, ref.rootP);
    let pw = 0; for (const s of SEG) pw = Math.max(pw, W[s.name].p.distanceTo(Wr[s.name].p));
    console.log(name, 'max joint diff', worst.toFixed(3), 'deg at', ws, '| max position diff', (pw * 1000).toFixed(2), 'mm');
  };
  cmp(build({ phi: phi0, y: PL.rootP.y, z: PL.rootP.z, ...P0, arm: { mode: 'planted', E: E0, T: T0 } }), PL, 'plank');
  const d3 = E0.clone().sub(WPL.hum_L.p).normalize(), f3 = T0.clone().sub(E0).normalize();
  cmp(build({ phi: phi4, y: HF.rootP.y, z: HF.rootP.z, ...P4, arm: { mode: 'free', u: 1, uf: 1, d3, f3, twist0: IDQ, swing0: IDQ, tw: 1, sw: 1 } }), HF, 'half ');
} else {
  const t0 = Date.now();
  const { frames, report } = run(process.argv.includes('-v'));
  save(frames);
  const worst = report.reduce((a, b) => (b.cost > a.cost ? b : a), report[0]);
  console.log('seg2:', frames.length, 'frames in', Date.now() - t0, 'ms; worst residual cost', worst.cost.toExponential(2), 'at t', worst.t);
}
