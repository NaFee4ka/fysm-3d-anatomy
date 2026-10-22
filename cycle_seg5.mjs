// seg5: Планка на ладонях → Наклон. «Подшагиваю правой ногой», «Подшагиваю левой», «Поднимаюсь в наклон».
// writes cycle_keys/fold.json (fold placed where the feet land) and cycle_keys/seg5.json (fps 10).
// usage: node cycle_seg5.mjs [--dry] [--diag]   (--dry: seg5 to the scratchpad; --diag: per-frame numbers)
//
// While the palms are on the mat the trunk is driven by two points: the pelvis and the middle of the shoulders
// (the pitch of the pelvis, the rounding of the spine and a small side bend are solved to put the shoulders there),
// so the arms always reach their planted hands. Planted toes keep their place and the heel rises as the hips travel;
// a swinging foot flies on a curve under the body and lands flat on its place in the fold.
import fs from 'fs';
import { A, V, THREE, S, SEG, qAx, X, Y, Z, computeWorld, pt, mirrorQ, mirP, LANDMARKS, D, IDQ, comOf, scapulaPose } from './cycle_lib.mjs';
import { SPREAD, GRIP, blendFingers, fromData, toData, bisect } from './seg5_util.mjs';
import { solveBody, pchip, smooth, smoother, seg01, lerp, slerpQ, ankleFromMTP, REACH0, measureConf } from './seg5_rig.mjs';

const DRY = process.argv.includes('--dry'), DIAG = process.argv.includes('--diag');
const SCR = '/tmp/claude-0/-home-claude/e37a82bc-007b-5e07-be95-11ef82530e9e/scratchpad/';
const FPS = 10, DUR = 5.6;
const ZFT = 0.12; // toe tips of the fold on the mat (the steps land here)
const LT = S.shank_L.J.distanceTo(S.thigh_L.J), LS = S.foot_L.J.distanceTo(S.shank_L.J);

// ---------- anchors ----------
const hpD = JSON.parse(fs.readFileSync('cycle_keys/highplank.json', 'utf8'));
const hp = fromData(hpD);
const Whp = computeWorld(hp.L, hp.rootQ, hp.rootP);
const poses = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const fc = A.poseFromData(poses.fold);
const foldRootP = A.placeW(fc.local, fc.rootQ).pelvis.p.clone().add(V(0, 0, ZFT + 0.004));
const fold = { L: fc.local, rootQ: fc.rootQ, rootP: foldRootP };
const Wf = computeWorld(fold.L, fold.rootQ, fold.rootP);
const foldData = { local: poses.fold.local, root: poses.fold.root, rootP: [foldRootP.x, foldRootP.y, foldRootP.z].map(v => +v.toFixed(6)) };
const eulerX = q => new THREE.Euler().setFromQuaternion(q, 'XYZ').x / D;
const FOLD_SPINE = ['lumA', 'lumB', 'thA', 'thB', 'thC'].map(k => eulerX(fold.L[k]));
const FOLD_PITCH = 2 * Math.acos(fold.rootQ.w) / D;

// ---------- fixed targets ----------
const HAND = { L: { wrist: Whp.hand_L.p.clone(), q: Whp.hand_L.q.clone() }, R: { wrist: Whp.hand_R.p.clone(), q: Whp.hand_R.q.clone() } };
const TOES = {}; // high-plank toes, planted
for (const sd of ['L', 'R']) TOES[sd] = { mtp: Whp['toes_' + sd].p.clone(), toesQ: Whp['toes_' + sd].q.clone(), footQ: Whp['foot_' + sd].q.clone() };
const FLAT = {}; // feet of the fold, planted after landing
for (const sd of ['L', 'R']) FLAT[sd] = { ankle: Wf['foot_' + sd].p.clone(), footQ: Wf['foot_' + sd].q.clone(), toesQ: Wf['toes_' + sd].q.clone(), toesLocal: fold.L['toes_' + sd].clone() };
const GRIPREL = {}; // grip of the fold, relative to the shank
for (const sd of ['L', 'R']) {
  const sq = Wf['shank_' + sd].q, sp = Wf['shank_' + sd].p, qi = sq.clone().invert();
  GRIPREL[sd] = { wrist: Wf['hand_' + sd].p.clone().sub(sp).applyQuaternion(qi), q: qi.clone().multiply(Wf['hand_' + sd].q), elbow: Wf['fore_' + sd].p.clone().sub(sp).applyQuaternion(qi) };
}

// ---------- timing ----------
const T = { liftR: 0.45, landR: 1.55, liftL: 2.15, landL: 3.15, handsOff: 3.45, grasp: 4.75, end: DUR };
const PHASES = [{ t0: 0, t1: 1.6, label: 'Подшагиваю правой ногой' }, { t0: 1.6, t1: 3.2, label: 'Подшагиваю левой' }, { t0: 3.2, t1: DUR, label: 'Поднимаюсь в наклон' }];

// ---------- trunk keys while the palms are down: pelvis, middle of the shoulders, pelvis roll/tilt, neck, scapulae ----------
const HP_SH = Whp.hum_L.p.clone().add(Whp.hum_R.p).multiplyScalar(0.5);
const KEYS = [
  //  t      pelvis xyz                     shoulders yz      rollY tiltZ   neck [nA nB head]   scap [up post prot clavRot elev]
  [0.00, [0.000, 0.442, -0.074], [HP_SH.y, HP_SH.z], 0, 0, [0, 0, 0], [14, 2, 12, 6, 0]],
  [0.45, [0.015, 0.500, -0.080], [0.585, 0.395], -2, 2, [-2, -2, 0], [16, 3, 13, 7, 0]],
  [1.00, [0.020, 0.570, -0.100], [0.575, 0.360], -5, 3, [-6, -4, -2], [18, 3, 14, 8, 0]],
  [1.55, [-0.010, 0.560, -0.120], [0.570, 0.300], -3, 1, [-8, -6, -4], [19, 3, 14, 8, 0]],
  [2.15, [-0.060, 0.560, -0.150], [0.570, 0.260], 2, -2, [-8, -6, -4], [20, 3, 14, 8, 0]],
  [2.65, [-0.050, 0.600, -0.135], [0.565, 0.270], 4, -3, [-6, -4, -2], [20, 3, 14, 8, 0]],
  [3.15, [0.000, 0.620, -0.130], [0.555, 0.280], 0, 0, [-4, -2, 0], [18, 2, 12, 6, 0]],
  [3.45, [0.000, 0.660, -0.128], [0.540, 0.260], 0, 0, [-2, -1, 0], [16, 2, 11, 5, 0]],
];
const fPel = pchip(KEYS.map(k => [k[0], k[1]])), fSh = pchip(KEYS.map(k => [k[0], k[2]])), fRoll = pchip(KEYS.map(k => [k[0], [k[3], k[4]]]));
const fNeck0 = pchip(KEYS.map(k => [k[0], k[5]])), fScap0 = pchip(KEYS.map(k => [k[0], k[6]]));

const SPW = [1, 1, 0.8, 0.8, 0.6], SPWS = SPW.reduce((a, b) => a + b, 0);
const NAMES = ['lumA', 'lumB', 'thA', 'thB', 'thC'];
function trunkL(sol, nk, sc) {
  const L = {}; for (const s of SEG) L[s.name] = new THREE.Quaternion();
  NAMES.forEach((k, i) => { L[k] = qAx(X, sol.spine ? sol.spine[i] : sol.F * SPW[i] / SPWS).multiply(qAx(Z, (sol.side || 0) * SPW[i] / SPWS)); });
  ['neckA', 'neckB', 'head'].forEach((k, i) => { L[k] = qAx(X, nk[i]); });
  const scp = scapulaPose(sc[0], sc[1], sc[2], null, sc[3], sc[4]);
  L.clav_L = scp.clav.clone(); L.scap_L = scp.scap.clone(); L.clav_R = mirrorQ(scp.clav); L.scap_R = mirrorQ(scp.scap);
  return L;
}
const rootQof = (pitch, roll) => qAx(X, pitch).multiply(qAx(Y, roll[0])).multiply(qAx(Z, roll[1]));
const shMid = W => W.hum_L.p.clone().add(W.hum_R.p).multiplyScalar(0.5);
// pitch, spine rounding F and side bend so that the middle of the shoulders sits on the target (x = 0)
function solveTrunk(P, Sh, roll, nk, sc, guess) {
  let x = [guess.phi, guess.F, guess.side];
  const f = v => { const L = trunkL({ F: v[1], side: v[2] }, nk, sc); const W = computeWorld(L, rootQof(v[0], roll), P); const m = shMid(W); return [m.x - 0, m.y - Sh[0], m.z - Sh[1]]; };
  for (let it = 0; it < 20; it++) {
    const r = f(x);
    if (Math.hypot(...r) < 1e-5) break;
    const J = [[], [], []], h = [0.05, 0.05, 0.05];
    for (let k = 0; k < 3; k++) { const xp = x.slice(); xp[k] += h[k]; const rp = f(xp); for (let i = 0; i < 3; i++) J[i][k] = (rp[i] - r[i]) / h[k]; }
    const dx = solve3(J, r.map(v => -v));
    let lam = 1;
    for (let k = 0; k < 3; k++) if (!isFinite(dx[k])) dx[k] = 0;
    const step = Math.max(Math.abs(dx[0]), Math.abs(dx[1]), Math.abs(dx[2]));
    if (step > 15) lam = 15 / step;
    x = x.map((v, k) => v + lam * dx[k]);
    x[1] = Math.max(-10, Math.min(90, x[1]));
  }
  return { phi: x[0], F: x[1], side: x[2], res: Math.hypot(...f(x)) };
}
function solve3(M, b) {
  const a = M.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c; for (let r = c + 1; r < 3; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    if (Math.abs(a[c][c]) < 1e-12) return [0, 0, 0];
    for (let r = 0; r < 3; r++) if (r !== c) { const k = a[r][c] / a[c][c]; for (let j = c; j < 4; j++) a[r][j] -= k * a[c][j]; }
  }
  return [a[0][3] / a[0][0], a[1][3] / a[1][1], a[2][3] / a[2][2]];
}

// trunk after the palms leave the mat: straight interpolation of the solved values to the fold
let OFF = null; // solution at T.handsOff
let fLate = null;
function lateTrunk(t) {
  if (!fLate) {
    const sp0 = SPW.map(w => OFF.F * w / SPWS);
    const mid = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);
    fLate = {
      pel: pchip([[T.handsOff, OFF.P], [T.grasp, [0, 0.86, foldRootP.z - 0.002]], [T.end, [foldRootP.x, foldRootP.y, foldRootP.z]]]),
      pitch: pchip([[T.handsOff, OFF.phi], [T.grasp, 108], [T.end, FOLD_PITCH]]),
      side: pchip([[T.handsOff, OFF.side], [T.grasp, 0], [T.end, 0]]),
      spine: pchip([[T.handsOff, sp0], [T.grasp, mid(sp0, FOLD_SPINE, 0.7)], [T.end, FOLD_SPINE]]),
      neck: pchip([[T.handsOff, fNeck0(T.handsOff)], [T.end, [0, 0, 0]]]),
      scap: pchip([[T.handsOff, fScap0(T.handsOff)], [T.grasp, [10, 1, 8, 2, -0.004]], [T.end, [8, 0, 6, 0, -0.006]]]),
    };
  }
  return fLate;
}

// ---------- legs ----------
function heelFor(sd, hip, lo, hi) {
  // heel rise (deg about the MTP axis) that leaves the knee nearly straight; the heel never sinks below the high plank
  const tz = TOES[sd];
  const reach = LT + LS - 0.006;
  const d = a => ankleFromMTP(tz.mtp, qAx(X, a).multiply(tz.footQ), sd).distanceTo(hip) - reach;
  if (d(lo) <= 0) return lo;
  return bisect(d, lo, hi);
}
let heelAtLift = { L: null, R: null };
const LIFTLOC = { L: null, R: null }; // foot and toes locals at take-off
const PF_MAX = 38; // plantarflexion limit of the swinging foot, deg from the standing angle
function clampPF(q, sd) {
  const ql = sd === 'L' ? q.clone() : mirrorQ(q);
  const e = new THREE.Euler().setFromQuaternion(ql, 'XYZ');
  if (e.x / D < -PF_MAX) e.x = -PF_MAX * D;
  const r = new THREE.Quaternion().setFromEuler(e);
  return sd === 'L' ? r : mirrorQ(r);
}
function legCtl(sd, t, Wp) {
  const lift = sd === 'R' ? T.liftR : T.liftL, land = sd === 'R' ? T.landR : T.landL;
  const tz = TOES[sd], fl = FLAT[sd];
  const hip = Wp['thigh_' + sd].p;
  if (t <= lift) {
    const a = heelFor(sd, hip, 0, 45);
    heelAtLift[sd] = a;
    const Qf = qAx(X, a).multiply(tz.footQ);
    return { ankle: ankleFromMTP(tz.mtp, Qf, sd), pole: hip.clone().add(V(sd === 'L' ? 0.05 : -0.05, -0.6, 0.25)), footQ: Qf, toesQ: tz.toesQ.clone(), mode: 'toes', heel: a };
  }
  if (t >= land) return { ankle: fl.ankle.clone(), pole: fl.ankle.clone().add(V(sd === 'L' ? 0.04 : -0.04, 0.45, 0.6)), footQ: fl.footQ.clone(), toesQ: fl.toesQ.clone(), mode: 'flat' };
  const u = smoother(seg01(t, lift, land));
  const Qf0 = qAx(X, heelAtLift[sd] ?? 0).multiply(tz.footQ);
  const A0 = ankleFromMTP(tz.mtp, Qf0, sd), A3 = fl.ankle;
  const A1 = A0.clone().add(V(0, 0.16, 0.22)), A2 = A3.clone().add(V(0, 0.18, -0.12));
  const b = (p0, p1, p2, p3, s) => p0.clone().multiplyScalar((1 - s) ** 3).addScaledVector(p1, 3 * (1 - s) ** 2 * s).addScaledVector(p2, 3 * (1 - s) * s * s).addScaledVector(p3, s ** 3);
  const ankle = b(A0, A1, A2, A3, u);
  const footQ = slerpQ(Qf0, fl.footQ, smooth(seg01(u, 0.15, 0.9)));
  const toesLocal = slerpQ(Qf0.clone().invert().multiply(tz.toesQ), fl.toesLocal, smooth(seg01(u, 0.0, 0.6)));
  const toesQ = footQ.clone().multiply(toesLocal);
  const pole = hip.clone().add(V(sd === 'L' ? 0.05 : -0.05, lerp(-0.6, 0.45, smooth(seg01(u, 0.2, 0.9))), 0.6));
  // the swinging foot: ankle angle relative to the shank goes from the take-off angle to the landing angle, relaxed in between
  let footLocal = null, toesLocalS = null;
  if (LIFTLOC[sd]) {
    const s = smooth(seg01(u, 0.1, 0.85));
    footLocal = clampPF(slerpQ(LIFTLOC[sd].foot, fold.L['foot_' + sd], s), sd);
    toesLocalS = slerpQ(LIFTLOC[sd].toes, fold.L['toes_' + sd], smooth(seg01(u, 0.0, 0.6)));
  }
  // ankle kept within a relaxed range while the foot is in the air (hinge shank–foot ≤ 98°, standing ≈ 66°), opening to the lunge's angle on landing
  return { ankle, pole, footQ, toesQ, mode: 'swing', u };
}

// ---------- arms ----------
// elbow direction of an anchor: the elbow's offset from the shoulder–wrist line (keeps the roll of the arm equal to the anchor's)
const elbowPerp = (W, sd) => { const s = W['hum_' + sd].p, w = W['hand_' + sd].p, e = W['fore_' + sd].p; const a = w.clone().sub(s).normalize(); const d = e.clone().sub(s); d.addScaledVector(a, -d.dot(a)); return d.normalize(); };
const PERP_HP = { L: elbowPerp(Whp, 'L'), R: elbowPerp(Whp, 'R') };
const PERP_F = {}; // fold elbow direction relative to the shank
for (const sd of ['L', 'R']) PERP_F[sd] = elbowPerp(Wf, sd).applyQuaternion(Wf['shank_' + sd].q.clone().invert());
const TW = {}; // forearm twist and wrist of the hand at the start and the end of its travel
function armCtl(sd, t, Wp) {
  const h = HAND[sd], sgn = sd === 'L' ? 1 : -1;
  const sh = Wp['hum_' + sd].p;
  const poleFloor = sh.clone().lerp(h.wrist, 0.5).addScaledVector(PERP_HP[sd], 0.4);
  if (t <= T.handsOff) return { wrist: h.wrist.clone(), pole: poleFloor, handQ: h.q.clone(), fingers: SPREAD };
  const shank = { p: Wp['shank_' + sd].p, q: Wp['shank_' + sd].q };
  const gr = GRIPREL[sd];
  const gripW = gr.wrist.clone().applyQuaternion(shank.q).add(shank.p), gripQ = shank.q.clone().multiply(gr.q);
  const u = smooth(seg01(t, T.handsOff, T.grasp));
  const P0 = h.wrist.clone(), P3 = gripW;
  const P1 = P0.clone().add(V(sgn * 0.05, 0.12, -0.06)), P2 = P3.clone().add(V(sgn * 0.13, 0.06, 0.06));
  const wrist = P0.clone().multiplyScalar((1 - u) ** 3).addScaledVector(P1, 3 * (1 - u) ** 2 * u).addScaledVector(P2, 3 * (1 - u) * u * u).addScaledVector(P3, u ** 3);
  const handQ = slerpQ(h.q, gripQ, smooth(seg01(u, 0.1, 0.95)));
  const elbowGrip = gr.elbow.clone().applyQuaternion(shank.q).add(shank.p);
  const poleGrip = elbowGrip.clone().addScaledVector(PERP_F[sd].clone().applyQuaternion(shank.q), 0.4);
  const pole = poleFloor.lerp(poleGrip, smooth(u));
  const local = TW[sd] && t < T.grasp - 1e-9 ? { twist: slerpQ(TW[sd].t0, TW[sd].t1, smooth(seg01(u, 0.05, 0.98))), swing: slerpQ(TW[sd].s0, TW[sd].s1, smooth(seg01(u, 0.05, 0.98))) } : null;
  return { wrist, pole, handQ, local, fingers: blendFingers(SPREAD, GRIP, smooth(seg01(u, 0.3, 1))) };
}

// ---------- one frame ----------
let guess = { phi: 2 * Math.acos(hp.rootQ.w) / D, F: 6, side: 0 };
let prevL = null;
const pickLegs = L => { const o = {}; for (const k of ['thigh_L', 'shank_L', 'foot_L', 'toes_L', 'thigh_R', 'shank_R', 'foot_R', 'toes_R']) o[k] = L[k]; return o; };
function frameAt(t) {
  let Ltr, rootQ, rootP, diag = {};
  if (t <= T.handsOff + 1e-9) {
    const P = V(...fPel(t)), Sh = fSh(t), roll = fRoll(t), nk = fNeck0(t), sc = fScap0(t);
    const sol = solveTrunk(P, Sh, roll, nk, sc, guess);
    guess = sol;
    Ltr = trunkL(sol, nk, sc); rootQ = rootQof(sol.phi, roll); rootP = P;
    diag = { phi: sol.phi, F: sol.F, side: sol.side, res: sol.res };
    if (Math.abs(t - T.handsOff) < 1e-9) OFF = { P: [P.x, P.y, P.z], phi: sol.phi, F: sol.F, side: sol.side };
  } else {
    const f = lateTrunk(t);
    const P = V(...f.pel(t)), sp = f.spine(t), nk = f.neck(t), sc = f.scap(t);
    Ltr = trunkL({ spine: sp, side: f.side(t) }, nk, sc); rootQ = qAx(X, f.pitch(t)); rootP = P;
    diag = { phi: f.pitch(t), F: sp.reduce((a, b) => a + b, 0) };
  }
  const Wt = computeWorld(prevL ? { ...Ltr, ...pickLegs(prevL) } : Ltr, rootQ, rootP);
  const spine = {}; for (const k of ['pelvis', 'lumA', 'lumB', 'thA', 'thB', 'thC', 'neckA', 'neckB', 'head']) spine[k] = Ltr[k];
  const c = { rootP, rootQ, spine, clav: { L: Ltr.clav_L, R: Ltr.clav_R }, scap: { L: Ltr.scap_L, R: Ltr.scap_R }, arm: {}, leg: {} };
  for (const sd of ['L', 'R']) c.leg[sd] = legCtl(sd, t, Wt);
  for (const sd of ['L', 'R']) c.arm[sd] = armCtl(sd, t, Wt);
  let r = solveBody(c);
  for (const sd of ['L', 'R']) if (c.leg[sd].mode === 'toes') LIFTLOC[sd] = { foot: r.L['foot_' + sd].clone(), toes: r.L['toes_' + sd].clone() };
  if (t > T.handsOff) { const W2 = computeWorld(r.L, rootQ, rootP); for (const sd of ['L', 'R']) c.arm[sd] = armCtl(sd, t, W2); r = solveBody(c); }
  prevL = r.L;
  return { ...r, diag: { ...diag, shortL: r.info.armL.short, shortR: r.info.armR.short, elL: r.info.armL.theta, elR: r.info.armR.theta, legErrL: r.info.legL.err, legErrR: r.info.legR.err, heelL: c.leg.L.heel, heelR: c.leg.R.heel } };
}

// ---------- run ----------
// pre-pass: march to the moment the palms leave the mat, so the late trunk starts from the solved one
{ const g0 = { ...guess }; for (let t = 0.05; t < T.handsOff - 1e-9; t += 0.05) frameAt(t); frameAt(T.handsOff); guess = g0; prevL = null; }
{
  const dec = (r, sd) => { const fl = r.info['arm' + sd].flexQ; const fore = sd === 'L' ? r.L['fore_' + sd] : mirrorQ(r.L['fore_' + sd]); const hand = sd === 'L' ? r.L['hand_' + sd] : mirrorQ(r.L['hand_' + sd]); return { tw: fl.clone().invert().multiply(fore), sw: hand.clone() }; };
  const g0 = { ...guess };
  for (let t = 0.05; t < T.handsOff - 1e-9; t += 0.05) frameAt(t);
  const r0 = frameAt(T.handsOff);
  for (let t = T.handsOff + 0.05; t < T.grasp - 1e-9; t += 0.05) frameAt(t);
  const r1 = frameAt(T.grasp);
  for (const sd of ['L', 'R']) { const a = dec(r0, sd), b = dec(r1, sd); if (a.tw.dot(b.tw) < 0) b.tw.set(-b.tw.x, -b.tw.y, -b.tw.z, -b.tw.w); if (a.sw.dot(b.sw) < 0) b.sw.set(-b.sw.x, -b.sw.y, -b.sw.z, -b.sw.w); TW[sd] = { t0: a.tw, t1: b.tw, s0: a.sw, s1: b.sw }; }
  guess = g0; prevL = null;
}
const frames = [];
const n = Math.round(DUR * FPS);
for (let i = 0; i <= n; i++) {
  const t = +(i / FPS).toFixed(2);
  let d, dg = null;
  if (i === 0) d = { ...toData(hp.L, hp.rootQ, hp.rootP) };
  else if (i === n) d = { local: foldData.local, root: foldData.root, rootP: foldData.rootP };
  else { const r = frameAt(t); d = toData(r.L, r.rootQ, r.rootP); dg = r.diag; }
  const grip = t >= T.grasp - 1e-9 ? ['L', 'R'] : [];
  const note = t < T.liftR ? 'ладони и пальцы стоп на полу; вес вперёд, таз вверх' : t < T.landR ? 'правая нога в воздухе: опора на ладони и левые пальцы' : t < T.liftL ? 'выпад: ладони, правая стопа, левые пальцы' : t < T.landL ? 'левая нога в воздухе: опора на ладони и правую стопу' : t < T.handsOff ? 'стопы вместе, колени согнуты, ладони на полу' : t < T.grasp ? 'ладони идут к голеням' : 'держу голени, выпрямляю ноги';
  frames.push({ t, ...d, grip, note });
  if (DIAG && dg) {
    const cf = fromData(d); const m = measureConf(cf);
    const r1 = v => (v == null ? '-' : (+v).toFixed(1));
    console.log(String(t).padStart(4), 'phi', r1(dg.phi), 'F', r1(dg.F), 'side', r1(dg.side), 'res', dg.res != null ? (dg.res * 1000).toFixed(1) : '-', '| el', r1(dg.elL), r1(dg.elR), 'short', ((dg.shortL || 0) * 1000).toFixed(0), ((dg.shortR || 0) * 1000).toFixed(0), '| legErr', ((dg.legErrL || 0) * 1000).toFixed(0), ((dg.legErrR || 0) * 1000).toFixed(0), 'heel', r1(dg.heelL), r1(dg.heelR), '| minY', (m.minY * 1000).toFixed(1), m.minL, '| com', m.com.x.toFixed(3), m.com.z.toFixed(3), 'marg', (m.margin * 100).toFixed(1), '|', m.regs.join(' '));
    if (process.argv.includes('--joints')) console.log('      ', JSON.stringify(m.joints));
  }
}
const out = { id: 'seg5', from: 'highplank', to: 'fold', fps: FPS, duration: DUR, phases: PHASES, frames };
const segFile = DRY ? SCR + 'seg5_try.json' : 'cycle_keys/seg5.json';
fs.writeFileSync('cycle_keys/fold.json', JSON.stringify(foldData));
fs.writeFileSync(segFile, JSON.stringify(out));
console.log('wrote', segFile, frames.length, 'frames; cycle_keys/fold.json rootP', foldData.rootP);
