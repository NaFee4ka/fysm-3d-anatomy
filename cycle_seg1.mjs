// seg1 of the round: Горка → Планка на предплечьях («опустил локти и таз»). Writes cycle_keys/seg1.json.
// 1. «Сгибаю локти и опускаю предплечья»: the hips stay high, the elbows bend and come down to the mat; the palms
//    stay planted, then roll onto the little-finger edge and close into fists, so the wrists travel ~5 cm forward
//    without the hands ever leaving the floor (dolphin).
// 2. «Отшагиваю назад и соединяю стопы»: the pelvis starts down along an arc round the feet; one foot, then the other,
//    steps back onto the tucked toes, big toes together (the plank needs ~14 cm more length than the dog).
// 3. «Опускаю таз в планку»: the pelvis keeps lowering on the same arc while the shoulders travel forward over the
//    elbows; the legs stay almost straight, the feet roll from the balls onto the tips of the toes.
import fs from 'fs';
import * as C from './cycle_lib.mjs';
const { A, V, THREE, computeWorld, pt, LANDMARKS, comOf, qAx, X, Z, IDQ, cloneL, armIK, orientHand, legIK, orientFoot, LH, LT, LS } = C;

const an = JSON.parse(fs.readFileSync('cycle_keys/anchors.json', 'utf8'));
const q4 = a => new THREE.Quaternion(...a);
const confOf = k => { const L = {}; for (const [s, v] of Object.entries(an[k].local)) L[s] = q4(v); return { L, rootQ: q4(an[k].root), rootP: V(...an[k].rootP) }; };
const DOG = confOf('dog'), PL = confOf('plank');
const WD = computeWorld(DOG.L, DOG.rootQ, DOG.rootP), WP = computeWorld(PL.L, PL.rootQ, PL.rootP);

const P = {
  T1: 1.6, T: 4.2,              // forearms down by T1, plank at T
  thetaA: 45,                   // dolphin: upper arm tilted this far behind the vertical through the elbow
  yA: 0.64,                     // dolphin: pelvis height (dog 0.669)
  palmHold: 0.15, roll: [0.15, 1.6], fingers: [0.4, 3.0], handLift: [1.35, 1.6],
  tilt: 0, fingerClear: 0.0007, settle: [0.85, 1.5],  // no-slip roll: the hand pivots on its contact, then settles into place once the forearm carries the load
  spine: [1.3, 3.9], girdle: [1.5, 3.9],
  pelvis2: [1.6, 4.2],          // pelvis down along the arc
  stepR: [2.0, 2.7], stepL: [2.7, 3.4], lift: 0.03,
  blend: 0.3,                   // first/last frames eased onto the anchors
};
const FPS = 10, N = Math.round(P.T * FPS);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const smooth = (a, b, t) => { const u = clamp((t - a) / (b - a), 0, 1); return u * u * u * (u * (u * 6 - 15) + 10); };
const lerp = (a, b, u) => a + (b - a) * u;
const DEG = Math.PI / 180;
const SPINE = ['lumA', 'lumB', 'thA', 'thB', 'thC', 'neckA', 'neckB', 'head'];
const GIRDLE = ['clav_L', 'scap_L', 'clav_R', 'scap_R'];
const FING = A.SEG.map(s => s.name).filter(n => /^f\d[a-cm]_[LR]$/.test(n));
const HAND_RG = new Set(['ладони', 'кулаки', 'кисти', 'предплечья', 'локти']);
const handLm = sd => LANDMARKS.filter(l => l.side === sd && HAND_RG.has(l.region));
const fingerLm = sd => LANDMARKS.filter(l => l.side === sd && l.region === 'пальцы рук');
const palmLm = sd => LANDMARKS.filter(l => l.side === sd && /^(ладони|пальцы рук|кулаки|кисти)$/.test(l.region));
const kneeFlexAt = t => (t <= P.T1 ? lerp(10, 5, smooth(0, P.T1, t)) : lerp(5, 0, smooth(P.T1, P.T, t)));
const legLen = k => Math.sqrt(LT * LT + LS * LS + 2 * LT * LS * Math.cos(k * DEG));

// ---------- trunk ----------
function trunkLocals(L, t) {
  const a = smooth(...P.spine, t), b = smooth(...P.girdle, t);
  for (const s of SPINE) L[s] = DOG.L[s].clone().slerp(PL.L[s], a);
  for (const s of GIRDLE) L[s] = DOG.L[s].clone().slerp(PL.L[s], b);
  const f = smooth(...P.fingers, t);
  for (const s of FING) L[s] = DOG.L[s].clone().slerp(PL.L[s], f);
}
const E = { L: WP.fore_L.p.clone(), R: WP.fore_R.p.clone() };
const shoulderLocal = L => computeWorld(L, new THREE.Quaternion(), V(0, 0, 0)).hum_L.p.clone();
const lyzOf = sx => Math.sqrt(LH * LH - (sx - E.L.x) ** 2);
const onElbowCircle = (th, sx) => V(sx, E.L.y + lyzOf(sx) * Math.cos(th * DEG), E.L.z - lyzOf(sx) * Math.sin(th * DEG));
// pitch (about X) that turns the pelvis-frame vector v into the world vector w (both in the y–z plane)
const pitchOf = (v, w) => (Math.atan2(w.z, w.y) - Math.atan2(v.z, v.y)) / DEG;
const wrapNear = (a, ref) => { while (a - ref > 180) a -= 360; while (a - ref < -180) a += 360; return a; };

// phase 1 end: shoulders on the circle round the landing elbows, pelvis at yA
const L_T1 = cloneL(DOG.L); trunkLocals(L_T1, P.T1);
const sT1 = shoulderLocal(L_T1);
const SA = onElbowCircle(P.thetaA, sT1.x);
const SD = WD.hum_L.p.clone();
function phase1Root(L, t, prevPhi) {
  const s = shoulderLocal(L);
  const u = smooth(0, P.T1, t);
  const St = SD.clone().lerp(SA, u);
  const yP = lerp(WD.pelvis.p.y, P.yA, u);
  const h = St.y - yP, R = Math.hypot(s.y, s.z), del = Math.atan2(s.z, s.y);
  const c = clamp(h / R, -1, 1);
  const phi = [Math.acos(c) - del, -Math.acos(c) - del].map(v => wrapNear(v / DEG, prevPhi)).sort((a, b) => Math.abs(a - prevPhi) - Math.abs(b - prevPhi))[0];
  const rootQ = qAx(X, phi), S0 = s.clone().applyQuaternion(rootQ);
  return { phi, rootQ, rootP: V(0, St.y - S0.y, St.z - S0.z) };
}
// phase 2: pelvis on an arc round the plank's ankles, shoulders on the circle round the planted elbows
let ARC = null;
function phase2Root(L, t, prevPhi) {
  const s = shoulderLocal(L);
  const u = smooth(...P.pelvis2, t);
  const r = lerp(ARC.rA, ARC.rP, u), psi = lerp(ARC.psiA, ARC.psiP, u);
  const pel = V(0, ARC.cy + r * Math.sin(psi), ARC.cz + r * Math.cos(psi));
  const Rt = Math.hypot(s.y, s.z);
  const f = th => { const S_ = onElbowCircle(th, s.x); return Math.hypot(S_.y - pel.y, S_.z - pel.z) - Rt; };
  // root of f nearest to the previous angle
  let best = null;
  for (let th = -30; th < 80; th += 0.5) {
    const a = f(th), b = f(th + 0.5);
    if ((a > 0) !== (b > 0)) {
      let lo = th, hi = th + 0.5, flo = a;
      for (let k = 0; k < 50; k++) { const m = (lo + hi) / 2, fm = f(m); if ((fm > 0) === (flo > 0)) { lo = m; flo = fm; } else hi = m; }
      const root = (lo + hi) / 2;
      if (best === null || Math.abs(root - ARC.prevTh) < Math.abs(best - ARC.prevTh)) best = root;
    }
  }
  if (best === null) throw new Error('trunk does not close at t=' + t);
  ARC.prevTh = best;
  const S_ = onElbowCircle(best, s.x);
  const w = V(0, S_.y - pel.y, S_.z - pel.z);
  const phi = wrapNear(pitchOf(s, w), prevPhi);
  const rootQ = qAx(X, phi), S0 = s.clone().applyQuaternion(rootQ);
  return { phi, rootQ, rootP: V(0, S_.y - S0.y, S_.z - S0.z), theta: best };
}

// ---------- feet ----------
const rotAbout = (axis, deg) => new THREE.Quaternion().setFromAxisAngle(axis, deg * DEG);
const FOOT = {};
for (const sd of ['L', 'R']) {
  FOOT[sd] = {
    old: { mtp: WD['toes_' + sd].p.clone(), axis: V(1, 0, 0).applyQuaternion(WD['toes_' + sd].q), ankle: WD['foot_' + sd].p.clone(), footQ: WD['foot_' + sd].q.clone(), toesQ: WD['toes_' + sd].q.clone() },
    neu: { mtp: WP['toes_' + sd].p.clone(), axis: V(1, 0, 0).applyQuaternion(WP['toes_' + sd].q), ankle: WP['foot_' + sd].p.clone(), footQ: WP['foot_' + sd].q.clone(), toesQ: WP['toes_' + sd].q.clone() },
    prev: { old: 0, neu: 0 },
  };
}
// a planted foot turns about its toe joint (toes stay flat on the mat) until the leg has the wanted knee bend
function plantedFoot(st, H, k, prevA) {
  const want = legLen(k);
  const ankleAt = a => st.mtp.clone().add(st.ankle.clone().sub(st.mtp).applyQuaternion(rotAbout(st.axis, a)));
  const f = a => H.distanceTo(ankleAt(a)) - want;
  let a = prevA;
  let lo = null;
  for (let d = 0; d <= 70; d += 0.5) {
    for (const s of [1, -1]) {
      const x0 = prevA + s * d, x1 = x0 + s * 0.5;
      if ((f(x0) > 0) !== (f(x1) > 0)) { lo = [Math.min(x0, x1), Math.max(x0, x1)]; break; }
    }
    if (lo) break;
  }
  let reach = true;
  if (lo) { let [x0, x1] = lo, f0 = f(x0); for (let i = 0; i < 50; i++) { const m = (x0 + x1) / 2, fm = f(m); if ((fm > 0) === (f0 > 0)) { x0 = m; f0 = fm; } else x1 = m; } a = (x0 + x1) / 2; }
  else { reach = false; let bestA = prevA, bf = Infinity; for (let x = -60; x <= 70; x += 0.5) if (Math.abs(f(x)) < bf) { bf = Math.abs(f(x)); bestA = x; } a = bestA; }
  return { a, ankle: ankleAt(a), footQ: rotAbout(st.axis, a).multiply(st.footQ), toesQ: st.toesQ.clone(), reach };
}
const stepWin = sd => (sd === 'R' ? P.stepR : P.stepL);

// ---------- one frame ----------
const QH = { D: { L: WD.hand_L.q.clone(), R: WD.hand_R.q.clone() }, P: { L: WP.hand_L.q.clone(), R: WP.hand_R.q.clone() } };
const WR = { D: { L: WD.hand_L.p.clone(), R: WD.hand_R.p.clone() }, P: { L: WP.hand_L.p.clone(), R: WP.hand_R.p.clone() } };
const ELB_D = { L: WD.fore_L.p.clone(), R: WD.fore_R.p.clone() };
const TILT_SIGN = (() => {
  const tip = (sg) => { const L = cloneL(DOG.L); const fl = L.fore_L.clone(); orientHand(L, DOG.rootQ, DOG.rootP, 'L', QH.D.L.clone().multiply(qAx(Z, sg * 10)), armIK(L, DOG.rootQ, DOG.rootP, 'L', WR.D.L, WD.fore_L.p, null)); const W = computeWorld(L, DOG.rootQ, DOG.rootP); return pt(W, 'f3c_L', V(0, 0, 0)).y; };
  return tip(1) > tip(-1) ? 1 : -1;
})();
const swingCache = {};

function hipPos(L, rootQ, rootP, sd) { return computeWorld(L, rootQ, rootP)['thigh_' + sd].p.clone(); }

function legFor(L, rootQ, rootP, sd, t, diag) {
  const Fd = FOOT[sd], H = hipPos(L, rootQ, rootP, sd), k = kneeFlexAt(t);
  const [s0, s1] = stepWin(sd);
  let ankle, footQ, toesQ, state;
  if (t <= s0 + 1e-9) {
    const r = plantedFoot(Fd.old, H, k, Fd.prev.old); Fd.prev.old = r.a;
    ({ ankle, footQ, toesQ } = r); state = 'old ' + r.a.toFixed(1) + (r.reach ? '' : ' NO-REACH');
  } else if (t >= s1 - 1e-9) {
    const r = plantedFoot(Fd.neu, H, k, Fd.prev.neu); Fd.prev.neu = r.a;
    ({ ankle, footQ, toesQ } = r); state = 'new ' + r.a.toFixed(1) + (r.reach ? '' : ' NO-REACH');
  } else {
    // swing: from the lift-off state to the landing state, toes cleared off the mat
    const c = swingCache[sd];
    const u = smooth(s0, s1, t);
    footQ = c.q0.clone().slerp(c.q1, u);
    toesQ = c.t0.clone().slerp(c.t1, u).multiply(qAx(X, 12 * Math.sin(Math.PI * u)));
    const mtp = c.m0.clone().lerp(c.m1, u); mtp.y += P.lift * Math.sin(Math.PI * u);
    ankle = mtp.clone().add(c.v.clone().applyQuaternion(footQ));
    state = 'swing ' + u.toFixed(2);
  }
  // knee faces the way the toes point
  const fwd = V(0, 0, 1).applyQuaternion(footQ), leg = ankle.clone().sub(H).normalize();
  fwd.addScaledVector(leg, -fwd.dot(leg)).normalize();
  const pole = H.clone().lerp(ankle, 0.5).addScaledVector(fwd, 0.4);
  legIK(L, rootQ, rootP, sd, ankle, pole);
  orientFoot(L, rootQ, rootP, sd, footQ);
  const W = computeWorld(L, rootQ, rootP);
  L['toes_' + sd] = W['foot_' + sd].q.clone().invert().multiply(toesQ);
  if (diag) diag['foot_' + sd] = state;
  return ankle;
}
function prepareSwing(sd, rootAt) {
  const [s0, s1] = stepWin(sd), Fd = FOOT[sd];
  const a = rootAt(s0), b = rootAt(s1);
  const r0 = plantedFoot(Fd.old, a.H[sd], kneeFlexAt(s0), Fd.prev.old);
  const r1 = plantedFoot(Fd.neu, b.H[sd], kneeFlexAt(s1), 0);
  Fd.prev.neu = r1.a;
  swingCache[sd] = { q0: r0.footQ, q1: r1.footQ, t0: r0.toesQ, t1: r1.toesQ, m0: Fd.old.mtp.clone(), m1: Fd.neu.mtp.clone(), v: Fd.old.ankle.clone().sub(Fd.old.mtp).applyQuaternion(Fd.old.footQ.clone().invert()) };
}

function armFor(L, rootQ, rootP, sd, t) {
  const u = smooth(...P.roll, t);
  const handQ = QH.D[sd].clone().slerp(QH.P[sd], u);
  // the fingers come off the mat as the hand rolls: the load stays on the heel and the outer edge of the palm
  const tw = Math.sin(Math.PI * smooth(P.roll[0], P.roll[1] + 0.4, t));
  if (tw > 0) handQ.multiply(qAx(Z, TILT_SIGN * P.tilt * tw));
  const wrist = WR.D[sd].clone().lerp(WR.P[sd], u);
  const ks = 1 - smooth(...P.settle, t);
  wrist.x += ROLL.off[sd].x * ks; wrist.z += ROLL.off[sd].z * ks;
  wrist.y += 0.02 * Math.sin(Math.PI * smooth(P.settle[0] - 0.1, P.settle[1] + 0.15, t)); // the hand is carried, not dragged, back into line
  const s1 = smooth(0, P.T1, t);
  const pole = ELB_D[sd].clone().add(V(0, -0.1, 0)).lerp(E[sd], s1);
  const solve = w => { const fl = armIK(L, rootQ, rootP, sd, w, pole, null); orientHand(L, rootQ, rootP, sd, handQ, fl); };
  solve(wrist);
  // until the forearm is down, the lowest point of hand and forearm stays on the mat (the hand rolls onto its edge,
  // the forearm settles from the wrist end to the elbow); the fingers never go under the mat
  if (t > 0 && t < P.roll[1]) {
    for (let it = 0; it < 3; it++) {
      const W = computeWorld(L, rootQ, rootP);
      let mn = Infinity, mf = Infinity;
      for (const l of handLm(sd)) mn = Math.min(mn, pt(W, l.seg, l.pc).y);
      for (const l of fingerLm(sd)) mf = Math.min(mf, pt(W, l.seg, l.pc).y);
      const w = smooth(...P.handLift, t);
      const want = lerp(0.0012, mn, w);
      const shift = Math.max(want - mn, lerp(0.0007, P.fingerClear, smooth(P.roll[0], P.roll[0] + 0.3, t)) - mf);
      if (Math.abs(shift) < 2e-5) break;
      wrist.y += shift; solve(wrist);
    }
  }
  // no slip: what touched the mat in the previous frame and still touches it does not slide
  if (ROLL.live && t > 0 && t < P.settle[0]) {
    for (let it = 0; it < 3; it++) {
      const W = computeWorld(L, rootQ, rootP);
      const d = V(0, 0, 0); let n = 0;
      for (const l of palmLm(sd)) {
        const k = l.seg + '|' + l.pc.x + '|' + l.pc.y + '|' + l.pc.z, p = pt(W, l.seg, l.pc), q = ROLL.prev[sd][k];
        if (p.y < 0.004 && q) { d.x += p.x - q.x; d.z += p.z - q.z; n++; }
      }
      if (!n) break;
      d.multiplyScalar(1 / n);
      if (Math.hypot(d.x, d.z) < 2e-4) break;
      wrist.x -= d.x; wrist.z -= d.z; ROLL.off[sd].x -= d.x; ROLL.off[sd].z -= d.z;
      solve(wrist);
    }
  }
  if (ROLL.live) {
    const W = computeWorld(L, rootQ, rootP), cur = {};
    for (const l of palmLm(sd)) { const p = pt(W, l.seg, l.pc); if (p.y < 0.004) cur[l.seg + '|' + l.pc.x + '|' + l.pc.y + '|' + l.pc.z] = p; }
    ROLL.prev[sd] = cur;
    if (process.env.RDBG && sd === 'L') console.log('t', t, 'off', ROLL.off.L.x.toFixed(3), ROLL.off.L.z.toFixed(3), Object.keys(cur).length);
  }
}
const ROLL = { live: false, off: { L: V(0, 0, 0), R: V(0, 0, 0) }, prev: { L: {}, R: {} } };

function buildFrame(t, prevPhi, diag) {
  const L = cloneL(DOG.L);
  trunkLocals(L, t);
  const r = t <= P.T1 ? phase1Root(L, t, prevPhi) : phase2Root(L, t, prevPhi);
  for (const sd of ['L', 'R']) armFor(L, r.rootQ, r.rootP, sd, t);
  for (const sd of ['L', 'R']) legFor(L, r.rootQ, r.rootP, sd, t, diag);
  if (diag) { diag.phi = r.phi; diag.theta = r.theta; }
  return { L, rootQ: r.rootQ, rootP: r.rootP, phi: r.phi };
}

// ---------- run ----------
// arc for phase 2 from the pelvis at T1 to the plank's pelvis, centred on the plank's ankles
{
  let phi = 2 * Math.atan2(DOG.rootQ.x, DOG.rootQ.w) / DEG;
  for (let i = 0; i <= Math.round(P.T1 * FPS); i++) { const L = cloneL(DOG.L); trunkLocals(L, i / FPS); phi = phase1Root(L, i / FPS, phi).phi; }
  const L = cloneL(DOG.L); trunkLocals(L, P.T1);
  const r = phase1Root(L, P.T1, phi);
  const cy = WP.foot_L.p.y, cz = WP.foot_L.p.z;
  const pa = r.rootP, pp = WP.pelvis.p;
  ARC = { cy, cz, rA: Math.hypot(pa.y - cy, pa.z - cz), psiA: Math.atan2(pa.y - cy, pa.z - cz), rP: Math.hypot(pp.y - cy, pp.z - cz), psiP: Math.atan2(pp.y - cy, pp.z - cz), prevTh: P.thetaA, phiT1: r.phi };
}
// root at any time (for the swing set-up)
function rootAt(t) {
  let phi = 2 * Math.atan2(DOG.rootQ.x, DOG.rootQ.w) / DEG;
  const saveTh = ARC.prevTh; ARC.prevTh = P.thetaA;
  let r = null;
  for (let i = 0; i <= Math.round(t * FPS); i++) { const tt = i / FPS; const L = cloneL(DOG.L); trunkLocals(L, tt); r = tt <= P.T1 ? phase1Root(L, tt, phi) : phase2Root(L, tt, phi); phi = r.phi; }
  const L = cloneL(DOG.L); trunkLocals(L, t);
  ARC.prevTh = saveTh;
  return { ...r, H: { L: hipPos(L, r.rootQ, r.rootP, 'L'), R: hipPos(L, r.rootQ, r.rootP, 'R') } };
}
for (const sd of ['L', 'R']) prepareSwing(sd, rootAt);
for (const sd of ['L', 'R']) { FOOT[sd].prev.old = 0; FOOT[sd].prev.neu = swingCache[sd] ? FOOT[sd].prev.neu : 0; }
ARC.prevTh = P.thetaA;

ROLL.live = true;
const frames = [];
const diags = [];
let phi = 2 * Math.atan2(DOG.rootQ.x, DOG.rootQ.w) / DEG;
for (let i = 0; i <= N; i++) {
  const t = +(i / FPS).toFixed(2);
  const diag = { t };
  const f = buildFrame(t, phi, diag);
  phi = f.phi;
  // ease the first and last frames onto the anchors (removes the last millimetres of the reaches)
  const wa = 1 - smooth(0, P.blend, t), wb = smooth(P.T - P.blend, P.T, t);
  for (const s of A.SEG) {
    if (wa > 0) f.L[s.name].slerp(DOG.L[s.name], wa);
    if (wb > 0) f.L[s.name].slerp(PL.L[s.name], wb);
  }
  diags.push(diag);
  frames.push(f);
}
// exact anchors at both ends
frames[0] = { L: cloneL(DOG.L), rootQ: DOG.rootQ.clone(), rootP: DOG.rootP.clone() };
frames[N] = { L: cloneL(PL.L), rootQ: PL.rootQ.clone(), rootP: PL.rootP.clone() };

const r6 = v => +v.toFixed(6);
const phases = [
  { t0: 0, t1: P.T1, label: 'Сгибаю локти и опускаю предплечья' },
  { t0: P.T1, t1: P.stepL[1], label: 'Отшагиваю назад, стопы вместе' },
  { t0: P.stepL[1], t1: P.T, label: 'Опускаю таз в планку' },
];
const noteAt = t => {
  if (t <= P.palmHold) return 'ладони на месте, локти сгибаются';
  if (t < P.T1) return 'ладони перекатываются на ребро и собираются в кулаки, запястья уходят вперёд на 5 см, предплечья ложатся на коврик';
  if (t > P.stepR[0] && t < P.stepR[1]) return 'правая стопа отшагивает назад и внутрь на подогнутые пальцы';
  if (t > P.stepL[0] && t < P.stepL[1]) return 'левая стопа отшагивает назад, большие пальцы вместе';
  return t < P.T ? 'таз опускается по дуге вокруг стоп, плечи выходят над локтями' : 'планка';
};
const out = {
  id: 'seg1', from: 'dog', to: 'plank', fps: FPS, duration: P.T, phases,
  frames: frames.map((f, i) => ({
    t: +(i / FPS).toFixed(2),
    local: Object.fromEntries(A.SEG.map(s => [s.name, f.L[s.name].toArray().map(r6)])),
    root: f.rootQ.toArray().map(r6), rootP: [f.rootP.x, f.rootP.y, f.rootP.z].map(r6),
    grip: [], note: noteAt(i / FPS),
  })),
};
fs.writeFileSync('cycle_keys/seg1.json', JSON.stringify(out));
if (process.argv.includes('--diag')) for (const d of diags) console.log(d.t.toFixed(1), 'phi', d.phi.toFixed(1), 'theta', d.theta === undefined ? '-' : d.theta.toFixed(1), '| L', d.foot_L, '| R', d.foot_R);
console.log('seg1.json', frames.length, 'frames,', P.T, 's');
