// seg3 of the round: Полупрогиб → Корзинка («перехватился в корзинку»), writes cycle_keys/seg3.json.
// Phases: knees bend while the W of the arms opens; the arms sweep back past the hips and the hands take the shins
// from the inside; the shins kick back into the hands and lift thighs and chest — the body rocks onto the belly.
import fs from 'fs';
import * as K from './cycle_seg3_lib.mjs';
const { A, V, THREE, S, SEG, computeWorld, pt, LANDMARKS, comOf, qAx, X, Y, Z, D, IDQ, scapulaPose, mirrorQ, LH, LF,
  armIK, orientHand, setSide, blank, setLR, fingers, SOFT, GRIP, OPEN, blendHand, gripHand, wristFor, LP, wpt, settle } = K;

const FPS = 10, DUR = 4.9, N = Math.round(DUR * FPS), T_GRIP = 3.3, T_KNEE = 1.5;
const an = JSON.parse(fs.readFileSync('cycle_keys/anchors.json', 'utf8'));
const qa = a => new THREE.Quaternion(...a);
const confOf = fr => { const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = qa(v); return { L, rootQ: qa(fr.root), rootP: V(...fr.rootP) }; };
const HALF = confOf(an.half), BOW = confOf(an.bow);
const W_HALF = computeWorld(HALF.L, HALF.rootQ, HALF.rootP), W_BOW = computeWorld(BOW.L, BOW.rootQ, BOW.rootP);
const angX = q => 2 * Math.atan2(q.x, q.w) / D;

// ---------- 1-D monotone cubic through keys (zero slope at both ends and at local extrema) ----------
function pchip(keys) {
  const n = keys.length, t = keys.map(k => k[0]), v = keys.map(k => k[1]);
  const h = [], dlt = [];
  for (let i = 0; i < n - 1; i++) { h.push(t[i + 1] - t[i]); dlt.push((v[i + 1] - v[i]) / h[i]); }
  const m = new Array(n).fill(0);
  for (let i = 1; i < n - 1; i++) {
    if (dlt[i - 1] * dlt[i] <= 0) m[i] = 0;
    else { const w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1]; m[i] = (w1 + w2) / (w1 / dlt[i - 1] + w2 / dlt[i]); }
  }
  return x => {
    if (x <= t[0]) return v[0];
    if (x >= t[n - 1]) return v[n - 1];
    let i = 0; while (x > t[i + 1]) i++;
    const s = (x - t[i]) / h[i], s2 = s * s, s3 = s2 * s;
    return (2 * s3 - 3 * s2 + 1) * v[i] + (s3 - 2 * s2 + s) * h[i] * m[i] + (-2 * s3 + 3 * s2) * v[i + 1] + (s3 - s2) * h[i] * m[i + 1];
  };
}
const vecKeys = (times, vals) => { const fs_ = vals[0].map((_, k) => pchip(times.map((t, i) => [t, vals[i][k]]))); return x => fs_.map(f => f(x)); };
const mix = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);

// ---------- body parameters over time ----------
const HALFSC = [20, 8, -20, 8, -0.012], BOWSC = [-6, -8, -14, -4, -0.01]; // scapUp, scapPost, scapProt, clavRot, scapElev
const KNEE_BOW = angX(BOW.L.shank_L);
const TK = [0, T_KNEE, T_GRIP, DUR];
const P = {
  spine: vecKeys(TK, [[-2, -3, -2, -12, -12], [-3, -4, -3, -8, -8], [-7, -7, -5, -7, -6.5], [-10, -10, -7, -9, -8]]),
  neck: vecKeys([0, T_GRIP, DUR], [[0, 0, 0], [-1, -1, 0], [-3, -2, 0]]),
  knee: pchip([[0, 0], [T_KNEE, 118], [T_GRIP, 124], [DUR, KNEE_BOW]]),
  hipExt: pchip([[0, -4], [T_KNEE, -3.5], [T_GRIP, 3], [DUR, 22]]),
  hipAdd: pchip([[0, 0], [T_GRIP, 0.5], [DUR, 2]]),
  foot: pchip([[0, 60], [T_KNEE, 14], [T_GRIP, -10], [DUR, -15]]),
  toes: pchip([[0, 28], [T_KNEE, 10], [T_GRIP, -2], [DUR, -8]]),
  scap: vecKeys(TK, [HALFSC, mix(HALFSC, BOWSC, 0.45), mix(HALFSC, BOWSC, 0.85), BOWSC]),
  open: pchip([[0, 0], [1.9, 1], [DUR, 1]]),      // SOFT → OPEN
  close: pchip([[0, 0], [2.15, 0], [T_GRIP + 0.15, 1], [DUR, 1]]), // OPEN → GRIP
};
function bodyAt(t) {
  const L = blank();
  const sp = P.spine(t); ['lumA', 'lumB', 'thA', 'thB', 'thC'].forEach((k, i) => { L[k] = qAx(X, sp[i]); });
  const nk = P.neck(t); L.neckA = qAx(X, nk[0]); L.neckB = qAx(X, nk[1]); L.head = qAx(X, nk[2]);
  const left = {};
  left.thigh = qAx(X, P.hipExt(t)).multiply(qAx(Z, -P.hipAdd(t)));
  left.shank = qAx(X, P.knee(t)); left.foot = qAx(X, P.foot(t)); left.toes = qAx(X, P.toes(t));
  const sc = P.scap(t); const s = scapulaPose(sc[0], sc[1], sc[2], null, sc[3], sc[4]); left.clav = s.clav; left.scap = s.scap;
  left.hum = IDQ.clone(); left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  const hand = blendHand(blendHand(SOFT, OPEN, P.open(t)), GRIP, P.close(t));
  fingers(left, hand);
  setLR(L, left);
  fitGripReach(L, t);
  return L;
}

// ---------- arms: body-relative targets, solved for the left arm and mirrored ----------
const gripPt = LP('shank', 0.07, 0.16, -0.012);
const toLocal = (W, seg, p) => p.clone().sub(W[seg].p).applyQuaternion(W[seg].q.clone().invert());
const toWorld = (W, seg, p) => p.clone().applyQuaternion(W[seg].q).add(W[seg].p);
const qToLocal = (W, seg, q) => W[seg].q.clone().invert().multiply(q);
const qToWorld = (W, seg, q) => W[seg].q.clone().multiply(q);
const backOf = W => V(0, 0, -1).applyQuaternion(W.pelvis.q);
// roll of the arm round the shoulder–wrist line: the direction of the elbow crease (humerus front axis) as an angle
const N_HUM = S.fore_L.J.clone().sub(S.hum_L.J).normalize();
function basisQ(y, z) { const x = y.clone().cross(z).normalize(); const zz = x.clone().cross(y).normalize(); return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, zz)); }
const R0 = basisQ(N_HUM.clone().negate(), V(0, 0, 1));
const ZZ0 = V(0, 0, 1).applyQuaternion(R0);
function rollFrame(W, d) { const ref = backOf(W); const lat = ref.clone().addScaledVector(d, -ref.dot(d)).normalize(); return { lat, oth: d.clone().cross(lat).normalize() }; }
function rollOf(W) {
  const d = W.hand_L.p.clone().sub(W.hum_L.p).normalize();
  const cr = ZZ0.clone().applyQuaternion(W.hum_L.q); cr.addScaledVector(d, -cr.dot(d)).normalize();
  const { lat, oth } = rollFrame(W, d);
  return Math.atan2(cr.dot(oth), cr.dot(lat));
}
const creaseAt = (W, d, alpha) => { const { lat, oth } = rollFrame(W, d); return lat.multiplyScalar(Math.cos(alpha)).addScaledVector(oth, Math.sin(alpha)); };
function gripTarget(W) {
  const Cg = wpt(W, gripPt), Sh = W.hum_L.p;
  const n = V(1, 0, 0).applyQuaternion(W.shank_L.q);
  let wrist = Cg.clone(), Qh = null;
  for (let i = 0; i < 6; i++) { Qh = gripHand(wrist.clone().sub(Sh).normalize(), n); wrist = wristFor(Cg, Qh); }
  return { wrist, Qh, n };
}
// Keep the reaching arm close to its final extended shape while the hands take
// the inside of the shins. A deeply folded knee brings the grip too close to
// the shoulder and makes the final almost-straight elbow plane turn inwards.
const FINAL_GRIP_MARGIN = LH + LF - W_BOW.hum_L.p.distanceTo(gripTarget(W_BOW).wrist);
function fitGripReach(L, t) {
  if (t <= 2.2 || t >= DUR) return;
  const gripProgress = Math.max(0, Math.min(1, (t - T_GRIP) / (DUR - T_GRIP)));
  const margin = 0.004 + (FINAL_GRIP_MARGIN - 0.004) * ease3(gripProgress);
  const target = LH + LF - margin;
  const original = angX(L.shank_L);
  let lo = 90, hi = 145;
  for (let it = 0; it < 35; it++) {
    const angle = (lo + hi) / 2;
    L.shank_L = qAx(X, angle); L.shank_R = mirrorQ(L.shank_L);
    const W = computeWorld(L, HALF.rootQ, V(0, 0, 0));
    const distance = W.hum_L.p.distanceTo(gripTarget(W).wrist);
    if (distance > target) lo = angle; else hi = angle;
  }
  const u = Math.max(0, Math.min(1, (t - 2.2) / (T_GRIP - 2.2)));
  L.shank_L = qAx(X, original + ((lo + hi) / 2 - original) * ease3(u));
  L.shank_R = mirrorQ(L.shank_L);
}
// fixed references taken from the two anchors
const ALPHA_HALF = rollOf(W_HALF), ALPHA_BOW = rollOf(W_BOW);
const START = { wrist: toLocal(W_HALF, 'pelvis', W_HALF.hand_L.p), q: qToLocal(W_HALF, 'pelvis', W_HALF.hand_L.q) };
// the arm sweeps back past the hip: wrist beside the hip, a hand's breadth off the floor, palm towards the thigh
const HIP_PT = V(0.27, 0.07, -0.02);
// on the way the hands open out to the sides (wings), a hand's breadth off the floor
const ARC_PT = V(0.62, 0.085, 0.36); // from the pelvis joint (0.10 above the floor): out to the side, 12 cm off the floor, by the hip
const ROLL_UNWRAP = (() => { let a = ALPHA_BOW - ALPHA_HALF; while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; })();

// the elbow points (away from the crease): from the W (elbows back), up and a little out while the arms sweep back
// past the hips, then the way the straight arm turns at the grip — interpolated in the pelvis frame
const ELBOW_T = V(0, -0.25, -0.97).normalize(), ELBOW_SWEEP = V(0.3, 0, -0.95).normalize(); // pelvis frame (rest axes): x out, y to the head, -z dorsal
const T_ARC = 0.9, T_HIP = 1.75;
const ELBOWP = pchip([[0, 0], [T_ARC, 1], [T_HIP, 2], [T_GRIP - 0.3, 2.96], [T_GRIP, 3]]);
const TWISTP = pchip([[0, 0], [0.15, 0.01], [T_ARC, 0.1], [T_HIP, 0.45], [T_GRIP - 0.3, 0.97], [T_GRIP, 1]]);
const slerpV = (a, b, u) => { const q = new THREE.Quaternion().setFromUnitVectors(a, b); return a.clone().applyQuaternion(new THREE.Quaternion().slerp(q, u)); };
const W_HALF0 = computeWorld(HALF.L, HALF.rootQ, V(0, 0, 0));
const ELBOW_START = (() => { const W = W_HALF0; const e = W.hand_L.p.clone().sub(W.hum_L.p).normalize(); const el = W.fore_L.p.clone().sub(W.hum_L.p); el.addScaledVector(e, -el.dot(e)).normalize(); return el.applyQuaternion(W.pelvis.q.clone().invert()); })();
const TWIST0 = HALF.L.ulna_L.clone().invert(), SWING0 = HALF.L.hand_L.clone();
let GRIP_SOL = null; // twist, swing and elbow direction of the arm at the moment of the grip
function armCore(L, rootQ, wrist, crease) {
  const W = computeWorld(L, rootQ, V(0, 0, 0));
  const Sh = W.hum_L.p;
  const mid = Sh.clone().add(wrist).multiplyScalar(0.5);
  return armIK(L, rootQ, V(0, 0, 0), 'L', wrist, mid.clone().addScaledVector(crease, -1.0), crease);
}
function mirrorArms(L) { for (const s of ['hum', 'fore', 'ulna', 'hand']) L[s + '_R'] = mirrorQ(L[s + '_L']); }
function solveArm(L, rootQ, t) {
  const W = computeWorld(L, rootQ, V(0, 0, 0));
  const g = gripTarget(W);
  const Sh = W.hum_L.p;
  if (t >= T_GRIP - 1e-9) {
    const e = g.wrist.clone().sub(Sh).normalize();
    const crease = creaseAt(W, e, ALPHA_BOW);
    const flexQ = armCore(L, rootQ, g.wrist, crease);
    orientHand(L, rootQ, V(0, 0, 0), 'L', g.Qh, flexQ);
    mirrorArms(L);
    const W2 = computeWorld(L, rootQ, V(0, 0, 0));
    return { reachErr: W2.hand_L.p.distanceTo(g.wrist), margin: LH + LF - Sh.distanceTo(g.wrist), crease };
  }
  if (!GRIP_SOL) throw new Error('grip solution first');
  const C0 = toWorld(W, 'pelvis', START.wrist);
  const CT = W.pelvis.p.clone().add(ARC_PT);
  const C1 = W.pelvis.p.clone().add(HIP_PT);
  const up = backOf(W);
  // Approach from outside the leg rather than curling the wrist towards the
  // shoulder. The latter folded both elbows through the body's midplane.
  const C2 = g.wrist.clone().add(V(0.03, 0, 0)).addScaledVector(up, 0.02);
  const C3 = g.wrist.clone();
  // Shape-preserving component curves prevent the Catmull-Rom control points
  // from overshooting inside the final grip before the hand turns onto it.
  const wrist = V(...vecKeys([0, 1, 2, 3, 4], [C0, CT, C1, C2, C3].map(v => v.toArray()))(PATH(t)));
  // elbow direction
  const se = ELBOWP(t);
  const Eg = GRIP_SOL.elbowDir;
  const Ep = se <= 1 ? slerpV(ELBOW_START, ELBOW_T, se) : (se <= 2 ? slerpV(ELBOW_T, ELBOW_SWEEP, se - 1) : slerpV(ELBOW_SWEEP, Eg, se - 2));
  const Ew = Ep.clone().applyQuaternion(W.pelvis.q);
  const e = wrist.clone().sub(Sh).normalize();
  const crease = Ew.clone().addScaledVector(e, -Ew.dot(e)).normalize().negate();
  const flexQ = armCore(L, rootQ, wrist, crease);
  // forearm turn and wrist: from the W to the grip, in the joints themselves
  const u = TWISTP(t);
  const tw = new THREE.Quaternion().slerpQuaternions(TWIST0, GRIP_SOL.twist, u);
  const sw = new THREE.Quaternion().slerpQuaternions(SWING0, GRIP_SOL.swing, u);
  L.fore_L = flexQ.clone().multiply(tw); L.ulna_L = tw.clone().invert(); L.hand_L = sw;
  mirrorArms(L);
  const W2 = computeWorld(L, rootQ, V(0, 0, 0));
  return { reachErr: W2.hand_L.p.distanceTo(wrist), margin: LH + LF - Sh.distanceTo(g.wrist), crease };
}
function initGrip() {
  const L = bodyAt(T_GRIP);
  const rootQ = HALF.rootQ.clone();
  const r = solveArm(L, rootQ, T_GRIP);
  const W = computeWorld(L, rootQ, V(0, 0, 0));
  const e = W.hand_L.p.clone().sub(W.hum_L.p).normalize();
  const el = W.fore_L.p.clone().sub(W.hum_L.p); el.addScaledVector(e, -el.dot(e));
  const elbowDir = (el.length() > 1e-3 ? el.normalize() : r.crease.clone().negate()).applyQuaternion(W.pelvis.q.clone().invert());
  GRIP_SOL = { twist: L.ulna_L.clone().invert(), swing: L.hand_L.clone(), elbowDir };
}
const ease3 = u => u * u * (3 - 2 * u);
// Path parameter: start, open wing, beside hip, just outside shin, on shin.
const PATH = pchip([[0, 0], [0.14, 0.03], [T_ARC, 1], [T_HIP, 2], [T_GRIP - 0.45, 3], [T_GRIP, 4]]);
const HANDP = pchip([[0, 0], [0.3, 0.04], [1.4, 0.5], [2.25, 0.97], [T_GRIP, 1]]);
const ROLLP = pchip([[0, 0], [0.3, 0.05], [1.4, 0.55], [2.3, 0.95], [T_GRIP, 1]]);
// ---------- rest on the floor ----------
const PELVIS_LM = LANDMARKS.find(l => l.region === 'таз' && l.side === 'L' && !l.tag);
const Z_PELVIS = pt(W_HALF, PELVIS_LM.seg, PELVIS_LM.pc).z;
function placeOnFloor(L, phi) {
  const rootQ = qAx(X, phi);
  const W0 = computeWorld(L, rootQ, V(0, 0, 0));
  let minY = Infinity; for (const l of LANDMARKS) minY = Math.min(minY, pt(W0, l.seg, l.pc).y);
  const zp = pt(W0, PELVIS_LM.seg, PELVIS_LM.pc).z;
  return { rootQ, rootP: V(0, -minY, Z_PELVIS - zp) };
}

// ---------- frames ----------
initGrip();
const raw = [];
let phi = angX(HALF.rootQ);
for (let i = 0; i <= N; i++) {
  const t = i / FPS;
  const L = bodyAt(t);
  // arms in the current body pitch (targets are body-relative, so the pitch only matters through the roll reference)
  let info = solveArm(L, qAx(X, phi), t);
  const phiS = settle(L, phi);
  info = solveArm(L, qAx(X, phiS), t);
  phi = settle(L, phiS);
  raw.push({ t, L, phi, info });
}
// a little temporal smoothing of the pitch (rocking without jumps), then the anchors exactly at both ends
const phis = raw.map(r => r.phi);
const sm = phis.map((p, i) => { if (i === 0 || i === N) return p; let s = 0, w = 0; for (let k = -2; k <= 2; k++) { const j = Math.max(0, Math.min(N, i + k)); const wk = 3 - Math.abs(k); s += phis[j] * wk; w += wk; } return s / w; });
const frames = [];
const blendEnd = (i) => { const a = Math.max(0, 1 - i / 3), b = Math.max(0, 1 - (N - i) / 3); return { a, b }; };
raw.forEach((r, i) => {
  let L = r.L;
  let { rootQ, rootP } = placeOnFloor(L, sm[i]);
  const { a, b } = blendEnd(i);
  if (a > 0 || b > 0) {
    const ref = a > 0 ? HALF : BOW, w = a > 0 ? a : b;
    const L2 = {}; for (const s of SEG) L2[s.name] = new THREE.Quaternion().slerpQuaternions(L[s.name], ref.L[s.name], w);
    L = L2;
    const pl = placeOnFloor(L, sm[i] + (angX(ref.rootQ) - sm[i]) * w);
    rootQ = pl.rootQ; rootP = pl.rootP;
    if (w === 1) { rootQ = ref.rootQ.clone(); rootP = ref.rootP.clone(); }
  }
  const r6 = v => +v.toFixed(6);
  frames.push({
    t: +r.t.toFixed(2),
    local: Object.fromEntries(SEG.map(s => [s.name, L[s.name].toArray().map(r6)])),
    root: rootQ.toArray().map(r6), rootP: rootP.toArray().map(r6),
    grip: r.t >= T_GRIP - 1e-9 ? ['L', 'R'] : [],
    ...(i === Math.round(T_GRIP * FPS) ? { note: 'кисти легли на голени изнутри' } : {}),
  });
  r.out = { phi: sm[i], margin: r.info.margin, reachErr: r.info.reachErr };
});
// exact anchors at both ends
frames[0] = { ...frames[0], local: an.half.local, root: an.half.root, rootP: an.half.rootP, grip: [] };
frames[N] = { ...frames[N], local: an.bow.local, root: an.bow.root, rootP: an.bow.rootP, grip: ['L', 'R'] };
const seg = {
  id: 'seg3', from: 'half', to: 'bow', fps: FPS, duration: DUR,
  phases: [{ t0: 0, t1: T_KNEE, label: 'Сгибаю колени' }, { t0: T_KNEE, t1: T_GRIP, label: 'Беру голени изнутри' }, { t0: T_GRIP, t1: DUR, label: 'Поднимаюсь в корзинку' }],
  frames,
};
fs.writeFileSync('cycle_keys/seg3.json', JSON.stringify(seg));
if (process.argv.includes('-v')) for (const r of raw) console.log(r.t.toFixed(1), 'phi', r.out.phi.toFixed(2), 'raw', r.phi.toFixed(2), 'gripMargin', (r.out.margin * 100).toFixed(1), 'cm reachErr', (r.out.reachErr * 1000).toFixed(1), 'mm');
console.log('seg3.json', frames.length, 'frames; alpha half', (ALPHA_HALF / D).toFixed(1), 'bow', (ALPHA_BOW / D).toFixed(1), 'unwrap', (ROLL_UNWRAP / D).toFixed(1));
