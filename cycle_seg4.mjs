// Segment 4 of the round: Корзинка → Планка на ладонях («опустил руки, ноги … через планку»).
//   A «Отпускаю руки и ноги»: the hands open and let go of the shins, the legs, chest and arms come down to the mat
//   B «Ставлю ладони под плечи»: palms go flat beside the lower ribs where the high plank has them, toes tuck under
//   C «Выжимаюсь в планку»: the body rises as one straight piece, pivoting over the toes, into the high plank
// usage: node cycle_seg4.mjs [--prov]   → cycle_keys/seg4.json   (--prov: provisional high plank if seg5's is missing)
import fs from 'fs';
import * as C from './cycle_lib.mjs';
const { A, V, THREE, S, SEG, qAx, X, Y, Z, computeWorld, pt, LANDMARKS, comOf, D, IDQ, mirrorQ, scapulaPose, cloneL, armIK, orientHand, legIK, LH, LF, LT, LS, ease } = C;

// ---------- data ----------
const q4 = a => new THREE.Quaternion(...a);
const fromData = d => { const L = {}; for (const [s, v] of Object.entries(d.local)) L[s] = q4(v); return { L, rootQ: q4(d.root), rootP: V(...d.rootP) }; };
const r6 = v => +v.toFixed(6);
const toFrame = (c, t, extra = {}) => ({ t: +t.toFixed(2), local: Object.fromEntries(SEG.map(s => [s.name, c.L[s.name].toArray().map(r6)])), root: c.rootQ.toArray().map(r6), rootP: [c.rootP.x, c.rootP.y, c.rootP.z].map(r6), grip: [], ...extra });
const anchors = JSON.parse(fs.readFileSync('cycle_keys/anchors.json', 'utf8'));
const BOW = fromData(anchors.bow);
const HPF = 'cycle_keys/highplank.json';
export const HP_DATA = fs.existsSync(HPF) ? JSON.parse(fs.readFileSync(HPF, 'utf8')) : null;

// ---------- small helpers ----------
const W_of = c => computeWorld(c.L, c.rootQ, c.rootP);
const setLR = (L, left) => { for (const k in left) { L[k + '_L'] = left[k].clone(); L[k + '_R'] = mirrorQ(left[k]); } };
const leftOf = (L, names) => Object.fromEntries(names.map(n => [n, L[n + '_L'].clone()]));
const slerpL = (La, Lb, u) => { const o = {}; for (const s of SEG) o[s.name] = new THREE.Quaternion().slerpQuaternions(La[s.name], Lb[s.name], u); return o; };
const lmk = (region, side, tag) => LANDMARKS.find(l => l.region === region && l.side === side && (tag === undefined ? !l.tag : l.tag === tag));
const lmY = (W, pred) => { let m = Infinity; for (const l of LANDMARKS) if (pred(l)) m = Math.min(m, pt(W, l.seg, l.pc).y); return m; };
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const smooth = (a, b, t) => { const u = clamp((t - a) / (b - a), 0, 1); return u * u * (3 - 2 * u); };
const ARM = ['clav', 'scap', 'hum', 'fore', 'ulna', 'hand', 'f1m', 'f1a', 'f1b', 'f2a', 'f2b', 'f2c', 'f3a', 'f3b', 'f3c', 'f4a', 'f4b', 'f4c', 'f5a', 'f5b', 'f5c'];
const FING = ARM.slice(6);
const LEG = ['thigh', 'shank', 'foot', 'toes'];

// fingers (same parametrisation as poses_build)
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
export const SPREAD = { spread: [0, 14, 0, 10, 20], flex: [[7.5, 0, 0], [5.7, 0, 0], [4.8, 0, 0], [2.8, 0, 0]], thumb: { abd: 42 } };
export const SOFT = { spread: [0, 4, 0, 3, 6], flex: [[15, 22, 12]], thumb: { abd: 18, flex: 5, mcp: 5, ip: 5 } };
export const RELAX = { spread: [0, 3, 0, 2, 4], flex: [[22, 30, 16]], thumb: { abd: 14, flex: 10, mcp: 10, ip: 10 } };
const fingerSet = o => { const left = {}; fingers(left, o); return left; };

// hand orientation helpers (as in poses_build)
function handBasis(fingersDir, palmNormal) {
  const g = fingersDir.clone().normalize();
  const n = palmNormal.clone(); n.addScaledVector(g, -n.dot(g)); n.normalize();
  const Xw = n.clone().negate(), Yw = g.clone().negate(), Zw = Xw.clone().cross(Yw).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(Xw, Yw, Zw));
}
const PALM_OFF = V(0.196 - 0.212, 0.75 - 0.815, 0.008 + 0.022);
const wristFor = (Cp, Qh) => Cp.clone().sub(PALM_OFF.clone().applyQuaternion(Qh));

// resting on the floor: lower convex hull in the sagittal plane (as poses_build.settle), warm-started
function hull2(P) {
  const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Q) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
export function settle(L, phi0, filter) {
  const W0 = computeWorld(L, qAx(X, phi0), V(0, 0, 0));
  const pts = LANDMARKS.filter(l => !filter || filter(l)).map(l => pt(W0, l.seg, l.pc));
  const com = comOf(W0);
  const H = hull2(pts.map(p => [p.z, p.y]));
  let best = null;
  for (let i = 0; i < H.length; i++) {
    const a = H[i], b = H[(i + 1) % H.length];
    const dz = b[0] - a[0], dy = b[1] - a[1];
    if (Math.hypot(dz, dy) < 1e-4) continue;
    const base = Math.atan2(dy, dz);
    for (const d of [base, base + Math.PI]) {
      const c = Math.cos(d), s = Math.sin(d);
      const rot = qq => [qq[1] * s + qq[0] * c, qq[1] * c - qq[0] * s];
      const ra = rot(a), rb = rot(b);
      let ok = true;
      for (const qq of H) if (rot(qq)[1] < ra[1] - 1e-7) { ok = false; break; }
      if (!ok) continue;
      const rc = rot([com.z, com.y]);
      if (rc[0] < Math.min(ra[0], rb[0]) - 1e-6 || rc[0] > Math.max(ra[0], rb[0]) + 1e-6) continue;
      const dd = Math.atan2(Math.sin(d), Math.cos(d));
      if (best === null || Math.abs(dd) < Math.abs(best)) best = dd;
    }
  }
  return phi0 + (best || 0) / D;
}
const rootPitch = q => { const v = V(0, 1, 0).applyQuaternion(q); return Math.atan2(v.z, v.y) / D; };
// put the body on the floor (lowest landmark at y = 0) and keep a chosen landmark at its z
function place(L, rootQ, pin, pinZ) {
  const W0 = computeWorld(L, rootQ, V(0, 0, 0));
  let mn = Infinity; for (const l of LANDMARKS) mn = Math.min(mn, pt(W0, l.seg, l.pc).y);
  const pz = pt(W0, pin.seg, pin.pc).z;
  return V(0, -mn, pinZ - pz);
}

export { fromData, toFrame, W_of, setLR, leftOf, slerpL, lmk, lmY, smooth, ARM, FING, LEG, fingers, fingerSet, handBasis, wristFor, rootPitch, place, BOW };

// ================= keyframes =================
const PIN = lmk('таз', 'L');                     // pelvis contact: stays where it is in the bow while the body comes down
const PIN_Z = pt(W_of(BOW), PIN.seg, PIN.pc).z;
const bowLeft = leftOf(BOW.L, [...ARM, ...LEG]);

// prone on the mat: spine almost neutral, head slightly lifted, legs straight with the tops of the feet down
export function proneTrunk(L, o = {}) {
  const p = Object.assign({ lumA: -1, lumB: -1, thA: 0, thB: -1, thC: -1, neck: [-7, -6, -4] }, o);
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) L[k] = qAx(X, p[k]);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
}
export function proneLegs(left, o = {}) {
  const p = Object.assign({ hip: -3, add: 0, knee: 0, plantar: 60, toes: 28 }, o);
  left.thigh = qAx(X, p.hip).multiply(qAx(Z, -p.add)); left.shank = qAx(X, p.knee); left.foot = qAx(X, p.plantar); left.toes = qAx(X, p.toes);
}
// arm from the thorax: flexion (towards the floor when lying prone), abduction, forearm pronation, elbow flexion
export function armFromThorax(left, o) {
  const p = Object.assign({ flex: 0, abd: 0, er: 0, elbow: 0, pron: 0, wristExt: 0, scap: [0, 0, 0, 0, 0] }, o);
  const sp = scapulaPose(p.scap[0], p.scap[1], p.scap[2], null, p.scap[3], p.scap[4]);
  left.clav = sp.clav; left.scap = sp.scap;
  const RTH = qAx(X, -p.flex).multiply(qAx(Z, p.abd)).multiply(qAx(Y, p.er));
  left.hum = left.clav.clone().multiply(left.scap).invert().multiply(RTH);
  const dAx = S.hand_L.J.clone().sub(S.fore_L.J).normalize();
  left.fore = qAx(X, -p.elbow).multiply(new THREE.Quaternion().setFromAxisAngle(dAx, p.pron * D));
  left.ulna = new THREE.Quaternion().setFromAxisAngle(dAx, -p.pron * D);
  left.hand = qAx(Z, p.wristExt);
}
const ARM_REG = new Set(['ладони', 'пальцы рук', 'кулаки', 'кисти', 'предплечья', 'локти', 'плечи']);
export const noArms = l => !ARM_REG.has(l.region);
export const isArmL = l => ARM_REG.has(l.region) && l.side === 'L';
export function bisect(f, a, b, n = 50) {
  let fa = f(a), fb = f(b);
  if (fa === 0) return a;
  if (fb === 0) return b;
  if ((fa > 0) === (fb > 0)) return Math.abs(fa) < Math.abs(fb) ? a : b;
  for (let i = 0; i < n; i++) { const m = (a + b) / 2, fm = f(m); if ((fm > 0) === (fa > 0)) { a = m; fa = fm; } else { b = m; fb = fm; } }
  return (a + b) / 2;
}

// ---------- keyframe K3: prone on the mat, arms along the body, tops of the feet down ----------
export function buildProne(o = {}) {
  const p = Object.assign({ neck: [-4, -3, -2], abd: 9, pron: 20 }, o);
  const L = C.blankLocal();
  proneTrunk(L, { neck: p.neck });
  const lf = {}; proneLegs(lf); armFromThorax(lf, { flex: 0, abd: p.abd, pron: p.pron }); fingers(lf, RELAX); setLR(L, lf);
  const phi = settle(L, 90, noArms);
  const rootQ = qAx(X, phi);
  const flex = bisect(fl => { armFromThorax(lf, { flex: fl, abd: p.abd, pron: p.pron }); setLR(L, lf); const rp = place(L, rootQ, PIN, PIN_Z); return lmY(computeWorld(L, rootQ, rp), isArmL) - 0.001; }, -30, 40);
  armFromThorax(lf, { flex, abd: p.abd, pron: p.pron }); setLR(L, lf);
  return { L, rootQ, rootP: place(L, rootQ, PIN, PIN_Z), info: { phi, armFlex: flex } };
}

// arm (left side) aimed at a wrist point with the elbow towards a pole, hand turned to a world orientation
function aimLeft(c, wristT, pole, handQ, crease) {
  const flexQ = armIK(c.L, c.rootQ, c.rootP, 'L', wristT, pole, crease);
  if (handQ) orientHand(c.L, c.rootQ, c.rootP, 'L', handQ, flexQ);
  for (const n of ARM) c.L[n + '_R'] = mirrorQ(c.L[n + '_L']);
}
export { aimLeft, PIN, PIN_Z };

// ---------- smooth multi-key tracks: Catmull–Rom through quaternion keys (Barry–Goldman with slerp), one ease over the span ----------
const sl = (a, b, u) => new THREE.Quaternion().slerpQuaternions(a, b, u);
export function trackAt(keys, t, names) {
  const n = keys.length;
  const nm = names || Object.keys(keys[0].L);
  const out = {};
  if (n === 1 || t <= keys[0].t) { for (const k of nm) out[k] = keys[0].L[k].clone(); return out; }
  if (t >= keys[n - 1].t) { for (const k of nm) out[k] = keys[n - 1].L[k].clone(); return out; }
  const T0 = keys[0].t, T1 = keys[n - 1].t;
  const tau = T0 + (T1 - T0) * ease((t - T0) / (T1 - T0));
  let i = 0; while (i < n - 2 && tau > keys[i + 1].t) i++;
  const k1 = keys[i], k2 = keys[i + 1];
  const k0 = keys[i - 1] || { t: k1.t - (k2.t - k1.t), L: k1.L };
  const k3 = keys[i + 2] || { t: k2.t + (k2.t - k1.t), L: k2.L };
  const [t0, t1, t2, t3] = [k0.t, k1.t, k2.t, k3.t];
  for (const k of nm) {
    const q1 = k1.L[k].clone(), q0 = k0.L[k].clone(), q2 = k2.L[k].clone(), q3 = k3.L[k].clone();
    if (q0.dot(q1) < 0) q0.set(-q0.x, -q0.y, -q0.z, -q0.w);
    if (q2.dot(q1) < 0) q2.set(-q2.x, -q2.y, -q2.z, -q2.w);
    if (q3.dot(q2) < 0) q3.set(-q3.x, -q3.y, -q3.z, -q3.w);
    const A1 = sl(q0, q1, (tau - t0) / (t1 - t0)), A2 = sl(q1, q2, (tau - t1) / (t2 - t1)), A3 = sl(q2, q3, (tau - t2) / (t3 - t2));
    const B1 = sl(A1, A2, (tau - t0) / (t2 - t0)), B2 = sl(A2, A3, (tau - t1) / (t3 - t1));
    out[k] = sl(B1, B2, (tau - t1) / (t2 - t1)).normalize();
  }
  return out;
}
const sidesOf = (Lleft) => { const o = {}; for (const k in Lleft) { o[k + '_L'] = Lleft[k].clone(); o[k + '_R'] = mirrorQ(Lleft[k]); } return o; };
export { sidesOf };

// ================= phase A: «Отпускаю руки и ноги» =================
export const TA = { gripEnd: 0.2, fingers: [0, 0.7], arms: [0.2, 1.95], trunk: [0.3, 1.8], legs: [0.3, 1.95], end: 2.0, corr: [0.2, 0.75] };
const ARM_NOFING = ARM.filter(n => !FING.includes(n));
let PIN_TARGET = null; // set by main: pelvis contact z the push-up needs
function placeBody(L, phi0, t) { const ph = settle(L, phi0, noArms); const rootQ = qAx(X, ph); const z = PIN_TARGET == null ? PIN_Z : PIN_Z + (PIN_TARGET - PIN_Z) * smooth(0.2, 2.0, t); return { L, rootQ, rootP: place(L, rootQ, PIN, z), phi: ph }; }
export const setPinTarget = z => { PIN_TARGET = z; };
// scalar Catmull–Rom through keyed parameter sets, one ease over the span (same timing as trackAt)
export function paramAt(keys, t) {
  const n = keys.length;
  if (t <= keys[0].t) return JSON.parse(JSON.stringify(keys[0].p));
  if (t >= keys[n - 1].t) return JSON.parse(JSON.stringify(keys[n - 1].p));
  const T0 = keys[0].t, T1 = keys[n - 1].t;
  const tau = T0 + (T1 - T0) * ease((t - T0) / (T1 - T0));
  let i = 0; while (i < n - 2 && tau > keys[i + 1].t) i++;
  const k1 = keys[i], k2 = keys[i + 1];
  const k0 = keys[i - 1] || { t: k1.t - (k2.t - k1.t), p: k1.p }, k3 = keys[i + 2] || { t: k2.t + (k2.t - k1.t), p: k2.p };
  const [t0, t1, t2, t3] = [k0.t, k1.t, k2.t, k3.t];
  const cr = (p0, p1, p2, p3) => {
    const A1 = p0 + (p1 - p0) * (tau - t0) / (t1 - t0), A2 = p1 + (p2 - p1) * (tau - t1) / (t2 - t1), A3 = p2 + (p3 - p2) * (tau - t2) / (t3 - t2);
    const B1 = A1 + (A2 - A1) * (tau - t0) / (t2 - t0), B2 = A2 + (A3 - A2) * (tau - t1) / (t3 - t1);
    return B1 + (B2 - B1) * (tau - t1) / (t2 - t1);
  };
  const out = {};
  for (const k in k1.p) out[k] = Array.isArray(k1.p[k]) ? k1.p[k].map((_, j) => cr(k0.p[k][j], k1.p[k][j], k2.p[k][j], k3.p[k][j])) : cr(k0.p[k], k1.p[k], k2.p[k], k3.p[k]);
  return out;
}
export const ARM_A = [
  { t: 0.2, p: { flex: -62, abd: -13, er: 90, elbow: 4, pron: -90, wristExt: -7, scap: [-6, -8, -14, -4, -0.01] } },
  { t: 0.6, p: { flex: -47, abd: -12, er: 60, elbow: 32, pron: -60, wristExt: -2, scap: [-5, -6, -10, -3, -0.008] } },
  { t: 1.15, p: { flex: -30, abd: 14, er: 18, elbow: 28, pron: -8, wristExt: 4, scap: [-2, -3, -5, -1, -0.004] } },
  { t: 1.95, p: { flex: 3.6, abd: 9, er: 0, elbow: 0, pron: 20, wristExt: 0, scap: [0, 0, 0, 0, 0] } },
];
export function buildPhaseA(K3, armKeys = ARM_A) {
  const bow = BOW;
  K3 = K3 || buildProne();
  armKeys[armKeys.length - 1].p.flex = K3.info.armFlex;
  const trunkNames = SEG.map(s => s.name).filter(n => !ARM.includes(n.replace(/_[LR]$/, '')) && !LEG.includes(n.replace(/_[LR]$/, '')));
  const legNames = SEG.map(s => s.name).filter(n => LEG.includes(n.replace(/_[LR]$/, '')));
  const trunkKeys = [{ t: TA.trunk[0], L: bow.L }, { t: TA.trunk[1], L: K3.L }];
  const legKeys = [{ t: TA.legs[0], L: bow.L }, { t: TA.legs[1], L: K3.L }];
  const bodyAt = t => { const L = C.cloneL(bow.L); Object.assign(L, trackAt(trunkKeys, t, trunkNames), trackAt(legKeys, t, legNames)); return L; };
  // correction that makes the parametric arm start exactly at the bow's arm; faded out as the arm moves away
  const pl = {}; armFromThorax(pl, armKeys[0].p);
  const corr = {}; for (const n of ARM_NOFING) corr[n] = pl[n].clone().invert().multiply(bow.L[n + '_L']);
  const pEnd = {}; armFromThorax(pEnd, armKeys[armKeys.length - 1].p);
  const corrEnd = {}; for (const n of ARM_NOFING) corrEnd[n] = pEnd[n].clone().invert().multiply(K3.L[n + '_L']);
  const fGrip = leftOf(bow.L, FING), fRelax = fingerSet(RELAX);
  const frames = [];
  let phi = rootPitch(bow.rootQ);
  const n = Math.round(TA.end * 10);
  for (let i = 0; i <= n; i++) {
    const t = i / 10;
    const uf = smooth(TA.fingers[0], TA.fingers[1], t);
    const fl = {}; for (const f of FING) fl[f] = sl(fGrip[f], fRelax[f], uf);
    if (t <= TA.gripEnd + 1e-9) {
      const c = { L: C.cloneL(bow.L), rootQ: bow.rootQ.clone(), rootP: bow.rootP.clone() };
      for (const f of FING) { c.L[f + '_L'] = fl[f].clone(); c.L[f + '_R'] = mirrorQ(fl[f]); }
      frames.push({ t, c, grip: ['L', 'R'], arm: armKeys[0].p });
      continue;
    }
    const L = bodyAt(t);
    const p = paramAt(armKeys, t);
    const left = {}; armFromThorax(left, p);
    const uc = smooth(TA.corr[0], TA.corr[1], t), ue = smooth(1.4, TA.arms[1], t);
    for (const k of ARM_NOFING) left[k] = left[k].multiply(sl(corr[k], IDQ, uc)).multiply(sl(IDQ, corrEnd[k], ue));
    Object.assign(left, fl);
    setLR(L, left);
    const c = placeBody(L, phi, t); phi = c.phi;
    frames.push({ t, c, grip: [], arm: p });
  }
  return frames;
}

// ================= high plank and the prone push-up start (KB) =================
const HP = HP_DATA ? fromData(HP_DATA) : null;
const TOE_LM = LANDMARKS.filter(l => l.region === 'пальцы стоп');
const toeMean = W => { const m = V(0, 0, 0); for (const l of TOE_LM) m.add(pt(W, l.seg, l.pc)); return m.multiplyScalar(1 / TOE_LM.length); };
const BODY_NAMES = SEG.map(s => s.name).filter(n => !ARM.includes(n.replace(/_[LR]$/, '')));
const ARM_NAMES = SEG.map(s => s.name).filter(n => ARM.includes(n.replace(/_[LR]$/, '')));
const FOOT_NAMES = ['foot_L', 'foot_R'];
// the high plank body rotated down about the toes by alpha (degrees); feet keep their world orientation, toes pinned
function hpRotated(alpha) {
  const W0 = W_of(HP);
  const piv = toeMean(W0);
  const R = new THREE.Quaternion().setFromAxisAngle(V(1, 0, 0), alpha * D);
  const L = C.cloneL(HP.L);
  const rootQ = R.clone().multiply(HP.rootQ);
  let rootP = HP.rootP.clone().sub(piv).applyQuaternion(R).add(piv);
  // feet: undo the rotation so the foot keeps the world orientation it has in the plank
  for (const sd of ['L', 'R']) {
    const Wt = computeWorld(L, rootQ, rootP);
    const want = W0['foot_' + sd].q;
    L['foot_' + sd] = Wt['shank_' + sd].q.clone().invert().multiply(want);
  }
  const W1 = computeWorld(L, rootQ, rootP);
  rootP = rootP.clone().add(toeMean(W0).sub(toeMean(W1)));
  return { L, rootQ, rootP };
}
const isTrunkLm = l => !ARM_REG.has(l.region) && !['пальцы стоп', 'подушечки стоп', 'пятки', 'тыл стоп', 'наружный край стопы'].includes(l.region);
function buildKB() {
  // lower until the chest/belly/thighs touch the mat (arms are placed separately)
  const a = bisect(al => { const c = hpRotated(al); return lmY(W_of(c), isTrunkLm) - 0.0005; }, 0, 40);
  const c = hpRotated(a);
  return { ...c, alpha: a };
}
const TRUNK_SEGS = ['lumA', 'lumB', 'thA', 'thB', 'thC', 'neckA', 'neckB', 'head'];
function buildKL(K3L) {
  const pelvisLm = LANDMARKS.filter(l => l.region === 'таз');
  const rot = (al, th) => { const c = hpRotated(al); for (const n of TRUNK_SEGS) c.L[n] = K3L[n].clone();
    c.L.lumA = c.L.lumA.clone().multiply(qAx(X, th)); c.L.lumB = c.L.lumB.clone().multiply(qAx(X, th)); return c; };
  const solve = th => { const a = bisect(al => lmY(W_of(rot(al, th)), isTrunkLm) - 0.0005, 0, 45); return { a, c: rot(a, th) }; };
  const pelvisY = th => { const { c } = solve(th); return lmY(W_of(c), l => pelvisLm.includes(l)) - 0.0012; };
  const th = bisect(pelvisY, -12, 12, 30);
  const { a, c } = solve(th);
  return { ...c, alpha: a, lumbar: th };
}
const HPW = HP ? W_of(HP) : null;
const wristHP = sd => HPW['hand_' + sd].p.clone();
const handQHP = sd => HPW['hand_' + sd].q.clone();
// both arms to the high plank's hands from the current trunk; elbows back along the ribs
function armsToHP(c, lift = 0, frac = 1, fromW = null, alpha = null) {
  for (const sd of ['L']) {
    const sx = sd === 'L' ? 1 : -1;
    let T = wristHP(sd);
    let Q = handQHP(sd);
    if (fromW && frac < 1) {
      T = fromW['hand_' + sd].p.clone().lerp(T, frac);
      Q = fromW['hand_' + sd].q.clone().slerp(Q, frac);
    }
    T.y += lift;
    const W = W_of(c);
    const Sh = W['hum_' + sd].p;
    let pole = Sh.clone().add(V(0.08 * sx, 0.5, -0.05));
    if (alpha != null) { const piv = toeMean(HPW); const R = new THREE.Quaternion().setFromAxisAngle(V(1, 0, 0), alpha * D); const eHP = HPW['fore_' + sd].p.clone().sub(piv).applyQuaternion(R).add(piv); const w = smooth(KL_ALPHA * 0.6, 0, alpha); pole = pole.lerp(eHP.addScaledVector(eHP.clone().sub(Sh), 2), w); }
    const flexQ = armIK(c.L, c.rootQ, c.rootP, sd, T, pole, null);
    orientHand(c.L, c.rootQ, c.rootP, sd, Q, flexQ);
  }
  for (const n of ARM) c.L[n + '_R'] = mirrorQ(c.L[n + '_L']);
}
const spreadL = fingerSet(SPREAD);
const KNEE_BUMP = 14;
let KL_ALPHA = 22;
// hand path in phase B: up off the mat, forward beside the ribs, palm turns flat early, then down onto the plank's palm spot
function handTarget(sd, uh, W3) {
  const sx = sd === 'L' ? 1 : -1;
  const a = W3['hand_' + sd].p, b = wristHP(sd);
  const T = a.clone().lerp(b, smooth(0.1, 0.9, uh));
  T.y += 0.12 * smooth(0, 0.3, uh) * (1 - smooth(0.72, 1, uh));
  const Q0 = W3['hand_' + sd].q.clone(), Q2 = handQHP(sd);
  const Q1 = handBasis(V(sx * 0.85, 0, 0.5), V(0, -1, 0));
  const Q = uh < 0.5 ? Q0.clone().slerp(Q1, smooth(0.05, 0.5, uh)) : Q1.clone().slerp(Q2, smooth(0.5, 0.85, uh));
  return { T, Q };
}
const mirrorArm = c => { for (const n of ARM) c.L[n + '_R'] = mirrorQ(c.L[n + '_L']); };
// forearm rotation split: fore = flex · twist(θ about the forearm axis), ulna = twist⁻¹, hand = swing (wrist only)
const D_AX = S.hand_L.J.clone().sub(S.fore_L.J).normalize();
const twistQ = th => new THREE.Quaternion().setFromAxisAngle(D_AX, th * D);
function decompose(c, flexQ, Q) {
  c.L.fore_L = flexQ.clone(); c.L.ulna_L = IDQ.clone(); c.L.hand_L = IDQ.clone();
  const W = W_of(c);
  const delta = W.fore_L.q.clone().invert().multiply(Q);
  const v = V(delta.x, delta.y, delta.z);
  let th = 2 * Math.atan2(v.dot(D_AX), delta.w) / D;
  const swing = twistQ(th).invert().multiply(delta);
  return { th, swing };
}
const unwrap = (a, ref) => { while (a - ref > 180) a -= 360; while (a - ref < -180) a += 360; return a; };
let TH_PREV = null;
let E_REQ = {}, E_SMOOTH = null, CUR_T = 0;
function armsHands(c, uh, W3) {
  for (const n of ['clav', 'scap']) c.L[n + '_L'] = sl(W3._K3L[n + '_L'], HP.L[n + '_L'], smooth(0.1, 0.9, uh));
  const { T } = handTarget('L', uh, W3);
  const Sh = W_of(c).hum_L.p;
  const flexQ = armIK(c.L, c.rootQ, c.rootP, 'L', T, Sh.clone().add(V(0.1, 0.35, -0.1)), null);
  // start: the prone arm's own forearm rotation and wrist; end: the plank's flat palm on its spot
  const f0 = W3._K3L.fore_L, v0 = V(f0.x, f0.y, f0.z);
  const th0 = 2 * Math.atan2(v0.dot(D_AX), f0.w) / D;
  const sw0 = W3._K3L.hand_L.clone();
  const end = decompose(c, flexQ, handQHP('L'));
  let th1 = unwrap(end.th, TH_PREV == null ? th0 : TH_PREV);
  TH_PREV = th1;
  const th = th0 + (th1 - th0) * smooth(0.08, 0.92, uh);
  let sw = sl(sw0, IDQ, smooth(0, 0.3, uh));
  sw = sl(sw, end.swing, smooth(0.45, 1.0, uh));
  c.L.fore_L = flexQ.clone().multiply(twistQ(th));
  c.L.ulna_L = twistQ(th).invert();
  c.L.hand_L = sw;
  // while the hand travels, the fingers never touch the mat: extend (or flex) the wrist just enough
  {
    const handLm = l => l.side === 'L' && /^(hand|f\d[a-c]|f1m)_L$/.test(l.seg);
    const clear = uh < 0.8 ? 0.004 + 0.02 * Math.sin(Math.PI * uh / 0.8) : 0.0005;
    const lowAt = e => { c.L.hand_L = sw.clone().multiply(qAx(Z, e)); return lmY(W_of(c), handLm) - clear; };
    let e = 0;
    if (lowAt(0) < 0) { for (const cand of [10, 20, 30, 40, 50, 60]) { e = cand; if (lowAt(cand) >= 0) break; } e = Math.max(0, bisect(lowAt, e - 10, e, 20)); }
    E_REQ[CUR_T.toFixed(1)] = e;
    const es = E_SMOOTH ? (E_SMOOTH[CUR_T.toFixed(1)] || 0) : e;
    c.L.hand_L = sw.clone().multiply(qAx(Z, Math.min(60, Math.max(e, es))));
  }
  mirrorArm(c);
}
// after blending arm locals, put the hand back to its wanted world orientation (hand is a child of the forearm)
function setHandWorld(c, sd, Q) { const W = W_of(c); c.L['hand_' + sd] = W['fore_' + sd].q.clone().invert().multiply(Q); }
function fixHands(c, uh, W3, ws) { const { Q } = handTarget('L', uh, W3); setHandWorld(c, 'L', W3['hand_L'].q.clone().slerp(Q, ws)); c.L.hand_R = mirrorQ(c.L.hand_L); }

// ================= phase B «Ставлю ладони под плечи» (2.0–3.2) and C «Выжимаюсь в планку» (3.2–5.0) =================
export const TB = { start: 2.0, hands: [2.0, 3.3], toes: [2.2, 3.35], end: 3.4 };
export const TC = { start: 3.4, end: 5.0, armBlend: [4.3, 5.0] };
function buildBC(K3, KB, KL) {
  const frames = [];
  const W3 = W_of(K3); W3._K3L = K3.L;
  const K3arm = {}; for (const n of ARM_NAMES) K3arm[n] = K3.L[n].clone();
  // B
  const W3b = W3;
  for (let i = Math.round(TB.start * 10) + 1; i <= Math.round(TB.end * 10); i++) {
    const t = i / 10;
    const ub = smooth(TB.toes[0], TB.toes[1], t);           // legs/trunk K3 → KB
    const L = C.cloneL(K3.L);
    for (const n of BODY_NAMES) L[n] = sl(K3.L[n], KL.L[n], ub);
    // the feet leave the mat while they turn: knees bend a little, then straighten onto the tucked toes
    const bump = Math.sin(Math.PI * ub);
    for (const sd of ['L', 'R']) L['shank_' + sd] = L['shank_' + sd].clone().multiply(qAx(X, KNEE_BUMP * bump));
    const rootQ = sl(K3.rootQ, KL.rootQ, ub);
    const rootP = K3.rootP.clone().lerp(KL.rootP, ub);
    const c = { L, rootQ, rootP };
    const W0 = W_of(c); c.rootP.y -= lmY(W0, isTrunkLm);
    const uh = smooth(TB.hands[0], TB.hands[1], t);
    CUR_T = t;
    if (uh > 0) {
      armsHands(c, uh, W3b);
      const ws = smooth(TB.hands[0], TB.hands[0] + 0.3, t);
      for (const n of ARM_NAMES) c.L[n] = sl(K3arm[n], c.L[n], ws);
      const uf = smooth(2.2, 3.1, t);
      for (const f of FING) { const q = sl(K3.L[f + '_L'], spreadL[f], uf); c.L[f + '_L'] = q; c.L[f + '_R'] = mirrorQ(q); }
    }
    frames.push({ t, c, grip: [] });
  }
  // C: C1 the trunk lifts off the mat into one straight line (KL → KB), C2 the line rises about the toes (KB → plank)
  const n0 = Math.round(TC.start * 10), n1 = Math.round(TC.end * 10);
  for (let i = n0 + 1; i <= n1; i++) {
    const t = i / 10;
    if (i === n1) { frames.push({ t, c: { L: C.cloneL(HP.L), rootQ: HP.rootQ.clone(), rootP: HP.rootP.clone() }, grip: [] }); continue; }
    const u = smooth(TC.start, TC.end, t);
    // one continuous rotation from KL's angle to 0; the trunk shape goes K3 → plank during the first part
    const alpha = KL.alpha * (1 - u);
    const c = hpRotated(alpha);
    const ut = smooth(TC.start, TC.start + 0.8, t);
    for (const n of TRUNK_SEGS) c.L[n] = sl(KL.L[n], HP.L[n], ut);
    armsToHP(c, 0, 1, null, alpha);
    setHandWorld(c, 'L', handQHP('L')); c.L.hand_R = mirrorQ(c.L.hand_L);
    for (const f of FING) { c.L[f + '_L'] = HP.L[f + '_L'].clone(); c.L[f + '_R'] = HP.L[f + '_R'].clone(); }
    frames.push({ t, c, grip: [] });
  }
  return frames;
}

function main() {
  if (!HP) throw new Error('cycle_keys/highplank.json missing');
  const KB = buildKB();
  const K3raw = buildProne();
  const KL = buildKL(K3raw.L); KL_ALPHA = KL.alpha;
  { const W = W_of(KL); setPinTarget(pt(W, PIN.seg, PIN.pc).z); console.log('KL alpha', KL.alpha.toFixed(2), 'lumbar', KL.lumbar.toFixed(2), 'pelvis contact z', pt(W, PIN.seg, PIN.pc).z.toFixed(3), 'y', pt(W, PIN.seg, PIN.pc).y.toFixed(3)); }
  console.log('KB alpha', KB.alpha.toFixed(2), 'pelvis z', W_of(KB).pelvis.p.z.toFixed(3), 'K3 pelvis z', W_of(K3raw).pelvis.p.z.toFixed(3));
  const A_ = buildPhaseA(K3raw);
  const K3 = A_[A_.length - 1].c;
  buildBC(K3, KB, KL);
  { const ks = Object.keys(E_REQ).map(Number).sort((a, b) => a - b); const mx = {};
    for (const k of ks) { let m = 0; for (const j of ks) if (Math.abs(j - k) <= 0.31) m = Math.max(m, E_REQ[j.toFixed(1)]); mx[k] = m; }
    E_SMOOTH = {}; for (const k of ks) { let a = 0, n = 0; for (const j of ks) if (Math.abs(j - k) <= 0.21) { a += mx[j]; n++; } E_SMOOTH[k.toFixed(1)] = a / n; }
    TH_PREV = null; }
  const BC = buildBC(K3, KB, KL);
  const all = [...A_, ...BC];
  const frames = all.map(f => toFrame(f.c, f.t, { grip: f.grip }));
  frames[0] = { ...anchors.bow, t: 0, grip: ['L', 'R'] };
  frames[frames.length - 1] = { local: HP_DATA.local, root: HP_DATA.root, rootP: HP_DATA.rootP, t: 5.0, grip: [] };
  const out = { id: 'seg4', from: 'bow', to: 'highplank', fps: 10, duration: 5.0,
    phases: [{ t0: 0, t1: 2.0, label: 'Отпускаю руки и ноги' }, { t0: 2.0, t1: 3.4, label: 'Ставлю ладони под плечи' }, { t0: 3.4, t1: 5.0, label: 'Выжимаюсь в планку' }],
    frames };
  for (const f of frames) if (f.t > 0.2 && f.t <= 2.0) f.note = 'корпус перекатывается и чуть съезжает назад на животе, пока опускаются ноги (≈9 см за 2 с)';
  fs.writeFileSync('cycle_keys/seg4.json', JSON.stringify(out));
  console.log('seg4.json', frames.length, 'frames');
}
if (process.argv[1] && process.argv[1].endsWith('cycle_seg4.mjs')) main();
