// Build-time pose solver: computes joint rotations for the five poses and writes poses.json.
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';

const A = createAnatomy(THREE);
const { V, S, SEG, qAx, AX, computeWorld, pt, mirrorQ, scapulaPose, comOf, LANDMARKS, D, AZ, IDQ, placeW, sideName, measure, support } = A;
const X = AX.X, Y = AX.Y, Z = AX.Z;

const blankLocal = () => { const L = {}; for (const s of SEG) L[s.name] = new THREE.Quaternion(); return L; };
const setLR = (L, left) => { for (const k in left) { if (k.startsWith('_')) continue; L[k + '_L'] = left[k].clone(); L[k + '_R'] = mirrorQ(left[k]); } };
function bisect(f, a, b, n = 60) {
  let fa = f(a), fb = f(b);
  if (fa === 0) return a;
  if (fb === 0) return b;
  if ((fa > 0) === (fb > 0)) return Math.abs(fa) < Math.abs(fb) ? a : b;
  for (let i = 0; i < n; i++) { const m = (a + b) / 2, fm = f(m); if ((fm > 0) === (fa > 0)) { a = m; fa = fm; } else { b = m; fb = fm; } }
  return (a + b) / 2;
}
const LP = (base, x, y, z) => ({ seg: sideName(base, 'L'), p: V(...AZ(base, [x, y, z])) });
const wpt = (W, q) => pt(W, q.seg, q.p);
const lmksOn = (pred) => LANDMARKS.filter(pred);

// ---------- fingers ----------
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
// fingers flat on the floor: a few degrees at the knuckles bring the finger pads down to the level of the palm
const SPREAD = { spread: [0, 14, 0, 10, 20], flex: [[7.5, 0, 0], [5.7, 0, 0], [4.8, 0, 0], [2.8, 0, 0]], thumb: { abd: 42 } };
const FIST = { flex: [[85, 100, 60]], thumb: { abd: -5, flex: 40, mcp: 35, ip: 25 } };
const GRIP = { flex: [[45, 55, 30], [48, 58, 32], [50, 60, 34], [52, 62, 35]], thumb: { abd: 35, flex: 25, mcp: 15, ip: 10 } };
const SOFT = { spread: [0, 4, 0, 3, 6], flex: [[15, 22, 12]], thumb: { abd: 18, flex: 5, mcp: 5, ip: 5 } };

// ---------- hand orientation: split into forearm rotation and wrist ----------
function handBasis(fingersDir, palmNormal) {
  const g = fingersDir.clone().normalize();
  const n = palmNormal.clone(); n.addScaledVector(g, -n.dot(g)); n.normalize();
  const Xw = n.clone().negate(), Yw = g.clone().negate(), Zw = Xw.clone().cross(Yw).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(Xw, Yw, Zw));
}
// hand that continues the forearm as seen along the palm normal: the wrist bends only in flexion/extension
function gripHand(f, n) {
  const g = f.clone().addScaledVector(n, -f.dot(n)).normalize();
  return handBasis(g, n);
}
// straight hand with the palm towards n, then bent back at the wrist by ext degrees
function bentHand(f, n, ext) {
  const nn = n.clone().addScaledVector(f, -n.dot(f)).normalize();
  const t = nn.clone().cross(f).normalize();
  const q = new THREE.Quaternion().setFromAxisAngle(t, ext * D);
  return handBasis(f.clone().applyQuaternion(q), nn.applyQuaternion(q));
}
// wrist position that puts the palm centre on point C for a given hand orientation
const PALM_OFF = V(0.196 - 0.212, 0.75 - 0.815, 0.008 + 0.022);
const wristFor = (C, Qh) => C.clone().sub(PALM_OFF.clone().applyQuaternion(Qh));
function orientHand(L, left, rootQ, rootP, target, flexQ) {
  left.fore = flexQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  setLR(L, left);
  const W = computeWorld(L, rootQ, rootP);
  const delta = W.fore_L.q.clone().invert().multiply(target);
  const dAx = S.hand_L.J.clone().sub(S.fore_L.J).normalize();
  const vv = V(delta.x, delta.y, delta.z);
  const pr = dAx.clone().multiplyScalar(vv.dot(dAx));
  let twist = new THREE.Quaternion(pr.x, pr.y, pr.z, delta.w);
  if (twist.lengthSq() < 1e-12) twist = IDQ.clone();
  twist.normalize();
  const swing = twist.clone().invert().multiply(delta);
  left.fore = flexQ.clone().multiply(twist);
  left.ulna = twist.clone().invert();
  left.hand = swing;
  setLR(L, left);
}

// same as orientHand, but the forearm turn (pronation) is given: the rest of the hand orientation goes into the wrist
function orientHandTw(L, left, rootQ, rootP, target, flexQ, phiDeg) {
  left.fore = flexQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  setLR(L, left);
  const W = computeWorld(L, rootQ, rootP);
  const delta = W.fore_L.q.clone().invert().multiply(target);
  const dAx = S.hand_L.J.clone().sub(S.fore_L.J).normalize();
  const twist = new THREE.Quaternion().setFromAxisAngle(dAx, phiDeg * D);
  left.fore = flexQ.clone().multiply(twist);
  left.ulna = twist.clone().invert();
  left.hand = twist.clone().invert().multiply(delta);
  setLR(L, left);
}

// ---------- arm: two-bone reach to a wrist target ----------
const LH = S.fore_L.J.distanceTo(S.hum_L.J), LF = S.hand_L.J.distanceTo(S.fore_L.J);
const N_HUM = S.fore_L.J.clone().sub(S.hum_L.J).normalize();
function basisQ(y, z) { const x = y.clone().cross(z).normalize(); const zz = x.clone().cross(y).normalize(); return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, zz)); }
const R0 = basisQ(N_HUM.clone().negate(), V(0, 0, 1));
function aimArm(L, left, rootQ, rootP, wristT, pole, creaseDir) {
  setLR(L, left);
  const W = computeWorld(L, rootQ, rootP);
  const Sh = W.hum_L.p.clone();
  const T = wristT.clone();
  let flex = 0;
  for (let it = 0; it < 8; it++) {
    const c = Math.min(T.distanceTo(Sh), LH + LF - 1e-5);
    const e = T.clone().sub(Sh).normalize();
    const cosA = Math.max(-1, Math.min(1, (LH * LH + c * c - LF * LF) / (2 * LH * c)));
    let perp = pole.clone().sub(Sh); perp.addScaledVector(e, -perp.dot(e));
    if (perp.lengthSq() < 1e-12) perp = V(0, 0, 1);
    perp.normalize();
    const E = Sh.clone().addScaledVector(e, LH * cosA).addScaledVector(perp, LH * Math.sqrt(Math.max(0, 1 - cosA * cosA)));
    const d = E.clone().sub(Sh).normalize();
    const f = T.clone().sub(E).normalize();
    flex = Math.acos(Math.max(-1, Math.min(1, d.dot(f)))) / D;
    let zt = creaseDir && flex < 3 ? creaseDir.clone() : f.clone();
    zt.addScaledVector(d, -zt.dot(d));
    if (zt.lengthSq() < 1e-10) zt = perp.clone();
    zt.normalize();
    const Rw = basisQ(d.clone().negate(), zt).multiply(R0.clone().invert());
    left.hum = W.scap_L.q.clone().invert().multiply(Rw);
    left.fore = qAx(X, -flex); left.ulna = IDQ.clone();
    setLR(L, left);
    const W2 = computeWorld(L, rootQ, rootP);
    const err = wristT.clone().sub(W2.hand_L.p);
    if (err.length() < 2e-4) break;
    T.add(err);
  }
  return qAx(X, -flex);
}

// ---------- resting on the floor: lower convex hull in the sagittal plane ----------
function hull2(P) {
  const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Q) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
function settle(L, phi0) {
  const W0 = computeWorld(L, qAx(X, phi0), V(0, 0, 0));
  const pts = LANDMARKS.map(l => pt(W0, l.seg, l.pc));
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
      const rot = q => [q[1] * s + q[0] * c, q[1] * c - q[0] * s];
      const ra = rot(a), rb = rot(b);
      let ok = true;
      for (const q of H) if (rot(q)[1] < ra[1] - 1e-7) { ok = false; break; }
      if (!ok) continue;
      const rc = rot([com.z, com.y]);
      if (rc[0] < Math.min(ra[0], rb[0]) - 1e-6 || rc[0] > Math.max(ra[0], rb[0]) + 1e-6) continue;
      const dd = Math.atan2(Math.sin(d), Math.cos(d));
      if (best === null || Math.abs(dd) < Math.abs(best)) best = dd;
    }
  }
  return phi0 + (best || 0) / D;
}
const heightsAt = (L, rootQ) => { const W = placeW(L, rootQ); return { W, h: (pred) => { let m = Infinity; for (const l of LANDMARKS) if (pred(l)) m = Math.min(m, pt(W, l.seg, l.pc).y); return m; } }; };
const isL = l => l.side === 'L' || l.side === 'C';

// ================= 1. Горка =================
const DOG = {
  lumA: -7, lumB: -7, thA: -6, thB: -7.5, thC: -6, neckA: 2, neckB: 1, head: 0,
  hipFlex: 116, hipAbd: 0, hipRot: -8, kneeFlex: 10,
  scapUp: 44, scapPost: 15, scapProt: -6, scapElev: -0.012, clavRot: 22,
  thFlex: 188, thAbd: 6, thER: 4, ankleY: 0.115, hand: SPREAD,
};
function buildDog(o) {
  const p = Object.assign({}, DOG, o || {});
  const L = blankLocal();
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC', 'neckA', 'neckB', 'head']) L[k] = qAx(X, p[k]);
  const left = {};
  left.thigh = qAx(X, -p.hipFlex).multiply(qAx(Z, p.hipAbd)).multiply(qAx(Y, p.hipRot));
  left.shank = qAx(X, p.kneeFlex);
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  const RTH = qAx(X, -p.thFlex).multiply(qAx(Z, p.thAbd)).multiply(qAx(Y, p.thER));
  left.hum = left.clav.clone().multiply(left.scap).invert().multiply(RTH);
  left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  fingers(left, p.hand);
  setLR(L, left);
  const handLm = LANDMARKS.filter(l => l.side === 'L' && /^(hand|f\d[a-c]|f1m)_L$/.test(l.seg));
  // palm flat, middle finger along the forearm, thumb towards the other hand; palm defines the floor
  const flatHand = rootQ => {
    const W = computeWorld(L, rootQ, V(0, 0, 0));
    const fdir = W.hand_L.p.clone().sub(W.fore_L.p); fdir.y = 0; fdir.normalize();
    orientHand(L, left, rootQ, V(0, 0, 0), handBasis(fdir, V(0, -1, 0)), IDQ.clone());
    const W2 = computeWorld(L, rootQ, V(0, 0, 0));
    let mn = Infinity; for (const l of handLm) mn = Math.min(mn, pt(W2, l.seg, l.pc).y);
    return { W: W2, rootP: V(0, -mn, 0) };
  };
  const phi = bisect(ph => { const { W, rootP } = flatHand(qAx(X, ph)); return W.foot_L.p.y + rootP.y - p.ankleY; }, 100, 178);
  const rootQ = qAx(X, phi);
  const { rootP } = flatHand(rootQ);
  // ball of the foot on the floor, toes flat
  const minOf = (W, pred) => { let m = Infinity; for (const l of LANDMARKS) if (l.side === 'L' && pred(l)) m = Math.min(m, pt(W, l.seg, l.pc).y); return m; };
  left.foot = qAx(X, bisect(a => { left.foot = qAx(X, a); setLR(L, left); return minOf(computeWorld(L, rootQ, rootP), l => l.region === 'подушечки стоп') - 0.001; }, -60, 5));
  setLR(L, left);
  left.toes = qAx(X, bisect(a => { left.toes = qAx(X, a); setLR(L, left); return minOf(computeWorld(L, rootQ, rootP), l => l.seg === 'toes_L') - 0.001; }, -80, 30));
  setLR(L, left);
  return { L, rootQ };
}

// ================= 2. Планка на предплечьях =================
const PLANK = { lum: 3, thor: [1, 1, 0], neck: [0, 0, 0], hipExt: 5, ankleDorsi: 12, scapUp: 18, scapPost: 4, scapProt: 10, scapElev: 0, clavRot: 8, hand: FIST };
function buildPlank(o) {
  const p = Object.assign({}, PLANK, o || {});
  const L = blankLocal();
  L.lumA = qAx(X, p.lum); L.lumB = qAx(X, p.lum);
  L.thA = qAx(X, p.thor[0]); L.thB = qAx(X, p.thor[1]); L.thC = qAx(X, p.thor[2]);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
  const left = {};
  left.thigh = qAx(X, p.hipExt); left.shank = IDQ.clone(); left.foot = qAx(X, -p.ankleDorsi);
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  fingers(left, p.hand);
  const st = { sh: 90, el: 90, toes: 60, ir: 40, add: 0 };
  const elbowC = LP('ulna', 0.205, 1.045, -0.05), toeC = LP('toes', 0.1, 0.004, 0.178), toeB = LP('toes', 0.1, 0.006, 0.145);
  const hallux = LP('toes', 0.064, 0.012, 0.188);
  const apply = () => {
    const RTH = qAx(X, -st.sh).multiply(qAx(Y, -st.ir));
    left.hum = left.clav.clone().multiply(left.scap).invert().multiply(RTH);
    left.fore = qAx(X, -st.el); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
    left.toes = qAx(X, -st.toes);
    // legs brought together until the big toes touch
    left.thigh = qAx(X, p.hipExt).multiply(qAx(Z, st.add));
    setLR(L, left);
    const phi = bisect(ph => { const W = computeWorld(L, qAx(X, ph), V(0, 0, 0)); return wpt(W, elbowC).y - wpt(W, toeC).y; }, 50, 140);
    return { phi, W: computeWorld(L, qAx(X, phi), V(0, 0, 0)) };
  };
  for (let it = 0; it < 6; it++) {
    st.sh = bisect(v => { st.sh = v; const { W } = apply(); return W.fore_L.p.z - W.hum_L.p.z; }, 50, 130);
    st.el = bisect(v => { st.el = v; const { W } = apply(); return W.hand_L.p.y - W.fore_L.p.y; }, 30, 150);
    st.toes = bisect(v => { st.toes = v; const { W } = apply(); return wpt(W, toeC).y - wpt(W, toeB).y; }, -10, 100);
    // forearms parallel: each hand straight in front of its elbow
    st.ir = bisect(v => { st.ir = v; const { W } = apply(); return W.hand_L.p.x - W.fore_L.p.x; }, -40, 80);
    st.add = bisect(a => { st.add = a; const { W } = apply(); return wpt(W, hallux).x - 0.0015; }, -12, 6);
  }
  const { phi } = apply();
  return { L, rootQ: qAx(X, phi), info: st };
}

// ================= 3. Полупрогиб =================
// elbows drawn back behind the line of the back ("локти сводим за спиной") at the height of the shoulders:
// upper arms and forearms parallel to the floor, hands beside the shoulders, wrists pulled back
const HALF = { lumA: -2, lumB: -3, thA: -2, thB: -12, thC: -12, neck: [0, 0, 0], hipExt: 0, toes: 28, scapUp: 20, scapPost: 8, scapProt: -20, scapElev: -0.012, clavRot: 8, hand: SOFT, armBack: 45, elbowFlex: 145, forearmUp: -10, wristExt: 55 };
function halfArms(L, left, p, rootQ) {
  setLR(L, left);
  const W0 = computeWorld(L, rootQ, V(0, 0, 0));
  const up = V(0, 1, 0);
  const head = V(0, 1, 0).applyQuaternion(W0.thC.q);
  const fwd = head.clone().addScaledVector(up, -head.dot(up)).normalize();
  const lat = up.clone().cross(fwd).normalize();
  const Sh = W0.hum_L.p.clone();
  // upper arm level with the floor, out to the side and armBack degrees towards the feet
  const a = p.armBack * D;
  const u = lat.clone().multiplyScalar(Math.cos(a)).addScaledVector(fwd, -Math.sin(a));
  const E = Sh.clone().addScaledVector(u, LH);
  // forearm folds forward, also level with the floor (forearmUp tilts it up)
  const back = u.clone().negate();
  const sgn = Math.sign(back.clone().cross(fwd).dot(up)) || 1;
  const fh = back.clone().applyAxisAngle(up, sgn * (180 - p.elbowFlex) * D);
  const f = fh.multiplyScalar(Math.cos(p.forearmUp * D)).addScaledVector(up, Math.sin(p.forearmUp * D)).normalize();
  const flexQ = aimArm(L, left, rootQ, V(0, 0, 0), E.clone().addScaledVector(f, LF), E);
  const W1 = computeWorld(L, rootQ, V(0, 0, 0));
  // palm down, then the hand pulled back towards the forearm ("кисти тянем на себя"): fingers point up
  orientHand(L, left, rootQ, V(0, 0, 0), bentHand(W1.hand_L.p.clone().sub(W1.fore_L.p).normalize(), V(0, -1, 0), p.wristExt), flexQ);
  setLR(L, left);
}
function halfConfig(p, phiArms) {
  const L = blankLocal();
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) L[k] = qAx(X, p[k]);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
  const left = {};
  left.thigh = qAx(X, p.hipExt); left.shank = IDQ.clone(); left.foot = qAx(X, p.plantar || 45); left.toes = qAx(X, p.toes);
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  left.hum = IDQ.clone(); left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  fingers(left, p.hand);
  setLR(L, left);
  halfArms(L, left, p, qAx(X, phiArms ?? 95));
  return { L, left };
}
function buildHalf(o) {
  const p = Object.assign({}, HALF, o || {});
  let best = null, phiArms = 95;
  // the arms are levelled with the floor, which depends on how the body settles: two passes
  for (let pass = 0; pass < 2; pass++) {
  best = null;
  for (let hipExt = -6; hipExt <= 10; hipExt += 1) {
    for (let plantar = 20; plantar <= 72; plantar += 2) {
      const q = Object.assign({}, p, { hipExt, plantar });
      const conf = halfConfig(q, phiArms), phi = settle(conf.L, 90);
      const { h } = heightsAt(conf.L, qAx(X, phi));
      const feet = h(l => l.region === 'тыл стоп' && isL(l)), knee = h(l => l.region === 'колени' && isL(l)), ribs = h(l => l.region === 'нижние рёбра'), pubis = h(l => l.region === 'таз'), belly = h(l => l.region === 'живот');
      const arms = h(l => ['локти', 'предплечья', 'кисти', 'ладони', 'пальцы рук', 'кулаки', 'плечи'].includes(l.region));
      const pen = (feet < 0.004 ? 0 : 1 + feet * 20) + (ribs < 0.006 ? 0 : 0.5 + ribs * 20) + Math.max(0, 0.012 - knee) * 20 + (Math.min(pubis, belly) < 0.006 ? 0 : 0.5 + Math.min(pubis, belly) * 20) + (arms > 0.02 ? 0 : 1);
      const thigh = h(l => l.region === 'бёдра' && isL(l));
      const score = pen + plantar / 2000 + Math.abs(knee - 0.02) * 10 + Math.max(0, thigh - 0.012) * 10;
      if (!best || score < best.score) best = { score, hipExt, plantar, conf, phi, feet, knee, ribs, pubis, belly, thigh };
    }
  }
  phiArms = best.phi;
  }
  // final arms for the settled body
  const conf = halfConfig(Object.assign({}, p, { hipExt: best.hipExt, plantar: best.plantar }), best.phi);
  const phi = settle(conf.L, 90);
  return { L: conf.L, rootQ: qAx(X, phi), info: { hipExt: best.hipExt, plantar: best.plantar, feet: best.feet, knee: best.knee, ribs: best.ribs, pubis: best.pubis, belly: best.belly, thigh: best.thigh, phi, phiArms: best.phi } };
}

// ================= 4. Корзинка =================
const BOW = { lumA: -10, lumB: -10, thA: -7, thB: -9, thC: -8, neck: [-3, -2, 0], hipExt: 22, hipAdd: 2, dorsi: 15, toes: -8, gripX: 0.07, gripY: 0.16, gripZ: -0.012, scapUp: -6, scapPost: -8, scapProt: -14, scapElev: -0.01, clavRot: -4, hand: GRIP };
function buildBow(o) {
  const p = Object.assign({}, BOW, o || {});
  const L = blankLocal();
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) L[k] = qAx(X, p[k]);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
  const left = {};
  left.thigh = qAx(X, p.hipExt).multiply(qAx(Z, -p.hipAdd));
  left.foot = qAx(X, -p.dorsi); left.toes = qAx(X, p.toes);
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  left.hum = IDQ.clone(); left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  fingers(left, p.hand);
  // hands hold the shins from the inside: palm on the inner surface of the lower shin, fingers round its front
  const gripPt = LP('shank', p.gripX, p.gripY, p.gripZ);
  const grip = W => {
    const C = wpt(W, gripPt), Sh = W.hum_L.p;
    const n = V(1, 0, 0).applyQuaternion(W.shank_L.q);
    let wrist = C.clone(), Qh = null;
    for (let i = 0; i < 5; i++) { Qh = gripHand(wrist.clone().sub(Sh).normalize(), n); wrist = wristFor(C, Qh); }
    return { wrist, Qh };
  };
  let knee = bisect(k => {
    left.shank = qAx(X, k); setLR(L, left);
    const W = computeWorld(L, IDQ, V(0, 0, 0));
    return W.hum_L.p.distanceTo(grip(W).wrist) - (LH + LF - 0.0003);
  }, 40, 165);
  left.shank = qAx(X, knee); setLR(L, left);
  let W = computeWorld(L, IDQ, V(0, 0, 0));
  const g = grip(W);
  // arm roll: "локоть смотрит внутрь" = outward rotation of the shoulder; the forearm turns the palm onto the shin
  const J = id => A.JOINTS.find(j => j.id === id);
  const sh = W.hum_L.p.clone();
  const aimWith = alpha => {
    const d = g.wrist.clone().sub(sh).normalize();
    const lat = V(1, 0, 0); lat.addScaledVector(d, -lat.dot(d)).normalize();
    const oth = d.clone().cross(lat).normalize();
    const crease = lat.clone().multiplyScalar(Math.cos(alpha)).addScaledVector(oth, Math.sin(alpha));
    const flexQ = aimArm(L, left, IDQ, V(0, 0, 0), g.wrist, g.wrist.clone().add(V(-1, 0, 0)), crease);
    orientHand(L, left, IDQ, V(0, 0, 0), g.Qh, flexQ);
    const Wc = computeWorld(L, IDQ, V(0, 0, 0));
    const comp = (id, key) => A.analyzeJoint(J(id), L, Wc).comps.find(c => c.key === key).signed;
    return { er: comp('gh_L', 'er'), pron: comp('el_L', 'pron'), ext: comp('wr_L', 'ext'), dev: comp('wr_L', 'dev') };
  };
  // palms on the inner shins need about 180° of outward shoulder rotation plus supination together:
  // the arm roll shares it so that neither the shoulder nor the forearm goes further than the other
  const cost = r => Math.abs(r.er - (-r.pron)) + (r.er < 0 ? 100 : 0);
  let alpha = 0, best = Infinity;
  for (let a = -Math.PI; a < Math.PI; a += Math.PI / 360) { const c = cost(aimWith(a)); if (c < best) { best = c; alpha = a; } }
  const res = aimWith(alpha);
  const phi = settle(L, 90);
  return { L, rootQ: qAx(X, phi), info: { knee, phi, er: res.er, pron: res.pron, wristExt: res.ext, wristDev: res.dev } };
}

// ================= 5. Наклон =================
const FOLD = { hipFlex: 128, lum: [4, 4], thor: [3, 3, 3], neck: [0, 0, 0], scapUp: 8, scapPost: 0, scapProt: 6, scapElev: -0.006, clavRot: 0, hand: GRIP, gripC: [0.125, 0.17, -0.04] };
function buildFold(o) {
  const p = Object.assign({}, FOLD, o || {});
  const L = blankLocal();
  L.lumA = qAx(X, p.lum[0]); L.lumB = qAx(X, p.lum[1]);
  L.thA = qAx(X, p.thor[0]); L.thB = qAx(X, p.thor[1]); L.thC = qAx(X, p.thor[2]);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
  const left = {};
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  left.hum = IDQ.clone(); left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  left.shank = IDQ.clone(); left.toes = IDQ.clone(); left.foot = IDQ.clone();
  fingers(left, p.hand);
  const eA = V(0.112, 0.02, -0.05), eB = V(0.13, 0.015, 0.118);
  const yaw0 = -Math.atan2(eB.x - eA.x, eB.z - eA.z) / D;
  const footYaw = qAx(Y, yaw0);
  const st = { add: 0, rot: 0, phi: 110 };
  const hallux = LP('toes', 0.064, 0.012, 0.188), heel = LP('foot', 0.094, 0.005, -0.045), toeTip = LP('toes', 0.074, 0.004, 0.19);
  const legs = () => { left.thigh = qAx(X, -p.hipFlex).multiply(qAx(Z, st.add)).multiply(qAx(Y, st.rot)); };
  const flatFeet = (rootQ) => { setLR(L, left); const W = computeWorld(L, rootQ, V(0, 0, 0)); left.foot = W.shank_L.q.clone().invert().multiply(footYaw); setLR(L, left); };
  const arms = () => {
    setLR(L, left);
    const W = computeWorld(L, IDQ, V(0, 0, 0));
    // palm on the back-outer side of the lower shin, fingers down towards the heel, curling round the Achilles tendon
    const C = pt(W, 'shank_L', V(...p.gripC));
    const n = pt(W, 'shank_L', V(0.095, p.gripC[1], -0.008)).sub(C).normalize();
    const elbowPole = pt(W, 'shank_L', V(0.14, 0.5, -0.02));
    let wrist = C.clone().addScaledVector(V(0, 1, 0).applyQuaternion(W.shank_L.q), 0.07), Qh = null, flexQ = null;
    for (let i = 0; i < 5; i++) {
      flexQ = aimArm(L, left, IDQ, V(0, 0, 0), wrist, elbowPole);
      const W2 = computeWorld(L, IDQ, V(0, 0, 0));
      Qh = gripHand(W2.hand_L.p.clone().sub(W2.fore_L.p).normalize(), n);
      wrist = wristFor(C, Qh);
    }
    flexQ = aimArm(L, left, IDQ, V(0, 0, 0), wrist, elbowPole);
    orientHand(L, left, IDQ, V(0, 0, 0), Qh, flexQ);
  };
  for (let it = 0; it < 5; it++) {
    legs(); arms();
    st.phi = bisect(ph => { const R = qAx(X, ph); flatFeet(R); const W = computeWorld(L, R, V(0, 0, 0)); const com = comOf(W); const mid = (wpt(W, heel).z + wpt(W, toeTip).z) / 2; return com.z - mid; }, 60, 170);
    const R = qAx(X, st.phi);
    st.rot = bisect(r => { st.rot = r; legs(); flatFeet(R); const W = computeWorld(L, R, V(0, 0, 0)); const kd = V(0, 0, 1).applyQuaternion(W.shank_L.q); return Math.atan2(kd.x, kd.z) / D - yaw0; }, -30, 30);
    st.add = bisect(a => { st.add = a; legs(); flatFeet(R); const W = computeWorld(L, R, V(0, 0, 0)); return wpt(W, hallux).x - 0.0015; }, -12, 6);
  }
  legs(); arms();
  const R = qAx(X, st.phi); flatFeet(R);
  return { L, rootQ: R, info: st };
}


// ================= 6. Стойка на руках =================
// body upside down (a half turn about the z axis keeps the face towards +z), arms vertical, palms flat, fingers forward;
// the trunk leans a little about the shoulders until the centre of mass is over the middle of the palms
// final values: found by hs_opt.mjs (least effort with support margin >= 3 cm), hands at shoulder width (thAbd)
const HSTAND = { lumA: 5, lumB: -5, thA: -10, thB: 8, thC: 6, neck: [-1, -2, -1], hipExt: 7, plantar: 48, toes: 12, scapUp: 46.5, scapPost: 20, scapProt: 4, scapElev: 0.038, clavRot: 26, thAbd: -2, thER: 45, armTilt: -5.5, comAhead: 0.05, hand: SPREAD };
function buildHandstand(o) {
  const p = Object.assign({}, HSTAND, o || {});
  const L = blankLocal();
  L.lumA = qAx(X, p.lumA); L.lumB = qAx(X, p.lumB);
  L.thA = qAx(X, p.thA); L.thB = qAx(X, p.thB); L.thC = qAx(X, p.thC);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
  const left = {};
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
  left.shank = IDQ.clone(); left.foot = qAx(X, p.plantar); left.toes = qAx(X, p.toes);
  fingers(left, p.hand);
  const hallux = LP('toes', 0.064, 0.012, 0.188);
  const palmL = A.LANDMARKS.find(l => l.tag === 'palm' && l.side === 'L'), palmR = A.LANDMARKS.find(l => l.tag === 'palm' && l.side === 'R');
  const rootOf = t => qAx(Z, 180).multiply(qAx(X, t));
  const st = { tilt: 0, add: 0, pron: 0 };
  const spineSum = p.lumA + p.lumB + p.thA + p.thB + p.thC; // the thorax leans by the sum of the spine angles; the arms are kept vertical in the world
  const apply = () => {
    const RTH = qAx(X, -(180 + st.tilt + spineSum - p.armTilt)).multiply(qAx(Z, p.thAbd)).multiply(qAx(Y, p.thER));
    left.hum = left.clav.clone().multiply(left.scap).invert().multiply(RTH);
    left.thigh = qAx(X, p.hipExt).multiply(qAx(Z, st.add));
    left.fore = IDQ.clone(); left.ulna = IDQ.clone(); left.hand = IDQ.clone();
    setLR(L, left);
    const rq = rootOf(st.tilt);
    const target = handBasis(V(0, 0, 1), V(0, -1, 0)), wrJ = A.JOINTS.find(j => j.id === 'wr_L');
    const devAt = ph => { orientHandTw(L, left, rq, V(0, 0, 0), target, IDQ.clone(), ph); return A.analyzeJoint(wrJ, L, computeWorld(L, rq, V(0, 0, 0))).comps.find(c => c.key === 'dev').signed; };
    st.pron = bisect(devAt, -110, 110);
    orientHandTw(L, left, rq, V(0, 0, 0), target, IDQ.clone(), st.pron);
    return { rq, W: placeW(L, rq) };
  };
  for (let it = 0; it < 4; it++) {
    // legs together: the big toes touch (measured in the standing frame, where the left side is at +x)
    st.add = bisect(a => { st.add = a; apply(); return wpt(computeWorld(L, IDQ, V(0, 0, 0)), hallux).x - 0.0015; }, -12, 6);
    st.tilt = bisect(t => { st.tilt = t; const { W } = apply(); const pz = (wpt(W, { seg: palmL.seg, p: palmL.p }).z + wpt(W, { seg: palmR.seg, p: palmR.p }).z) / 2; return comOf(W).z - (W.hand_L.p.z + p.comAhead); }, -30, 30);
  }
  const { rq, W } = apply();
  const zc = q => +(q * 100).toFixed(1);
  const pz = (wpt(W, { seg: palmL.seg, p: palmL.p }).z + wpt(W, { seg: palmR.seg, p: palmR.p }).z) / 2;
  const info = { add: +st.add.toFixed(2), tilt: +st.tilt.toFixed(2), pron: +st.pron.toFixed(1), 'cm z rel. shoulder': { com: zc(comOf(W).z - W.hum_L.p.z), wrist: zc(W.hand_L.p.z - W.hum_L.p.z), palm: zc(pz - W.hum_L.p.z), hip: zc(W.thigh_L.p.z - W.hum_L.p.z), ankle: zc(W.foot_L.p.z - W.hum_L.p.z), head: zc(W.head.p.z - W.hum_L.p.z) } };
  // the body is built facing +z (a half turn about z); the final half turn about the vertical makes it a plain forward flip about x (X(180° + lean)),
  // so that the slider from standing turns the body over forward like the other poses and the left side stays at +x
  return { L, rootQ: qAx(Y, 180).multiply(rq), info };
}

export { buildHandstand, HSTAND, buildDog, buildPlank, buildHalf, buildBow, buildFold, A };

// ================= run =================
import { fileURLToPath } from 'url';
if (process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1]) {
  const out = {};
  const toData = c => ({ local: Object.fromEntries(SEG.map(s => [s.name, c.L[s.name].toArray().map(v => +v.toFixed(6))])), root: c.rootQ.toArray().map(v => +v.toFixed(6)) });
  const builders = { dog: buildDog, plank: buildPlank, half: buildHalf, bow: buildBow, fold: buildFold, handstand: buildHandstand };
  const only = process.argv[2];
  for (const id of Object.keys(builders)) {
    if (only && only !== id) continue;
    const t0 = Date.now();
    const c = builders[id](id === 'handstand' && process.env.HS ? JSON.parse(process.env.HS) : undefined);
    out[id] = toData(c);
    const W = placeW(c.L, c.rootQ);
    const m = measure(id, c.L, W);
    const sup = m.support;
    const short = {};
    for (const [k, v] of Object.entries(m)) if (typeof v === 'number') short[k] = +(Math.abs(v) < 1 && !/Pct/.test(k) ? v.toFixed(4) : v.toFixed(1));
    if (process.env.JOINTS) for (const jid of ['gh_L', 'st_L', 'sc_L', 'el_L', 'wr_L', 'hip_L', 'kn_L', 'an_L']) { const j = A.JOINTS.find(x => x.id === jid); if (!j) continue; const r = A.analyzeJoint(j, c.L, W); console.log('  ', jid.padEnd(5), r.comps.map(k => k.key + ' ' + k.signed.toFixed(1) + (k.rom ? '/' + k.rom : '')).join('  ')); }
    console.log(id, (Date.now() - t0) + 'ms', JSON.stringify(c.info || {}), '\n   ', JSON.stringify(short), '\n    floor:', sup.regions.join(', '), sup.split ? 'split arms ' + (sup.split.arms * 100).toFixed(0) + '%' : '');
  }
  // counterfactual for the Горка cue "сводим лопатки, опускаем плечи к тазу"
  if (!only || only === 'dog') {
    const c = buildDog({ scapProt: 4, scapElev: 0 });
    const W = placeW(c.L, c.rootQ);
    const m = measure('dog', c.L, W);
    out.extras = { dog: { clavElevNoCue: +m.clavElev.toFixed(2), retractNoCue: +m.retract.toFixed(2) } };
    const h = buildHalf({ scapElev: 0 });
    const mh = measure('half', h.L, placeW(h.L, h.rootQ));
    out.extras.half = { clavElevNoCue: +mh.clavElev.toFixed(2) };
    console.log('half without the shoulders-down cue: clavElev', mh.clavElev.toFixed(1));
    console.log('dog without the scapula cue: clavElev', m.clavElev.toFixed(1), 'retract', m.retract.toFixed(1));
  }
  fs.writeFileSync(only ? 'poses_dbg.json' : 'poses.json', JSON.stringify(out));

}
