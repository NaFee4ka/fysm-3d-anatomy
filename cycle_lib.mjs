// Tools for one round of the flow: keyframes in world placement, two-bone reaches for arms and legs on either
// side, the trunk fitted to the supports, and in-between frames that keep planted hands and feet on the floor.
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';

export const A = createAnatomy(THREE);
export const { V, S, SEG, qAx, AX, computeWorld, pt, mirrorQ, scapulaPose, comOf, LANDMARKS, D, AZ, IDQ, sideName } = A;
export const X = AX.X, Y = AX.Y, Z = AX.Z;
export { THREE };

export const cloneL = L => { const o = {}; for (const k in L) o[k] = L[k].clone(); return o; };
export const mirP = p => V(-p.x, p.y, p.z);
export const ease = u => u * u * (3 - 2 * u);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
export const blankLocal = () => { const L = {}; for (const s of SEG) L[s.name] = new THREE.Quaternion(); return L; };
export const W_of = k => computeWorld(k.L, k.rootQ, k.rootP);
export const setSide = (L, side, base, q) => { L[base + '_' + side] = side === 'L' ? q.clone() : mirrorQ(q); };
export const getSideL = (L, side, base) => (side === 'L' ? L[base + '_L'].clone() : mirrorQ(L[base + '_R']));

// placement used by the page for a single pose: lowest point on the floor, centred along the mat
export function rootPlaced(L, rootQ) {
  const W0 = computeWorld(L, rootQ, V(0, 0, 0));
  let minY = Infinity, zmin = Infinity, zmax = -Infinity;
  for (const l of LANDMARKS) { const p = pt(W0, l.seg, l.pc); minY = Math.min(minY, p.y); zmin = Math.min(zmin, p.z); zmax = Math.max(zmax, p.z); }
  return V(0, -minY, -(zmin + zmax) / 2);
}

// the right side seen as a left side (mirror x): lets the left-side solvers work for both
export function asLeft(W, side) {
  if (side === 'L') return W;
  const out = {};
  for (const k in W) {
    const m = k.match(/^(.*)_(L|R)$/);
    const src = m ? m[1] + '_' + (m[2] === 'L' ? 'R' : 'L') : k;
    out[k] = { p: mirP(W[src].p), q: mirrorQ(W[src].q) };
  }
  return out;
}
const mQ = (q, side) => (side === 'L' ? q.clone() : mirrorQ(q));
const mV = (v, side) => (v ? (side === 'L' ? v.clone() : mirP(v)) : v);

// ---------- arm ----------
export const LH = S.fore_L.J.distanceTo(S.hum_L.J), LF = S.hand_L.J.distanceTo(S.fore_L.J);
const N_HUM = S.fore_L.J.clone().sub(S.hum_L.J).normalize();
function basisQ(y, z) { const x = y.clone().cross(z).normalize(); const zz = x.clone().cross(y).normalize(); return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, zz)); }
const R0 = basisQ(N_HUM.clone().negate(), V(0, 0, 1));
// wrist of `side` to wristT, elbow towards pole; creaseDir keeps the roll of a nearly straight arm
export function armIK(L, rootQ, rootP, side, wristT, pole, creaseDir) {
  const T0 = mV(wristT, side), P0 = mV(pole, side), C0 = mV(creaseDir, side);
  const T = T0.clone();
  let flex = 0;
  for (let it = 0; it < 10; it++) {
    const W = asLeft(computeWorld(L, rootQ, rootP), side);
    const Sh = W.hum_L.p.clone();
    const c = Math.min(T.distanceTo(Sh), LH + LF - 1e-5);
    const e = T.clone().sub(Sh).normalize();
    const cosA = clamp((LH * LH + c * c - LF * LF) / (2 * LH * c), -1, 1);
    let perp = P0.clone().sub(Sh); perp.addScaledVector(e, -perp.dot(e));
    if (perp.lengthSq() < 1e-12) perp = V(0, 0, 1);
    perp.normalize();
    const E = Sh.clone().addScaledVector(e, LH * cosA).addScaledVector(perp, LH * Math.sqrt(Math.max(0, 1 - cosA * cosA)));
    const d = E.clone().sub(Sh).normalize();
    const f = T.clone().sub(E).normalize();
    flex = Math.acos(clamp(d.dot(f), -1, 1)) / D;
    let zt = C0 && flex < 3 ? C0.clone() : f.clone();
    zt.addScaledVector(d, -zt.dot(d));
    if (zt.lengthSq() < 1e-10) zt = perp.clone();
    zt.normalize();
    const Rw = basisQ(d.clone().negate(), zt).multiply(R0.clone().invert());
    setSide(L, side, 'hum', W.scap_L.q.clone().invert().multiply(Rw));
    setSide(L, side, 'fore', qAx(X, -flex)); setSide(L, side, 'ulna', IDQ);
    const W2 = asLeft(computeWorld(L, rootQ, rootP), side);
    const err = T0.clone().sub(W2.hand_L.p);
    if (err.length() < 2e-4) break;
    T.add(err);
  }
  return qAx(X, -flex);
}
// hand to a world orientation: forearm rotation round its own axis, the rest at the wrist
export function orientHand(L, rootQ, rootP, side, handQ, flexQ) {
  setSide(L, side, 'fore', flexQ); setSide(L, side, 'ulna', IDQ); setSide(L, side, 'hand', IDQ);
  const W = asLeft(computeWorld(L, rootQ, rootP), side);
  const delta = W.fore_L.q.clone().invert().multiply(mQ(handQ, side));
  const dAx = S.hand_L.J.clone().sub(S.fore_L.J).normalize();
  const vv = V(delta.x, delta.y, delta.z);
  const pr = dAx.clone().multiplyScalar(vv.dot(dAx));
  let twist = new THREE.Quaternion(pr.x, pr.y, pr.z, delta.w);
  if (twist.lengthSq() < 1e-12) twist = IDQ.clone();
  twist.normalize();
  const swing = twist.clone().invert().multiply(delta);
  setSide(L, side, 'fore', flexQ.clone().multiply(twist));
  setSide(L, side, 'ulna', twist.clone().invert());
  setSide(L, side, 'hand', swing);
}

// ---------- leg ----------
export const LT = S.shank_L.J.distanceTo(S.thigh_L.J), LS = S.foot_L.J.distanceTo(S.shank_L.J);
const A0 = S.shank_L.J.clone().sub(S.thigh_L.J).normalize();
const X0 = V(1, 0, 0).addScaledVector(A0, -A0.x).normalize();
const M0q = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(X0, A0, X0.clone().cross(A0)));
// ankle of `side` to ankleT, knee towards pole
export function legIK(L, rootQ, rootP, side, ankleT, pole) {
  const T0 = mV(ankleT, side), P0 = mV(pole, side);
  const T = T0.clone();
  for (let it = 0; it < 10; it++) {
    const W = asLeft(computeWorld(L, rootQ, rootP), side);
    const H = W.thigh_L.p.clone();
    const c = Math.min(T.distanceTo(H), LT + LS - 1e-5);
    const e = T.clone().sub(H).normalize();
    const cosA = clamp((LT * LT + c * c - LS * LS) / (2 * LT * c), -1, 1);
    let perp = P0.clone().sub(H); perp.addScaledVector(e, -perp.dot(e));
    if (perp.lengthSq() < 1e-12) perp = V(0, 0, 1);
    perp.normalize();
    const K = H.clone().addScaledVector(e, LT * cosA).addScaledVector(perp, LT * Math.sqrt(Math.max(0, 1 - cosA * cosA)));
    const d = K.clone().sub(H).normalize();
    const f = T.clone().sub(K).normalize();
    let n = d.clone().cross(f);
    if (n.lengthSq() < 1e-8) n = d.clone().cross(perp.clone().negate());
    n.normalize();
    const flex = Math.acos(clamp(d.dot(f), -1, 1)) / D;
    const M1 = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(n, d, n.clone().cross(d)));
    const Rw = M1.multiply(M0q.clone().invert());
    setSide(L, side, 'thigh', W.pelvis.q.clone().invert().multiply(Rw));
    setSide(L, side, 'shank', qAx(X, flex));
    const W2 = asLeft(computeWorld(L, rootQ, rootP), side);
    const err = T0.clone().sub(W2.foot_L.p);
    if (err.length() < 2e-4) break;
    T.add(err);
  }
}
export function orientFoot(L, rootQ, rootP, side, footQ) {
  const W = asLeft(computeWorld(L, rootQ, rootP), side);
  setSide(L, side, 'foot', W.shank_L.q.clone().invert().multiply(mQ(footQ, side)));
}

// ---------- trunk fitted to supports: pitch about X and a shift in the sagittal plane ----------
export function fitRoot(rootQ, rootP, bodyPts, targets, weights) {
  let wsum = 0; const cb = V(0, 0, 0), ct = V(0, 0, 0);
  bodyPts.forEach((b, i) => { const w = weights ? weights[i] : 1; cb.addScaledVector(b, w); ct.addScaledVector(targets[i], w); wsum += w; });
  cb.multiplyScalar(1 / wsum); ct.multiplyScalar(1 / wsum);
  let sxx = 0, sxy = 0;
  bodyPts.forEach((b, i) => {
    const w = weights ? weights[i] : 1;
    const u = b.z - cb.z, v = b.y - cb.y, ut = targets[i].z - ct.z, vt = targets[i].y - ct.y;
    sxx += w * (u * ut + v * vt); sxy += w * (u * vt - v * ut);
  });
  const phi = bodyPts.length > 1 ? Math.atan2(sxy, sxx) : 0;
  const R = new THREE.Quaternion().setFromAxisAngle(V(1, 0, 0), -phi);
  const nq = R.clone().multiply(rootQ);
  const np = rootP.clone().sub(cb).applyQuaternion(R).add(ct);
  np.x = rootP.x;
  return { rootQ: nq, rootP: np };
}

// ---------- contacts of a configuration ----------
export const HAND_REG = new Set(['ладони', 'пальцы рук', 'кулаки', 'кисти', 'предплечья', 'локти']);
export const FOOT_REG = new Set(['пятки', 'подушечки стоп', 'наружный край стопы', 'пальцы стоп', 'тыл стоп']);
export function contactsOf(W, thr = 0.004) {
  return LANDMARKS.map(l => ({ l, p: pt(W, l.seg, l.pc) })).filter(c => c.p.y < thr);
}
export function effectors(k) {
  const W = W_of(k);
  const cs = contactsOf(W);
  const eff = {};
  for (const sd of ['L', 'R']) {
    const hc = cs.filter(c => c.l.side === sd && HAND_REG.has(c.l.region));
    const fc = cs.filter(c => c.l.side === sd && FOOT_REG.has(c.l.region));
    eff['hand_' + sd] = { planted: hc.length > 0, forearm: hc.some(c => c.l.region === 'предплечья' || c.l.region === 'локти'), wrist: W['hand_' + sd].p.clone(), elbow: W['fore_' + sd].p.clone(), q: W['hand_' + sd].q.clone() };
    eff['foot_' + sd] = { planted: fc.length > 0, ankle: W['foot_' + sd].p.clone(), q: W['foot_' + sd].q.clone(), toes: k.L['toes_' + sd].clone() };
  }
  eff.trunk = cs.filter(c => !HAND_REG.has(c.l.region) && !FOOT_REG.has(c.l.region)).map(c => ({ l: c.l, p: c.p.clone() }));
  return eff;
}
export function summary(k) {
  const W = W_of(k);
  let minY = Infinity, minL = null;
  for (const l of LANDMARKS) { const p = pt(W, l.seg, l.pc); if (p.y < minY) { minY = p.y; minL = l; } }
  const cs = contactsOf(W);
  return { minY, minL: minL.region + (minL.tag ? ':' + minL.tag : '') + ':' + minL.side, contacts: [...new Set(cs.map(c => c.l.region + ':' + c.l.side))].join(', '), com: comOf(W) };
}
