// seg5 rig: a configuration from a compact set of controls (pelvis, spine, arms to targets, legs to targets),
// with an exact arm reach (keeps the carrying angle of the forearm) and feet planted on tucked toes, flat, or swinging.
import fs from 'fs';
import { A, V, THREE, S, SEG, qAx, X, Y, Z, computeWorld, pt, mirrorQ, mirP, LANDMARKS, D, IDQ, comOf, contactsOf, asLeft, legIK, orientHand, setSide, rootPlaced, cloneL } from './cycle_lib.mjs';
import { setLR, bisect, fingers, SPREAD, GRIP, blendFingers, handBasis, gripHand, wristFor, fromData, toData, handLmL } from './seg5_util.mjs';

export const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
export const smooth = u => { u = clamp(u, 0, 1); return u * u * (3 - 2 * u); };
export const smoother = u => { u = clamp(u, 0, 1); return u * u * u * (u * (u * 6 - 15) + 10); };
export const seg01 = (t, t0, t1) => clamp((t - t0) / (t1 - t0), 0, 1);
export const lerp = (a, b, u) => a + (b - a) * u;
export const lerpV = (a, b, u) => a.clone().lerp(b, u);
export const slerpQ = (a, b, u) => new THREE.Quaternion().slerpQuaternions(a, b, u);

// ---------- monotone cubic (pchip) through keys [[t, value], ...] for scalars or arrays ----------
export function pchip(keys) {
  const n = keys.length, ts = keys.map(k => k[0]);
  const dim = Array.isArray(keys[0][1]) ? keys[0][1].length : 1;
  const vals = keys.map(k => (dim === 1 && !Array.isArray(k[1]) ? [k[1]] : k[1]));
  const ms = [];
  for (let d = 0; d < dim; d++) {
    const y = vals.map(v => v[d]);
    const h = [], del = [];
    for (let i = 0; i < n - 1; i++) { h.push(ts[i + 1] - ts[i]); del.push((y[i + 1] - y[i]) / h[i]); }
    const m = new Array(n).fill(0);
    for (let i = 1; i < n - 1; i++) {
      if (del[i - 1] * del[i] <= 0) m[i] = 0;
      else { const w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1]; m[i] = (w1 + w2) / (w1 / del[i - 1] + w2 / del[i]); }
    }
    ms.push(m);
  }
  return t => {
    let i = 0;
    if (t <= ts[0]) return dim === 1 && !Array.isArray(keys[0][1]) ? vals[0][0] : vals[0].slice();
    if (t >= ts[n - 1]) return dim === 1 && !Array.isArray(keys[0][1]) ? vals[n - 1][0] : vals[n - 1].slice();
    while (t > ts[i + 1]) i++;
    const hh = ts[i + 1] - ts[i], s = (t - ts[i]) / hh;
    const h00 = 2 * s ** 3 - 3 * s ** 2 + 1, h10 = s ** 3 - 2 * s ** 2 + s, h01 = -2 * s ** 3 + 3 * s ** 2, h11 = s ** 3 - s ** 2;
    const out = [];
    for (let d = 0; d < dim; d++) out.push(h00 * vals[i][d] + h10 * hh * ms[d][i] + h01 * vals[i + 1][d] + h11 * hh * ms[d][i + 1]);
    return dim === 1 && !Array.isArray(keys[0][1]) ? out[0] : out;
  };
}

// ---------- exact arm reach ----------
const LHv = S.fore_L.J.clone().sub(S.hum_L.J), FV = S.hand_L.J.clone().sub(S.fore_L.J);
const wLoc = th => LHv.clone().add(FV.clone().applyQuaternion(qAx(X, -th)));
export const REACH0 = wLoc(0).length();
const thetaFor = dist => (dist >= REACH0 ? 0 : bisect(th => wLoc(th).length() - dist, 0, 170, 50));
export function armExact(L, rootQ, rootP, side, wristT, poleP) {
  const W = asLeft(computeWorld(L, rootQ, rootP), side);
  const T0 = side === 'L' ? wristT.clone() : mirP(wristT), P0 = side === 'L' ? poleP.clone() : mirP(poleP);
  const Sh = W.hum_L.p.clone();
  const Tv = T0.clone().sub(Sh), dist = Tv.length();
  const th = thetaFor(dist);
  const w = wLoc(th), a = w.clone().normalize();
  const ep = LHv.clone().addScaledVector(a, -LHv.dot(a)).normalize();
  const c = a.clone().cross(ep);
  const Aw = Tv.clone().normalize();
  const Pw = P0.clone().sub(Sh); Pw.addScaledVector(Aw, -Pw.dot(Aw)); Pw.normalize();
  const Cw = Aw.clone().cross(Pw);
  const M = new THREE.Matrix4().makeBasis(Aw, Pw, Cw).multiply(new THREE.Matrix4().makeBasis(a, ep, c).transpose());
  const Q = new THREE.Quaternion().setFromRotationMatrix(M);
  setSide(L, side, 'hum', W.scap_L.q.clone().invert().multiply(Q));
  setSide(L, side, 'fore', qAx(X, -th)); setSide(L, side, 'ulna', IDQ);
  return { theta: th, short: Math.max(0, dist - REACH0), flexQ: qAx(X, -th) };
}

// ---------- feet ----------
export function setFootWorld(L, rootQ, rootP, side, Qf) { const W = computeWorld(L, rootQ, rootP); L['foot_' + side] = W['shank_' + side].q.clone().invert().multiply(Qf); }
export function setToesWorld(L, rootQ, rootP, side, Qt) { const W = computeWorld(L, rootQ, rootP); L['toes_' + side] = W['foot_' + side].q.clone().invert().multiply(Qt); }
const ANK_TO_MTP = S.toes_L.J.clone().sub(S.foot_L.J); // rest vector, left
export const mtpOffset = side => (side === 'L' ? ANK_TO_MTP.clone() : mirP(ANK_TO_MTP));
// ankle position for a foot orientation with the MTP joint at a fixed point
export const ankleFromMTP = (mtp, Qf, side) => mtp.clone().sub(mtpOffset(side).applyQuaternion(Qf));

// ---------- the whole body from controls ----------
// c = { rootP: V, rootQ: Q, spine: {lumA,..,head: Q}, clav: {L,R}, scap: {L,R},
//       arm: {L: {wrist: V, pole: V, handQ: Q, fingers: {...}}, R: ...},
//       leg: {L: {ankle: V, pole: V, footQ: Q, toesQ: Q (world)}, R: ...} }
export function solveBody(c) {
  const L = {};
  for (const s of SEG) L[s.name] = new THREE.Quaternion();
  for (const [k, q] of Object.entries(c.spine)) L[k] = q.clone();
  for (const sd of ['L', 'R']) { L['clav_' + sd] = c.clav[sd].clone(); L['scap_' + sd] = c.scap[sd].clone(); }
  const info = {};
  for (const sd of ['L', 'R']) {
    const a = c.arm[sd];
    const left = {}; fingers(left, a.fingers);
    for (const k of ['f1m', 'f1a', 'f1b', 'f2a', 'f2b', 'f2c', 'f3a', 'f3b', 'f3c', 'f4a', 'f4b', 'f4c', 'f5a', 'f5b', 'f5c']) setSide(L, sd, k, left[k]);
    const r = armExact(L, c.rootQ, c.rootP, sd, a.wrist, a.pole);
    if (a.local) { // forearm rotation and wrist given directly (smooth hand travel between two known grips)
      setSide(L, sd, 'fore', r.flexQ.clone().multiply(a.local.twist));
      setSide(L, sd, 'ulna', a.local.twist.clone().invert());
      setSide(L, sd, 'hand', a.local.swing);
    } else orientHand(L, c.rootQ, c.rootP, sd, a.handQ, r.flexQ);
    info['arm' + sd] = r;
  }
  for (const sd of ['L', 'R']) {
    const g = c.leg[sd];
    legIK(L, c.rootQ, c.rootP, sd, g.ankle, g.pole);
    if (g.footLocal) {
      L['foot_' + sd] = g.footLocal.clone(); L['toes_' + sd] = g.toesLocal.clone();
      if (g.blend > 0) { // towards the landing orientation in the world
        const W1 = computeWorld(L, c.rootQ, c.rootP);
        setFootWorld(L, c.rootQ, c.rootP, sd, W1['foot_' + sd].q.clone().slerp(g.footQ, g.blend));
        const W2 = computeWorld(L, c.rootQ, c.rootP);
        setToesWorld(L, c.rootQ, c.rootP, sd, W2['toes_' + sd].q.clone().slerp(g.toesQ, g.blend));
      }
    }
    else { setFootWorld(L, c.rootQ, c.rootP, sd, g.footQ); setToesWorld(L, c.rootQ, c.rootP, sd, g.toesQ); }
    if (g.anMax) {
      const hingeAn = () => { const W = computeWorld(L, c.rootQ, c.rootP); return W['foot_' + sd].p.clone().sub(W['shank_' + sd].p).angleTo(W['toes_' + sd].p.clone().sub(W['foot_' + sd].p)) / D; };
      const base = L['foot_' + sd].clone(), rot = a => { L['foot_' + sd] = base.clone().multiply(sd === 'L' ? qAx(X, a) : mirrorQ(qAx(X, a))); return hingeAn(); };
      if (rot(0) > g.anMax) {
        const sgn = rot(10) < rot(-10) ? 1 : -1;
        const a = bisect(x => rot(sgn * x) - g.anMax, 0, 70, 40);
        rot(sgn * a);
      } else rot(0);
    }
    const W = computeWorld(L, c.rootQ, c.rootP);
    info['leg' + sd] = { err: W['foot_' + sd].p.distanceTo(g.ankle) };
  }
  return { L, rootQ: c.rootQ.clone(), rootP: c.rootP.clone(), info };
}

// ---------- measurements ----------
export function hull(P) {
  const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (Q.length < 3) return Q;
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Q) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
export function margin(poly, p) {
  if (poly.length < 3) return -1;
  let inside = true, dmin = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const ex = b[0] - a[0], ez = b[1] - a[1];
    if (ex * (p[1] - a[1]) - ez * (p[0] - a[0]) < 0) inside = false;
    const L2 = ex * ex + ez * ez, t = Math.max(0, Math.min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ez) / (L2 || 1)));
    dmin = Math.min(dmin, Math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ez));
  }
  return inside ? dmin : -dmin;
}
export function measureConf(cf) {
  const W = computeWorld(cf.L, cf.rootQ, cf.rootP);
  const P = LANDMARKS.map(l => pt(W, l.seg, l.pc));
  let minY = Infinity, minL = null;
  P.forEach((p, k) => { if (p.y < minY) { minY = p.y; minL = LANDMARKS[k]; } });
  const inC = P.map(p => p.y < 0.004);
  const poly = hull(P.filter((p, k) => inC[k]).map(p => [p.x, p.z]));
  const com = comOf(W);
  const regs = [...new Set(LANDMARKS.filter((l, k) => inC[k]).map(l => l.region + ':' + l.side))];
  const joints = {};
  for (const j of A.JOINTS) {
    if (!/^(hip|kn|an|mtp|el|wr)_/.test(j.id)) continue;
    const r = A.analyzeJoint(j, cf.L, W);
    joints[j.id] = r.comps.map(c => `${c.key}${c.signed >= 0 ? '' : '-'}${c.value.toFixed(0)}/${c.rom}`).join(' ');
  }
  return { W, minY, minL: minL.region + (minL.tag ? ':' + minL.tag : '') + ':' + minL.side, com, margin: margin(poly, [com.x, com.z]), regs, joints };
}
