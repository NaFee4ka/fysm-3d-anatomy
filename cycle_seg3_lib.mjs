// seg3 helpers (copies of the pose-builder tools, kept local so cycle_lib.mjs stays frozen)
import * as C from './cycle_lib.mjs';
const { A, V, THREE, S, SEG, computeWorld, pt, LANDMARKS, comOf, qAx, X, Y, Z, D, IDQ, scapulaPose, mirrorQ, LH, LF } = C;
export * from './cycle_lib.mjs';
export const blank = () => { const L = {}; for (const s of SEG) L[s.name] = new THREE.Quaternion(); return L; };
export const setLR = (L, left) => { for (const k in left) { if (k.startsWith('_')) continue; L[k + '_L'] = left[k].clone(); L[k + '_R'] = mirrorQ(left[k]); } };
export function bisect(f, a, b, n = 60) {
  let fa = f(a), fb = f(b);
  if (fa === 0) return a;
  if (fb === 0) return b;
  if ((fa > 0) === (fb > 0)) return Math.abs(fa) < Math.abs(fb) ? a : b;
  for (let i = 0; i < n; i++) { const m = (a + b) / 2, fm = f(m); if ((fm > 0) === (fa > 0)) { a = m; fa = fm; } else { b = m; fb = fm; } }
  return (a + b) / 2;
}
export const LP = (base, x, y, z) => ({ seg: A.sideName(base, 'L'), p: V(...A.AZ(base, [x, y, z])) });
export const wpt = (W, q) => pt(W, q.seg, q.p);
export function fingers(left, o) {
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
export const SOFT = { spread: [0, 4, 0, 3, 6], flex: [[15, 22, 12]], thumb: { abd: 18, flex: 5, mcp: 5, ip: 5 } };
export const GRIP = { flex: [[45, 55, 30], [48, 58, 32], [50, 60, 34], [52, 62, 35]], thumb: { abd: 35, flex: 25, mcp: 15, ip: 10 } };
export const OPEN = { spread: [0, 6, 0, 5, 9], flex: [[8, 10, 6]], thumb: { abd: 30, flex: 8, mcp: 5, ip: 5 } };
// finger shapes blended parameter by parameter (both shapes expanded to 4 fingers)
export function blendHand(a, b, u) {
  const ex = h => ({ spread: (h.spread || [0, 0, 0, 0, 0]).slice(), flex: [0, 1, 2, 3].map(i => (h.flex[i] || h.flex[0]).slice()), thumb: { abd: 0, flex: 0, mcp: 0, ip: 0, ...(h.thumb || {}) } });
  const A1 = ex(a), B1 = ex(b), m = (x, y) => x + (y - x) * u;
  return { spread: A1.spread.map((v, i) => m(v, B1.spread[i])), flex: A1.flex.map((f, i) => f.map((v, k) => m(v, B1.flex[i][k]))), thumb: Object.fromEntries(Object.keys(A1.thumb).map(k => [k, m(A1.thumb[k], B1.thumb[k] || 0)])) };
}
export function handBasis(fingersDir, palmNormal) {
  const g = fingersDir.clone().normalize();
  const n = palmNormal.clone(); n.addScaledVector(g, -n.dot(g)); n.normalize();
  const Xw = n.clone().negate(), Yw = g.clone().negate(), Zw = Xw.clone().cross(Yw).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(Xw, Yw, Zw));
}
export function gripHand(f, n) { const g = f.clone().addScaledVector(n, -f.dot(n)).normalize(); return handBasis(g, n); }
export const PALM_OFF = V(0.196 - 0.212, 0.75 - 0.815, 0.008 + 0.022);
export const wristFor = (Cc, Qh) => Cc.clone().sub(PALM_OFF.clone().applyQuaternion(Qh));
function hull2(P) {
  const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Q) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
// resting pitch on the floor (as the pose builder does): the lower hull edge under the centre of mass
export function settle(L, phi0) {
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
