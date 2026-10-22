// seg5 helpers: copies of the pose-building tools from poses_build.mjs (that file runs on import), left side + mirror.
import { A, V, THREE, S, SEG, qAx, X, Y, Z, computeWorld, pt, mirrorQ, LANDMARKS, D, AZ, IDQ, sideName, comOf } from './cycle_lib.mjs';

export const setLR = (L, left) => { for (const k in left) { if (k.startsWith('_')) continue; L[k + '_L'] = left[k].clone(); L[k + '_R'] = mirrorQ(left[k]); } };
export function bisect(f, a, b, n = 60) {
  let fa = f(a), fb = f(b);
  if (fa === 0) return a;
  if (fb === 0) return b;
  if ((fa > 0) === (fb > 0)) return Math.abs(fa) < Math.abs(fb) ? a : b;
  for (let i = 0; i < n; i++) { const m = (a + b) / 2, fm = f(m); if ((fm > 0) === (fa > 0)) { a = m; fa = fm; } else { b = m; fb = fm; } }
  return (a + b) / 2;
}
export const LP = (base, x, y, z) => ({ seg: sideName(base, 'L'), p: V(...AZ(base, [x, y, z])) });
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
export const SPREAD = { spread: [0, 14, 0, 10, 20], flex: [[7.5, 0, 0], [5.7, 0, 0], [4.8, 0, 0], [2.8, 0, 0]], thumb: { abd: 42 } };
export const GRIP = { flex: [[45, 55, 30], [48, 58, 32], [50, 60, 34], [52, 62, 35]], thumb: { abd: 35, flex: 25, mcp: 15, ip: 10 } };
export const SOFT = { spread: [0, 4, 0, 3, 6], flex: [[15, 22, 12]], thumb: { abd: 18, flex: 5, mcp: 5, ip: 5 } };
// finger sets blended joint by joint (for the hand closing round the shin)
export function blendFingers(a, b, u) {
  const out = {};
  const sa = a.spread || [0, 0, 0, 0, 0], sb = b.spread || [0, 0, 0, 0, 0];
  out.spread = sa.map((v, i) => v + (sb[i] - v) * u);
  out.flex = [0, 1, 2, 3].map(i => { const fa = (a.flex[i] || a.flex[0]), fb = (b.flex[i] || b.flex[0]); return fa.map((v, k) => v + (fb[k] - v) * u); });
  const ta = a.thumb || {}, tb = b.thumb || {};
  out.thumb = {}; for (const k of ['abd', 'flex', 'mcp', 'ip']) out.thumb[k] = (ta[k] || 0) + ((tb[k] || 0) - (ta[k] || 0)) * u;
  return out;
}
export function handBasis(fingersDir, palmNormal) {
  const g = fingersDir.clone().normalize();
  const n = palmNormal.clone(); n.addScaledVector(g, -n.dot(g)); n.normalize();
  const Xw = n.clone().negate(), Yw = g.clone().negate(), Zw = Xw.clone().cross(Yw).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(Xw, Yw, Zw));
}
export function gripHand(f, n) { const g = f.clone().addScaledVector(n, -f.dot(n)).normalize(); return handBasis(g, n); }
export const PALM_OFF = V(0.196 - 0.212, 0.75 - 0.815, 0.008 + 0.022);
export const wristFor = (C, Qh) => C.clone().sub(PALM_OFF.clone().applyQuaternion(Qh));
export function orientHandL(L, left, rootQ, rootP, target, flexQ) {
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
export const blankLocal = () => { const L = {}; for (const s of SEG) L[s.name] = new THREE.Quaternion(); return L; };
export const toData = (L, rootQ, rootP) => ({ local: Object.fromEntries(SEG.map(s => [s.name, L[s.name].toArray().map(v => +v.toFixed(6))])), root: rootQ.toArray().map(v => +v.toFixed(6)), rootP: [rootP.x, rootP.y, rootP.z].map(v => +v.toFixed(6)) });
export const fromData = d => { const L = {}; for (const [s, v] of Object.entries(d.local)) L[s] = new THREE.Quaternion(...v); return { L, rootQ: new THREE.Quaternion(...d.root), rootP: V(...d.rootP) }; };
export const handLmL = LANDMARKS.filter(l => l.side === 'L' && /^(hand|f\d[a-c]|f1m)_L$/.test(l.seg));
export const lmTag = (W, tag, side) => { const l = LANDMARKS.find(x => x.tag === tag && x.side === side); return pt(W, l.seg, l.pc); };
