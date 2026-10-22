// Posture search for the handstand: the practitioner "stacks the bones" so that the pose costs the least muscle effort.
// usage: node hs_opt.mjs [rounds] ; prints the best overrides of HSTAND (JSON) and writes hs_opt_best.json
import fs from 'fs';
import { buildHandstand, HSTAND, A } from './poses_build.mjs';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE, POSE_STOPS } from './physics_params.mjs';
const PH = makePhysics(A, { mass: 70 });
const hull = P => { const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]); if (Q.length < 3) return Q; const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); const lo = [], up = []; for (const p of Q) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); } for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); } lo.pop(); up.pop(); return lo.concat(up); };
const margin = (poly, p) => { if (poly.length < 3) return -1; let inside = true, dmin = Infinity; for (let i = 0; i < poly.length; i++) { const a = poly[i], b = poly[(i + 1) % poly.length]; const ex = b[0] - a[0], ez = b[1] - a[1]; if (ex * (p[1] - a[1]) - ez * (p[0] - a[0]) < 0) inside = false; const L2 = ex * ex + ez * ez, t = Math.max(0, Math.min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ez) / (L2 || 1))); dmin = Math.min(dmin, Math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ez)); } return inside ? dmin : -dmin; };

export function evaluate(over, iters = 7000) {
  const c = buildHandstand(over);
  const r = PH.solve({ local: c.L, rootQ: c.rootQ }, { tone: CUE_TONE.handstand, grips: [], stops: POSE_STOPS.handstand, iters });
  const FM = r.FMAX; let num = 0, den = 0;
  r.act.forEach((a, j) => { num += FM[j] * a * a; den += FM[j]; });
  const E = 100 * num / den;
  const res = Object.values(r.reserve).reduce((a, x) => a + Math.abs(x), 0);
  const pts = r.contactF.filter(ct => ct.F.y > 0.002 * r.BW).map(ct => [ct.P.x, ct.P.z]);
  const marg = margin(hull(pts), [r.com.x, r.com.z]);
  const ls = r.jr.L5S1.compression / r.BW;
  const W = r.W, sh = W.hum_L.p, an = W.foot_L.p, wr = W.hand_L.p;
  const lean = Math.abs(an.z - sh.z); // how far the ankles are from the vertical through the shoulders
  const wext = A.analyzeJoint(A.JOINTS.find(j => j.id === 'wr_L'), c.L, W).comps.find(k => k.key === 'ext').signed;
  const J = E + 4 * ls + 0.6 * res + 400 * Math.max(0, 0.03 - marg) + 150 * Math.max(0, lean - 0.09) + 2 * Math.max(0, wext - 96);
  return { J, E, ls, res, marg, lean, wext, info: c.info };
}

const SPACE = {
  thA: [-10, 8, 2], thB: [-10, 8, 2], thC: [-10, 8, 2], lumA: [-8, 10, 2], lumB: [-8, 10, 2],
  hipExt: [-4, 8, 2], armTilt: [-6, 2, 1.5], comAhead: [0.03, 0.07, 0.01],
  scapPost: [-5, 20, 4], scapProt: [-5, 20, 4], scapUp: [46, 60, 3], scapElev: [0, 0.04, 0.008],
};
if (process.argv[1] && process.argv[1].endsWith('hs_opt.mjs')) {
  const rounds = +(process.argv[2] || 2);
  const cur = {}; for (const k of Object.keys(SPACE)) cur[k] = HSTAND[k];
  Object.assign(cur, JSON.parse(process.env.START || '{}'));
  let best = evaluate(cur);
  console.log('start', JSON.stringify(cur), best.J.toFixed(2), 'E', best.E.toFixed(2), 'L5S1', best.ls.toFixed(2), 'res', best.res.toFixed(1), 'marg', (best.marg * 100).toFixed(1), 'lean', (best.lean * 100).toFixed(0), 'ext', best.wext.toFixed(0));
  const t0 = Date.now();
  for (let rd = 0; rd < rounds; rd++) {
    const f = 1 / (rd + 1);
    for (const [k, [lo, hi, st]] of Object.entries(SPACE)) {
      for (const dir of [1, -1]) {
        for (let n = 0; n < 4; n++) {
          const v = +(cur[k] + dir * st * f).toFixed(4);
          if (v < lo || v > hi) break;
          const o = Object.assign({}, cur, { [k]: v });
          const e = evaluate(o);
          if (e.J < best.J - 1e-3) { best = e; cur[k] = v; console.log(`  r${rd} ${k} -> ${v}  J ${e.J.toFixed(2)} E ${e.E.toFixed(2)} L5S1 ${e.ls.toFixed(2)} res ${e.res.toFixed(1)} marg ${(e.marg * 100).toFixed(1)} lean ${(e.lean * 100).toFixed(0)} ext ${e.wext.toFixed(0)}  (${((Date.now() - t0) / 1000).toFixed(0)}s)`); } else break;
        }
      }
    }
  }
  fs.writeFileSync('hs_opt_best.json', JSON.stringify({ params: cur, result: best }, null, 1));
  console.log('best', JSON.stringify(cur));
}
