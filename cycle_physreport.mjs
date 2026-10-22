// Checks and summary of the round's physics (agent phys).
// usage: node cycle_physreport.mjs [--phys cycle_phys.json] [--stats cycle_stats.json] [--hold 15]
//   1. anchors: frames that are exactly a held pose vs physics.json (cold static solution of that pose); the round is solved
//      with warm start + smoothing, so redundant muscles may share a little differently — reported as max |Δa| and where.
//   2. reserves: frames where muscles alone can't hold the posture (Σ|reserve| > 5 N·m), grouped by segment.
//   3. flicker: activation reversals between neighbouring frames (from cycle_stats.json smooth).
//   4. top muscles / ligaments over the round with the page's hold rule.
import fs from 'fs';
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf(k); return i >= 0 && i + 1 < args.length ? args[i + 1] : d; };
const P = JSON.parse(fs.readFileSync(arg('--phys', 'cycle_phys.json'), 'utf8'));
const S = JSON.parse(fs.readFileSync(arg('--stats', 'cycle_stats.json'), 'utf8'));
const PJ = JSON.parse(fs.readFileSync('physics.json', 'utf8'));
const H = +arg('--hold', 15);
const pct = v => (v * 100).toFixed(0) + '%';
console.log(`round ${P.T} s, ${P.frames.length} frames${P.provisional ? ' (PROVISIONAL segments)' : ''}, smooth ${P.opts.smooth}`);
// 1. anchors
for (const h of P.holds) {
  const f = P.frames.find(x => Math.abs(x.T - h.T) < 1e-6);
  const ref = PJ[h.pose];
  if (!f || !ref) { console.log('anchor', h.pose, f ? '' : '(frame not solved)'); continue; }
  let dmax = 0, du = '', dcold = 0;
  P.units.forEach((u, i) => {
    const a = ref.muscles[u] ? ref.muscles[u].a : 0;
    const d = Math.abs(f.aL[i] - a); if (d > dmax) { dmax = d; du = u; }
    if (P.poses[h.pose]) dcold = Math.max(dcold, Math.abs(P.poses[h.pose].aL[i] - a));
  });
  console.log(`anchor ${h.pose.padEnd(6)} T=${String(h.T).padEnd(5)} round frame vs physics.json max|Δa| ${dmax.toFixed(3)} (${du})   cold re-solve vs physics.json ${dcold.toFixed(4)}`);
}
// 2. reserves
const bad = P.frames.filter(f => f.reserves > 5);
const bySeg = {};
for (const f of bad) (bySeg[f.seg] ||= []).push(f);
console.log(`reserves > 5 N·m: ${bad.length} frames` + Object.entries(bySeg).map(([s, l]) => `  ${s}: ${l.length} (max ${Math.max(...l.map(f => f.reserves)).toFixed(1)} at T=${l.reduce((a, b) => (b.reserves > a.reserves ? b : a)).T})`).join(''));
const maxRes = P.frames.reduce((a, b) => (b.resid > a.resid ? b : a));
console.log(`largest residual ${maxRes.resid} at T=${maxRes.T} (${maxRes.seg})`);
const off = P.frames.filter(f => f.margin < 0);
if (off.length) console.log(`COM outside support: ${off.length} frames, e.g. T=${off.slice(0, 5).map(f => f.T).join(', ')}`);
// 3. flicker
if (S.smooth) console.log(`flicker: ${S.smooth.reversals} reversals; worst`, (S.smooth.worst || []).slice(0, 5).map(w => `${w.unit}_${w.side}@${w.T} ${w.jump}`).join(', '));
// 4. top with hold H
const POSES = ['dog', 'plank', 'half', 'bow', 'fold'];
const rows = Object.entries(S.muscles).map(([u, m]) => {
  const sides = ['L', 'R'].map(sd => {
    const tr = m.trans[sd], pv = POSES.map(p => (m.pose[p] ? m.pose[p][sd] : 0));
    return { peak: Math.max(tr.peak, ...pv), int: tr.int + H * pv.reduce((a, b) => a + b, 0), t08: tr.t08 + H * pv.filter(a => a >= 0.08).length };
  });
  return { u, peak: Math.max(sides[0].peak, sides[1].peak), mean: (sides[0].int + sides[1].int) / 2 / (S.T + 5 * H), t08: Math.max(sides[0].t08, sides[1].t08) };
});
const nAct = rows.filter(r => r.peak >= 0.08).length, nStrong = rows.filter(r => r.peak >= 0.4).length;
console.log(`hold ${H} s: ${nAct}/${rows.length} muscles ≥ 8 %, ${nStrong} ≥ 40 %; round with holds ${(S.T + 5 * H).toFixed(1)} s`);
console.log('top by mean:', rows.sort((a, b) => b.mean - a.mean).slice(0, 10).map(r => `${r.u} ${pct(r.mean)} (peak ${pct(r.peak)})`).join(', '));
console.log('top by peak:', [...rows].sort((a, b) => b.peak - a.peak).slice(0, 10).map(r => `${r.u} ${pct(r.peak)}`).join(', '));
const lr = Object.entries(S.ligs).map(([k, l]) => {
  const pk = Math.max(l.trans.L.peak, l.trans.R.peak, ...POSES.map(p => (l.pose[p] ? Math.max(l.pose[p].L, l.pose[p].R) : -1)));
  const tt = Math.max(...['L', 'R'].map(sd => l.trans[sd].t02 + H * POSES.filter(p => l.pose[p] && l.pose[p][sd] >= 0.02).length));
  return { k, pk, tt };
});
console.log('ligaments by peak strain:', lr.sort((a, b) => b.pk - a.pk).slice(0, 10).map(r => `${r.k} ${pct(r.pk)} (${r.tt.toFixed(0)} s taut)`).join(', '));
