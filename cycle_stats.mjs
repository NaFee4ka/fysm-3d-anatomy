// How much every muscle, ligament and joint worked over one round (agent phys).
// usage: node cycle_stats.mjs [--in cycle_phys.json] [--out cycle_stats.json]
//
// Time integrals use the frames present (trapezoid weights over the global time T), so a run with --every N still works.
// Output (cycle_stats.json), keys without Russian (the page translates):
// { T, segs, holds, provisional, thresholds: { act: [0.08, 0.2, 0.4], lig: [0.02, 0.05] },
//   muscles: { unit: { trans: { L: { peak (lasting ≥ 2 frames: max of 3-frame median), peakRaw, peakT, int, t08, t20, t40 }, R: {…} },        // over the transitions; int in activation·s
//                      pose: { dog: { L, R }, plank, half, bow, fold },                  // activation while holding the pose
//                      stretch: { peak, peakT, pose: { dog, … } } } },                   // muscle-tendon stretch vs standing (max of L/R)
//   ligs:    { lig: { trans: { L: { peak, peakT, t02, t05, int }, R: {…} },               // strain vs standing; int = ∫ max(0, strain) dt
//                     pose: { dog: { L, R }, … } } },                                     // ligaments by A.LIGS key, capsules "cap_<joint>"
//   joints:  { L5S1|L5S1shear|T12L1|hip|knee|ankle|gh|elbow|wrist: { trans: { peak, peakT, mean }, pose: { dog, … } } },  // × body weight, max of L/R
//   byHold:  { "0"|"5"|"15"|"30"|"60": { T, active08, active20, active40, ligs02, ligs05, topInt: [unit…], topPeak: [unit…] } },
//   flags:   [{ i, T, seg, reserves, resid }],
//   spikes:  [{ i, T, seg, n, units }],  // frames where muscles jump > 30 % above their 3-frame median (posture glitch in that frame)   // frames the static model can't hold well (large reserve torques / residuals)
//   smooth:  { reversals, worst: [{ unit, side, T, jump }] } }
// Holding H seconds in each of the five poses (what the page does with its hold selector):
//   peak = max(trans.peak, pose values); int = trans.int + H·Σ pose a; time active = t08 + H·#(poses with a ≥ 0.08); mean = int / (T + 5H)
//   for ligaments the pose term is H·Σ max(0, pose strain) and time taut t02 + H·#(poses with strain ≥ 0.02).
import fs from 'fs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf(k); return i >= 0 && i + 1 < args.length ? args[i + 1] : d; };
const IN = arg('--in', 'cycle_phys.json'), OUT = arg('--out', 'cycle_stats.json');
const P = JSON.parse(fs.readFileSync(IN, 'utf8'));
const POSES = ['dog', 'plank', 'half', 'bow', 'fold'];
const ACT = [0.08, 0.2, 0.4], LIG = [0.02, 0.05];
const r3 = v => Math.round(v * 1000) / 1000, r2 = v => Math.round(v * 100) / 100;
const F = [...P.frames].sort((a, b) => a.T - b.T);
const n = F.length;
// trapezoid weights in seconds
const w = F.map((f, i) => ((i < n - 1 ? F[i + 1].T : f.T) - (i > 0 ? F[i - 1].T : f.T)) / 2);
const Ttrans = F[n - 1].T - F[0].T;

// peak = largest value that lasts: maximum of the 3-frame running median (a value present in a single 0.1 s frame only
// is a posture glitch of that frame, not a load the body holds); peakRaw = plain maximum. Integrals and times use raw values.
const SPIKES = new Map();
const med3 = (a, b, c) => Math.max(Math.min(a, b), Math.min(Math.max(a, b), c));
function series(get, tag) {
  let peak = -Infinity, peakT = 0, peakRaw = -Infinity, int = 0;
  const vals = F.map(get);
  vals.forEach((v, i) => {
    const m = n < 3 ? v : med3(vals[Math.max(0, i - 1)], v, vals[Math.min(n - 1, i + 1)]);
    if (m > peak) { peak = m; peakT = F[i].T; }
    if (v > peakRaw) peakRaw = v;
    if (tag && v - m > 0.3) SPIKES.set(F[i].i, (SPIKES.get(F[i].i) || []).concat(tag));
    int += w[i] * v;
  });
  const above = th => vals.reduce((a, v, i) => a + (v >= th ? w[i] : 0), 0);
  return { vals, peak, peakT, peakRaw, int, above };
}
const muscles = {};
P.units.forEach((u, k) => {
  const trans = {};
  for (const sd of ['L', 'R']) {
    const s = series(f => f['a' + sd][k], u + '_' + sd);
    trans[sd] = { peak: r3(s.peak), peakRaw: r3(s.peakRaw), peakT: r2(s.peakT), int: r3(s.int), t08: r2(s.above(ACT[0])), t20: r2(s.above(ACT[1])), t40: r2(s.above(ACT[2])) };
  }
  const pose = {};
  for (const p of POSES) if (P.poses[p]) pose[p] = { L: P.poses[p].aL[k], R: P.poses[p].aR[k] };
  const st = series(f => Math.max(f.sL[k], f.sR[k]));
  const stPose = {};
  for (const p of POSES) if (P.poses[p]) stPose[p] = r3(Math.max(P.poses[p].sL[k], P.poses[p].sR[k]));
  muscles[u] = { trans, pose, stretch: { peak: r3(st.peak), peakT: r2(st.peakT), pose: stPose } };
});
const ligs = {};
P.ligs.forEach((g, k) => {
  const trans = {};
  for (const sd of ['L', 'R']) {
    const s = series(f => f['lig' + sd][k]);
    const pos = series(f => Math.max(0, f['lig' + sd][k]));
    trans[sd] = { peak: r3(s.peak), peakT: r2(s.peakT), t02: r2(s.above(LIG[0])), t05: r2(s.above(LIG[1])), int: r3(pos.int) };
  }
  const pose = {};
  for (const p of POSES) if (P.poses[p]) pose[p] = { L: P.poses[p].ligL[k], R: P.poses[p].ligR[k] };
  ligs[g] = { trans, pose };
});
const JOINTS = { L5S1: f => f.jr.L5S1[0], L5S1shear: f => f.jr.L5S1[1], T12L1: f => f.jr.T12L1[0] };
for (const j of ['hip', 'knee', 'ankle', 'gh', 'elbow', 'wrist']) JOINTS[j] = f => Math.max(f.jr[j][0], f.jr[j][1]);
const joints = {};
for (const [j, get] of Object.entries(JOINTS)) {
  const s = series(get);
  const pose = {};
  for (const p of POSES) if (P.poses[p]) pose[p] = r3(get(P.poses[p]));
  joints[j] = { trans: { peak: r3(s.peak), peakT: r2(s.peakT), mean: r3(s.int / Math.max(1e-9, Ttrans)) }, pose };
}
// headline numbers for the hold times the page offers
const byHold = {};
for (const H of [0, 5, 15, 30, 60]) {
  const has = POSES.filter(p => P.poses[p]);
  let a08 = 0, a20 = 0, a40 = 0, l02 = 0, l05 = 0;
  const intU = [], peakU = [];
  for (const [u, m] of Object.entries(muscles)) {
    const pk = Math.max(m.trans.L.peak, m.trans.R.peak, ...has.map(p => Math.max(m.pose[p].L, m.pose[p].R)));
    if (pk >= ACT[0]) a08++; if (pk >= ACT[1]) a20++; if (pk >= ACT[2]) a40++;
    const int = (m.trans.L.int + m.trans.R.int) / 2 + H * has.reduce((a, p) => a + (m.pose[p].L + m.pose[p].R) / 2, 0);
    intU.push([u, int]); peakU.push([u, pk]);
  }
  for (const [g, l] of Object.entries(ligs)) {
    const pk = Math.max(l.trans.L.peak, l.trans.R.peak, ...has.map(p => Math.max(l.pose[p].L, l.pose[p].R)));
    if (pk >= LIG[0]) l02++; if (pk >= LIG[1]) l05++;
  }
  byHold[H] = { T: r2(Ttrans + has.length * H), active08: a08, active20: a20, active40: a40, ligs02: l02, ligs05: l05,
    topInt: intU.sort((a, b) => b[1] - a[1]).slice(0, 10).map(x => x[0]), topPeak: peakU.sort((a, b) => b[1] - a[1]).slice(0, 10).map(x => x[0]) };
}
// frames the static model can't hold well
const flags = F.filter(f => f.reserves > 5 || f.resid > 1).map(f => ({ i: f.i, T: f.T, seg: f.seg, reserves: f.reserves, resid: f.resid }));
// smoothness: reversals of activation between consecutive frames (both steps > 0.03), largest single jumps
let reversals = 0; const jumps = [];
const consecutive = F.every((f, i) => i === 0 || f.i - F[i - 1].i === 1);
if (consecutive) P.units.forEach((u, k) => { for (const sd of ['L', 'R']) {
  const v = F.map(f => f['a' + sd][k]);
  for (let i = 1; i < n - 1; i++) { const d1 = v[i] - v[i - 1], d2 = v[i + 1] - v[i]; if (Math.abs(d1) > 0.03 && Math.abs(d2) > 0.03 && Math.sign(d1) !== Math.sign(d2)) reversals++; }
  for (let i = 1; i < n; i++) jumps.push({ unit: u, side: sd, T: F[i].T, jump: r3(v[i] - v[i - 1]) });
} });
jumps.sort((a, b) => Math.abs(b.jump) - Math.abs(a.jump));
const spikes = [...SPIKES.entries()].map(([i, us]) => ({ i, T: F.find(f => f.i === i).T, seg: F.find(f => f.i === i).seg, n: us.length, units: us.slice(0, 6) })).sort((a, b) => b.n - a.n);
const out = { spikes, T: r2(Ttrans), segs: P.segs, holds: P.holds, provisional: !!P.provisional, thresholds: { act: ACT, lig: LIG }, muscles, ligs, joints, byHold, flags, smooth: { consecutive, reversals, worst: jumps.slice(0, 12) } };
fs.writeFileSync(OUT, JSON.stringify(out));

// ---------- summary ----------
const top = (obj, f, k = 10) => Object.entries(obj).map(([u, m]) => [u, f(m)]).sort((a, b) => b[1] - a[1]).slice(0, k);
const pct = v => Math.round(v * 100) + '%';
console.log(`${OUT}: ${n} frames, transitions ${r2(Ttrans)} s${P.provisional ? ' (PROVISIONAL frames)' : ''}`);
console.log('muscles by peak in transitions:', top(muscles, m => Math.max(m.trans.L.peak, m.trans.R.peak)).map(([u, v]) => `${u} ${pct(v)}`).join(', '));
console.log('muscles by integral in transitions (a·s, mean of L/R):', top(muscles, m => (m.trans.L.int + m.trans.R.int) / 2).map(([u, v]) => `${u} ${v.toFixed(2)}`).join(', '));
console.log('ligaments by peak strain:', top(ligs, l => Math.max(l.trans.L.peak, l.trans.R.peak)).map(([u, v]) => `${u} ${(v * 100).toFixed(1)}%`).join(', '));
console.log('ligaments by time taut ≥2% (s, max L/R):', top(ligs, l => Math.max(l.trans.L.t02, l.trans.R.t02)).map(([u, v]) => `${u} ${v.toFixed(1)}`).join(', '));
for (const H of [0, 15]) console.log(`hold ${H}s: round ${byHold[H].T}s, muscles ≥8% ${byHold[H].active08}/${P.units.length}, ≥20% ${byHold[H].active20}, ≥40% ${byHold[H].active40}; ligaments ≥2% ${byHold[H].ligs02}/${P.ligs.length}, ≥5% ${byHold[H].ligs05}; most work: ${byHold[H].topInt.slice(0, 5).join(', ')}`);
console.log('joints peak (×BW):', Object.entries(joints).map(([j, v]) => `${j} ${v.trans.peak} @${v.trans.peakT}s`).join(', '));
console.log('flagged frames:', flags.length, flags.slice(0, 8).map(f => `#${f.i} ${f.seg} T=${f.T} res ${f.reserves}N·m resid ${f.resid}`).join('; '));
console.log('single-frame spikes (> 30 % above the 3-frame median):', spikes.length, 'frames;', spikes.slice(0, 8).map(x => `#${x.i} ${x.seg} T=${x.T} ${x.n} units (${x.units.slice(0, 3).join(',')})`).join('; '));
console.log('smoothness:', consecutive ? `${reversals} reversals; largest jumps ` + jumps.slice(0, 5).map(j => `${j.unit}_${j.side} ${j.jump > 0 ? '+' : ''}${pct(j.jump)} @${j.T}s`).join(', ') : 'frames not consecutive (--every > 1): not measured');
// pose values against physics.json (left side)
if (fs.existsSync('physics.json')) {
  const PJ = JSON.parse(fs.readFileSync('physics.json', 'utf8'));
  let d = 0, wu = '';
  for (const p of POSES) if (P.poses[p] && PJ[p]) P.units.forEach((u, k) => { const e = Math.abs(P.poses[p].aL[k] - PJ[p].muscles[u].a); if (e > d) { d = e; wu = p + ' ' + u; } });
  console.log('pose activations vs physics.json: max |Δ|', d.toFixed(4), wu);
}
