// Quasi-static physics of every frame of the round (agent phys).
// Each frame is solved as a static pose at its own place on the mat (slow yoga transitions: inertia is neglected).
//
// usage: node cycle_phys.mjs [--in cycle_keys/cycle.json] [--out cycle_phys.json] [--every N] [--from i] [--to j]
//                            [--seg segN] [--merge] [--smooth 0.15] [--iters 16000] [--no-poses] [--quiet]
//   --every N      solve every N-th frame (and the last one); the stats integrate over the frames present
//   --from/--to    frame indices (inclusive) of cycle.json; --seg limits to one segment
//   --merge        put the solved frames into the existing --out file (frames matched by index), keep the rest
//   --smooth w     extra cost w·Fmax/1000·(a − a_prev)²/2 per strand towards the previous solved frame (0 = off);
//                  only the sharing between redundant muscles changes, equilibrium stays exact
//   heavy: run as  flock ./.heavy.lock node cycle_phys.mjs …
//
// Cue tone during a transition (minimum activation of the muscles a cue switches on, CUE_TONE in physics_params.mjs):
//   A = tone of the pose the segment leaves, B = tone of the pose it enters, S = min(A, B) per muscle (cues both poses share);
//   during the FIRST phase of the segment A's own cues fade to S, during the LAST phase B's own cues rise from S;
//   tone = S + fA·(A − S) + fB·(B − S), fA = 1 − ease(t / end of first phase), fB = ease((t − start of last phase) / its length).
//   A frame's own "tone" (written by a segment) is merged with max(). The high plank (not a held pose) uses the forearm
//   plank's straight-line cues without the fist: quadriceps 0.15, rectus abdominis 0.15, gluteus maximus 0.12.
// Grips: frame.grip = ["L","R"] — the palm holds the shank of the same side (pull-only cone round the arm, as in physics_build);
//   the exact anchor frames of bow and fold always get both grips (as physics.json does).
//
// Output (cycle_phys.json):
// { src, fps, T, units: [91 muscle units], ligs: [27 ligaments + "cap_<joint>" ×6], segs, holds, opts,
//   poses: { dog|plank|half|bow|fold: <frame record> }   // each pose solved cold exactly as physics_build does (= physics.json)
//   frames: [ { i, T (s, whole round), seg, t (s, in segment),
//       aL, aR   — activation per unit (0..1, force / max force of the unit's strands), left / right
//       sL, sR   — stretch per unit: mean over its strands of L/L_standing − 1 (muscle-tendon path)
//       ligL, ligR — strain per ligament / capsule: max over its strands of L/L0 − 1, L0 = length in plain standing
//       floor: [{ region, side, F: [x,y,z] N, cop: [x,z], pct: % of body weight }], grips: [{ side, P: [x,y,z], F: [x,y,z] N }],
//       com: [x,y,z], margin: m (centre of mass over the support polygon; > 0 inside, −1 = fewer than 3 support points),
//       jr: { L5S1: [compression, shear], T12L1: [compression, shear], hip|knee|ankle|gh|elbow|wrist: [left, right] }  (× body weight)
//       resid: N or N·m (largest equilibrium residual), reserves: Σ|reserve torque| N·m, iters, ms, tone: number of toned units } ] }
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf(k); return i >= 0 && i + 1 < args.length ? args[i + 1] : d; };
const flag = k => args.includes(k);
const IN = arg('--in', 'cycle_keys/cycle.json'), OUT = arg('--out', 'cycle_phys.json');
const EVERY = +arg('--every', 1), FROM = +arg('--from', 0), TO = arg('--to', null), SEGF = arg('--seg', null);
const SMOOTH = +arg('--smooth', 0.15), ITERS = +arg('--iters', 16000), QUIET = flag('--quiet');

const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const q = a => new THREE.Quaternion(...a);
const r3 = v => Math.round(v * 1000) / 1000, r1 = v => Math.round(v * 10) / 10;
const arr = v => [r3(v.x), r3(v.y), r3(v.z)];
const ease = u => { u = Math.max(0, Math.min(1, u)); return u * u * (3 - 2 * u); };
const BW = PH.BW;

const cyc = JSON.parse(fs.readFileSync(IN, 'utf8'));
const poseData = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const segById = Object.fromEntries(cyc.segs.map(s => [s.id, s]));

// ---------- tone ----------
const QUADS = ['vastLat', 'vastMed', 'vastInt', 'rectFem'];
const TONE = { ...CUE_TONE, highplank: Object.assign(Object.fromEntries(QUADS.map(u => [u, 0.15])), { rectAbd: 0.15, gluteMax: 0.12 }) };
function toneAt(fr) {
  const s = segById[fr.seg];
  const Ta = TONE[s.from] || {}, Tb = TONE[s.to] || {};
  const dur = s.t1 - s.t0, ph = s.phases.map(p => ({ t0: p.t0 - s.t0, t1: p.t1 - s.t0 }));
  const first = ph[0], last = ph[ph.length - 1];
  const fA = 1 - ease(fr.t / Math.max(1e-6, first.t1));
  const fB = ease((fr.t - last.t0) / Math.max(1e-6, dur - last.t0));
  const out = {};
  for (const u of new Set([...Object.keys(Ta), ...Object.keys(Tb)])) {
    const a = Ta[u] || 0, b = Tb[u] || 0, sh = Math.min(a, b);
    const v = sh + fA * (a - sh) + fB * (b - sh);
    if (v > 1e-4) out[u] = v;
  }
  if (fr.tone) for (const [u, v] of Object.entries(fr.tone)) out[u] = Math.max(out[u] || 0, v);
  return out;
}

// ---------- geometry: ligaments, capsules, muscle stretch ----------
const UNITS = Object.keys(A.UNITS);
const ALL = A.strandInstances();
const LIG_INST = ALL.filter(i => i.kind === 'lig' || i.kind === 'cap');
const ligKey = i => (i.kind === 'cap' ? 'cap_' : '') + i.unit;
const LIGS = [...Object.keys(A.LIGS), ...Object.keys(A.CAP_NAMES).map(k => 'cap_' + k)];
const LIG_IX = Object.fromEntries(LIGS.map((k, i) => [k, i]));
const W_STAND = A.placeW(A.STAND.local, A.STAND.rootQ);
const LIG_L0 = LIG_INST.map(i => A.pathLength(A.strandPath(W_STAND, i)));
const U_IX = Object.fromEntries(UNITS.map((u, i) => [u, i]));
function ligStrains(W) {
  const L = new Array(LIGS.length).fill(-1), R = new Array(LIGS.length).fill(-1);
  LIG_INST.forEach((inst, k) => {
    const e = A.pathLength(A.strandPath(W, inst)) / LIG_L0[k] - 1;
    const ix = LIG_IX[ligKey(inst)];
    const t = inst.side === 'R' ? R : L;
    if (e > t[ix]) t[ix] = e;
  });
  return { L: L.map(r3), R: R.map(r3) };
}

// ---------- support polygon ----------
function hull(P) {
  const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (Q.length < 3) return Q;
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Q) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
function margin(poly, p) {
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

// ---------- one frame ----------
function confOf(fr) { const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = q(v); return { local: L, rootQ: q(fr.root), rootP: V(...fr.rootP) }; }
function gripsFor(W, sides) {
  return (sides || []).map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: 'hum_' + sd, cone: 1 }; });
}
function record(r, extra) {
  const aL = new Array(UNITS.length).fill(0), aR = new Array(UNITS.length).fill(0);
  const fm = { L: new Array(UNITS.length).fill(0), R: new Array(UNITS.length).fill(0) };
  const ff = { L: new Array(UNITS.length).fill(0), R: new Array(UNITS.length).fill(0) };
  const st = { L: new Array(UNITS.length).fill(0), R: new Array(UNITS.length).fill(0) }, ns = { L: new Array(UNITS.length).fill(0), R: new Array(UNITS.length).fill(0) };
  r.INST.forEach((s, j) => {
    const ix = U_IX[s.unit], sd = s.side === 'R' ? 'R' : 'L';
    ff[sd][ix] += r.force[j]; fm[sd][ix] += r.FMAX[j];
    st[sd][ix] += r.Lm[j] / r.L_STAND[j] - 1; ns[sd][ix] += 1;
  });
  for (let i = 0; i < UNITS.length; i++) { aL[i] = r3(fm.L[i] ? ff.L[i] / fm.L[i] : 0); aR[i] = r3(fm.R[i] ? ff.R[i] / fm.R[i] : 0); }
  const sL = st.L.map((v, i) => r3(ns.L[i] ? v / ns.L[i] : 0)), sR = st.R.map((v, i) => r3(ns.R[i] ? v / ns.R[i] : 0));
  const groups = {};
  for (const c of r.contactF) {
    const k = c.region + '|' + c.side;
    const g = (groups[k] ||= { region: c.region, side: c.side, F: V(0, 0, 0), m: V(0, 0, 0) });
    g.F.add(c.F); g.m.addScaledVector(c.P, c.F.y);
  }
  const floor = Object.values(groups).filter(g => g.F.y > 0.002 * BW).map(g => ({ region: g.region, side: g.side, F: [r1(g.F.x), r1(g.F.y), r1(g.F.z)], cop: [r3(g.m.x / g.F.y), r3(g.m.z / g.F.y)], pct: r1(g.F.y / BW * 100) }));
  const poly = hull(r.contactF.filter(c => c.F.y > 0.002 * BW).map(c => [c.P.x, c.P.z]));
  const lig = ligStrains(r.W);
  const bw = F => r3(F.length() / BW);
  const jr = {
    L5S1: [r3(r.jr.L5S1.compression / BW), r3(r.jr.L5S1.shear / BW)], T12L1: [r3(r.jr.T12L1.compression / BW), r3(r.jr.T12L1.shear / BW)],
    hip: [bw(r.jr.hip_L.F), bw(r.jr.hip_R.F)], knee: [bw(r.jr.knee_L.F), bw(r.jr.knee_R.F)], ankle: [bw(r.jr.ankle_L.F), bw(r.jr.ankle_R.F)],
    gh: [bw(r.jr.gh_L.F), bw(r.jr.gh_R.F)], elbow: [bw(r.jr.elbow_L.F), bw(r.jr.elbow_R.F)], wrist: [bw(r.jr.wrist_L.F), bw(r.jr.wrist_R.F)],
  };
  return {
    ...extra, aL, aR, sL, sR, ligL: lig.L, ligR: lig.R, floor,
    grips: r.gripF.map(g => ({ side: g.side, P: arr(g.P), F: [r1(g.F.x), r1(g.F.y), r1(g.F.z)] })),
    com: arr(r.com), margin: r3(margin(poly, [r.com.x, r.com.z])), jr,
    resid: +r.maxRes.toPrecision(3), reserves: r1(Object.values(r.reserve).reduce((a, x) => a + Math.abs(x), 0)), iters: r.iters,
  };
}
function solveFrame(c, tone, grips, prev) {
  const o = { rootP: c.rootP, tone, grips, iters: ITERS };
  if (prev) { o.warm = prev.warm; if (SMOOTH > 0) { o.smooth = SMOOTH; o.prevAct = prev.act; } }
  return PH.solve({ local: c.local, rootQ: c.rootQ }, o);
}

// ---------- the five held poses, exactly as physics_build solves them ----------
const poses = {};
if (!flag('--no-poses')) for (const id of ['dog', 'plank', 'half', 'bow', 'fold']) {
  const c = A.poseFromData(poseData[id]);
  const W = A.placeW(c.local, c.rootQ);
  const t0 = Date.now();
  const r = PH.solve({ local: c.local, rootQ: c.rootQ }, { tone: CUE_TONE[id] || {}, grips: (id === 'bow' || id === 'fold') ? gripsFor(W, ['L', 'R']) : [], iters: 16000 });
  poses[id] = record(r, { pose: id, ms: Date.now() - t0, tone: Object.keys(CUE_TONE[id] || {}).length });
  if (!QUIET) console.log('pose', id.padEnd(6), poses[id].ms + 'ms', 'iters', r.iters, 'resid', poses[id].resid, 'reserves', poses[id].reserves);
}

// ---------- frames ----------
const N = cyc.frames.length;
const TOi = TO == null ? N - 1 : +TO;
const want = [];
for (let i = FROM; i <= Math.min(TOi, N - 1); i++) {
  const fr = cyc.frames[i];
  if (SEGF && fr.seg !== SEGF) continue;
  if ((i - FROM) % EVERY === 0 || i === Math.min(TOi, N - 1)) want.push(i);
}
const out = [];
let prev = null, prevIdx = -10;
const tStart = Date.now();
for (const i of want) {
  const fr = cyc.frames[i];
  const s = segById[fr.seg];
  const c = confOf(fr);
  const W = A.computeWorld(c.local, c.rootQ, c.rootP);
  const tone = toneAt(fr);
  // the exact anchors of bow and fold hold the shins as in physics.json
  const atStart = fr.t <= 1e-6, atEnd = Math.abs(fr.t - (s.t1 - s.t0)) < 1e-6;
  let gs = fr.grip || [];
  if ((atStart && (s.from === 'bow' || s.from === 'fold')) || (atEnd && (s.to === 'bow' || s.to === 'fold'))) gs = ['L', 'R'];
  const t0 = Date.now();
  // warm start only from the frame just before (a jump of several frames is a different problem)
  const r = solveFrame(c, tone, gripsFor(W, gs), i - prevIdx <= Math.max(1, EVERY) ? prev : null);
  const rec = record(r, { i, T: fr.T, seg: fr.seg, t: fr.t, ms: Date.now() - t0, tone: Object.keys(tone).length });
  out.push(rec);
  prev = r; prevIdx = i;
  if (!QUIET && (out.length % 10 === 1 || i === want[want.length - 1])) console.log(String(i).padStart(4), ('T=' + fr.T).padEnd(8), fr.seg, String(rec.ms).padStart(5) + 'ms', 'iters', String(rec.iters).padStart(5), 'resid', rec.resid, 'reserves', rec.reserves, 'margin', rec.margin, gs.length ? 'grip ' + gs.join('') : '');
}
let frames = out;
if (flag('--merge') && fs.existsSync(OUT)) {
  const old = JSON.parse(fs.readFileSync(OUT, 'utf8'));
  const byI = new Map(old.frames.map(f => [f.i, f]));
  for (const f of out) byI.set(f.i, f);
  frames = [...byI.values()].sort((a, b) => a.i - b.i);
  if (flag('--no-poses')) Object.assign(poses, old.poses);
}
const res = { src: IN, fps: cyc.fps, T: cyc.T, units: UNITS, ligs: LIGS, segs: cyc.segs, holds: cyc.holds, provisional: cyc.segs.some(s => s.provisional), opts: { smooth: SMOOTH, iters: ITERS, every: EVERY }, poses, frames };
fs.writeFileSync(OUT, JSON.stringify(res));
console.log('wrote', OUT, frames.length, 'frames', ((Date.now() - tStart) / 1000).toFixed(1) + 's');
