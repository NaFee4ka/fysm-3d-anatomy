// Myers lines and fascial structures over the round: books/fascia_round.json (+ console summary).
// usage: node fascia_round.mjs [--cycle cycle_keys/cycle.json] [--phys cycle_phys.json] [--hold 15]
//   geometry: every frame of cycle.json; physics: cycle_phys.json frames (often every N-th) interpolated linearly in time.
//   per line and side: strain (geometry, vs standing) and engagement (Tact+Tpas)/ΣFmax, see fascia3.js.
//   summary per line: peak and when, time stretched ≥ 5 % / ≥ 10 %, peak engagement, time engaged ≥ 10 % / ≥ 20 %,
//   and the same during holds of H seconds in each of the five poses (static physics.json, symmetric).
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { createFascia } from './fascia3.js';

const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const A = createAnatomy(THREE);
const MODEL = fs.existsSync('books/fascia_model.json') ? JSON.parse(fs.readFileSync('books/fascia_model.json', 'utf8')) : null;
const F = createFascia(A, THREE, MODEL);
const cyc = JSON.parse(fs.readFileSync(arg('--cycle', 'cycle_keys/cycle.json'), 'utf8'));
const physPath = arg('--phys', 'cycle_phys.json');
const CP = fs.existsSync(physPath) ? JSON.parse(fs.readFileSync(physPath, 'utf8')) : null;
const PS = JSON.parse(fs.readFileSync('physics.json', 'utf8'));
const HOLD = +arg('--hold', 15);
const Fmax = Object.fromEntries(Object.entries(PS.dog.muscles).map(([u, m]) => [u, m.Fmax]));
const q = a => new THREE.Quaternion(...a);
// activations at time T: linear between the solved frames
const pf = CP ? CP.frames.slice().sort((a, b) => a.T - b.T) : [];
function physAt(T) {
  if (!pf.length) return null;
  let j = pf.findIndex(f => f.T >= T); if (j < 0) j = pf.length - 1;
  const b = pf[j], a = pf[Math.max(0, j - 1)];
  const u = b.T > a.T ? Math.max(0, Math.min(1, (T - a.T) / (b.T - a.T))) : 0;
  const mix = (x, y) => x.map((v, k) => v + (y[k] - v) * u);
  return F.physFromFrame({ aL: mix(a.aL, b.aL), aR: mix(a.aR, b.aR) }, CP.units, Fmax);
}
const ids = F.LINES.map(l => l.id);
const per = { T: [], lines: Object.fromEntries(ids.map(id => [id, { sL: [], sR: [], eL: [], eR: [] }])), el: Object.fromEntries(F.ELEMENTS.map(e => [e.id, []])) };
const t0 = Date.now();
for (const fr of cyc.frames) {
  const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = q(v);
  const W = A.computeWorld(L, q(fr.root), new THREE.Vector3(...fr.rootP));
  const m = F.withPhysics(F.measure(W), physAt(fr.T));
  per.T.push(fr.T);
  for (const id of ids) {
    const a = m.lines[id + '_L'], b = m.lines[id + '_R'], o = per.lines[id];
    o.sL.push(+a.strain.toFixed(4)); o.sR.push(+b.strain.toFixed(4)); o.eL.push(+(a.engage || 0).toFixed(4)); o.eR.push(+(b.engage || 0).toFixed(4));
  }
  for (const e of F.ELEMENTS) per.el[e.id].push(+Math.max(m.el[e.id + '_L'].strain, m.el[e.id + '_R'].strain).toFixed(4));
}
// static holds
const holdPoses = ['dog', 'plank', 'half', 'bow', 'fold'];
const poses = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const hold = {};
for (const id of holdPoses) {
  const c = A.poseFromData(poses[id]);
  const m = F.withPhysics(F.measure(A.placeW(c.local, c.rootQ)), F.physFromStatic(PS[id]));
  hold[id] = Object.fromEntries(ids.map(l => [l, { s: +((m.lines[l + '_L'].strain + m.lines[l + '_R'].strain) / 2).toFixed(4), e: +((m.lines[l + '_L'].engage + m.lines[l + '_R'].engage) / 2).toFixed(4) }]));
}
const dt = cyc.frames.length > 1 ? (cyc.T / (cyc.frames.length - 1)) : 0.1;
const segOf = T => (cyc.segs.find(s => T >= s.t0 - 1e-6 && T <= s.t1 + 1e-6) || {}).id;
const phaseOf = T => { for (const s of cyc.segs) for (const p of s.phases) if (T >= p.t0 - 1e-6 && T <= p.t1 + 1e-6) return p.label; return null; };
const summary = {};
for (const id of ids) {
  const o = per.lines[id];
  const s = o.sL.map((v, k) => Math.max(v, o.sR[k])), e = o.eL.map((v, k) => Math.max(v, o.eR[k]));
  const kS = s.indexOf(Math.max(...s)), kE = e.indexOf(Math.max(...e)), kMin = s.indexOf(Math.min(...s));
  const time = (arr, thr) => +(arr.filter(v => v >= thr).length * dt).toFixed(1);
  const hs = holdPoses.map(p => hold[p][id]);
  summary[id] = {
    ru: F.LINE[id].ru,
    trans: { strainPeak: s[kS], strainPeakT: per.T[kS], strainPeakAt: phaseOf(per.T[kS]), shortest: s[kMin], shortestT: per.T[kMin], t05: time(s, 0.05), t10: time(s, 0.1), engPeak: e[kE], engPeakT: per.T[kE], engPeakAt: phaseOf(per.T[kE]), e10: time(e, 0.1), e20: time(e, 0.2), engMean: +(e.reduce((a, b) => a + b, 0) / e.length).toFixed(4), asym: +Math.max(...o.sL.map((v, k) => Math.abs(v - o.sR[k]))).toFixed(4) },
    hold: Object.fromEntries(holdPoses.map((p, k) => [p, hs[k]])),
    withHold: { H: HOLD, strainPeak: Math.max(s[kS], ...hs.map(h => h.s)), t05: +(time(s, 0.05) + HOLD * hs.filter(h => h.s >= 0.05).length).toFixed(1), e10: +(time(e, 0.1) + HOLD * hs.filter(h => h.e >= 0.1).length).toFixed(1), engMean: +((e.reduce((a, b) => a + b, 0) * dt + HOLD * hs.reduce((a, h) => a + h.e, 0)) / (cyc.T + 5 * HOLD)).toFixed(4) },
  };
}
const out = { about: 'Миофасциальные линии Майерса за круг: геометрия по кадрам cycle.json, физика по cycle_phys.json (между решёнными кадрами — линейно).', cycle: { T: cyc.T, frames: cyc.frames.length, provisional: cyc.segs.some(s => s.provisional), segs: cyc.segs.map(s => ({ id: s.id, from: s.from, to: s.to, t0: s.t0, t1: s.t1 })) }, physFrames: pf.length, holdSeconds: HOLD, summary, perFrame: per };
fs.writeFileSync('books/fascia_round.json', JSON.stringify(out));
console.log(`round: ${cyc.frames.length} frames (${out.cycle.provisional ? 'PROVISIONAL segments' : 'real segments'}), physics frames ${pf.length}, ${((Date.now() - t0) / cyc.frames.length).toFixed(1)} ms/frame`);
console.log('line   peak stretch (when)                         shortest   t≥5%   peak engagement (when)                     t≥10%');
for (const id of ids) { const r = summary[id].trans; console.log(id.padEnd(6), ((r.strainPeak * 100).toFixed(1) + '% @' + r.strainPeakT + 's ' + (r.strainPeakAt || '')).padEnd(46).slice(0, 46), ((r.shortest * 100).toFixed(1) + '%').padStart(7), String(r.t05).padStart(6), ((r.engPeak * 100).toFixed(1) + '% @' + r.engPeakT + 's ' + (r.engPeakAt || '')).padEnd(44).slice(0, 44), String(r.e10).padStart(6)); }
