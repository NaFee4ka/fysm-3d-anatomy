// Model numbers for the yoga-books comparison: activation (physics.json), muscle stretch and ligament strain vs standing (geometry),
// joint angles, joint loads, floor reactions. Writes books/yoga_model_data.json.
import fs from 'fs';
import { A, THREE } from '../cycle_lib.mjs';
const data = JSON.parse(fs.readFileSync(new URL('../poses.json', import.meta.url), 'utf8'));
const PHYS = JSON.parse(fs.readFileSync(new URL('../physics.json', import.meta.url), 'utf8'));
const insts = A.strandInstances();
const confOf = id => (id === 'stand' ? A.STAND : A.poseFromData(data[id]));
const Wof = id => { const c = confOf(id); return { c, W: A.placeW(c.local, c.rootQ) }; };
const S0 = Wof('stand');
const len = (W, i) => A.pathLength(A.strandPath(W, i));
const L0 = insts.map(i => len(S0.W, i));
const ids = ['stand', 'dog', 'plank', 'half', 'bow', 'fold'];
const out = { units: {}, ligs: {}, caps: {}, joints: {}, angles: {}, floor: {}, spine: {}, com: {} };
for (const id of ids) {
  const { c, W } = Wof(id);
  const by = {};
  insts.forEach((ins, j) => {
    if (ins.side !== 'L' && ins.side !== 'C' && ins.side) return;
    const e = len(W, ins) / L0[j] - 1;
    const key = ins.kind + '|' + ins.unit;
    (by[key] ||= []).push(e);
  });
  for (const [key, es] of Object.entries(by)) {
    const [kind, unit] = key.split('|');
    const mean = es.reduce((a, b) => a + b, 0) / es.length, max = Math.max(...es), min = Math.min(...es);
    const tgt = kind === 'muscle' ? out.units : kind === 'lig' ? out.ligs : out.caps;
    (tgt[unit] ||= {})[id] = { ...(tgt[unit][id] || {}), stretchMean: +mean.toFixed(3), stretchMax: +max.toFixed(3), stretchMin: +min.toFixed(3) };
  }
  const ph = PHYS[id];
  for (const [u, m] of Object.entries(ph.muscles)) (out.units[u] ||= {})[id] = { ...(out.units[u][id] || {}), a: m.a, F: m.F, Fp: m.Fp, cue: m.cue, tone: m.tone };
  out.joints[id] = ph.joints;
  out.floor[id] = ph.floor.map(f => ({ region: f.region, side: f.side, pct: f.pct, tang: f.tang }));
  out.spine[id] = ph.joints.ls && ph.joints.ls.jr;
  out.com[id] = { com: ph.com, margin: ph.support.margin, bw: ph.bw };
  // joint angles (left side and spine)
  const ang = {};
  for (const def of A.JOINTS) {
    if (def.side === 'R' || /_R$/.test(def.id || '')) continue;
    try { const r = A.analyzeJoint(def, c.local, W); ang[def.id || def.name] = { primary: r.primary != null ? +(+r.primary).toFixed(1) : null, comps: (r.comps || []).map(x => ({ name: x.name || x.label, v: x.value != null ? +(+x.value).toFixed(1) : (x.deg != null ? +(+x.deg).toFixed(1) : null) })) }; } catch (e) { ang[def.id || def.name] = { err: String(e).slice(0, 80) }; }
  }
  out.angles[id] = ang;
}
out.names = Object.fromEntries(Object.entries(A.UNITS).map(([k, v]) => [k, v.name]));
out.ligNames = Object.fromEntries(Object.entries(A.LIGS).map(([k, v]) => [k, v.name]));
fs.writeFileSync(new URL('./yoga_model_data.json', import.meta.url), JSON.stringify(out));
console.log('JOINT def sample', JSON.stringify(A.JOINTS[0]).slice(0, 300));
console.log('angles sample', JSON.stringify(out.angles.dog).slice(0, 1200));
