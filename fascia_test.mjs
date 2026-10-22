// Fascia and Myers lines in the five poses and plain standing: books/fascia_pose_table.json + console tables.
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { createFascia } from './fascia3.js';
const A = createAnatomy(THREE);
const useBuiltin = process.argv.includes('--builtin');
const MODEL = !useBuiltin && fs.existsSync('books/fascia_model.json') ? JSON.parse(fs.readFileSync('books/fascia_model.json', 'utf8')) : null;
const F = createFascia(A, THREE, MODEL);
console.log('model:', MODEL ? 'books/fascia_model.json' : 'built-in');
const poses = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const phys = JSON.parse(fs.readFileSync('physics.json', 'utf8'));
const ids = ['stand', 'dog', 'plank', 'half', 'bow', 'fold'];
const pct = v => (v * 100).toFixed(1);
const out = { generated: new Date().toISOString(), poses: ids, elements: {}, lines: {} };
const t0 = Date.now();
const res = {};
for (const id of ids) {
  const conf = id === 'stand' ? A.STAND : A.poseFromData(poses[id]);
  const W = A.placeW(conf.local, conf.rootQ);
  const m = F.measure(W);
  F.withPhysics(m, F.physFromStatic(phys[id]));
  res[id] = m;
}
const ms = (Date.now() - t0) / ids.length;
for (const e of F.ELEMENTS) out.elements[e.id] = { name: e.ru, link: e.link || null, kind: e.kind, strain: Object.fromEntries(ids.map(id => [id, +((res[id].el[e.id + '_L'].strain + res[id].el[e.id + '_R'].strain) / 2).toFixed(4)])), max: Object.fromEntries(ids.map(id => [id, +Math.max(res[id].el[e.id + '_L'].max, res[id].el[e.id + '_R'].max).toFixed(4)])) };
for (const l of F.LINES) {
  const o = { ru: l.ru, en: l.en, L0: +res.stand.lines[l.id + '_L'].L0.toFixed(4), strain: {}, chainStrain: {}, engage: {}, actMean: {}, actMax: {}, Tact: {}, Tpas: {}, groups: {} };
  for (const id of ids) {
    const a = res[id].lines[l.id + '_L'], b = res[id].lines[l.id + '_R'];
    const av = k => +(((a[k] || 0) + (b[k] || 0)) / 2).toFixed(4);
    o.strain[id] = av('strain'); o.chainStrain[id] = av('chainStrain'); o.engage[id] = av('engage'); o.actMean[id] = av('actMean');
    o.actMax[id] = { a: +Math.max(a.actMax, b.actMax).toFixed(3), unit: a.actMax >= b.actMax ? a.actMaxUnit : b.actMaxUnit };
    o.Tact[id] = Math.round((a.Tact + b.Tact) / 2); o.Tpas[id] = Math.round((a.Tpas + b.Tpas) / 2);
    o.groups[id] = a.groups.map(gr => ({ ru: gr.ru, strain: +gr.strain.toFixed(4) }));
  }
  out.lines[l.id] = o;
}
fs.mkdirSync('books', { recursive: true });
fs.writeFileSync(MODEL ? 'books/fascia_pose_table.json' : 'books/fascia_pose_table_builtin.json', JSON.stringify(out, null, 1));
console.log(`measure: ${ms.toFixed(0)} ms per pose, ${F.instances.length} fascia strands, ${F.ELEMENTS.length} elements, ${F.LINES.length} lines`);
console.log('\nLINES: strain % (geometry, vs standing) | engagement % ((Tact+Tpas)/ΣFmax) | most active member');
console.log('line'.padEnd(6), ids.slice(1).map(id => id.padStart(15)).join(''));
for (const l of F.LINES) {
  const o = out.lines[l.id];
  console.log(l.id.padEnd(6), ids.slice(1).map(id => (pct(o.strain[id]) + '/' + pct(o.engage[id])).padStart(15)).join(''), '  ', ids.slice(1).map(id => o.actMax[id].unit + ' ' + Math.round(o.actMax[id].a * 100)).join(', '));
}
console.log('\nELEMENTS: mean strain % (max)');
for (const e of F.ELEMENTS) { const o = out.elements[e.id]; console.log(e.id.padEnd(14), ids.slice(1).map(id => (pct(o.strain[id]) + ' (' + pct(o.max[id]) + ')').padStart(15)).join('')); }
// sanity: what the books say must come out of the geometry
const chk = [
  ['SBL lengthens in dog', out.lines.SBL.strain.dog > 0.02], ['SBL lengthens in fold', out.lines.SBL.strain.fold > 0.02],
  ['SFL lengthens in half', out.lines.SFL.strain.half > 0.0], ['SFL lengthens in bow', out.lines.SFL.strain.bow > 0.02],
  ['SFL shortens in fold', out.lines.SFL.strain.fold < 0], ['SBL shortens in bow', out.lines.SBL.strain.bow < 0],
  ['SFAL lengthens in dog (arms overhead)', out.lines.SFAL.strain.dog > 0.02],
  ['DFL psoas group lengthens in bow', out.lines.DFL.groups.bow.find(g => /оясничн/.test(g.ru)).strain > 0],
  ['standing strain = 0', Object.values(out.lines).every(o => Math.abs(o.strain.stand) < 1e-9)],
];
console.log('\nCHECKS'); for (const [n, ok] of chk) console.log(ok ? '  ok  ' : '  FAIL', n);
