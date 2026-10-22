// Merges the atlas structures (books/fascia_structures.json: Неттер, Синельников) and Myers' lines (books/myers_lines.json)
// into one model for fascia3.js: books/fascia_model.json. Ids of both research files are kept; Myers' fascia ids are
// mapped to atlas structures (or fibre subsets of them) through ALIASES. The big sheets get drawing surfaces (GRIDS).
// usage: node fascia_model_build.mjs
import fs from 'fs';
import { GRIDS, WRAPS, LINE_COLORS, BUILTIN } from './fascia3.js';

const atlas = JSON.parse(fs.readFileSync('books/fascia_structures.json', 'utf8'));
const myers = JSON.parse(fs.readFileSync('books/myers_lines.json', 'utf8'));

// atlas structure → element
const SURF = { tlfPost: [GRIDS.tlf], fasciaLata: [GRIDS.fasciaLata], cruralFascia: [GRIDS.cruralPost, GRIDS.cruralAnt], rectusSheath: [GRIDS.rectusSheath] };
const elements = [];
for (const s of atlas.structures) {
  const g = s.geometry || {};
  const e = {
    id: s.id, ru: s.ru, lat: s.lat, type: s.type, side: s.side === 'C' ? 'C' : 'LR', src: 'atlas',
    myers: s.myers || [], attachments: s.attachments || [], muscles: s.muscles || null, role: s.role || null,
    netter: s.netter || null, sinelnikov: s.sinelnikov || null, rigid: !!s.rigidInModel,
    link: s.modelLink || null,
    strands: s.modelLink ? [] : (g.strands || []).map(st => ({ label: st.label || null, pts: st.pts, cross: !!st.cross, wraps: st.wraps })),
    wraps: g.wraps || [],
  };
  if (g.r) e.r = g.r;
  if (g.flat) e.flat = g.flat;
  if (SURF[s.id]) e.surfaces = SURF[s.id];
  if (s.id === 'calcanealTendon') { e.pathProxy = true; e.r = 0.0045; }
  elements.push(e);
}
const have = new Set(elements.map(e => e.id));
// structures only Myers' file defines (resolved {seg, p} points of the left side)
const myersOnly = [];
for (const [id, f] of Object.entries(myers.fascia || {})) {
  if (['thoracolumbarFascia', 'tlfCross', 'lineaAlba', 'rectusSheath', 'galea', 'cruralFasciaAnt', 'cruralFasciaLat', 'fasciaLata', 'lateralSeptumArm', 'medialSeptumArm', 'palmarAponeurosis', 'clavipectoral', 'diaphragm'].includes(id)) continue;
  if (have.has(id)) continue;
  const strands = [];
  if (f.band && f.band.length > 1) strands.push({ label: 'по линии Майерса', pts: f.band });
  if (f.type === 'sheet' && f.outline && f.outline.length >= 4) {
    // outline → ladder of fibres across the sheet (pairs of points from both ends of the outline)
    const o = f.outline, n = o.length;
    for (let k = 1; k < Math.floor(n / 2); k++) strands.push({ label: 'поперёк листка ' + k, pts: [o[k], o[n - k]] });
  }
  if (!strands.length) continue;
  elements.push({ id, ru: f.ru, lat: f.lat, type: f.type === 'sheet' ? 'fascia' : 'band', side: f.side === 'C' ? 'C' : 'LR', src: 'myers', myers: f.lines || [], rigid: !!f.static, strands, wraps: [] });
  myersOnly.push(id);
}
// a sternal fascia on one rigid bone (the rig bends the thorax between thA and thC; the sternum does not stretch)
const sf = elements.find(e => e.id === 'sternalFascia');
if (sf) for (const st of sf.strands) st.pts = st.pts.map(p => (Array.isArray(p) ? ['thB', p[1], p[2], p[3]] : { seg: 'thB', p: p.p }));

// Myers fascia ids → members of the merged model
const ALIASES = {
  thoracolumbarFascia: [{ fas: 'tlfPost' }],
  tlfCross: [{ fas: 'tlfPost', idx: [5] }],
  lineaAlba: [{ fas: 'lineaAlba' }],
  rectusSheath: [{ fas: 'rectusSheath' }],
  galea: [{ fas: 'galeaAponeurotica' }],
  cruralFasciaAnt: [{ fas: 'cruralFascia', idx: [0] }, { fas: 'ankleRetinacula', idx: [0, 1, 2] }],
  cruralFasciaLat: [{ fas: 'cruralFascia', idx: [1] }, { fas: 'ankleRetinacula', idx: [4] }],
  fasciaLata: [{ fas: 'fasciaLata' }],
  lateralSeptumArm: [{ fas: 'brachialFascia', idx: [3] }],
  medialSeptumArm: [{ fas: 'brachialFascia', idx: [2] }],
  palmarAponeurosis: [{ fas: 'palmarAponeurosis' }, { fas: 'wristRetinacula', idx: [0] }],
  clavipectoral: [{ fas: 'clavipectoralFascia' }],
  diaphragm: [{ fas: 'diaphragm' }],
};
for (const id of myersOnly) ALIASES[id] = [{ fas: id }];
const ELSET = new Set(elements.map(e => e.id));
const famembers = id => (ALIASES[id] || (ELSET.has(id) ? [{ fas: id }] : [])).filter(m => ELSET.has(m.fas));
// per line refinements of a track's fascia (the lateral line takes only the lateral fibres of the fascia lata)
const REFINE = { 'LL:3': { fasciaLata: [{ fas: 'fasciaLata', idx: [1, 2, 3] }] }, 'SFL:4': {}, 'BFL:2': {} };

const lines = [];
const unresolved = [];
for (const l of myers.lines) {
  const groups = l.tracks.map(t => {
    const members = [];
    for (const u of (t.model && t.model.units) || []) members.push({ mus: u });
    for (const g of (t.model && t.model.ligs) || []) members.push({ lig: g });
    for (const f of (t.model && t.model.fascia) || []) {
      const ref = REFINE[l.id + ':' + t.n] && REFINE[l.id + ':' + t.n][f];
      const ms = ref || famembers(f);
      if (!ms.length) unresolved.push(l.id + ' track ' + t.n + ': ' + f);
      for (const m of ms) if (!members.some(x => JSON.stringify(x) === JSON.stringify(m))) members.push(m);
    }
    // representative strand for drawing and the line's length: the research file's path, else the first member
    let rep = null;
    const p0 = (t.path || [])[0];
    if (p0) {
      if (p0.unit) rep = { mus: p0.unit, idx: p0.k != null ? [p0.k] : undefined };
      else if (p0.lig) rep = { lig: p0.lig };
      else if (p0.fascia) { const ms = famembers(p0.fascia); if (ms.length) rep = ms[0]; }
    }
    return { n: t.n, ru: t.ru, lat: t.lat, side: t.side, cov: t.cov, note: t.note || null, members, rep, from: t.from || null, to: t.to || null, related: (t.model && t.model.related) || [] };
  });
  lines.push({ id: l.id, ru: l.ru, abbr: l.abbr, en: l.en, color: LINE_COLORS[l.id], crossing: !!l.crossing, sideNote: l.sideNote || null, fn: l.fn || null, evidence: l.evidence || null, groups });
}
const model = {
  about: 'Фасции и миофасциальные линии для fascia3.js: структуры по атласам Неттера и Синельникова (books/fascia_structures.json), линии по Т. Майерсу (books/myers_lines.json); координаты — сырые точки модели (см. fascia3.js).',
  built: new Date().toISOString(),
  sources: { atlas: atlas.meta ? atlas.meta.about : null, myers: myers.source || null },
  elements, lines, aliases: ALIASES, missingMuscles: myers.missingMuscles || {},
};
fs.writeFileSync('books/fascia_model.json', JSON.stringify(model));
console.log('fascia_model.json:', elements.length, 'elements (' + elements.filter(e => e.link).length + ' links to model ligaments, Myers-only: ' + myersOnly.join(', ') + '),', lines.length, 'lines,', lines.reduce((a, l) => a + l.groups.length, 0), 'tracks;', (JSON.stringify(model).length / 1024).toFixed(0), 'KB');
if (unresolved.length) console.log('unresolved fascia refs:', unresolved.join('; '));
