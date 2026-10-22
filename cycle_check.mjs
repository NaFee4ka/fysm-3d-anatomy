// Shared checker for a segment of the round (coordinator file; the same numbers for everyone).
// usage: node cycle_check.mjs cycle_keys/seg1.json [--frames] [--json]
//   floor: lowest landmark (penetration < 0 mm), "hover" landmarks 4–6 mm above the floor (ambiguous contacts)
//   slide: horizontal travel of landmarks that stay in contact between two frames (per frame and in total)
//   balance: centre of mass over the support polygon of all contacts (margin > 0 inside, metres)
//   speed: largest change of a segment's world orientation between frames (deg per 0.1 s)
//   ends: first/last frame against the anchor poses (cycle_keys/anchors.json, highplank.json, fold.json)
import fs from 'fs';
import { A, V, THREE, computeWorld, pt, LANDMARKS, comOf, SEG } from './cycle_lib.mjs';

const file = process.argv[2];
const showFrames = process.argv.includes('--frames'), asJson = process.argv.includes('--json');
const seg = JSON.parse(fs.readFileSync(file, 'utf8'));
const frames = seg.frames || seg;
const q = a => new THREE.Quaternion(...a);
const anchors = JSON.parse(fs.readFileSync('cycle_keys/anchors.json', 'utf8'));
for (const extra of ['highplank', 'fold']) if (fs.existsSync(`cycle_keys/${extra}.json`)) anchors[extra] = JSON.parse(fs.readFileSync(`cycle_keys/${extra}.json`, 'utf8'));
const confOf = fr => { const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = q(v); return { L, rootQ: q(fr.root), rootP: V(...fr.rootP) }; };
const angDeg = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(a.dot(b)))) * 180 / Math.PI;
const lmName = l => l.region + (l.tag ? ':' + l.tag : '') + ':' + l.side;

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

const rows = [];
let prev = null;
const slideTot = {};
for (let i = 0; i < frames.length; i++) {
  const c = confOf(frames[i]);
  const W = computeWorld(c.L, c.rootQ, c.rootP);
  const P = LANDMARKS.map(l => pt(W, l.seg, l.pc));
  let minY = Infinity, minL = null, hover = [];
  P.forEach((p, k) => { if (p.y < minY) { minY = p.y; minL = LANDMARKS[k]; } if (p.y >= 0.004 && p.y < 0.006) hover.push(lmName(LANDMARKS[k])); });
  const inC = P.map(p => p.y < 0.004);
  const regions = [...new Set(LANDMARKS.filter((l, k) => inC[k]).map(l => l.region + ':' + l.side))];
  const poly = hull(P.filter((p, k) => inC[k]).map(p => [p.x, p.z]));
  const com = comOf(W);
  const marg = margin(poly, [com.x, com.z]);
  let slide = 0, slideL = null, rot = 0, rotS = null;
  if (prev) {
    P.forEach((p, k) => { if (inC[k] && prev.inC[k]) { const d = Math.hypot(p.x - prev.P[k].x, p.z - prev.P[k].z); const nm = lmName(LANDMARKS[k]); slideTot[nm] = (slideTot[nm] || 0) + d; if (d > slide) { slide = d; slideL = nm; } } });
    for (const s of SEG) { const a = angDeg(W[s.name].q, prev.W[s.name].q); if (a > rot) { rot = a; rotS = s.name; } }
  }
  rows.push({ i, t: frames[i].t, minY, minL: lmName(minL), hover: hover.length, hoverL: hover.slice(0, 3), contacts: regions.join(' '), margin: marg, slide, slideL, rot, rotS, grip: (frames[i].grip || []).join('') });
  prev = { W, P, inC };
}
// ends against the anchors
const endErr = (fr, id) => {
  if (!anchors[id]) return null;
  const a = anchors[id];
  let qa = 0; for (const s of SEG) qa = Math.max(qa, angDeg(q(fr.local[s.name]), q(a.local[s.name])));
  return { pose: id, maxJointDeg: +qa.toFixed(3), rootDeg: +angDeg(q(fr.root), q(a.root)).toFixed(3), rootPmm: +(V(...fr.rootP).distanceTo(V(...a.rootP)) * 1000).toFixed(2) };
};
const worst = (k, f) => rows.reduce((b, r) => (f(r[k], b[k]) ? r : b), rows[0]);
const sum = {
  file, frames: frames.length, duration: seg.duration, fps: seg.fps,
  start: endErr(frames[0], seg.from), end: endErr(frames[frames.length - 1], seg.to),
  penetration_mm: +(Math.min(0, worst('minY', (a, b) => a < b).minY) * 1000).toFixed(1), penetrationAt: worst('minY', (a, b) => a < b).i + ' ' + worst('minY', (a, b) => a < b).minL,
  hoverMax: worst('hover', (a, b) => a > b).hover, hoverAt: worst('hover', (a, b) => a > b).i + ' ' + worst('hover', (a, b) => a > b).hoverL.join(','),
  slideMax_cm_per_frame: +(worst('slide', (a, b) => a > b).slide * 100).toFixed(2), slideAt: worst('slide', (a, b) => a > b).i + ' ' + worst('slide', (a, b) => a > b).slideL,
  slideTotal_cm: Object.fromEntries(Object.entries(slideTot).filter(([, v]) => v > 0.01).sort((a, b) => b[1] - a[1]).slice(0, 8).map(([k, v]) => [k, +(v * 100).toFixed(1)])),
  marginMin_cm: +(worst('margin', (a, b) => a < b).margin * 100).toFixed(1), marginAt: worst('margin', (a, b) => a < b).i,
  rotMax_deg_per_frame: +worst('rot', (a, b) => a > b).rot.toFixed(1), rotAt: worst('rot', (a, b) => a > b).i + ' ' + worst('rot', (a, b) => a > b).rotS,
};
if (asJson) console.log(JSON.stringify({ sum, rows }));
else {
  console.log(JSON.stringify(sum, null, 1));
  if (showFrames) for (const r of rows) console.log(String(r.i).padStart(3), String(r.t).padStart(5), 'minY', (r.minY * 1000).toFixed(1).padStart(6), 'mm', r.minL.padEnd(26), 'hover', r.hover, 'margin', (r.margin * 100).toFixed(1).padStart(6), 'cm', 'slide', (r.slide * 100).toFixed(2), 'rot', r.rot.toFixed(1), r.rotS || '', r.grip ? 'grip ' + r.grip : '', '|', r.contacts);
}
