import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { PASSIVE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const inst = A.strandInstances().filter(i => i.kind === 'muscle' && i.side === 'L');
const W0 = A.placeW(A.STAND.local, A.STAND.rootQ);
const L0 = inst.map(i => A.pathLength(A.strandPath(W0, i)));
const units = Object.keys(PASSIVE).concat(['quadFem', 'piriformis', 'gemelli']);
const rows = {};
for (const id of ['dog', 'plank', 'half', 'bow', 'fold']) {
  const c = A.poseFromData(data[id]); const W = A.placeW(c.local, c.rootQ);
  inst.forEach((i, j) => { if (!units.includes(i.unit)) return; const e = A.pathLength(A.strandPath(W, i)) / L0[j] - 1; const r = (rows[i.unit] ||= {}); (r[id] ||= []).push(e); });
}
console.log('unit'.padEnd(12), ['dog', 'plank', 'half', 'bow', 'fold'].map(s => s.padStart(14)).join(''), '   e0  eLim');
for (const [u, r] of Object.entries(rows)) console.log(u.padEnd(12), ['dog', 'plank', 'half', 'bow', 'fold'].map(id => { const a = r[id]; const mx = Math.max(...a), av = a.reduce((x, y) => x + y, 0) / a.length; return (Math.round(av * 100) + '/' + Math.round(mx * 100) + '%').padStart(14); }).join(''), PASSIVE[u] ? '   ' + PASSIVE[u].e0 + ' ' + PASSIVE[u].eLim : '');
