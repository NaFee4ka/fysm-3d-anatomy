// prints muscle/ligament lengths in the handstand relative to standing (the same measure the page shows)
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const poses = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2] || 'handstand';
const insts = A.strandInstances();
const c = A.poseFromData(poses[id]);
const W = A.placeW(c.local, c.rootQ);
const W0 = A.placeW(A.STAND.local, A.STAND.rootQ);
const unit = {}, cnt = {}, lig = {}, lc = {};
for (const i of insts) {
  if (i.kind === 'cap') continue;
  const e = A.pathLength(A.strandPath(W, i)) / A.pathLength(A.strandPath(W0, i)) - 1;
  const [o, n] = i.kind === 'muscle' ? [unit, cnt] : [lig, lc];
  o[i.unit] = (o[i.unit] || 0) + e; n[i.unit] = (n[i.unit] || 0) + 1;
}
for (const u in unit) unit[u] /= cnt[u];
for (const u in lig) lig[u] /= lc[u];
const top = o => Object.entries(o).sort((a, b) => b[1] - a[1]);
console.log('LONGER:', top(unit).slice(0, 22).map(([u, s]) => u + ' +' + Math.round(s * 100) + '%').join(', '));
console.log('SHORTER:', top(unit).slice(-14).map(([u, s]) => u + ' ' + Math.round(s * 100) + '%').join(', '));
console.log('LIGS:', top(lig).slice(0, 10).map(([u, s]) => u + ' ' + Math.round(s * 100) + '%').join(', '));
