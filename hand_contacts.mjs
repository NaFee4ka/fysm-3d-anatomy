import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
for (const id of (process.argv[2] || 'dog').split(',')) {
  const c = A.poseFromData(data[id]); const W = A.placeW(c.local, c.rootQ);
  console.log('==', id);
  for (const l of A.LANDMARKS) { if (l.side !== 'L') continue; const P = A.pt(W, l.seg, l.pc); if (P.y < 0.03) console.log(l.region.padEnd(22), (l.tag || '').padEnd(8), l.seg.padEnd(8), 'h=' + (P.y * 1000).toFixed(1) + 'mm', 'x', P.x.toFixed(3), 'z', P.z.toFixed(3)); }
}
