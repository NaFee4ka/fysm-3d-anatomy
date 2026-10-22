import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const inst = A.strandInstances().filter(i => i.side === 'L' && /^(quadFem|gemelli|obtInt|obtExt|piriformis)$/.test(i.unit));
const W0 = A.placeW(A.STAND.local, A.STAND.rootQ);
for (const id of ['dog', 'fold', 'half', 'bow']) {
  const c = A.poseFromData(data[id]); const W = A.placeW(c.local, c.rootQ);
  console.log(id, inst.map(i => i.unit + '#' + i.idx + ' ' + Math.round((A.pathLength(A.strandPath(W, i)) / A.pathLength(A.strandPath(W0, i)) - 1) * 100) + '%').join(', '));
}
