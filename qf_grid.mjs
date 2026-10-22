import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const inst = A.strandInstances().filter(i => i.side === 'L' && i.unit === 'quadFem')[0];
console.log(JSON.stringify(inst.pts.map(p => [p.seg, p.p.x, p.p.y, p.p.z])), inst.wraps.length);
const W0 = A.placeW(A.STAND.local, A.STAND.rootQ);
const Ws = ['dog', 'fold'].map(id => { const c = A.poseFromData(data[id]); return A.placeW(c.local, c.rootQ); });
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const test = (o, i) => {
  const m = { ...inst, pts: [{ seg: 'pelvis', p: V(...o) }, { seg: 'thigh_L', p: V(...i) }] };
  const L0 = A.pathLength(A.strandPath(W0, m));
  return [L0, ...Ws.map(W => A.pathLength(A.strandPath(W, m)) / L0 - 1)];
};
for (const o of [[0.07, 0.866, -0.052], [0.062, 0.878, -0.05], [0.058, 0.884, -0.048], [0.055, 0.89, -0.046], [0.062, 0.888, -0.046]])
  for (const i of [[0.138, 0.886, -0.024], [0.136, 0.892, -0.026], [0.134, 0.898, -0.024], [0.132, 0.888, -0.03]]) {
    const [L0, d, f] = test(o, i);
    console.log(JSON.stringify(o), JSON.stringify(i), 'L0', (L0 * 100).toFixed(1) + 'cm', 'dog', Math.round(d * 100) + '%', 'fold', Math.round(f * 100) + '%');
  }
