import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
for (const id of ['dog', 'plank', 'half', 'bow', 'fold']) {
  const c = A.poseFromData(data[id]); const W = A.placeW(c.local, c.rootQ);
  const u = W.hum_L.p.clone().sub(W.fore_L.p).normalize(), f = W.hand_L.p.clone().sub(W.fore_L.p).normalize();
  console.log(id, 'elbow flexion', (180 - Math.acos(u.dot(f)) * 180 / Math.PI).toFixed(1));
}
