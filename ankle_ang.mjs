import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const ang = (W) => { const s = W.shank_L.p.clone().sub(W.foot_L.p).normalize(); const f = W.toes_L.p.clone().sub(W.foot_L.p).normalize(); return Math.acos(s.dot(f)) * 180 / Math.PI; };
const W0 = A.placeW(A.STAND.local, A.STAND.rootQ); const a0 = ang(W0);
for (const id of ['dog', 'plank', 'half', 'bow', 'fold']) {
  const c = A.poseFromData(data[id]); const W = A.placeW(c.local, c.rootQ);
  const e = new THREE.Euler().setFromQuaternion(c.local.foot_L, 'XYZ');
  console.log(id.padEnd(6), 'ankle plantarflexion (geom)', (ang(W) - a0).toFixed(1), '| foot local X', (e.x * 180 / Math.PI).toFixed(1));
}
