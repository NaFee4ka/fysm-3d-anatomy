import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
for (const id of ['dog', 'plank', 'half', 'bow', 'fold']) {
  const c = A.poseFromData(data[id]);
  const q = c.local.hand_L; const e = new THREE.Euler().setFromQuaternion(q, 'ZXY');
  const W = A.placeW(c.local, c.rootQ);
  const kn = W.shank_L.p.clone().sub(W.thigh_L.p).normalize(), sh = W.foot_L.p.clone().sub(W.shank_L.p).normalize();
  console.log(id.padEnd(6), 'wrist local euler ZXY: ext(Z)', (e.z * 180 / Math.PI).toFixed(1), 'dev(X)', (e.x * 180 / Math.PI).toFixed(1), '| knee flex', (Math.acos(kn.dot(sh)) * 180 / Math.PI).toFixed(1));
}
