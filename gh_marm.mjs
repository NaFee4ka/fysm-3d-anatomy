// sagittal moment arms of shoulder muscles: rotation that moves the arm toward the chest side (+) in a pose
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2] || 'dog';
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
const W = A.placeW(conf.local, conf.rootQ);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const h = W.fore_L.p.clone().sub(W.hum_L.p).normalize();
const chest = V(0, 0, 1).applyQuaternion(W.thC.q);
const up = V(0, 1, 0).applyQuaternion(W.thC.q);
const ang = Math.atan2(h.dot(chest), -h.dot(up)) * 180 / Math.PI; // 0 = hanging, 90 = forward, 180 = overhead
const sag = Math.atan2(-h.dot(chest) * 0 + 0, 1);
console.log(id, 'arm angle in trunk sagittal plane from hanging:', (Math.atan2(h.dot(chest), -h.dot(up)) * 180 / Math.PI).toFixed(1), '(>180 behind the head when chest comp<0)', 'h.chest', h.dot(chest).toFixed(3), 'h.up', h.dot(up).toFixed(3));
const a = h.clone().cross(chest).normalize(); // + rotation moves arm toward chest side
const set = PH.subtree('hum_L');
const rot = (ang) => { const R = new THREE.Quaternion().setFromAxisAngle(a, ang); const c = W.hum_L.p; const W2 = {}; for (const n in W) { if (!set.has(n)) { W2[n] = W[n]; continue; } W2[n] = { q: R.clone().multiply(W[n].q), p: W[n].p.clone().sub(c).applyQuaternion(R).add(c) }; } return W2; };
const d = 0.3 * Math.PI / 180, Wp = rot(d), Wm = rot(-d);
const by = {};
for (const m of PH.INST) { if (m.side !== 'L' || !/^(deltAnt|deltMid|deltPost|pecMaj|lat|teresMaj|teresMin|infra|supra|subscap|coracobr|bicLong|bicShort|tricLong)$/.test(m.unit)) continue;
  const r = -(A.pathLength(A.strandPath(Wp, m)) - A.pathLength(A.strandPath(Wm, m))) / (2 * d);
  (by[m.unit] ||= []).push(r * 100);
}
console.log(Object.entries(by).map(([u, rs]) => u + ' ' + rs.map(x => x.toFixed(1)).join('/')).join(' | '));
