import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const conf = A.poseFromData(data.dog);
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE.dog, iters: 16000 });
const W = r.W, V = (x, y, z) => new THREE.Vector3(x, y, z);
const f = v => v.toArray().map(x => x.toFixed(3)).join(', ');
console.log('shoulder L', f(W.hum_L.p), 'elbow', f(W.fore_L.p), 'wrist', f(W.hand_L.p));
console.log('hip L', f(W.thigh_L.p), 'knee', f(W.shank_L.p), 'ankle', f(W.foot_L.p), 'COM', f(r.com));
const arm = PH.subtree('hum_L');
let M = V(0, 0, 0), Fsum = V(0, 0, 0);
for (const c of r.contactF) if (arm.has(c.seg)) { M.add(c.P.clone().sub(W.hum_L.p).cross(c.F)); Fsum.add(c.F); console.log('  contact', c.region, f(c.P), 'F', f(c.F)); }
let mArm = 0; const cg = V(0, 0, 0);
for (const m of A.MASS) if (arm.has(m.seg)) { const P = A.pt(W, m.seg, m.p); const Fg = V(0, -m.m * r.BW, 0); M.add(P.clone().sub(W.hum_L.p).cross(Fg)); mArm += m.m; cg.addScaledVector(P, m.m); }
console.log('arm mass frac', mArm.toFixed(3), 'arm COM', f(cg.multiplyScalar(1 / mArm)));
console.log('hand floor force sum', f(Fsum), '| external moment about shoulder (world)', f(M));
// trunk axis and arm direction
const h = W.fore_L.p.clone().sub(W.hum_L.p).normalize();
const up = V(0, 1, 0).applyQuaternion(W.thC.q);
console.log('arm dir', f(h), 'trunk up', f(up), 'angle arm vs trunk-up', (Math.acos(h.dot(up)) * 180 / Math.PI).toFixed(1));
const down = V(0, -1, 0).applyQuaternion(W.thC.q); let eEl = down.clone().cross(h).normalize();
console.log('eEl', f(eEl), 'moment along eEl', M.dot(eEl).toFixed(1), '(positive = raises arm?)');
// lateral axis of the trunk
const lat = V(1, 0, 0).applyQuaternion(W.thC.q);
console.log('trunk lateral', f(lat), 'moment about trunk lateral axis', M.dot(lat).toFixed(1));
