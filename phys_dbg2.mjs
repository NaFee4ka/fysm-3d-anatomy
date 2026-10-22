import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const conf = A.poseFromData(data.dog);
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE.dog, iters: 3000 });
const W = r.W;
for (const jid of (process.argv[2] || 'gh_L_y,gh_L_x,gh_L_z').split(',')) {
  const k = PH.dofIndex[jid];
  const d = PH.DOF[k], ax = d.axis(W), c = d.center(W);
  const parts = [];
  for (const m of A.MASS) { if (!d.moved.has(m.seg)) continue; const P = A.pt(W, m.seg, m.p); parts.push(['mass ' + m.seg, ax.clone().cross(P.clone().sub(c)).y * (-m.m * r.BW)]); }
  for (const ct of r.contactF) if (d.moved.has(ct.seg)) parts.push(['contact ' + ct.region + (ct.tag ? '/' + ct.tag : ''), ax.clone().cross(ct.P.clone().sub(c)).dot(ct.F)]);
  console.log(jid, 'axis', ax.toArray().map(v => v.toFixed(2)).join(','), parts.filter(p => Math.abs(p[1]) > 0.3).map(([n, v]) => n + ' ' + v.toFixed(1)).join(' | '));
}
const hum = W.hum_L.p, el = W.fore_L.p, wr = W.hand_L.p;
console.log('humerus axis', el.clone().sub(hum).normalize().toArray().map(v => v.toFixed(2)).join(','), 'forearm axis', wr.clone().sub(el).normalize().toArray().map(v => v.toFixed(2)).join(','));
console.log('scap Y', new THREE.Vector3(0, 1, 0).applyQuaternion(W.scap_L.q).toArray().map(v => v.toFixed(2)).join(','));
