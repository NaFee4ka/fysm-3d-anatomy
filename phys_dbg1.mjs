import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const conf = A.poseFromData(data.dog);
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: PH.CUE_TONE.dog, iters: 6000 });
const W = r.W;
for (const jid of ['t89_x', 'tl_x', 'l34_x', 'ls_x', 'gh_L_x', 'sc_L_x']) {
  const k = PH.DOF.findIndex(d => d.id === jid);
  const d = PH.DOF[k], ax = d.axis(W), c = d.center(W);
  let g = 0, grf = 0, grfV = 0, st = 0;
  for (const m of A.MASS) { const P = A.pt(W, m.seg, m.p); if (d.moved.has(m.seg)) g += ax.clone().cross(P.clone().sub(c)).y * (-m.m * r.BW); }
  for (const ct of r.contactF) if (d.moved.has(ct.seg)) { const v = ax.clone().cross(ct.P.clone().sub(c)); grf += v.dot(ct.F); grfV += v.y * ct.F.y; }
  for (const s of r.stF) { const va = d.moved.has(s.segA) ? ax.clone().cross(s.P.clone().sub(c)).dot(s.F) : 0; const vb = d.moved.has(s.segB) ? ax.clone().cross(s.P.clone().sub(c)).dot(s.F) : 0; st += va - vb; }
  console.log(jid, 'gravity', g.toFixed(1), 'GRF', grf.toFixed(1), '(vertical part', grfV.toFixed(1) + ')', 'ST', st.toFixed(1), 'axis', ax.toArray().map(v => v.toFixed(2)).join(','), 'center y', c.y.toFixed(2));
}
console.log('hand contact world', r.contactF.filter(c => c.region === 'ладони' && c.side === 'L').map(c => c.P.toArray().map(v => v.toFixed(2)).join(',')).join(' | '));
console.log('shoulder', W.hum_L.p.toArray().map(v => v.toFixed(2)).join(','), 't89', W.thB.p.toArray().map(v => v.toFixed(2)).join(','), 'pelvis', W.pelvis.p.toArray().map(v => v.toFixed(2)).join(','));
