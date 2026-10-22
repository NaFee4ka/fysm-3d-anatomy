import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2] || 'dog';
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, Object.assign({ tone: CUE_TONE[id] || {}, iters: 8000 }, JSON.parse(process.argv[4] || '{}')));
for (const jid of (process.argv[3] || 'sc_L_x,sc_L_y,sc_L_z').split(',')) {
  const k = PH.dofIndex[jid];
  const d = r.dofOut[k];
  const contrib = r.INST.map((s, j) => [s.unit + '#' + s.idx, r.R[j][k] * r.force[j], r.R[j][k], r.act[j]]).filter(x => Math.abs(x[1]) > 0.8).sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
  console.log(jid, 'ext', d.ext.toFixed(1), 'ST', d.st.toFixed(1), 'passive', d.pas.toFixed(1), 'muscles', d.mus.toFixed(1), 'res', d.res.toFixed(1), '\n   ', contrib.slice(0, 14).map(([n, m, ra, a]) => `${n} ${m.toFixed(1)} (r ${(ra * 100).toFixed(1)}, a ${(a * 100).toFixed(0)}%)`).join(', '));
}
