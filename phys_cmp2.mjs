import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2];
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
for (const v of JSON.parse(process.argv[3])) {
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, Object.assign({ tone: CUE_TONE[id] || {}, iters: 10000 }, v));
  const reg = {};
  for (const c of r.contactF) { const k = c.region + (c.side === 'C' ? '' : ' ' + c.side); reg[k] = (reg[k] || 0) + c.F.y / r.BW * 100; }
  const u = {};
  r.INST.forEach((s, j) => { if (s.side !== 'L') return; (u[s.unit] ||= [0, 0]); u[s.unit][0] += r.force[j]; u[s.unit][1] += r.FMAX[j]; });
  const rows = Object.entries(u).map(([k, [F, Fm]]) => [k, F / Fm]).sort((a, b) => b[1] - a[1]);
  console.log(JSON.stringify(v), 'resid', r.maxRes.toExponential(1), '\n   floor', Object.entries(reg).filter(([k]) => !/ R$/.test(k)).map(([k, x]) => `${k} ${x.toFixed(1)}`).join(', '), '\n   ', rows.slice(0, 16).map(([k, a]) => `${k} ${(a * 100).toFixed(0)}`).join(', '), '\n    ST', r.stF.filter(s => s.side === 'L').map(s => (s.F.length() / r.BW * 100).toFixed(0)).join('/'), 'L5S1 compr', (r.jr.L5S1.compression / r.BW).toFixed(2), 'BW');
}
