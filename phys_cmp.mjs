import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2];
const variants = JSON.parse(process.argv[3]);
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
for (const v of variants) {
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, Object.assign({ tone: PH.CUE_TONE[id] || {}, iters: 8000 }, v));
  const u = {};
  r.INST.forEach((s, j) => { if (s.side !== 'L') return; (u[s.unit] ||= [0, 0]); u[s.unit][0] += r.force[j]; u[s.unit][1] += r.FMAX[j]; });
  const rows = Object.entries(u).map(([k, [F, Fm]]) => [k, F / Fm]).sort((a, b) => b[1] - a[1]);
  const res = Object.entries(r.reserve).filter(([k, x]) => Math.abs(x) > 0.5 && /_L_|^[a-z0-9]+_[xyz]$/.test(k)).map(([k, x]) => `${k}:${x.toFixed(1)}`);
  console.log(JSON.stringify(v), 'resid', r.maxRes.toExponential(1), '\n  ', rows.slice(0, 14).map(([k, a]) => `${k} ${(a * 100).toFixed(0)}`).join(', '), '\n   ST', r.stF.filter(s => s.side === 'L').map(s => (s.F.length() / r.BW * 100).toFixed(0)).join('/'), 'reserves', res.join(' '));
}
