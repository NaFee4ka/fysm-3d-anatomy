import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2];
const conf = A.poseFromData(data[id]);
const W = A.placeW(conf.local, conf.rootQ);
const grips = ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd }; });
for (const tv of JSON.parse(process.argv[3])) {
  const tone = Object.assign({}, CUE_TONE[id], tv);
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone, grips, iters: 10000 });
  const u = {};
  r.INST.forEach((s, j) => { if (s.side !== 'L') return; (u[s.unit] ||= [0, 0]); u[s.unit][0] += r.force[j]; u[s.unit][1] += r.FMAX[j]; });
  const rows = Object.entries(u).map(([k, [F, Fm]]) => [k, F / Fm]).sort((a, b) => b[1] - a[1]);
  const eff = jid => { const d = r.dofOut[PH.dofIndex[jid]]; return (Math.abs(d.mus) / Math.max(1, d.mus >= 0 ? d.capPos : d.capNeg) * 100).toFixed(0) + '%' + (d.mus >= 0 ? '+' : '-'); };
  console.log(JSON.stringify(tv), 'grip', r.gripF[0].F.length().toFixed(0), 'N', '\n   ', rows.slice(0, 14).map(([k, a]) => `${k} ${(a * 100).toFixed(0)}`).join(', '), '\n    eff ls', eff('ls_x'), 'tl', eff('tl_x'), 't89', eff('t89_x'), 'hip', eff('hip_L_x'), 'knee', eff('kn_L_x'), 'gh', eff('gh_L_x'), '| erector', (rows.find(x => x[0] === 'erector')[1] * 100).toFixed(0), 'gluteMax', (rows.find(x => x[0] === 'gluteMax')[1] * 100).toFixed(0), 'L5S1', (r.jr.L5S1.compression / r.BW).toFixed(2));
}
