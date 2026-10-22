// moments produced by one muscle unit on every dof, in a pose
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const [id, unit] = process.argv.slice(2); const opt = process.argv[4] ? JSON.parse(process.argv[4]) : {};
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
const W = A.placeW(conf.local, conf.rootQ);
const grips = (id === 'bow' || id === 'fold') ? ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: 'hum_' + sd, cone: 1 }; }) : [];
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: Object.assign({}, CUE_TONE[id], opt.tone || {}), grips, iters: 16000 });
r.INST.forEach((s, j) => {
  if (s.unit !== unit || s.side !== 'L') return;
  const parts = []; for (let k = 0; k < PH.DOF.length; k++) if (r.R[j][k]) parts.push(PH.DOF[k].id + ' r=' + (r.R[j][k] * 100).toFixed(2) + 'cm M=' + (r.R[j][k] * r.force[j]).toFixed(2));
  console.log(unit + '#' + s.idx, (s.label || ''), 'F=' + r.force[j].toFixed(0) + 'N a=' + (r.act[j] * 100).toFixed(0) + '% ', parts.join(' | '));
});
