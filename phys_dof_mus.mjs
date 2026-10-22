// which muscles act on a given dof in a pose, with their moment
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE, PASSIVE_GROUPS } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2]; const dofs = process.argv[3].split(','); const opt = process.argv[4] ? JSON.parse(process.argv[4]) : {};
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
const W = A.placeW(conf.local, conf.rootQ);
const grips = (id === 'bow' || id === 'fold') ? ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: opt.cone != null ? 'hum_' + sd : undefined, cone: opt.cone }; }) : [];
const tone = Object.assign({}, CUE_TONE[id] || {}, opt.tone || {});
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone, grips, iters: 16000, passiveGroups: opt.pg ? PASSIVE_GROUPS : null });
for (const dn of dofs) {
  const k = PH.dofIndex[dn]; const d = r.dofOut[k];
  console.log('==', dn, 'ext', d.ext.toFixed(2), 'mus', d.mus.toFixed(2), 'pas', d.pas.toFixed(2));
  const by = {};
  r.INST.forEach((s, j) => { const rr = r.R[j][k]; if (!rr || s.side !== 'L') return; const b = (by[s.unit] ||= { M: 0, F: 0, r: 0, n: 0 }); b.M += rr * r.force[j]; b.F += r.force[j]; b.r += rr; b.n++; });
  console.log(Object.entries(by).sort((a, b) => Math.abs(b[1].M) - Math.abs(a[1].M)).slice(0, 14).map(([u, b]) => `${u} M=${b.M.toFixed(2)} F=${Math.round(b.F)} r=${(b.r / b.n * 100).toFixed(2)}cm`).join('\n'));
}
