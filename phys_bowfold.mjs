import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
for (const id of (process.argv[2] || 'bow,fold').split(',')) {
  const conf = A.poseFromData(data[id]);
  const W = A.placeW(conf.local, conf.rootQ);
  const grips = ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd }; });
  const t0 = Date.now();
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE[id], grips, iters: 12000 });
  const reg = {};
  for (const c of r.contactF) { const k = c.region + (c.side === 'C' ? '' : ' ' + c.side); reg[k] = (reg[k] || V0()).add(c.F); }
  function V0() { return new THREE.Vector3(); }
  const u = {};
  r.INST.forEach((s, j) => { if (s.side !== 'L') return; (u[s.unit] ||= [0, 0, 0]); u[s.unit][0] += r.force[j]; u[s.unit][1] += r.FMAX[j]; u[s.unit][2] += r.FP[j]; });
  const rows = Object.entries(u).map(([k, [F, Fm, Fp]]) => [k, F / Fm, Fp]).sort((a, b) => b[1] - a[1]);
  console.log('==', id, Date.now() - t0, 'ms resid', r.maxRes.toExponential(1));
  console.log('   floor', Object.entries(reg).filter(([k]) => !/ R$/.test(k)).map(([k, F]) => `${k} ${(F.y / r.BW * 100).toFixed(1)}% (tang ${(F.x / r.BW * 100).toFixed(1)}, ${(F.z / r.BW * 100).toFixed(1)})`).join(', '));
  console.log('   grip L', r.gripF[0].F.toArray().map(v => (v / r.BW * 100).toFixed(1)).join(', '), '%BW', (r.gripF[0].F.length()).toFixed(0), 'N');
  console.log('   ', rows.slice(0, 18).map(([k, a, Fp]) => `${k} ${(a * 100).toFixed(0)}${Fp > 20 ? '(+' + Fp.toFixed(0) + 'N pass)' : ''}`).join(', '));
  const pas = rows.filter(r => r[2] > 30).map(([k, a, Fp]) => `${k} ${Fp.toFixed(0)}N`);
  console.log('    passive >30N:', pas.join(', '));
  console.log('    reserves', Object.entries(r.reserve).filter(([k, x]) => Math.abs(x) > 0.5 && !/_R_/.test(k)).map(([k, x]) => `${k}:${x.toFixed(1)}`).join(' ') || 'none', '| ST', r.stF.filter(s => s.side === 'L').map(s => (s.F.length() / r.BW * 100).toFixed(0)).join('/'), '| L5S1', (r.jr.L5S1.compression / r.BW).toFixed(2), 'BW shear', (r.jr.L5S1.shear / r.BW).toFixed(2));
  for (const d of r.dofOut) if (/^(ls|tl|t89|hip_L|kn_L|an_L|gh_L|el_L|sc_L)_x$/.test(d.id) || d.id === 'wr_L_z') console.log('     ', d.id.padEnd(8), 'ext', d.ext.toFixed(1).padStart(7), 'pas', d.pas.toFixed(1).padStart(6), 'mus', d.mus.toFixed(1).padStart(7), 'cap', (d.mus >= 0 ? d.capPos : d.capNeg).toFixed(0).padStart(4), 'eff', (Math.abs(d.mus) / Math.max(1, d.mus >= 0 ? d.capPos : d.capNeg) * 100).toFixed(0) + '%');
}
