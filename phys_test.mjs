import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const t0 = Date.now();
const PH = makePhysics(A);
console.log('setup', Date.now() - t0, 'ms; dofs', PH.DOF.length, 'muscle strands', PH.INST.length);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const which = (process.argv[2] || 'stand').split(',');
for (const id of which) {
  const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
  const t1 = Date.now();
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE[id] || {}, iters: +(process.argv[3] || 20000) });
  console.log('\n==', id, (Date.now() - t1) + 'ms', 'max residual', r.maxRes.toExponential(2));
  // contacts by region/side
  const reg = {};
  for (const c of r.contactF) { const k = c.region + ' ' + c.side; reg[k] ||= V0(); reg[k].add(c.F); }
  function V0() { return new THREE.Vector3(); }
  for (const [k, F] of Object.entries(reg)) console.log('  contact', k.padEnd(28), 'N', (F.y / r.BW * 100).toFixed(1) + '%BW', 'tang', (F.x / r.BW * 100).toFixed(1), (F.z / r.BW * 100).toFixed(1));
  // activations by unit (left side)
  const u = {};
  r.INST.forEach((s, j) => { if (s.side !== 'L') return; (u[s.unit] ||= []).push([r.act[j], r.force[j], r.FMAX[j]]); });
  const rows = Object.entries(u).map(([k, arr]) => { const F = arr.reduce((a, x) => a + x[1], 0), Fm = arr.reduce((a, x) => a + x[2], 0); return [k, F / Fm, F]; }).sort((a, b) => b[1] - a[1]);
  console.log('  top muscles (L):', rows.slice(0, 18).map(([k, a, F]) => `${k} ${(a * 100).toFixed(0)}% ${F.toFixed(0)}N`).join(', '));
  const res = Object.entries(r.reserve).filter(([k, v]) => Math.abs(v) > 0.5).map(([k, v]) => `${k}:${v.toFixed(1)}`);
  console.log('  reserves >0.5 Nm:', res.join(' ') || 'none');
  const st = r.stF.map(s => (s.F.length() / r.BW * 100).toFixed(1)).join(' ');
  console.log('  ST contacts %BW:', st);
}
if (process.argv[4] === 'dof') {
  const id = which[0];
  const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE[id] || {}, iters: +(process.argv[3] || 8000) });
  for (const d of r.dofOut) if (!/_R_/.test(d.id) && d.joint !== 'root') console.log(d.id.padEnd(9), 'ext', d.ext.toFixed(1).padStart(7), 'st', (d.st||0).toFixed(1).padStart(6), 'pas', (d.pas||0).toFixed(1).padStart(6), 'mus', d.mus.toFixed(1).padStart(7), 'res', d.res.toFixed(1).padStart(6), 'cap+', d.capPos.toFixed(0).padStart(5), 'cap-', d.capNeg.toFixed(0).padStart(5));
  const k = PH.DOF.findIndex(d => d.id === (process.argv[5] || 'sc_L_x'));
  const contrib = r.INST.map((s, j) => [s.unit + '#' + s.idx + s.side, r.R[j][k] * r.force[j], r.R[j][k]]).filter(x => Math.abs(x[1]) > 0.5).sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
  console.log(process.argv[5] || 'sc_L_x', contrib.slice(0, 16).map(([n, m, ra]) => `${n} ${m.toFixed(1)}Nm (r ${(ra * 100).toFixed(1)}cm)`).join(', '));
}
