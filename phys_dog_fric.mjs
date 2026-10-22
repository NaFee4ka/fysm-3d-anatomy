import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE, FRICTION } from './physics_params.mjs';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2] || 'dog';
const conf = A.poseFromData(data[id]);
for (const [label, mu, wT] of [['mu1 wT.3', 1.0, 0.3], ['mu.5 wT.3', 0.5, 0.3], ['mu.3', 0.3, 0.3], ['mu.15', 0.15, 0.3], ['mu1 wT3', 1.0, 3]]) {
  for (const k of Object.keys(FRICTION)) if (FRICTION[k] >= 0.9 || FRICTION[k] === undefined) {}
  const saved = { ...FRICTION };
  for (const k of ['ладони', 'пальцы рук', 'пятки', 'подушечки стоп', 'пальцы стоп']) FRICTION[k] = mu;
  const PH = makePhysics(A, { mass: 70 });
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE[id] || {}, iters: 16000, wT });
  Object.assign(FRICTION, saved);
  const mus = {};
  let cost = 0;
  r.INST.forEach((s, j) => { cost += r.FMAX[j] / 1000 * r.act[j] ** 2; if (s.side !== 'L') return; const m = (mus[s.unit] ||= { F: 0, Fmax: 0 }); m.F += r.force[j]; m.Fmax += r.FMAX[j]; });
  const fl = {}; for (const c of r.contactF) { const k = c.region; const g = (fl[k] ||= { y: 0, x: 0, z: 0 }); g.y += c.F.y; g.x += c.F.x; g.z += c.F.z; }
  console.log('==', label, 'muscle cost', cost.toFixed(3), '| floor', Object.entries(fl).filter(([, v]) => v.y > 1).map(([k, v]) => `${k} N${Math.round(v.y / r.BW * 100)} Tz${Math.round(v.z / r.BW * 100)} Tx${Math.round(v.x / r.BW * 100)}`).join(', '));
  console.log('  ', Object.entries(mus).map(([u, m]) => [u, m.F / m.Fmax]).sort((a, b) => b[1] - a[1]).slice(0, 14).map(([u, a]) => u + ' ' + Math.round(a * 100)).join(', '));
  const d = n => r.dofOut[PH.dofIndex[n]];
  console.log('   hip_x', d('hip_L_x').mus.toFixed(1), 'ls_x', d('ls_x').mus.toFixed(1), 'gh_x', d('gh_L_x').mus.toFixed(1), 'el', d('el_L_x').mus.toFixed(1), 'kn', d('kn_L_x').mus.toFixed(1), 'an', d('an_L_x').mus.toFixed(1), 'resid', r.maxRes.toExponential(1));
}
