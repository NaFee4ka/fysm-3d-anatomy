import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const V = (x, y, z) => new THREE.Vector3(x, y, z);
for (const id of ['stand', 'dog', 'plank', 'half', 'bow', 'fold']) {
  const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
  const W = A.placeW(conf.local, conf.rootQ);
  const grips = (id === 'bow' || id === 'fold') ? ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: 'hum_' + sd, cone: 1 }; }) : [];
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone: CUE_TONE[id] || {}, grips, iters: 16000 });
  const set = PH.subtree('lumA');
  const axis = V(0, 1, 0).applyQuaternion(r.W.lumA.q);
  const along = F => -F.dot(axis); // compression positive (reaction = -sum of external on the upper body)
  let grav = V(0, 0, 0); for (const m of A.MASS) if (set.has(m.seg)) grav.y -= m.m * r.BW;
  let cont = V(0, 0, 0); for (const c of r.contactF) if (set.has(c.seg)) cont.add(c.F);
  for (const g of r.gripF) { if (set.has(g.hand)) cont.add(g.F); if (set.has(g.shank)) cont.sub(g.F); }
  const byU = {};
  r.INST.forEach((s, j) => { const f = r.force[j] + r.FP[j]; if (f < 0.5) return; const F = PH.pullOn(r.paths[j], set, f); const c = along(F); if (Math.abs(c) > 0.5) byU[s.unit] = (byU[s.unit] || 0) + c; });
  const tot = along(grav) + along(cont) + Object.values(byU).reduce((a, b) => a + b, 0);
  console.log(id.padEnd(6), 'total', Math.round(tot), '| gravity', Math.round(along(grav)), 'contacts', Math.round(along(cont)), '| muscles', Math.round(Object.values(byU).reduce((a, b) => a + b, 0)), ':', Object.entries(byU).sort((a, b) => b[1] - a[1]).slice(0, 7).map(([u, v]) => u + ' ' + Math.round(v)).join(', '));
}
