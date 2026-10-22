import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const kz = PH.dofIndex.wr_L_z, kx = PH.dofIndex.wr_L_x;
const units = (process.argv[2] || 'fingerFlex,wristFlex,wristExt,apl,palmaris').split(',');
console.log('hand joint (world, standing):', A.placeW(A.STAND.local, A.STAND.rootQ).hand_L.p.toArray().map(v => v.toFixed(3)).join(','));
for (const ext of [-60, -30, 0, 30, 55, 80]) {
  const local = {}; for (const k in A.STAND.local) local[k] = A.STAND.local[k].clone();
  local.hand_L = new THREE.Quaternion().setFromAxisAngle(V(0, 0, 1), ext * Math.PI / 180).multiply(local.hand_L || new THREE.Quaternion());
  const W = A.placeW(local, A.STAND.rootQ);
  const d = 0.3 * Math.PI / 180;
  const rows = [];
  for (const [k, nm] of [[kz, 'z'], [kx, 'x']]) {
    const ax = PH.DOF[k].axis(W), c = PH.DOF[k].center(W);
    const Wp = PH.rotW(W, k, d, ax, c), Wm = PH.rotW(W, k, -d, ax, c);
    const by = {};
    for (const m of PH.INST) { if (m.side !== 'L' || !units.includes(m.unit)) continue; const r = -(A.pathLength(A.strandPath(Wp, m)) - A.pathLength(A.strandPath(Wm, m))) / (2 * d); (by[m.unit] ||= []).push((r * 100).toFixed(1)); }
    rows.push(nm + ': ' + Object.entries(by).map(([u, v]) => u + ' ' + v.join('/')).join('  '));
  }
  console.log('wrist ext ' + ext + '°\n  ' + rows.join('\n  '));
}
