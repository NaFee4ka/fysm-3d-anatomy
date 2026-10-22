import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const units = 'gluteMax,bicFemL,semiM,psoas,iliacus,rectFem,sartorius,tfl,pectineus,addLong,addMag,gluteMed,gluteMin'.split(',');
for (const ang of [0, 30, 60, 90, 100, 116, 128]) {
  const local = {}; for (const k in A.STAND.local) local[k] = A.STAND.local[k].clone();
  local.thigh_L = new THREE.Quaternion().setFromAxisAngle(V(1, 0, 0), -ang * Math.PI / 180);
  const W = A.placeW(local, A.STAND.rootQ);
  const k = PH.dofIndex.hip_L_x, ax = PH.DOF[k].axis(W), c = PH.DOF[k].center(W), d = 0.3 * Math.PI / 180;
  const Wp = PH.rotW(W, k, d, ax, c), Wm = PH.rotW(W, k, -d, ax, c);
  const by = {};
  for (const m of PH.INST) { if (m.side !== 'L' || !units.includes(m.unit)) continue; const r = -(A.pathLength(A.strandPath(Wp, m)) - A.pathLength(A.strandPath(Wm, m))) / (2 * d); (by[m.unit] ||= []).push((r * 100).toFixed(1)); }
  console.log(String(ang).padStart(4) + '°  ' + Object.entries(by).map(([u, v]) => u + ' ' + v.join('/')).join('  '));
}
