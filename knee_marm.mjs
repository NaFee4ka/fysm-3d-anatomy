import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const joint = process.argv[2] || 'kn';
const cfg = { kn: { seg: 'shank_L', dof: 'kn_L_x', axis: V(1, 0, 0), units: 'bicFemL,bicFemS,semiT,semiM,gracilis,sartorius,gastroM,gastroL,popliteus,plantaris,rectFem,vastLat,vastMed,vastInt', angles: [0, 30, 60, 90, 100, 110, 125] },
              el: { seg: 'fore_L', dof: 'el_L_x', axis: V(1, 0, 0), units: 'bicLong,bicShort,brachialis,brachiorad,pronTeres,tricLong,tricLat,tricMed,anconeus,wristFlex,wristExt,fingerFlex', angles: [0, 30, 60, 90, 120, 135, 147] } }[joint];
const units = cfg.units.split(',');
for (const ang of cfg.angles) {
  const local = {}; for (const k in A.STAND.local) local[k] = A.STAND.local[k].clone();
  // knee flexion: shank rotates about X so the foot goes back (+X flex): use the same sign as poses (qAx(X, +flex)) ; elbow flexion: forearm forward (qAx(X, -flex))
  const sgn = joint === 'kn' ? 1 : -1;
  local[cfg.seg] = new THREE.Quaternion().setFromAxisAngle(cfg.axis, sgn * ang * Math.PI / 180);
  if (joint === 'el') local.ulna_L = new THREE.Quaternion();
  const W = A.placeW(local, A.STAND.rootQ);
  const k = PH.dofIndex[cfg.dof], ax = PH.DOF[k].axis(W), c = PH.DOF[k].center(W), d = 0.3 * Math.PI / 180;
  const Wp = PH.rotW(W, k, d, ax, c), Wm = PH.rotW(W, k, -d, ax, c);
  const by = {};
  for (const m of PH.INST) { if (m.side !== 'L' || !units.includes(m.unit)) continue; const r = -(A.pathLength(A.strandPath(Wp, m)) - A.pathLength(A.strandPath(Wm, m))) / (2 * d); (by[m.unit] ||= []).push((r * 100).toFixed(1)); }
  console.log(String(ang).padStart(4) + '°  ' + Object.entries(by).map(([u, v]) => u + ' ' + v.join('/')).join('  '));
}
