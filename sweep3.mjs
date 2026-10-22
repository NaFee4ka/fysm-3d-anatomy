import * as THREE from 'three';
import { createAnatomy } from './anatomy.js';
const A = createAnatomy(THREE);
const ang = (a, b) => { const d = b.clone().sub(a); return Math.atan2(d.y, Math.hypot(d.x, d.z)) * 180 / Math.PI; };
const angB = (o, a, b) => a.clone().sub(o).angleTo(b.clone().sub(o)) * 180 / Math.PI;
for (const [thFlex, hipFlex] of [[184, 114], [186, 114], [188, 114], [186, 116], [188, 116], [190, 116]]) {
  const dog = A.buildDog({ thFlex, hipFlex });
  const P = A.poseAt(dog, 1), W = P.W;
  const hip = W.thigh_L.p, ankle = W.foot_L.p, gh = W.hum_L.p, wr = W.hand_L.p, knee = W.shank_L.p;
  const ghMid = gh.clone().add(W.hum_R.p).multiplyScalar(0.5);
  const t = (gh.z - wr.z) / (hip.z - wr.z); const lineY = wr.y + (hip.y - wr.y) * t;
  console.log(thFlex, hipFlex, 'legs', ang(ankle, hip).toFixed(1), 'trunk', ang(ghMid, W.pelvis.p).toFixed(1), 'arms', ang(wr, gh).toFixed(1), 'hipA', angB(hip, knee, gh).toFixed(1), 'back', angB(gh, hip, wr).toFixed(1), 'sag', (lineY - gh.y).toFixed(3), 'gh', A.quatAngleDeg(P.local.hum_L).toFixed(1), 'headY', A.pt(W, 'head', A.V(0, 1.672, 0.008)).y.toFixed(3), 'heel', A.pt(W, 'foot_L', A.V(0.094, 0.009, -0.045)).y.toFixed(3));
}
