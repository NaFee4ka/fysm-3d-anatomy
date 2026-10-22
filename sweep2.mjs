import * as THREE from 'three';
import { createAnatomy } from './anatomy.js';
const A = createAnatomy(THREE);
const ang = (a, b) => { const d = b.clone().sub(a); return Math.atan2(d.y, Math.hypot(d.x, d.z)) * 180 / Math.PI; };
const angB = (o, a, b) => a.clone().sub(o).angleTo(b.clone().sub(o)) * 180 / Math.PI;
function measure(p) {
  const dog = A.buildDog(p);
  const P = A.poseAt(dog, 1), W = P.W;
  const hip = W.thigh_L.p, ankle = W.foot_L.p, gh = W.hum_L.p, wr = W.hand_L.p, knee = W.shank_L.p;
  const ghMid = gh.clone().add(W.hum_R.p).multiplyScalar(0.5);
  const t = (gh.z - wr.z) / (hip.z - wr.z); const lineY = wr.y + (hip.y - wr.y) * t;
  return { legs: ang(ankle, hip), trunk: ang(ghMid, W.pelvis.p), arms: ang(wr, gh), hipA: angB(hip, knee, gh), back: angB(gh, hip, wr), sag: lineY - gh.y, gh: A.quatAngleDeg(P.local.hum_L), headY: A.pt(W, 'head', A.V(0, 1.672, 0.008)).y };
}
const res = [];
for (const thFlex of [180, 184, 188, 192, 196])
  for (const hipFlex of [110, 113, 116, 119])
    for (const sc of [1, 1.5, 2])
      for (const scapUp of [42, 48, 54]) {
        const p = { thFlex, hipFlex, scapUp, lumA: -3 * sc, lumB: -3 * sc, thA: -4 * sc, thB: -5 * sc, thC: -4 * sc };
        const m = measure(p);
        const err = (m.trunk - 54) ** 2 + (m.arms - 34) ** 2 + (m.legs - 51.5) ** 2 + ((m.back - 160) / 1.5) ** 2 + ((m.sag - 0.1) * 100) ** 2;
        res.push({ err, p, m });
      }
res.sort((a, b) => a.err - b.err);
for (const r of res.slice(0, 6)) console.log(r.err.toFixed(1), JSON.stringify(r.p), Object.entries(r.m).map(([k, v]) => k + '=' + v.toFixed(3)).join(' '));
