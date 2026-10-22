import * as THREE from 'three';
import { createAnatomy } from './anatomy.js';
const A = createAnatomy(THREE);
const ang = (a, b) => { const d = b.clone().sub(a); return Math.atan2(d.y, Math.hypot(d.x, d.z)) * 180 / Math.PI; };
const angB = (o, a, b) => a.clone().sub(o).angleTo(b.clone().sub(o)) * 180 / Math.PI;
function measure(p) {
  const dog = A.buildDog(p);
  const W = A.poseAt(dog, 1).W;
  const hip = W.thigh_L.p, ankle = W.foot_L.p, gh = W.hum_L.p, wr = W.hand_L.p, knee = W.shank_L.p;
  const ghMid = gh.clone().add(W.hum_R.p).multiplyScalar(0.5);
  return { legs: ang(ankle, hip), trunk: ang(ghMid, W.pelvis.p), arms: ang(wr, gh), hipA: angB(hip, knee, gh), back: angB(gh, hip, wr), headY: A.pt(W, 'head', A.V(0, 1.672, 0.008)).y };
}
let best = null;
for (const thFlex of [170, 176, 182, 188, 194])
  for (const hipFlex of [104, 108, 112, 116])
    for (const sc of [1, 1.5, 2, 2.5]) {
      const p = { thFlex, hipFlex, lumA: -3 * sc, lumB: -3 * sc, thA: -4 * sc, thB: -5 * sc, thC: -4 * sc };
      const m = measure(p);
      const err = (m.trunk - 54) ** 2 + (m.arms - 34) ** 2 + (m.legs - 51.5) ** 2 + ((m.back - 160) / 1.5) ** 2;
      if (!best || err < best.err) best = { err, p, m };
    }
console.log(JSON.stringify(best, null, 1));
