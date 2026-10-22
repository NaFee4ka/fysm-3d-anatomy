import * as THREE from 'three';
import { createAnatomy } from './anatomy.js';
const A = createAnatomy(THREE);
const { V, pt, S } = A;
const inst = A.muscleInstances().filter(m => m.side === 'L');
for (const cfg of [
  { scapPivot: [0.095, 1.385, -0.107], scapUp: 48, scapProt: 8 },
  { scapPivot: [0.12, 1.395, -0.085], scapUp: 48, scapProt: 5 },
  { scapPivot: [0.12, 1.395, -0.085], scapUp: 44, scapProt: 4 },
  { scapPivot: [0.135, 1.40, -0.075], scapUp: 44, scapProt: 4 },
]) {
  const dog = A.buildDog(Object.assign({ thFlex: 188, hipFlex: 116 }, cfg));
  const P1 = A.poseAt(dog, 1), P0 = A.poseAt(dog, 0);
  const th = (Wx, seg, p) => pt(Wx, seg, p).sub(Wx.thC.p).applyQuaternion(Wx.thC.q.clone().invert()).add(S.thC.J);
  const inf = th(P1.W, 'scap_L', V(0.091, 1.25, -0.1045));
  const ac = th(P1.W, 'scap_L', S.scap_L.J);
  const sc = A.quatAngleDeg(P1.local.clav_L), st = A.quatAngleDeg(P1.local.clav_L.clone().multiply(P1.local.scap_L)), gh = A.quatAngleDeg(P1.local.hum_L);
  const sp = A.scapulaPose(cfg.scapUp, 15, cfg.scapProt, V(...cfg.scapPivot));
  const acErr = sp.tr(S.scap_L.J).distanceTo(ac);
  const strains = {};
  for (const m of inst) { const L0 = A.pathLength(A.musclePath(P0.W, m)), L1 = A.pathLength(A.musclePath(P1.W, m)); (strains[m.unit] ||= []).push(((L1 / L0 - 1) * 100).toFixed(0)); }
  console.log(JSON.stringify(cfg), 'inf', inf.toArray().map(v => v.toFixed(3)).join(','), 'AC rise', (ac.y - S.scap_L.J.y).toFixed(3), 'acErr', acErr.toFixed(3), 'sc', sc.toFixed(1), 'st', st.toFixed(1), 'gh', gh.toFixed(1));
  console.log('   ', ['pecMin', 'serratus', 'trapUp', 'trapMid', 'trapLow', 'rhomb', 'lat', 'teresMaj', 'pecMaj', 'teresMin', 'infra', 'supra', 'deltMid'].map(u => u + ' ' + strains[u].join('/')).join(' | '));
}
