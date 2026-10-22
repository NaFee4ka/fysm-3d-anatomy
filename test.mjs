import * as THREE from 'three';
import { createAnatomy } from './anatomy.js';

const A = createAnatomy(THREE);
const { V, pt, S } = A;
const dog = A.buildDog();
console.log('phi', dog.phi.toFixed(1), 'twist', dog.twistDeg.toFixed(1));
const P = A.poseAt(dog, 1);
const W = P.W;
const g = (seg, p) => pt(W, seg, p);
const f = v => `(${v.x.toFixed(3)}, ${v.y.toFixed(3)}, ${v.z.toFixed(3)})`;
const hip = W.thigh_L.p, knee = W.shank_L.p, ankle = W.foot_L.p, gh = W.hum_L.p, el = W.fore_L.p, wr = W.hand_L.p;
console.log('hip', f(hip), 'knee', f(knee), 'ankle', f(ankle));
console.log('gh', f(gh), 'elbow', f(el), 'wrist', f(wr));
console.log('head', f(g('head', V(0, 1.672, 0.008))), 'C7', f(W.neckA.p));
console.log('heel', f(g('foot_L', V(0.094, 0.009, -0.045))), 'ball', f(g('foot_L', V(0.088, 0.006, 0.135))), 'toe', f(g('toes_L', V(0.09, 0.003, 0.19))));
console.log('palm', f(g('hand_L', V(0.2, 0.75, 0.01))), 'fingertip', f(g('hand_L', V(0.205, 0.64, 0.01))));
const ang = (a, b) => { const d = b.clone().sub(a); return Math.atan2(d.y, Math.hypot(d.x, d.z)) * 180 / Math.PI; };
const hipMid = W.pelvis.p;
console.log('leg angle (ankle->hip) vs floor', ang(ankle, hip).toFixed(1));
const ghMid = gh.clone().add(W.hum_R.p).multiplyScalar(0.5);
console.log('trunk angle (shoulder->hip) vs floor', ang(ghMid, hipMid).toFixed(1));
console.log('arm angle (wrist->shoulder) vs floor', ang(wr, gh).toFixed(1));
const angBetween = (o, a, b) => a.clone().sub(o).angleTo(b.clone().sub(o)) * 180 / Math.PI;
console.log('hip angle trunk-thigh', angBetween(hip, knee, gh).toFixed(1));
console.log('shoulder angle hips-shoulder-wrist (back side)', angBetween(gh, hip, wr).toFixed(1));
// is shoulder below the hands-hips line?
const lineY = (z) => { const a = wr, b = hip; const t = (z - a.z) / (b.z - a.z); return a.y + (b.y - a.y) * t; };
console.log('shoulder below hands-hip line by', (lineY(gh.z) - gh.y).toFixed(3));

// scapula check: inferior angle relative to thorax, neutral vs dog
const infAng = V(0.091, 1.25, -0.104);
const thoraxLocal = (Wx, seg, p) => { const w = pt(Wx, seg, p); return w.sub(Wx.thC.p).applyQuaternion(Wx.thC.q.clone().invert()).add(S.thC.J); };
const P0 = A.poseAt(dog, 0);
console.log('inf angle (thorax frame) neutral', f(thoraxLocal(P0.W, 'scap_L', infAng)), 'dog', f(thoraxLocal(W, 'scap_L', infAng)));
const glen = V(0.16, 1.388, -0.012);
console.log('glenoid (thorax frame) neutral', f(thoraxLocal(P0.W, 'scap_L', glen)), 'dog', f(thoraxLocal(W, 'scap_L', glen)));
// humerus direction relative to thorax
const humDir = (Wx) => { const a = pt(Wx, 'hum_L', V(0.185, 1.395, 0)), b = pt(Wx, 'hum_L', V(0.205, 1.07, 0)); return b.sub(a).normalize().applyQuaternion(Wx.thC.q.clone().invert()); };
console.log('humerus dir in thorax frame', f(humDir(W)), 'elev deg', (humDir(W).angleTo(V(0, -1, 0)) * 180 / Math.PI).toFixed(1));

// joint angles
for (const j of A.JOINTS) {
  if (j.side === 'R') continue;
  let q = P.local[j.seg];
  if (j.special === 'st') q = P.local['clav_L'].clone().multiply(P.local['scap_L']);
  console.log('joint', j.id.padEnd(8), A.quatAngleDeg(q).toFixed(1));
}
// muscle strains
const inst = A.muscleInstances();
const len = (Wx, m) => A.pathLength(A.musclePath(Wx, m));
const units = {};
for (const m of inst) {
  if (m.side !== 'L') continue;
  const L0 = len(P0.W, m), L1 = len(W, m);
  (units[m.unit] ||= []).push((L1 / L0 - 1) * 100);
}
for (const [u, arr] of Object.entries(units)) console.log('strain', u.padEnd(10), arr.map(x => x.toFixed(1)).join(', '));
