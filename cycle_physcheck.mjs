// Physics check of one segment (or the whole round) — where the static model needs help it can't get from muscles
// (agent phys). Large reserve torques mean the posture can't be held by muscles in that frame, most often because a joint
// is past its anatomical range (passive tissues push back) or the body is off balance.
// usage: node cycle_physcheck.mjs cycle_keys/seg3.json [--every 2] [--thr 5]
//   prints per frame: total reserve torque (N·m), the dofs that need it (with the passive part), residual, balance margin,
//   strongest muscles, and the hand / elbow / knee / ankle angles of the left and right side.
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE } from './physics_params.mjs';

const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf(k); return i >= 0 && i + 1 < args.length ? args[i + 1] : d; };
const file = args[0], EVERY = +arg('--every', 1), THR = +arg('--thr', 5);
const seg = JSON.parse(fs.readFileSync(file, 'utf8'));
const frames = seg.frames;
const q = a => new THREE.Quaternion(...a);
const D = 180 / Math.PI;
const euler = qq => { const e = new THREE.Euler().setFromQuaternion(qq, 'XYZ'); return [e.x * D, e.y * D, e.z * D].map(v => Math.round(v)); };
const hinge = (W, a, b, c) => Math.round(W[b].p.clone().sub(W[a].p).angleTo(W[c].p.clone().sub(W[b].p)) * D);
let prev = null, bad = 0;
const T0 = Date.now();
for (let i = 0; i < frames.length; i++) {
  if (i % EVERY && i !== frames.length - 1) continue;
  const fr = frames[i];
  const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = q(v);
  const rootQ = q(fr.root), rootP = new THREE.Vector3(...fr.rootP);
  const W = A.computeWorld(L, rootQ, rootP);
  const grips = (fr.grip || []).map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: 'hum_' + sd, cone: 1 }; });
  // cue tone: the nearer end pose's cues (enough for a check)
  const tone = CUE_TONE[fr.t < (seg.duration || 0) / 2 ? seg.from : seg.to] || {};
  const r = PH.solve({ local: L, rootQ }, { rootP, grips, tone, iters: 16000, warm: prev && prev.warm });
  prev = r;
  const resTot = Object.values(r.reserve).reduce((a, v) => a + Math.abs(v), 0);
  const mus = {}; r.INST.forEach((s, j) => { const k = s.unit + '_' + s.side; const m = (mus[k] ||= { F: 0, Fm: 0 }); m.F += r.force[j]; m.Fm += r.FMAX[j]; });
  const top = Object.entries(mus).map(([k, m]) => [k, m.F / m.Fm]).sort((a, b) => b[1] - a[1]).slice(0, 4).map(([k, v]) => k + ' ' + Math.round(v * 100) + '%').join(', ');
  const ang = sd => `hand ${euler(L['hand_' + sd]).join('/')} el ${hinge(W, 'hum_' + sd, 'fore_' + sd, 'hand_' + sd)} kn ${hinge(W, 'thigh_' + sd, 'shank_' + sd, 'foot_' + sd)} an ${hinge(W, 'shank_' + sd, 'foot_' + sd, 'toes_' + sd)}`;
  let line = `${String(i).padStart(3)} t=${String(fr.t).padEnd(4)} reserves ${resTot.toFixed(1).padStart(6)} N·m  resid ${r.maxRes.toExponential(1)}  iters ${String(r.iters).padStart(5)}  | ${top}`;
  if (resTot > THR || r.maxRes > 1) {
    bad++;
    const dofs = r.dofOut.filter(d => Math.abs(d.res) > 1).sort((a, b) => Math.abs(b.res) - Math.abs(a.res)).slice(0, 4).map(d => `${d.id} ${d.res.toFixed(1)} (passive ${d.pas.toFixed(1)}, load ${d.ext.toFixed(1)}, muscles could ${d.capPos.toFixed(0)}/−${d.capNeg.toFixed(0)})`);
    line += `\n      ⚠ ${dofs.join('; ')}\n      L: ${ang('L')} | R: ${ang('R')}`;
  }
  console.log(line);
}
console.log(`${file}: ${bad} frame(s) over ${THR} N·m of reserves or residual > 1; ${((Date.now() - T0) / 1000).toFixed(1)} s`);
console.log('hand = local Euler XYZ of the wrist (X ≈ radial/ulnar deviation, Z ≈ flexion/extension; anatomical deviation ≈ −35…+20°, flex/ext ≈ ±70–85°)');
