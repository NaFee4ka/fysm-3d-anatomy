// Joins the five segments of the round into cycle_keys/cycle.json (coordinator file).
// usage: node cycle_build.mjs [--prov]   (--prov: a missing segN.json is replaced by prov_segN.json)
import fs from 'fs';
import { A, V, THREE, computeWorld, pt, LANDMARKS } from './cycle_lib.mjs';
import { validateMotionCollisions } from './cycle_collision_check.mjs';

const prov = process.argv.includes('--prov');
const ORDER = ['seg1', 'seg2', 'seg3', 'seg4', 'seg5'];
const POSE_NAME = { dog: 'Горка', plank: 'Планка на предплечьях', half: 'Полупрогиб', bow: 'Корзинка', fold: 'Наклон', highplank: 'Планка на ладонях' };
const segs = [];
for (const id of ORDER) {
  const f = `cycle_keys/${id}.json`, pf = `cycle_keys/prov_${id}.json`;
  if (fs.existsSync(f)) segs.push(JSON.parse(fs.readFileSync(f, 'utf8')));
  else if (prov && fs.existsSync(pf)) segs.push(JSON.parse(fs.readFileSync(pf, 'utf8')));
  else throw new Error('missing ' + f);
}
const frames = [], segInfo = [];
let T = 0;
const r2 = v => +v.toFixed(2);
segs.forEach((s, k) => {
  const t0 = T;
  s.frames.forEach((fr, i) => {
    if (k > 0 && i === 0) return; // same configuration as the last frame of the previous segment
    frames.push({ ...fr, T: r2(t0 + fr.t), seg: s.id });
  });
  T = t0 + s.duration;
  segInfo.push({ id: s.id, from: s.from, to: s.to, fromName: POSE_NAME[s.from], toName: POSE_NAME[s.to], t0: r2(t0), t1: r2(T), provisional: !!s.provisional, phases: s.phases.map(p => ({ t0: r2(t0 + p.t0), t1: r2(t0 + p.t1), label: p.label })) });
});
// centre the whole round on the mat (along z)
let zmin = Infinity, zmax = -Infinity, ymin = Infinity;
const q = a => new THREE.Quaternion(...a);
for (const fr of frames) {
  const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = q(v);
  const W = computeWorld(L, q(fr.root), V(...fr.rootP));
  for (const l of LANDMARKS) { const p = pt(W, l.seg, l.pc); zmin = Math.min(zmin, p.z); zmax = Math.max(zmax, p.z); ymin = Math.min(ymin, p.y); }
}
const dz = -(zmin + zmax) / 2;
for (const fr of frames) fr.rootP = [fr.rootP[0], fr.rootP[1], +(fr.rootP[2] + dz).toFixed(6)];
// where the five poses sit in time (holds are added by the page)
const holds = [{ pose: 'dog', T: 0 }];
segInfo.forEach(s => { if (POSE_NAME[s.to] && s.to !== 'highplank') holds.push({ pose: s.to, T: s.t1 }); });
const out = { fps: 10, T: r2(T), shiftZ: +dz.toFixed(4), footprint: [+(zmin + dz).toFixed(3), +(zmax + dz).toFixed(3)], minY: +ymin.toFixed(4), segs: segInfo, holds, frames };
// Reject crossed arm cores before spending time on the force calculation.
// The packer subsequently checks actual bone surfaces after quantization.
await validateMotionCollisions(out, { preflight: true });
fs.writeFileSync('cycle_keys/cycle.json', JSON.stringify(out));
console.log('cycle.json', frames.length, 'frames,', r2(T), 's, footprint', out.footprint, 'shift', out.shiftZ, 'minY', out.minY, segInfo.map(s => s.id + (s.provisional ? '(prov)' : '')).join(' '));
