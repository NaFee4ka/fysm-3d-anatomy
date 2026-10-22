// seg3 diagnostics: arm joint components, hand clearance from the floor and from the legs/pelvis, COM margin
import fs from 'fs';
import * as K from './cycle_seg3_lib.mjs';
const { A, V, THREE, computeWorld, pt, LANDMARKS, comOf } = K;
const seg = JSON.parse(fs.readFileSync(process.argv[2] || 'cycle_keys/seg3.json', 'utf8'));
const qa = a => new THREE.Quaternion(...a);
const J = id => A.JOINTS.find(j => j.id === id);
const handLm = LANDMARKS.filter(l => l.side === 'L' && /^(hand|f\d[a-c]|f1m|f1a|f1b)_L$/.test(l.seg));
const foreLm = LANDMARKS.filter(l => l.side === 'L' && /^(fore|ulna)_L$/.test(l.seg));
// rough body capsules (both legs, pelvis): [segA joint, segB joint, radius]
const caps = W => [
  ['thigh_L', W.thigh_L.p, W.shank_L.p, 0.075], ['thigh_R', W.thigh_R.p, W.shank_R.p, 0.075],
  ['shank_L', W.shank_L.p, W.foot_L.p, 0.045], ['shank_R', W.shank_R.p, W.foot_R.p, 0.045],
  ['glute', W.thigh_L.p.clone().add(W.thigh_R.p).multiplyScalar(0.5), W.lumA.p, 0.11],
];
const segDist = (p, a, b) => { const ab = b.clone().sub(a); const t = Math.max(0, Math.min(1, p.clone().sub(a).dot(ab) / ab.lengthSq())); return p.distanceTo(a.clone().addScaledVector(ab, t)); };
for (const fr of seg.frames.filter((f, i) => i % (+process.argv[3] || 2) === 0 || f.t === seg.duration)) {
  const L = {}; for (const [s, v] of Object.entries(fr.local)) L[s] = qa(v);
  const W = computeWorld(L, qa(fr.root), V(...fr.rootP));
  const c = n => { const r = A.analyzeJoint(J(n), L, W); return Object.fromEntries(r.comps.map(x => [x.key, Math.round(x.signed)])); };
  const gh = c('gh_L'), el = c('el_L'), wr = c('wr_L'), kn = c('kn_L'), hip = c('hip_L');
  let floorMin = Infinity; for (const l of handLm.concat(foreLm)) floorMin = Math.min(floorMin, pt(W, l.seg, l.pc).y);
  let bodyMin = Infinity, bodyWho = '';
  for (const l of handLm.concat(foreLm)) { const p = pt(W, l.seg, l.pc); for (const [nm, a, b, r] of caps(W)) { if (nm === 'shank_L' && fr.grip.length) continue; const d = segDist(p, a, b) - r; if (d < bodyMin) { bodyMin = d; bodyWho = nm + ':' + l.region + (l.tag ? ':' + l.tag : ''); } } }
  const hd = W.hand_L.p.distanceTo(W.hand_R.p);
  console.log(String(fr.t).padStart(4), 'gh', JSON.stringify(gh), 'el', JSON.stringify(el), 'wr', JSON.stringify(wr), 'hip', hip.flex, 'kn', kn.flex, '| hand floor', (floorMin * 1000).toFixed(0), 'mm, body gap', (bodyMin * 100).toFixed(1), 'cm', bodyWho, '| wrists apart', (hd * 100).toFixed(1), 'cm', fr.grip.length ? 'GRIP' : '');
}
