// World placement of the four poses whose position on the mat is fixed for the whole round (coordinator file).
// fold and the high plank are placed by the stepping segment (seg5) and written to cycle_keys/highplank.json / fold.json.
import fs from 'fs';
import { A, V, computeWorld, pt, LANDMARKS, rootPlaced, contactsOf, comOf } from './cycle_lib.mjs';
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const conf = id => A.poseFromData(data[id]);
const own = id => { const c = conf(id); return { c, rp: rootPlaced(c.local, c.rootQ) }; };
const cz = (W, region, side) => { const ps = contactsOf(W).filter(c => c.l.region === region && c.l.side === side).map(c => c.p); return ps.reduce((a, p) => a + p.z, 0) / ps.length; };
const out = {};
// 1. dog: as the page places it
{ const { c, rp } = own('dog'); out.dog = { rootP: rp }; }
// 2. forearm plank: wrists 5 cm ahead of the dog's wrists
{
  const d = own('dog'), p = own('plank');
  const Wd = computeWorld(d.c.local, d.c.rootQ, d.rp), Wp = computeWorld(p.c.local, p.c.rootQ, p.rp);
  const dz = (Wd.hand_L.p.z + 0.05) - Wp.hand_L.p.z;
  out.plank = { rootP: p.rp.clone().add(V(0, 0, dz)) };
}
// 3. half: pelvis 5 cm ahead of the plank's pelvis (the body pivots down about the toes)
{ const p = out.plank.rootP, h = own('half'); out.half = { rootP: V(0, h.rp.y, p.z + 0.05) }; }
// 4. bow: pelvis contact where the half's pelvis contact is
{
  const h = own('half'), b = own('bow');
  const Wh = computeWorld(h.c.local, h.c.rootQ, out.half.rootP), Wb = computeWorld(b.c.local, b.c.rootQ, b.rp);
  const dz = cz(Wh, 'таз', 'L') - cz(Wb, 'таз', 'L');
  out.bow = { rootP: b.rp.clone().add(V(0, 0, dz)) };
}
const r6 = v => +v.toFixed(6);
const res = {};
for (const id of ['dog', 'plank', 'half', 'bow']) {
  const c = conf(id);
  res[id] = { local: data[id].local, root: data[id].root, rootP: [out[id].rootP.x, out[id].rootP.y, out[id].rootP.z].map(r6) };
  const W = computeWorld(c.local, c.rootQ, out[id].rootP);
  const g = {};
  for (const ct of contactsOf(W)) { const k = ct.l.region + ':' + ct.l.side; (g[k] ||= []).push(ct.p); }
  console.log(id.padEnd(6), 'rootP', res[id].rootP.join(', '), '| com z', comOf(W).z.toFixed(3), '| wrist z', W.hand_L.p.z.toFixed(3), 'elbow z', W.fore_L.p.z.toFixed(3), 'shoulder z', W.hum_L.p.z.toFixed(3), 'ankle z', W.foot_L.p.z.toFixed(3));
  console.log('       contacts', Object.entries(g).map(([k, ps]) => k + ' z ' + (ps.reduce((a, p) => a + p.z, 0) / ps.length).toFixed(3) + ' x ' + (ps.reduce((a, p) => a + p.x, 0) / ps.length).toFixed(3)).join(' | '));
}
fs.writeFileSync('cycle_keys/anchors.json', JSON.stringify(res));
