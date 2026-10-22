// compare model muscle/ligament path changes with the fascia strands (debug for atlas_build.mjs)
import fs from 'fs';
import { A } from '../cycle_lib.mjs';
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const POSES = ['dog', 'plank', 'half', 'bow', 'fold'];
const Ws = Object.fromEntries(['stand', ...POSES].map(id => { const c = id === 'stand' ? A.STAND : A.poseFromData(data[id]); return [id, A.placeW(c.local, c.rootQ)]; }));
const I = A.strandInstances().filter(i => i.side === 'L');
const units = process.argv.slice(2);
for (const u of units) {
  const ss = I.filter(i => i.unit === u);
  for (const s of ss) {
    const L0 = A.pathLength(A.strandPath(Ws.stand, s));
    console.log((u + ' ' + (s.label || s.idx)).padEnd(34), POSES.map(p => ((A.pathLength(A.strandPath(Ws[p], s)) / L0 - 1) * 100).toFixed(1).padStart(6)).join(' '));
  }
}
const F = JSON.parse(fs.readFileSync('books/fascia_structures.json', 'utf8'));
for (const st of F.structures) if (process.argv.includes('--f') || units.includes(st.id)) {
  st.geometry.strands.forEach((s, k) => console.log(('  ' + st.id + ' #' + k + ' ' + s.label).slice(0, 70).padEnd(70), POSES.map(p => (st.poses[p].perStrand[k] * 100).toFixed(1).padStart(6)).join(' ')));
}
