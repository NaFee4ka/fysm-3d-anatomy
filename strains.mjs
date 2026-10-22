import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json'));
const inst = A.strandInstances().filter(i => i.side === 'L' && i.kind !== 'cap');
const P0 = A.poseAt(A.poseFromData(data.dog), 0).W;
const L0 = inst.map(i => A.pathLength(A.strandPath(P0, i)));
const res = {};
for (const id of ['dog', 'plank', 'half', 'bow', 'fold']) {
  const c = A.poseFromData(data[id]);
  const W = A.placeW(c.local, c.rootQ);
  inst.forEach((s, k) => { const e = A.pathLength(A.strandPath(W, s)) / L0[k] - 1; const key = s.kind + ':' + s.unit; (res[key] ||= {})[id] ||= []; res[key][id].push(e); });
}
const rows = [];
for (const [k, v] of Object.entries(res)) { const avg = id => v[id].reduce((a, b) => a + b, 0) / v[id].length; rows.push([k, ...['dog', 'plank', 'half', 'bow', 'fold'].map(id => Math.round(avg(id) * 100))]); }
const flag = process.argv[2] === 'all' ? rows : rows.filter(r => r.slice(1).some(x => Math.abs(x) > 70));
console.log('unit'.padEnd(22), 'dog plank half bow fold');
for (const r of flag) console.log(r[0].padEnd(22), r.slice(1).map(x => String(x).padStart(5)).join(''));
