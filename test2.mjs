import * as THREE from 'three';
import { createAnatomy } from './anatomy2.js';
const A = createAnatomy(THREE);
const dog = A.buildDog();
const P1 = A.poseAt(dog, 1), P0 = A.poseAt(dog, 0);
console.log('bones', A.BONES.length, 'joints', A.JOINTS.length);
const inst = A.strandInstances();
console.log('strands', inst.length, 'muscles', inst.filter(i => i.kind === 'muscle').length, 'ligs', inst.filter(i => i.kind === 'lig').length, 'caps', inst.filter(i => i.kind === 'cap').length);
// joints
for (const j of A.JOINTS) {
  if (j.side === 'R') continue;
  const r = A.analyzeJoint(j, P1.local, P1.W);
  console.log(j.id.padEnd(5), r.comps.map(c => c.label + ' ' + c.value.toFixed(1) + '/' + c.rom).join(' | '), 'total', r.total.toFixed(1));
}
console.log('regions', JSON.stringify(A.analyzeRegions(P1.local)));
// neutral check: all zero?
let maxNeutral = 0;
for (const j of A.JOINTS) { const r = A.analyzeJoint(j, P0.local, P0.W); for (const c of r.comps) maxNeutral = Math.max(maxNeutral, Math.abs(c.value)); }
console.log('max neutral comp', maxNeutral.toFixed(3));
// strains by unit
const units = {};
for (const m of inst) {
  if (m.side !== 'L') continue;
  const L0 = A.pathLength(A.strandPath(P0.W, m)), L1 = A.pathLength(A.strandPath(P1.W, m));
  (units[m.kind + ':' + m.unit] ||= []).push(((L1 / L0 - 1) * 100).toFixed(0));
}
for (const [u, arr] of Object.entries(units)) console.log(u.padEnd(18), arr.join(' '));
// R-L symmetry check
const sym = [];
for (const m of inst.filter(i => i.side === 'L')) {
  const r = inst.find(i => i.id === m.id.replace(/_L$/, '_R'));
  const a = A.pathLength(A.strandPath(P1.W, m)), b = A.pathLength(A.strandPath(P1.W, r));
  if (Math.abs(a - b) > 1e-6) sym.push(m.id + ' ' + (a - b).toFixed(5));
}
console.log('asym', sym.length, sym.slice(0, 5));
