// A detour must change the colliding parts relative to one another; moving
// unrelated toes or a common ancestor cannot repair their known contact.
import assert from 'node:assert/strict';
import { A,THREE } from './cycle_lib.mjs';
import { detourFamiliesForPair,findTransitionDetour } from './transition_path.mjs';

let cases=0;
for(const [a,b,expected]of [
  ['shank_L','foot_L',['foot']],
  ['foot_R','shank_R',['foot']],
  ['foot_L','toes_L',['toes']],
  ['pelvis','thigh_R',['thigh']],
  ['foot_L','foot_R',['foot','thigh']],
  ['toes_L','toes_R',['toes','foot','thigh']],
  ['shank_L','shank_R',['thigh']],
  ['thigh_L','shank_L',[]],
  ['fore_L','shank_L',['thigh']],
  ['hum_L','pelvis',[]],
  ['hand_L','f1b_L',[]],
  ['foot_L','foot_L',[]],
]) {
  assert.deepEqual(detourFamiliesForPair(A,{a,b}),expected,a+' / '+b);
  assert.deepEqual(detourFamiliesForPair(A,{a:b,b:a}),expected,'pair order must not change scope');
  cases++;
}
for(const pair of [null,{}, {a:'unknown',b:'foot_L'},{a:'__proto__',b:'foot_L'}]) {
  assert.deepEqual(detourFamiliesForPair(A,pair),[],'unknown topology cannot authorize a detour');cases++;
}
const cyclic={S:{...A.S,foot_L:{...A.S.foot_L,parent:'toes_L'}}};
assert.deepEqual(detourFamiliesForPair(cyclic,{a:'shank_L',b:'foot_L'}),[],'cyclic ancestry must fail closed');cases++;

const conf=angle=>({local:Object.fromEntries(A.SEG.map(s=>[s.name,new THREE.Quaternion()])),
  rootQ:new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),angle),
  rootP:new THREE.Vector3(.1,.2,.3)});
const from=conf(.2),to=conf(.5),before=JSON.stringify([from,to]);
let edgeCalls=0;
const path=findTransitionDetour({A,THREE,from,to,families:['foot'],collision:()=>false,
  clearEdge:()=>{edgeCalls++;return true;}});
assert(path&&path.length===1&&edgeCalls===2,'a requested family gets a checked two-edge candidate');
for(const c of path) for(const segment of A.SEG) {
  if(/^foot_[LR]$/.test(segment.name)) assert(Math.abs(c.local[segment.name].y)>0,'requested feet must actually change');
  else assert(c.local[segment.name].equals(from.local[segment.name]),segment.name+': unrelated joint changed');
}
assert.equal(JSON.stringify([from,to]),before,'scope selection never edits the original endpoints');cases++;

let emptyCalls=0;
assert.equal(findTransitionDetour({A,THREE,from,to,families:[],
  collision:()=>{emptyCalls++;return false;},clearEdge:()=>{emptyCalls++;return true;}}),null);
assert.equal(emptyCalls,0,'no relevant family means no unrelated search');cases++;
for(const families of [null,'foot',['hand'],['foot','foot'],['foot',undefined],[{}],Array(1)]) {
  assert.throws(()=>findTransitionDetour({A,THREE,from,to,families,collision:()=>false,clearEdge:()=>true}),
    /Invalid detour search inputs/);cases++;
}
const mutable=['foot'];
const stable=findTransitionDetour({A,THREE,from,to,families:mutable,
  collision:()=>{mutable.splice(0,mutable.length,'toes');return false;},clearEdge:()=>true});
assert(stable&&Math.abs(stable[0].local.foot_L.y)>0&&stable[0].local.toes_L.equals(from.local.toes_L),
  'callbacks cannot replace the validated family list during search');cases++;
console.log(JSON.stringify({pass:true,cases,scope:'relative-rig-ancestry-and-explicit-detour-families'}));
