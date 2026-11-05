import fs from 'node:fs';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createWholeBodyCollision } from './whole_body_collision.mjs';
import { createToeCollisionOracle } from './transition_toe_oracle.mjs';
const A=createAnatomy(THREE),bones=JSON.parse(fs.readFileSync(new URL('./skull/bones_bp3d.json',import.meta.url)));
const oracle=createToeCollisionOracle(A,THREE,bones),full=createWholeBodyCollision(A,THREE,bones);
const pairFilter=(a,b)=>a.seg.at(-1)===b.seg.at(-1)
  &&((a.seg.startsWith('foot_')&&b.seg.startsWith('toes_'))||(a.seg.startsWith('toes_')&&b.seg.startsWith('foot_')));
let seed=1729;
const random=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/2**32;};
const rotation=()=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(random()-.5,random()-.5,random()-.5).normalize(),(random()-.5)*Math.PI*2);
const keys=report=>report.pairs.map(p=>[p.aPart,p.bPart,p.kind].join(':')).sort();
for(let i=0;i<20;i++) {
  const local=Object.fromEntries(A.SEG.map(s=>[s.name,rotation()]));
  const W=A.computeWorld(local,rotation(),new THREE.Vector3(random()*3-1.5,random()*3-1.5,random()*3-1.5));
  const expected=full.inspect(W,{maxPairs:Infinity,pairFilter,pairFilterLabel:'independent-all-body-foot-toe-fixture'});
  const actual=oracle.inspect({local},{maxPairs:Infinity});
  assert.deepEqual(keys(actual),keys(expected),'intrinsic reduction must preserve exact original bone pairs across arbitrary global joint poses');
}
assert.equal(oracle.sourceParts.length,52);
const ids=bones.parts.map((p,i)=>({p,i})).filter(({p})=>p.seg==='foot_L'&&p.latin==='os metatarsi IV'
  ||p.seg==='toes_L'&&p.name==='Проксимальная фаланга IV пальца стопы').map(x=>x.i);
assert.equal(ids.length,2);
const focused=createToeCollisionOracle(A,THREE,bones,{sourcePartIndices:ids});
assert.equal(focused.scope,'focused-intrinsic-foot-toes-bones');
assert.equal(focused.sourceParts.length,2);
assert.throws(()=>createToeCollisionOracle(A,THREE,bones,{sourcePartIndices:[0]}),/complete original foot\/toe parts/);
assert.throws(()=>oracle.inspect({}),/Invalid local toe transform/);
console.log(JSON.stringify({pass:true,cases:24,globalRandomPoses:20,sourceParts:52,scope:oracle.scope}));
