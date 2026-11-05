import fs from 'node:fs';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createWholeBodyCollision } from './whole_body_collision.mjs';
import { createThumbCollisionOracle,findThumbRelease,thumbReleaseStages } from './transition_thumb_release.mjs';
const A=createAnatomy(THREE),read=p=>JSON.parse(fs.readFileSync(new URL(p,import.meta.url))),bones=read('./skull/bones_bp3d.json');
const oracle=createThumbCollisionOracle(A,THREE,bones),full=createWholeBodyCollision(A,THREE,bones);
const pairFilter=(a,b)=>a.seg.at(-1)===b.seg.at(-1)
  &&((a.seg.startsWith('hand_')&&b.seg.startsWith('f1b_'))||(a.seg.startsWith('f1b_')&&b.seg.startsWith('hand_')));
const keys=r=>r.pairs.map(p=>[p.aPart,p.bPart,p.kind].join(':')).sort();
let seed=4099;
const random=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/2**32;};
const rotation=()=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(random()-.5,random()-.5,random()-.5).normalize(),(random()-.5)*Math.PI*2);
for(let i=0;i<20;i++) {
  const local=Object.fromEntries(A.SEG.map(s=>[s.name,rotation()]));
  const W=A.computeWorld(local,rotation(),new THREE.Vector3(random(),random(),random()));
  assert.deepEqual(keys(oracle.inspect({local},{maxPairs:Infinity})),
    keys(full.inspect(W,{maxPairs:Infinity,pairFilter,pairFilterLabel:'independent-own-distal-thumb-fixture'})),
    'removing the hand/global transform preserves the exact original distal-thumb/palm pairs');
}
assert.equal(oracle.sourceParts.length,26);
assert(oracle.sourceParts.every(p=>/^(hand|f1b)_[LR]$/.test(p.seg)),'CMC/first-metacarpal gate must not be silently added');
const practice=read('./practice/poses.json').poses;
const ids=['detali-01','detali-03','detali-04','detali-05','detali-06','detali-07','detali-08'];
const decode=p=>({local:Object.fromEntries(Object.entries(p.local).map(([s,q])=>[s,new THREE.Quaternion(...q).normalize()])),
  rootQ:new THREE.Quaternion(...p.root).normalize(),rootP:new THREE.Vector3(...p.rootP)});
const base=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-35*Math.PI/180);
const results=[];
for(const id of ids) {
  const conf=decode(practice[id]),before=JSON.stringify(conf),thumbKeys=['f1m','f1a','f1b'].flatMap(s=>[s+'_L',s+'_R']);
  const neutral=thumbReleaseStages({A,THREE,conf,axis:'Z',degrees:0}).at(-1);
  const tipsFirst={local:Object.fromEntries(Object.entries(conf.local).map(([s,q])=>[s,q.clone()]))};
  for(const side of ['L','R'])for(const base of ['f1a','f1b'])tipsFirst.local[base+'_'+side].identity();
  let directHit=false;
  const former=[conf,tipsFirst,neutral];
  for(let edge=1;edge<former.length&&!directHit;edge++)for(let i=0;i<=1000&&!directHit;i++) {
    const local=Object.fromEntries(thumbKeys.map(s=>[s,former[edge-1].local[s].clone().slerp(former[edge].local[s],i/1000).normalize()]));
    directHit=oracle.inspect({local}).colliding;
  }
  assert(directHit,id+': reproduce the former tips-first then base-opening collision');
  const result=findThumbRelease({A,THREE,conf,oracle,samplesPerEdge:1000});
  assert(result&&result.stages.length===3,id+': a three-stage bounded radial release must exist');
  assert.equal(result.adjustment.scope,'intrinsic-own-distal-thumb-palm-bones');
  assert.equal(result.adjustment.requiresFullBodyValidation,true);
  assert(Math.abs(result.adjustment.mirroredBaseAbductionDegrees)<=45);
  for(const stage of result.stages) {
    assert(stage.rootP.equals(conf.rootP)&&stage.rootQ.equals(conf.rootQ));
    for(const s of A.SEG)if(!thumbKeys.includes(s.name))assert(stage.local[s.name].equals(conf.local[s.name]));
  }
  for(const side of ['L','R']) {
    assert(result.stages[0].local['f1a_'+side].equals(conf.local['f1a_'+side]),'initial abduction preserves the MCP/IP configuration');
    assert(result.stages[0].local['f1b_'+side].equals(conf.local['f1b_'+side]));
    assert(result.stages.at(-1).local['f1m_'+side].angleTo(base)<1e-7);
    assert(result.stages.at(-1).local['f1a_'+side].angleTo(new THREE.Quaternion())<1e-7);
    assert(result.stages.at(-1).local['f1b_'+side].angleTo(new THREE.Quaternion())<1e-7);
  }
  assert.equal(JSON.stringify(conf),before,'source pose must remain unchanged');
  results.push({id,axis:result.adjustment.axis,degrees:result.adjustment.mirroredBaseAbductionDegrees,samplesPerEdge:1000});
}
assert.throws(()=>thumbReleaseStages({A,THREE,conf:decode(practice[ids[0]]),degrees:46}),/Invalid bounded/);
assert.equal(findThumbRelease({A,THREE,conf:decode(practice[ids[0]]),oracle:{inspect:()=>({colliding:true})}}),null);
assert.throws(()=>oracle.inspect({}),/Invalid thumb local transform/);
console.log(JSON.stringify({pass:true,randomGlobalPoses:20,sourceParts:26,releasePoses:results,
  scope:oracle.scope,notEstablished:'whole-body paths, first-metacarpal/CMC clearance, dynamics or clinical anatomy'}));
