// Independently specified thin angle-band obstacles exercise curved paths and
// a two-waypoint clearance plateau without mirroring any bone-check code.
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { findTransitionDetour } from './transition_path.mjs';

const X=new THREE.Vector3(1,0,0),A={SEG:['pelvis','toes_L','toes_R'].map(name=>({name}))};
const conf=u=>({local:{pelvis:new THREE.Quaternion().setFromAxisAngle(X,u),toes_L:new THREE.Quaternion(),toes_R:new THREE.Quaternion()},
  rootQ:new THREE.Quaternion(),rootP:new THREE.Vector3(.1,.2,.3)});
const from=conf(0),to=conf(1),before=JSON.stringify([from,to]);
const progress=c=>2*Math.atan2(c.local.pelvis.x,c.local.pelvis.w);
const abduction=c=>Math.min(Math.abs(2*Math.atan2(c.local.toes_L.y,c.local.toes_L.w)),Math.abs(2*Math.atan2(c.local.toes_R.y,c.local.toes_R.w)));
const sample=(a,b,u)=>({local:Object.fromEntries(A.SEG.map(s=>[s.name,a.local[s.name].clone().slerp(b.local[s.name],u)])),
  rootQ:a.rootQ.clone().slerp(b.rootQ,u),rootP:a.rootP.clone().lerp(b.rootP,u)});
function checker(obstacle,steps=2000) {
  const collision=c=>obstacle(progress(c),abduction(c));
  const clearEdge=(a,b)=>{for(let i=0;i<=steps;i++)if(collision(sample(a,b,i/steps)))return false;return true;};
  return {collision,clearEdge};
}
const thin=checker((u,bias)=>u>.849&&u<.853&&bias<.005);
assert.equal(thin.clearEdge(from,to),false,'straight interpolation crosses a 0.004 rad wide forbidden band');
const path=findTransitionDetour({A,THREE,from,to,...thin});
assert(path&&path.length>0);
for(const c of path){assert(Math.abs(c.local.toes_L.y+c.local.toes_R.y)<1e-12);assert(c.rootP.distanceTo(from.rootP)<1e-12);}
const chain=[from,...path,to];
for(let i=1;i<chain.length;i++)assert(thin.clearEdge(chain[i-1],chain[i]));
assert.equal(JSON.stringify([from,to]),before,'original endpoint objects must remain byte-for-byte unchanged');

// A single peak small enough for the channel cannot maintain the necessary
// clearance across its whole middle. A bounded plateau can.
const corridor=checker((u,bias)=>bias>.04||(u>.2&&u<.8&&bias<.025),800);
const plateau=findTransitionDetour({A,THREE,from,to,...corridor});
assert(plateau&&plateau.length===2,'a two-waypoint detour holds the rotation through the entire forbidden band');
const plateauChain=[from,...plateau,to];
for(let i=1;i<plateauChain.length;i++)assert(corridor.clearEdge(plateauChain[i-1],plateauChain[i]));

let calls=0;
assert.equal(findTransitionDetour({A,THREE,from,to,maxCandidates:3,
  collision:c=>false,clearEdge:()=>{calls++;return false;}}),null);
assert(calls<=3,'a finite search budget must be honoured');
assert.equal(findTransitionDetour({A,THREE,from,to,collision:()=>true,clearEdge:()=>true}),null,'colliding endpoints cannot be repaired by silently moving them');
assert.throws(()=>findTransitionDetour({A,THREE,from,to,...thin,maxCandidates:0}),/Invalid detour search inputs/);
assert.equal(JSON.stringify([from,to]),before);
console.log(JSON.stringify({pass:true,cases:9,scope:'bounded-trajectory-detours; supplied geometric callbacks'}));
