// Scoped, exact femur/pelvis geometry for finding a bounded joint-space route.
// The common pelvic/global transform is removed; original bone chunks and the
// existing bounded hip-articulation policy are retained. Final whole-body,
// elbow, saved-frame/interpolation and floor checks remain mandatory.
import { gunzipSync,gzipSync } from 'node:zlib';
import { createWholeBodyCollision } from './whole_body_collision.mjs';

export function createHipCollisionOracle(A,THREE,boneData,{sourcePartIndices=null}={}) {
  const chosen=sourcePartIndices==null?null:new Set(sourcePartIndices),parts=[],chunks=[],original=[];
  if(!Array.isArray(boneData?.parts)||typeof boneData.data!=='string'||chosen
    &&(!Array.isArray(sourcePartIndices)||!chosen.size||sourcePartIndices.some(i=>!Number.isInteger(i)||i<0||i>=boneData.parts.length)))throw new Error('Invalid hip source geometry');
  const payload=gunzipSync(Buffer.from(boneData.data,'base64'));let offset=0;
  for(const [index,p]of boneData.parts.entries()) {
    const size=6*(p.n+p.m),chunk=payload.subarray(offset,offset+size);offset+=size;
    if(!/^(pelvis|thigh_[LR])$/.test(p.seg)||chosen&&!chosen.has(index))continue;
    parts.push({...p});chunks.push(chunk);original.push(index);
  }
  if(offset!==payload.length||!parts.length||chosen&&chosen.size!==original.length)throw new Error('Hip subset must retain complete original femur/pelvis parts');
  const collider=createWholeBodyCollision(A,THREE,{parts,data:gzipSync(Buffer.concat(chunks)).toString('base64')});
  const sides=[...new Set(parts.filter(p=>p.seg.startsWith('thigh_')).map(p=>p.seg.at(-1)))];
  const offsets=Object.fromEntries(sides.map(s=>[s,A.S['thigh_'+s].J.clone().sub(A.S.pelvis.J)]));
  const filter=(a,b)=>a.seg==='pelvis'&&b.seg.startsWith('thigh_')||b.seg==='pelvis'&&a.seg.startsWith('thigh_');
  const scope=chosen?'focused-intrinsic-femur-pelvis-bones':'intrinsic-bilateral-femur-pelvis-bones';
  return {scope,sourceParts:original.map((index,i)=>({index,seg:parts[i].seg,name:parts[i].name})),
    inspect(conf,{maxPairs=1}={}) {
      const local=conf?.local||conf,W={pelvis:{p:new THREE.Vector3(),q:new THREE.Quaternion()}};
      for(const side of sides) {
        const q=local?.['thigh_'+side];
        if(!q||![q.x,q.y,q.z,q.w].every(Number.isFinite)||!(q.lengthSq()>1e-12))throw new Error('Invalid hip local quaternion');
        W['thigh_'+side]={p:offsets[side].clone(),q:q.clone().normalize()};
      }
      const r=collider.inspect(W,{maxPairs,pairFilter:filter,pairFilterLabel:'canonical-femur-with-pelvis'});
      return {...r,scope,sourcePartIndices:[...original],pairs:r.pairs.map(p=>({...p,aPart:original[p.aPart],bPart:original[p.bPart]}))};
    },
  };
}

export function hipFlexionWaypoints({A,THREE,from,to,degrees}) {
  if(!Number.isFinite(degrees)||degrees<=0||degrees>15)throw new Error('Hip detour must use bounded positive flexion degrees');
  const delta=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-degrees*Math.PI/180);
  return [from,to].map(c=>{
    const local=Object.fromEntries(A.SEG.map(s=>[s.name,c.local[s.name].clone()]));
    for(const side of ['L','R'])local['thigh_'+side].premultiply(delta).normalize();
    return {local,rootQ:c.rootQ.clone(),rootP:c.rootP.clone()};
  });
}

export function findHipFlexionDetour({A,THREE,from,to,oracle,samplesPerEdge=1000,
  accept=()=>true,angles=[2,3,4,5,6,8,10,12,15]}) {
  if(!oracle||typeof oracle.inspect!=='function'||!Number.isInteger(samplesPerEdge)||samplesPerEdge<1
    ||!Array.isArray(angles)||angles.some(d=>!Number.isFinite(d)||d<=0||d>15))throw new Error('Invalid bounded hip detour search');
  if(oracle.inspect(from).colliding||oracle.inspect(to).colliding)return null;
  let samples=0;
  for(const degrees of angles) {
    const stages=hipFlexionWaypoints({A,THREE,from,to,degrees});
    if(stages.some(c=>!accept(c)||oracle.inspect(c).colliding))continue;
    const chain=[from,...stages,to];let clear=true;
    for(let e=1;e<chain.length&&clear;e++)for(let i=0;i<=samplesPerEdge;i++) {
      const local={};for(const side of ['L','R']){
        const s='thigh_'+side;local[s]=chain[e-1].local[s].clone().normalize().slerp(chain[e].local[s].clone().normalize(),i/samplesPerEdge).normalize();
      }
      samples++;if(oracle.inspect({local}).colliding){clear=false;break;}
    }
    if(clear)return {stages,adjustment:{kind:'temporary-bilateral-hip-flexion-detour',degrees,samplesPerEdge,
      scope:oracle.scope,requiresFullBodyValidation:true},evaluatedSamples:samples};
  }
  return null;
}
