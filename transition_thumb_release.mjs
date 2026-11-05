// Exact distal-thumb/palm dependency reduction and a staged geometric release.
// This intentionally does not enable a first-metacarpal/CMC collision scope:
// the existing source registration has separate unresolved limitations there.
// Every accepted route still needs the engine's all-body and floor validation.
import { gunzipSync,gzipSync } from 'node:zlib';
import { createWholeBodyCollision } from './whole_body_collision.mjs';

export function createThumbCollisionOracle(A,THREE,boneData,{sourcePartIndices=null}={}) {
  if(!Array.isArray(boneData?.parts)||typeof boneData.data!=='string')throw new Error('Missing thumb-oracle source geometry');
  if(sourcePartIndices!=null&&(!Array.isArray(sourcePartIndices)||!sourcePartIndices.length
    ||sourcePartIndices.some(i=>!Number.isInteger(i)||i<0||i>=boneData.parts.length)))throw new Error('Invalid thumb source subset');
  const selected=sourcePartIndices==null?null:new Set(sourcePartIndices),parts=[],chunks=[],originalIndices=[];
  const payload=gunzipSync(Buffer.from(boneData.data,'base64'));let offset=0;
  for(const [index,part]of boneData.parts.entries()) {
    const bytes=6*(part.n+part.m),chunk=payload.subarray(offset,offset+bytes);offset+=bytes;
    if(!/^(hand|f1b)_[LR]$/.test(part.seg)||selected&&!selected.has(index))continue;
    parts.push({...part});chunks.push(chunk);originalIndices.push(index);
  }
  if(offset!==payload.length||!parts.length||selected&&selected.size!==originalIndices.length)
    throw new Error('Thumb subset must contain complete original palm/distal-thumb parts');
  const collider=createWholeBodyCollision(A,THREE,{parts,data:gzipSync(Buffer.concat(chunks)).toString('base64')});
  const sides=[...new Set(parts.map(p=>p.seg.at(-1)))];
  const offsets=Object.fromEntries(sides.map(side=>[side,['f1m','f1a','f1b'].map((base,i)=>
    A.S[base+'_'+side].J.clone().sub(A.S[(i?['f1m','f1a'][i-1]:'hand')+'_'+side].J))]));
  const pairFilter=(a,b)=>a.seg.at(-1)===b.seg.at(-1)
    &&((a.seg.startsWith('hand_')&&b.seg.startsWith('f1b_'))||(a.seg.startsWith('f1b_')&&b.seg.startsWith('hand_')));
  const scope=selected?'focused-intrinsic-distal-thumb-palm-bones':'intrinsic-own-distal-thumb-palm-bones';
  return {scope,sourceParts:originalIndices.map((index,i)=>({index,seg:parts[i].seg,name:parts[i].name})),
    inspect(conf,{maxPairs=1}={}) {
      const local=conf?.local||conf,W={};
      for(const side of sides) {
        let p=new THREE.Vector3(),q=new THREE.Quaternion();
        W['hand_'+side]={p:p.clone(),q:q.clone()};
        for(const [i,base]of ['f1m','f1a','f1b'].entries()) {
          const lq=local?.[base+'_'+side];
          if(!lq||![lq.x,lq.y,lq.z,lq.w].every(Number.isFinite)||!(lq.lengthSq()>1e-12))throw new Error('Invalid thumb local transform '+base+'_'+side);
          p=p.clone().add(offsets[side][i].clone().applyQuaternion(q));
          q=q.clone().multiply(lq.clone().normalize());
        }
        W['f1b_'+side]={p,q};
      }
      const result=collider.inspect(W,{maxPairs,pairFilter,pairFilterLabel:'canonical-own-distal-thumb-with-palm'});
      return {...result,scope,sourcePartIndices:[...originalIndices],
        pairs:result.pairs.map(p=>({...p,aPart:originalIndices[p.aPart],bPart:originalIndices[p.bPart]}))};
    },
  };
}

export function thumbReleaseStages({A,THREE,conf,axis='Z',degrees}) {
  if(!['Y','Z'].includes(axis)||!Number.isFinite(degrees)||Math.abs(degrees)>45)throw new Error('Invalid bounded thumb release');
  const clone=()=>({local:Object.fromEntries(A.SEG.map(s=>[s.name,conf.local[s.name].clone()])),
    rootQ:conf.rootQ.clone(),rootP:conf.rootP.clone()});
  const radial=clone(),unfolded=clone(),neutral=clone();
  const direction=axis==='Z'?new THREE.Vector3(0,0,1):new THREE.Vector3(0,1,0);
  const target=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-35*Math.PI/180);
  for(const side of ['L','R']) {
    const turn=new THREE.Quaternion().setFromAxisAngle(direction,degrees*(side==='L'?1:-1)*Math.PI/180);
    radial.local['f1m_'+side].premultiply(turn).normalize();
    unfolded.local['f1m_'+side].copy(target).premultiply(turn).normalize();
    neutral.local['f1m_'+side].copy(target);
    for(const base of ['f1a','f1b']) {unfolded.local[base+'_'+side].identity();neutral.local[base+'_'+side].identity();}
  }
  return [radial,unfolded,neutral];
}

export function findThumbRelease({A,THREE,conf,oracle,samplesPerEdge=1000,accept=()=>true,
  angles=[2,-2,4,-4,6,-6,8,-8,10,-10,12,-12,15,-15,20,-20,25,-25,30,-30,35,-35,40,-40,45,-45]}) {
  if(!oracle||typeof oracle.inspect!=='function'||!Number.isInteger(samplesPerEdge)||samplesPerEdge<1
    ||typeof accept!=='function'||!Array.isArray(angles)||angles.some(d=>!Number.isFinite(d)||Math.abs(d)>45))throw new Error('Invalid thumb release search');
  if(oracle.inspect(conf).colliding)return null;
  let samples=0;
  const clear=(a,b)=>{
    for(let j=0;j<=samplesPerEdge;j++) {
      const u=j/samplesPerEdge,local={};
      for(const side of ['L','R'])for(const base of ['f1m','f1a','f1b']) {
        const s=base+'_'+side;local[s]=a.local[s].clone().normalize().slerp(b.local[s].clone().normalize(),u).normalize();
      }
      samples++;if(oracle.inspect({local}).colliding)return false;
    }
    return true;
  };
  for(const axis of ['Z','Y'])for(const degrees of angles) {
    const stages=thumbReleaseStages({A,THREE,conf,axis,degrees});
    if(stages.some(c=>!accept(c)||oracle.inspect(c).colliding))continue;
    const chain=[conf,...stages];let valid=true;
    for(let i=1;i<chain.length;i++)if(!clear(chain[i-1],chain[i])){valid=false;break;}
    if(valid)return {stages,adjustment:{kind:'staged-radial-thumb-release',axis,mirroredBaseAbductionDegrees:degrees,
      neutralBaseXDegrees:-35,samplesPerEdge,scope:oracle.scope,requiresFullBodyValidation:true},evaluatedSamples:samples};
  }
  return null;
}
