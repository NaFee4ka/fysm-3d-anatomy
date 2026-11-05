// Exact dependency reduction for foot/toe bone geometry. The full pose applies
// one common rigid transform to a foot and its toes. Removing that transform
// leaves only the original rig offset and local toe quaternion. No vertices,
// indices, joint-contact policy or radii are changed. This is a scoped search
// oracle; the whole trajectory still needs the full collision/floor guards.
import { gunzipSync,gzipSync } from 'node:zlib';
import { createWholeBodyCollision } from './whole_body_collision.mjs';

export function createToeCollisionOracle(A,THREE,boneData,{sourcePartIndices=null}={}) {
  if(!Array.isArray(boneData?.parts)||typeof boneData.data!=='string')throw new Error('Missing toe-oracle source geometry');
  if(sourcePartIndices!=null&&(!Array.isArray(sourcePartIndices)||!sourcePartIndices.length
    ||sourcePartIndices.some(i=>!Number.isInteger(i)||i<0||i>=boneData.parts.length)))throw new Error('Invalid focused source part list');
  const selected=sourcePartIndices==null?null:new Set(sourcePartIndices);
  const payload=gunzipSync(Buffer.from(boneData.data,'base64')),parts=[],chunks=[],originalIndices=[];
  let offset=0;
  for(const [index,part]of boneData.parts.entries()) {
    const bytes=6*(part.n+part.m),chunk=payload.subarray(offset,offset+bytes);offset+=bytes;
    if(!/^(foot|toes)_[LR]$/.test(part.seg)||selected&&!selected.has(index))continue;
    parts.push({...part});chunks.push(chunk);originalIndices.push(index);
  }
  if(offset!==payload.length||!parts.length||selected&&originalIndices.length!==selected.size)
    throw new Error('Toe-oracle source subset must contain only complete original foot/toe parts');
  const geometry={parts,data:gzipSync(Buffer.concat(chunks)).toString('base64')};
  const collider=createWholeBodyCollision(A,THREE,geometry);
  const sides=[...new Set(parts.map(p=>p.seg.at(-1)))];
  const pairFilter=(a,b)=>a.seg.at(-1)===b.seg.at(-1)
    &&((a.seg.startsWith('foot_')&&b.seg.startsWith('toes_'))||(a.seg.startsWith('toes_')&&b.seg.startsWith('foot_')));
  const offsets=Object.fromEntries(sides.map(side=>[side,A.S['toes_'+side].J.clone().sub(A.S['foot_'+side].J)]));
  const sourceRegion=r=>({...r,reason:r.reason.replace(/part (\d+)/g,(_,index)=>'part '+originalIndices[Number(index)])});
  const scope=selected?'focused-intrinsic-foot-toes-bones':'intrinsic-same-side-foot-toes-bones';
  return {
    scope,
    sourceParts:originalIndices.map((index,i)=>({index,seg:parts[i].seg,name:parts[i].name})),
    inspect(conf,{maxPairs=1}={}) {
      const local=conf?.local||conf,W={};
      for(const side of sides) {
        const q=local?.['toes_'+side];
        if(!q||![q.x,q.y,q.z,q.w].every(Number.isFinite)||!(q.lengthSq()>1e-12))throw new Error('Invalid local toe transform '+side);
        W['foot_'+side]={p:new THREE.Vector3(),q:new THREE.Quaternion()};
        W['toes_'+side]={p:offsets[side].clone(),q:q.clone().normalize()};
      }
      const report=collider.inspect(W,{maxPairs,pairFilter,pairFilterLabel:'canonical-same-side-foot-toes'});
      return {...report,scope,sourcePartIndices:originalIndices,
        pairs:report.pairs.map(p=>({...p,aPart:originalIndices[p.aPart],bPart:originalIndices[p.bPart],
          jointRegions:p.jointRegions.map(sourceRegion)})),jointRegions:report.jointRegions.map(sourceRegion)};
    },
  };
}
