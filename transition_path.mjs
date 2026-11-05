// Deterministic local joint-space detours. This edits a trajectory, never the
// colliding meshes, joint exception policy, or exact endpoint configurations.
// The caller supplies the actual geometry checks and subsequently validates
// every saved interval/interpolation used by the page. This helper alone is
// not continuous collision detection, a contact solver or a movement protocol.
const DETOUR_FAMILIES = ['toes','foot','thigh'];

// A joint above BOTH parts only rotates their common frame. A joint below
// neither part cannot change the pair either. Trying either as a "repair"
// can merely move a finite sampling grid past an already known collision.
export function detourFamiliesForPair(A,pair) {
  const segments=A?.S;
  const own=name=>typeof name==='string'&&segments&&Object.hasOwn(segments,name);
  if(!segments||Array.isArray(segments)||!own(pair?.a)||!own(pair?.b))return [];
  const ancestors=name=>{
    const result=new Set();
    while(name!=null) {
      if(!own(name)||result.has(name))return null;
      result.add(name);
      const segment=segments[name];
      if(!segment||typeof segment!=='object')return null;
      name=segment.parent;
      if(name!==null&&typeof name!=='string')return null;
    }
    return result;
  };
  const a=ancestors(pair.a),b=ancestors(pair.b);
  if(!a||!b)return [];
  return DETOUR_FAMILIES.filter(family=>['L','R'].some(side=>{
    const joint=family+'_'+side;
    return own(joint)&&(a.has(joint)!==b.has(joint));
  }));
}

export function findTransitionDetour({ A, THREE, from, to, clearEdge, collision,
  maxCandidates = 504, progresses = [.5,.75,.25], families = DETOUR_FAMILIES }) {
  if (!Array.isArray(A?.SEG) || !A.SEG.length || !THREE?.Quaternion
    || typeof clearEdge!=='function' || typeof collision!=='function'
    || !Number.isInteger(maxCandidates) || maxCandidates<1
    || !Array.isArray(families)||new Set(families).size!==families.length
    || [...families].some(f=>!DETOUR_FAMILIES.includes(f))
    || !Array.isArray(progresses) || !progresses.length
    || progresses.some(u=>!Number.isFinite(u)||u<=0||u>=1)) throw new Error('Invalid detour search inputs');
  const finiteQ=q=>q&&[q.x,q.y,q.z,q.w].every(Number.isFinite)&&q.lengthSq()>1e-12;
  const finiteP=p=>p&&[p.x,p.y,p.z].every(Number.isFinite);
  for(const conf of [from,to]) if(!conf||!finiteQ(conf.rootQ)||!finiteP(conf.rootP)
    ||A.SEG.some(s=>!finiteQ(conf.local?.[s.name]))) throw new Error('Invalid detour endpoint');
  const selectedFamilies=[...families];
  if(!selectedFamilies.length)return null;
  if(collision(from)||collision(to))return null;
  const mix = u=>({
    local:Object.fromEntries(A.SEG.map(s=>[s.name,from.local[s.name].clone().normalize()
      .slerp(to.local[s.name].clone().normalize(),u).normalize()])),
    rootQ:from.rootQ.clone().normalize().slerp(to.rootQ.clone().normalize(),u).normalize(),
    rootP:from.rootP.clone().lerp(to.rootP,u),
  });
  const axes={Y:new THREE.Vector3(0,1,0),Z:new THREE.Vector3(0,0,1)};
  const angles=[2,-2,4,-4,6,-6,8,-8,10,-10,12,-12,15,-15];
  let attempts=0;
  const attempt=(times,family,axis,degrees)=>{
    if(attempts>=maxCandidates)return undefined;
    attempts++;
    const mids=times.map(u=>{
      const conf=mix(u);
      for(const side of ['L','R']) {
        const segment=family+'_'+side;
        if(!conf.local[segment])continue;
        const q=new THREE.Quaternion().setFromAxisAngle(axes[axis],degrees*(side==='L'?1:-1)*Math.PI/180);
        conf.local[segment].premultiply(q).normalize();
      }
      return conf;
    });
    if(mids.some(c=>collision(c)))return null;
    const chain=[from,...mids,to];
    for(let i=1;i<chain.length;i++)if(clearEdge(chain[i-1],chain[i])!==true)return null;
    return mids;
  };
  // Prefer the smallest perturbation and distal joints. In addition to a
  // single bend, the two-waypoint plateau keeps clearance throughout a narrow
  // forbidden angle band, instead of rotating back into it immediately.
  const schedules=[...progresses.map(u=>[u]),[.25,.75],[.125,.875],[.375,.875]];
  for(const family of selectedFamilies) {
    if(!['L','R'].some(side=>Object.hasOwn(from.local,family+'_'+side)))continue;
    for(const times of schedules)for(const axis of ['Y','Z'])for(const degrees of angles) {
      const result=attempt(times,family,axis,degrees);
      if(result===undefined)return null;
      if(result)return result;
    }
  }
  return null;
}
