// Exact rigid-bone audit across all body regions. Joint-local natural articulation
// is reported separately. The predicate is inclusive: tangential surface contact
// and penetration are both candidates; this module does not estimate depth.
// This does not model skin, cartilage pressure or soft tissue contact.
import { gunzipSync } from 'node:zlib';
import { triangleIntersect } from './body_collision.mjs';
import { segmentClosest } from './collision_geometry.mjs';

const finitePoint = p => p && [p.x, p.y, p.z].every(Number.isFinite);

function exactDuplicateSimplex(triangle) {
  const unique=[];
  for(const point of triangle)if(!unique.some(p=>p.equals(point)))unique.push(point);
  return unique.length<=2?[unique[0],unique.at(-1)]:null;
}
function simplexTriangleLocus(segment,triangle,normal,roundoff) {
  const [a,b]=segment, direction=b.clone().sub(a);
  const da=a.clone().sub(triangle[0]).dot(normal), db=b.clone().sub(triangle[0]).dot(normal);
  let lo=0,hi=1;
  if(!(Math.abs(da)<=roundoff&&Math.abs(db)<=roundoff)) {
    if(da*db>0 || da===db)return {kind:'unresolved',points:[],reason:'simplex-plane-not-established'};
    lo=hi=da/(da-db);
  }
  for(let edge=0;edge<3;edge++) {
    const origin=triangle[edge],e=triangle[(edge+1)%3].clone().sub(origin),length=e.length();
    const offset=e.clone().cross(a.clone().sub(origin)).dot(normal)+roundoff*length;
    const slope=e.clone().cross(direction).dot(normal);
    if(Math.abs(slope)<=64*Number.EPSILON*length*direction.length()) {
      if(offset<0)return {kind:'unresolved',points:[],reason:'simplex-outside-triangle'};
    }else if(slope>0)lo=Math.max(lo,-offset/slope);
    else hi=Math.min(hi,-offset/slope);
  }
  if(lo>hi||lo<0||hi>1)return {kind:'unresolved',points:[],reason:'simplex-interval-not-established'};
  const points=[a.clone().addScaledVector(direction,lo),a.clone().addScaledVector(direction,hi)];
  return {kind:points[0].distanceTo(points[1])<=roundoff?'point':'segment',points};
}

// The intersection of two regular triangles is convex: a noncoplanar segment
// or a coplanar convex polygon. Its endpoints/vertices bound the entire locus
// inside any one convex joint sphere. Source-triangle vertices need not lie in
// that sphere: a long tibial triangle can meet the talus only at its distal end.
// Uncertain loci remain unresolved. The caller can independently prove that
// both entire source triangles are enclosed by one bounded convex region.
export function triangleContactLocus(a, b, epsilon = 1e-9) {
  if (!triangleIntersect(a,b,epsilon)) return {kind:'empty',points:[]};
  const normal = t=>t[1].clone().sub(t[0]).cross(t[2].clone().sub(t[0]));
  const scale = t=>Math.max(...[0,1,2].map(i=>t[i].distanceTo(t[(i+1)%3])));
  const na=normal(a), nb=normal(b), la=na.length(), lb=nb.length(), sa=scale(a), sb=scale(b);
  const coordinateScale=Math.max(1,sa,sb,...a.concat(b).flatMap(p=>[Math.abs(p.x),Math.abs(p.y),Math.abs(p.z)]));
  const roundoff=64*Number.EPSILON*coordinateScale;
  const regularA=la>(epsilon+16*Number.EPSILON*sa)*sa,regularB=lb>(epsilon+16*Number.EPSILON*sb)*sb;
  if(!regularA||!regularB) {
    // Quantisation can collapse two source vertices to exactly one position.
    // That is a provable segment/point, not permission to discard the face.
    const simplexA=exactDuplicateSimplex(a),simplexB=exactDuplicateSimplex(b);
    if(simplexA&&regularB)return simplexTriangleLocus(simplexA,b,nb.divideScalar(lb),roundoff);
    if(simplexB&&regularA)return simplexTriangleLocus(simplexB,a,na.divideScalar(la),roundoff);
    if(simplexA&&simplexB) {
      const closest=segmentClosest(...simplexA,...simplexB);
      if(closest.distance>roundoff)return {kind:'unresolved',points:[],reason:'simplex-contact-not-established'};
      const points=[closest.a,closest.b];
      for(const [source,target]of[[simplexA,simplexB],[simplexB,simplexA]])for(const p of source)
        if(segmentClosest(p,p,...target).distance<=roundoff)points.push(p.clone());
      const span=Math.max(...points.map(p=>p.distanceTo(points[0])));
      return {kind:span<=roundoff?'point':'segment',points};
    }
    return {kind:'unresolved',points:[],reason:'uncertain-degenerate-triangle'};
  }
  na.divideScalar(la);nb.divideScalar(lb);
  const direction=na.clone().cross(nb), sine=direction.length();
  const planeDistance=Math.max(...b.map(p=>Math.abs(p.clone().sub(a[0]).dot(na))),
    ...a.map(p=>Math.abs(p.clone().sub(b[0]).dot(nb))));
  if (sine<1e-8) {
    if (sine>64*Number.EPSILON || planeDistance>roundoff)
      return {kind:'unresolved',points:[],reason:'near-parallel-planes'};
    const clip = (subject, clipping, n) => {
      let polygon=subject.map(p=>p.clone());
      for(let edge=0;edge<3 && polygon.length;edge++) {
        const origin=clipping[edge], e=clipping[(edge+1)%3].clone().sub(origin), tolerance=roundoff*e.length();
        const signed=p=>e.clone().cross(p.clone().sub(origin)).dot(n)+tolerance;
        const next=[];
        for(let i=0;i<polygon.length;i++) {
          const p=polygon[i],q=polygon[(i+1)%polygon.length],dp=signed(p),dq=signed(q),pi=dp>=0,qi=dq>=0;
          if(pi)next.push(p);
          if(pi!==qi)next.push(p.clone().lerp(q,dp/(dp-dq)));
        }
        polygon=next;
      }
      return polygon;
    };
    // Both original planes are retained instead of projecting an offset sheet
    // onto the other. Clipping expands by roundoff only, never a model radius.
    const points=clip(a,b,nb).concat(clip(b,a,na));
    return points.length && points.every(finitePoint) ? {kind:'polygon',points}
      : {kind:'unresolved',points:[],reason:'coplanar-locus-not-established'};
  }
  // Work relative to a[0] to avoid cancellation in the two plane constants.
  const d=nb.dot(b[0].clone().sub(a[0]));
  const point=na.clone().multiplyScalar(-d).cross(direction).divideScalar(direction.lengthSq()).add(a[0]);
  direction.divideScalar(sine);
  if (!finitePoint(point) || Math.abs(point.clone().sub(a[0]).dot(na))>roundoff
      || Math.abs(point.clone().sub(b[0]).dot(nb))>roundoff)
    return {kind:'unresolved',points:[],reason:'unstable-plane-line'};
  let lo=-Infinity,hi=Infinity;
  for(const [triangle,n] of [[a,na],[b,nb]]) for(let edge=0;edge<3;edge++) {
    const origin=triangle[edge],e=triangle[(edge+1)%3].clone().sub(origin),edgeLength=e.length();
    const offset=e.clone().cross(point.clone().sub(origin)).dot(n)+roundoff*edgeLength;
    const slope=e.clone().cross(direction).dot(n);
    if(Math.abs(slope)<=64*Number.EPSILON*edgeLength) {
      if(offset<0)return {kind:'unresolved',points:[],reason:'parallel-edge-outside'};
    } else if(slope>0)lo=Math.max(lo,-offset/slope);
    else hi=Math.min(hi,-offset/slope);
  }
  if(!Number.isFinite(lo)||!Number.isFinite(hi)||lo>hi)
    return {kind:'unresolved',points:[],reason:'line-interval-not-established'};
  const points=[point.clone().addScaledVector(direction,lo),point.clone().addScaledVector(direction,hi)];
  return points.every(finitePoint)?{kind:hi-lo<=roundoff?'point':'segment',points}
    :{kind:'unresolved',points:[],reason:'nonfinite-locus'};
}

function boxesOverlap(a, b, epsilon) {
  return a.min.x <= b.max.x + epsilon && a.max.x + epsilon >= b.min.x
    && a.min.y <= b.max.y + epsilon && a.max.y + epsilon >= b.min.y
    && a.min.z <= b.max.z + epsilon && a.max.z + epsilon >= b.min.z;
}

function buildTree(triangles, THREE) {
  const box = new THREE.Box3();
  for (const triangle of triangles) box.union(triangle.box);
  const center=box.getCenter(new THREE.Vector3()),span=box.getSize(new THREE.Vector3());
  const sphereRadiusM=span.length()/2+64*Number.EPSILON*(1+center.length()+span.length());
  if (triangles.length <= 8) return { box,center,sphereRadiusM, triangles, count: triangles.length };
  const axis = span.x > span.y ? (span.x > span.z ? 'x' : 'z') : (span.y > span.z ? 'y' : 'z');
  triangles.sort((a, b) => a.center[axis] - b.center[axis]);
  const half = triangles.length >> 1;
  return { box,center,sphereRadiusM, count: triangles.length, left: buildTree(triangles.slice(0, half), THREE), right: buildTree(triangles.slice(half), THREE) };
}

function connectedComponents(vertices, triangles, THREE) {
  const parents = Int32Array.from(vertices, (_, i) => i), positions = new Map();
  const find = index => {
    while (parents[index] !== index) { parents[index] = parents[parents[index]]; index = parents[index]; }
    return index;
  };
  const unite = (a, b) => { const left = find(a), right = find(b); if (left !== right) parents[right] = left; };
  // OBJ normal/UV seams may duplicate the same geometric vertex. Weld only
  // exactly coincident decoded coordinates; nearby distinct sheets stay apart.
  for (const [i, p] of vertices.entries()) {
    const key = p.x + ',' + p.y + ',' + p.z;
    if (positions.has(key)) unite(i, positions.get(key)); else positions.set(key, i);
  }
  for (const triangle of triangles) {
    unite(triangle.indices[0], triangle.indices[1]); unite(triangle.indices[0], triangle.indices[2]);
  }
  const groups = new Map();
  for (const triangle of triangles) {
    const key = find(triangle.indices[0]);
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(triangle);
  }
  return [...groups.values()].map(group => ({ seed: group[0].indices[0],
    indices: [...new Set(group.flatMap(t => t.indices))], tree: buildTree(group, THREE) }));
}

export function createWholeBodyCollision(A, THREE, boneData) {
  if (!boneData || !Array.isArray(boneData.parts) || !boneData.parts.length || typeof boneData.data !== 'string')
    throw new Error('Missing bone data for body collision checks');
  const buffer = gunzipSync(Buffer.from(boneData.data, 'base64'));
  const expected = boneData.parts.reduce((n, part) => n + 6 * (part.n + part.m), 0);
  if (buffer.length !== expected) throw new Error('Bone collision manifest/payload length mismatch');
  const bones = [];
  let offset = 0;
  for (const [partIndex, part] of boneData.parts.entries()) {
    if (!Number.isInteger(part.n) || !Number.isInteger(part.m) || part.n < 1 || part.m < 1
      || !Array.isArray(part.lo) || !Array.isArray(part.hi) || part.lo.length !== 3 || part.hi.length !== 3
      || !part.lo.concat(part.hi).every(Number.isFinite)) throw new Error('Invalid bone collision manifest');
    const vertices = [], acc = [0, 0, 0];
    for (let v = 0; v < part.n; v++) {
      const xyz = [];
      for (let k = 0; k < 3; k++) {
        acc[k] = (acc[k] + buffer.readUInt16LE(offset)) & 65535; offset += 2;
        xyz.push(part.lo[k] + (part.hi[k] - part.lo[k]) * acc[k] / 65535);
      }
      vertices.push(new THREE.Vector3(...xyz));
    }
    const indices = [];
    let index = 0;
    for (let k = 0; k < part.m * 3; k++) {
      index = (index + buffer.readUInt16LE(offset)) & 65535; offset += 2;
      if (index >= part.n) throw new Error('Bone collision triangle index outside source part');
      indices.push(index);
    }
    if (!A.S[part.seg]) throw new Error('Unknown bone collision segment ' + part.seg);
    const triangles = [];
    for (let k = 0; k < indices.length; k += 3) {
      const ids = indices.slice(k, k + 3), points = ids.map(i => vertices[i]);
      const box = new THREE.Box3().setFromPoints(points);
      const center=box.getCenter(new THREE.Vector3()),span=box.getSize(new THREE.Vector3());
      const sphereRadiusM=span.length()/2+64*Number.EPSILON*(1+center.length()+span.length());
      triangles.push({ id: k / 3, indices: ids, box, center,sphereRadiusM });
    }
    const components = connectedComponents(vertices, triangles, THREE);
    const metatarsal=part.latin?.match(/^os metatarsi (I|II|III|IV|V)$/)?.[1]||null;
    const toeDigit=/^toes_[LR]$/.test(part.seg)&&part.latin==='phalanx proximalis'
      ? /большого/.test(part.name)?'I':/мизинца/.test(part.name)?'V':part.name.match(/\b(II|III|IV)\b/)?.[1]||null:null;
    let sourceCap=null;
    if(metatarsal||toeDigit) {
      const z=vertices.map(v=>v.z),lo=Math.min(...z),hi=Math.max(...z),span=hi-lo;
      const cap=vertices.filter(v=>metatarsal?v.z>=hi-.15*span:v.z<=lo+.15*span);
      if(cap.length)sourceCap=cap.reduce((sum,v)=>sum.add(v),new THREE.Vector3()).divideScalar(cap.length);
    }
    bones.push({ partIndex, seg: part.seg, name: part.name, side: part.seg.slice(-1), vertices,
      metatarsal,toeDigit,sourceCap,
      tree: components.length === 1 ? components[0].tree : buildTree(triangles, THREE), components });
  }
  const allPairs = [];
  for (let i = 0; i < bones.length; i++) for (let j = i + 1; j < bones.length; j++)
    if (bones[i].seg !== bones[j].seg) allPairs.push([bones[i], bones[j]]);
  const pairSelections = new WeakMap();
  const descriptors = new Map(bones.map(b=>[b,Object.freeze({seg:b.seg,partIndex:b.partIndex,name:b.name})]));
  function selectPairs(filter) {
    if (!pairSelections.has(filter)) pairSelections.set(filter,allPairs.filter(([a,b])=>filter(descriptors.get(a),descriptors.get(b))));
    return pairSelections.get(filter);
  }
  function jointRegions(a, b, W) {
    const regions = [];
    const add = (seg, r, reason, localCenter=null) => regions.push({ center:localCenter
      ? localCenter.clone().applyQuaternion(W[seg].q.clone().normalize()).add(W[seg].p):W[seg].p,
      radius: r, segment: seg, reason });
    const radius = seg => /^f[1-5]/.test(seg) ? .018 : /^toes/.test(seg) ? .025
      : /^hand/.test(seg) ? .04 : /^foot/.test(seg) ? .045 : /^fore|^ulna/.test(seg) ? .045
      : /^hum/.test(seg) ? .055 : /^thigh/.test(seg) ? .055 : /^shank/.test(seg) ? .055
      : /^scap|^clav/.test(seg) ? .055 : .055;
    const footToe=(/^foot_[LR]$/.test(a.seg)&&/^toes_[LR]$/.test(b.seg))
      ||(/^toes_[LR]$/.test(a.seg)&&/^foot_[LR]$/.test(b.seg));
    if (A.S[a.seg]?.parent === b.seg&&!footToe) add(a.seg, radius(a.seg), 'parent-child articulation');
    if (A.S[b.seg]?.parent === a.seg&&!footToe) add(b.seg, radius(b.seg), 'parent-child articulation');
    if(footToe&&a.side===b.side) {
      const metatarsal=a.metatarsal?a:b,toe=a.toeDigit?a:b;
      if(metatarsal.metatarsal&&metatarsal.metatarsal===toe.toeDigit&&metatarsal.sourceCap&&toe.sourceCap) {
        add(metatarsal.seg,.015,`matched MTP ${toe.toeDigit}; distal 15% source cap; part ${metatarsal.partIndex}`,metatarsal.sourceCap);
        add(toe.seg,.015,`matched MTP ${toe.toeDigit}; proximal 15% source base; part ${toe.partIndex}`,toe.sourceCap);
      }
    }
    for (const side of ['L', 'R']) {
      const pair = new Set([a.seg, b.seg]);
      if (pair.has('hum_' + side) && pair.has('clav_' + side)) add('hum_' + side, .055, 'shoulder articulation');
      if (pair.has('ulna_' + side) && pair.has('hand_' + side)) add('hand_' + side, .04, 'distal radioulnar/wrist articulation');
      if (pair.has('ulna_' + side) && pair.has('hum_' + side)) add('fore_' + side, .045, 'elbow articulation');
    }
    return regions;
  }
  function floorMinimum(W) {
    let minimumY = Infinity, at = null;
    for (const bone of bones) {
      const transform = W[bone.seg];
      if (!transform || !finitePoint(transform.p) || !transform.q
        || ![transform.q.x, transform.q.y, transform.q.z, transform.q.w].every(Number.isFinite)
        || !(transform.q.lengthSq() > 1e-12)) throw new Error('Invalid floor transform ' + bone.seg);
      const q = transform.q.clone().normalize();
      for (const p of bone.vertices) {
        const y = p.clone().applyQuaternion(q).add(transform.p).y;
        if (y < minimumY) { minimumY = y; at = { segment: bone.seg, name: bone.name, part: bone.partIndex }; }
      }
    }
    return { minimumY, ...at, scope: 'all-decoded-bone-vertices' };
  }

  function inspect(W, options = {}) {
    const epsilon = options.epsilon ?? 1e-9;
    const maxPairs = options.maxPairs ?? Infinity, triangleLimit = options.maxTriangleIntersections ?? 1;
    if (!Number.isFinite(epsilon) || epsilon < 0 || !(maxPairs > 0) || !(triangleLimit >= 1))
      throw new Error('Invalid bone collision options');
    const filter = options.pairFilter;
    if (filter != null && (typeof filter !== 'function' || typeof options.pairFilterLabel !== 'string'
      || !options.pairFilterLabel.trim())) throw new Error('A pair filter requires a function and an explicit scope label');
    const selectedPairs = filter ? selectPairs(filter) : allPairs;
    const scope = filter ? 'filtered-body-bones:' + options.pairFilterLabel.trim() : 'whole-body-bones-joint-local';
    const matrices = new Map(), states = new Map(), parentFrames = new Map();
    const matrixOf = seg => {
      if (!matrices.has(seg)) {
        const transform = W[seg];
        if (!transform || !finitePoint(transform.p) || !transform.q
          || ![transform.q.x, transform.q.y, transform.q.z, transform.q.w].every(Number.isFinite)
          || !(transform.q.lengthSq() > 1e-12)) throw new Error('Invalid bone collision transform ' + seg);
        matrices.set(seg, new THREE.Matrix4().compose(transform.p, transform.q.clone().normalize(), new THREE.Vector3(1, 1, 1)));
      }
      return matrices.get(seg);
    };
    const stateOf = bone => {
      if (!states.has(bone)) {
        const matrix = matrixOf(bone.seg), boxCache = new Map(), vertexCache = new Map(), triangleCache = new Map(),centerCache=new Map();
        const state = {
          box: node => {
            if (!boxCache.has(node)) boxCache.set(node, node.box.clone().applyMatrix4(matrix));
            return boxCache.get(node);
          },
          vertex: index => {
            if (!vertexCache.has(index)) vertexCache.set(index, bone.vertices[index].clone().applyMatrix4(matrix));
            return vertexCache.get(index);
          },
          triangle: triangle => {
            if (!triangleCache.has(triangle)) triangleCache.set(triangle, triangle.indices.map(i => state.vertex(i)));
            return triangleCache.get(triangle);
          },
          sphereInside: (node,region)=>{
            if(node.sphereRadiusM>=region.radius)return false;
            if(!centerCache.has(node))centerCache.set(node,node.center.clone().applyMatrix4(matrix));
            const center=centerCache.get(node),r=region.center;
            const margin=256*Number.EPSILON*(1+Math.abs(center.x)+Math.abs(center.y)+Math.abs(center.z)
              +Math.abs(r.x)+Math.abs(r.y)+Math.abs(r.z)+node.sphereRadiusM+region.radius);
            const allowance=region.radius-node.sphereRadiusM-margin;
            return allowance>=0&&center.distanceToSquared(r)<=allowance*allowance;
          },
        };
        states.set(bone, state);
      }
      return states.get(bone);
    };
    let candidatePairs = 0, triangleTests = 0, containmentTests = 0, jointLocalTrianglePairs = 0;
    let jointLocalEnclosureCandidates=0,jointLocalOverlapCertificates=0,jointLocalParentOverlapCertificates=0,parentFrameSeparationCertificates=0,jointLocalSingleTriangleCandidates=0;
    let activeRegions = [];
    const jointRegionsUsed = new Map();
    const entirelyLocal = points => activeRegions.some(region => points.every(p => p.distanceToSquared(region.center) <= region.radius * region.radius));
    const spherePairLocal=(a,b,sa,sb)=>activeRegions.some(r=>sa.sphereInside(a,r)&&sb.sphereInside(b,r));
    const completeTriangleLocal=points=>activeRegions.some(region=>{
      const center=region.center;
      const scale=1+Math.abs(center.x)+Math.abs(center.y)+Math.abs(center.z)
        +points.reduce((sum,p)=>sum+Math.abs(p.x)+Math.abs(p.y)+Math.abs(p.z),0)+region.radius;
      const allowance=region.radius-Math.sqrt(3)*epsilon-256*Number.EPSILON*scale;
      // A triangle is convex and its exact intersection with another triangle
      // is a subset of EACH complete source triangle. Enclosing either whole
      // triangle in ONE sphere is sufficient for this surface candidate only.
      // The subtracted halo also encloses genuine epsilon-near counterparts.
      return allowance>0&&points.every(p=>p.distanceToSquared(center)<=allowance*allowance);
    });
    const expandedOverlap=(a,b,roundoffScale=0,nearPadding=epsilon)=>{
      const magnitude=Math.max(roundoffScale,1+Math.max(...['x','y','z'].flatMap(k=>[Math.abs(a.min[k]),Math.abs(a.max[k]),Math.abs(b.min[k]),Math.abs(b.max[k])])));
      // Pad both AABBs for the inclusive epsilon predicate and Float64
      // transform/rounding error; an empty unexpanded intersection is not a
      // reason to discard a near-contact candidate.
      const padding=nearPadding+256*Number.EPSILON*magnitude;
      const lo={},hi={};
      for(const k of ['x','y','z']){lo[k]=Math.max(a.min[k],b.min[k])-padding;hi[k]=Math.min(a.max[k],b.max[k])+padding;}
      return {lo,hi,magnitude,separated:['x','y','z'].some(k=>lo[k]>hi[k])};
    };
    const overlapBoxLocal=(a,b,regions=activeRegions,roundoffScale=0,nearPadding=epsilon)=>{
      if(!regions.length)return false;
      const {lo,hi,magnitude,separated}=expandedOverlap(a,b,roundoffScale,nearPadding);
      if(separated)return false;
      return regions.some(region=>{
        const r=region.center,margin=256*Number.EPSILON*(magnitude+Math.abs(r.x)+Math.abs(r.y)+Math.abs(r.z)+region.radius);
        const allowance=region.radius-margin;
        const farthestSquared=['x','y','z'].reduce((sum,k)=>sum+Math.max((lo[k]-r[k])**2,(hi[k]-r[k])**2),0);
        return allowance>0&&farthestSquared<=allowance*allowance;
      });
    };
    const parentPartCertificate=(a,b,worldA,worldB)=>{
      // An axis-aligned world box can be needlessly loose after a common
      // rigid turn. Re-express BOTH complete parts and the SAME joint sphere
      // in an actual parent frame. Every possible common volume is still in
      // the intersection of these bounds: this is a sufficient spatial proof,
      // not a replacement collision classification in a fabricated pose.
      const frameSegments=new Set(activeRegions.map(r=>A.S[r.segment]?.parent).filter(Boolean));
      // Separation is ordinary broad phase and must still apply when natural
      // joint exceptions are disabled. Direct rig parents supply an actual
      // coordinate basis without creating any allowed contact region.
      if(A.S[a.seg]?.parent===b.seg)frameSegments.add(b.seg);
      if(A.S[b.seg]?.parent===a.seg)frameSegments.add(a.seg);
      if(!frameSegments.size)return null;
      const worldScale=1+Math.max(...['x','y','z'].flatMap(k=>[
        Math.abs(worldA.min[k]),Math.abs(worldA.max[k]),Math.abs(worldB.min[k]),Math.abs(worldB.max[k]),
      ]));
      for(const segment of frameSegments) {
        if(!parentFrames.has(segment)) {
          const t=W[segment];
          const usableQuaternion=typeof t?.q?.lengthSq==='function'&&typeof t.q.clone==='function';
          const normSquared=usableQuaternion?t.q.lengthSq():NaN;
          const valid=t&&finitePoint(t.p)&&usableQuaternion&&[t.q.x,t.q.y,t.q.z,t.q.w].every(Number.isFinite)
            &&Number.isFinite(normSquared)&&normSquared>1e-12;
          parentFrames.set(segment,valid?{inverse:matrixOf(segment).clone().invert(),bounds:new Map(),
            scale:1+Math.max(Math.abs(t.p.x),Math.abs(t.p.y),Math.abs(t.p.z))}:null);
        }
        const frame=parentFrames.get(segment);
        // Missing or invalid auxiliary frames confer no extra exception.
        // The original pair's normal transform validation/checks still run.
        if(!frame)continue;
        const bounds=bone=>{
          if(!frame.bounds.has(bone)) {
            const relative=frame.inverse.clone().multiply(matrixOf(bone.seg));
            const box=new THREE.Box3(),point=new THREE.Vector3();
            for(const vertex of bone.vertices)box.expandByPoint(point.copy(vertex).applyMatrix4(relative));
            frame.bounds.set(bone,box);
          }
          return frame.bounds.get(bone);
        };
        // A world-axis epsilon halo has Euclidean radius at most sqrt(3)*eps.
        // Preserve that halo in the turned frame and include the WORLD scale
        // in the roundoff budget for the inverse/composed transforms.
        const ba=bounds(a),bb=bounds(b),scale=Math.max(worldScale,frame.scale),halo=Math.sqrt(3)*epsilon;
        // STRICT separation of both expanded complete bounds proves there is
        // no common volume or epsilon-near-contact at all. This is an oriented
        // broad-phase proof, not a joint exception or a measured contact.
        if(expandedOverlap(ba,bb,scale,halo).separated)return 'separated';
        const regions=activeRegions.map(r=>({...r,center:r.center.clone().applyMatrix4(frame.inverse)}));
        if(overlapBoxLocal(ba,bb,regions,scale,halo))return 'joint-local';
      }
      return null;
    };
    function surfacePairs(aNode, bNode, aState, bState, hits) {
      if (!boxesOverlap(aState.box(aNode), bState.box(bNode), epsilon)) return;
      if(activeRegions.length&&spherePairLocal(aNode,bNode,aState,bState)) {
        jointLocalEnclosureCandidates+=aNode.count*bNode.count;return;
      }
      if (aNode.triangles && bNode.triangles) {
        for (const a of aNode.triangles) for (const b of bNode.triangles) {
          if (!boxesOverlap(aState.box(a), bState.box(b), epsilon)) continue;
          if(activeRegions.length&&spherePairLocal(a,b,aState,bState)){jointLocalEnclosureCandidates++;continue;}
          if(activeRegions.length&&(completeTriangleLocal(aState.triangle(a))||completeTriangleLocal(bState.triangle(b)))) {
            jointLocalSingleTriangleCandidates++;continue;
          }
          triangleTests++;
          if (triangleIntersect(aState.triangle(a), bState.triangle(b), epsilon)) {
            if (activeRegions.length) {
              // A sphere is convex. If it contains all six vertices, it also
              // contains both complete triangles and any epsilon-near-contact
              // locus, even when the exact intersection is empty/uncertain.
              // This sufficient enclosure test is stronger than a centroid
              // test and does not mix different spheres point by point.
              const enclosed=entirelyLocal(aState.triangle(a).concat(bState.triangle(b)));
              const locus=enclosed?null:triangleContactLocus(aState.triangle(a),bState.triangle(b),epsilon);
              if(enclosed || locus.points.length && locus.kind!=='unresolved' && entirelyLocal(locus.points)) {
                jointLocalTrianglePairs++; continue;
              }
            }
            hits.push([a.id, b.id]);
            if (hits.length >= triangleLimit) return;
          }
        }
      } else if (aNode.triangles || (!bNode.triangles && aNode.count < bNode.count)) {
        surfacePairs(aNode, bNode.left, aState, bState, hits);
        if (hits.length < triangleLimit) surfacePairs(aNode, bNode.right, aState, bState, hits);
      } else {
        surfacePairs(aNode.left, bNode, aState, bState, hits);
        if (hits.length < triangleLimit) surfacePairs(aNode.right, bNode, aState, bState, hits);
      }
    }
    function pointInside(point, bone, state) {
      // Independent non-axis rays avoid a single edge/vertex hit determining
      // parity. Identical boundary hits are clustered once, without face culling.
      let votes = 0;
      for (const vector of [[1, .371, .529], [.193, 1, .617], [.467, .283, 1]]) {
        const ray = new THREE.Ray(point, new THREE.Vector3(...vector).normalize()), distances = [];
        const visit = node => {
          if (!ray.intersectsBox(state.box(node))) return;
          if (!node.triangles) { visit(node.left); visit(node.right); return; }
          for (const triangle of node.triangles) {
            const at = ray.intersectTriangle(...state.triangle(triangle), false, new THREE.Vector3());
            if (at) { const t = point.distanceTo(at); if (t > epsilon) distances.push(t); }
          }
        };
        visit(bone.tree); distances.sort((a, b) => a - b);
        let count = 0, last = -Infinity;
        for (const distance of distances) if (distance - last > Math.max(1e-10, epsilon * 4)) { count++; last = distance; }
        votes += count % 2;
      }
      return votes >= 2;
    }
    const pairs = [];
    outer: for (const [a, b] of selectedPairs) {
      const sa = stateOf(a), sb = stateOf(b), ba = sa.box(a.tree), bb = sb.box(b.tree);
      if (!boxesOverlap(ba, bb, epsilon)) continue;
      activeRegions = options.includeNaturalJoints ? [] : jointRegions(a, b, W);
      for (const region of activeRegions) jointRegionsUsed.set(region.segment + ':' + region.reason, {
        segment: region.segment, radiusM: region.radius, centerM:region.center.toArray(), reason: region.reason });
      candidatePairs++;
      // Every possible common volume and epsilon-near-contact point belongs
      // to this expanded overlap box. Its containment in ONE joint sphere is
      // a pose-specific spatial proof, never a whole-bone pair exemption.
      if(overlapBoxLocal(ba,bb)){jointLocalOverlapCertificates++;continue;}
      const parentCertificate=parentPartCertificate(a,b,ba,bb);
      if(parentCertificate==='separated'){parentFrameSeparationCertificates++;continue;}
      if(parentCertificate==='joint-local'){jointLocalParentOverlapCertificates++;continue;}
      const hits = []; surfacePairs(a.tree, b.tree, sa, sb, hits);
      let kind = hits.length ? 'surface' : null;
      if (!kind) {
        // A part can contain several disconnected solids. Its whole AABB need
        // not contain the other part even when one component is fully inside.
        // Test every overlapping component and use one seed from each closed
        // connected surface in both directions. Per-component parity also
        // treats overlapping disconnected solids as their geometric union.
        components: for (const ac of a.components) for (const bc of b.components) {
          if (!boxesOverlap(sa.box(ac.tree), sb.box(bc.tree), epsilon)) continue;
          containmentTests++;
          // With no joint exception, one seed suffices after surface crossing
          // was ruled out. An articulation boundary may split the contained
          // region: retain every nonlocal vertex instead of exempting an
          // entire component merely because its first vertex lies at a joint.
          const contained = (component, source, target, targetState) => {
            const indices = activeRegions.length ? component.indices : [component.seed];
            for (const index of indices) {
              const point = source.vertex(index);
              if (entirelyLocal([point]) || !targetState.box(target.tree).containsPoint(point)) continue;
              if (pointInside(point, target, targetState)) return true;
            }
            return false;
          };
          if (contained(ac, sa, bc, sb) || contained(bc, sb, ac, sa)) {
            kind = 'containment'; break components;
          }
        }
      }
      if (kind) pairs.push({ a: a.seg, b: b.seg, aName: a.name, bName: b.name,
        aPart: a.partIndex, bPart: b.partIndex, kind, triangleIntersections: hits.length,
        triangleCountCapped: hits.length >= triangleLimit, firstTriangles: hits[0] || null,
        jointRegions: activeRegions.map(r => ({ segment: r.segment, radiusM: r.radius, centerM:r.center.toArray(), reason: r.reason })) });
      if (pairs.length >= maxPairs) break outer;
    }
    return { colliding: pairs.length > 0, scope, pairs,
      boneParts: bones.length, candidatePairs, triangleTests, containmentTests, jointLocalTrianglePairs,
      jointLocalEnclosureCandidates,jointLocalOverlapCertificates,jointLocalParentOverlapCertificates,parentFrameSeparationCertificates,jointLocalSingleTriangleCandidates,
      selectedBonePairs:selectedPairs.length,totalMovingBonePairs:allPairs.length,
      pairsTruncated: pairs.length >= maxPairs, epsilonM: epsilon,
      articulationPolicy: 'same-frame-excluded-only; one-local-sphere-encloses-common-part-bounds-or-one-complete-triangle-or-entire-established-contact-locus',
      predicate: 'inclusive-surface-contact-or-containment; penetration-depth-not-estimated',
      jointRegions: [...jointRegionsUsed.values()] };
  }
  return { inspect, floorMinimum, boneParts: bones.length, segments: [...new Set(bones.map(b => b.seg))], scope: 'whole-body-bones-joint-local' };
}

// Inventory comparison is diagnostic only. An inherited colliding pair can
// penetrate further during a transition, so matching a baseline is never a
// proof of nonpenetration. Retain the complete unresolved inventory and do not
// manufacture a `verified` or `collisionFree` flag from this comparison.
export function compareCollisionInventories(report, references) {
  if (!Array.isArray(report?.pairs) || !Array.isArray(references)
    || references.some(r => !Array.isArray(r?.pairs))) throw new Error('Invalid collision inventories');
  const key = p => [Math.min(p.aPart, p.bPart), Math.max(p.aPart, p.bPart), p.kind].join(':');
  const known = new Set(references.flatMap(r => r.pairs.map(key)));
  return {
    status: 'diagnostic-only', scope: report.scope,
    unresolvedPairs: report.pairs,
    baselineMatches: report.pairs.filter(p => known.has(key(p))),
    newlyReportedPairs: report.pairs.filter(p => !known.has(key(p))),
    completeInventories: !report.pairsTruncated && references.every(r => !r.pairsTruncated),
    limitation: 'Matching bone pairs do not bound penetration depth or establish a collision-free trajectory.',
  };
}
