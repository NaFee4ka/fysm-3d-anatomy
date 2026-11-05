// Independent exact-cube fixtures for the broadened rigid-bone audit.
import assert from 'node:assert/strict';
import { gzipSync } from 'node:zlib';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createWholeBodyCollision, compareCollisionInventories, triangleContactLocus } from './whole_body_collision.mjs';

const A = createAnatomy(THREE);
const V = (...v) => new THREE.Vector3(...v);
const Q = () => new THREE.Quaternion();
const localProofs=r=>r.jointLocalTrianglePairs+(r.jointLocalOverlapCertificates||0)
  +(r.jointLocalParentOverlapCertificates||0)+(r.jointLocalEnclosureCandidates||0)+(r.jointLocalSingleTriangleCandidates||0);
const vertices = [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]];
const indices = [0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,3,7,6,3,6,2,0,4,7,0,7,3,1,2,6,1,6,5];
function pack(specs) {
  const parts = [], chunks = [];
  for (const { seg, half = .02, center = [0,0,0], name='exact cube',latin } of specs) {
    const p = Uint16Array.from(vertices.flatMap(v => v.map(x => x < 0 ? 0 : 65535)));
    for (let i = p.length - 1; i >= 3; i--) p[i] = (p[i] - p[i-3]) & 65535;
    const t = Uint16Array.from(indices);
    for (let i = t.length - 1; i > 0; i--) t[i] = (t[i] - t[i-1]) & 65535;
    chunks.push(Buffer.from(p.buffer), Buffer.from(t.buffer));
    parts.push({ seg, name,latin, n: 8, m: 12,
      lo: center.map(x => x-half), hi: center.map(x => x+half) });
  }
  return { parts, data: gzipSync(Buffer.concat(chunks)).toString('base64') };
}
function frame(segments, positions) {
  return Object.fromEntries(segments.map((s,i) => [s, { p: V(...positions[i]), q: Q() }]));
}
function packMeshes(meshes) {
  const parts=[],chunks=[];
  for(const {seg,points,faces,name='analytic mesh',latin} of meshes) {
    const lo=[0,1,2].map(k=>Math.min(...points.map(p=>p[k]))),hi=[0,1,2].map(k=>Math.max(...points.map(p=>p[k])));
    const positions=Uint16Array.from(points.flatMap(p=>p.map((v,k)=>hi[k]===lo[k]?0:Math.round((v-lo[k])/(hi[k]-lo[k])*65535))));
    for(let i=positions.length-1;i>=3;i--)positions[i]=(positions[i]-positions[i-3])&65535;
    const triangles=Uint16Array.from(faces.flat());
    for(let i=triangles.length-1;i>0;i--)triangles[i]=(triangles[i]-triangles[i-1])&65535;
    chunks.push(Buffer.from(positions.buffer),Buffer.from(triangles.buffer));
    parts.push({seg,name,latin,n:points.length,m:faces.length,lo,hi});
  }
  return {parts,data:gzipSync(Buffer.concat(chunks)).toString('base64')};
}
let cases = 0;
for (const [a,b] of [['hum_L','thigh_R'],['fore_R','head'],['hand_L','pelvis'],['shank_L','shank_R'],['foot_L','foot_R']]) {
  const c = createWholeBodyCollision(A, THREE, pack([{seg:a},{seg:b}]));
  for (const [d,expected] of [[.039,true],[.04,true],[.041,false]]) {
    const W = frame([a,b],[[0,0,0],[d,0,0]]), result = c.inspect(W);
    assert.equal(result.colliding, expected, `${a}/${b}: cubes separated ${d} m`);
    if (expected) assert.equal(result.pairs[0].a, a);
    const turn = new THREE.Quaternion().setFromAxisAngle(V(.2,.7,.5).normalize(), 1.31), shift = V(1.1,-.7,2.3);
    const moved = Object.fromEntries(Object.entries(W).map(([s,f])=>[s,{p:f.p.clone().applyQuaternion(turn).add(shift),q:turn.clone()}]));
    assert.equal(c.inspect(moved).colliding, expected, 'collision invariant under a shared rigid transform');
    cases += 2;
  }
}
{
  const c = createWholeBodyCollision(A, THREE, pack([{seg:'head',half:.04},{seg:'hand_R',half:.01}]));
  const r = c.inspect(frame(['head','hand_R'],[[0,0,0],[0,0,0]]));
  assert.equal(r.pairs[0].kind, 'containment'); cases++;
}
{
  // An anatomical exception is spatially bounded. The same adjacent bones
  // still collide if their actual mesh overlap lies 12 cm from the joint.
  const W = frame(['hum_L','fore_L'],[[0,0,0],[0,0,0]]);
  const local = createWholeBodyCollision(A, THREE, pack([{seg:'hum_L',half:.01},{seg:'fore_L',half:.01}]));
  assert.equal(local.inspect(W).colliding, false);
  assert(localProofs(local.inspect(W)) > 0);
  assert.equal(local.inspect(W,{includeNaturalJoints:true}).colliding, true);
  const remote = createWholeBodyCollision(A, THREE, pack([
    {seg:'hum_L',half:.01,center:[.12,0,0]}, {seg:'fore_L',half:.01,center:[.12,0,0]},
  ]));
  assert.equal(remote.inspect(W).colliding, true, 'joint-local rule must not exempt the shaft'); cases += 4;
}
{
  const c = createWholeBodyCollision(A, THREE, pack([{seg:'head'},{seg:'head'}]));
  assert.equal(c.inspect(frame(['head'],[[0,0,0]])).colliding, false, 'same rigid frame is explicitly outside moving-pair scope'); cases++;
}
{
  const c = createWholeBodyCollision(A, THREE, pack([{seg:'head',half:.02},{seg:'foot_R',half:.01}]));
  const W = frame(['head','foot_R'],[[0,.1,0],[0,.009,0]]);
  const floor = c.floorMinimum(W);
  assert(Math.abs(floor.minimumY + .001) < 1e-14);
  assert.equal(floor.segment, 'foot_R');
  assert.equal(floor.scope, 'all-decoded-bone-vertices');
  W.foot_R.q.set(0,0,0,0);
  assert.throws(()=>c.inspect(W), /Invalid bone collision transform/);
  assert.throws(()=>c.floorMinimum(W), /Invalid floor transform/); cases += 3;
}
{
  const a = {aPart:1,bPart:7,kind:'surface'}, b={aPart:3,bPart:8,kind:'containment'};
  const comparison = compareCollisionInventories({scope:'test',pairs:[a,b]},[{pairs:[a]}]);
  assert.equal(comparison.status,'diagnostic-only');
  assert.deepEqual(comparison.baselineMatches,[a]);
  assert.deepEqual(comparison.newlyReportedPairs,[b]);
  assert.equal(comparison.unresolvedPairs.length,2);
  assert.equal('verified' in comparison,false);
  assert.equal('collisionFree' in comparison,false); cases++;
}
{
  const c = createWholeBodyCollision(A, THREE, pack([{seg:'head'},{seg:'hand_R'},{seg:'foot_L'}]));
  const W=frame(['head','hand_R','foot_L'],[[0,0,0],[0,0,0],[2,0,0]]);
  const onlyFoot=(a,b)=>a.seg==='foot_L'||b.seg==='foot_L';
  assert.throws(()=>c.inspect(W,{pairFilter:onlyFoot}),/explicit scope label/);
  const filtered=c.inspect(W,{pairFilter:onlyFoot,pairFilterLabel:'fixture-foot-pairs'});
  assert.equal(filtered.scope,'filtered-body-bones:fixture-foot-pairs');
  assert.equal(filtered.colliding,false);
  assert.equal(filtered.selectedBonePairs,2);
  assert.equal(filtered.totalMovingBonePairs,3);
  assert.equal(c.inspect(W).colliding,true,'a filtered diagnostic cannot change the full inspector');cases+=3;
}
{
  const a=[V(-1,-.01,0),V(1,-.01,0),V(0,.01,0)];
  const b=[V(0,-1,-.01),V(0,1,-.01),V(0,0,.01)];
  const locus=triangleContactLocus(a,b);
  assert.equal(locus.kind,'segment');
  assert(locus.points.every(p=>Math.abs(p.x)<1e-12&&Math.abs(p.z)<1e-12&&Math.abs(p.y)<=.010000001));
  assert(Math.abs(locus.points[0].distanceTo(locus.points[1])-.02)<1e-10,'independently known 20 mm segment');
  const coplanar=[V(-.01,-1,0),V(-.01,1,0),V(.01,0,0)];
  const polygon=triangleContactLocus(a,coplanar);
  assert.equal(polygon.kind,'polygon');
  assert(polygon.points.length>=3&&polygon.points.every(p=>Math.abs(p.x)<=.010000001&&Math.abs(p.y)<=.010000001&&p.z===0));
  const W=frame(['thigh_L','shank_L'],[[0,0,0],[0,0,0]]);
  for(const contact of [b,coplanar]) {
    const c=createWholeBodyCollision(A,THREE,packMeshes([
      {seg:'thigh_L',points:a.map(p=>p.toArray()),faces:[[0,1,2]]},
      {seg:'shank_L',points:contact.map(p=>p.toArray()),faces:[[0,1,2]]},
    ]));
    const r=c.inspect(W);
    assert.equal(r.colliding,false,'long source triangles meet only inside the bounded knee joint');
    assert(localProofs(r)>0);
    assert(c.inspect(W,{includeNaturalJoints:true}).colliding);
  }
  const remote=createWholeBodyCollision(A,THREE,pack([
    {seg:'thigh_L',half:.01,center:[.12,0,0]},{seg:'shank_L',half:.01,center:[.12,0,0]},
  ]));
  assert(remote.inspect(W).colliding,'adjacent thigh/shank cubes overlapping 12 cm from knee must remain rejected');
  const both=createWholeBodyCollision(A,THREE,packMeshes([
    {seg:'thigh_L',points:a.concat(a.map(p=>p.clone().add(V(.12,0,0)))).map(p=>p.toArray()),faces:[[0,1,2],[3,4,5]]},
    {seg:'shank_L',points:b.concat(b.map(p=>p.clone().add(V(.12,0,0)))).map(p=>p.toArray()),faces:[[0,1,2],[3,4,5]]},
  ]));
  assert(both.inspect(W).colliding,'disconnected remote contact must not inherit the local piece exception');
  const contained=createWholeBodyCollision(A,THREE,pack([
    {seg:'thigh_L',half:.16},{seg:'shank_L',half:.005,center:[.12,0,0]},
  ]));
  assert.equal(contained.inspect(W).pairs[0].kind,'containment','remote enclosed volume stays rejected');
  const degenerate=triangleContactLocus([V(0,0,0),V(.01,0,0),V(.02,0,0)],a);
  assert.equal(degenerate.kind,'unresolved','uncertain degenerate loci get no joint exemption');
  cases+=9;
  const crossing=[V(-.2,0,-.2),V(-.2,0,-.2),V(.2,0,.2)];
  const point=triangleContactLocus(crossing,a);
  assert.equal(point.kind,'point','duplicate source vertices establish a precise segment, not an uncertain normal');
  assert(point.points.every(p=>p.length()<1e-12));
  assert.equal(triangleContactLocus([V(0,0,0),V(0,0,0),V(0,0,0)],a).kind,'point');
  const duplicate=createWholeBodyCollision(A,THREE,packMeshes([
    {seg:'thigh_L',points:a.map(p=>p.toArray()),faces:[[0,1,2]]},
    {seg:'shank_L',points:crossing.map(p=>p.toArray()),faces:[[0,1,2]]},
  ]));
  assert.equal(duplicate.inspect(W).colliding,false,'provably joint-local exact segment contact is bounded too');
  const wide=[V(-1,-.1,0),V(1,-.1,0),V(0,.1,0)];
  const spanning=createWholeBodyCollision(A,THREE,packMeshes([
    {seg:'thigh_L',points:wide.map(p=>p.toArray()),faces:[[0,1,2]]},
    {seg:'shank_L',points:b.map(p=>p.toArray()),faces:[[0,1,2]]},
  ]));
  assert(spanning.inspect(W).colliding,'centroid inside a joint cannot exempt a contact segment with remote endpoints');
  const lineA=[V(-.1,0,0),V(-.1,0,0),V(.1,0,0)],lineB=[V(-.08,0,0),V(.08,0,0),V(.08,0,0)];
  const overlap=triangleContactLocus(lineA,lineB);
  assert.equal(overlap.kind,'segment');
  assert(overlap.points.some(p=>p.x<=-.079999)&&overlap.points.some(p=>p.x>=.079999),'both ends of a degenerate collinear overlap are retained');
  cases+=6;
}
{
  const matching=[{seg:'foot_L',half:.01,name:'I плюсневая кость',latin:'os metatarsi I'},
    {seg:'toes_L',half:.01,center:[0,0,.02],name:'Проксимальная фаланга большого пальца стопы',latin:'phalanx proximalis'}];
  const W=frame(['foot_L','toes_L'],[[0,0,0],[0,0,0]]);
  const correct=createWholeBodyCollision(A,THREE,pack(matching)),r=correct.inspect(W);
  assert.equal(r.colliding,false,'matched MTP head/base contact is locally bounded');
  assert(r.jointRegions.every(j=>j.radiusM===.015&&j.reason.includes('matched MTP I')));
  const moved=Object.fromEntries(Object.entries(W).map(([s,f])=>[s,{
    p:f.p.clone().applyQuaternion(new THREE.Quaternion().setFromAxisAngle(V(.3,.7,.2).normalize(),1.4)).add(V(.7,.8,-.4)),
    q:new THREE.Quaternion().setFromAxisAngle(V(.3,.7,.2).normalize(),1.4)}]));
  assert.equal(correct.inspect(moved).colliding,false,'source cap spheres follow their own rigid frames');
  const mismatch=createWholeBodyCollision(A,THREE,pack([matching[0],{...matching[1],name:'Проксимальная фаланга II пальца стопы'}]));
  assert(mismatch.inspect(W).colliding,'a mismatched toe gets no MTP exception');
  const distal=createWholeBodyCollision(A,THREE,pack([matching[0],{...matching[1],name:'Дистальная фаланга большого пальца стопы',latin:'phalanx distalis'}]));
  assert(distal.inspect(W).colliding,'distal phalanges do not inherit a proximal MTP exception');
  const remote=createWholeBodyCollision(A,THREE,pack(matching.map(s=>({...s,half:.03,center:[0,0,0]}))));
  assert(remote.inspect(W).colliding,'remote contact of the same matched pair is rejected outside both small cap spheres');
  cases+=6;
}
{
  // The inclusive guard sees a 0.5575 nm gap, while the exact plane-line locus
  // is empty. Both entire triangles fit within one unchanged knee sphere.
  const gap=5.575e-10;
  const a=[[-.01,-.01,0],[.01,-.01,0],[0,.01,0]],b=[[0,0,gap],[0,0,.01],[0,.01,.01]];
  assert.equal(triangleContactLocus(a.map(p=>V(...p)),b.map(p=>V(...p))).kind,'unresolved');
  const inspectAt=x=>{
    const data=packMeshes([{seg:'thigh_L',points:a.map(p=>[p[0]+x,p[1],p[2]]),faces:[[0,1,2]]},
      {seg:'shank_L',points:b.map(p=>[p[0]+x,p[1],p[2]]),faces:[[0,1,2]]}]);
    return createWholeBodyCollision(A,THREE,data).inspect(frame(['thigh_L','shank_L'],[[0,0,0],[0,0,0]]));
  };
  const local=inspectAt(0),remote=inspectAt(.12);
  assert.equal(local.colliding,false,'whole-triangle convex enclosure proves the entire near-contact local');
  assert(local.jointLocalOverlapCertificates>0,'epsilon-expanded overlap is certified despite the empty raw intersection');
  assert.equal(local.jointLocalTrianglePairs,0,'a spatial certificate is not reported as a measured triangle contact');
  assert.equal(remote.colliding,true,'the same epsilon-near-contact outside the joint remains rejected');
  assert.equal(remote.jointLocalOverlapCertificates,0,'a remote near-contact cannot use a local overlap certificate');
  cases+=3;
}
{
  // Two 15 mm spheres can jointly cover the vertices of a 40 mm-wide contact
  // polygon, but neither one can contain that entire polygon. It must fail.
  const face=[[-.02,-.001,0],[.02,-.001,0],[0,.001,0]];
  const body=(seg,side)=>({seg,name:side<0?'I плюсневая кость':'Проксимальная фаланга большого пальца стопы',
    latin:side<0?'os metatarsi I':'phalanx proximalis',
    points:face.concat(Array.from({length:30},()=>[side*.01,0,0])),faces:[[0,1,2]]});
  const data=packMeshes([body('foot_L',-1),body('toes_L',1)]);
  const result=createWholeBodyCollision(A,THREE,data).inspect(frame(['foot_L','toes_L'],[[0,0,0],[0,0,0]]));
  assert.equal(result.pairs[0].jointRegions.length,2);
  assert(face.every(p=>result.pairs[0].jointRegions.some(r=>V(...p).distanceTo(V(...r.centerM))<r.radiusM)),
    'fixture vertices are individually covered by the union');
  assert(result.colliding,'point-by-point union coverage cannot exempt an entire contact spanning both spheres');
  assert.equal(result.jointLocalSingleTriangleCandidates,0,'no complete triangle fits in any one sphere');
  cases+=3;
}
{
  // These crossing rectangular solids have a small exact common box around
  // the joint. Turning both by 45 degrees makes their WORLD AABB overlap
  // broad, while the actual parent-frame bounds still prove it local.
  const bar=(seg,axis)=>({seg,points:vertices.map(v=>v.map((x,k)=>x*(k===axis?.2:.005))),
    faces:Array.from({length:indices.length/3},(_,i)=>indices.slice(3*i,3*i+3))});
  const c=createWholeBodyCollision(A,THREE,packMeshes([bar('thigh_L',0),bar('shank_L',1)]));
  const turn=new THREE.Quaternion().setFromAxisAngle(V(0,0,1),Math.PI/4),shift=V(.7,-.9,1.2);
  const W=Object.fromEntries(['thigh_L','shank_L'].map(seg=>[seg,{q:turn.clone(),p:shift.clone()}]));
  const r=c.inspect(W);
  assert.equal(r.colliding,false);
  assert.equal(r.jointLocalOverlapCertificates,0,'world-axis overlap is too loose in this fixture');
  assert.equal(r.jointLocalParentOverlapCertificates,1,'complete transformed part bounds certify the same local common volume');
  assert.equal(r.parentFrameSeparationCertificates,0,'an actual common box is not a separation');
  assert.equal(r.triangleTests+r.containmentTests,0,'a proved whole-part overlap enclosure needs no candidate contact claim');
  assert(c.inspect(W,{includeNaturalJoints:true}).colliding,'the fixture solids really intersect');
  const remote=createWholeBodyCollision(A,THREE,pack([
    {seg:'thigh_L',half:.16},{seg:'shank_L',half:.005,center:[.12,0,0]},
  ]));
  const outside=remote.inspect(W);
  assert.equal(outside.pairs[0].kind,'containment','a remote volume in the same parent frame remains rejected');
  assert.equal(outside.jointLocalParentOverlapCertificates,0);
  assert.equal(outside.parentFrameSeparationCertificates,0,'remote containment cannot be culled by a false separation');
  const epsilon=.001;
  const boundary=createWholeBodyCollision(A,THREE,pack([
    {seg:'thigh_L',half:.0001,center:[.055-epsilon/2,0,0]},
    {seg:'shank_L',half:.0001,center:[.055-epsilon/2,0,0]},
  ]));
  assert.equal(boundary.inspect(W,{epsilon}).jointLocalParentOverlapCertificates,0,
    'the expanded epsilon halo crossing the sphere boundary cannot receive this certificate');
  const shoulder=createWholeBodyCollision(A,THREE,pack([
    {seg:'hum_L',half:.01,center:[.12,0,0]},{seg:'clav_L',half:.01,center:[.12,0,0]},
  ]));
  for(const auxiliary of [undefined,{p:V(0,0,0),q:new THREE.Quaternion(0,0,0,0)},
    {p:V(NaN,0,0),q:Q()},{p:V(0,0,0),q:new THREE.Quaternion(1e308,0,0,0)},
    {p:V(0,0,0),q:{x:0,y:0,z:0,w:1}}]) {
    const invalid={hum_L:{p:V(0,0,0),q:turn.clone()},clav_L:{p:V(0,0,0),q:turn.clone()}};
    if(auxiliary)invalid.scap_L=auxiliary;
    const hit=shoulder.inspect(invalid);
    assert(hit.colliding,'an unavailable or invalid auxiliary parent never exempts a remote contact');
    assert.equal(hit.jointLocalParentOverlapCertificates,0);
  }
  cases+=14;
}
{
  // Parallel, closed rectangular solids are separated in their actual parent
  // basis, although a common 45-degree turn makes their world AABBs overlap.
  const bar=(seg,y)=>({seg,points:vertices.map(v=>[v[0]*.2,y+v[1]*.005,v[2]*.005]),
    faces:Array.from({length:indices.length/3},(_,i)=>indices.slice(3*i,3*i+3))});
  const q=new THREE.Quaternion().setFromAxisAngle(V(0,0,1),Math.PI/4),p=V(.8,-.7,.4);
  const W=Object.fromEntries(['thigh_L','shank_L'].map(seg=>[seg,{q:q.clone(),p:p.clone()}]));
  for(const [gap,separated] of [[.02,true],[.5e-9,false],[10e-9,true]]) {
    const collider=createWholeBodyCollision(A,THREE,packMeshes([bar('thigh_L',0),bar('shank_L',.01+gap)]));
    const r=collider.inspect(W);
    assert.equal(r.candidatePairs,1,'the rotated world bounds alone cannot resolve this fixture');
    assert.equal(r.parentFrameSeparationCertificates,separated?1:0);
    assert.equal(r.jointLocalParentOverlapCertificates+r.jointLocalOverlapCertificates,0,'separation is not reported as joint-local contact');
    assert.equal(r.colliding,!separated,'the inclusive epsilon-near contact must still be checked');
    const unrestricted=collider.inspect(W,{includeNaturalJoints:true});
    assert.equal(unrestricted.colliding,!separated,'strict separation is ordinary broad phase, independent of allowed joint contacts');
    assert.equal(unrestricted.parentFrameSeparationCertificates,separated?1:0);
    cases+=2;
  }
}
{
  const cube=(center,half)=>vertices.map(v=>v.map((x,k)=>center[k]+x*half));
  const combined=specs=>({points:specs.flatMap(([center,half])=>cube(center,half)),
    faces:specs.flatMap((_,i)=>Array.from({length:indices.length/3},(_,f)=>indices.slice(3*f,3*f+3).map(j=>j+8*i)))});
  const rod={seg:'thigh_L',points:vertices.map(v=>[v[0]*.2,v[1]*.005,v[2]*.005]),
    faces:Array.from({length:indices.length/3},(_,i)=>indices.slice(3*i,3*i+3))};
  const W=frame(['thigh_L','shank_L'],[[0,0,0],[0,0,0]]);
  const inspectCubes=center=>createWholeBodyCollision(A,THREE,packMeshes([rod,
    {seg:'shank_L',...combined([[[0,0,0],.008],[center,.008]])}])).inspect(W);
  const local=inspectCubes([.12,.08,0]);
  assert.equal(local.colliding,false,'the actual common volume is local despite broad complete-part bounds');
  assert(local.jointLocalSingleTriangleCandidates>0,'one complete local triangle discharges individual surface candidates');
  assert.equal(local.jointLocalParentOverlapCertificates+local.jointLocalOverlapCertificates,0,'this fixture needs triangle-level locality');
  assert(local.containmentTests>0,'surface certificates never skip the remaining component-volume pass');
  const remote=inspectCubes([.12,0,0]);
  assert(remote.colliding,'a disconnected remote surface in the SAME part must still be checked');
  const containing=createWholeBodyCollision(A,THREE,packMeshes([
    {seg:'thigh_L',...combined([[[0,0,0],.16]])},
    {seg:'shank_L',...combined([[[0,0,0],.005],[[.12,0,0],.005]])},
  ])).inspect(W);
  assert.equal(containing.pairs[0].kind,'containment','a local first component cannot hide a remote contained second component');
  const spanning=[[-.01,0,0],[.01,0,0],[.12,.01,0]];
  const partial=createWholeBodyCollision(A,THREE,packMeshes([
    {seg:'thigh_L',points:spanning,faces:[[0,1,2]]},{seg:'shank_L',points:spanning,faces:[[0,1,2]]},
  ])).inspect(W);
  assert(partial.colliding,'two inside vertices and an inside centroid do not enclose the complete triangle');
  assert.equal(partial.jointLocalSingleTriangleCandidates,0);
  const epsilon=1e-6,at=x=>[[x,-.0001,0],[x,.0001,0],[x,0,.0001]];
  const halo=createWholeBodyCollision(A,THREE,packMeshes([
    {seg:'thigh_L',points:at(.055+epsilon/4),faces:[[0,1,2]]},
    {seg:'shank_L',points:at(.055-epsilon/2),faces:[[0,1,2]]},
  ])).inspect(W,{epsilon});
  assert.equal(halo.jointLocalSingleTriangleCandidates,0,'the complete triangle requires enough inward margin for its epsilon halo');
  assert(halo.colliding,'an epsilon-near counterpart outside the sphere remains unresolved');
  cases+=5;
}
console.log(JSON.stringify({pass:true,cases,scope:'whole-body-bones-joint-local',notTested:'soft-tissue contact or physiological accuracy'}));
