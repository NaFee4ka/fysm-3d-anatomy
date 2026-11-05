// Regression for exact BP3D ankle pairs whose local contact was previously
// classified differently after rotating the entire body. Payload bytes are
// copied unchanged from the production atlas, not rebuilt from a proxy mesh.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gunzipSync, gzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createWholeBodyCollision } from './whole_body_collision.mjs';
const A=createAnatomy(THREE), V=(...v)=>new THREE.Vector3(...v);
const bytes=fs.readFileSync(new URL('./skull/bones_bp3d.json',import.meta.url));
const sourceSha256=createHash('sha256').update(bytes).digest('hex');
const atlas=JSON.parse(bytes), raw=gunzipSync(Buffer.from(atlas.data,'base64'));
const fixtures=[
  {id:'S4 fibula/talus',parts:[90,92],world:{
    shank_L:{p:[.15411170121322093,-.40169318375857,-.00556637535196724],q:[.0020568789342746163,-.00003646679599088342,.06955463673043881,.9975760223805099]},
    foot_L:{p:[.2140226117790444,-.8436351583820504,-.011395085421347949],q:[.001346765188791993,-.00008597846931987491,.06955459314979372,.9975772338034767]},
  }},
  {id:'S7 tibia/calcaneus',parts:[89,93],world:{
    shank_L:{p:[.4121531860580703,.0802267919139261,.27176270979470646],q:[.5368677666344829,.3634601742863973,.4442476437283976,.618315238287373]},
    foot_L:{p:[.4787282332997713,.06685143620128467,-.1690602285199025],q:[.5972898043100054,.40698902131577125,.40474585084835923,.5601657097830818]},
  }},
  {id:'D6 tibia/cuneiform',parts:[119,127],certificate:'separated',world:{
    shank_R:{p:[-.37284401415934176,-.1656730429069235,-.24976129906010425],q:[.8153278334601002,.1737565970798008,-.35929724245103506,.4194694989192771]},
    foot_R:{p:[-.6305261611144646,.09964401203431703,-.4990458182302795],q:[-.8959528602475131,-.32774723399835337,.29373327983475817,-.059757703589119276]},
  }},
  {id:'C8 metatarsal/proximal toe',parts:[99,104],certificate:'one-triangle',radius:.015,regions:2,world:{
    shank_L:{p:[.12774923341085442,-.12901283240606842,.3895634536907391],q:[-.24517051009162522,.01714351901518435,.04250561790580162,.9683959898634622]},
    foot_L:{p:[.16617408805740538,-.5218374223596124,.597285315980729],q:[-.006223180286417274,.026820253581389564,.0371658686916901,.9989297493964954]},
    toes_L:{p:[.1813063779840758,-.5752926285337393,.7474803716065943],q:[-.268259662516312,.016122090374222867,.04290344974175597,.962255748576675]},
  }},
];
let seed=419983;
const random=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
const results=[];
for(const fixture of fixtures) {
  let offset=0;
  const parts=[],chunks=[];
  for(const [i,part] of atlas.parts.entries()) {
    const length=6*(part.n+part.m);
    if(fixture.parts.includes(i)){parts.push({...part});chunks.push(raw.subarray(offset,offset+length));}
    offset+=length;
  }
  assert.equal(parts.length,2);
  const collider=createWholeBodyCollision(A,THREE,{parts,data:gzipSync(Buffer.concat(chunks)).toString('base64')});
  const source=Object.fromEntries(Object.entries(fixture.world).map(([id,w])=>[id,{p:V(...w.p),q:new THREE.Quaternion(...w.q)}]));
  const original=collider.inspect(source);
  assert.equal(original.colliding,false,fixture.id);
  if(fixture.certificate==='one-triangle') {
    assert(original.jointLocalSingleTriangleCandidates>0,'the exact MTP case needs individual whole-triangle enclosures');
    assert(original.containmentTests>0,'individual surface candidates do not bypass the later whole-volume checks');
  } else {
    assert.equal(fixture.certificate==='separated'?original.parentFrameSeparationCertificates:original.jointLocalParentOverlapCertificates,1,
      'the exact formerly failing world configuration needs the specified parent-frame proof');
    assert.equal(original.triangleTests+original.containmentTests,0,'this proves a spatial enclosure; it does not invent a measured contact');
  }
  assert.equal(original.jointRegions[0].radiusM,fixture.radius||.045,'existing articulation regions are unchanged');
  const unrestricted=collider.inspect(source,{includeNaturalJoints:true});
  let parentCertificates=0,worldCertificates=0,separationCertificates=0,worldBroadPhaseSeparations=0,singleTriangleCandidates=0;
  for(let i=0;i<1000;i++) {
    const rotation=new THREE.Quaternion().setFromEuler(new THREE.Euler(random()*2*Math.PI,random()*2*Math.PI,random()*2*Math.PI));
    const translation=V(10*(random()-.5),10*(random()-.5),10*(random()-.5));
    const W=Object.fromEntries(Object.entries(source).map(([id,w])=>[id,{
      p:w.p.clone().applyQuaternion(rotation).add(translation),q:rotation.clone().multiply(w.q),
    }]));
    const report=collider.inspect(W);
    assert.equal(report.colliding,false,fixture.id+' global placement '+i);
    assert(report.jointRegions.every(r=>r.radiusM===(fixture.radius||.045)));
    assert.equal(report.jointRegions.length,report.candidatePairs*(fixture.regions||1));
    const wholeCertificates=report.jointLocalParentOverlapCertificates+report.jointLocalOverlapCertificates+report.parentFrameSeparationCertificates;
    if(fixture.certificate==='one-triangle'&&report.candidatePairs&&!wholeCertificates) {
      assert(report.jointLocalSingleTriangleCandidates>0&&report.containmentTests>0);
    } else assert.equal(wholeCertificates,report.candidatePairs);
    parentCertificates+=report.jointLocalParentOverlapCertificates;
    worldCertificates+=report.jointLocalOverlapCertificates;
    separationCertificates+=report.parentFrameSeparationCertificates;
    worldBroadPhaseSeparations+=report.candidatePairs===0?1:0;
    singleTriangleCandidates+=report.jointLocalSingleTriangleCandidates;
  }
  results.push({id:fixture.id,sourcePartIndices:fixture.parts,globalPlacements:1000,parentCertificates,worldCertificates,separationCertificates,worldBroadPhaseSeparations,singleTriangleCandidates,
    unrestrictedPredicateColliding:unrestricted.colliding});
}
assert.deepEqual(fs.readFileSync(new URL('./skull/bones_bp3d.json',import.meta.url)),bytes,'source bones are immutable');
console.log(JSON.stringify({status:'PASS',sourceSha256,results,scope:'bounded rigid-bone articulation; not soft-tissue or continuous full-body contact'},null,2));
