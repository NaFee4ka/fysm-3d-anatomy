// Reproducible diagnostic inventory. This intentionally does not turn inherited
// collisions into a pass. Run with --tracks transitions/data.json to audit saved
// trajectory frames and one independently sampled midpoint per interval too.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createTransitions } from './transition_runtime.mjs';
import { createWholeBodyCollision, compareCollisionInventories } from './whole_body_collision.mjs';

const root = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const option = name => { const i = argv.indexOf(name); return i < 0 ? null : argv[i + 1]; };
const output = path.resolve(option('--out') || '/tmp/fysm-whole-body-audit.json');
const tracksPath = option('--tracks');
const preview = argv.includes('--preview');
const samplesPerTrack = option('--samples') ? Number(option('--samples')) : null;
if (samplesPerTrack != null && (!Number.isInteger(samplesPerTrack) || samplesPerTrack < 2)) throw new Error('At least two integer samples per track are required');
if (preview && !tracksPath) throw new Error('--preview requires --tracks');
const read = name => JSON.parse(fs.readFileSync(path.resolve(root, name)));
const hash = name => createHash('sha256').update(fs.readFileSync(path.resolve(root,name))).digest('hex');
const sources = ['anatomy3.js','body_collision.mjs','collision_geometry.mjs','whole_body_collision.mjs',
  'whole_body_collision_audit.mjs','skull/bones_bp3d.json','poses.json','practice/poses.json'];
if (tracksPath) sources.push(tracksPath,'transition_runtime.mjs');
if (sources.some(s => path.resolve(root,s) === output)) throw new Error('Audit output must not replace an input');
const sourceSha256 = Object.fromEntries(sources.map(s => [s,hash(s)]));
const A = createAnatomy(THREE), collider = createWholeBodyCollision(A,THREE,read('skull/bones_bp3d.json'));
const staticData = read('poses.json'), practiceData = read('practice/poses.json').poses;
const trackData = tracksPath ? read(tracksPath) : null;
const configs = trackData?.endpoints || { ...Object.fromEntries(A.POSES.map(p=>[p.id,staticData[p.id]])), ...practiceData };
if (Object.keys(configs).filter(id=>!id.startsWith('__')).length !== 30) throw new Error('Exactly 30 public endpoint configurations are required');
const world = data => {
  const conf = A.poseFromData(data);
  // FYSM endpoints are explicitly normalised by the existing page adapter.
  if (data.rootP) {
    for (const q of Object.values(conf.local)) q.normalize(); conf.rootQ.normalize();
    return A.computeWorld(conf.local,conf.rootQ,new THREE.Vector3(...data.rootP));
  }
  return A.placeW(conf.local,conf.rootQ);
};
const baseline = {};
for (const [id,data] of Object.entries(configs)) {
  const W = world(data), collision = collider.inspect(W);
  if (collision.pairsTruncated) throw new Error('A complete baseline inventory is required');
  baseline[id] = { collision, floor:collider.floorMinimum(W) };
  console.log(`${id}: ${collision.pairs.length} unresolved bone pairs; ${collision.jointLocalTrianglePairs} bounded-joint triangle contacts`);
}
const trajectories = {};
if (tracksPath) {
  const data = trackData, runtime = preview ? null : createTransitions(THREE,A,data);
  if (!preview && !runtime.valid) throw new Error('Transition runtime rejects this package');
  const samplePreview = (track,t) => {
    // Explicit diagnostic sampler for unpublished input. It never changes
    // `verified`, invents release checks, or feeds the page's runtime adapter.
    const frames = track.frames;
    if (!Array.isArray(frames) || frames.length < 2) throw new Error('Missing preview frames');
    let lo=0,hi=frames.length-1;
    t = Math.max(0,Math.min(track.duration,t));
    while(hi-lo>1){const mid=(lo+hi)>>1;if(frames[mid].t<=t)lo=mid;else hi=mid;}
    const a=frames[lo], b=frames[hi], u=(t-a.t)/(b.t-a.t);
    const conf = {local:Object.fromEntries(A.SEG.map(s=>[s.name,new THREE.Quaternion(...a.local[s.name]).normalize()
      .slerp(new THREE.Quaternion(...b.local[s.name]).normalize(),u).normalize()])),
      rootQ:new THREE.Quaternion(...a.root).normalize().slerp(new THREE.Quaternion(...b.root).normalize(),u).normalize(),
      rootP:new THREE.Vector3(...a.rootP).lerp(new THREE.Vector3(...b.rootP),u)};
    conf.rootP.y += 4*u*(1-u)*(a.floorLiftM||0);
    return conf;
  };
  const category = seg => /^(hum|fore|ulna|hand|f[1-5])/.test(seg) ? 'arm'
    : /^(thigh|shank|foot|toes)/.test(seg) ? 'leg'
      : /^(clav|scap)/.test(seg) ? 'girdle' : 'torso';
  const major = p => !p.jointRegions?.length && [category(p.a),category(p.b)].every(c=>c!=='girdle')
    && (category(p.a)!=='torso'||category(p.b)!=='torso');
  for (const track of Object.values(data.tracks)) {
    const references = [baseline[track.from]?.collision,baseline[track.to]?.collision].filter(Boolean);
    const times = samplesPerTrack ? Array.from({length:samplesPerTrack},(_,i)=>track.duration*i/(samplesPerTrack-1))
      : [...new Set(track.frames.flatMap((f,i,all)=> i+1<all.length ? [f.t,(f.t+all[i+1].t)/2] : [f.t]))];
    const samples = [];
    for (const t of times) {
      const conf = preview ? samplePreview(track,t) : runtime.sampleTrack(track.id,t), W = A.computeWorld(conf.local,conf.rootQ,conf.rootP);
      const collision = collider.inspect(W), comparison = compareCollisionInventories(collision,references);
      samples.push({t,unresolvedPairs:collision.pairs,newlyReportedPairs:comparison.newlyReportedPairs,
        newRegionalPairs:comparison.newlyReportedPairs.filter(major),
        floor:collider.floorMinimum(W)});
    }
    trajectories[track.id] = {from:track.from,to:track.to,sampleCount:times.length,
      maxStepSeconds:Math.max(...times.slice(1).map((t,i)=>t-times[i])),samples};
    const regional = new Map(samples.flatMap(s=>s.newRegionalPairs.map(p=>[p.a+':'+p.b,p])));
    console.log(`${track.id}: ${times.length} states; ${regional.size} NEW regional segment pairs: ${[...regional.keys()].join(', ')}`);
  }
}
for (const [s,digest] of Object.entries(sourceSha256)) if (hash(s) !== digest) throw new Error('Audit input changed during execution: '+s);
const report = {schemaVersion:1,status:'diagnostic-only',complete:true,scope:collider.scope,
  preview,inputVerified:trackData?.verified??null,sampling:samplesPerTrack?'uniform-coarse':'saved-frames-and-midpoints',
  boneParts:collider.boneParts,poseCount:30,sourceSha256,
  baseline,trajectories,
  limitations:[
    'Same rigid-frame parts are outside the moving-pair scope. Natural articulation exceptions are bounded joint spheres.',
    'Matched metatarsal/proximal-phalanx pairs use separate 15 mm source-cap/base spheres following their own bones; the rig still has one central toe hinge, not five independent toe joints.',
    'All unresolved pairs are retained, including registration overlaps, opposite feet and same-side grasps.',
    'The inclusive surface predicate does not distinguish tangential contact from penetration or measure penetration depth.',
    'A pair matching an endpoint inventory may penetrate more deeply during motion; baseline matching is not a nonpenetration proof.',
    'This is discrete rigid-bone geometry. Soft-tissue contact, contact forces and continuous collision are not established.',
  ]};
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(`Wrote diagnostic inventory to ${output}; this is not a collision-free certificate.`);
