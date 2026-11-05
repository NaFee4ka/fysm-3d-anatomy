// Coordinate authoring only: these tests import no old geometry PASS state.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { A, THREE } from './cycle_lib.mjs';
import { createTransitions } from './transition_runtime.mjs';
const source = fs.readFileSync(new URL('transition_build.mjs', import.meta.url), 'utf8');
const names = [...vm.runInNewContext(source.match(/const originalInputs = (\[[\s\S]*?\]);/)[1])];
const extract = (a, b, globals) => vm.runInNewContext('(' + source.split(a + '\n')[1].split(b)[0].trim() + ')', globals);
const validate = extract('// CANDIDATE_VALIDATOR_BEGIN', '// CANDIDATE_VALIDATOR_END', { createTransitions });
const select = extract('// MOTION_REPAIR_SELECTOR_BEGIN', '// MOTION_REPAIR_SELECTOR_END', { validateRawPlan: validate });
const archive = fs.readFileSync(new URL('transitions/plan.json.gz', import.meta.url));
const data = JSON.parse(gunzipSync(archive)), before = JSON.stringify(data);
const oldArchiveSha256 = createHash('sha256').update(archive).digest('hex');
const sourceHashes = { ...data.sourceHashes, 'transition_bow_release.mjs': 'c'.repeat(64),
  'transition_collision.mjs': 'd'.repeat(64), 'transition_build.mjs': createHash('sha256').update(source).digest('hex') };
const context = () => ({ A, THREE, catalog: structuredClone(data.poses), endpoints: structuredClone(data.endpoints),
  sourceHashes: { ...sourceHashes }, sourceNames: [...names], oldArchiveSha256 });
const selected = select(data, context());
assert.equal(JSON.stringify(data), before, 'coordinate authoring selection must not mutate the old archive');
assert.equal(JSON.stringify(selected.preparePoseIds), JSON.stringify(['bow', 'half']));
assert.equal(JSON.stringify(selected.origin.discardedTrackIds), JSON.stringify(['release-bow', 'release-half']));
assert.equal(JSON.stringify(selected.requests.map(r => r.id)), JSON.stringify(['release-bow', 'release-half']));
assert.equal(Object.keys(selected.retainedTracks).length, Object.keys(data.tracks).length - 2);
let retainedFrames = 0;
for (const [id, track] of Object.entries(selected.retainedTracks)) {
  assert.equal(JSON.stringify(track), JSON.stringify(data.tracks[id]), 'all retained coordinates, endpoints, phases and times stay exact');
  retainedFrames += track.frames.length;
}
assert.equal(JSON.stringify(selected.retainedFallbacks), JSON.stringify(data.neighbourFallbacks));
const alters = [
  c => { c.sourceHashes['anatomy3.js'] = '0'.repeat(64); },
  c => { c.sourceHashes['muscle_surface_render.js'] = '0'.repeat(64); },
  c => { c.sourceHashes['practice/poses.json'] = '0'.repeat(64); },
  c => { c.sourceHashes['whole_body_collision.mjs'] = '0'.repeat(64); },
  c => { c.endpoints.__neutral.rootP[1] += 1e-9; },
  c => { c.endpoints.half.rootP[0] += 1e-9; },
  c => { c.endpoints.bow.local.hand_L[0] += 1e-9; },
  c => { c.endpoints.dog.sourceLocal.hum_R[0] += 1e-9; },
  c => { c.endpoints.plank.sourceRootP[1] += 1e-9; },
  c => { c.endpoints.half.groundOffsetM += 1e-9; },
  c => { c.endpoints.bow.extraSemantic = true; },
  c => { c.endpoints.extra = structuredClone(c.endpoints.bow); },
];
for (const [i, alter] of alters.entries()) {
  const c = context(); alter(c);
  assert.throws(() => select(data, c), /Bow\/half motion repair rejected/, 'selection rejection ' + i);
}
const incident = structuredClone(data); delete incident.coordinateOrigin;
incident.tracks.extraBow = structuredClone(incident.tracks['release-bow']); incident.tracks.extraBow.id = 'extraBow';
assert.throws(() => select(incident, context()), /unexpected incident track/);
const missing = structuredClone(data); delete missing.coordinateOrigin; delete missing.tracks['release-struna-02'];
assert.throws(() => select(missing, context()), /missing canonical release/);
// A pure parser fixture uses old angle rows as UNTRUSTED candidate coordinates;
// it establishes only the new input/provenance contract, never physical safety.
const candidate = structuredClone(data), c = context();
candidate.sourceHashes = { ...c.sourceHashes };
candidate.tracks = { ...structuredClone(selected.retainedTracks),
  'release-bow': structuredClone(data.tracks['release-bow']), 'release-half': structuredClone(data.tracks['release-half']) };
candidate.coordinateOrigin = { ...structuredClone(selected.origin), regeneratedTrackIds: ['release-bow', 'release-half'] };
assert.equal(validate(candidate, c), true, 'a complete still-unverified repair coordinate candidate is structurally usable');
const corruptions = [
  p => { p.coordinateOrigin.kind = 'partial-coordinate-replanning'; },
  p => { p.coordinateOrigin.affectedPoseIds.reverse(); },
  p => { p.coordinateOrigin.endpointRowsPreserved = false; },
  p => { p.coordinateOrigin.endpointCount = 30; },
  p => { p.coordinateOrigin.retainedCoordinateTracksPreserved = false; },
  p => { p.coordinateOrigin.regeneratedRequests.pop(); },
  p => { p.coordinateOrigin.regeneratedTrackIds.pop(); },
  p => { p.coordinateOrigin.discardedTrackIds.pop(); },
  p => { p.coordinateOrigin.retainedTrackIds.push('release-bow'); },
  p => { p.coordinateOrigin.retainedTrackIds.pop(); },
  p => { p.coordinateOrigin.oldTrackCount++; },
  p => { p.coordinateOrigin.regeneratedFallbacks.push({ from: 'bow', to: '__neutral', reason: 'must not fallback' }); },
  p => { p.coordinateOrigin.retainedNeighbourFallbacks = []; },
  p => { p.neighbourFallbacks[0].reason += '!'; },
  p => { p.coordinateOrigin.allowedChangedFiles.push('skull/bones_bp3d.json'); },
  p => { p.coordinateOrigin.oldSourceHashes['anatomy3.js'] = '0'.repeat(64); },
  p => { p.coordinateOrigin.changedSourceFiles.pop(); },
  p => { p.coordinateOrigin.replanningProducerSha256 = '0'.repeat(64); },
];
for (const [i, alter] of corruptions.entries()) {
  const invalid = structuredClone(candidate); alter(invalid);
  assert.equal(validate(invalid, c), false, 'repair-origin rejection ' + i);
}
// Bind the actual half waypoint implementation to the confirmed relative path.
const clone = conf => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, conf.local[s.name].clone()])),
  rootQ: conf.rootQ.clone(), rootP: conf.rootP.clone() });
const waypoint = extract('// HALF_CLAVICLE_WAYPOINT_BEGIN', '// HALF_CLAVICLE_WAYPOINT_END', { clone });
const from = ep => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...ep.local[s.name]).normalize()])),
  rootQ: new THREE.Quaternion(...ep.root).normalize(), rootP: new THREE.Vector3(...ep.rootP) });
const half = from(data.endpoints.half), hub = from(data.endpoints.__neutral), mid = waypoint(half, hub);
for (const s of A.SEG) assert.deepEqual(mid.local[s.name].toArray(),
  (s.name.startsWith('clav_') ? hub : half).local[s.name].toArray(), 'half waypoint changes only the clavicle locals');
assert.deepEqual(mid.rootQ.toArray(), half.rootQ.toArray()); assert.deepEqual(mid.rootP.toArray(), half.rootP.toArray());
console.log('PASS explicit bow/half coordinate repair:', alters.length + 2, 'selection rejections,', corruptions.length,
  'origin rejections;', Object.keys(selected.retainedTracks).length, 'tracks /', retainedFrames,
  'coordinate frames preserved exactly; ALL31endpoint rows exact; two mandatory releases regenerated; no inherited PASS.');
