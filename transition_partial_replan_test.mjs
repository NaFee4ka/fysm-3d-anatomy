// Selection/provenance tests only: no cached collision proof or physical PASS
// state is accepted, and no producer/mesh/floor workload is executed here.
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
const select = extract('// PARTIAL_REPLAN_SELECTOR_BEGIN', '// PARTIAL_REPLAN_SELECTOR_END', { validateRawPlan: validate });
const archive = fs.readFileSync(new URL('transitions/plan.json.gz', import.meta.url));
const data = JSON.parse(gunzipSync(archive)), before = JSON.stringify(data);
const oldArchiveSha256 = createHash('sha256').update(archive).digest('hex');
const sourceHashes = { ...data.sourceHashes, 'muscle_surface_render.js': 'c'.repeat(64), 'practice/poses.json': 'd'.repeat(64),
  'transition_build.mjs': createHash('sha256').update(source).digest('hex') };
const endpoints = structuredClone(data.endpoints);
for (const ep of Object.values(endpoints)) { ep.rootP[1] += .001; ep.groundOffsetM += .001; }
const q = new THREE.Quaternion(...endpoints['centering-06'].local.hum_L);
q.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), .0007));
endpoints['centering-06'].local.hum_L = q.toArray();
endpoints['centering-06'].rootP[0] += .00002;
const context = () => ({ A, THREE, catalog: structuredClone(data.poses), endpoints: structuredClone(endpoints),
  sourceHashes: { ...sourceHashes }, sourceNames: [...names], oldArchiveSha256 });
const selected = select(data, context());
assert.equal(JSON.stringify(data), before);
assert.equal(JSON.stringify(selected.origin.affectedPoseIds), JSON.stringify(['centering-06']));
assert.equal(Object.keys(selected.retainedTracks).length, 47);
assert.equal(JSON.stringify(selected.origin.discardedTrackIds), JSON.stringify(['neighbour-centering-06-centering-07', 'release-centering-06']));
assert.equal(JSON.stringify(selected.requests.map(r => r.id)), JSON.stringify([
  'release-centering-06', 'neighbour-centering-05-centering-06', 'neighbour-centering-06-centering-07']));
assert(selected.preparePoseIds.includes('centering-05') && selected.preparePoseIds.includes('centering-07'));
assert(!selected.retainedFallbacks.some(f => [f.from, f.to].includes('centering-06')), 'old C5→C6 fallback is rescheduled despite missing old track');
let interiorFrames = 0;
for (const [id, track] of Object.entries(selected.retainedTracks)) {
  assert.equal(JSON.stringify(track), JSON.stringify(data.tracks[id]), 'retained coordinates/phases/times changed');
  interiorFrames += track.frames.length - 2;
}
assert.equal(selected.origin.endpointGroundingRebases.length, 31, 'Y-only rebases are recorded separately for all current rows');
assert.equal(selected.origin.allCanonicalEndpointRowsReplaced, true);
// Every incident track is discarded by endpoints, independent of its name.
const extra = structuredClone(data);
// This synthetic inventory is a new coordinate-only fixture. Its altered
// track list cannot retain an earlier author's inventory provenance.
delete extra.coordinateOrigin;
extra.tracks.unexpectedIncident = structuredClone(extra.tracks['release-centering-06']);
extra.tracks.unexpectedIncident.id = 'unexpectedIncident';
const expanded = select(extra, context());
assert(expanded.origin.discardedTrackIds.includes('unexpectedIncident'));
assert(!Object.hasOwn(expanded.retainedTracks, 'unexpectedIncident'));
assert.equal(Object.keys(expanded.retainedTracks).length, 47);
const negativeContexts = [
  c => { c.sourceHashes['anatomy3.js'] = '0'.repeat(64); },
  c => { c.sourceHashes['skull/bones_bp3d.json'] = '0'.repeat(64); },
  c => { c.endpoints.__neutral.local.fore_L = [Math.sin(.1), 0, 0, Math.cos(.1)]; },
  c => { c.endpoints.plank.rootP[0] += .001; },
  c => { c.endpoints.plank.sourceRoot[0] += .001; },
  c => { c.endpoints.plank.local.hum_L = [Math.sin(.01), 0, 0, Math.cos(.01)]; },
  c => { c.endpoints.plank.extraSemantic = true; },
  c => { c.endpoints.extra = structuredClone(c.endpoints.dog); },
];
for (const [i, alter] of negativeContexts.entries()) { const c = context(); alter(c); assert.throws(() => select(data, c), /Partial coordinate replanning rejected/, 'selection negative ' + i); }
const noRelease = structuredClone(data);
delete noRelease.coordinateOrigin;
delete noRelease.tracks['release-struna-02'];
const declared = { ...data.sourceHashes, 'transition_build.mjs': sourceHashes['transition_build.mjs'] };
assert.equal(validate(noRelease, { ...context(), endpoints: data.endpoints, sourceHashes: declared }), true,
  'fixture remains connected through neighbours but lacks a canonical mandatory release');
assert.throws(() => select(noRelease, context()), /missing canonical release/);
const movedOther = context(); movedOther.endpoints.plank.sourceRootP[2] += .001;
assert.throws(() => select(data, movedOther), /C6 scope/);
// Assemble a structurally valid, still UNVERIFIED partial candidate. Both
// incident neighbours deliberately use freshly declared graph fallbacks here;
// this tests the actual loader's new origin branch without claiming geometry.
const c = context(), candidate = structuredClone(data);
candidate.sourceHashes = { ...c.sourceHashes }; candidate.endpoints = c.endpoints;
candidate.tracks = structuredClone(selected.retainedTracks);
candidate.tracks['release-centering-06'] = structuredClone(data.tracks['release-centering-06']);
for (const t of Object.values(candidate.tracks)) {
  t.frames[0] = { ...c.endpoints[t.from], t: 0 };
  t.frames[t.frames.length - 1] = { ...c.endpoints[t.to], t: t.duration };
}
const rebuiltFallbacks = selected.requests.filter(r => r.kind === 'neighbour')
  .map(r => ({ from: r.from, to: r.to, reason: 'unit-only current graph fallback' }));
candidate.neighbourFallbacks = [...structuredClone(selected.retainedFallbacks), ...rebuiltFallbacks];
candidate.coordinateOrigin = { ...structuredClone(selected.origin), regeneratedTrackIds: ['release-centering-06'],
  regeneratedFallbacks: structuredClone(rebuiltFallbacks) };
assert.equal(validate(candidate, c), true, 'the actual partial-origin parser must accept a complete synthetic rebuilt candidate');
const corruptOrigins = [
  p => { p.coordinateOrigin.affectedPoseIds = ['plank']; },
  p => { p.coordinateOrigin.retainedTrackIds.push('release-centering-06'); },
  p => { p.coordinateOrigin.regeneratedRequests.pop(); },
  p => { p.coordinateOrigin.oldTrackCount++; },
  p => { p.coordinateOrigin.endpointCount = 30; },
  p => { p.coordinateOrigin.endpointGroundingRebases[0].currentRootY += .001; },
  p => { p.coordinateOrigin.endpointGroundingRebases.pop(); },
  p => { p.coordinateOrigin.endpointGroundingRebases = []; },
  p => { p.coordinateOrigin.replanningProducerSha256 = '0'.repeat(64); },
  p => { p.coordinateOrigin.allowedChangedFiles.push('anatomy3.js'); },
  p => { p.coordinateOrigin.regeneratedFallbacks.pop(); },
  p => { p.neighbourFallbacks = p.neighbourFallbacks.filter(f => f.from !== 'centering-05'); },
  p => { p.neighbourFallbacks.find(f => f.from === 'centering-05').reason = 'mismatched'; },
];
for (const [i, alter] of corruptOrigins.entries()) {
  const invalid = structuredClone(candidate); alter(invalid);
  assert.equal(validate(invalid, c), false, 'partial-origin corruption ' + i);
}
console.log('PASS partial coordinate selector:', negativeContexts.length + 2,
  'selection rejections plus', corruptOrigins.length, 'origin rejections;47tracks /', interiorFrames, 'interiorframes retainedexactly; allC6incidenttracks dropped and BOTHcurrentneighbourrequests scheduled. No inherited PASS.');
