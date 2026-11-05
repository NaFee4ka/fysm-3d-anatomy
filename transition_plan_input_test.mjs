// Exercise the actual pure validator/fitter declarations from the single
// producer module without executing its expensive build entry point. Mocks
// below concern IO/floor measurements only; production code is not mirrored.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { createHash } from 'node:crypto';
import { gunzipSync } from 'node:zlib';
import { A, THREE } from './cycle_lib.mjs';
import { createTransitions } from './transition_runtime.mjs';
const source = fs.readFileSync(new URL('transition_build.mjs', import.meta.url), 'utf8');
const originalInputs = vm.runInNewContext(source.match(/const originalInputs = (\[[\s\S]*?\]);/)[1]);
assert.equal(originalInputs.length, 30);
const body = source.split('// CANDIDATE_VALIDATOR_BEGIN\n')[1].split('// CANDIDATE_VALIDATOR_END')[0].trim();
const validate = vm.runInNewContext('(' + body + ')', { createTransitions });
const data = JSON.parse(gunzipSync(fs.readFileSync(new URL('transitions/plan.json.gz', import.meta.url))));
const hash = name => createHash('sha256').update(fs.readFileSync(new URL(name, import.meta.url))).digest('hex');
// Schema fixtures use their declared source context. The CLI supplies fresh real
// current hashes; a renderer/context change must reject until explicit migration.
const sourceHashes = { ...data.sourceHashes, 'transition_build.mjs': hash('transition_build.mjs'),
  'transitions/plan.json.gz': hash('transitions/plan.json.gz') };
const expected = { A, THREE, catalog: structuredClone(data.poses), endpoints: structuredClone(data.endpoints),
  sourceHashes, sourceNames: [...originalInputs] };
const snapshot = JSON.stringify(data);
assert.equal(validate(data, expected), true, 'complete unverified coordinates with matching declared context are usable input');
assert.equal(JSON.stringify(data), snapshot, 'input validation must not mutate coordinate data');
const changedProducer = structuredClone(data); changedProducer.sourceHashes['transition_build.mjs'] = 'a'.repeat(64);
if (changedProducer.coordinateOrigin) {
  const field = changedProducer.coordinateOrigin.kind === 'surface-context-root-y-rebase'
    ? 'migrationProducerSha256' : 'replanningProducerSha256';
  changedProducer.coordinateOrigin[field] = changedProducer.sourceHashes['transition_build.mjs'];
}
assert.equal(validate(changedProducer, expected), true, 'the old producer hash is informational, never an imported PASS proof');
const first = Object.keys(data.tracks)[0], endpoint = data.tracks[first].from;
const cases = [
  p => { p.verified = true; }, p => { p.floorVerified = true; }, p => { p.complete = false; },
  p => { p.sourcesFrozen = false; }, p => { p.kind = 'kinematic-demonstration'; },
  p => { p.planningFailures.push({ id: 'dog' }); },
  p => { delete p.sourceHashes['anatomy3.js']; }, p => { p.sourceHashes.unexpected = 'a'.repeat(64); },
  p => { p.sourceHashes['transition_runtime.mjs'] = '0'.repeat(64); },
  p => { p.sourceHashes['transition_build.mjs'] = 'invalid'; },
  p => { p.segNames.reverse(); }, p => { p.poses[0].name += '!'; },
  p => { p.endpoints[endpoint].sourceRootP[0] += .01; },
  p => { p.endpoints[endpoint].rootP[1] += 67e-9; },
  p => { p.tracks[first].frames[0].rootP[1] += 67e-9; },
  p => { p.tracks[first].id = 'mismatch'; },
  p => { p.tracks[first].frames[1].local.hum_L = [0, 0, 0, 0]; },
  p => { p.tracks[first].frames[1].t = 0; }, p => { p.tracks[first].phases[0].t0 = .01; },
  p => { p.tracks[first].phases[0].reverseLabel = null; },
  p => { p.checks = { floor: true }; }, p => { p.jointMotion = { verified: true }; },
  p => { p.tracks[first].groundModelId = 'detali-08'; },
  p => { p.tracks[first].frames[1].floorLiftM = .001; },
  p => { p.tracks[first].frames[1].local.hum_L = [Math.sin(.4), 0, 0, Math.cos(.4)]; },
  p => { p.tracks = []; }, p => { p.neighbourFallbacks = null; },
  p => { p.tracks[first].frames[1].local.extra = [0, 0, 0, 1]; },
  p => { delete p.tracks['release-dog']; },
];
for (const [i, alter] of cases.entries()) {
  const invalid = structuredClone(data); alter(invalid);
  assert.equal(validate(invalid, expected), false, 'candidate negative case ' + i);
}
// Reproduce the real failure scale: a floor fitter can request a 67 nm root
// change due to Float32 skinning. Only interiors may be fitted independently;
// incident tracks must retain exactly the shared endpoint record/metadata.
const originalTrack = data.tracks['release-plank'];
const tracks = { first: structuredClone(originalTrack), second: structuredClone(originalTrack) };
tracks.second.id = 'second';
let measured = 0;
const fitterCode = source.match(/function fitSavedTrackFrames\(\) \{[\s\S]*?\n\}/)[0];
const fit = vm.runInNewContext('(' + fitterCode + ')', { tracks, endpoints: expected.endpoints,
  CLEARANCE: .00075, number: n => Math.round(n * 1e9) / 1e9, progress: () => {},
  fromData: f => ({ rootP: new THREE.Vector3(...f.rootP) }),
  floor: { minimum: () => { measured++; return { minimumY: .00075 - 67e-9 }; } } });
const originalInteriorY = tracks.first.frames[1].rootP[1];
fit();
assert.equal(measured, 2 * (originalTrack.frames.length - 2), 'floor fitting must not independently re-place endpoints');
assert(Math.abs(tracks.first.frames[1].rootP[1] - originalInteriorY - 67e-9) < 1.1e-9);
for (const t of Object.values(tracks)) {
  assert.equal(JSON.stringify(t.frames[0]), JSON.stringify({ ...expected.endpoints[t.from], t: 0 }));
  assert.equal(JSON.stringify(t.frames.at(-1)), JSON.stringify({ ...expected.endpoints[t.to], t: t.duration }));
}
console.log('PASS unverified plan input:', cases.length,
  'negative provenance/schema cases; changed producer informational; exact shared endpoints despite 67 nm fitter residual. No geometry proof imported.');
