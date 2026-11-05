// Execute the actual pure migration declaration without running its authoring
// entry point or importing any old verification state. These are schema and
// coordinate-provenance fixtures, not physical/mesh certificates.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { A, THREE } from './cycle_lib.mjs';
import { createTransitions } from './transition_runtime.mjs';
const source = fs.readFileSync(new URL('transition_build.mjs', import.meta.url), 'utf8');
const names = [...vm.runInNewContext(source.match(/const originalInputs = (\[[\s\S]*?\]);/)[1])];
const readFunction = (begin, end, globals) => vm.runInNewContext('(' + source.split(begin + '\n')[1].split(end)[0].trim() + ')', globals);
const validate = readFunction('// CANDIDATE_VALIDATOR_BEGIN', '// CANDIDATE_VALIDATOR_END', { createTransitions });
const migrate = readFunction('// COORDINATE_MIGRATION_BEGIN', '// COORDINATE_MIGRATION_END', { validateRawPlan: validate });
const archive = fs.readFileSync(new URL('transitions/plan.json.gz', import.meta.url));
const data = JSON.parse(gunzipSync(archive)), snapshot = JSON.stringify(data);
const oldArchiveSha256 = createHash('sha256').update(archive).digest('hex');
const producer = createHash('sha256').update(source).digest('hex');
const currentHashes = { ...data.sourceHashes, 'muscle_surface_render.js': 'c'.repeat(64),
  'practice/poses.json': 'd'.repeat(64), 'transition_build.mjs': producer };
const currentEndpoints = structuredClone(data.endpoints);
for (const ep of Object.values(currentEndpoints)) {
  ep.rootP[1] += .012345; ep.sourceRootP[1] += .007345; ep.groundOffsetM += .005;
}
const context = () => ({ A, THREE, catalog: structuredClone(data.poses), endpoints: structuredClone(currentEndpoints),
  sourceHashes: { ...currentHashes }, sourceNames: [...names], oldArchiveSha256 });
assert.equal(validate(data, context()), false, 'a changed renderer/source context must not be silently rebound by the loader');
const result = migrate(data, context()), migrated = result.plan;
assert.equal(validate(migrated, context()), true);
assert.equal(JSON.stringify(data), snapshot, 'migration must not mutate its original input');
assert.equal(migrated.verified, false); assert.equal(migrated.floorVerified, false);
assert(!Object.hasOwn(migrated, 'checks') && !Object.hasOwn(migrated, 'jointMotion'));
assert.equal(migrated.coordinateOrigin.oldArchiveSha256, oldArchiveSha256);
assert.equal(JSON.stringify(migrated.coordinateOrigin.oldSourceHashes), JSON.stringify(data.sourceHashes));
assert.equal(migrated.coordinateOrigin.migrationProducerSha256, producer);
assert.equal(migrated.coordinateOrigin.endpointCount, 31);
let preserved = 0;
for (const [id, t] of Object.entries(migrated.tracks)) {
  const old = data.tracks[id];
  assert.equal(JSON.stringify(t.frames.slice(1, -1)), JSON.stringify(old.frames.slice(1, -1)), 'interior coordinates/times changed: ' + id);
  assert.equal(JSON.stringify(t.phases), JSON.stringify(old.phases)); assert.equal(t.duration, old.duration);
  assert.equal(JSON.stringify(t.frames[0]), JSON.stringify({ ...currentEndpoints[t.from], t: 0 }));
  assert.equal(JSON.stringify(t.frames.at(-1)), JSON.stringify({ ...currentEndpoints[t.to], t: t.duration }));
  preserved += t.frames.length - 2;
}
assert.equal(preserved, result.report.preservedInteriorFrames);
const invalidContexts = [
  c => { c.sourceHashes['anatomy3.js'] = '0'.repeat(64); },
  c => { c.sourceHashes['skull/bones_bp3d.json'] = '0'.repeat(64); },
  c => { c.oldArchiveSha256 = 'invalid'; },
  c => { c.endpoints.dog.local.hum_L = [Math.sin(.01), 0, 0, Math.cos(.01)]; },
  c => { c.endpoints.dog.root = [Math.sin(.01), 0, 0, Math.cos(.01)]; },
  c => { c.endpoints.dog.rootP[0] += .001; },
  c => { c.endpoints.dog.sourceRootP[2] += .001; },
  c => { c.endpoints.dog.sourceLocal.thigh_L = [Math.sin(.01), 0, 0, Math.cos(.01)]; },
  c => { c.endpoints.dog.extraSemanticField = 'changed'; },
  c => { c.catalog[0].name += '!'; },
  c => { c.endpoints.extra = structuredClone(c.endpoints.dog); },
];
for (const [i, alter] of invalidContexts.entries()) {
  const c = context(); alter(c);
  assert.throws(() => migrate(data, c), /Coordinate migration rejected/, 'migration negative case ' + i);
}
const badOrigin = structuredClone(migrated);
badOrigin.coordinateOrigin.allowedChangedFiles.push('anatomy3.js');
assert.equal(validate(badOrigin, context()), false, 'origin cannot authorize another anatomical change');
const antipodal = context();
antipodal.endpoints.dog.root = antipodal.endpoints.dog.root.map(x => -x);
assert.equal(validate(migrate(data, antipodal).plan, antipodal), true, 'the same root rotation may use its antipodal representation');
console.log('PASS explicit coordinate migration:', invalidContexts.length + 1,
  'rejection cases;', preserved, 'interior frames/timelines preserved; rootY/grounding-only changes, exact current endpoints and original provenance retained. No PASS state imported.');
