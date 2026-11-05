// These independent cubes ensure the animation's filtered inspector cannot
// accidentally drop neighbouring shafts or a complete body-region category.
import assert from 'node:assert/strict';
import { gzipSync } from 'node:zlib';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { A, THREE } from './cycle_lib.mjs';
import { createTransitionCollision } from './transition_collision.mjs';
const vertices = [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]];
const faces = [0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,3,7,6,3,6,2,0,4,7,0,7,3,1,2,6,1,6,5];
function cubes(segments, offset) {
  const parts = [], chunks = [];
  for (const entry of segments) {
    const seg = typeof entry === 'string' ? entry : entry.seg;
    const latin = typeof entry === 'string' ? /^fore_/.test(seg) ? 'radius' : /^ulna_/.test(seg) ? 'ulna'
      : /^f1m_/.test(seg) ? 'os metacarpale I' : /^scap_/.test(seg) ? 'scapula' : undefined : entry.latin;
    const half = typeof entry === 'string' ? .01 : entry.half ?? .01;
    const center = typeof entry === 'string' ? [offset, 0, 0] : entry.center ?? [offset, 0, 0];
    const v = Uint16Array.from(vertices.flatMap(p => p.map(x => x < 0 ? 0 : 65535))), t = Uint16Array.from(faces);
    for (let i = v.length - 1; i >= 3; i--) v[i] = (v[i] - v[i - 3]) & 65535;
    for (let i = t.length - 1; i > 0; i--) t[i] = (t[i] - t[i - 1]) & 65535;
    chunks.push(Buffer.from(v.buffer), Buffer.from(t.buffer));
    parts.push({ seg, latin, name: typeof entry === 'string' ? 'independent closed cube' : entry.name ?? 'independent closed cube', n: 8, m: 12,
      lo: center.map(c => c - half), hi: center.map(c => c + half) });
  }
  return { parts, data: gzipSync(Buffer.concat(chunks)).toString('base64') };
}
const frames = segments => Object.fromEntries(segments.map(s => [s, { p: new THREE.Vector3(), q: new THREE.Quaternion() }]));
const partialTestGeometry = { partialGeometryForTests: true };
let cases = 0;
if (!process.argv.includes('--actual-only')) {
for (const segments of [['thigh_L','shank_L'], ['shank_L','foot_L'], ['thigh_L','pelvis']]) {
  const local = createTransitionCollision(A, THREE, cubes(segments, 0), partialTestGeometry), W = frames(segments);
  assert.equal(local.inspect(W).colliding, false, 'Only the bounded natural joint may be exempted: ' + segments);
  const remote = createTransitionCollision(A, THREE, cubes(segments, .12), partialTestGeometry);
  assert.equal(remote.inspect(W).colliding, true, 'A shaft 12 cm from its joint cannot be omitted: ' + segments);
  cases += 2;
}
// Foot/toe contact uses matched source-cap spheres, not the rig's one central
// toe hinge. Touching cap faces are local; oversized remote contact is retained.
const matchedMtp = [{ seg: 'foot_L', latin: 'os metatarsi I', name: 'I плюсневая кость' },
  { seg: 'toes_L', latin: 'phalanx proximalis', name: 'Проксимальная фаланга большого пальца стопы', center: [0, 0, .02] }];
const mtpWorld = frames(['foot_L', 'toes_L']);
assert.equal(createTransitionCollision(A, THREE, cubes(matchedMtp, 0), partialTestGeometry).inspect(mtpWorld).colliding, false);
assert.equal(createTransitionCollision(A, THREE, cubes(matchedMtp.map(p => ({ ...p, half: .03, center: [0, 0, 0] })), 0),
  partialTestGeometry).inspect(mtpWorld).colliding, true, 'Matched MTP does not excuse remote surface overlap');
assert.equal(createTransitionCollision(A, THREE, cubes(['foot_L', 'toes_L'], 0), partialTestGeometry).inspect(mtpWorld).colliding,
  true, 'An unmatched generic foot/toe pair gets no whole-pair articulation exemption');
cases += 3;
for (const segments of [['hum_L','thigh_R'], ['ulna_R','shank_R'], ['hand_L','pelvis'],
  ['fore_L','head'], ['foot_R','head'], ['shank_L','shank_R'], ['foot_L','foot_R'], ['f1b_L','hand_L']]) {
  const collider = createTransitionCollision(A, THREE, cubes(segments, .12), partialTestGeometry), W = frames(segments);
  assert.equal(collider.inspect(W).colliding, true, 'Required scope must include ' + segments);
  W[segments[1]].p.y = 1;
  assert.equal(collider.inspect(W).colliding, false, 'Separated real volumes must stay clear');
  cases += 2;
}
const th3 = { seg: 'thC', latin: 'vertebra thoracica Th3' };
const addedPairs = [
  ...['L', 'R'].flatMap(side => ['fore', 'ulna'].map(seg => [seg + '_' + side, 'f1m_' + side])),
  ['scap_L', th3], ['scap_R', th3],
];
for (const pair of addedPairs) for (const offset of [0, .12]) for (const parts of [pair, [...pair].reverse()]) {
  const segments = parts.map(p => typeof p === 'string' ? p : p.seg), W = frames(segments);
  const collider = createTransitionCollision(A, THREE, cubes(parts, offset), partialTestGeometry);
  assert.equal(collider.inspect(W).colliding, true, 'Semantic scope must retain intersecting parts at their origins and remote shafts: ' + segments);
  assert.equal(collider.inspectUpper(W).colliding, true, 'New categories belong to the upper-motion cache: ' + segments);
  assert.equal(collider.inspectLower(W).colliding, false, 'The lower cache cannot omit a moving scapula or forearm from its key');
  W[segments[1]].p.y = 1;
  assert.equal(collider.inspect(W).colliding, false, 'Separated semantic-category volumes remain clear');
  cases += 4;
}
for (const parts of [['scap_L', { seg: 'thC', latin: 'vertebra thoracica Th4' }],
  ['scap_R', { seg: 'thC', latin: 'costa III' }], ['fore_L', 'f1m_R']]) {
  const segments = parts.map(p => typeof p === 'string' ? p : p.seg);
  const collider = createTransitionCollision(A, THREE, cubes(parts, .12), partialTestGeometry);
  assert.equal(collider.inspect(frames(segments)).colliding, false, 'The added scope must not claim unverified scapular ribs/other levels or opposite-arm pairs');
  cases++;
}
assert.throws(() => createTransitionCollision(A, THREE, cubes(['fore_L', 'f1m_L'], .12)), /semantic inventory/,
  'Partial geometry is forbidden by default, even when one new pair exists');
cases++;
}

// These are sampled configurations from the pre-repair packet, not samples
// from whatever repaired transitions/data.json happens to contain today.
const fixture = JSON.parse(fs.readFileSync(new URL('./transitions/collision-regression.json', import.meta.url)));
const digest = filename => createHash('sha256').update(fs.readFileSync(new URL(filename, import.meta.url))).digest('hex');
assert.equal(fixture.schemaVersion, 1);
for (const [name, hash] of Object.entries(fixture.sourceHashes)) assert.equal(digest(name), hash, 'stale known-bad geometry: ' + name);
assert.match(fixture.origin.packetSha256, /^[0-9a-f]{64}$/);
const boneData = JSON.parse(fs.readFileSync(new URL('./skull/bones_bp3d.json', import.meta.url)));
const actual = createTransitionCollision(A, THREE, boneData);
assert.equal(actual.scope, 'transition-critical-bone-pairs-v2');
assert.deepEqual(actual.categories, ['all-arm-leg-pairs', 'limbs-axial-body-including-head', 'all-leg-pairs',
  'distal-fingers-own-palms', 'own-forearm-first-metacarpals', 'scapulae-third-thoracic-vertebra']);
assert.deepEqual(actual.semanticPairCounts, { ownForearmFirstMetacarpals: 4, scapulaTh3: 2 });
assert(Object.isFrozen(actual.categories) && Object.isFrozen(actual.semanticPairCounts));
let malformedInventories = 0;
for (const modify of [
  parts => { delete parts.find(p => p.latin === 'scapula').latin; },
  parts => { parts.find(p => p.latin === 'vertebra thoracica Th3').latin = 'vertebra thoracica Th4'; },
  parts => { parts.find(p => p.seg === 'f1m_R').latin = 'phalanx proximalis'; },
  parts => { parts.find(p => p.seg === 'ulna_L').latin = 'radius'; },
  parts => { parts.find(p => p.seg === 'f1m_L').seg = 'f1m_R'; },
  parts => { parts.push({ ...parts.find(p => p.latin === 'scapula') }); },
  parts => { parts.push({ ...parts.find(p => p.latin === 'vertebra thoracica Th3') }); },
]) {
  const damaged = { ...boneData, parts: boneData.parts.map(p => ({ ...p })) };
  modify(damaged.parts);
  assert.throws(() => createTransitionCollision(A, THREE, damaged), /semantic inventory/,
    'Missing, duplicated or mislabelled semantic roles must fail before collision checking');
  malformedInventories++;
}
const arm = s => /^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s);
const leg = s => /^(thigh|shank|foot|toes)_[LR]$/.test(s);
const axial = s => /^(pelvis|lumA|lumB|thA|thB|thC|neckA|neckB|head)$/.test(s);
const legacyFilter = (a, b) => arm(a.seg) && leg(b.seg) || leg(a.seg) && arm(b.seg)
  || ((arm(a.seg) || leg(a.seg)) && axial(b.seg) || axial(a.seg) && (arm(b.seg) || leg(b.seg)))
  || leg(a.seg) && leg(b.seg)
  || (/^hand_[LR]$/.test(a.seg) && /^f[1-5][bc]_[LR]$/.test(b.seg)
    || /^f[1-5][bc]_[LR]$/.test(a.seg) && /^hand_[LR]$/.test(b.seg)) && a.seg.slice(-1) === b.seg.slice(-1);
let actualCases = 0;
for (const sample of fixture.cases) {
  const c = sample.configuration;
  assert.deepEqual(Object.keys(c.local), A.SEG.map(s => s.name), sample.id + ': frozen full rig required');
  const W = A.computeWorld(Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...c.local[s.name])])),
    new THREE.Quaternion(...c.root), new THREE.Vector3(...c.rootP));
  const former = actual.collider.inspect(W, { pairFilter: legacyFilter, pairFilterLabel: 'former-four-category-regression', maxPairs: 1 });
  assert.equal(former.colliding, false, sample.id + ': fixture must demonstrate the former scope gap');
  const report = actual.inspect(W, { maxPairs: Infinity });
  assert.equal(report.colliding, true, sample.id + ': the supplementary guard must reject the frozen bad motion');
  assert.equal(actual.inspectUpper(W).colliding, true, sample.id + ': new pairs must invalidate the upper-motion cache');
  assert.equal(actual.inspectLower(W).colliding, false, sample.id + ': a scapular/arm motion must not be cached as lower-only');
  const relevant = report.pairs.filter(p => sample.expectedCategory === 'own-forearm-first-metacarpals'
    ? /^(fore|ulna)_[LR]$/.test(p.a) && /^f1m_[LR]$/.test(p.b)
    : (boneData.parts[p.aPart].latin === 'scapula' && boneData.parts[p.bPart].latin === 'vertebra thoracica Th3')
      || (boneData.parts[p.bPart].latin === 'scapula' && boneData.parts[p.aPart].latin === 'vertebra thoracica Th3'));
  assert(relevant.length >= sample.expectedMinimumNewPairCount, sample.id + ': all expected semantic bone pairs must be reported');
  assert(relevant.every(p => p.jointRegions.length === 0), sample.id + ': no new wrist or scapular joint exception');
  actualCases++;
}
let endpointCases = 0;
const endpoints = JSON.parse(fs.readFileSync(new URL('./transitions/data.json', import.meta.url))).endpoints;
assert.equal(Object.keys(endpoints).length, 31, 'All30 animation endpoints plus the shared neutral endpoint are required');
const newPairs = new Set();
for (let i = 0; i < boneData.parts.length; i++) for (let j = i + 1; j < boneData.parts.length; j++) {
  const a = boneData.parts[i], b = boneData.parts[j];
  const ownThumb = (a, b) => /^(fore|ulna)_[LR]$/.test(a.seg) && ['radius', 'ulna'].includes(a.latin)
    && /^f1m_[LR]$/.test(b.seg) && b.latin === 'os metacarpale I' && a.seg.slice(-1) === b.seg.slice(-1);
  const scapTh3 = (a, b) => /^scap_[LR]$/.test(a.seg) && a.latin === 'scapula' && b.latin === 'vertebra thoracica Th3';
  if (ownThumb(a, b) || ownThumb(b, a) || scapTh3(a, b) || scapTh3(b, a)) newPairs.add(i + ':' + j);
}
assert.equal(newPairs.size, 6, 'The endpoint test must exercise all four wrist and both scapula/Th3 source pairs');
const addedPairFilter = (a, b) => newPairs.has(Math.min(a.partIndex, b.partIndex) + ':' + Math.max(a.partIndex, b.partIndex));
for (const [id, c] of Object.entries(endpoints)) {
  const local = Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...c.local[s.name]).normalize()]));
  const W = A.computeWorld(local, new THREE.Quaternion(...c.root).normalize(), new THREE.Vector3(...c.rootP));
  const report = actual.collider.inspect(W, { pairFilter: addedPairFilter, pairFilterLabel: 'six-added-semantic-pairs-at-current-endpoints' });
  assert.equal(report.selectedBonePairs, 6);
  assert.equal(report.colliding, false, 'The six added pairs must be clear at every actual endpoint: ' + id);
  endpointCases++;
}
console.log('PASS transition collision scope:', cases, 'independent closed-cube cases;', malformedInventories,
  'malformed semantic inventories;', actualCases, 'frozen actual bad-motion cases;', endpointCases, 'actual endpoints; adjacent shafts retained.');
