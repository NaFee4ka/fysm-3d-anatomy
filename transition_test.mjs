import fs from 'node:fs';
import assert from 'node:assert/strict';
import { A, THREE } from './cycle_lib.mjs';
import { createTransitions } from './transition_runtime.mjs';
import './transition_routing_test.mjs';

const data = JSON.parse(fs.readFileSync(new URL('transitions/data.json', import.meta.url)));
const before = JSON.stringify(data), runtime = createTransitions(THREE, A, data);
assert.equal(runtime.valid, true, 'The complete published graph must be usable');
assert.equal(runtime.poses.length, 30);
const conf = c => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...c.local[s.name]).normalize()])),
  rootQ: new THREE.Quaternion(...c.root).normalize(), rootP: new THREE.Vector3(...c.rootP) });
function equalConf(a, b, epsilon = 2e-8) {
  assert(a && b);
  assert(a.rootP.distanceTo(b.rootP) < epsilon, `root position mismatch ${a.rootP.distanceTo(b.rootP)}`);
  for (const [x, y] of [[a.rootQ, b.rootQ], ...A.SEG.map(s => [a.local[s.name], b.local[s.name]])])
    assert(Math.min(Math.hypot(...x.toArray().map((v, i) => v - y.toArray()[i])),
      Math.hypot(...x.toArray().map((v, i) => v + y.toArray()[i]))) < epsilon, 'rotation mismatch');
}
let directed = 0, joins = 0, reversedStates = 0;
assert.equal(data.jointMotion?.verified, true);
for (const side of ['L', 'R']) {
  assert(data.jointMotion.minimumFlexDeg[side] >= -data.jointMotion.modelExtensionLimitDeg - 1e-5);
  assert(data.jointMotion.maximumFlexDeg[side] <= data.jointMotion.modelFlexionLimitDeg + 1e-5);
  const joint = A.JOINTS.find(j => j.type === 'elbow' && j.side === side);
  const hub = conf(data.endpoints.__neutral), flex = A.analyzeJoint(joint, hub.local, {}).comps.find(c => c.key === 'flex').signed;
  assert(flex > 10 && flex < 20, 'The neutral hub bends the elbow normally rather than hyperextending it');
}
for (const a of runtime.poses) for (const b of runtime.poses) {
  const route = runtime.route(a.id, b.id);
  assert(route && route.duration >= 0);
  equalConf(runtime.sample(route, 0), conf(data.endpoints[a.id]));
  equalConf(runtime.sample(route, route.duration), conf(data.endpoints[b.id]));
  equalConf(runtime.sample(route, -1), conf(data.endpoints[a.id]));
  equalConf(runtime.sample(route, route.duration + 1), conf(data.endpoints[b.id]));
  assert.equal(runtime.sample(route, NaN), null);
  if (a.id === b.id) { assert.equal(route.edges.length, 0); continue; }
  directed++;
  const reverse = { from: b.id, to: a.id, duration: route.duration,
    edges: route.edges.toReversed().map(e => ({ trackId: e.trackId, reverse: !e.reverse })) };
  for (const fraction of [0, .13, .5, .79, 1]) {
    equalConf(runtime.sample(route, route.duration * fraction), runtime.sample(reverse, route.duration * (1 - fraction))); reversedStates++;
    const backward = runtime.route(b.id, a.id);
    equalConf(runtime.sample(route, route.duration * fraction), runtime.sample(backward, backward.duration * (1 - fraction)));
  }
  let elapsed = 0;
  for (let i = 0; i < route.edges.length - 1; i++) {
    const edge = route.edges[i], track = data.tracks[edge.trackId], next = route.edges[i + 1], nextTrack = data.tracks[next.trackId];
    elapsed += track.duration;
    const last = runtime.sampleTrack(track.id, edge.reverse ? 0 : track.duration), first = runtime.sampleTrack(nextTrack.id, next.reverse ? nextTrack.duration : 0);
    equalConf(last, first);
    equalConf(runtime.sample(route, elapsed), last);
    assert(runtime.sample(route, elapsed - 1e-8).rootP.distanceTo(runtime.sample(route, elapsed + 1e-8).rootP) < 1e-6);
    joins++;
  }
}
assert.equal(directed, 870);
assert.equal(runtime.route('__proto__', 'dog'), null);
assert.equal(runtime.route('dog', 'missing'), null);
assert.equal(runtime.sampleTrack('missing', 0), null);
const first = Object.keys(data.tracks)[0];
assert.equal(runtime.sampleTrack(first, Infinity), null);
const freshA = runtime.sampleTrack(first, 0), freshB = runtime.sampleTrack(first, 0);
freshA.rootP.y = 100; freshA.local.hum_L.identity();
equalConf(freshB, conf(data.tracks[first].frames[0]));
assert.equal(JSON.stringify(data), before, 'Sampling and routing must not mutate saved geometry');

// Timing is illustrative. A checked neighbouring flow should still be used
// when an indirect route has a shorter clock. This fixture changes times
// only in memory; it is never a geometric certificate or a published packet.
const publicIds = new Set(data.poses.map(p => p.id));
const neighbour = Object.values(data.tracks).find(t => publicIds.has(t.from) && publicIds.has(t.to));
assert(neighbour, 'the graph should retain at least one prepared neighbouring flow');
const directFixture = structuredClone(data);
for (const [id, t] of Object.entries(directFixture.tracks))
  if (t.from === neighbour.from && t.to === neighbour.to || t.from === neighbour.to && t.to === neighbour.from)
    delete directFixture.tracks[id];
const indirectRuntime = createTransitions(THREE, A, directFixture);
assert(indirectRuntime.valid, 'mandatory release edges retain connectivity without this optional direct flow');
const indirectRoute = indirectRuntime.route(neighbour.from, neighbour.to);
assert(indirectRoute.edges.length >= 2);
const slowDirect = structuredClone(neighbour), factor = Math.max(100, indirectRoute.duration * 10 / slowDirect.duration);
slowDirect.duration *= factor;
for (const frame of slowDirect.frames) frame.t *= factor;
for (const phase of slowDirect.phases) { phase.t0 *= factor; phase.t1 *= factor; }
directFixture.tracks[slowDirect.id] = slowDirect;
const directRuntime = createTransitions(THREE, A, directFixture);
assert(directRuntime.valid);
for (const [from, to] of [[neighbour.from, neighbour.to], [neighbour.to, neighbour.from]]) {
  const path = directRuntime.route(from, to);
  assert.equal(path.edges.length, 1, 'an available direct flow takes precedence over an indirect clock');
  assert.equal(path.edges[0].trackId, neighbour.id);
  assert(path.duration > indirectRoute.duration);
  equalConf(directRuntime.sample(path, 0), conf(data.endpoints[from]));
  equalConf(directRuntime.sample(path, path.duration), conf(data.endpoints[to]));
}
const alternative = structuredClone(slowDirect); alternative.id = '__unit_direct_choice';
alternative.duration /= 2;
for (const frame of alternative.frames) frame.t /= 2;
for (const phase of alternative.phases) { phase.t0 /= 2; phase.t1 /= 2; }
directFixture.tracks[alternative.id] = alternative;
const multipleDirectRuntime = createTransitions(THREE, A, directFixture);
assert(multipleDirectRuntime.valid);
assert.equal(multipleDirectRuntime.route(neighbour.from, neighbour.to).edges[0].trackId, alternative.id,
  'multiple direct flows use their smallest duration');
assert.equal(multipleDirectRuntime.route(neighbour.to, neighbour.from).edges[0].trackId, alternative.id);

const corruptions = [
  d => { d.verified = false; }, d => { d.checks.floor = 'true'; }, d => { d.checks.oppositeArms = false; },
  d => { d.checks.criticalRegions = false; },
  d => { d.checks.elbowMotion = false; },
  d => { d.poses[0] = null; }, d => { d.poses[0].name = null; }, d => { d.poses[0].kind = 'unknown'; },
  d => { d.poses[6].algorithmId = '__proto__'; }, d => { d.poses[6].stepIndex = 8; },
  d => { d.poses[1].id = d.poses[0].id; }, d => { d.endpoints = null; }, d => { d.tracks = []; },
  d => { d.sourceHashes = {}; }, d => { d.sourceHashes['anatomy3.js'] = 'stale'; },
  d => { d.segNames[0] = 'unexpected'; }, d => { d.endpoints.dog.t = NaN; },
  d => { d.endpoints.dog.rootP[0] = Infinity; }, d => { d.endpoints.dog.local.hum_L = [0, 0, 0, 0]; },
  d => { d.endpoints.dog.rootP[0] = 1e308; },
  d => { d.tracks[first].id = 'duplicate'; }, d => { d.tracks[first].duration = NaN; },
  d => { d.tracks[first].frames[0].t = -.1; }, d => { d.tracks[first].frames[1].t = 0; },
  d => { d.tracks[first].frames[0].local.hum_L = null; },
  d => { d.tracks[first].frames[0].rootP[0] += .01; },
  d => { d.tracks[first].frames[0].floorLiftM = NaN; }, d => { d.tracks[first].frames[0].floorLiftM = -.001; },
  d => { d.tracks[first].frames[0].floorLiftM = .051; },
  d => { d.tracks[first].phases = []; }, d => { d.tracks[first].phases[0] = null; },
  d => { d.tracks[first].phases[0].t0 = .001; }, d => { d.tracks[first].phases[0].t1 = Infinity; },
  d => { d.tracks[first].phases[0].label = ''; }, d => { d.tracks[first].phases[0].reverseLabel = null; },
];
for (const [i, mutate] of corruptions.entries()) {
  const d = structuredClone(data); mutate(d);
  assert.doesNotThrow(() => { assert.equal(createTransitions(THREE, A, d).valid, false, 'invalid case ' + i); });
}
for (const d of [null, undefined, [], {}]) assert.equal(createTransitions(THREE, A, d).valid, false);
console.log('PASS transition graph:', directed, 'directed routes,', joins, 'continuous joins,', reversedStates,
  'reversed samples,', corruptions.length + 4, 'malformed/fail-closed packets; inputs immutable.');
