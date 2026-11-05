// Synthetic routing fixtures only: these timings and edges are not motion
// geometry, and are never written into the application's verified dataset.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { A, THREE } from './cycle_lib.mjs';
import { createTransitions } from './transition_runtime.mjs';
const saved = JSON.parse(fs.readFileSync('poses.json'));
const practice = JSON.parse(fs.readFileSync('practice/poses.json')).poses;
const algorithms = JSON.parse(fs.readFileSync('practice/algorithms.json')).algorithms;
const poses = A.POSES.map(p => ({ id: p.id, name: p.name || p.id, kind: 'pose' }));
for (const a of algorithms) for (const [stepIndex, p] of a.steps.entries())
  poses.push({ id: p.id, name: p.name, kind: 'practice', algorithmId: a.id, stepIndex });
const endpoints = Object.fromEntries(poses.map(p => {
  const c = saved[p.id] || practice[p.id];
  return [p.id, { t: 0, local: structuredClone(c.local), root: [...c.root], rootP: [...(c.rootP || [0, 0, 0])] }];
}));
endpoints.__neutral = structuredClone(endpoints.dog);
endpoints.__x = structuredClone(endpoints.half);
endpoints.__y = structuredClone(endpoints.bow);
const edge = (id, from, to, duration) => ({ id, from, to, duration,
  frames: [{ ...structuredClone(endpoints[from]), t: 0 }, { ...structuredClone(endpoints[to]), t: duration }],
  phases: [{ t0: 0, t1: duration, label: id + ' forward', reverseLabel: id + ' reverse' }] });
// Insertion order deliberately makes independently searched reverse routes
// choose Y while the forward route chooses X when equal-cost ties are broken
// from different ends.
const tracks = {
  ax: edge('ax', 'dog', '__x', 1), by: edge('by', 'plank', '__y', 1),
  xb: edge('xb', '__x', 'plank', 1), ya: edge('ya', '__y', 'dog', 1),
};
for (const p of poses) tracks['hub-' + p.id] = edge('hub-' + p.id, p.id, '__neutral', 100);
const packet = { schemaVersion: 1, verified: true, kind: 'kinematic-demonstration',
  segNames: A.SEG.map(s => s.name), poses, endpoints, tracks,
  sourceHashes: { 'routing-fixture-only': '0'.repeat(64) },
  checks: { oppositeArms: true, armsFeet: true, floor: true, criticalRegions: true, elbowMotion: true } };
function same(a, b) {
  assert(a && b); assert(a.rootP.distanceTo(b.rootP) < 2e-8);
  for (const [x, y] of [[a.rootQ, b.rootQ], ...A.SEG.map(s => [a.local[s.name], b.local[s.name]])])
    assert(Math.min(Math.hypot(...x.toArray().map((v, i) => v - y.toArray()[i])),
      Math.hypot(...x.toArray().map((v, i) => v + y.toArray()[i]))) < 2e-8);
}
let runtime = createTransitions(THREE, A, packet); assert(runtime.valid);
const forward = runtime.route('dog', 'plank'), backward = runtime.route('plank', 'dog');
assert.deepEqual(forward.edges.map(e => e.trackId), ['ax', 'xb']);
assert.deepEqual(backward.edges.map(e => e.trackId), ['xb', 'ax']);
assert.deepEqual(backward.phases.map(p => p.label), ['xb reverse', 'ax reverse']);
assert.deepEqual(backward.phases.map(p => [p.t0, p.t1]), [[0, 1], [1, 2]]);
let reversedStates = 0;
for (let i = 0; i <= 100; i++) {
  same(runtime.sample(forward, forward.duration * i / 100), runtime.sample(backward, backward.duration * (1 - i / 100)));
  reversedStates++;
}
assert(A.SEG.some(s => new THREE.Quaternion(...endpoints.__x.local[s.name])
  .angleTo(new THREE.Quaternion(...endpoints.__y.local[s.name])) > .1), 'X/Y must represent different configurations');
tracks.slow = edge('slow', 'dog', 'plank', 10);
runtime = createTransitions(THREE, A, packet); assert(runtime.valid);
assert.equal(runtime.route('dog', 'plank').edges[0].trackId, 'slow');
assert.equal(runtime.route('dog', 'plank').edges.length, 1, 'direct flow takes precedence over the shorter clock through X');
tracks.zdirect = edge('zdirect', 'plank', 'dog', 5);
runtime = createTransitions(THREE, A, packet); assert(runtime.valid);
assert.deepEqual(runtime.route('dog', 'plank').edges, [{ trackId: 'zdirect', reverse: true }]);
assert.deepEqual(runtime.route('plank', 'dog').edges, [{ trackId: 'zdirect', reverse: false }]);
tracks.adirect = edge('adirect', 'dog', 'plank', 5);
runtime = createTransitions(THREE, A, packet); assert(runtime.valid);
assert.equal(runtime.route('dog', 'plank').edges[0].trackId, 'adirect', 'equal direct clocks use a stable track ID');
assert.equal(runtime.route('plank', 'dog').edges[0].trackId, 'adirect');
console.log('PASS routing-only synthetic equal-cost topology:', reversedStates,
  'actual reverse-route samples; direct priority, minimum duration and stable ties. No geometry certificate.');
