// The displayed ligament route must keep its actual corners, not cut a
// wrapping obstacle with chords between uniform arc-length samples.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { resampleRoutedPath, createConnectiveSurfaces } from './connective_surface_render.js';
import { createAnatomy } from './anatomy3.js';

const V = (...p) => new THREE.Vector3(...p);
const out = n => Array.from({ length: n }, () => V());
const length = points => points.slice(1).reduce((sum, p, i) => sum + p.distanceTo(points[i]), 0);
function check(points, n) {
  const snapshot = points.map(p => p.toArray()), sampled = out(n);
  const total = resampleRoutedPath(points, sampled);
  assert.deepEqual(points.map(p => p.toArray()), snapshot, 'input path mutated');
  assert.deepEqual(sampled[0].toArray(), points[0].toArray());
  assert.deepEqual(sampled.at(-1).toArray(), points.at(-1).toArray());
  let previous = 0;
  for (const p of points) {
    const index = sampled.findIndex((q, i) => i >= previous && q.distanceToSquared(p) === 0);
    assert(index >= previous, 'an original routed corner was omitted or reordered'); previous = index;
  }
  assert(Math.abs(length(sampled) - total) < 1e-10 * Math.max(1, total));
  assert(Math.abs(length(points) - total) < 1e-12 * Math.max(1, total));
  return sampled;
}

// An independently specified narrow rectangular obstacle. The source route
// travels along its two sides and above its top; skipping a 10-mm corner
// joins neighbouring segments by a chord through its interior.
const route = [V(-1, 0, 0), V(-1, .01, 0), V(1, .01, 0), V(1, 0, 0)];
const samples = check(route, 25);
const inside = p => p.x > -1 + 1e-9 && p.x < 1 - 1e-9 && p.y >= -.01 && p.y < .009;
for (let i = 1; i < samples.length; i++) for (let k = 0; k <= 16; k++)
  assert(!inside(samples[i - 1].clone().lerp(samples[i], k / 16)), 'rendered chord cuts the obstacle');
const oldTotal = length(route), firstOldDistance = oldTotal / 24;
const firstOldSample = V(-1 + firstOldDistance - .01, .01, 0);
assert(inside(route[0].clone().lerp(firstOldSample, .5)), 'regression no longer demonstrates the old chord error');
check([V(2, 3, 4), V(2, 3, 4), V(2, 3, 4)], 25);
assert.throws(() => resampleRoutedPath(route, out(3)), /more routed segments/);
assert.throws(() => resampleRoutedPath([V(NaN, 0, 0)], out(25)), /Invalid/);
let seed = 12345;
const random = () => ((seed = (1664525 * seed + 1013904223) >>> 0) / 2 ** 32);
for (let k = 0; k < 250; k++) {
  const points = Array.from({ length: 2 + k % 18 }, () => V(random() * 4 - 2, random() * 3, random() - .5));
  check(points, 25);
}

const A = createAnatomy(THREE);
const render = createConnectiveSurfaces(A, THREE, {
  capsuleRims: JSON.parse(fs.readFileSync('joints/capsule-rims.json')).capsuleRims,
});
const instances = render.parts.flatMap(part => [...part.insts, ...(part.displayInsts || []), ...(part.displayColumnInsts || [])]);
const poses = JSON.parse(fs.readFileSync('poses.json'));
const practice = JSON.parse(fs.readFileSync('practice/poses.json')).poses;
const cycle = JSON.parse(fs.readFileSync('cycle_keys/cycle.json')).frames;
const frames = [...Object.values(poses).filter(p => p?.local), ...Object.values(practice), ...cycle];
let testedPaths = 0, maxPoints = 0;
for (const frame of frames) {
  const local = Object.fromEntries(Object.entries(frame.local).map(([name, q]) => [name, new THREE.Quaternion(...q).normalize()]));
  const rootQ = new THREE.Quaternion(...frame.root).normalize();
  const W = frame.rootP ? A.computeWorld(local, rootQ, V(...frame.rootP)) : A.placeW(local, rootQ);
  for (const instance of instances) {
    const points = A.strandPath(W, instance);
    check(points, 25); testedPaths++; maxPoints = Math.max(maxPoints, points.length);
  }
}
render.dispose();
console.log(JSON.stringify({ passed: true, syntheticRoutes: 252, oldChordRejected: true,
  states: frames.length, testedPaths, maxOriginalPoints: maxPoints, displayRows: 25 }));
