// Geometry/force-route contracts for the six poses and sampled asymmetric round.
// Run from the repository root: node muscle_paths_test.mjs
// Optional regression against a prior anatomy module:
//   node muscle_paths_test.mjs --baseline /path/to/previous/anatomy3.js
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { pathToFileURL } from 'node:url';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';

const A = createAnatomy(THREE), PH = makePhysics(A);
const instances = A.strandInstances();
const poseData = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const frames = [{ id: 'stand', W: PH.W_STAND }];
for (const pose of A.POSES) {
  const c = A.poseFromData(poseData[pose.id]);
  frames.push({ id: pose.id, W: A.placeW(c.local, c.rootQ) });
}
const cycle = JSON.parse(fs.readFileSync('cycle_keys/cycle.json', 'utf8'));
for (let i = 0; i < cycle.frames.length; i += 17) {
  const f = cycle.frames[i], c = A.poseFromData(f);
  frames.push({ id: `cycle:${i}`, W: A.computeWorld(c.local, c.rootQ, A.V(...f.rootP)) });
}
const bi = process.argv.indexOf('--baseline');
const baseline = bi < 0 ? null : (await import(pathToFileURL(process.argv[bi + 1]).href)).createAnatomy(THREE);
const priorInstances = baseline ? new Map(baseline.strandInstances().map(m => [m.id, m])) : null;
const moving = PH.subtree('fore_L'), rest = new Set(A.SEG.map(s => s.name).filter(s => !moving.has(s)));
let compared = 0;
for (const { id, W } of frames) for (const m of instances) {
  const plain = A.strandPath(W, m), owned = PH.ownedPath(W, m);
  assert.equal(plain.length, owned.length, `${id}/${m.id}: trace point count`);
  assert.equal(owned[0].seg, m.pts[0].seg, `${id}/${m.id}: origin owner`);
  assert.equal(owned.at(-1).seg, m.pts.at(-1).seg, `${id}/${m.id}: insertion owner`);
  assert.ok(plain[0].distanceTo(A.pt(W, m.pts[0].seg, m.pts[0].p)) < 1e-12);
  assert.ok(plain.at(-1).distanceTo(A.pt(W, m.pts.at(-1).seg, m.pts.at(-1).p)) < 1e-12);
  for (let k = 0; k < plain.length; k++) {
    assert.ok(A.S[owned[k].seg], `${id}/${m.id}: valid bone owner`);
    assert.ok(Number.isFinite(plain[k].lengthSq()), `${id}/${m.id}: finite point`);
    assert.ok(plain[k].distanceTo(owned[k].p) < 1e-12, `${id}/${m.id}: length and force trace diverged`);
  }
  assert.ok(PH.pullOn(owned, moving, 1).add(PH.pullOn(owned, rest, 1)).length() < 1e-12,
    `${id}/${m.id}: action/reaction across elbow boundary`);
  if (baseline) {
    const before = baseline.strandPath(W, priorInstances.get(m.id));
    assert.equal(before.length, plain.length, `${id}/${m.id}: baseline point count`);
    for (let k = 0; k < plain.length; k++) assert.ok(plain[k].distanceTo(before[k]) < 1e-12,
      `${id}/${m.id}: baseline geometry changed`);
  }
  compared++;
}

// An intersecting synthetic path exercises generated wrap-contact ownership,
// rather than only checking endpoints already assigned to rig segments.
const wrap = { seg: 'pelvis', c: A.V(.09, .93, 0), r: A.V(.04, .04, .04), depth: 2 };
const strand = {
  pts: [{ seg: 'pelvis', p: A.V(.01, .93, .015) }, { seg: 'lumA', p: A.V(.17, .93, .015) }],
  wraps: [wrap],
};
const traced = A.ownedStrandPath(PH.W_STAND, strand);
assert.ok(traced.length > 2, 'synthetic path must wrap');
for (const { p, seg } of traced.slice(1, -1)) {
  assert.equal(seg, wrap.seg, 'contact force must be carried by the wrapped bone');
  const q = p.clone().sub(PH.W_STAND[seg].p).add(A.S[seg].J).sub(wrap.c).divide(wrap.r);
  assert.ok(Math.abs(q.length() - 1) < 1e-10, 'contact lies on wrap surface');
}

// Global placement must not alter muscle lengths or which bone carries a force.
const rotation = new THREE.Quaternion().setFromAxisAngle(A.V(1, 2, 3).normalize(), .73);
const shift = A.V(.34, -.21, .57);
const rotatedW = Object.fromEntries(Object.entries(PH.W_STAND).map(([s, w]) => [s, {
  p: w.p.clone().applyQuaternion(rotation).add(shift), q: rotation.clone().multiply(w.q),
}]));
for (const m of instances) {
  const before = A.ownedStrandPath(PH.W_STAND, m), after = A.ownedStrandPath(rotatedW, m);
  assert.equal(before.length, after.length, `${m.id}: rigid placement changes trace`);
  for (let k = 0; k < before.length; k++) {
    assert.equal(before[k].seg, after[k].seg);
    assert.ok(before[k].p.clone().applyQuaternion(rotation).add(shift).distanceTo(after[k].p) < 1e-10,
      `${m.id}: path is not covariant under global placement`);
  }
}
console.log(`PASS: ${instances.length} strands; ${frames.length} pose/round frames; ${compared} force/length traces; wrap contacts; rigid placement${baseline ? '; prior geometry unchanged' : ''}.`);
