// Anatomical fascia must follow its routed geometry under every whole-body pose.
// These checks validate the renderer, not patient-specific material mechanics.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createFascia } from './fascia3.js';
import { createFasciaRender, sampleFasciaRoute, sampleFasciaSheet } from './fascia_render.js';
import { createTransitions } from './transition_runtime.mjs';

const V = (...x) => new THREE.Vector3(...x);
const length = points => points.slice(1).reduce((sum, p, i) => sum + p.distanceTo(points[i]), 0);
const route = [V(-1, 0, 0), V(-1, .01, 0), V(1, .01, 0), V(1, 0, 0)];
const snapshot = route.map(p => p.toArray());
const sampled = sampleFasciaRoute(route, 25);
assert.deepEqual(route.map(p => p.toArray()), snapshot, 'source route mutated');
assert.equal(sampled.length, 25);
assert(Math.abs(length(sampled) - 2.02) < 1e-12);
for (const p of route) assert(sampled.some(q => q.equals(p)), 'routed corner omitted');
for (let i = 1; i < sampled.length; i++) for (let j = 0; j <= 30; j++) {
  const p = sampled[i - 1].clone().lerp(sampled[i], j / 30);
  assert(p.x <= -1 + 1e-12 || p.x >= 1 - 1e-12 || p.y >= .01 - 1e-12, 'route cuts rectangular obstacle');
}
// The old smoothed curve leaves the independently specified route and omits
// its two short corners. Here it bows above the obstacle by more than 5 mm.
const old = new THREE.CatmullRomCurve3(route, false, 'centripetal').getSpacedPoints(24);
assert(old.some(p => p.y > .015), 'fixture no longer demonstrates old smoothing error');
assert(!old.some(p => p.equals(route[1])), 'fixture no longer omits the short original corner');
assert.equal(sampleFasciaRoute(route, 2).length, route.length, 'budget must adapt to preserve corners');
assert.throws(() => sampleFasciaRoute([V(NaN, 0, 0)]), /Invalid/);
assert.throws(() => sampleFasciaRoute([], 12), /Invalid/);
assert.deepEqual(sampleFasciaRoute([V(1, 2, 3), V(1, 2, 3)], 4).map(p => p.toArray()),
  Array.from({ length: 4 }, () => [1, 2, 3]));

const columns = [route, [V(-.9, -.2, .1), V(-.9, .03, .1), V(.9, .03, .1), V(.9, -.2, .1)],
  [V(-.7, -.1, .2), V(-.7, .05, .2), V(.7, .05, .2), V(.7, -.1, .2)]];
const sheet = sampleFasciaSheet(columns);
for (let j = 0; j < columns.length; j++) {
  const column = sheet.across.indexOf(j / (columns.length - 1));
  assert(column >= 0, 'sheet omits original column');
  for (const corner of columns[j]) assert(sheet.points.some(row => row[column].distanceTo(corner) < 1e-12), 'sheet cuts routed corner');
}
assert.equal(sheet.across[0], 0); assert.equal(sheet.across.at(-1), 1);
assert.equal(sheet.rows[0], 0); assert.equal(sheet.rows.at(-1), 1);

const A = createAnatomy(THREE);
const F = createFascia(A, THREE, JSON.parse(fs.readFileSync('books/fascia_model.json')));
const renderer = createFasciaRender(F, A, THREE);
renderer.setLayer('lines', true);
const poses = JSON.parse(fs.readFileSync('poses.json'));
const practice = JSON.parse(fs.readFileSync('practice/poses.json')).poses;
const cycle = JSON.parse(fs.readFileSync('cycle_keys/cycle.json')).frames;
const inputs = [...Object.entries(poses).filter(([, p]) => p?.local), ...Object.entries(practice),
  ...cycle.filter((_, i) => i % 13 === 0).map((p, i) => ['cycle/' + i * 13, p])];
const localFrom = frame => Object.fromEntries(Object.entries(frame.local).map(([name, q]) => [name, new THREE.Quaternion(...q).normalize()]));
const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(.73, -1.17, .39)), shift = V(.71, -.32, .98);
let states = 0, transitionStates = 0, checkedPaths = 0, checkedCorners = 0, checkedVertices = 0, maxRigidError = 0, maxNormalError = 0;
const regions = new Set();
function checkWorld(W, label) {
  renderer.update(W, null, { light: false });
  for (const band of renderer.bands) {
    const path = A.strandPath(W, band.inst);
    let start = 0;
    for (const p of path) {
      const at = band.center.findIndex((q, i) => i >= start && q.distanceTo(p) < 1e-12);
      assert(at >= start, label + ': missing/out-of-order fascia band corner'); start = at; checkedCorners++;
    }
    assert(Math.abs(length(band.center) - length(path)) < 1e-10 * Math.max(1, length(path)), label + ': fascia route length changed');
    checkedPaths++; regions.add(band.reference.seg);
  }
  for (const surface of renderer.sheets) for (let j = 0; j < surface.cols.length; j++) {
    const column = surface.sampled.across.findIndex(x => Math.abs(x - j / (surface.cols.length - 1)) < 1e-12);
    assert(column >= 0);
    for (const corner of A.strandPath(W, surface.cols[j])) {
      assert(surface.sampled.points.some(row => row[column].distanceTo(corner) < 1e-10), label + ': sheet skips original route corner');
      checkedCorners++;
    }
  }
  for (const part of [...renderer.bands, ...renderer.sheets, ...renderer.lines]) {
    for (const name of ['position', 'normal']) assert([...part.geo.attributes[name].array].every(Number.isFinite), label + ': nonfinite fascia ' + name);
  }
}
function checkRigid(W, label) {
  const before = [...renderer.bands, ...renderer.sheets, ...renderer.lines].map(p => ({
    positions: p.geo.attributes.position.array.slice(), normals: p.geo.attributes.normal.array.slice(),
  }));
  const transformed = Object.fromEntries(Object.entries(W).map(([seg, pose]) => [seg, {
    q: rotation.clone().multiply(pose.q), p: pose.p.clone().applyQuaternion(rotation).add(shift),
  }]));
  checkWorld(transformed, label + '/rigid');
  for (let part = 0; part < before.length; part++) {
    const after = [...renderer.bands, ...renderer.sheets, ...renderer.lines][part].geo.attributes;
    assert.equal(after.position.array.length, before[part].positions.length, label + ': rigid rotation changes fascia topology');
    for (let at = 0; at < before[part].positions.length; at += 3) {
      const expected = V(...before[part].positions.slice(at, at + 3)).applyQuaternion(rotation).add(shift);
      const actual = V(...after.position.array.slice(at, at + 3));
      const error = actual.distanceTo(expected); maxRigidError = Math.max(maxRigidError, error);
      assert(error < 6e-7, `${label}: fascia rigid covariance ${error}m`);
      const expectedNormal = V(...before[part].normals.slice(at, at + 3)).applyQuaternion(rotation);
      const actualNormal = V(...after.normal.array.slice(at, at + 3));
      if (expectedNormal.lengthSq() > .5 && actualNormal.lengthSq() > .5) {
        const normalError = actualNormal.distanceTo(expectedNormal); maxNormalError = Math.max(maxNormalError, normalError);
        assert(normalError < .002, `${label}: fascia normal rigid covariance ${normalError} part ${[...renderer.bands, ...renderer.sheets, ...renderer.lines][part].mesh.userData.id} #${part} vertex ${at / 3}`);
      }
      checkedVertices++;
    }
  }
}
for (const [label, frame] of inputs) {
  const local = localFrom(frame), root = new THREE.Quaternion(...frame.root).normalize();
  const W = frame.rootP ? A.computeWorld(local, root, V(...frame.rootP)) : A.placeW(local, root);
  checkWorld(W, label);
  checkRigid(W, label);
  states++;
}
// Explicit full-motion coverage uses the very same normalized sampler and
// floor-envelope translation as the page, rather than rebuilding an approximate
// midpoint in this test. Shared graph tracks cover all 870 routed selections.
if (process.argv.includes('--require-transitions') || process.argv.includes('--transitions')) {
  const data = JSON.parse(fs.readFileSync('transitions/data.json'));
  const runtime = createTransitions(THREE, A, data);
  assert(runtime.valid, 'release transition packet must be runtime-valid');
  for (const track of Object.values(data.tracks)) {
    for (let i = 0; i < track.frames.length; i++) {
      const times = [track.frames[i].t];
      if (i + 1 < track.frames.length) times.push((track.frames[i].t + track.frames[i + 1].t) / 2);
      for (const time of times) {
        const conf = runtime.sampleTrack(track.id, time);
        const W = A.computeWorld(conf.local, conf.rootQ, conf.rootP), label = track.id + '/' + time;
        checkWorld(W, label);
        if (i === Math.floor(track.frames.length / 2) && time === track.frames[i].t) checkRigid(W, label);
        transitionStates++;
      }
    }
  }
  assert(transitionStates > 0, 'new transition states required');
}
assert([...regions].some(x => x === 'head'), 'head fascia not covered');
assert([...regions].some(x => /^neck/.test(x)), 'neck fascia not covered');
assert([...regions].some(x => /^th/.test(x)), 'trunk fascia not covered');
assert([...regions].some(x => /^hand/.test(x)), 'hand fascia not covered');
assert([...regions].some(x => /^foot/.test(x)), 'foot fascia not covered');
renderer.dispose();
console.log(JSON.stringify({ passed: true, states, transitionStates, checkedPaths, checkedCorners, checkedVertices, maxRigidError, maxNormalError,
  routedBands: true, routedSheetColumns: true, oldSmoothingRegression: true, renderedMyersLines: renderer.lines.length, coveredAnchorSegments: [...regions].sort() }));
