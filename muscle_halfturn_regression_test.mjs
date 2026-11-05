// Actual 121k-vertex atlas regression for the measured 28 mm / 6.158 us
// hemisphere jump. This checks display geometry, not physical tissue laws.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { A, THREE } from './cycle_lib.mjs';
import { createMuscleSurfaces } from './muscle_surface_render.js';

const read = path => JSON.parse(fs.readFileSync(new URL(path, import.meta.url)));
const regression = read('./muscles/halfturn_regression.json');
const surfaces = await createMuscleSurfaces(A, THREE, read('./muscles/muscles_bp3d.json'));
const V = () => new THREE.Vector3();
const configs = [0, .25, .5, .75, 1].map(fraction => {
  const local = Object.fromEntries(A.SEG.map(({ name }) => [name,
    new THREE.Quaternion(...regression.a.local[name]).normalize().slerp(
      new THREE.Quaternion(...regression.b.local[name]).normalize(), fraction)]));
  const root = new THREE.Quaternion(...regression.a.root).normalize().slerp(
    new THREE.Quaternion(...regression.b.root).normalize(), fraction);
  const rootP = V().fromArray(regression.a.rootP).lerp(V().fromArray(regression.b.rootP), fraction); rootP.y = 0;
  return A.computeWorld(local, root, rootP);
});
const positions = [], vertex = V(), edgeA = V(), edgeB = V();
let vertexCount = 0, deepVertexCount = 0, outerVertexCount = 0, triangleCount = 0, maxStepM = 0, maxEndpointDeltaM = 0, maxEdgeStepM = 0;
let maxSeamGapM = 0, minTriangleAreaRatio = Infinity, witnessDeltaM = null;
try {
  for (const mesh of surfaces.pickables) {
    vertexCount += mesh.geometry.attributes.position.count;
    if (A.UNITS[mesh.userData.unit]?.deep) deepVertexCount += mesh.geometry.attributes.position.count;
    else outerVertexCount += mesh.geometry.attributes.position.count;
    triangleCount += mesh.geometry.index.count / 3;
  }
  const seams = surfaces.pickables.map(mesh => {
    const groups = new Map(), p = mesh.geometry.attributes.position;
    for (let k = 0; k < p.count; k++) {
      const key = [p.getX(k), p.getY(k), p.getZ(k)].join(',');
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key).push(k);
    }
    return [...groups.values()].filter(group => group.length > 1);
  });
  for (const W of configs) {
    surfaces.update(W);
    const frame = [];
    for (let mi = 0; mi < surfaces.pickables.length; mi++) {
      const mesh = surfaces.pickables[mi], rest = mesh.geometry.attributes.position, indices = mesh.geometry.index;
      const actual = new Float64Array(rest.count * 3);
      for (let k = 0; k < rest.count; k++) mesh.getVertexPosition(k, vertex).toArray(actual, k * 3);
      frame.push(actual);
      for (const group of seams[mi]) {
        const first = V().fromArray(actual, group[0] * 3);
        for (const k of group.slice(1)) maxSeamGapM = Math.max(maxSeamGapM, first.distanceTo(vertex.fromArray(actual, k * 3)));
      }
      for (let k = 0; k < indices.count; k += 3) {
        const ids = [indices.getX(k), indices.getX(k + 1), indices.getX(k + 2)];
        const a = V().fromArray(actual, ids[0] * 3), b = V().fromArray(actual, ids[1] * 3), c = V().fromArray(actual, ids[2] * 3);
        const ar = V().fromBufferAttribute(rest, ids[0]), br = V().fromBufferAttribute(rest, ids[1]), cr = V().fromBufferAttribute(rest, ids[2]);
        const restArea2 = br.sub(ar).cross(cr.sub(ar)).length();
        if (restArea2 > 1e-14) minTriangleAreaRatio = Math.min(minTriangleAreaRatio, b.clone().sub(a).cross(c.clone().sub(a)).length() / restArea2);
        if (positions.length) {
          const previous = positions.at(-1)[mi];
          for (let j = 0; j < 3; j++) {
            const x = ids[j], y = ids[(j + 1) % 3];
            const currentLength = edgeA.fromArray(actual, x * 3).distanceTo(edgeB.fromArray(actual, y * 3));
            const priorLength = edgeA.fromArray(previous, x * 3).distanceTo(edgeB.fromArray(previous, y * 3));
            maxEdgeStepM = Math.max(maxEdgeStepM, Math.abs(currentLength - priorLength));
          }
        }
      }
    }
    if (positions.length) for (let mi = 0; mi < frame.length; mi++) {
      for (let k = 0; k < frame[mi].length; k += 3) maxStepM = Math.max(maxStepM,
        vertex.fromArray(frame[mi], k).distanceTo(edgeA.fromArray(positions.at(-1)[mi], k)));
    }
    positions.push(frame);
  }
  for (let mi = 0; mi < positions[0].length; mi++) {
    for (let k = 0; k < positions[0][mi].length; k += 3) maxEndpointDeltaM = Math.max(maxEndpointDeltaM,
      vertex.fromArray(positions[0][mi], k).distanceTo(edgeA.fromArray(positions.at(-1)[mi], k)));
    const mesh = surfaces.pickables[mi];
    if (mesh.userData.unit === 'gracilis' && mesh.userData.side === 'R') witnessDeltaM =
      vertex.fromArray(positions[0][mi], 206 * 3).distanceTo(edgeA.fromArray(positions.at(-1)[mi], 206 * 3));
  }
  // 0.1 mm across the fixed 6.158 us regression interval catches the measured
  // 28 mm pop by > 2 orders of magnitude; it is a numerical regression bound.
  assert(maxEndpointDeltaM < .0001, 'actual atlas still jumps across the fixed half-turn interval');
  assert(maxStepM < .0001 && maxEdgeStepM < .0001, 'a surface edge still pops in the half-turn interval');
  assert(maxSeamGapM < 1e-8, 'duplicate atlas seam vertices separate');
  assert(witnessDeltaM !== null && witnessDeltaM < .0001, 'gracilis_R witness still jumps');
  console.log(JSON.stringify({ passed: true, frames: positions.length, meshCount: surfaces.pickables.length,
    vertexCount, deepVertexCount, outerVertexCount,
    sampledVertices: vertexCount * positions.length, triangleCount,
    intervalSeconds: regression.b.t - regression.a.t, maxStepM, maxEndpointDeltaM,
    maxEdgeStepM, maxSeamGapM, minTriangleAreaRatio, witnessDeltaM,
    sourceSha256: createHash('sha256').update(fs.readFileSync(new URL('./muscle_surface_render.js', import.meta.url))).digest('hex') }));
} finally { surfaces.dispose(); }
