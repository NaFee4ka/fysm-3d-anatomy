// Static reconstructions of the 24 photographs on pages 4–6 of the supplied PDF.
// This builds geometry, not transitions, contact forces or muscle recruitment.
// Run from the repository root: node practice_build.mjs
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { A, THREE, rootPlaced } from './cycle_lib.mjs';
import { buildStruna } from './practice/struna.mjs';
import { buildDetali } from './practice/detali.mjs';
import { buildCentering } from './practice/centering.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const read = name => fs.readFileSync(path.join(ROOT, name));
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const manifest = JSON.parse(read('practice/algorithms.json'));
const ids = manifest.algorithms.flatMap(algorithm => algorithm.steps.map(step => step.id));
const built = { ...buildStruna(), ...buildDetali(), ...buildCentering() };
const boneData = JSON.parse(read('skull/bones_bp3d.json'));
const armCollision = createBodyCollision(A, THREE, boneData);
const armFootCollision = createArmFootCollision(A, THREE, boneData);
if (manifest.algorithms.length !== 3 || manifest.algorithms.some(a => a.steps.length !== 8)
    || ids.length !== new Set(ids).size || Object.keys(built).length !== ids.length
    || ids.some(id => !built[id])) throw new Error('Practice must contain exactly the 3 × 8 source poses');
const round = n => Math.round(n * 1e9) / 1e9;
function quaternion(q, label) {
  if (!q || !q.toArray().every(Number.isFinite) || q.lengthSq() < 1e-12)
    throw new Error('Invalid practice quaternion: ' + label);
  return q.clone().normalize().toArray().map(round);
}
const poses = {};
for (const id of ids) {
  const p = built[id];
  const local = Object.fromEntries(A.SEG.map(segment => [segment.name,
    quaternion(p.local[segment.name], id + ':' + segment.name)]));
  const root = quaternion(p.rootQ, id + ':root');
  const decodedLocal = Object.fromEntries(Object.entries(local).map(([name, q]) =>
    [name, new THREE.Quaternion(...q).normalize()]));
  const rootQ = new THREE.Quaternion(...root).normalize();
  const rootP = (p.rootP || rootPlaced(decodedLocal, rootQ)).toArray().map(round);
  if (rootP.length !== 3 || !rootP.every(Number.isFinite)) throw new Error('Invalid root position: ' + id);
  const W = A.computeWorld(decodedLocal, rootQ, new THREE.Vector3(...rootP));
  const collision = armCollision.inspect(W, { maxPairs: 1 });
  if (collision.colliding) throw new Error(`Practice pose ${id} intersects opposite-arm bones: ${JSON.stringify(collision.pairs)}`);
  const armFoot = armFootCollision.inspect(W, { maxPairs: 1 });
  if (armFoot.colliding) throw new Error(`Practice pose ${id} intersects arm/foot bones: ${JSON.stringify(armFoot.pairs)}`);
  const markers = A.LANDMARKS.map(marker => ({
    region: marker.region, side: marker.side, y: A.pt(W, marker.seg, marker.pc).y,
  }));
  const minY = Math.min(...markers.map(marker => marker.y));
  if (Math.abs(minY) > 1e-5) throw new Error(`Practice pose ${id} has floor offset ${minY} m`);
  const view = p.view || [-1, .3, .65];
  if (view.length !== 3 || !view.every(Number.isFinite) || Math.hypot(...view) < .1)
    throw new Error('Invalid practice camera: ' + id);
  if (!Array.isArray(p.notes) || !p.notes.every(note => typeof note === 'string'))
    throw new Error('Missing reconstruction notes: ' + id);
  poses[id] = { local, root, rootP, view, notes: p.notes,
    geometry: { minMarkerHeightM: round(minY),
      contacts: [...new Set(markers.filter(marker => marker.y < .004)
        .map(marker => marker.region + ':' + marker.side))] } };
  console.log(id, 'floor', (minY * 1000).toFixed(3) + ' mm;', poses[id].geometry.contacts.join(', '));
}
const inputs = ['anatomy3.js', 'cycle_lib.mjs', 'seg5_util.mjs', 'poses_build.mjs', 'package-lock.json', 'practice_build.mjs',
  'practice/algorithms.json', 'practice/struna.mjs', 'practice/detali.mjs', 'practice/centering.mjs',
  'body_collision.mjs', 'collision_geometry.mjs', 'skull/bones_bp3d.json', 'practice/arm_foot_collision.mjs'];
const output = { schemaVersion: 1, sourceSha256: manifest.source.sha256,
  kind: 'static-photo-reconstructions', sourceHashes: Object.fromEntries(inputs.map(name => [name, sha(read(name))])),
  poses };
const geometryFile = JSON.stringify(output);
fs.writeFileSync(path.join(ROOT, 'practice/poses.json'), geometryFile);
fs.writeFileSync(path.join(ROOT, 'practice/check.json'), JSON.stringify({ schemaVersion: 1,
  verified: true, scope: 'opposite-arm-bones', poseCount: ids.length, boneParts: 60,
  armFootCheck: { verified: true, scope: 'arm-foot-bones', poseCount: ids.length, pairGroups: 4 },
  posesSha256: sha(geometryFile), sourceHashes: output.sourceHashes }));
console.log('Built', ids.length, 'static practice poses; no force estimates or transition trajectories.');
