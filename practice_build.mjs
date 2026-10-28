// Static reconstructions of the 24 photographs on pages 4–6 of the supplied PDF.
// Geometry/ground fitting only. The separate practice_physics_build.mjs solves
// forces for these saved poses; no transition trajectory is invented here.
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
import { createPracticeContacts } from './practice/contacts.mjs';
import { createPracticeGroundFitter } from './practice/ground_fit.mjs';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const read = name => fs.readFileSync(path.join(ROOT, name));
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const inputs = ['anatomy3.js', 'cycle_lib.mjs', 'seg5_util.mjs', 'poses_build.mjs', 'package-lock.json', 'practice_build.mjs',
  'practice/algorithms.json', 'practice/struna.mjs', 'practice/detali.mjs', 'practice/centering.mjs',
  'practice/kinematics_helpers.mjs', 'practice/ground_fit.mjs', 'practice/contacts.mjs',
  'muscle_surface_render.js', 'muscles/muscles_bp3d.json',
  'body_collision.mjs', 'collision_geometry.mjs', 'skull/bones_bp3d.json', 'practice/arm_foot_collision.mjs'];
const sourceHashes = () => Object.fromEntries(inputs.map(name => [name, sha(read(name))]));
const initialSourceHashes = sourceHashes();
const manifest = JSON.parse(read('practice/algorithms.json'));
const ids = manifest.algorithms.flatMap(algorithm => algorithm.steps.map(step => step.id));
const built = { ...buildStruna(), ...buildDetali(), ...buildCentering() };
const boneData = JSON.parse(read('skull/bones_bp3d.json'));
const armCollision = createBodyCollision(A, THREE, boneData);
const armFootCollision = createArmFootCollision(A, THREE, boneData);
const contactFactory = await createPracticeContacts(A, THREE, JSON.parse(read('muscles/muscles_bp3d.json')), { boneData });
const groundFitter = createPracticeGroundFitter({ A, THREE, contactFactory, boneData });
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
  const fitted = groundFitter.fitForBuild({ id, ...built[id] }, { contactAudit: true });
  const p = fitted.conf;
  const local = Object.fromEntries(A.SEG.map(segment => [segment.name,
    quaternion(p.local[segment.name], id + ':' + segment.name)]));
  const root = quaternion(p.rootQ, id + ':root');
  const decodedLocal = Object.fromEntries(Object.entries(local).map(([name, q]) =>
    [name, new THREE.Quaternion(...q).normalize()]));
  const rootQ = new THREE.Quaternion(...root).normalize();
  const rootP = (p.rootP || rootPlaced(decodedLocal, rootQ)).toArray().map(round);
  if (rootP.length !== 3 || !rootP.every(Number.isFinite)) throw new Error('Invalid root position: ' + id);
  const W = A.computeWorld(decodedLocal, rootQ, new THREE.Vector3(...rootP));
  // Verify the rounded saved pose, not only the full-precision fitting result.
  const actualGround = groundFitter.measure({ id, local: decodedLocal, rootQ, rootP: new THREE.Vector3(...rootP) });
  if (actualGround.minimum.heightM < -2e-6 || actualGround.primary.some(group => !group.present))
    throw new Error(`Practice pose ${id} has invalid actual floor/support geometry: ${JSON.stringify(actualGround.primary)}`);
  const contactModel = contactFactory.build(id, W, { audit: true });
  if (!contactModel.diagnostics.surfaceGeometryValid || !contactModel.diagnostics.contactModelComplete
      || contactModel.diagnostics.unclusteredSupport.margin === null || contactModel.diagnostics.unclusteredSupport.margin < -1e-6)
    throw new Error(`Practice pose ${id} lacks valid anatomical supports: ${JSON.stringify(contactModel.diagnostics)}`);
  const collision = armCollision.inspect(W, { maxPairs: 1 });
  if (collision.colliding) throw new Error(`Practice pose ${id} intersects opposite-arm bones: ${JSON.stringify(collision.pairs)}`);
  const armFoot = armFootCollision.inspect(W, { maxPairs: 1 });
  if (armFoot.colliding) throw new Error(`Practice pose ${id} intersects arm/foot bones: ${JSON.stringify(armFoot.pairs)}`);
  const markers = A.LANDMARKS.map(marker => ({
    region: marker.region, side: marker.side, y: A.pt(W, marker.seg, marker.pc).y,
  }));
  const minY = Math.min(...markers.map(marker => marker.y));
  const view = p.view || [-1, .3, .65];
  if (view.length !== 3 || !view.every(Number.isFinite) || Math.hypot(...view) < .1)
    throw new Error('Invalid practice camera: ' + id);
  if (!Array.isArray(p.notes) || !p.notes.every(note => typeof note === 'string'))
    throw new Error('Missing reconstruction notes: ' + id);
  const notes = [...p.notes];
  if (fitted.groundDiagnostics.localKinematicsChanged)
    notes.push('Положение конечностей дополнительно уточнено по геометрии опоры. Фотография не задаёт точных трёхмерных координат; это расчётная реконструкция.');
  poses[id] = { local, root, rootP, view, notes,
    geometry: { minMarkerHeightM: round(minY),
      ground: { ...fitted.groundDiagnostics, after: actualGround, nonpenetrating: true,
        primarySupportsComplete: true, savedGeometryVerified: true,
        support: contactModel.diagnostics.unclusteredSupport },
      contacts: [...new Set(markers.filter(marker => marker.y < .004)
        .map(marker => marker.region + ':' + marker.side))] } };
  console.log(id, 'floor', (minY * 1000).toFixed(3) + ' mm;', poses[id].geometry.contacts.join(', '));
}
contactFactory.dispose();
if (JSON.stringify(initialSourceHashes) !== JSON.stringify(sourceHashes()))
  throw new Error('Practice geometry sources changed while fitting. Rebuild from frozen sources.');
const output = { schemaVersion: 1, sourceSha256: manifest.source.sha256,
  kind: 'static-photo-reconstructions', sourceHashes: initialSourceHashes,
  poses };
const geometryFile = JSON.stringify(output);
fs.writeFileSync(path.join(ROOT, 'practice/poses.json'), geometryFile);
fs.writeFileSync(path.join(ROOT, 'practice/check.json'), JSON.stringify({ schemaVersion: 1,
  verified: true, scope: 'opposite-arm-bones', poseCount: ids.length, boneParts: 60,
  armFootCheck: { verified: true, scope: 'arm-foot-bones', poseCount: ids.length, pairGroups: 4 },
  groundCheck: { verified: true, scope: 'displayed-outer-muscles-all-bones-and-skin-proxies', poseCount: ids.length },
  posesSha256: sha(geometryFile), sourceHashes: output.sourceHashes }));
console.log('Built', ids.length, 'static practice poses. Run practice_physics_build.mjs for their separate force estimates.');
