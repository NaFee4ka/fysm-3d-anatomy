import fs from 'node:fs';
import assert from 'node:assert/strict';
import { A, THREE, rootPlaced } from './cycle_lib.mjs';
import { createPracticeContacts } from './practice/contacts.mjs';
import { createPracticeGroundFitter } from './practice/ground_fit.mjs';
import { createTransitionFloor } from './transition_floor.mjs';
const read = name => JSON.parse(fs.readFileSync(new URL(name, import.meta.url)));
const boneData = read('skull/bones_bp3d.json'), contacts = await createPracticeContacts(A, THREE, read('muscles/muscles_bp3d.json'), { boneData });
const floor = createTransitionFloor(A, THREE, contacts, boneData), reference = createPracticeGroundFitter({ A, THREE, contactFactory: contacts, boneData });
const main = read('poses.json'), source = { ...Object.fromEntries(A.POSES.map(p => [p.id, main[p.id]])), ...read('practice/poses.json').poses };
let maximumErrorM = 0, maximumTranslationErrorM = 0, count = 0;
for (const [id, p] of Object.entries(source)) {
  const c = { id, local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...p.local[s.name]).normalize()])),
    rootQ: new THREE.Quaternion(...p.root).normalize() };
  c.rootP = p.rootP ? new THREE.Vector3(...p.rootP) : rootPlaced(c.local, c.rootQ);
  const actual = floor.minimum(c, id), expected = reference.measure(c).minimum;
  maximumErrorM = Math.max(maximumErrorM, Math.abs(actual.minimumY - expected.heightM));
  assert(Math.abs(actual.minimumY - expected.heightM) < 2e-8, 'Fast floor must agree with the independent complete renderer audit: ' + id);
  c.rootP.y += .123;
  const translationError = Math.abs(floor.minimum(c, id).minimumY - actual.minimumY - .123);
  maximumTranslationErrorM = Math.max(maximumTranslationErrorM, translationError);
  // The real renderer stores bone uniforms in Float32. One micrometre bounds
  // their coordinate quantization throughout this body's two-metre scene.
  assert(translationError < 1e-6, 'Ground offset is covariant within real Float32 rendering precision: ' + id);
  count++;
}
contacts.dispose();
assert.equal(count, 30);
assert(floor.counts.muscles > 60000 && floor.counts.bones > 89000);
console.log('PASS transition floor:', count, 'complete poses; counts', floor.counts,
  'maximum reference error', maximumErrorM, 'm; Float32 translation error', maximumTranslationErrorM, 'm.');
