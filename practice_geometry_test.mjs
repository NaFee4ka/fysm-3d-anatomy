// Independent source/order and static-pose geometry checks for FYSM practice.
// This checks floor markers and the actual opposite-arm bone meshes; it is not
// a whole-body soft-tissue collision or physiological validation.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { gunzipSync, gzipSync } from 'node:zlib';
import { spawnSync } from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { createBodyCollision } from './body_collision.mjs';
import { createMuscleSurfaces } from './muscle_surface_render.js';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { createPracticeContacts } from './practice/contacts.mjs';
import { createPracticeGroundFitter } from './practice/ground_fit.mjs';
import { spineJointTorque } from './physics_core.mjs';
import { SPINE_PASSIVE } from './physics_params.mjs';

const read = name => JSON.parse(fs.readFileSync(new URL(name, import.meta.url)));
const SOURCE_SHA = 'a23ca63b746b87eca51e813ada626840fcd0fbfbaec1731b0fa6c37a19dee041';
// Transcribed independently from the captions on the three original PDF pages.
const EXPECTED = [
  ['struna', 'Струна', 4, [
    ['Вытяжение с пальцевым замком вверх', 'Урдхва Баддхангулиасана'],
    ['Вытяжение с пальцевым замком назад', 'Баддха Хастасана'],
    ['Стул с запястным замком', 'Баддха Хаста Уткатасана'],
    ['Стул самолётик', 'Ардха Уткатасана 3'],
    ['Паук', 'Бхуджапидасана'],
    ['Сумо скрутка', 'Паривритта Стамбхасана'],
    ['Нижний упор с одной ногой', 'Трианга Дандасана 2'],
    ['Лодка на ягодицах', 'Ардха Навасана'],
  ]],
  ['detali', 'Детали', 5, [
    ['Уголок', 'Ардха Брахмачариасана'],
    ['Поза посоха', 'Дандасана'],
    ['Лодка на спине', 'Ардха Навасана 2'],
    ['Полумост с одной ногой', 'Эка Пада Питхасана'],
    ['Низкая лодка', 'Ардха Навасана 3'],
    ['Широкий угол лежа на спине с коленями на полу', 'Урдхва Мукха Супта Конасана'],
    ['Вытяжение ног лёжа', 'Супта Пашчимотанасана'],
    ['Русалка', 'Паршва Урдхва Падасана'],
  ]],
  ['centering', 'Центрирование', 6, [
    ['Наклон стоя с закрытым замком', 'Баддха Хаста Уттанасана'],
    ['Нижний упор', 'Чатуранга Дандасана'],
    ['Воин 1', 'Вирабхадрасана 1'],
    ['Боковой воин', 'Вирабхадрасана 2'],
    ['Стойка на предплечьях с одной ногой', 'Эка Пада Пинча Маюрасана'],
    ['Полубакасана с одной ногой', 'Ардха Эка Падабакасана'],
    ['Скрутка на предплечье с колен', 'Вакра Гоасана'],
    ['Прогиб с колен', 'Анахатасана'],
  ]],
];
const manifest = read('./practice/algorithms.json');
assert.equal(manifest.schemaVersion, 1);
assert.deepEqual(manifest.source, {
  title: 'FYSM 1. Практика', fileName: 'FYSM 1. Практика.pdf',
  sha256: SOURCE_SHA, pages: [4, 5, 6],
});
assert.equal(manifest.section, 'Центр');
assert.deepEqual(manifest.algorithms, EXPECTED.map(([id, name, page, steps]) => ({
  id, name, page, steps: steps.map(([name, sanskrit], index) => ({
    id: `${id}-${String(index + 1).padStart(2, '0')}`, number: index + 1, name, sanskrit,
  })),
})), 'the three algorithms must preserve all 24 original captions and their order');

const data = read('./practice/poses.json');
const failures = [], results = [];
assert.equal(data.schemaVersion, 1);
assert.equal(data.sourceSha256, SOURCE_SHA);
assert.equal(data.kind, 'static-photo-reconstructions');
const sourceFiles = ['anatomy3.js', 'cycle_lib.mjs', 'seg5_util.mjs', 'poses_build.mjs', 'package-lock.json', 'practice_build.mjs',
  'practice/algorithms.json', 'practice/struna.mjs', 'practice/detali.mjs', 'practice/centering.mjs',
  'practice/kinematics_helpers.mjs', 'practice/ground_fit.mjs', 'practice/contacts.mjs',
  'muscle_surface_render.js', 'muscles/muscles_bp3d.json',
  'body_collision.mjs', 'collision_geometry.mjs', 'skull/bones_bp3d.json', 'practice/arm_foot_collision.mjs'];
assert.deepEqual(Object.keys(data.sourceHashes || {}).sort(), [...sourceFiles].sort());
for (const name of sourceFiles) {
  const digest = createHash('sha256').update(fs.readFileSync(new URL(name, import.meta.url))).digest('hex');
  if (data.sourceHashes[name] !== digest) failures.push('stale practice input: ' + name);
}
const publishedCheck = read('./practice/check.json');
assert.equal(publishedCheck.schemaVersion, 1);
assert.equal(publishedCheck.verified, true);
assert.equal(publishedCheck.scope, 'opposite-arm-bones');
assert.equal(publishedCheck.poseCount, 24);
assert.equal(publishedCheck.boneParts, 60);
assert.deepEqual(publishedCheck.armFootCheck,
  { verified: true, scope: 'arm-foot-bones', poseCount: 24, pairGroups: 4 });
assert.equal(publishedCheck.posesSha256,
  createHash('sha256').update(fs.readFileSync(new URL('./practice/poses.json', import.meta.url))).digest('hex'));
assert.deepEqual(publishedCheck.sourceHashes, data.sourceHashes);
const ids = manifest.algorithms.flatMap(algorithm => algorithm.steps.map(step => step.id));
assert.equal(new Set(ids).size, 24);
assert.deepEqual(Object.keys(data.poses).sort(), [...ids].sort(), 'exact geometry coverage of the source poses');

const A = createAnatomy(THREE);
assert.equal(A.SEG.length, 59);
const segments = A.SEG.map(segment => segment.name);
const boneData = read('./skull/bones_bp3d.json');
const collider = createBodyCollision(A, THREE, boneData);
assert.equal(collider.boneParts, 60, 'all original opposite-arm/hand/finger parts must be present');
assert.equal(collider.scope, 'opposite-arm-bones');
const armFootCollider = createArmFootCollision(A, THREE, boneData);
assert.equal(armFootCollider.scope, 'arm-foot-bones');
assert.equal(armFootCollider.pairGroups, 4);

// The adapter copies complete original compressed vertex/index chunks and
// aliases frames only. Check every combination independently with a known
// 40 mm cube, including a 1 mm overlap and a 10 mm separating gap.
function armFootAdapterFixtures() {
  const cube = {
    vertices: [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
      [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
    indices: [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
      3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5],
  };
  const parts = [], chunks = [];
  for (const seg of ['hum_L', 'hum_R', 'foot_L', 'foot_R']) {
    const positions = Uint16Array.from(cube.vertices.flatMap(point => point.map(value => value < 0 ? 0 : 65535)));
    for (let index = positions.length - 1; index >= 3; index--) positions[index] = (positions[index] - positions[index - 3]) & 65535;
    const indices = Uint16Array.from(cube.indices);
    for (let index = indices.length - 1; index > 0; index--) indices[index] = (indices[index] - indices[index - 1]) & 65535;
    chunks.push(Buffer.from(positions.buffer), Buffer.from(indices.buffer));
    parts.push({ seg, name: 'analytic 40 mm cube', n: 8, m: 12, lo: [-.02, -.02, -.02], hi: [.02, .02, .02] });
  }
  const analytic = createArmFootCollision(A, THREE, { parts, data: gzipSync(Buffer.concat(chunks)).toString('base64') });
  let cases = 0;
  for (const armSide of ['L', 'R']) for (const footSide of ['L', 'R']) {
    for (const [distance, expected] of [[.039, true], [.05, false]]) {
      const W = {};
      for (const side of ['L', 'R']) {
        W['hum_' + side] = { p: new THREE.Vector3(side === armSide ? 0 : 10, 0, 0), q: new THREE.Quaternion() };
        W['foot_' + side] = { p: new THREE.Vector3(side === footSide ? distance : 20, 0, 0), q: new THREE.Quaternion() };
      }
      const result = analytic.inspect(W, { maxPairs: 1 });
      assert.equal(result.colliding, expected, `arm/foot adapter ${armSide}/${footSide} at ${distance} m`);
      if (expected) assert(result.pairs.every(pair => pair.a === 'hum_' + armSide && pair.b === 'foot_' + footSide
        && pair.armSide === armSide && pair.footSide === footSide), 'adapter must retain the original frame identities');
      cases++;
    }
  }
  return cases;
}
const adapterFixtureCases = armFootAdapterFixtures();
const formerGrip = read('./test_fixtures/practice_grip_crossing.json');
assert.deepEqual(Object.keys(formerGrip.poses).sort(), ['detali-06', 'detali-07']);
const formerGripRejected = [];
for (const [id, pose] of Object.entries(formerGrip.poses)) {
  const local = Object.fromEntries(Object.entries(pose.local).map(([seg, quaternion]) =>
    [seg, new THREE.Quaternion(...quaternion).normalize()]));
  const W = A.computeWorld(local, new THREE.Quaternion(...pose.root).normalize(), new THREE.Vector3(...pose.rootP));
  const report = armFootCollider.inspect(W, { maxPairs: 1 });
  assert(report.colliding && report.pairs.length > 0, id + ': the literal former bone-penetrating grip must be rejected');
  formerGripRejected.push({ id, pairs: report.pairs });
}
function distalFemurVertices(data) {
  // Packed positions are offsets from the rest segment joint, not absolute
  // stand coordinates. Decode them without rebuilding, scaling or smoothing.
  const buffer = gunzipSync(Buffer.from(data.data, 'base64'));
  const output = {};
  let offset = 0;
  for (const part of data.parts) {
    if (!/^thigh_[LR]$/.test(part.seg) || part.latin !== 'femur') {
      offset += 6 * (part.n + part.m); continue;
    }
    const kneeOffset = A.S['shank_' + part.seg.at(-1)].J.clone().sub(A.S[part.seg].J);
    const accumulator = [0, 0, 0], points = [];
    for (let i = 0; i < part.n; i++) {
      const xyz = [];
      for (let axis = 0; axis < 3; axis++) {
        accumulator[axis] = (accumulator[axis] + buffer.readUInt16LE(offset)) & 65535; offset += 2;
        xyz.push(part.lo[axis] + (part.hi[axis] - part.lo[axis]) * accumulator[axis] / 65535);
      }
      // Include the condyles and the distal 8 cm of shaft above the rig knee.
      // This only selects a source-mesh region; it does not change its radius.
      if (xyz[1] <= kneeOffset.y + .08) points.push(new THREE.Vector3(...xyz));
    }
    offset += 6 * part.m;
    assert(points.length > 100, 'the complete source condylar region must be present: ' + part.seg);
    assert(!output[part.seg], 'a single unchanged source femur per side');
    output[part.seg] = points;
  }
  assert.equal(offset, buffer.length, 'femur decoder consumes the exact packed payload');
  assert.deepEqual(Object.keys(output).sort(), ['thigh_L', 'thigh_R']);
  return output;
}
const distalFemurs = distalFemurVertices(boneData);
const contactFactory = await createPracticeContacts(A, THREE, read('./muscles/muscles_bp3d.json'), { boneData });
const groundFitter = createPracticeGroundFitter({ A, THREE, contactFactory, boneData });
const kneeSurfaces = await createMuscleSurfaces(A, THREE, read('./muscles/muscles_bp3d.json'), {
  instances: A.strandInstances().filter(instance => instance.kind === 'muscle' && instance.unit === 'vastMed'),
});
assert(kneeSurfaces && kneeSurfaces.pickables.length === 2,
  'both original vastus medialis surfaces must load for the lateral knee support check');
const FLOOR_TOLERANCE = 2e-6; // Actual decoded geometry; landmarks are diagnostics.
const CONTACT_HEIGHT = .004; // The same geometric contact threshold used by anatomy3.js.
const FOOT = new Set(['пятки', 'подушечки стоп', 'наружный край стопы', 'пальцы стоп', 'тыл стоп']);
const HAND = new Set(['ладони', 'пальцы рук', 'кисти']);
const FOREARM = new Set(['предплечья', 'локти']);
const UPPER_BACK = new Set(['спина', 'плечи']);
const FRONT_TRUNK = new Set(['грудь', 'нижние рёбра']);
const ASYMMETRIC = new Set(['struna-06', 'struna-07', 'detali-04', 'detali-08',
  'centering-03', 'centering-04', 'centering-05', 'centering-06', 'centering-07']);
let candidatePairs = 0, triangleTests = 0, armFootCandidatePairs = 0, armFootTriangleTests = 0;
const check = (condition, label) => { if (!condition) failures.push(label); };
const finiteArray = (value, length) => Array.isArray(value) && value.length === length && value.every(Number.isFinite);
const spineSegments = { ls: 'lumA', l34: 'lumB', tl: 'thA', t89: 'thB', t45: 'thC', ct: 'neckA', c45: 'neckB', ao: 'head' };
// Regression for concentrating a photographed regional turn in one small
// model hinge. This is an input-quality check, not a human strength limit.
assert(Math.max(...spineJointTorque(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -25 * Math.PI / 180), SPINE_PASSIVE.t89).map(Math.abs)) > 1e6,
  'the old C7 concentrated-rotation fixture must expose extreme spring extrapolation');
const quat = (value, label) => {
  assert(finiteArray(value, 4), label + ' must contain four finite quaternion components');
  assert(Math.abs(Math.hypot(...value) - 1) <= 1e-6, label + ' must be a unit quaternion');
  return new THREE.Quaternion(...value).normalize();
};

for (const id of ids) {
  try {
    const pose = data.poses[id];
    assert.deepEqual(Object.keys(pose.local).sort(), [...segments].sort(), id + ': complete 59-segment rig');
    const local = Object.fromEntries(segments.map(name => [name, quat(pose.local[name], id + ':' + name)]));
    const root = quat(pose.root, id + ':root');
    assert(finiteArray(pose.rootP, 3), id + ': finite root position');
    assert(finiteArray(pose.view, 3) && Math.hypot(...pose.view) > .1, id + ': finite nonzero camera');
    assert(Array.isArray(pose.notes) && pose.notes.length > 0
      && pose.notes.every(note => typeof note === 'string' && /[а-яё]/iu.test(note)), id + ': Russian reconstruction notes');
    const W = A.computeWorld(local, root, new THREE.Vector3(...pose.rootP));
    for (const name of segments) {
      assert(W[name].p.toArray().every(Number.isFinite), id + ': finite world position of ' + name);
      assert(W[name].q.toArray().every(Number.isFinite), id + ': finite world rotation of ' + name);
    }
    const markers = A.LANDMARKS.map(marker => ({ ...marker, world: A.pt(W, marker.seg, marker.pc) }));
    assert(markers.every(marker => marker.world.toArray().every(Number.isFinite)), id + ': finite floor markers');
    const minY = Math.min(...markers.map(marker => marker.world.y));
    const actualGround = groundFitter.measure({ id, local, rootQ: root, rootP: new THREE.Vector3(...pose.rootP) });
    check(actualGround.minimum.heightM >= -FLOOR_TOLERANCE, `${id}: actual bone/muscle/skin floor penetration ${actualGround.minimum.heightM} m`);
    check(actualGround.primary.every(group => group.present), `${id}: an intended anatomical support is missing`);
    const contactModel = contactFactory.build(id, W, { audit: true });
    check(contactModel.diagnostics.surfaceGeometryValid, `${id}: actual outer muscle penetrates floor`);
    check(Number.isFinite(contactModel.diagnostics.unclusteredSupport.margin)
      && contactModel.diagnostics.unclusteredSupport.margin >= -1e-6, `${id}: COM outside geometric support`);
    const contacts = contactModel.contacts.map(c => ({ ...c, world: c.P }));
    const has = (regions, side) => contacts.some(marker => regions.has(marker.region) && (!side || marker.side === side));
    const sides = regions => ['L', 'R'].filter(side => has(regions, side));
    const both = (regions, label) => check(sides(regions).length === 2, id + ': missing bilateral ' + label + ' floor support');
    const any = (regions, label) => check(has(regions), id + ': missing ' + label + ' floor support');
    const regionSides = [...new Set(contacts.map(marker => marker.region + ':' + marker.side))].sort();
    const markerRegionSides = [...new Set(markers.filter(marker => marker.world.y < CONTACT_HEIGHT)
      .map(marker => marker.region + ':' + marker.side))].sort();
    check(contacts.length >= (['detali-03', 'detali-05', 'detali-06', 'detali-07', 'detali-08'].includes(id) ? 1 : 2),
      id + ': insufficient geometric floor markers');
    // Source-photo support constraints, evaluated on actual near-floor patches
    // and explicitly identified missing-skin proxies. This is geometry only.
    if (['struna-01', 'struna-02', 'struna-03', 'struna-04', 'struna-06',
      'centering-01', 'centering-03', 'centering-04'].includes(id)) both(FOOT, 'foot');
    if (['struna-05', 'detali-01', 'struna-07', 'centering-02'].includes(id)) both(HAND, 'hand');
    if (['struna-07', 'centering-02'].includes(id)) any(FOOT, 'foot');
    if (id === 'detali-01') check(!has(new Set(['ягодицы', 'таз'])), id + ': pelvis must be lifted in the L-sit');
    if (id === 'detali-04') { any(UPPER_BACK, 'upper back/shoulder'); any(FOOT, 'supporting foot'); }
    if (id === 'centering-05') { both(FOREARM, 'forearm'); any(FOOT, 'supporting foot'); }
    if (id === 'centering-06') {
      any(HAND, 'hand'); any(FOREARM, 'forearm');
      check(['L', 'R'].some(side => has(HAND, side) && has(FOREARM, side === 'L' ? 'R' : 'L')),
        id + ': requires a hand and the opposite forearm');
      check(!has(FOOT), id + ': feet must be lifted in the arm balance');
    }
    if (['centering-07', 'centering-08'].includes(id)) both(new Set(['колени']), 'knee');
    if (id === 'centering-07') any(FOREARM, 'forearm');
    if (id === 'centering-08') check(has(FOREARM) || has(FRONT_TRUNK), id + ': missing front trunk/forearm support');

    // Broad source-variant guardrails, not measured anatomy or physiological
    // tolerances. These distinguish the photographed heel lift, hand locks,
    // hands behind the head and lunges from substantially different poses.
    const variant = {};
    if (['struna-06', 'centering-07', 'centering-08'].includes(id)) {
      const moments = Object.entries(spineSegments).map(([joint, segment]) => ({ joint, segment,
        torqueNm: Array.from(spineJointTorque(local[segment], SPINE_PASSIVE[joint])) }));
      variant.spinalRotationRegression = moments;
      check(moments.every(m => m.torqueNm.every(n => Number.isFinite(n) && Math.abs(n) < 100)),
        id + ': concentrated spinal rotation causes extreme passive spring extrapolation');
    }
    const fingerTips = side => markers.filter(marker => marker.side === side && /^tip[1-5]$/.test(marker.tag || ''))
      .map(marker => marker.world);
    const jointFlex = (type, side) => {
      const joint = A.JOINTS.find(joint => joint.type === type && joint.side === side);
      assert(joint, id + ': missing anatomical ' + type + ' joint on side ' + side);
      return A.analyzeJoint(joint, local, W).comps.find(component => component.key === 'flex').signed;
    };
    if (id === 'detali-06') {
      // Its abducted knees rest on the lateral soft-tissue surface in the PDF.
      // The existing patellar marker faces anteriorly and is not that contact.
      // Requiring it to touch the floor would push the actual condyles through
      // the floor. Skin every vertex of the real displayed vastus medialis
      // meshes and inspect the 100 mm neighbourhood of each knee instead.
      // This is rendered-surface contact, not a solved pressure or force.
      kneeSurfaces.update(W);
      variant.kneeSurfaceSupport = {};
      for (const side of ['L', 'R']) {
        const seg = 'thigh_' + side, transform = W[seg];
        const kneeHeight = W['shank_' + side].p.y;
        const boneHeight = Math.min(...distalFemurs[seg].map(point => point.clone()
          .applyQuaternion(transform.q).add(transform.p).y));
        const mesh = kneeSurfaces.pickables.find(mesh => mesh.userData.side === side);
        assert(mesh && mesh.userData.unit === 'vastMed', id + ': original muscle surface on ' + side);
        const point = new THREE.Vector3();
        let nearbyVertices = 0, skinHeight = Infinity;
        for (let index = 0; index < mesh.geometry.attributes.position.count; index++) {
          mesh.getVertexPosition(index, point);
          assert(point.toArray().every(Number.isFinite), id + ': finite skinned knee surface vertex');
          if (point.distanceTo(W['shank_' + side].p) > .1) continue;
          nearbyVertices++; skinHeight = Math.min(skinHeight, point.y);
        }
        variant.kneeSurfaceSupport[side] = { jointHeightM: kneeHeight,
          distalFemurMinHeightM: boneHeight, sourceBoneVertices: distalFemurs[seg].length,
          muscleUnit: mesh.userData.unit, nearbyMuscleVertices: nearbyVertices, muscleMinHeightM: skinHeight };
        check(nearbyVertices > 100, `${id}: insufficient actual ${side} knee muscle vertices (${nearbyVertices})`);
        check(skinHeight >= -.001 && skinHeight < CONTACT_HEIGHT,
          `${id}: actual ${side} knee muscle surface must meet the floor without penetration (${skinHeight} m)`);
        check(boneHeight >= -.001,
          `${id}: actual ${side} distal femur penetrates the floor (${boneHeight} m)`);
      }
    }
    if (['struna-01', 'struna-04'].includes(id)) {
      variant.lowestHeelM = Math.min(...markers.filter(marker => marker.region === 'пятки').map(marker => marker.world.y));
      check(variant.lowestHeelM > .01, `${id}: source heels must be visibly lifted; lowest heel ${variant.lowestHeelM} m`);
    }
    if (['struna-01', 'struna-02'].includes(id)) {
      const left = fingerTips('L'), right = fingerTips('R');
      assert.equal(left.length, 5); assert.equal(right.length, 5);
      variant.nearestOppositeFingertipsM = Math.min(...left.flatMap(a => right.map(b => a.distanceTo(b))));
      check(variant.nearestOppositeFingertipsM < .03,
        `${id}: finger lock is open; nearest opposite fingertips ${variant.nearestOppositeFingertipsM} m`);
    }
    if (id === 'struna-03') {
      variant.nearestFingertipToOppositeWristM = Math.min(...['L', 'R'].flatMap(side =>
        fingerTips(side).map(tip => tip.distanceTo(W['hand_' + (side === 'L' ? 'R' : 'L')].p))));
      check(variant.nearestFingertipToOppositeWristM < .05,
        `${id}: wrist lock is open; nearest fingertip to opposite wrist ${variant.nearestFingertipToOppositeWristM} m`);
    }
    if (['struna-08', 'detali-02'].includes(id)) {
      variant.wristToHeadM = {}; variant.elbowFlexDeg = {};
      for (const side of ['L', 'R']) {
        const distance = W['hand_' + side].p.distanceTo(W.head.p);
        const flex = jointFlex('elbow', side);
        variant.wristToHeadM[side] = distance; variant.elbowFlexDeg[side] = flex;
        check(distance < .25, `${id}: ${side} wrist is too far from the head (${distance} m)`);
        check(flex > 70, `${id}: ${side} elbow must bend for the hands-behind-head variant (${flex} degrees)`);
      }
    }
    if (['centering-03', 'centering-04'].includes(id)) {
      // The source reconstruction uses a forward lunge along +Z for Warrior I
      // and a lateral lunge along +X for its photographed side-warrior variant.
      const axis = id === 'centering-03' ? 'z' : 'x';
      const front = W.foot_L.p[axis] > W.foot_R.p[axis] ? 'L' : 'R';
      const back = front === 'L' ? 'R' : 'L';
      variant.frontSide = front;
      variant.frontKneeFlexDeg = jointFlex('knee', front);
      variant.backKneeFlexDeg = jointFlex('knee', back);
      check(variant.frontKneeFlexDeg >= 65,
        `${id}: source front knee must be visibly bent (${variant.frontKneeFlexDeg} degrees)`);
      check(Math.abs(variant.backKneeFlexDeg) <= 15,
        `${id}: source back leg must stay approximately straight (${variant.backKneeFlexDeg} degrees)`);
    }

    // An asymmetric photograph must not accidentally become a mirrored pose.
    let asymmetryRad = 0;
    for (const segment of A.SEG) {
      if (segment.side === 'L') {
        const right = segment.name.replace(/_L$/, '_R');
        asymmetryRad = Math.max(asymmetryRad, A.mirrorQ(local[segment.name]).angleTo(local[right]));
      } else if (segment.side === 'C') {
        asymmetryRad = Math.max(asymmetryRad, A.mirrorQ(local[segment.name]).angleTo(local[segment.name]));
      }
    }
    if (ASYMMETRIC.has(id)) check(asymmetryRad > 1e-3, id + ': source asymmetry is missing');
    const collision = collider.inspect(W, { maxPairs: 6, maxTriangleIntersections: 1 });
    candidatePairs += collision.candidatePairs;
    triangleTests += collision.triangleTests;
    check(!collision.colliding, id + ': actual opposite-arm bones intersect: ' + JSON.stringify(collision.pairs));
    const armFootCollision = armFootCollider.inspect(W, { maxPairs: 6 });
    armFootCandidatePairs += armFootCollision.candidatePairs;
    armFootTriangleTests += armFootCollision.triangleTests;
    check(!armFootCollision.colliding, id + ': actual arm/foot bones intersect: ' + JSON.stringify(armFootCollision.pairs));
    // Compare saved diagnostics with the independently decoded geometry.
    assert(pose.geometry && Number.isFinite(pose.geometry.minMarkerHeightM), id + ': floor diagnostic');
    check(Math.abs(pose.geometry.minMarkerHeightM - minY) <= 1e-8, id + ': stale floor diagnostic');
    check(JSON.stringify([...pose.geometry.contacts].sort()) === JSON.stringify(markerRegionSides), id + ': stale marker contact diagnostic');
    check(Math.abs(pose.geometry.ground.after.minimum.heightM - actualGround.minimum.heightM) <= 1e-8,
      id + ': stale actual floor diagnostic');
    results.push({ id, minY, contacts: regionSides, variant, asymmetryDeg: asymmetryRad * 180 / Math.PI,
      colliding: collision.colliding, pairs: collision.pairs,
      armFootColliding: armFootCollision.colliding, armFootPairs: armFootCollision.pairs });
  } catch (error) { failures.push(id + ': ' + error.message); }
}
kneeSurfaces.dispose();
contactFactory.dispose();

// Exercise the real Python publication gate in an isolated checkout-like
// fixture. Every file that is changed here is a private copy, never a symlink.
// Other assets are only read through symlinks, and the output stays in /tmp.
function negativeBuildChecks() {
  const root = path.dirname(fileURLToPath(import.meta.url));
  const fixture = fs.mkdtempSync(path.join(os.tmpdir(), 'fysm-practice-build-'));
  const privateCopies = new Set(['build.py', ...sourceFiles, 'practice/poses.json', 'practice/check.json',
    'practice/physics.json', 'practice/physics-check.json', 'physics.json', 'cycle_data.json', 'cycle_phys.json']);
  const privateDirectories = new Set(['practice', 'skull']);
  const skipped = new Set(['.git', '.heavy.lock', 'node_modules', 'dist']);
  function snapshotDirectory(relative = '') {
    const from = path.join(root, relative), to = path.join(fixture, relative);
    fs.mkdirSync(to, { recursive: true });
    for (const entry of fs.readdirSync(from, { withFileTypes: true })) {
      const name = relative ? relative + '/' + entry.name : entry.name;
      if (!relative && skipped.has(entry.name)) continue;
      const source = path.join(root, name), destination = path.join(fixture, name);
      if (entry.isDirectory() && privateDirectories.has(name)) snapshotDirectory(name);
      else if (privateCopies.has(name)) fs.copyFileSync(source, destination);
      else fs.symlinkSync(source, destination);
    }
  }
  try {
    snapshotDirectory();
    const run = () => spawnSync('python3', ['build.py', 'dist-fixture'], {
      cwd: fixture, encoding: 'utf8', maxBuffer: 1024 * 1024,
      env: { ...process.env, CYCLE_DATA: path.join(fixture, 'cycle_data.json') },
    });
    const baseline = run();
    assert.equal(baseline.status, 0, 'the unmodified private build fixture must pass: ' + baseline.stderr);
    const output = path.join(fixture, 'dist-fixture');
    assert(!fs.readFileSync(path.join(output, 'index.html'), 'utf8').includes('__dbg'));
    const posesPath = path.join(fixture, 'practice/poses.json');
    const checkPath = path.join(fixture, 'practice/check.json');
    const builderPath = path.join(fixture, 'practice/struna.mjs');
    const originalPoses = fs.readFileSync(posesPath);
    const originalCheck = fs.readFileSync(checkPath);
    const originalBuilder = fs.readFileSync(builderPath);
    const physicsPath = path.join(fixture, 'practice/physics.json');
    const physicsCheckPath = path.join(fixture, 'practice/physics-check.json');
    const numericOriginals = Object.fromEntries(['practice/physics.json', 'practice/physics-check.json',
      'physics.json', 'cycle_data.json', 'cycle_phys.json'].map(name => [name, fs.readFileSync(path.join(fixture, name))]));
    const sha = bytes => createHash('sha256').update(bytes).digest('hex');
    const rewritePhysics = mutate => {
      const changed = JSON.parse(numericOriginals['practice/physics.json']); mutate(changed);
      const bytes = JSON.stringify(changed); fs.writeFileSync(physicsPath, bytes);
      const witness = JSON.parse(numericOriginals['practice/physics-check.json']); witness.physicsSha256 = sha(bytes);
      fs.writeFileSync(physicsCheckPath, JSON.stringify(witness));
    };
    const cases = [
      ['changed-geometry', () => {
        const changed = JSON.parse(originalPoses);
        changed.poses['struna-01'].rootP[0] += .001;
        fs.writeFileSync(posesPath, JSON.stringify(changed));
      }],
      ['stale-practice-builder', () => fs.appendFileSync(builderPath, '\n// stale-input regression fixture\n')],
      ['missing-verification', () => {
        const changed = JSON.parse(originalCheck); delete changed.verified;
        fs.writeFileSync(checkPath, JSON.stringify(changed));
      }],
      ['changed-practice-forces', () => fs.appendFileSync(physicsPath, ' '), /Practice physics verification/],
      ['unverified-force-assembly', () => rewritePhysics(p => { p.poses['struna-01'].diag.physicalBalance.verified = false; }), /Practice physics verification/],
      ['unverified-contact-geometry', () => rewritePhysics(p => { p.poses['struna-01'].diag.contactGeometryVerified = false; }), /Practice physics verification/],
      ['contradictory-physical-tolerance', () => rewritePhysics(p => { p.poses['struna-01'].diag.physicalBalance.forceResidualN = 1e6; }), /Practice physics verification/],
      ['missing-pad-sensitivity', () => rewritePhysics(p => { delete p.poses['struna-08'].modelLimits.softTissueContactSensitivity; }), /Practice soft-tissue contact sensitivity/],
      ['stale-legacy-force-output', () => fs.appendFileSync(path.join(fixture, 'physics.json'), ' '), /Static physics verification/],
      ['stale-cycle-force-output', () => fs.appendFileSync(path.join(fixture, 'cycle_phys.json'), ' '), /Round physics verification/],
      ['unverified-packed-cycle', () => {
        const p = JSON.parse(numericOriginals['cycle_data.json']); p.modelQuality.physicalBalanceVerified = false;
        fs.writeFileSync(path.join(fixture, 'cycle_data.json'), JSON.stringify(p));
      }, /Round physics verification/],
    ];
    for (const [name, mutate, expected = /Practice geometry is missing or stale/] of cases) {
      fs.writeFileSync(posesPath, originalPoses);
      fs.writeFileSync(checkPath, originalCheck);
      fs.writeFileSync(builderPath, originalBuilder);
      for (const [name, bytes] of Object.entries(numericOriginals)) fs.writeFileSync(path.join(fixture, name), bytes);
      fs.rmSync(output, { recursive: true, force: true });
      mutate();
      const result = run();
      assert.notEqual(result.status, 0, name + ' must block publication');
      assert.match(result.stderr, expected, name + ': fail for the actual publication guard');
      assert(!fs.existsSync(output), name + ': no page may be emitted after rejection');
    }
    return cases.length;
  } finally { fs.rmSync(fixture, { recursive: true, force: true }); }
}
let staleBuildsBlocked = 0;
if (!failures.some(message => message.startsWith('stale practice input:'))) {
  try { staleBuildsBlocked = negativeBuildChecks(); }
  catch (error) { failures.push('isolated publication guard: ' + error.message); }
}

console.log(JSON.stringify({ passed: failures.length === 0, algorithms: 3, sourcePoses: 24,
  testedPoses: results.length, boneParts: collider.boneParts, scope: collider.scope,
  actualFloorToleranceM: FLOOR_TOLERANCE, floorMarkersDiagnosticOnly: true, contactMarkerHeightM: CONTACT_HEIGHT,
  candidatePairs, triangleTests, armFootCandidatePairs, armFootTriangleTests,
  armFootScope: armFootCollider.scope, armFootPairGroups: armFootCollider.pairGroups,
  adapterFixtureCases, formerGripRejected, staleBuildsBlocked, poses: results, failures }, null, 2));
assert.equal(failures.length, 0, 'Practice geometry failed:\n' + failures.join('\n'));
