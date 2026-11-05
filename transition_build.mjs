// Offline, geometrically audited demonstrations between the 30 saved asanas.
// This does not assert measured choreography, balance, or dynamic muscle force.
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { gunzipSync } from 'node:zlib';
import { A, THREE, V, X, Z, qAx, blankLocal, rootPlaced, setSide, armIK, orientHand } from './cycle_lib.mjs';
import { createBodyCollision } from './body_collision.mjs';
import { createArmFootCollision } from './practice/arm_foot_collision.mjs';
import { createPracticeContacts } from './practice/contacts.mjs';
import { createTransitionFloor } from './transition_floor.mjs';
import { createTransitions, transitionSlerp } from './transition_runtime.mjs';
import { createTransitionCollision } from './transition_collision.mjs';
import { rotateElbowPlane } from './practice/kinematics_helpers.mjs';
import { repairBowGrip } from './transition_grip.mjs';
import { findTransitionDetour, detourFamiliesForPair } from './transition_path.mjs';
import { createToeCollisionOracle } from './transition_toe_oracle.mjs';
import { followFoldShins } from './transition_fold_grip.mjs';
import { createElbowMotionGuard } from './transition_motion.mjs';
import { prepareBowRelease } from './transition_bow_release.mjs';
import { createThumbCollisionOracle, findThumbRelease } from './transition_thumb_release.mjs';
import { prepareSpiderRelease } from './transition_spider_release.mjs';
import { createHipCollisionOracle, findHipFlexionDetour } from './transition_hip_detour.mjs';

const probeAll = process.argv.includes('--probe-all');
const fromPlan = process.argv.includes('--from-plan');
const migratePlan = process.argv.includes('--migrate-plan');
const partialReplan = process.argv.includes('--partial-replan');
const repairReplan = process.argv.includes('--repair-bow-half-releases');
const deferredFloor = process.argv.includes('--deferred-floor');
const geometryOnly = !fromPlan && (partialReplan || repairReplan || process.argv.includes('--geometry-only') || deferredFloor);
const candidateGeneration = partialReplan || repairReplan || migratePlan || probeAll || process.argv.includes('--geometry-only') || process.argv.includes('--plan-only');
if (fromPlan && candidateGeneration) throw new Error('A plan input cannot also be a candidate-generation output');
if (migratePlan && (probeAll || geometryOnly || process.argv.includes('--plan-only')))
  throw new Error('Coordinate migration must run independently of planning/release modes');
if (partialReplan && (migratePlan || probeAll || deferredFloor || process.argv.includes('--geometry-only') || process.argv.includes('--plan-only')))
  throw new Error('Partial coordinate replanning must run independently');
if (repairReplan && (partialReplan || migratePlan || probeAll || deferredFloor || process.argv.includes('--geometry-only') || process.argv.includes('--plan-only')))
  throw new Error('Bow/half motion repair must run independently');
const planSourceName = 'transitions/plan.json.gz';

const read = name => fs.readFileSync(new URL(name, import.meta.url));
const json = name => JSON.parse(read(name));
const sha = value => createHash('sha256').update(value).digest('hex');
const originalInputs = ['anatomy3.js', 'poses.json', 'practice/poses.json', 'practice/algorithms.json', 'cycle_lib.mjs',
  'transition_runtime.mjs', 'transition_build.mjs', 'transition_floor.mjs', 'transition_collision.mjs', 'transition_grip.mjs', 'transition_fold_grip.mjs', 'transition_bow_release.mjs', 'transition_thumb_release.mjs', 'transition_spider_release.mjs', 'transition_hip_detour.mjs', 'transition_motion.mjs', 'transition_path.mjs', 'transition_toe_oracle.mjs', 'practice/ground_fit.mjs', 'practice/contacts.mjs', 'practice/kinematics_helpers.mjs', 'seg5_util.mjs',
  'muscle_surface_render.js', 'muscles/muscles_bp3d.json', 'skull/bones_bp3d.json', 'body_collision.mjs',
  'collision_geometry.mjs', 'practice/arm_foot_collision.mjs', 'whole_body_collision.mjs', 'package-lock.json'];
// Candidate generation cannot hash its own future compressed output. Every
// release records the raw candidate artifact as ordinary, untrusted input.
const inputs = candidateGeneration ? originalInputs : [...originalInputs, planSourceName];
const hashes = () => Object.fromEntries(inputs.map(name => [name, sha(read(name))]));
const sourceHashes = hashes();
const motionGuard = createElbowMotionGuard(A), motionCache = new Map();
let jointMotionRejectedCandidates = 0;
const boneData = json('skull/bones_bp3d.json'), arm = createBodyCollision(A, THREE, boneData),
  armFoot = createArmFootCollision(A, THREE, boneData),
  contacts = await createPracticeContacts(A, THREE, json('muscles/muscles_bp3d.json'), { boneData }),
  floor = createTransitionFloor(A, THREE, contacts, boneData), critical = createTransitionCollision(A, THREE, boneData),
  toeOracle = createToeCollisionOracle(A, THREE, boneData), thumbOracle = createThumbCollisionOracle(A, THREE, boneData),
  hipOracle = createHipCollisionOracle(A, THREE, boneData);
const poseData = json('poses.json'), practice = json('practice/poses.json').poses, algorithms = json('practice/algorithms.json').algorithms;
const clone = c => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, c.local[s.name].clone()])), rootQ: c.rootQ.clone(), rootP: c.rootP.clone() });
const fromData = p => {
  const c = { local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion(...p.local[s.name]).normalize()])),
    rootQ: new THREE.Quaternion(...p.root).normalize() };
  c.rootP = p.rootP ? new THREE.Vector3(...p.rootP) : rootPlaced(c.local, c.rootQ);
  return c;
};
// Nanometre / quaternion 1e-9 storage is well below the geometric guard
// tolerances; all between-frame checks use these saved values again.
const number = n => Math.round(n * 1e9) / 1e9;
const encode = (c, t = 0) => ({ t: number(t), local: Object.fromEntries(A.SEG.map(s => [s.name, c.local[s.name].toArray().map(number)])),
  root: c.rootQ.toArray().map(number), rootP: c.rootP.toArray().map(number) });
const mix = (a, b, u) => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, transitionSlerp(a.local[s.name].clone(), b.local[s.name], u)])),
  rootQ: transitionSlerp(a.rootQ.clone(), b.rootQ, u), rootP: a.rootP.equals(b.rootP) ? a.rootP.clone() : a.rootP.clone().lerp(b.rootP, u) });
const ease = u => u * u * (3 - 2 * u);
const identicalPose = (a, b) => a.rootQ.equals(b.rootQ) && a.rootP.equals(b.rootP)
  && A.SEG.every(s => a.local[s.name].equals(b.local[s.name]));
function edgeSpec(a, b) {
  const angle = Math.max(a.rootQ.angleTo(b.rootQ), ...A.SEG.map(s => a.local[s.name].angleTo(b.local[s.name])));
  const guide = a.transitionGuide === true && b.transitionGuide === true;
  return { angle, guide, count: Math.max(guide ? 2 : 12, Math.ceil(angle * (guide ? 1 : 1.5) / .04)),
    duration: number(Math.max(guide ? .08 : .7, angle / .75)), curve: guide ? u => u : ease };
}
const catalog = A.POSES.map(p => ({ id: p.id, name: p.name || p.ru || p.id, kind: 'pose' }));
for (const algorithm of algorithms) for (const [stepIndex, step] of algorithm.steps.entries())
  catalog.push({ id: step.id, name: step.name, kind: 'practice', algorithmId: algorithm.id, stepIndex });
if (catalog.length !== 30 || new Set(catalog.map(p => p.id)).size !== 30) throw new Error('Expected 30 distinct asanas');
const raw = Object.fromEntries(catalog.map(p => [p.id, fromData(p.kind === 'pose' ? poseData[p.id] : practice[p.id])]));
const hub = { local: blankLocal(), rootQ: new THREE.Quaternion(), rootP: V(0, 0, 0) };
for (const side of ['L', 'R']) {
  setSide(hub.local, side, 'hum', qAx(Z, 55));
  setSide(hub.local, side, 'fore', qAx(X, -15));
  setSide(hub.local, side, 'thigh', qAx(Z, 8));
  setSide(hub.local, side, 'f1m', qAx(X, -35));
}
raw.__neutral = hub;
const CLEARANCE = .00075;
const endpointMetadata = {};
const endpointCacheFile = '/tmp/fysm-transition-endpoints-cache.json';
const endpointSourceNames = ['anatomy3.js', 'poses.json', 'practice/poses.json', 'practice/algorithms.json',
  'skull/bones_bp3d.json', 'muscles/muscles_bp3d.json', 'muscle_surface_render.js', 'package-lock.json',
  'transition_grip.mjs', 'transition_fold_grip.mjs', 'cycle_lib.mjs', 'practice/kinematics_helpers.mjs'];
const endpointSources = Object.fromEntries(endpointSourceNames.map(name => [name, sourceHashes[name]]));
const endpointPolicySha256 = sha(JSON.stringify({ version: 1,
  functions: [endpointVariant, armEndpointVariant, withdrawArms, clone, encode, fromData].map(fn => fn.toString()),
  neutral: encode(hub), clearanceM: CLEARANCE }));
let endpointCache = { schemaVersion: 1, verified: false, sourceHashes: endpointSources, endpointPolicySha256, candidates: {} };
try {
  const previous = JSON.parse(fs.readFileSync(endpointCacheFile));
  if (previous.schemaVersion === 1 && previous.endpointPolicySha256 === endpointPolicySha256
      && JSON.stringify(previous.sourceHashes) === JSON.stringify(endpointSources)
      && previous.candidates && typeof previous.candidates === 'object') endpointCache = previous;
} catch {}
let collisionStates = 0, collisionEvaluations = 0, collisionCacheHits = 0, lowerEvaluations = 0, lowerCacheHits = 0;
const collisionCache = new Map(), lowerCache = new Map();
const lowerSegments = A.SEG.filter(s => !/^(clav|scap|hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/.test(s.name)).map(s => s.name);
function lowerReport(c, W) {
  const key = JSON.stringify(lowerSegments.map(s => c.local[s].toArray()));
  if (lowerCache.has(key)) { lowerCacheHits++; return lowerCache.get(key); }
  lowerEvaluations++;
  const report = critical.inspectLower(W || A.computeWorld(c.local, new THREE.Quaternion(), V(0, 0, 0)));
  const cached = report.colliding ? { colliding: true, scope: report.scope, pairs: report.pairs }
    : { colliding: false, scope: report.scope, pairs: [] };
  if (lowerCache.size > 100000) lowerCache.clear();
  lowerCache.set(key, cached); return cached;
}
function armEndpointVariant(id, source) {
  const first = collision(source);
  if (!first) return { conf: source, adjustment: null };
  if (id === 'bow') {
    const grip = repairBowGrip({ A, THREE, conf: source, accept: candidate => !collision(candidate) });
    if (grip) return grip;
    throw new Error('A bounded closed bow grip could not be verified: ' + JSON.stringify(first));
  }
  if (['plank', 'centering-02', 'handstand'].includes(id)) for (const degrees of [.5, 1, 1.5, 2, 3, 4, 6, 8]) {
    const c = clone(source);
    for (const side of ['L', 'R']) c.local['thigh_' + side].premultiply(qAx(Z, (side === 'L' ? 1 : -1) * degrees)).normalize();
    if (!collision(c)) return { conf: c, adjustment: { kind: 'bilateral-hip-abduction', degrees } };
  }
  if (id === 'detali-08') for (const degrees of [2, 4, 6, 8, 12, 18, 25, 35]) {
    const c = clone(source); c.local.hum_R.premultiply(qAx(Z, -degrees)).normalize();
    if (!collision(c)) return { conf: c, adjustment: { kind: 'right-upper-arm-abduction', degrees } };
  }
  for (const degrees of [-15, 15, -30, 30, -45, 45, -60, 60, -90, 90, -120, 120]) {
    const c = clone(source);
    for (const side of ['L', 'R']) rotateElbowPlane(c, side, (side === 'L' ? 1 : -1) * degrees);
    if (!collision(c)) return { conf: c, adjustment: { kind: 'bilateral-elbow-plane', degrees } };
  }
  for (const distance of [.005, .01, .015, .02, .03, .04, .06, .08, .1, .14])
    for (const vector of [[1, 0, 0], [-1, 0, 0], [1, 0, -1], [-1, 0, -1], [1, 0, 1], [-1, 0, 1], [0, 0, 1], [0, 0, -1], [1, 1, 0], [1, -1, 0]]) {
      const c = withdrawArms(source, ['L', 'R'], distance, vector, false);
      if (!collision(c)) return { conf: c, adjustment: { kind: 'bilateral-wrist-withdrawal', distanceM: distance, bodyDirection: vector } };
      if (id === 'fold') for (const degrees of [-30, 30, -60, 60, -90, 90]) {
        const turn = clone(c);
        for (const side of ['L', 'R']) rotateElbowPlane(turn, side, (side === 'L' ? 1 : -1) * degrees);
        if (!collision(turn)) return { conf: turn, adjustment: { kind: 'bilateral-wrist-withdrawal-and-elbow-plane', distanceM: distance, bodyDirection: vector, degrees } };
      }
    }
  throw new Error('Source pose requires an unresolved nonadjacent collision repair: ' + id + ': ' + JSON.stringify(first));
}
function endpointVariant(id, original) {
  const lowerHit = c => lowerReport(c).colliding;
  let source = original, lowerAdjustment = null;
  if (lowerHit(source)) {
    const choices = [];
    for (const toeScale of [1, .95, .9, .85, .8, .7, .6, .4, .2, 0])
      for (const spreadDegrees of [0, .5, 1, 1.5, 2, 3, 4, 6, 8])
        for (const hipScale of [1, .99, .98, .96, .94, .9, .85, .8, .75])
          choices.push({ toeScale, spreadDegrees, hipScale, cost: (1 - toeScale) * 30 + spreadDegrees + (1 - hipScale) * 100 });
    choices.sort((a, b) => a.cost - b.cost);
    for (const option of choices) {
      const candidate = clone(original);
      for (const side of ['L', 'R']) {
        candidate.local['toes_' + side].slerp(new THREE.Quaternion(), 1 - option.toeScale).normalize();
        candidate.local['thigh_' + side].slerp(new THREE.Quaternion(), 1 - option.hipScale)
          .premultiply(qAx(Z, (side === 'L' ? 1 : -1) * option.spreadDegrees)).normalize();
      }
      if (!lowerHit(candidate)) {
        source = candidate;
        lowerAdjustment = { kind: 'lower-limb-bone-clearance', toeHingeScale: option.toeScale,
          hipRotationScale: option.hipScale, bilateralHipAbductionDegrees: option.spreadDegrees,
          note: 'Для анимации углы ног и пальцев стоп уточнены по реальным костным поверхностям; это адаптированный геометрический вариант исходной позы.' };
        break;
      }
    }
    if (!lowerAdjustment) {
      const referenceWorld = A.computeWorld(original.local, original.rootQ, original.rootP);
      searchPelvis: for (const absoluteTilt of [2, 4, 6, 8, 10, 12, 15, 20, 25, 30]) for (const sign of [-1, 1])
        for (const toeScale of [1, .9, .8, .6, .4, 0]) for (const spreadDegrees of [0, 1, 2, 4]) {
          const tilt = absoluteTilt * sign, candidate = clone(original), delta = qAx(X, tilt), inverse = qAx(X, -tilt);
          // Tilt the pelvic bone relative to the femoral heads. Compensating
          // hip rotations retain the leg orientations; distributed spinal
          // rotations avoid concentrating the entire change in one hinge.
          const pelvisLocal = original.local.pelvis;
          candidate.rootQ.multiply(pelvisLocal.clone().multiply(delta).multiply(pelvisLocal.clone().invert())).normalize();
          for (const side of ['L', 'R']) {
            candidate.local['thigh_' + side].premultiply(inverse)
              .premultiply(qAx(Z, (side === 'L' ? 1 : -1) * spreadDegrees)).normalize();
            candidate.local['toes_' + side].slerp(new THREE.Quaternion(), 1 - toeScale).normalize();
          }
          for (const segment of ['lumA', 'lumB', 'thA']) candidate.local[segment].premultiply(qAx(X, -tilt / 3)).normalize();
          let world = A.computeWorld(candidate.local, candidate.rootQ, candidate.rootP);
          candidate.rootP.add(referenceWorld.thigh_L.p.clone().sub(world.thigh_L.p));
          world = A.computeWorld(candidate.local, candidate.rootQ, candidate.rootP);
          if (referenceWorld.thC.q.angleTo(world.thC.q) > 3 * Math.PI / 180 || lowerHit(candidate)) continue;
          source = candidate;
          lowerAdjustment = { kind: 'pelvic-spinal-bone-clearance', pelvicTiltDegrees: tilt,
            spinalCounterRotationDegrees: -tilt / 3, toeHingeScale: toeScale, bilateralHipAbductionDegrees: spreadDegrees,
            note: 'Для анимации наклон таза и сгибание спины согласованы по реальным костным поверхностям. Это адаптированный геометрический вариант исходной позы, без расчёта динамического равновесия.' };
          break searchPelvis;
        }
    }
    if (!lowerAdjustment) throw new Error('Could not clear the actual lower-limb bones without a large posture change: ' + id);
  }
  if (id === 'fold' && lowerAdjustment) {
    const followed = followFoldShins({ A, THREE, original, conf: source });
    source = followed.conf;
    lowerAdjustment.gripTargetsFollowShins = true;
    lowerAdjustment.gripAdjustment = followed.adjustment;
    lowerAdjustment.note += ' ' + followed.adjustment.note;
  }
  const arms = armEndpointVariant(id, source);
  if (!lowerAdjustment) return arms;
  return { conf: arms.conf, adjustment: arms.adjustment
    ? { kind: 'multiple-animation-adaptations', adjustments: [lowerAdjustment, arms.adjustment],
      note: 'Для анимации адаптированы углы ног и положение рук. Исходная статическая поза и её оценки нагрузки сохранены отдельно.' }
    : lowerAdjustment };
}
for (const [id, c] of Object.entries(raw)) {
  const source = encode(c), originalSha256 = sha(JSON.stringify(source));
  let variant = null;
  // Only the coarse private probe may reuse rounded optimizer coordinates.
  // Candidate authoring and release must derive the exact same full-precision
  // endpoints; decode/encode cache roundtrips can move Float32 minima by nm.
  if (probeAll && endpointCache.candidates[id]?.originalSha256 === originalSha256 && collision(c)) {
    try {
      const record = endpointCache.candidates[id], candidate = fromData(record.conf);
      if (candidate.rootP.toArray().every(n => Number.isFinite(n) && Math.abs(n) <= 10) && !collision(candidate))
        variant = { conf: candidate, adjustment: record.adjustment };
    } catch {}
  }
  if (!variant) variant = endpointVariant(id, c);
  const posed = variant.conf;
  const freshHit = collision(posed);
  if (freshHit) throw new Error('Endpoint candidate failed its fresh actual geometry/motion check: ' + id + ': ' + JSON.stringify(freshHit));
  if (variant.adjustment && !variant.adjustment.note) {
    variant.adjustment.note = variant.adjustment.kind === 'bilateral-elbow-plane'
      ? 'Для анимации изменена плоскость сгибания локтей; цели запястий и ориентация кистей сохранены.'
      : variant.adjustment.kind === 'bilateral-hip-abduction'
        ? 'Для анимации ноги немного разведены, чтобы реальные костные поверхности не пересекались.'
        : 'Для анимации положение рук уточнено по реальным костным поверхностям. Это геометрическая адаптация исходной позы; сила и давление контакта не рассчитаны.';
  }
  endpointCache.candidates[id] = { originalSha256, conf: encode(posed), adjustment: variant.adjustment };
  fs.writeFileSync(endpointCacheFile, JSON.stringify(endpointCache));
  const before = floor.minimum(posed, id).minimumY;
  posed.rootP.y += CLEARANCE - before; raw[id] = posed;
  endpointMetadata[id] = { sourceLocal: source.local, sourceRoot: source.root, sourceRootP: source.rootP,
    groundOffsetM: CLEARANCE - before, kinematicAdjustment: variant.adjustment };
  if (variant.adjustment) console.log('Animation endpoint adapted', id, JSON.stringify(variant.adjustment));
}
const endpoints = Object.fromEntries(Object.entries(raw).map(([id, c]) => [id, { ...encode(c), ...endpointMetadata[id] }]));
function collision(c) {
  collisionStates++;
  const motionKey = JSON.stringify([c.local.fore_L.toArray(), c.local.fore_R.toArray()]);
  if (!motionCache.has(motionKey)) {
    if (motionCache.size > 100000) motionCache.clear();
    motionCache.set(motionKey, motionGuard.inspect(c));
  }
  const motion = motionCache.get(motionKey);
  if (!motion.valid) { jointMotionRejectedCandidates++; return { scope: 'model-elbow-extension-limit', pair: motion.violations[0] }; }
  const key = JSON.stringify(A.SEG.map(s => c.local[s.name].toArray()));
  if (collisionCache.has(key)) { collisionCacheHits++; return collisionCache.get(key); }
  // Uniform rigid root rotations/translation do not alter self-intersection.
  const W = A.computeWorld(c.local, new THREE.Quaternion(), V(0, 0, 0));
  collisionEvaluations++;
  const a = arm.inspect(W, { maxPairs: 1 });
  if (a.colliding) { const hit = { scope: a.scope, pair: a.pairs[0] }; collisionCache.set(key, hit); return hit; }
  const f = armFoot.inspect(W, { maxPairs: 1 });
  if (f.colliding) { const hit = { scope: f.scope, pair: f.pairs[0] }; collisionCache.set(key, hit); return hit; }
  const low = lowerReport(c, W), report = low.colliding ? low : critical.inspectUpper(W);
  const hit = report.colliding ? { scope: report.scope, pair: report.pairs[0] } : null;
  if (collisionCache.size > 100000) collisionCache.clear();
  collisionCache.set(key, hit); return hit;
}
function releasePose(id, source) {
  if (!['struna-03', 'detali-07'].includes(id)) return null;
  const c = clone(source), originalRoot = c.rootQ.clone();
  // Solve the actual wrist target before opening the fingers. A locked hand
  // moved directly to neutral otherwise cuts the opposite thumb / own toes.
  c.rootQ.identity(); c.rootP.set(0, 0, 0);
  const sides = id === 'struna-03' ? ['L'] : ['L', 'R'];
  const distance = id === 'struna-03' ? .05 : .015;
  for (const side of sides) {
    const W = A.computeWorld(c.local, c.rootQ, c.rootP), sign = side === 'L' ? 1 : -1;
    const direction = V(sign, 0, id === 'struna-03' ? 1 : 0).normalize().multiplyScalar(distance).applyQuaternion(W.thC.q);
    const flex = armIK(c.local, c.rootQ, c.rootP, side,
      W['hand_' + side].p.clone().add(direction), W['fore_' + side].p.clone().add(direction));
    orientHand(c.local, c.rootQ, c.rootP, side, W['hand_' + side].q, flex);
    for (const segment of A.SEG) if (/^f[1-5]/.test(segment.name) && segment.name.endsWith('_' + side))
      c.local[segment.name].identity();
  }
  c.rootQ.copy(originalRoot); c.rootP.copy(source.rootP);
  return c;
}
function clearEdge(a, b) {
  if (identicalPose(a, b)) return !collision(a);
  // Dense end samples matter for interlaced fingers: a millimetre-scale cut
  // can occur long before a uniform 5% preview sample.
  const steps = probeAll ? 10 : 100;
  const samples = new Set([0, 1, ...Array.from({ length: steps + 1 }, (_, i) => i / steps)]);
  for (const u of probeAll ? [.001, .01, .025] : [.000001, .00001, .0001, .001, .003, .006, .015, .025]) { samples.add(u); samples.add(1 - u); }
  for (const u of [...samples].sort((x, y) => x - y)) {
    const hit = collision(mix(a, b, u));
    if (hit) { clearEdge.lastFailure = { u, ...hit }; return false; }
  }
  return true;
}
function strictClearEdge(a, b) {
  const { count, curve } = edgeSpec(a, b), keys = [];
  for (let j = 0; j <= count; j++) {
    const c = mix(a, b, curve(j / count));
    for (const s of A.SEG) c.local[s.name].set(...c.local[s.name].toArray().map(number)).normalize();
    const hit = collision(c);
    if (hit) { strictClearEdge.lastFailure = { u: j / count, ...hit }; return false; }
    keys.push(c);
  }
  for (let i = 0; i < count; i++) for (let j = 1; j < 20; j++) {
    const hit = collision(mix(keys[i], keys[i + 1], j / 20));
    if (hit) { strictClearEdge.lastFailure = { u: (i + j / 20) / count, ...hit }; return false; }
  }
  return true;
}
const toeResults = new WeakMap();
function toeEdgeClear(a, b, oracle = toeOracle) {
  if (!toeResults.has(oracle)) toeResults.set(oracle, new Map());
  const cache = toeResults.get(oracle);
  const check = local => {
    const key = JSON.stringify([local.toes_L.toArray(), local.toes_R.toArray()]);
    if (cache.has(key)) return cache.get(key);
    const clear = !oracle.inspect({ local }).colliding;
    if (cache.size > 100000) cache.clear(); cache.set(key, clear); return clear;
  };
  const { count, curve } = edgeSpec(a, b), keys = [];
  for (let i = 0; i <= count; i++) {
    const local = Object.fromEntries(['toes_L', 'toes_R'].map(s => {
      const q = transitionSlerp(a.local[s].clone(), b.local[s], curve(i / count));
      q.set(...q.toArray().map(number)).normalize(); return [s, q];
    }));
    if (!check(local)) return false; keys.push(local);
  }
  for (let i = 0; i < count; i++) for (let j = 1; j < 20; j++) {
    const local = Object.fromEntries(['toes_L', 'toes_R'].map(s => [s, transitionSlerp(keys[i][s].clone(), keys[i + 1][s], j / 20)]));
    if (!check(local)) return false;
  }
  return true;
}
function withdrawArms(source, sides, distance, vector, openFingers) {
  const c = clone(source), rootQ = c.rootQ.clone(), rootP = c.rootP.clone();
  c.rootQ.identity(); c.rootP.set(0, 0, 0);
  for (const side of sides) {
    const W = A.computeWorld(c.local, c.rootQ, c.rootP), sign = side === 'L' ? 1 : -1;
    const direction = V(vector[0] * sign, vector[1], vector[2]).normalize().multiplyScalar(distance).applyQuaternion(W.thC.q);
    const flex = armIK(c.local, c.rootQ, c.rootP, side,
      W['hand_' + side].p.clone().add(direction), W['fore_' + side].p.clone().add(direction));
    orientHand(c.local, c.rootQ, c.rootP, side, W['hand_' + side].q, flex);
    if (openFingers) for (const segment of A.SEG) if (/^f[1-5]/.test(segment.name) && segment.name.endsWith('_' + side)) c.local[segment.name].identity();
  }
  c.rootQ.copy(rootQ); c.rootP.copy(rootP); return c;
}
function chooseRelease(id, source, target) {
  if (clearEdge(source, target)) return null;
  const prescribed = releasePose(id, source);
  if (prescribed && clearEdge(source, prescribed) && clearEdge(prescribed, target)) return prescribed;
  for (const openFingers of [false, true]) for (const sides of [['L', 'R'], ['L'], ['R']])
    for (const distance of [.02, .04, .06, .08, .12])
      for (const vector of [[1, 0, 0], [1, 0, 1], [1, 0, -1], [1, 1, 0], [1, -1, 0], [0, 0, 1], [0, 0, -1], [0, 1, 0]]) {
        const release = withdrawArms(source, sides, distance, vector, openFingers);
        if (!collision(release) && clearEdge(source, release) && clearEdge(release, target)) {
          console.log('Release refinement', id, JSON.stringify({ openFingers, sides, distance, vector }));
          return release;
        }
      }
  throw new Error('No nonintersecting grip-release path found for ' + id);
}
function unfoldedThumbTips(input) {
  const c = clone(input);
  for (const side of ['L', 'R']) for (const base of ['f1a', 'f1b']) c.local[base + '_' + side].identity();
  return c;
}
function openArms(input, abduction = 90, elbow = 20) {
  const c = clone(input);
  for (const side of ['L', 'R']) {
    setSide(c.local, side, 'hum', qAx(Z, abduction));
    setSide(c.local, side, 'fore', qAx(X, -elbow));
    setSide(c.local, side, 'ulna', new THREE.Quaternion());
    setSide(c.local, side, 'hand', new THREE.Quaternion());
    for (const s of A.SEG) if (/^f[1-5]/.test(s.name) && s.name.endsWith('_' + side)) c.local[s.name].identity();
    setSide(c.local, side, 'f1m', qAx(X, -35));
  }
  return c;
}
// HALF_CLAVICLE_WAYPOINT_BEGIN
function halfClavicleFirstWaypoint(open, wideHub) {
  const c = clone(open);
  for (const side of ['L', 'R']) c.local['clav_' + side].copy(wideHub.local['clav_' + side]);
  return c;
}
// HALF_CLAVICLE_WAYPOINT_END
function releaseStages(id) {
  const source = raw[id];
  let withdrawalAttempts = 0;
  // D7's hands start together. Separating intact hands before opening either
  // thumb avoids the opposite-thumb collision of an in-place release.
  const thumbSource = id === 'detali-07' ? withdrawArms(source, ['L', 'R'], .01, [1, 0, 0], false) : source;
  const thumbRelease = /^detali-(01|03|04|05|06|07|08)$/.test(id)
    ? findThumbRelease({ A, THREE, conf: thumbSource, oracle: thumbOracle,
      samplesPerEdge: probeAll ? 100 : 1000, accept: c => !collision(c) }) : null;
  for (const abduction of [90, 110, 70, 130]) for (const elbow of [20, 45, 0, 75]) {
    const open = openArms(source, abduction, elbow), wideHub = openArms(hub, abduction, elbow);
    const openHit = id === 'struna-05' ? null : collision(open), hubHit = collision(wideHub);
    if (openHit || hubHit) { console.log('Open-arm waypoint rejected', id, abduction, elbow, JSON.stringify(openHit || hubHit)); continue; }
    if (!clearEdge(wideHub, hub)) {
      console.log('Open-arm body path rejected', id, abduction, elbow, JSON.stringify(clearEdge.lastFailure)); continue;
    }
    if (id === 'struna-05') {
      const special = prepareSpiderRelease({ A, THREE, conf: source, collision, clearEdge, openTarget: wideHub });
      if (special) return { exit: special, wideHub };
      continue;
    }
    let bodyGuides = [];
    if (id === 'half') {
      // The scapula and Th3 share the thoracic frame. Relaxing the clavicles
      // first changes their relative path; moving the root/spine cannot do so.
      const first = halfClavicleFirstWaypoint(open, wideHub);
      if (collision(first) || !clearEdge(open, first) || !clearEdge(first, wideHub)) continue;
      bodyGuides = [first];
    } else if (!clearEdge(open, wideHub)) {
      if (id === 'handstand') {
        const detour = findHipFlexionDetour({ A, THREE, from: open, to: wideHub, oracle: hipOracle,
          samplesPerEdge: probeAll ? 100 : 1000, accept: c => !collision(c) });
        if (detour) {
          const chain = [open, ...detour.stages, wideHub];
          if (chain.slice(1).every((c, i) => clearEdge(chain[i], c))) bodyGuides = detour.stages;
        }
      }
      if (!bodyGuides.length) {
        console.log('Open-arm body path rejected', id, abduction, elbow, JSON.stringify(clearEdge.lastFailure)); continue;
      }
    }
    const exit = stages => [...stages, ...bodyGuides];
    if (id === 'bow') {
      const special = prepareBowRelease({ A, THREE, conf: source, collision, clearEdge, openTarget: open });
      if (special) return { exit: exit([...special, open]), wideHub };
    }
    if (thumbRelease) {
      const prefix = id === 'detali-07' ? [thumbSource] : [];
      const chain = [source, ...prefix, ...thumbRelease.stages, open];
      if (chain.slice(1).every((c, i) => clearEdge(chain[i], c)))
        return { exit: exit([...prefix, ...thumbRelease.stages, open]), wideHub };
    }
    const tips = unfoldedThumbTips(source);
    if (!collision(tips) && clearEdge(source, tips) && clearEdge(tips, open)) return { exit: exit([tips, open]), wideHub };
    for (const sides of [['L', 'R'], ['L'], ['R']]) for (const distance of [.02, .04, .06, .08, .12])
      for (const vector of [[1, 0, 0], [1, 0, 1], [1, 0, -1], [1, 1, 0], [1, -1, 0], [0, 0, 1], [0, 0, -1], [0, 1, 0]]) {
        if (probeAll && withdrawalAttempts++ >= 48)
          throw new Error('Private coarse probe withdrawal budget exhausted: ' + id + ': ' + JSON.stringify(clearEdge.lastFailure));
        const withdrawn = withdrawArms(source, sides, distance, vector, false), straight = unfoldedThumbTips(withdrawn);
        if (!collision(withdrawn) && !collision(straight) && clearEdge(source, withdrawn)
            && clearEdge(withdrawn, straight) && clearEdge(straight, open)) {
          console.log('Staged release', id, JSON.stringify({ sides, distance, vector, abduction, elbow }));
          return { exit: exit([withdrawn, straight, open]), wideHub };
        }
      }
  }
  throw new Error('Unable to prepare staged nonintersecting arm release: ' + id);
}
if (probeAll) {
  const results = [], candidates = {};
  for (const pose of catalog) {
    if (pose.id === 'bow' && process.argv.includes('--skip-bow')) {
      results.push({ id: pose.id, status: 'skipped', reason: 'Dedicated grip-release repair in progress' });
      console.log('PROBE', pose.id, 'skipped'); continue;
    }
    try {
      const stage = releaseStages(pose.id), sequence = [raw[pose.id], ...stage.exit, stage.wideHub, hub];
      candidates['release-' + pose.id] = { id: 'release-' + pose.id, from: pose.id, to: '__neutral',
        duration: sequence.length - 1,
        phases: [{ t0: 0, t1: sequence.length - 1, label: 'Private coarse candidate only' }],
        frames: sequence.map((c, i) => encode(c, i)) };
      results.push({ id: pose.id, status: 'coarse-candidate', waypointCount: sequence.length });
      console.log('PROBE', pose.id, 'coarse candidate', sequence.length, 'waypoints');
    } catch (error) {
      results.push({ id: pose.id, status: 'blocked', error: String(error.message), lastFailure: clearEdge.lastFailure || null });
      console.log('PROBE', pose.id, 'BLOCKED', String(error.message));
    }
    fs.writeFileSync('/tmp/fysm-transition-probe.json', JSON.stringify({ schemaVersion: 1, verified: false,
      kind: 'private-coarse-transition-probe', sourceHashes, segNames: A.SEG.map(s => s.name), poses: catalog,
      endpoints, tracks: candidates, results, complete: false }));
  }
  fs.writeFileSync('/tmp/fysm-transition-probe.json', JSON.stringify({ schemaVersion: 1, verified: false,
    kind: 'private-coarse-transition-probe', sourceHashes, segNames: A.SEG.map(s => s.name), poses: catalog,
    endpoints, tracks: candidates, results, complete: true,
    sourcesFrozen: JSON.stringify(sourceHashes) === JSON.stringify(hashes()),
    limitations: ['Private coarse candidate collection only. Dense decoded motion, rendered floor and full production verification have not run.'] }));
  contacts.dispose();
  console.log('PRIVATE PROBE COMPLETE:', results.filter(r => r.status === 'coarse-candidate').length,
    'candidates;', results.filter(r => r.status === 'blocked').length, 'blocked;', results.filter(r => r.status === 'skipped').length,
    'skipped. No production packet was written.');
  process.exit(0);
}
const tracks = {}, planningFailures = [], neighbourFallbacks = [];
function progress(stage, current, interval = null) {
  fs.writeFileSync('/tmp/fysm-transition-progress.json', JSON.stringify({ verified: false,
    stage, current, interval, completedTracks: Object.keys(tracks), planningFailures,
    collisionStates, collisionEvaluations, collisionCacheHits, lowerEvaluations, lowerCacheHits,
    fullSurfaceEvaluations: floor.evaluations, updatedAt: new Date().toISOString() }));
}
function fitSavedTrackFrames() {
  for (const track of Object.values(tracks)) {
    progress('fit-rendered-floor-keys', track.id);
    for (let i = 1; i < track.frames.length - 1; i++) {
      const frame = track.frames[i];
      const c = fromData(frame);
      c.rootP.y += CLEARANCE - floor.minimum(c, track.groundModelId).minimumY;
      frame.rootP = c.rootP.toArray().map(number);
    }
    // Shared endpoints were already placed from their actual surfaces. The
    // saved normalized quaternions can change a Float32 minimum by nanometres;
    // never independently move the same endpoint for each incident track.
    track.frames[0] = { ...endpoints[track.from], t: 0 };
    track.frames[track.frames.length - 1] = { ...endpoints[track.to], t: track.duration };
  }
}
// CANDIDATE_VALIDATOR_BEGIN
function validateRawPlan(saved, { A, THREE, catalog, endpoints, sourceHashes, sourceNames }) {
  const object = o => !!o && typeof o === 'object' && !Array.isArray(o);
  const exact = (a, b) => JSON.stringify(a) === JSON.stringify(b);
  if (!object(saved) || saved.schemaVersion !== 1 || saved.verified !== false
      || saved.kind !== 'private-bone-motion-plan' || saved.complete !== true || saved.floorVerified !== false
      || saved.sourcesFrozen !== true || !Array.isArray(saved.planningFailures) || saved.planningFailures.length
      || !object(saved.sourceHashes) || !Array.isArray(sourceNames)
      || !exact(Object.keys(saved.sourceHashes).sort(), [...sourceNames].sort())
      || sourceNames.some(name => typeof saved.sourceHashes[name] !== 'string' || !/^[a-f0-9]{64}$/.test(saved.sourceHashes[name])
        || name !== 'transition_build.mjs' && saved.sourceHashes[name] !== sourceHashes[name])
      || !exact(saved.segNames, A.SEG.map(s => s.name)) || !exact(saved.poses, catalog)
      || !exact(saved.endpoints, endpoints) || Object.hasOwn(saved, 'checks') || Object.hasOwn(saved, 'jointMotion')
      || !object(saved.tracks) || Object.values(saved.tracks).some(t => !object(t)
        || t.groundModelId !== ([t.from, t.to].includes('detali-08') ? 'detali-08' : t.from)
        || !Array.isArray(t.frames) || t.frames.some(f => !object(f) || Object.hasOwn(f, 'floorLiftM')))
      || !Array.isArray(saved.neighbourFallbacks)) return false;
  if (Object.hasOwn(saved, 'coordinateOrigin')) {
    const origin = saved.coordinateOrigin;
    const allowed = origin?.kind === 'bow-half-motion-repair-replanning'
      ? ['transition_build.mjs', 'transition_bow_release.mjs', 'transition_collision.mjs']
      : ['muscle_surface_render.js', 'practice/poses.json', 'transition_build.mjs'];
    if (!object(origin) || origin.schemaVersion !== 1
        || !['surface-context-root-y-rebase', 'partial-coordinate-replanning', 'bow-half-motion-repair-replanning'].includes(origin.kind)
        || !/^[a-f0-9]{64}$/.test(origin.oldArchiveSha256) || !object(origin.oldSourceHashes)
        || !exact(Object.keys(origin.oldSourceHashes).sort(), [...sourceNames].sort())
        || Object.values(origin.oldSourceHashes).some(hash => typeof hash !== 'string' || !/^[a-f0-9]{64}$/.test(hash))
        || (origin.kind === 'surface-context-root-y-rebase' ? origin.migrationProducerSha256 : origin.replanningProducerSha256)
          !== saved.sourceHashes['transition_build.mjs']
        || !exact(origin.allowedChangedFiles, allowed)
        || !Array.isArray(origin.changedSourceFiles) || new Set(origin.changedSourceFiles).size !== origin.changedSourceFiles.length
        || origin.changedSourceFiles.some(name => !allowed.includes(name))
        || !exact(origin.changedSourceFiles, sourceNames.filter(name => origin.oldSourceHashes[name] !== saved.sourceHashes[name]).sort())) return false;
    if (origin.kind === 'surface-context-root-y-rebase') {
      if (origin.scope !== 'endpoint-rootY-and-grounding-metadata-only' || origin.interiorCoordinatesPreserved !== true
          || origin.endpointCount !== Object.keys(endpoints).length) return false;
    } else if (origin.kind === 'partial-coordinate-replanning') {
      const list = xs => Array.isArray(xs) && xs.every(x => typeof x === 'string') && new Set(xs).size === xs.length;
      const requests = [{ id: 'release-centering-06', kind: 'release', from: 'centering-06', to: '__neutral' },
        { id: 'neighbour-centering-05-centering-06', kind: 'neighbour', from: 'centering-05', to: 'centering-06' },
        { id: 'neighbour-centering-06-centering-07', kind: 'neighbour', from: 'centering-06', to: 'centering-07' }];
      if (origin.scope !== 'preserve-unaffected-coordinates-replan-changed-endpoints'
          || origin.endpointCount !== Object.keys(endpoints).length || origin.allCanonicalEndpointRowsReplaced !== true
          || origin.retainedInteriorCoordinatesPreserved !== true || !Array.isArray(origin.endpointGroundingRebases)
          || !exact(origin.affectedPoseIds, ['centering-06']) || !exact(origin.regeneratedRequests, requests)
          || !list(origin.retainedTrackIds) || !list(origin.discardedTrackIds) || !list(origin.regeneratedTrackIds)
          || !Number.isInteger(origin.oldTrackCount) || origin.oldTrackCount !== origin.retainedTrackIds.length + origin.discardedTrackIds.length
          || origin.retainedTrackIds.some(id => origin.discardedTrackIds.includes(id) || origin.regeneratedTrackIds.includes(id))
          || !exact([...origin.retainedTrackIds, ...origin.regeneratedTrackIds].sort(), Object.keys(saved.tracks).sort())
          || !Array.isArray(origin.regeneratedFallbacks)) return false;
      if (!exact(origin.endpointGroundingRebases.map(r => r?.id).sort(), Object.keys(endpoints).sort())
          || origin.endpointGroundingRebases.some(r => !object(r) || !Object.hasOwn(endpoints, r.id)
            || ['previousRootY', 'currentRootY', 'previousSourceRootY', 'currentSourceRootY', 'previousGroundOffsetM', 'currentGroundOffsetM'].some(k => !Number.isFinite(r[k]))
            || r.currentRootY !== endpoints[r.id].rootP[1] || r.currentSourceRootY !== endpoints[r.id].sourceRootP[1]
            || r.currentGroundOffsetM !== endpoints[r.id].groundOffsetM)) return false;
      for (const id of origin.retainedTrackIds) if ([saved.tracks[id].from, saved.tracks[id].to].includes('centering-06')) return false;
      for (const request of requests) {
        const present = origin.regeneratedTrackIds.includes(request.id), track = saved.tracks[request.id];
        const fallbacks = origin.regeneratedFallbacks.filter(f => f?.from === request.from && f?.to === request.to);
        if (present ? !track || track.from !== request.from || track.to !== request.to || fallbacks.length
          : request.kind === 'release' || fallbacks.length !== 1) return false;
      }
      if (origin.regeneratedTrackIds.some(id => !requests.some(r => r.id === id))
          || origin.regeneratedFallbacks.some(f => typeof f?.reason !== 'string'
            || !requests.some(r => r.kind === 'neighbour' && r.from === f.from && r.to === f.to))
          || !exact(origin.regeneratedFallbacks, saved.neighbourFallbacks.filter(f => [f?.from, f?.to].includes('centering-06')))) return false;
    } else {
      const requests = [{ id: 'release-bow', kind: 'release', from: 'bow', to: '__neutral' },
        { id: 'release-half', kind: 'release', from: 'half', to: '__neutral' }];
      const ids = requests.map(r => r.id);
      const list = xs => Array.isArray(xs) && xs.every(x => typeof x === 'string') && new Set(xs).size === xs.length;
      if (origin.scope !== 'unchanged-endpoints-replan-bow-half-releases'
          || origin.endpointCount !== Object.keys(endpoints).length || origin.endpointRowsPreserved !== true
          || origin.retainedCoordinateTracksPreserved !== true || !exact(origin.affectedPoseIds, ['bow', 'half'])
          || !exact(origin.regeneratedRequests, requests) || !exact(origin.discardedTrackIds, [...ids].sort())
          || !exact(origin.regeneratedTrackIds, [...ids].sort()) || !list(origin.retainedTrackIds)
          || origin.retainedTrackIds.some(id => ids.includes(id))
          || !Number.isInteger(origin.oldTrackCount) || origin.oldTrackCount !== origin.retainedTrackIds.length + ids.length
          || !exact([...origin.retainedTrackIds, ...ids].sort(), Object.keys(saved.tracks).sort())
          || !exact(origin.regeneratedFallbacks, []) || !exact(origin.retainedNeighbourFallbacks, saved.neighbourFallbacks)) return false;
      for (const id of origin.retainedTrackIds)
        if (['bow', 'half'].includes(saved.tracks[id].from) || ['bow', 'half'].includes(saved.tracks[id].to)) return false;
      for (const request of requests) {
        const track = saved.tracks[request.id];
        if (!track || track.from !== request.from || track.to !== request.to) return false;
      }
    }
  }
  // This facade checks only schema/graph structure. No old verification flag
  // or collision result is imported: all final decoded states are checked
  // again against the current geometry, motion guard and rendered surfaces.
  const structured = createTransitions(THREE, A, { schemaVersion: 1, verified: true, kind: 'kinematic-demonstration',
    sourceHashes, segNames: saved.segNames, poses: saved.poses, endpoints: saved.endpoints, tracks: saved.tracks,
    checks: { oppositeArms: true, armsFeet: true, floor: true, criticalRegions: true, elbowMotion: true } });
  if (!structured.valid) return false;
  // Keep the original angular sampling contract before applying twenty fresh
  // substeps. Unit endpoint quaternions alone cannot establish key density.
  for (const track of Object.values(saved.tracks)) for (let i = 1; i < track.frames.length; i++) {
    const a = track.frames[i - 1], b = track.frames[i];
    for (const name of ['root', ...saved.segNames]) {
      const qa = name === 'root' ? a.root : a.local[name], qb = name === 'root' ? b.root : b.local[name];
      const cosine = Math.abs(qa.reduce((sum, x, j) => sum + x * qb[j], 0) / (Math.hypot(...qa) * Math.hypot(...qb)));
      if (2 * Math.acos(Math.min(1, cosine)) > .040001) return false;
    }
  }
  return true;
}
// CANDIDATE_VALIDATOR_END
// COORDINATE_MIGRATION_BEGIN
function migrateCoordinatePlan(saved, { A, THREE, catalog, endpoints, sourceHashes, sourceNames, oldArchiveSha256 }) {
  const allowedChangedFiles = ['muscle_surface_render.js', 'practice/poses.json', 'transition_build.mjs'];
  const fail = message => { throw new Error('Coordinate migration rejected: ' + message + '; regenerate with --geometry-only'); };
  if (!/^[a-f0-9]{64}$/.test(oldArchiveSha256)) fail('invalid original archive hash');
  if (!validateRawPlan(saved, { A, THREE, catalog, endpoints: saved?.endpoints,
    sourceHashes: saved?.sourceHashes, sourceNames })) fail('original coordinate input is malformed');
  const changedSourceFiles = sourceNames.filter(name => saved.sourceHashes[name] !== sourceHashes[name]).sort();
  if (changedSourceFiles.some(name => !allowedChangedFiles.includes(name))) fail('an unsupported physical dependency changed');
  if (JSON.stringify(Object.keys(saved.endpoints).sort()) !== JSON.stringify(Object.keys(endpoints).sort())) fail('endpoint inventory changed');
  const sameQuaternion = (a, b) => {
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== 4 || b.length !== 4
        || [...a, ...b].some(x => !Number.isFinite(x))) return false;
    const na = Math.hypot(...a), nb = Math.hypot(...b);
    if (na < 1e-12 || nb < 1e-12) return false;
    return Math.min(Math.hypot(...a.map((x, i) => x / na - b[i] / nb)),
      Math.hypot(...a.map((x, i) => x / na + b[i] / nb))) < 1e-8;
  };
  const sameXZ = (a, b) => Array.isArray(a) && Array.isArray(b) && a.length === 3 && b.length === 3
    && [...a, ...b].every(Number.isFinite) && Math.abs(a[0] - b[0]) < 1e-8 && Math.abs(a[2] - b[2]) < 1e-8;
  const semantic = endpoint => {
    const copy = { ...endpoint };
    for (const key of ['local', 'root', 'rootP', 'sourceLocal', 'sourceRoot', 'sourceRootP', 'groundOffsetM']) delete copy[key];
    return JSON.stringify(copy);
  };
  const rootYChanges = [];
  for (const [id, current] of Object.entries(endpoints)) {
    const old = saved.endpoints[id];
    if (!sameQuaternion(old.root, current.root) || !sameQuaternion(old.sourceRoot, current.sourceRoot)
        || !A.SEG.every(s => sameQuaternion(old.local?.[s.name], current.local?.[s.name])
          && sameQuaternion(old.sourceLocal?.[s.name], current.sourceLocal?.[s.name]))
        || !sameXZ(old.rootP, current.rootP) || !sameXZ(old.sourceRootP, current.sourceRootP))
      fail(id + ' intrinsic/root orientation or horizontal position changed');
    if (!Number.isFinite(old.groundOffsetM) || !Number.isFinite(current.groundOffsetM)
        || semantic(old) !== semantic(current)) fail(id + ' semantic endpoint metadata changed');
    rootYChanges.push({ id, previousRootY: old.rootP[1], currentRootY: current.rootP[1],
      previousSourceRootY: old.sourceRootP[1], currentSourceRootY: current.sourceRootP[1],
      previousGroundOffsetM: old.groundOffsetM, currentGroundOffsetM: current.groundOffsetM });
  }
  const migrated = JSON.parse(JSON.stringify(saved));
  migrated.sourceHashes = Object.fromEntries(sourceNames.map(name => [name, sourceHashes[name]]));
  migrated.endpoints = JSON.parse(JSON.stringify(endpoints));
  for (const track of Object.values(migrated.tracks)) {
    track.frames[0] = { ...migrated.endpoints[track.from], t: 0 };
    track.frames[track.frames.length - 1] = { ...migrated.endpoints[track.to], t: track.duration };
  }
  delete migrated.planProvenance;
  migrated.coordinateOrigin = { schemaVersion: 1, kind: 'surface-context-root-y-rebase', oldArchiveSha256,
    oldSourceHashes: { ...saved.sourceHashes }, migrationProducerSha256: sourceHashes['transition_build.mjs'],
    allowedChangedFiles, changedSourceFiles, scope: 'endpoint-rootY-and-grounding-metadata-only',
    interiorCoordinatesPreserved: true, endpointCount: Object.keys(endpoints).length };
  if (!validateRawPlan(migrated, { A, THREE, catalog, endpoints, sourceHashes, sourceNames })) fail('rebased structure/density is invalid');
  return { plan: migrated, report: { changedSourceFiles, rootYChanges,
    preservedInteriorFrames: Object.values(migrated.tracks).reduce((n, t) => n + t.frames.length - 2, 0) } };
}
// COORDINATE_MIGRATION_END
// PARTIAL_REPLAN_SELECTOR_BEGIN
function selectPartialCoordinateReplan(saved, { A, THREE, catalog, endpoints, sourceHashes, sourceNames, oldArchiveSha256 }) {
  const fail = text => { throw new Error('Partial coordinate replanning rejected: ' + text + '; use --geometry-only'); };
  const allowedChangedFiles = ['muscle_surface_render.js', 'practice/poses.json', 'transition_build.mjs'];
  if (!/^[a-f0-9]{64}$/.test(oldArchiveSha256) || !validateRawPlan(saved, { A, THREE, catalog,
    endpoints: saved?.endpoints, sourceHashes: saved?.sourceHashes, sourceNames })) fail('invalid original coordinate context');
  const changedSourceFiles = sourceNames.filter(name => saved.sourceHashes[name] !== sourceHashes[name]).sort();
  if (changedSourceFiles.some(name => !allowedChangedFiles.includes(name))) fail('unsupported physical dependency changed');
  if (JSON.stringify(Object.keys(saved.endpoints).sort()) !== JSON.stringify(Object.keys(endpoints).sort())) fail('endpoint inventory changed');
  const sameQ = (a, b) => {
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== 4 || b.length !== 4 || [...a, ...b].some(x => !Number.isFinite(x))) return false;
    const na = Math.hypot(...a), nb = Math.hypot(...b);
    return na > 1e-12 && nb > 1e-12 && Math.min(Math.hypot(...a.map((x, i) => x / na - b[i] / nb)),
      Math.hypot(...a.map((x, i) => x / na + b[i] / nb))) < 1e-8;
  };
  const sameXZ = (a, b) => Array.isArray(a) && Array.isArray(b) && a.length === 3 && b.length === 3
    && [...a, ...b].every(Number.isFinite) && Math.abs(a[0] - b[0]) < 1e-8 && Math.abs(a[2] - b[2]) < 1e-8;
  const semantic = ep => {
    const copy = { ...ep };
    for (const key of ['local', 'root', 'rootP', 'sourceLocal', 'sourceRoot', 'sourceRootP', 'groundOffsetM']) delete copy[key];
    return JSON.stringify(copy);
  };
  const changedEndpoints = Object.keys(endpoints).filter(id => {
    const a = saved.endpoints[id], b = endpoints[id];
    return !sameQ(a.root, b.root) || !sameQ(a.sourceRoot, b.sourceRoot)
      || !A.SEG.every(s => sameQ(a.local?.[s.name], b.local?.[s.name]) && sameQ(a.sourceLocal?.[s.name], b.sourceLocal?.[s.name]))
      || !sameXZ(a.rootP, b.rootP) || !sameXZ(a.sourceRootP, b.sourceRootP)
      || !Number.isFinite(a.groundOffsetM) || !Number.isFinite(b.groundOffsetM) || semantic(a) !== semantic(b);
  });
  const endpointGroundingRebases = Object.keys(endpoints).map(id => ({ id, previousRootY: saved.endpoints[id].rootP[1], currentRootY: endpoints[id].rootP[1],
    previousSourceRootY: saved.endpoints[id].sourceRootP[1], currentSourceRootY: endpoints[id].sourceRootP[1],
    previousGroundOffsetM: saved.endpoints[id].groundOffsetM, currentGroundOffsetM: endpoints[id].groundOffsetM }));
  if (changedEndpoints.includes('__neutral')) fail('neutral hub changed');
  // This recovery is deliberately limited to the measured C6 recalculation.
  if (JSON.stringify(changedEndpoints) !== JSON.stringify(['centering-06'])) fail('changed endpoints exceed the authorized C6 scope');
  for (const pose of catalog) {
    const track = saved.tracks['release-' + pose.id];
    if (!track || track.from !== pose.id || track.to !== '__neutral') fail('missing canonical release ' + pose.id);
  }
  const affected = new Set(changedEndpoints), retainedTracks = {}, discardedTrackIds = [];
  for (const [id, track] of Object.entries(saved.tracks)) {
    if (affected.has(track.from) || affected.has(track.to)) discardedTrackIds.push(id);
    else retainedTracks[id] = JSON.parse(JSON.stringify(track));
  }
  const requests = changedEndpoints.map(id => ({ id: 'release-' + id, kind: 'release', from: id, to: '__neutral' }));
  for (const pose of catalog) {
    if (pose.kind !== 'practice' || pose.stepIndex === 7) continue;
    const next = catalog.find(p => p.algorithmId === pose.algorithmId && p.stepIndex === pose.stepIndex + 1);
    if (next && (affected.has(pose.id) || affected.has(next.id)))
      requests.push({ id: 'neighbour-' + pose.id + '-' + next.id, kind: 'neighbour', from: pose.id, to: next.id });
  }
  const publicIds = new Set(catalog.map(p => p.id));
  if (saved.neighbourFallbacks.some(f => !f || !publicIds.has(f.from) || !publicIds.has(f.to) || typeof f.reason !== 'string'))
    fail('invalid original neighbour fallback');
  return { retainedTracks, retainedFallbacks: saved.neighbourFallbacks.filter(f => !affected.has(f.from) && !affected.has(f.to)),
    requests, preparePoseIds: [...new Set(requests.flatMap(r => [r.from, r.to]).filter(id => publicIds.has(id)))],
    origin: { schemaVersion: 1, kind: 'partial-coordinate-replanning', oldArchiveSha256, oldSourceHashes: { ...saved.sourceHashes },
      replanningProducerSha256: sourceHashes['transition_build.mjs'], allowedChangedFiles, changedSourceFiles,
      scope: 'preserve-unaffected-coordinates-replan-changed-endpoints', affectedPoseIds: changedEndpoints,
      endpointGroundingRebases,
      endpointCount: Object.keys(endpoints).length, allCanonicalEndpointRowsReplaced: true,
      retainedInteriorCoordinatesPreserved: true,
      oldTrackCount: Object.keys(saved.tracks).length, retainedTrackIds: Object.keys(retainedTracks).sort(),
      discardedTrackIds: discardedTrackIds.sort(), regeneratedRequests: requests,
      regeneratedTrackIds: [], regeneratedFallbacks: [] } };
}
// PARTIAL_REPLAN_SELECTOR_END
// MOTION_REPAIR_SELECTOR_BEGIN
function selectBowHalfMotionRepair(saved, { A, THREE, catalog, endpoints, sourceHashes, sourceNames, oldArchiveSha256 }) {
  const fail = text => { throw new Error('Bow/half motion repair rejected: ' + text + '; use --geometry-only'); };
  const allowedChangedFiles = ['transition_build.mjs', 'transition_bow_release.mjs', 'transition_collision.mjs'];
  if (!/^[a-f0-9]{64}$/.test(oldArchiveSha256) || !validateRawPlan(saved, { A, THREE, catalog,
    endpoints: saved?.endpoints, sourceHashes: saved?.sourceHashes, sourceNames })) fail('invalid original coordinate context');
  const changedSourceFiles = sourceNames.filter(name => saved.sourceHashes[name] !== sourceHashes[name]).sort();
  if (changedSourceFiles.some(name => !allowedChangedFiles.includes(name))) fail('unsupported physical dependency changed');
  if (JSON.stringify(saved.endpoints) !== JSON.stringify(endpoints)) fail('all 31 canonical endpoint rows must remain exact');
  for (const pose of catalog) {
    const track = saved.tracks['release-' + pose.id];
    if (!track || track.from !== pose.id || track.to !== '__neutral') fail('missing canonical release ' + pose.id);
  }
  const requests = [{ id: 'release-bow', kind: 'release', from: 'bow', to: '__neutral' },
    { id: 'release-half', kind: 'release', from: 'half', to: '__neutral' }];
  const replaced = new Set(requests.map(r => r.id)), retainedTracks = {};
  for (const [id, track] of Object.entries(saved.tracks)) {
    if (replaced.has(id)) continue;
    if (['bow', 'half'].includes(track.from) || ['bow', 'half'].includes(track.to)) fail('unexpected incident track ' + id);
    retainedTracks[id] = JSON.parse(JSON.stringify(track));
  }
  const publicIds = new Set(catalog.map(p => p.id));
  if (saved.neighbourFallbacks.some(f => !f || !publicIds.has(f.from) || !publicIds.has(f.to)
    || typeof f.reason !== 'string' || ['bow', 'half'].includes(f.from) || ['bow', 'half'].includes(f.to))) fail('invalid neighbour fallback inventory');
  return { retainedTracks, retainedFallbacks: JSON.parse(JSON.stringify(saved.neighbourFallbacks)), requests,
    preparePoseIds: ['bow', 'half'], origin: { schemaVersion: 1, kind: 'bow-half-motion-repair-replanning',
      oldArchiveSha256, oldSourceHashes: { ...saved.sourceHashes }, replanningProducerSha256: sourceHashes['transition_build.mjs'],
      allowedChangedFiles, changedSourceFiles, scope: 'unchanged-endpoints-replan-bow-half-releases',
      affectedPoseIds: ['bow', 'half'], endpointCount: Object.keys(endpoints).length, endpointRowsPreserved: true,
      retainedCoordinateTracksPreserved: true, oldTrackCount: Object.keys(saved.tracks).length,
      retainedTrackIds: Object.keys(retainedTracks).sort(), discardedTrackIds: [...replaced].sort(),
      regeneratedRequests: requests, regeneratedTrackIds: [], regeneratedFallbacks: [],
      retainedNeighbourFallbacks: JSON.parse(JSON.stringify(saved.neighbourFallbacks)) } };
}
// MOTION_REPAIR_SELECTOR_END
function plan(id, source, destination, release = null) {
  const target = raw[destination], start = raw[source];
  const groundModelId = [source, destination].includes('detali-08') ? 'detali-08' : source;
  const requestedKeys = release ? [start, ...(Array.isArray(release) ? release : [release]), target] : [start, target];
  const initialKeys = requestedKeys.filter((c, i) => !i || !identicalPose(c, requestedKeys[i - 1]));
  if (initialKeys.length === 1) initialKeys.push(target);
  const keys = [initialKeys[0]];
  for (let i = 1; i < initialKeys.length; i++) {
    const a = initialKeys[i - 1], b = initialKeys[i];
    if (!strictClearEdge(a, b)) {
      console.log('Fine path needs detour', id, i - 1, JSON.stringify(strictClearEdge.lastFailure));
      const pair = strictClearEdge.lastFailure?.pair;
      let focused = null;
      if (pair && /^(foot|toes)_/.test(pair.a) && /^(foot|toes)_/.test(pair.b)) {
        const names = new Set([boneData.parts[pair.aPart].name, boneData.parts[pair.bPart].name]);
        const indices = boneData.parts.flatMap((part, index) => /^(foot|toes)_[LR]$/.test(part.seg) && names.has(part.name) ? [index] : []);
        focused = createToeCollisionOracle(A, THREE, boneData, { sourcePartIndices: indices });
      }
      const detour = findTransitionDetour({ A, THREE, from: a, to: b,
        families: detourFamiliesForPair(A, pair),
        clearEdge: (x, y) => (!focused || toeEdgeClear(x, y, focused)) && toeEdgeClear(x, y)
          && clearEdge(x, y) && strictClearEdge(x, y), collision });
      if (!detour) throw new Error('No strictly checked joint-space detour: ' + id + ':' + (i - 1) + ': ' + JSON.stringify(strictClearEdge.lastFailure));
      keys.push(...detour);
      console.log('Prepared bounded joint detour', id, i - 1, detour.length, 'waypoints');
    }
    keys.push(b);
  }
  const specs = keys.slice(1).map((c, i) => edgeSpec(keys[i], c));
  const frames = [], phases = []; let offset = 0;
  for (let part = 0; part < keys.length - 1; part++) {
    const { duration, count, curve } = specs[part];
    phases.push({ t0: number(offset), t1: number(offset + duration),
      label: release && part === 0 ? 'Подготовка к переходу' : destination === '__neutral' ? 'Переход через раскрытое положение' : 'Переход к следующей асане',
      reverseLabel: release && part === 0 ? 'Переход к выбранному положению' : destination === '__neutral' ? 'Переход к выбранной форме' : 'Переход к предыдущей асане' });
    for (let j = part ? 1 : 0; j <= count; j++) {
      const c = mix(keys[part], keys[part + 1], curve(j / count));
      const at = collision(c);
      if (at) throw new Error(`Planned ${id} intersects at part ${part} u=${j / count}: ${JSON.stringify(at)}`);
      // Correct the root, never the skeleton or thickness of any tissue.
      if (!geometryOnly) c.rootP.y += CLEARANCE - floor.minimum(c, groundModelId).minimumY;
      frames.push(encode(c, offset + duration * j / count));
    }
    offset = number(offset + duration);
  }
  // Share bit-identical hub/pose endpoints despite separately solved envelopes.
  frames[0] = { ...endpoints[source], t: 0 };
  frames[frames.length - 1] = { ...endpoints[destination], t: offset };
  const mergedPhases = [];
  for (const phase of phases) {
    const previous = mergedPhases.at(-1);
    if (previous && previous.label === phase.label && previous.reverseLabel === phase.reverseLabel
        && Math.abs(previous.t1 - phase.t0) < 1e-8) previous.t1 = phase.t1;
    else mergedPhases.push({ ...phase });
  }
  tracks[id] = { id, from: source, to: destination, duration: offset, phases: mergedPhases, frames, groundModelId };
  progress('bone-motion-planning', id);
}
function planNeighbour(source, destination) {
  if (!prepared[source] || !prepared[destination]) throw new Error('Missing current release preparation');
  try {
  const clean = clearEdge(raw[source], raw[destination]);
  if (clean) plan('neighbour-' + source + '-' + destination, source, destination);
  else {
    const keys = [...prepared[source].exit, ...prepared[destination].exit.toReversed()];
    const chain = [raw[source], ...keys, raw[destination]];
    if (chain.slice(1).some((c, i) => !clearEdge(chain[i], c))) {
      neighbourFallbacks.push({ from: source, to: destination,
        reason: 'A checked direct or released-neighbour shortcut was not established; the checked graph route is used.' });
      console.log('Neighbour uses prepared graph', source, destination); return false;
    }
    plan('neighbour-' + source + '-' + destination, source, destination, keys);
  }
  console.log('Planned neighbour', source, destination);
  } catch (error) {
    // This shortcut is optional: the already checked release graph still
    // connects these endpoints in both directions without inventing a direct
    // neighbour path. Mandatory pose releases above can never be skipped.
    neighbourFallbacks.push({ from: source, to: destination, reason: String(error.message) });
    console.log('Neighbour uses prepared graph after dense check', source, destination, String(error.message));
  }
  return Object.hasOwn(tracks, 'neighbour-' + source + '-' + destination);
}

const prepared = {};
let partialSelection = null, partialPrevious = null, partialPreviousBytes = null;
if (migratePlan) {
  const previousBytes = read(planSourceName), oldArchiveSha256 = sha(previousBytes);
  const previous = JSON.parse(gunzipSync(previousBytes));
  const result = migrateCoordinatePlan(previous, { A, THREE, catalog, endpoints, sourceHashes,
    sourceNames: originalInputs, oldArchiveSha256 });
  if (JSON.stringify(sourceHashes) !== JSON.stringify(hashes()) || sha(read(planSourceName)) !== oldArchiveSha256)
    throw new Error('Source context or coordinate input changed during migration');
  const backup = 'transitions/plan-origin-' + oldArchiveSha256 + '.json.gz';
  fs.writeFileSync(new URL(backup, import.meta.url), previousBytes);
  const nextBytes = (await import('node:zlib')).gzipSync(JSON.stringify(result.plan), { level: 9, mtime: 0 });
  fs.writeFileSync(new URL(planSourceName + '.next', import.meta.url), nextBytes);
  fs.renameSync(new URL(planSourceName + '.next', import.meta.url), new URL(planSourceName, import.meta.url));
  contacts.dispose();
  console.log(JSON.stringify({ coordinateMigrationOnly: true, verified: false, floorVerified: false,
    oldArchiveSha256, newArchiveSha256: sha(nextBytes), backup, ...result.report }));
  process.exit(0);
}
let reusedPlan = false;
const planProvenance = { mode: fromPlan ? 'unverified-coordinate-input' : 'regenerated-from-poses',
  candidateFile: candidateGeneration ? null : planSourceName,
  candidateSha256: candidateGeneration ? null : sourceHashes[planSourceName],
  originProducerSha256: sourceHashes['transition_build.mjs'],
  currentProducerSha256: sourceHashes['transition_build.mjs'], priorVerificationTrusted: false };
if (fromPlan) {
  const saved = JSON.parse(gunzipSync(read(planSourceName)));
  if (!validateRawPlan(saved, { A, THREE, catalog, endpoints, sourceHashes, sourceNames: originalInputs }))
    throw new Error('Untrusted motion-plan input is incomplete, malformed or has stale source geometry/endpoints');
  planProvenance.originProducerSha256 = saved.coordinateOrigin?.oldSourceHashes['transition_build.mjs']
    || saved.sourceHashes['transition_build.mjs'];
  Object.assign(tracks, saved.tracks);
  if (Array.isArray(saved.neighbourFallbacks)) neighbourFallbacks.push(...saved.neighbourFallbacks);
  fitSavedTrackFrames();
  // Final decoded keys, all twenty bone/motion substeps and rendered floor
  // are freshly audited below; no checkpoint receives release verification.
  reusedPlan = true; console.log('Loaded untrusted coordinate input:', Object.keys(tracks).length,
    'tracks; all floor fitting and final decoded guards still run.');
}
if (partialReplan || repairReplan) {
  partialPreviousBytes = read(planSourceName);
  partialPrevious = JSON.parse(gunzipSync(partialPreviousBytes));
  partialSelection = (repairReplan ? selectBowHalfMotionRepair : selectPartialCoordinateReplan)(partialPrevious, { A, THREE, catalog, endpoints, sourceHashes,
    sourceNames: originalInputs, oldArchiveSha256: sha(partialPreviousBytes) });
  Object.assign(tracks, partialSelection.retainedTracks);
  neighbourFallbacks.push(...partialSelection.retainedFallbacks);
  for (const track of Object.values(tracks)) {
    track.frames[0] = { ...endpoints[track.from], t: 0 };
    track.frames[track.frames.length - 1] = { ...endpoints[track.to], t: track.duration };
  }
  for (const id of partialSelection.preparePoseIds) {
    progress('prepare-current-partial-release', id);
    prepared[id] = releaseStages(id);
  }
  for (const request of partialSelection.requests) {
    if (request.kind === 'release') {
      const stage = prepared[request.from];
      plan(request.id, request.from, request.to, [...stage.exit, stage.wideHub]);
    } else planNeighbour(request.from, request.to);
  }
  reusedPlan = true;
}
if (!reusedPlan) for (const pose of catalog) {
  progress('prepare-release', pose.id);
  try {
    prepared[pose.id] = releaseStages(pose.id);
    plan('release-' + pose.id, pose.id, '__neutral', [...prepared[pose.id].exit, prepared[pose.id].wideHub]);
    console.log('Planned', pose.id, tracks['release-' + pose.id].frames.length, 'frames');
  } catch (error) {
    if (!geometryOnly) throw error;
    planningFailures.push({ id: pose.id, error: String(error.message) });
    console.log('MOTION PLAN BLOCKED', pose.id, String(error.message));
  }
}
if (deferredFloor && planningFailures.length) {
  // Attempt every mandatory endpoint so failures can be repaired together,
  // then stop before optional shortcuts or any rendered-floor release work.
  fs.writeFileSync('/tmp/fysm-transition-motion-plan.json', JSON.stringify({ schemaVersion: 1,
    verified: false, kind: 'private-bone-motion-plan', complete: false, floorVerified: false,
    sourceHashes, segNames: A.SEG.map(s => s.name), poses: catalog, endpoints, tracks,
    checks: { oppositeArms: true, armsFeet: true, floor: false, criticalRegions: true, elbowMotion: true },
    catalogAttempted: catalog.length, planningFailures, neighbourFallbacks,
    sourcesFrozen: JSON.stringify(sourceHashes) === JSON.stringify(hashes()) }));
  progress('mandatory-catalog-blocked', null);
  contacts.dispose();
  console.error('Mandatory catalog blocked:', planningFailures.length,
    'failures; optional neighbours and rendered-floor release were not attempted.');
  process.exit(1);
}
// Checked optional neighbour requests use the same planner in full and partial authoring.
if (!reusedPlan) for (const algorithm of algorithms) for (let i = 0; i < algorithm.steps.length - 1; i++) {
  const source = algorithm.steps[i].id, destination = algorithm.steps[i + 1].id;
  if (prepared[source] && prepared[destination]) planNeighbour(source, destination);
}
const output = { schemaVersion: 1, verified: false, kind: 'kinematic-demonstration', sourceHashes,
  planProvenance,
  segNames: A.SEG.map(s => s.name), poses: catalog, endpoints, tracks,
  neighbourFallbacks,
  checks: { oppositeArms: true, armsFeet: true, floor: false, criticalRegions: true, elbowMotion: true },
  limitations: [
    'These are constructed geometric demonstrations, not choreography measured from the PDF or a verified movement instruction.',
    'Floor placement follows displayed outer-muscle and bone geometry; support balance, inertia and force during these transitions are not solved.',
    'The complete body has existing registration overlaps. Opposite arms and all arm/foot pairs are checked separately; a complete soft-tissue contact model is absent.',
  ] };
if (partialReplan || repairReplan) {
  const origin = partialSelection.origin;
  origin.regeneratedTrackIds = partialSelection.requests.filter(r => Object.hasOwn(tracks, r.id)).map(r => r.id).sort();
  origin.regeneratedFallbacks = neighbourFallbacks.filter(f => origin.affectedPoseIds.includes(f.from) || origin.affectedPoseIds.includes(f.to));
  for (const id of origin.retainedTrackIds) {
    const current = tracks[id], previous = partialPrevious.tracks[id];
    if (repairReplan && JSON.stringify(current) !== JSON.stringify(previous))
      throw new Error('Retained coordinate track changed during motion repair: ' + id);
    if (JSON.stringify(current.frames.slice(1, -1)) !== JSON.stringify(previous.frames.slice(1, -1))
        || JSON.stringify(current.phases) !== JSON.stringify(previous.phases) || current.duration !== previous.duration)
      throw new Error('Retained coordinate data changed during partial authoring: ' + id);
  }
  const { checks: ignoredPartialChecks, jointMotion: ignoredPartialMotion, planProvenance: ignoredPartialProvenance, ...coordinates } = output;
  const candidate = { ...coordinates, kind: 'private-bone-motion-plan', verified: false, floorVerified: false,
    complete: true, planningFailures: [], sourcesFrozen: true, coordinateOrigin: origin };
  if (!validateRawPlan(candidate, { A, THREE, catalog, endpoints, sourceHashes, sourceNames: originalInputs }))
    throw new Error('Partial coordinate authoring did not produce a complete valid candidate graph');
  if (JSON.stringify(sourceHashes) !== JSON.stringify(hashes()) || sha(read(planSourceName)) !== origin.oldArchiveSha256)
    throw new Error('Source context or original coordinate archive changed during partial authoring');
  const backup = 'transitions/plan-origin-' + origin.oldArchiveSha256 + '.json.gz';
  fs.writeFileSync(new URL(backup, import.meta.url), partialPreviousBytes);
  const next = (await import('node:zlib')).gzipSync(JSON.stringify(candidate), { level: 9, mtime: 0 });
  fs.writeFileSync(new URL(planSourceName + '.next', import.meta.url), next);
  fs.renameSync(new URL(planSourceName + '.next', import.meta.url), new URL(planSourceName, import.meta.url));
  contacts.dispose();
  console.log(JSON.stringify({ partialCoordinateAuthoringOnly: true, motionRepairAuthoringOnly: repairReplan, verified: false, floorVerified: false,
    oldArchiveSha256: origin.oldArchiveSha256, newArchiveSha256: sha(next), backup,
    retainedTrackIds: origin.retainedTrackIds, discardedTrackIds: origin.discardedTrackIds,
    regeneratedRequests: origin.regeneratedRequests, regeneratedTrackIds: origin.regeneratedTrackIds,
    regeneratedFallbacks: origin.regeneratedFallbacks, endpointGroundingRebases: origin.endpointGroundingRebases }));
  process.exit(0);
}
if (geometryOnly) {
  const { checks: ignoredCandidateChecks, jointMotion: ignoredCandidateMotion, ...coordinates } = output;
  fs.writeFileSync('/tmp/fysm-transition-motion-plan.json', JSON.stringify({ ...coordinates,
    verified: false, kind: 'private-bone-motion-plan', complete: !planningFailures.length,
    planningFailures, sourcesFrozen: JSON.stringify(sourceHashes) === JSON.stringify(hashes()),
    floorVerified: false }));
  console.log('PRIVATE MOTION PLAN:', Object.keys(tracks).length, 'tracks;', planningFailures.length,
    'failures; rendered floor has not been verified.');
  if (!deferredFloor || planningFailures.length || JSON.stringify(sourceHashes) !== JSON.stringify(hashes())) {
    contacts.dispose();
    process.exit(deferredFloor ? 1 : 0);
  }
  fitSavedTrackFrames();
}
fs.mkdirSync(new URL('transitions/', import.meta.url), { recursive: true });
if (process.argv.includes('--plan-only')) {
  fs.writeFileSync(new URL('transitions/preview.json', import.meta.url), JSON.stringify(output));
  contacts.dispose(); console.log('Preview only; not release verified.'); process.exit(0);
}
// Each saved interval uses precisely the browser's normalized slerp. Build a
// local, symmetric floor lift after checking quarter points, then audit all
// twenty substeps by real bones and selected full surface states.
// The internal candidate sampler needs a structurally complete graph. This
// private facade shares the exact track arrays; output remains unverified and
// floor:false until every actual decoded floor/bone/motion check has passed.
const runtime = createTransitions(THREE, A, { ...output, verified: true,
  checks: { ...output.checks, floor: true } });
if (!runtime.valid) throw new Error('Runtime rejected the newly constructed graph');
let floorStates = 0, floorMinimumM = Infinity, floorMaximumM = -Infinity, maxLiftM = 0;
let reusedTranslatedStates = 0, reusedIdenticalStates = 0, translationMaxResidualM = 0;
const translationToleranceM = 1e-6;
for (const track of Object.values(tracks)) {
  progress('rendered-floor-and-decoded-bones', track.id, 0);
  for (let i = 0; i < track.frames.length - 1; i++) {
    if (!(i % 100)) progress('rendered-floor-and-decoded-bones', track.id, i);
    const a = track.frames[i], b = track.frames[i + 1];
    delete a.floorLiftM;
    let lift = 0, split = null;
    const quarterMinima = new Map();
    for (const u of [.25, .5, .75]) {
      const c = runtime.sampleTrack(track.id, a.t + (b.t - a.t) * u);
      const minimum = floor.minimum(c, track.groundModelId).minimumY; floorStates++;
      quarterMinima.set(Math.round(u * 20), { minimum, rootY: c.rootP.y });
      lift = Math.max(lift, (CLEARANCE - minimum) / (4 * u * (1 - u)));
      if (minimum > .0035) split = { conf: c, minimum, t: a.t + (b.t - a.t) * u };
    }
    if (lift > .025 && !split) {
      const t = (a.t + b.t) / 2, conf = runtime.sampleTrack(track.id, t);
      split = { conf, minimum: floor.minimum(conf, track.groundModelId).minimumY, t }; floorStates++;
    }
    if (split) {
      if (track.frames.length >= 3000 || b.t - a.t < 1e-5) throw new Error('Unresolved floor trajectory ' + track.id);
      split.conf.rootP.y += CLEARANCE - split.minimum;
      track.frames.splice(i + 1, 0, encode(split.conf, split.t)); i--; continue;
    }
    a.floorLiftM = number(Math.max(0, lift)); maxLiftM = Math.max(maxLiftM, a.floorLiftM);
    const anchorStep = [5, 10, 15][i % 3];
    let anchor = null, translatedReuse = false;
    if (a.floorLiftM > 0) {
      const t = a.t + (b.t - a.t) * anchorStep / 20, c = runtime.sampleTrack(track.id, t);
      const pre = quarterMinima.get(anchorStep), predicted = pre.minimum + c.rootP.y - pre.rootY;
      const minimum = floor.minimum(c, track.groundModelId).minimumY; floorStates++;
      const residual = Math.abs(minimum - predicted);
      translationMaxResidualM = Math.max(translationMaxResidualM, residual);
      translatedReuse = residual <= translationToleranceM;
      anchor = { minimum };
    }
    for (let j = 1; j < 20; j++) {
      const u = j / 20, t = a.t + (b.t - a.t) * u, c = runtime.sampleTrack(track.id, t), at = collision(c);
      if (at) throw new Error(`Between-key ${track.id}:${i}:${j} intersects: ${JSON.stringify(at)}`);
      // Four additional off-calibration samples catch a thin dip or contact
      // switch missed by the three envelope-construction quarter points.
      if ([2, 5, 7, 10, 13, 15, 18].includes(j)) {
        let minimum, uncertainty = 0;
        const pre = quarterMinima.get(j);
        if (pre && a.floorLiftM === 0) {
          // Same complete configuration as the actual pre-envelope query.
          minimum = pre.minimum; reusedIdenticalStates++; floorStates++;
        } else if (pre && j === anchorStep && anchor) {
          minimum = anchor.minimum;
        } else if (pre && translatedReuse) {
          const predicted = pre.minimum + c.rootP.y - pre.rootY;
          if (predicted - translationToleranceM >= -2e-6 && predicted + translationToleranceM <= .004) {
            minimum = predicted; uncertainty = translationToleranceM; reusedTranslatedStates++; floorStates++;
          }
        }
        if (minimum === undefined) { minimum = floor.minimum(c, track.groundModelId).minimumY; floorStates++; }
        if (minimum < -2e-6 || minimum > .004) {
          if (track.frames.length >= 3000 || b.t - a.t < 1e-5) throw new Error(`Invalid rendered floor contact ${track.id}:${i}:${j}: ${minimum} m`);
          c.rootP.y += CLEARANCE - minimum;
          delete a.floorLiftM; track.frames.splice(i + 1, 0, encode(c, t)); i--; break;
        }
        floorMinimumM = Math.min(floorMinimumM, minimum - uncertainty); floorMaximumM = Math.max(floorMaximumM, minimum + uncertainty);
      }
    }
  }
  // Check the final key inventory after adaptive insertions as well as every
  // between-key state above. A newly inserted key is never assumed safe.
  for (const frame of track.frames) {
    const c = runtime.sampleTrack(track.id, frame.t), hit = collision(c);
    if (hit) throw new Error(`Saved decoded frame ${track.id}:${frame.t} intersects: ${JSON.stringify(hit)}`);
    const minimum = floor.minimum(c, track.groundModelId).minimumY; floorStates++;
    if (minimum < -2e-6 || minimum > .004) throw new Error(`Saved decoded floor contact ${track.id}:${frame.t}: ${minimum} m`);
    floorMinimumM = Math.min(floorMinimumM, minimum); floorMaximumM = Math.max(floorMaximumM, minimum);
  }
  console.log('Verified', track.id, 'floor lift≤', (maxLiftM * 1000).toFixed(3), 'mm');
}
contacts.dispose();
const jointMotion = motionGuard.audit(runtime, output);
output.jointMotion = jointMotion;
if (JSON.stringify(sourceHashes) !== JSON.stringify(hashes())) throw new Error('Transition sources changed during generation. Rebuild frozen sources.');
let routeCount = 0;
for (const a of catalog) for (const b of catalog) if (a.id !== b.id) {
  const route = runtime.route(a.id, b.id);
  if (!route || !(route.duration > 0) || !runtime.sample(route, 0) || !runtime.sample(route, route.duration)) throw new Error('Disconnected transition graph');
  routeCount++;
}
output.verified = true; output.checks.floor = true;
const body = JSON.stringify(output), check = { schemaVersion: 1, verified: true, kind: output.kind,
  dataSha256: sha(body), sourceHashes, poseCount: catalog.length, trackCount: Object.keys(tracks).length, routeCount,
  checks: output.checks, collisionSubsteps: 20, collisionStates, floorStates,
  planProvenance,
  collisionStatesScope: 'all-discovery-and-final-queries-including-rejected-candidates',
  finalCollisionStates: Object.values(tracks).reduce((n, t) => n + 20 * (t.frames.length - 1) + 1, 0),
  collisionEvaluations, collisionCacheHits, lowerEvaluations, lowerCacheHits,
  jointMotion, jointMotionRejectedCandidates,
  fullSurfaceEvaluations: floor.evaluations, reusedTranslatedStates, reusedIdenticalStates,
  translationMaxResidualM, translationToleranceM,
  criticalCollision: { verified: true, scope: critical.scope, substeps: 20,
    states: Object.values(tracks).reduce((n, t) => n + 20 * (t.frames.length - 1) + 1, 0),
    boneParts: critical.collider.boneParts,
    categories: critical.categories,
    wholePairExclusions: [], articulationPolicy: 'bounded-joint-local-only' },
  floorAuditSubsteps: [2, 5, 7, 10, 13, 15, 18],
  floorScope: floor.scope, floorVertexCounts: floor.counts, floorMinimumM, floorMaximumM, maxLiftM,
  fullBodyCollisionFree: false, dynamicBalanceVerified: false,
  limitations: output.limitations };
fs.writeFileSync(new URL('transitions/data.json', import.meta.url), body);
fs.writeFileSync(new URL('transitions/check.json', import.meta.url), JSON.stringify(check));
progress('verified-local-packet-written', null);
console.log('Built', catalog.length, 'asanas,', routeCount, 'directed routes;', collisionStates, 'collision samples;', floorStates,
  'audited floor states;', floor.evaluations, 'full surface queries.');
