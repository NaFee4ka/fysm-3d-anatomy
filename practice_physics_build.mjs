// Build-time, side-aware static equilibrium for the 24 photographed FYSM poses.
// Importing this module never solves a pose or writes a file.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { createPracticeContacts, convexHull, supportMargin } from './practice/contacts.mjs';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const read = name => fs.readFileSync(path.join(ROOT, name));
const json = name => JSON.parse(read(name));
const sha = data => createHash('sha256').update(data).digest('hex');
const V = (...p) => new THREE.Vector3(...p);
const vector = p => p?.toArray ? p.toArray() : p || null;
const finite = n => typeof n === 'number' && Number.isFinite(n);
const INTRINSIC_UNIDENTIFIED = ['thenar', 'hypothenar', 'interossei'];

export function practiceConfiguration(pose) {
  return { local: Object.fromEntries(Object.entries(pose.local).map(([name, q]) =>
    [name, new THREE.Quaternion(...q).normalize()])),
  rootQ: new THREE.Quaternion(...pose.root).normalize(), rootP: V(...pose.rootP) };
}
function muscleResults(r) {
  const output = {}, groups = new Map();
  const unidentified = new Set([...(r.unidentifiedUnits || []), ...INTRINSIC_UNIDENTIFIED]);
  r.INST.forEach((inst, j) => {
    const key = inst.unit + '|' + inst.side;
    if (!groups.has(key)) groups.set(key, { unit: inst.unit, side: inst.side, js: [] });
    groups.get(key).js.push(j);
  });
  for (const { unit, side, js } of groups.values()) {
    const sum = f => js.reduce((s, j) => s + f(j), 0);
    const F = sum(j => r.force[j]), Fp = sum(j => r.FP[j]), Fmax = sum(j => r.FMAX[j]), Fe = sum(j => r.FE[j]);
    const quality = js.map(j => r.muscleQuality?.[j] || { status: 'estimated', quarantined: false });
    const quarantinedStrands = quality.filter(q => q.quarantined || q.status === 'quarantined').length;
    const allQuarantined = quarantinedStrands === js.length;
    const missingDOF = unidentified.has(unit);
    const status = missingDOF ? 'unidentified' : allQuarantined ? 'quarantined' : 'estimated';
    const lengths = js.map(j => r.lt[j]);
    const limitations = [];
    if (missingDOF) limitations.push('no-independent-finger-degrees-of-freedom');
    if (quarantinedStrands) limitations.push('invalid-path-strands-excluded-from-force');
    if (Fe <= 1e-9) limitations.push('negligible-active-capacity-at-model-length');
    if (lengths.some(length => length < .4 || length > 1.7)) limitations.push('force-length-outside-typical-range');
    (output[unit] ||= {})[side] = {
      F, Fp, Fmax, Fe, a: status === 'estimated' ? F / Fmax : null,
      activation: status === 'estimated' ? sum(j => r.act[j] * r.FMAX[j]) / Fmax : null,
      status, available: status === 'estimated' && Fe > 1e-9, quarantined: allQuarantined,
      strands: js.length, validPathStrands: js.length - quarantinedStrands,
      availableStrands: quality.filter(q => q.available ?? !q.quarantined).length, quarantinedStrands,
      availableNominalCapacityN: sum(j => r.muscleQuality?.[j]?.quarantined ? 0 : r.FMAX[j]),
      slackFiberStrands: lengths.filter(length => length <= 0).length,
      normalizedFiberLengthRange: [Math.min(...lengths), Math.max(...lengths)],
      minNormalizedFiberLength: Math.min(...lengths), maxNormalizedFiberLength: Math.max(...lengths), limitations,
    };
  }
  return output;
}
function jointResults(A, PH, r) {
  const joints = {};
  for (const d of r.dofOut) {
    if (d.joint === 'root') continue;
    const cap = d.mus >= 0 ? d.capPos : d.capNeg;
    (joints[d.joint] ||= { comps: [] }).comps.push({
      c: d.id.split('_').at(-1), need: d.mus, load: d.ext + d.st,
      passive: d.pas, res: d.res, cap, eff: Math.abs(d.mus) / Math.max(1, cap),
      axisFrame: 'parent-anatomical-segment',
    });
  }
  const W = r.W;
  for (const side of ['L', 'R']) {
    const id = name => name + '_' + side;
    const sk = ['x', 'y', 'z'].map(axis => PH.dofIndex[id('gh') + '_' + axis]);
    const axes = sk.map(k => PH.DOF[k].axis(W));
    const moment = field => axes.reduce((sum, axis, i) => sum.addScaledVector(axis, r.dofOut[sk[i]][field]), V(0, 0, 0));
    const h = W[id('fore')].p.clone().sub(W[id('hum')].p).normalize();
    const down = V(0, -1, 0).applyQuaternion(W.thC.q);
    const elevation = down.clone().cross(h); if (elevation.lengthSq() < 1e-6) elevation.copy(axes[0]); elevation.normalize();
    const capacity = axis => {
      let positive = 0, negative = 0;
      r.INST.forEach((inst, j) => {
        const lever = sk.reduce((value, k, i) => value + r.R[j][k] * axes[i].dot(axis), 0);
        if (lever > 0) positive += lever * r.FE[j]; else negative -= lever * r.FE[j];
      });
      return { positive, negative };
    };
    const Mm = moment('mus'), Me = moment('ext'), Mp = moment('pas');
    const component = axis => {
      const need = Mm.dot(axis), c = capacity(axis), cap = need >= 0 ? c.positive : c.negative;
      return { need, load: Me.dot(axis), passive: Mp.dot(axis), cap, eff: Math.abs(need) / Math.max(1, cap) };
    };
    joints[id('gh')].arm = { elev: component(elevation), rot: component(h) };
    const reactionMap = { gh: 'gh', el: 'elbow', wr: 'wrist', hip: 'hip', kn: 'knee', an: 'ankle' };
    for (const [joint, reaction] of Object.entries(reactionMap)) {
      const value = r.jr[id(reaction)];
      if (joints[id(joint)] && value) joints[id(joint)].jr = { F: value.F.length(), bw: value.F.length() / r.BW, vector: value.F.toArray() };
    }
    const scapular = r.stF.filter(contact => contact.side === side).reduce((sum, contact) => sum + contact.F.length(), 0);
    joints[id('st')] = { st: { F: scapular, bw: scapular / r.BW } };
    // Same reconstructed patellofemoral resultant as the existing exporter,
    // evaluated separately for the two limbs rather than mirroring the left.
    let quadriceps = 0; const direction = V(0, 0, 0), subtree = PH.subtree(id('shank'));
    r.INST.forEach((inst, j) => {
      if (inst.side !== side || !/^(rectFem|vastLat|vastMed|vastInt)$/.test(inst.unit)) return;
      const force = r.force[j] + r.FP[j]; if (force <= 0) return;
      for (let i = 1; i < r.paths[j].length; i++) {
        const a = r.paths[j][i - 1], b = r.paths[j][i];
        if (!subtree.has(a.seg) && subtree.has(b.seg)) { direction.addScaledVector(a.p.clone().sub(b.p).normalize(), force); quadriceps += force; }
      }
    });
    const s = side === 'L' ? 1 : -1;
    const patella = A.pt(W, id('shank'), V(s * .098, .528, .04));
    const tuberosity = A.pt(W, id('shank'), V(s * .098, .452, .03));
    const pf = direction.add(tuberosity.sub(patella).normalize().multiplyScalar(quadriceps));
    if (joints[id('kn')]) joints[id('kn')].pf = { F: pf.length(), bw: pf.length() / r.BW, quad: quadriceps,
      method: 'reconstructed-quadriceps-patellar-resultant' };
  }
  for (const [joint, reaction] of [['ls', 'L5S1'], ['tl', 'T12L1']]) {
    const value = r.jr[reaction];
    (joints[joint] ||= { comps: [] }).jr = { F: value.F.length(), bw: value.F.length() / r.BW,
      comp: value.compression, shear: value.shear, vector: value.F.toArray() };
  }
  return joints;
}
function serializeInternal(contact) {
  return { id: contact.id, segA: contact.segA, segB: contact.segB, P: vector(contact.P), F: vector(contact.F),
    kind: contact.kind, side: contact.side, normal: vector(contact.normal), mu: contact.mu ?? 0,
    maxForceN: contact.maxForceN ?? null, maxNormalN: contact.maxNormalN ?? null,
    gapM: contact.gapM ?? null, source: contact.source || null };
}
export function serializePracticePhysics(A, PH, r, contactData) {
  if (r.quality?.converged !== true || r.quality?.finite !== true || r.quality?.status !== 'solved')
    throw new Error('Practice static physics did not produce a verified solution');
  const floor = r.contactF.map(c => ({ id: c.id ?? String(c.li), region: c.region, side: c.side, seg: c.seg,
    P: c.P.toArray(), cop: c.P.toArray(), F: c.F.toArray(), pct: c.F.y / r.BW * 100,
    tang: Math.hypot(c.F.x, c.F.z) / r.BW * 100, mu: c.mu, k: c.k, areaM2: c.areaM2 ?? null, source: c.source || null }));
  const force = r.contactF.reduce((sum, c) => sum.add(c.F), V(0, 0, 0));
  const moment = r.contactF.reduce((sum, c) => sum.add(c.P.clone().cross(c.F)), V(0, 0, 0));
  moment.add(r.com.clone().cross(V(0, -r.BW, 0)));
  const weightedCOP = r.contactF.reduce((sum, c) => sum.addScaledVector(c.P, c.F.y), V(0, 0, 0));
  const cop = force.y > 0 ? weightedCOP.multiplyScalar(1 / force.y).toArray() : null;
  const loaded = r.contactF.filter(c => c.F.y > .002 * r.BW);
  const poly = convexHull(loaded.map(c => [c.P.x, c.P.z]));
  const geometricSupport = contactData.diagnostics.unclusteredSupport || contactData.diagnostics.support;
  const limits = { pathQuality: r.pathQuality || null, constitutive: r.constitutive || null,
    contactQuality: { ...(r.contactQuality || {}),
      surfaceGeometryValid: contactData.diagnostics.surfaceGeometryValid,
      maxSurfacePenetrationM: contactData.diagnostics.maxSurfacePenetrationM,
      missingPrimarySupports: contactData.diagnostics.missingPrimarySupports || [],
      inferredPads: contactData.diagnostics.inferredPads || [],
      missingProxyGeometry: contactData.diagnostics.missingProxyGeometry || false,
      minFingertipBoneHeightM: contactData.diagnostics.minFingertipBoneHeightM ?? null,
      supportDegenerate: geometricSupport.poly.length < 3,
      geometricSupportMarginM: geometricSupport.margin },
    unidentifiedUnits: [...new Set([...(r.unidentifiedUnits || []), ...INTRINSIC_UNIDENTIFIED])],
    forceLengthOutsideTypicalStrands: r.lt.filter(length => length < .4 || length > 1.7).length,
    forceLengthClampedStrands: 0, fibreFractionFloorUnits: [],
    contactOwnership: 'surface points mechanically attached to their dominant anatomical segment',
    softTissueContact: 'quasistatic surface quadrature; no pressure/FEM or person-specific calibration',
  };
  const contactGeometryVerified = contactData.diagnostics.surfaceGeometryValid
    && contactData.diagnostics.contactModelComplete
    && !contactData.diagnostics.missingProxyGeometry
    && (contactData.diagnostics.minFingertipBoneHeightM === null
      || contactData.diagnostics.minFingertipBoneHeightM === undefined
      || contactData.diagnostics.minFingertipBoneHeightM >= -1e-6)
    && !(contactData.diagnostics.missingPrimarySupports || []).length
    && geometricSupport.margin !== null && geometricSupport.margin >= -1e-6;
  const result = {
    bw: r.BW, mass: r.BODY, com: r.com.toArray(),
    support: { poly, margin: supportMargin(poly, [r.com.x, r.com.z]), cop,
      geometric: geometricSupport },
    floor, internalContacts: (r.internalContactF || []).map(serializeInternal),
    muscles: muscleResults(r), joints: jointResults(A, PH, r),
    balance: { forceN: force.toArray(), weightN: r.BW,
      forceErrorN: force.clone().add(V(0, -r.BW, 0)).toArray(), momentErrorNm: moment.toArray() },
    diag: { quality: r.quality, physicalBalance: r.physicalBalance || null, contactGeometryVerified,
      forceResidualN: r.quality.forceResidualN, momentResidualNm: r.quality.momentResidualNm,
      resid: r.maxRes, reserves: Object.values(r.reserve).reduce((sum, value) => sum + Math.abs(value), 0),
      maxReserveNm: Math.max(0, ...Object.values(r.reserve).map(Math.abs)), reserveByDof: r.reserve,
      contacts: contactData.diagnostics },
    modelLimits: limits,
  };
  function check(value, route = '') {
    if (typeof value === 'number' && !finite(value)) throw new Error('Nonfinite practice physics result: ' + route);
    if (value && typeof value === 'object') for (const [key, child] of Object.entries(value)) check(child, route + '.' + key);
  }
  check(result);
  return result;
}
export const PRACTICE_PHYSICS_SOURCES = ['practice/poses.json', 'practice/algorithms.json',
  'practice/contacts.mjs', 'practice_physics_build.mjs', 'anatomy3.js', 'physics_core.mjs', 'physics_params.mjs',
  'muscle_surface_render.js', 'muscles/muscles_bp3d.json', 'skull/bones_bp3d.json', 'package-lock.json'];

export function isVerifiedPracticeSensitivity(sensitivity) {
  return sensitivity?.kind === 'two-deterministic-geometric-area-hypotheses'
    && sensitivity.verified === true && sensitivity.isConfidenceInterval === false
    && Array.isArray(sensitivity.hypotheses) && sensitivity.hypotheses.length === 2
    && sensitivity.hypotheses.every((hypothesis, index) => hypothesis.areaScale === [1, .5][index]
      && Number.isFinite(hypothesis.contactAreaM2) && hypothesis.contactAreaM2 > 0
      && hypothesis.contactGeometryVerified === true && hypothesis.physicalBalance?.verified === true
      && hypothesis.quality?.status === 'solved' && hypothesis.quality.converged === true
      && hypothesis.quality.finite === true);
}

export function practiceInputProvenance(options = {}) {
  const callerOverrides = ['anatomy', 'physics', 'poseData', 'manifest', 'contacts']
    .filter(key => options[key] !== undefined);
  return { kind: callerOverrides.length ? 'caller-supplied-overrides' : 'repository-sources', callerOverrides,
    repositoryBound: callerOverrides.length === 0 };
}

export function practiceCheckpointPath(input) {
  const target = path.resolve(input);
  const reserved = new Set([...PRACTICE_PHYSICS_SOURCES, 'practice/physics.json', 'practice/physics-check.json',
    'practice/check.json', 'physics.json', 'poses.json', 'cycle.json', 'dist/index.html']
    .map(name => path.resolve(ROOT, name)));
  if (reserved.has(target)) throw new Error('A practice checkpoint cannot overwrite a production file or a physics source: ' + target);
  if (!target.startsWith('/tmp/')) throw new Error('Nonpublishing practice physics checkpoints must be written under /tmp');
  return target;
}

export async function buildPracticePhysics(options = {}) {
  const inputProvenance = practiceInputProvenance(options);
  const sourceHashes = Object.fromEntries(PRACTICE_PHYSICS_SOURCES.map(name => [name, sha(read(name))]));
  const A = options.anatomy || createAnatomy(THREE), mass = options.mass ?? 70;
  const PH = options.physics || makePhysics(A, { mass });
  const poseData = options.poseData || json('practice/poses.json');
  const manifest = options.manifest || json('practice/algorithms.json');
  const expectedIds = manifest.algorithms.flatMap(a => a.steps.map(step => step.id));
  if (manifest.algorithms.length !== 3 || manifest.algorithms.some(a => a.steps.length !== 8)
      || expectedIds.length !== 24 || new Set(expectedIds).size !== 24)
    throw new Error('Practice static physics requires exactly the three source algorithms and 24 unique poses');
  const ids = options.onlyIds || expectedIds;
  for (const id of ids) if (!Object.hasOwn(poseData.poses, id)) throw new Error('Unknown practice physics pose: ' + id);
  const factory = options.contacts || await createPracticeContacts(A, THREE, json('muscles/muscles_bp3d.json'),
    { boneData: json('skull/bones_bp3d.json') });
  const poses = {}, reports = {};
  try {
    for (const id of ids) {
      const conf = practiceConfiguration(poseData.poses[id]);
      const W = A.computeWorld(conf.local, conf.rootQ, conf.rootP);
      let contacts;
      try { contacts = factory.build(id, W, { audit: !!options.contactsOnly }); }
      catch (error) {
        if (!options.contactsOnly) throw error;
        reports[id] = { error: error.message, contactModelComplete: false };
        options.progress?.(id, { contacts: reports[id] }); continue;
      }
      reports[id] = contacts.diagnostics;
      if (options.contactsOnly) { options.progress?.(id, { contacts: contacts.diagnostics }); continue; }
      let result;
      try {
        result = PH.solve(conf, { contacts: contacts.contacts, internalContacts: contacts.internalContacts,
          tone: {}, iters: options.iters ?? 20000 });
      } catch (error) {
        error.message = 'Practice pose ' + id + ': ' + error.message;
        error.practicePoseId = id;
        throw error;
      }
      poses[id] = serializePracticePhysics(A, PH, result, contacts);
      if (id === 'struna-08' && options.sensitivity !== false) {
        const alternateContacts = factory.build(id, W, { areaScale: .5 });
        const alternateResult = PH.solve(conf, { contacts: alternateContacts.contacts,
          internalContacts: alternateContacts.internalContacts, tone: {}, iters: options.iters ?? 20000 });
        const alternate = serializePracticePhysics(A, PH, alternateResult, alternateContacts);
        const hypothesis = (pose, areaScale) => ({ areaScale,
          contactAreaM2: pose.diag.contacts.inferredPads.reduce((sum, pad) => sum + pad.areaM2, 0),
          contactGeometryVerified: pose.diag.contactGeometryVerified,
          quality: pose.diag.quality, physicalBalance: pose.diag.physicalBalance,
          reserveTotalNm: pose.diag.reserves, maximumReserveNm: pose.diag.maxReserveNm,
          muscles: pose.muscles, joints: pose.joints, floor: pose.floor,
        });
        poses[id].modelLimits.softTissueContactSensitivity = {
          kind: 'two-deterministic-geometric-area-hypotheses', isConfidenceInterval: false,
          verified: alternate.diag.contactGeometryVerified && alternate.diag.physicalBalance?.verified === true,
          unchangedParameters: ['muscle-strength', 'passive-tissue', 'friction', 'relative-contact-stiffness', 'kinematics'],
          hypotheses: [hypothesis(poses[id], 1), hypothesis(alternate, .5)],
        };
      }
      options.progress?.(id, poses[id]);
    }
  } finally { if (!options.contacts) factory.dispose(); }
  for (const name of PRACTICE_PHYSICS_SOURCES) if (sourceHashes[name] !== sha(read(name)))
    throw new Error('Practice physics source changed during calculation: ' + name);
  if (options.contactsOnly) return { schemaVersion: 1, kind: 'practice-contact-audit',
    posesSha256: sourceHashes['practice/poses.json'], sourceHashes, poses: reports };
  const complete = ids.length === expectedIds.length && expectedIds.every(id => Object.hasOwn(poses, id));
  const sensitivityVerified = isVerifiedPracticeSensitivity(poses['struna-08']?.modelLimits?.softTissueContactSensitivity);
  return { schemaVersion: 1, method: 'quasistatic-static-optimization',
    verified: complete && inputProvenance.repositoryBound && sensitivityVerified
      && Object.values(poses).every(p => p.diag.contactGeometryVerified
      && p.diag.physicalBalance?.verified === true
      && p.modelLimits.softTissueContactSensitivity?.verified !== false), mass,
    posesSha256: sourceHashes['practice/poses.json'], sourceHashes, inputProvenance, poses };
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2), index = args.indexOf('--only'), reportIndex = args.indexOf('--report');
  const checkpointIndex = args.indexOf('--checkpoint');
  const checkpointPath = checkpointIndex >= 0 ? practiceCheckpointPath(args[checkpointIndex + 1]) : null;
  const checkpoint = { schemaVersion: 1, publication: false,
    posesSha256: sha(read('practice/poses.json')), poses: {} };
  const onlyIds = index >= 0 ? args[index + 1].split(',') : null;
  const contactsOnly = args.includes('--contacts-only');
  const result = await buildPracticePhysics({ onlyIds, contactsOnly,
    progress(id, pose) {
      if (checkpointPath) {
        checkpoint.poses[id] = pose;
        fs.writeFileSync(checkpointPath, JSON.stringify(checkpoint));
      }
      console.log(id, contactsOnly ? 'contacts ' + pose.contacts.contactCount
        : 'solved; reserve ' + pose.diag.reserves.toFixed(2) + ' N*m; force residual ' + pose.diag.forceResidualN + ' N');
    } });
  if (reportIndex >= 0) fs.writeFileSync(path.resolve(args[reportIndex + 1]), JSON.stringify(result, null, 2));
  if (!contactsOnly && !onlyIds) {
    if (!result.verified) throw new Error('Cannot publish an incomplete practice physics package');
    const output = JSON.stringify(result);
    fs.writeFileSync(path.join(ROOT, 'practice/physics.json'), output);
    fs.writeFileSync(path.join(ROOT, 'practice/physics-check.json'), JSON.stringify({
      schemaVersion: 1, verified: true, poseCount: 24, physicsSha256: sha(output),
      posesSha256: result.posesSha256, sourceHashes: result.sourceHashes,
    }));
  }
}
