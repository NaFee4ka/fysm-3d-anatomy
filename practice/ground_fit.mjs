// Placement against the geometry actually displayed by the app. These are
// static geometric fits, not a deformable skin/contact simulation.
import { gunzipSync } from 'node:zlib';
import { PRACTICE_SUPPORT_INTENT, practiceSurfaceRegion, matchesPracticeSupport, convexHull, supportMargin, FLOOR_BAND_M } from './contacts.mjs';
import { armIK, orientHand, LH } from '../cycle_lib.mjs';

const finitePoint = p => p && [p.x, p.y, p.z].every(Number.isFinite);
const sideOf = seg => /_([LR])$/.exec(seg)?.[1] || null;
const copyWitness = p => p ? { ...p, point: [...p.point] } : null;

// BodyParts3D bone coordinates are already local to the named segment origin.
// Do not subtract the anatomical reference joint a second time.
export function decodeGroundBones(data, A) {
  if (!data || !Array.isArray(data.parts) || !data.parts.length || typeof data.data !== 'string')
    throw new Error('Ground fit requires the complete bone atlas');
  const bytes = gunzipSync(Buffer.from(data.data, 'base64'));
  const expected = data.parts.reduce((n, p) => n + 6 * (p.n + p.m), 0);
  if (!Number.isSafeInteger(expected) || bytes.length !== expected) throw new Error('Invalid ground bone payload length');
  let offset = 0;
  return data.parts.map((part, index) => {
    if (!Object.hasOwn(A.S, part.seg) || !Number.isInteger(part.n) || part.n <= 0 || !Number.isInteger(part.m) || part.m < 0
        || !Array.isArray(part.lo) || !Array.isArray(part.hi) || part.lo.length !== 3 || part.hi.length !== 3
        || !part.lo.every((v, i) => Number.isFinite(v) && Number.isFinite(part.hi[i]) && part.hi[i] >= v))
      throw new Error('Invalid ground bone part ' + index);
    const positions = new Float64Array(part.n * 3), acc = [0, 0, 0];
    for (let v = 0; v < part.n; v++) for (let axis = 0; axis < 3; axis++) {
      acc[axis] = (acc[axis] + bytes.readUInt16LE(offset)) & 65535; offset += 2;
      positions[v * 3 + axis] = part.lo[axis] + (part.hi[axis] - part.lo[axis]) * acc[axis] / 65535;
    }
    offset += part.m * 6;
    return { id: part.id || part.name || String(index), seg: part.seg, positions };
  });
}

export function groundedRootPosition(rootP, minimumY, clearanceM = 0) {
  if (!finitePoint(rootP) || !Number.isFinite(minimumY) || !Number.isFinite(clearanceM) || clearanceM < 0 || clearanceM > FLOOR_BAND_M)
    throw new Error('Invalid ground translation');
  const out = rootP.clone(); out.y += clearanceM - minimumY; return out;
}

function cloneConfiguration(conf, A, THREE) {
  if (!conf || !conf.local || !finitePoint(conf.rootP) || !conf.rootQ?.isQuaternion)
    throw new Error('Ground fit requires a normalized configuration');
  const local = {};
  for (const seg of A.SEG) {
    const q = conf.local[seg.name];
    if (!q?.isQuaternion || !q.toArray().every(Number.isFinite) || Math.abs(q.lengthSq() - 1) > 1e-5)
      throw new Error('Invalid ground fit quaternion: ' + seg.name);
    local[seg.name] = q.clone().normalize();
  }
  if (!conf.rootQ.toArray().every(Number.isFinite) || Math.abs(conf.rootQ.lengthSq() - 1) > 1e-5)
    throw new Error('Invalid ground fit root quaternion');
  return { ...conf, local, rootQ: conf.rootQ.clone().normalize(), rootP: new THREE.Vector3().copy(conf.rootP) };
}

// Search a small world pitch/roll correction of a frozen photographed pose.
// All points undergo the same rigid transform: no tissue or limb is stretched.
// The floor is the supporting plane of the complete displayed point cloud.
export function optimizeGroundTilt(points, groups, THREE, options = {}) {
  const maxPitchDeg = options.maxPitchDeg ?? 6, maxRollDeg = options.maxRollDeg ?? 6;
  const targetGapM = options.targetGapM ?? .001;
  if (![maxPitchDeg, maxRollDeg].every(n => Number.isFinite(n) && n >= 0 && n <= 20) || !points.length)
    throw new Error('Invalid bounded ground tilt');
  if (!Number.isFinite(targetGapM) || targetGapM < 0 || targetGapM > FLOOR_BAND_M) throw new Error('Invalid ground tilt support tolerance');
  const balance = !!options.balance;
  if (balance && (!Array.isArray(options.com) || options.com.length !== 3 || !options.com.every(Number.isFinite)))
    throw new Error('Ground balance fit requires the model centre of mass');
  const xyz = new Float64Array(points.length * 3), members = groups.map(() => []);
  const outer = [];
  points.forEach((p, i) => {
    if (!Array.isArray(p.point) || p.point.length !== 3 || !p.point.every(Number.isFinite)) throw new Error('Invalid ground tilt point');
    xyz.set(p.point, i * 3);
    if (p.kind !== 'bone') {
      outer.push(i);
      groups.forEach((g, j) => {
        if (matchesPracticeSupport(g, p)) members[j].push(i);
      });
    }
  });
  const yAxis = new THREE.Vector3(0, 1, 0), xAxis = new THREE.Vector3(1, 0, 0), zAxis = new THREE.Vector3(0, 0, 1);
  const matrix = new THREE.Matrix4(), quaternion = new THREE.Quaternion(), rollQ = new THREE.Quaternion();
  const evaluated = new Map();
  const evaluate = (pitchDeg, rollDeg) => {
    const key = pitchDeg.toFixed(6) + ':' + rollDeg.toFixed(6); if (evaluated.has(key)) return evaluated.get(key);
    quaternion.setFromAxisAngle(xAxis, pitchDeg * Math.PI / 180);
    rollQ.setFromAxisAngle(zAxis, rollDeg * Math.PI / 180); quaternion.premultiply(rollQ);
    matrix.makeRotationFromQuaternion(quaternion); const e = matrix.elements;
    yAxis.set(e[1], e[5], e[9]);
    let minimum = Infinity;
    for (let i = 0; i < xyz.length; i += 3) minimum = Math.min(minimum, yAxis.x * xyz[i] + yAxis.y * xyz[i + 1] + yAxis.z * xyz[i + 2]);
    const gaps = members.map(indices => {
      let h = Infinity;
      for (const i of indices) h = Math.min(h, yAxis.x * xyz[i * 3] + yAxis.y * xyz[i * 3 + 1] + yAxis.z * xyz[i * 3 + 2]);
      return Number.isFinite(h) ? h - minimum : null;
    });
    // A weak angular prior picks the smallest nearby correction when several
    // planes restore the same supports. It is not a material stiffness.
    let marginM = null;
    if (balance) {
      const floorPoints = [];
      for (const i of outer) {
        const x = xyz[i * 3], y = xyz[i * 3 + 1], z = xyz[i * 3 + 2];
        if (yAxis.x * x + yAxis.y * y + yAxis.z * z - minimum <= FLOOR_BAND_M)
          floorPoints.push([e[0] * x + e[4] * y + e[8] * z, e[2] * x + e[6] * y + e[10] * z]);
      }
      const [x, y, z] = options.com;
      marginM = supportMargin(convexHull(floorPoints), [e[0] * x + e[4] * y + e[8] * z, e[2] * x + e[6] * y + e[10] * z]);
    }
    const score = gaps.reduce((sum, h) => sum + (h === null ? 1 : Math.max(0, h - targetGapM) ** 2), 0)
      + (balance ? (marginM === null ? .02 : Math.max(0, .001 - marginM)) ** 2 : 0)
      + (pitchDeg ** 2 + rollDeg ** 2) * 2.5e-9;
    const result = { pitchDeg, rollDeg, score, gapsM: gaps, geometryMarginM: marginM,
      quaternion: quaternion.clone(), minimumY: minimum };
    evaluated.set(key, result); return result;
  };
  let best = evaluate(0, 0);
  const improve = candidate => { if (candidate.score < best.score - 1e-16) best = candidate; };
  for (const pitch of [-maxPitchDeg, 0, maxPitchDeg])
    for (const roll of [-maxRollDeg, 0, maxRollDeg]) improve(evaluate(pitch, roll));
  for (let pitch = -maxPitchDeg; pitch <= maxPitchDeg + 1e-9; pitch += 1)
    for (let roll = -maxRollDeg; roll <= maxRollDeg + 1e-9; roll += 1) improve(evaluate(pitch, roll));
  for (const step of [.2, .04]) {
    const centre = best;
    for (let i = -5; i <= 5; i++) for (let j = -5; j <= 5; j++) {
      const pitch = centre.pitchDeg + i * step, roll = centre.rollDeg + j * step;
      if (Math.abs(pitch) <= maxPitchDeg + 1e-9 && Math.abs(roll) <= maxRollDeg + 1e-9) improve(evaluate(pitch, roll));
    }
  }
  return { ...best, evaluatedPlanes: evaluated.size,
    boundReached: (maxPitchDeg > 0 && Math.abs(best.pitchDeg) >= maxPitchDeg - .041)
      || (maxRollDeg > 0 && Math.abs(best.rollDeg) >= maxRollDeg - .041) };
}

export function createPracticeGroundFitter({ A, THREE, surfaces, boneData, boneParts, contactFactory }) {
  surfaces ||= contactFactory?.surfaces;
  if (!A || !THREE || !surfaces?.pickables || typeof surfaces.update !== 'function')
    throw new Error('Ground fit requires the production muscle surfaces');
  const bones = boneParts || decodeGroundBones(boneData, A);
  const records = surfaces.pickables.filter(mesh => !A.UNITS[mesh.userData.unit]?.deep).map(mesh => {
    const skin = mesh.geometry.getAttribute('skinIndex'), weights = mesh.geometry.getAttribute('skinWeight');
    const count = mesh.geometry.getAttribute('position').count, segments = [];
    if (!skin || !weights || count !== skin.count || count !== weights.count) throw new Error('Missing surface skin ownership');
    for (let i = 0; i < count; i++) {
      let slot = 0;
      for (let k = 1; k < 4; k++) if (weights.getComponent(i, k) > weights.getComponent(i, slot)) slot = k;
      const seg = mesh.skeleton.bones[skin.getComponent(i, slot)]?.name;
      if (!Object.hasOwn(A.S, seg)) throw new Error('Unknown ground surface owner');
      segments.push(seg);
    }
    return { mesh, count, segments, unit: mesh.userData.unit, side: mesh.userData.side };
  });
  const target = new THREE.Vector3();
  function measure(conf, options = {}) {
    const W = A.computeWorld(conf.local, conf.rootQ, conf.rootP), intent = PRACTICE_SUPPORT_INTENT[conf.id] || [];
    const primary = intent.map(group => ({ ...group, regions: [...group.regions], minimum: null }));
    const minima = { muscle: null, bone: null, marker: null, proxy: null }, counts = { muscle: 0, bone: 0, marker: 0, proxy: 0 };
    const points = options.cloud ? [] : null, floorPoints = [], byRegion = new Map();
    const add = (kind, point, descriptor) => {
      if (!finitePoint(point)) throw new Error('Non-finite ground geometry');
      counts[kind]++;
      const witness = { kind, ...descriptor, heightM: point.y, point: point.toArray() };
      if (!minima[kind] || point.y < minima[kind].heightM) minima[kind] = witness;
      if (points) points.push(witness);
      if (kind !== 'bone') {
        const key = descriptor.region + ':' + (descriptor.side || 'C');
        if (!byRegion.has(key) || point.y < byRegion.get(key).heightM) byRegion.set(key, witness);
        for (const group of primary) if (matchesPracticeSupport(group, descriptor)
            && (!group.minimum || point.y < group.minimum.heightM)) group.minimum = witness;
        if (Math.abs(point.y) <= FLOOR_BAND_M) floorPoints.push([point.x, point.z]);
      }
    };
    surfaces.update(W); surfaces.group?.updateMatrixWorld(true);
    for (const rec of records) for (let i = 0; i < rec.count; i++) {
      rec.mesh.getVertexPosition(i, target); rec.mesh.localToWorld(target);
      const seg = rec.segments[i];
      add('muscle', target, { unit: rec.unit, side: rec.side || sideOf(seg), seg, vertex: i,
        region: practiceSurfaceRegion(rec.unit, seg, target, W) });
    }
    for (const part of bones) {
      const transform = W[part.seg];
      if (!transform || !finitePoint(transform.p)) throw new Error('Invalid ground bone transform');
      for (let i = 0; i < part.positions.length; i += 3) {
        target.fromArray(part.positions, i).applyQuaternion(transform.q).add(transform.p);
        add('bone', target, { part: part.id, seg: part.seg, side: sideOf(part.seg), vertex: i / 3 });
      }
    }
    const proxies = contactFactory?.groundProxies?.(W, { id: conf.id }) || [];
    for (const [index, marker] of A.LANDMARKS.entries()) {
      // D8's former fingertip/palm markers are not registered to the posed
      // distal atlas bones. The explicit bone-adjacent skin model replaces
      // those particular markers, rather than adding a second incompatible skin.
      if (conf.id === 'detali-08' && proxies.length && /^(hand|f[1-5][mabc])_[LR]$/.test(marker.seg)
          && ['ладони', 'пальцы рук', 'кулаки', 'кисти'].includes(marker.region)) continue;
      add('marker', A.pt(W, marker.seg, marker.pc),
        { index, seg: marker.seg, side: marker.side || null, region: marker.region, tag: marker.tag || null });
    }
    for (const proxy of proxies) add('proxy', proxy.point,
      { id: proxy.id, seg: proxy.seg, side: proxy.side || null, region: proxy.region, source: proxy.source });
    const minimum = Object.values(minima).filter(Boolean).reduce((a, p) => !a || p.heightM < a.heightM ? p : a, null);
    if (!minimum) throw new Error('Empty ground geometry');
    const com = A.comOf(W), poly = convexHull(floorPoints);
    return { minimum: copyWitness(minimum), minima, counts, primary: primary.map(group => ({ ...group,
      gapM: group.minimum ? Math.max(0, group.minimum.heightM) : null,
      present: !!group.minimum && Math.abs(group.minimum.heightM) <= FLOOR_BAND_M })),
      byRegion: [...byRegion.values()], geometrySupport: { poly, marginM: supportMargin(poly, [com.x, com.z]),
        definition: 'all displayed outer-muscle vertices and skin markers in the contact band; not force quadrature' },
      com: com.toArray(), ...(points ? { points } : {}) };
  }
  function fit(input, options = {}) {
    if (!input || !Object.hasOwn(PRACTICE_SUPPORT_INTENT, input.id)) throw new Error('Unknown practice ground support intent');
    const conf = cloneConfiguration(input, A, THREE), before = measure(conf, { cloud: !!options.rigidTilt });
    let rigidTilt = null;
    if (options.rigidTilt) {
      const optimum = optimizeGroundTilt(before.points, before.primary, THREE,
        { ...(typeof options.rigidTilt === 'object' ? options.rigidTilt : {}), com: before.com });
      const pivot = new THREE.Vector3().fromArray(before.com);
      conf.rootQ.premultiply(optimum.quaternion).normalize();
      conf.rootP.sub(pivot).applyQuaternion(optimum.quaternion).add(pivot);
      rigidTilt = { pitchDeg: optimum.pitchDeg, rollDeg: optimum.rollDeg, score: optimum.score,
        predictedPrimaryGapsM: optimum.gapsM, predictedGeometryMarginM: optimum.geometryMarginM,
        evaluatedPlanes: optimum.evaluatedPlanes, boundReached: optimum.boundReached };
      delete before.points;
    }
    const tilted = rigidTilt ? measure(conf) : before;
    conf.rootP = groundedRootPosition(conf.rootP, tilted.minimum.heightM, options.clearanceM ?? 0);
    const after = measure(conf);
    const contactResult = options.contactAudit && contactFactory ? contactFactory.buildFloor(A.computeWorld(conf.local, conf.rootQ, conf.rootP), { id: conf.id }) : null;
    const unresolved = after.primary.filter(group => !group.present);
    return { conf, groundDiagnostics: { method: rigidTilt ? 'actual-rendered-geometry-bounded-rigid-fit' : 'actual-rendered-geometry-root-translation',
      scope: ['displayed-outer-muscle-vertices', 'all-original-bone-vertices', 'anatomical-skin-landmarks', 'explicit-estimated-skin-proxies'],
      displacementM: conf.rootP.clone().sub(input.rootP).toArray(),
      rigidTilt,
      before, after, nonpenetrating: after.minimum.heightM >= -2e-6,
      primarySupportsComplete: unresolved.length === 0, unresolvedPrimarySupports: unresolved,
      sourceSkeletonPreserved: true, localKinematicsChanged: false,
      ...(contactResult ? { contactDiagnostics: contactResult.diagnostics } : {}),
      limitations: ['Outer-muscle surfaces and anatomical skin markers approximate the missing full skin.',
        'Translation preserves photographed joint rotations and internal grips; missing primary supports require a separate kinematic fit.',
        'No tissue indentation, measured contact area, or personal ground reaction is inferred.'] } };
  }
  function fitForBuild(input, options = {}) {
    const standard = { rigidTilt: { maxPitchDeg: 6, maxRollDeg: 6 }, ...options };
    const compact = result => {
      for (const key of ['before', 'after']) {
        delete result.groundDiagnostics[key].byRegion;
        delete result.groundDiagnostics[key].points;
      }
      return result;
    };
    if (input.id === 'detali-08') {
      if (!contactFactory?.groundProxies) throw new Error('D8 requires the explicit distal-bone skin model');
      const baseline = fit(input), conf = baseline.conf, targets = [], cumulativeTravel = { L: 0, R: 0 };
      const Z = new THREE.Vector3(0, 0, 1);
      const localTurn = (side, degrees) => {
        const turn = new THREE.Quaternion().setFromAxisAngle(Z, degrees * Math.PI / 180);
        if (side === 'R') { turn.y *= -1; turn.z *= -1; }
        conf.local['hand_' + side].multiply(turn).normalize();
      };
      const placeTips = side => {
        const W = A.computeWorld(conf.local, conf.rootQ, conf.rootP);
        const proxies = contactFactory.groundProxies(W, { id: input.id }).filter(p => p.side === side);
        if (!proxies.length) throw new Error('D8 requires actual distal phalanges for side ' + side);
        const minimumY = Math.min(...proxies.map(p => p.point.y)), wrist = W['hand_' + side].p.clone();
        const dy = .001 - minimumY;
        cumulativeTravel[side] += Math.abs(dy);
        if (cumulativeTravel[side] > .1) throw new Error('D8 support retarget exceeds the bounded 10 cm source fit');
        wrist.y += dy;
        const flex = armIK(conf.local, conf.rootQ, conf.rootP, side, wrist, W['fore_' + side].p);
        orientHand(conf.local, conf.rootQ, conf.rootP, side, W['hand_' + side].q, flex);
        const reached = A.computeWorld(conf.local, conf.rootQ, conf.rootP)['hand_' + side].p;
        const reachErrorM = reached.distanceTo(wrist);
        if (reachErrorM > .002) throw new Error('D8 wrist support target cannot be reached with the unchanged arm lengths');
        targets.push({ side, requestedVerticalChangeM: dy, reachErrorM });
      };
      // Hand orientations and pronation are photographic fit parameters. The
      // original distal bone shapes, lengths and finger-joint poses remain.
      localTurn('L', -10); localTurn('R', 40);
      placeTips('L'); placeTips('R');
      localTurn('R', -2);
      const W = A.computeWorld(conf.local, conf.rootQ, conf.rootP);
      const axis = W.hand_R.p.clone().sub(W.fore_R.p).normalize();
      const foreQ = new THREE.Quaternion().setFromAxisAngle(axis, 20 * Math.PI / 180).multiply(W.fore_R.q).normalize();
      conf.local.fore_R = W.hum_R.q.clone().invert().multiply(foreQ).normalize();
      conf.local.ulna_R = foreQ.clone().invert().multiply(W.ulna_R.q).normalize();
      placeTips('R');
      const result = fit(conf, standard);
      Object.assign(result.groundDiagnostics, { method: 'actual-rendered-geometry-supported-kinematic-fit',
        before: baseline.groundDiagnostics.before, displacementM: result.conf.rootP.clone().sub(input.rootP).toArray(),
        localKinematicsChanged: true, retarget: { profile: 'detali-08-distal-tips',
          handFitTurnsDeg: { L: -10, R: 38 }, rightForearmPronationFitDeg: 20,
          targetProxyHeightM: .001, wristTargets: targets, cumulativeRequestedWristTravelM: cumulativeTravel,
          originalFingerJointPosesPreserved: true, skinProxyOffsetM: .002,
          assumption: 'bounded unmeasured photographic fit to arched fingertip supports; original bone dimensions; no independent finger force or measured grip strength' } });
      return compact(result);
    }
    if (input.id !== 'centering-06') return compact(fit(input, standard));
    // The raised, folded right leg is not an external support in this source
    // image. A small fit within its unmeasured photographic orientation, with
    // both supporting hand targets retained, brings the COM over real support.
    const baseline = fit(input, { rigidTilt: { maxPitchDeg: 6, maxRollDeg: 6 } });
    const conf = baseline.conf, W = A.computeWorld(conf.local, conf.rootQ, conf.rootP);
    const shiftZ = .02, legDeg = 20, shoulder = W.hum_R.p, elbow = W.fore_R.p;
    const remaining = LH * LH - (shoulder.x - elbow.x) ** 2 - (shoulder.z + shiftZ - elbow.z) ** 2;
    if (remaining <= 0) throw new Error('Centred forearm support cannot be reached within the unchanged humerus length');
    conf.rootP.z += shiftZ;
    conf.rootP.y += elbow.y + Math.sign(shoulder.y - elbow.y) * Math.sqrt(remaining) - shoulder.y;
    const targetErrorsM = {};
    for (const side of ['L', 'R']) {
      const flex = armIK(conf.local, conf.rootQ, conf.rootP, side, W['hand_' + side].p, W['fore_' + side].p);
      orientHand(conf.local, conf.rootQ, conf.rootP, side, W['hand_' + side].q, flex);
      targetErrorsM[side] = A.computeWorld(conf.local, conf.rootQ, conf.rootP)['hand_' + side].p.distanceTo(W['hand_' + side].p);
    }
    const movedW = A.computeWorld(conf.local, conf.rootQ, conf.rootP);
    const legRotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), legDeg * Math.PI / 180);
    conf.local.thigh_R = movedW.pelvis.q.clone().invert().multiply(legRotation).multiply(movedW.thigh_R.q).normalize();
    const result = fit(conf, { ...standard, rigidTilt: { maxPitchDeg: 6, maxRollDeg: 6, balance: true } });
    Object.assign(result.groundDiagnostics, { method: 'actual-rendered-geometry-supported-kinematic-fit', before: baseline.groundDiagnostics.before,
      displacementM: result.conf.rootP.clone().sub(input.rootP).toArray(), localKinematicsChanged: true,
      retarget: { profile: 'centering-06-supported-arm-and-free-leg', rootForwardShiftM: shiftZ,
        raisedFoldedFreeLegRotationDeg: legDeg, handTargetErrorsM: targetErrorsM,
        assumption: 'small unmeasured photographic fit; fixed limb lengths, wrist targets and hand orientations; no change to mass or tissue strength' } });
    return compact(result);
  }
  return { measure, fit, fitForBuild, boneParts: bones };
}

export function fitPracticeGround(conf, options) {
  return createPracticeGroundFitter(options).fit(conf, options);
}
