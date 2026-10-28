// Release/preview guards for already solved cycle data. This module never runs a
// solver and does not turn solver convergence into anatomical validation.
export const HELD_POSES = Object.freeze(['dog', 'plank', 'half', 'bow', 'fold']);
// Keys are canonical even when the CLI receives a different input filename:
// the cycle key hashes the exact raw bytes supplied to --in / --cycle.
// Only code and inputs occur here, never cycle_phys.json or another output.
export const CYCLE_PHYSICS_INPUT_KEY = 'cycle_keys/cycle.json';
export const CYCLE_PHYSICS_SOURCE_FILES = Object.freeze([
  'anatomy3.js', 'physics_core.mjs', 'physics_params.mjs', 'cycle_phys.mjs',
  'cycle_quality.mjs', 'poses.json', 'package-lock.json', CYCLE_PHYSICS_INPUT_KEY,
]);
const REACTIONS = ['L5S1', 'T12L1', 'hip', 'knee', 'ankle', 'gh', 'elbow', 'wrist'];
const QUALITY_NUMBERS = ['primalResidual', 'dualResidual', 'equalityViolation', 'inequalityViolation', 'boundViolation', 'primalTolerance', 'dualTolerance', 'forceResidualN', 'momentResidualNm'];

function requireTrue(condition, message) {
  if (!condition) throw new Error('Round physics: ' + message);
}
const finite = value => typeof value === 'number' && Number.isFinite(value);
const close = (a, b) => finite(a) && finite(b) && Math.abs(a - b) <= 1e-6;
const sha256 = value => typeof value === 'string' && /^[a-f0-9]{64}$/.test(value);

// Pure validation: callers supply hashes of their current raw inputs. Keeping
// filesystem reads outside this module permits independent synthetic fixtures.
export function validateCyclePhysicsSources(physics, current) {
  for (const [label, hashes] of [['recorded', physics?.sourceSha256], ['current', current]]) {
    requireTrue(hashes && typeof hashes === 'object' && !Array.isArray(hashes)
      && Object.keys(hashes).length === CYCLE_PHYSICS_SOURCE_FILES.length
      && CYCLE_PHYSICS_SOURCE_FILES.every(name => sha256(hashes[name])),
    'missing or invalid ' + label + ' source SHA-256 manifest');
  }
  for (const name of CYCLE_PHYSICS_SOURCE_FILES) requireTrue(physics.sourceSha256[name] === current[name],
    'stale physics source ' + name + '; rerun ./cycle_run.sh');
  return { ...physics.sourceSha256 };
}

function numericArray(array, count, name) {
  requireTrue(Array.isArray(array) && array.length === count && array.every(finite), 'invalid numeric array ' + name);
}

function uniqueNames(names, expected, label) {
  requireTrue(Array.isArray(names) && names.length > 0 && names.every(n => typeof n === 'string')
    && new Set(names).size === names.length, 'invalid or duplicate ' + label);
  if (expected) requireTrue(names.length === expected.length && expected.every(name => names.includes(name)), 'stale ' + label + ' coverage');
}

function checkRecord(record, units, ligs, label) {
  const quality = record && record.quality;
  requireTrue(quality && quality.status === 'solved' && quality.converged === true && quality.finite === true,
    'unverified solution ' + label);
  requireTrue(QUALITY_NUMBERS.every(name => finite(quality[name]) && quality[name] >= 0),
    'missing or non-finite quality metric ' + label);
  requireTrue(quality.primalResidual <= quality.primalTolerance && quality.dualResidual <= quality.dualTolerance
    && quality.equalityViolation <= quality.primalTolerance && quality.inequalityViolation <= quality.primalTolerance
    && quality.boundViolation <= quality.primalTolerance, 'quality flags contradict numerical tolerances ' + label);
  requireTrue(quality.physicalBalanceVerified === true && record.physicalBalance?.verified === true
    && record.physicalBalance.assemblyVerified === true,
    'physical force-path equilibrium was not checked ' + label);
  requireTrue(['forceResidualN', 'momentResidualNm', 'forceToleranceN', 'momentToleranceNm', 'maxAssemblyDifference'].every(name =>
    finite(record.physicalBalance[name]) && record.physicalBalance[name] >= 0)
    && record.physicalBalance.maxAssemblyDifference <= 1e-7
    && record.physicalBalance.forceResidualN <= record.physicalBalance.forceToleranceN
    && record.physicalBalance.momentResidualNm <= record.physicalBalance.momentToleranceNm,
    'physical force-path assembly disagrees with equilibrium ' + label);
  for (const side of ['L', 'R']) {
    numericArray(record['a' + side], units, label + '/a' + side);
    requireTrue(record['a' + side].every(value => value >= 0 && value <= 1), 'effort outside [0,1] ' + label);
    numericArray(record['s' + side], units, label + '/s' + side);
    numericArray(record['lig' + side], ligs, label + '/lig' + side);
  }
  for (const reaction of REACTIONS) numericArray(record.jr && record.jr[reaction], 2, label + '/jr/' + reaction);
  numericArray(record.com, 3, label + '/com');
  requireTrue(finite(record.reserves) && record.reserves >= 0 && finite(record.resid) && record.resid >= 0,
    'invalid reserve/residual diagnostics ' + label);
  requireTrue(Number.isInteger(record.modelLimits?.forceLengthClampedStrands)
    && record.modelLimits.forceLengthClampedStrands >= 0, 'missing force-length diagnostics ' + label);
  requireTrue(Array.isArray(record.floor) && record.floor.length > 0, 'missing floor forces ' + label);
  for (const [index, floor] of record.floor.entries()) {
    numericArray(floor.F, 3, label + '/floor/' + index);
    requireTrue(Array.isArray(floor.cop) && [2, 3].includes(floor.cop.length) && floor.cop.every(finite), 'invalid floor centre ' + label);
  }
  for (const [index, grip] of (record.grips || []).entries()) {
    numericArray(grip.F, 3, label + '/grip-force/' + index);
    numericArray(grip.P, 3, label + '/grip-point/' + index);
  }
}

export function validateCyclePhysics(cycle, physics, expected = {}) {
  if (!physics) return null; // A geometry-only preview keeps its placeholder flag.
  const sourceSha256 = validateCyclePhysicsSources(physics, expected.sourceSha256);
  requireTrue(sha256(expected.physicsSha256), 'missing or invalid raw physics SHA-256');
  requireTrue(Array.isArray(cycle.frames) && cycle.frames.length >= 2 && finite(cycle.fps) && cycle.fps > 0,
    'invalid kinematic frame grid');
  const count = cycle.frames.length;
  // The packed format samples i/fps; merely matching an irregular physics
  // timestamp to an equally irregular source timestamp would certify the wrong
  // interpolation times in the browser.
  requireTrue(close(cycle.T, (count - 1) / cycle.fps)
    && cycle.frames.every((frame, i) => close(frame.T, i / cycle.fps)), 'nonuniform kinematic frame grid');
  requireTrue(close(physics.fps, cycle.fps) && close(physics.T, cycle.T), 'stale duration or frame rate');
  requireTrue(Array.isArray(physics.frames) && physics.frames.length > 0, 'no solved frames');
  uniqueNames(physics.units, expected.units, 'muscle units');
  uniqueNames(physics.ligs, expected.ligs, 'ligament units');
  const every = physics.opts && physics.opts.every;
  requireTrue(Number.isInteger(every) && every >= 1, 'invalid sampling metadata');
  let previousIndex = -1, previousTime = -Infinity;
  for (const frame of physics.frames) {
    requireTrue(Number.isInteger(frame.i) && frame.i >= 0 && frame.i < count && frame.i > previousIndex,
      'duplicate, out-of-order or out-of-range frame index');
    const reference = cycle.frames[frame.i];
    requireTrue(close(frame.T, reference.T) && frame.T > previousTime
      && frame.seg === reference.seg && close(frame.t, reference.t), 'stale frame time or segment at index ' + frame.i);
    checkRecord(frame, physics.units.length, physics.ligs.length, 'frame ' + frame.i);
    previousIndex = frame.i; previousTime = frame.T;
  }
  requireTrue(physics.frames[0].i === 0 && physics.frames.at(-1).i === count - 1,
    'missing first/last frame; extrapolated partial clips cannot be published');
  const completeIndices = physics.frames.length === count && physics.frames.every((frame, i) => frame.i === i);
  requireTrue(every > 1 || completeIndices, 'missing frame in a full-resolution release');
  const holds = cycle.holds || [];
  requireTrue(HELD_POSES.every(id => holds.some(hold => hold.pose === id)) && holds.length === HELD_POSES.length,
    'kinematics does not contain the five held poses');
  requireTrue(holds.every(hold => finite(hold.T) && hold.T >= 0 && hold.T <= cycle.T
    && close(hold.T, Math.round(hold.T * cycle.fps) / cycle.fps)), 'held pose is not on the kinematic frame grid');
  requireTrue(physics.poses && HELD_POSES.every(id => physics.poses[id]), 'missing held-pose solution');
  requireTrue(Array.isArray(physics.holds) && physics.holds.length === holds.length
    && holds.every(hold => physics.holds.some(p => p.pose === hold.pose && close(p.T, hold.T))), 'stale held-pose timing');
  for (const id of HELD_POSES) checkRecord(physics.poses[id], physics.units.length, physics.ligs.length, 'held pose ' + id);
  const fullCoverage = completeIndices && every === 1;
  return {
    verified: fullCoverage, fullCoverage, solvedFramesVerified: true,
    sourcesVerified: true, sourceSha256, physicsSha256: expected.physicsSha256,
    frameCount: physics.frames.length, publishedFrameCount: count, heldPoseCount: HELD_POSES.length,
    maxForceResidualN: Math.max(...physics.frames.map(frame => frame.quality.forceResidualN)),
    maxMomentResidualNm: Math.max(...physics.frames.map(frame => frame.quality.momentResidualNm)),
    physicalBalanceVerified: true,
    maxPhysicalForceResidualN: Math.max(...physics.frames.map(frame => frame.physicalBalance.forceResidualN)),
    maxPhysicalMomentResidualNm: Math.max(...physics.frames.map(frame => frame.physicalBalance.momentResidualNm)),
    maxForceAssemblyDifference: Math.max(...physics.frames.map(frame => frame.physicalBalance.maxAssemblyDifference)),
    maxReserveTotalNm: Math.max(...physics.frames.map(frame => frame.reserves)),
    forceLengthLimitedFrames: physics.frames.filter(frame => frame.modelLimits.forceLengthClampedStrands > 0).length,
    forceLengthExtrapolatedFrames: physics.frames.filter(frame => frame.modelLimits.forceLengthOutsideTypicalStrands > 0).length,
    unidentifiedUnits: [...new Set(physics.frames.flatMap(frame => frame.modelLimits.unidentifiedUnits || []))].sort(),
    modelLimits: {
      unidentifiedUnits: [...new Set(physics.frames.flatMap(frame => frame.modelLimits.unidentifiedUnits || []))].sort(),
      pathQuality: {
        quarantinedFrames: physics.frames.filter(frame => frame.modelLimits.pathQuality?.quarantinedStrands > 0).length,
        maxQuarantinedStrands: Math.max(0, ...physics.frames.map(frame => frame.modelLimits.pathQuality?.quarantinedStrands || 0)),
        quarantinedStrands: Math.max(0, ...physics.frames.map(frame => frame.modelLimits.pathQuality?.quarantinedStrands || 0)),
        invalidPaths: [...new Map(physics.frames.flatMap(frame => frame.modelLimits.pathQuality?.invalidPaths || [])
          .map(item => [item.id, item])).values()],
        quarantinedUnits: [...new Set(physics.frames.flatMap(frame => frame.modelLimits.pathQuality?.quarantinedUnits || []))].sort(),
        maxDerivativeMismatchM: Math.max(0, ...physics.frames.map(frame => frame.modelLimits.pathQuality?.maxDerivativeMismatchM || 0)),
        maxDisabledNominalCapacityN: Math.max(0, ...physics.frames.map(frame => frame.modelLimits.pathQuality?.disabledNominalCapacityN || 0)),
      },
      constitutive: {
        assumption: 'rigid-tendon-relative-to-standing', fibreFractionFloorApplied: false, forceLengthClamped: false,
        slackFiberFrames: physics.frames.filter(frame => frame.modelLimits.constitutive?.slackFiberStrands > 0).length,
        maxSlackFiberStrands: Math.max(0, ...physics.frames.map(frame => frame.modelLimits.constitutive?.slackFiberStrands || 0)),
        slackFiberStrands: Math.max(0, ...physics.frames.map(frame => frame.modelLimits.constitutive?.slackFiberStrands || 0)),
      },
    },
    friction: { kind: 'inscribed-coulomb-polygon', facets: 16 },
    recruitment: 'prescribed-cues', effortDefinition: 'active-force-over-maximum-isometric-force', dynamics: 'quasi-static',
  };
}

export function packJointLoads(values) {
  requireTrue(values.length > 0 && Array.from(values).every(finite), 'missing or non-finite joint loads');
  const maximum = Math.max(0, ...values);
  // Preserve the historical 0.02 BW resolution unless the actual maximum would
  // clip at 5.1 BW. The existing browser decoder already reads this scale.
  const integerScale = Math.floor(255 / (maximum + 1e-10));
  const scale = maximum <= 5.1 ? 50 : integerScale >= 1 ? Math.min(50, integerScale) : 255 / (maximum + 1e-8);
  const packed = Uint8Array.from(values, value => {
    const encoded = Math.round(Math.max(0, value) * scale);
    requireTrue(encoded >= 0 && encoded <= 255, 'joint load exceeds selected quantization range');
    return encoded;
  });
  return { values: packed, scale };
}
