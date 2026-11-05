// A display-only correction for the saved bow grip. The registered finger
// bones originally cross the shin. A small, symmetric wrist deviation clears
// them while preserving the wrist target, elbow plane and every finger angle.
// It does not alter the source pose or establish a mechanically loaded grasp.
export function repairBowGrip({ A, THREE, conf, accept = () => true }) {
  if (!A || !THREE || !conf?.local || typeof accept !== 'function')
    throw new Error('Invalid bow grip repair input');
  const local = Object.fromEntries(A.SEG.map(segment => {
    const q = conf.local[segment.name];
    if (!q || ![q.x, q.y, q.z, q.w].every(Number.isFinite) || q.lengthSq() < 1e-12)
      throw new Error('Invalid bow grip segment: ' + segment.name);
    return [segment.name, q.clone()];
  }));
  const candidate = { local, rootQ: conf.rootQ.clone(), rootP: conf.rootP.clone() };
  const delta = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 18);
  candidate.local.hand_L.multiply(delta).normalize();
  candidate.local.hand_R.multiply(A.mirrorQ(delta)).normalize();
  if (!accept(candidate)) return null;
  return { conf: candidate, adjustment: {
    kind: 'bilateral-wrist-deviation-for-bone-clearance', degrees: -10,
    wristTranslationM: 0, elbowPlaneChangeDegrees: 0, fingerAnglesPreserved: true,
    note: 'Для анимации положение кистей изменено на 10°. Цели запястий, положение локтей и углы пальцев сохранены; сила хвата не рассчитана.',
  } };
}
