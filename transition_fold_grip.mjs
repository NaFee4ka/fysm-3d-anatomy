// Keep the fold's anatomical wrist/shin relationship when its hip angles are
// adjusted for the animation. Solve in the common body frame so placement on
// the mat does not change the IK pole/twist branch. This is geometry, not force.
import { armIK, orientHand } from './cycle_lib.mjs';

export function followFoldShins({ A, THREE, original, conf }) {
  const clone = input => ({
    local: Object.fromEntries(A.SEG.map(segment => [segment.name, input.local[segment.name].clone()])),
    rootQ: input.rootQ.clone(), rootP: input.rootP.clone(),
  });
  const candidate = clone(conf), root = new THREE.Quaternion(), position = new THREE.Vector3();
  const oldWorld = A.computeWorld(original.local, root, position);
  const newWorld = A.computeWorld(candidate.local, root, position);
  const targets = {};
  for (const side of ['L', 'R']) {
    const shin = 'shank_' + side, hand = 'hand_' + side;
    const attached = oldWorld[hand].p.clone().sub(oldWorld[shin].p).applyQuaternion(oldWorld[shin].q.clone().invert());
    const target = attached.applyQuaternion(newWorld[shin].q).add(newWorld[shin].p);
    const orientation = newWorld[shin].q.clone().multiply(oldWorld[shin].q.clone().invert()).multiply(oldWorld[hand].q);
    const flex = armIK(candidate.local, root, position, side, target, newWorld['fore_' + side].p);
    orientHand(candidate.local, root, position, side, orientation, flex);
    targets[side] = target;
  }
  const posed = A.computeWorld(candidate.local, root, position);
  const maximumTargetErrorM = Math.max(...['L', 'R'].map(side => posed['hand_' + side].p.distanceTo(targets[side])));
  if (!Number.isFinite(maximumTargetErrorM) || maximumTargetErrorM > .0002)
    throw new Error('Fold grip wrist target is outside the verified IK tolerance');
  return { conf: candidate, adjustment: {
    kind: 'fold-grip-follows-shins', extraWristWithdrawalM: 0, maximumTargetErrorM,
    note: 'Для анимации кисти следуют исходным точкам относительно своих голеней. Дополнительного отвода кистей от голеней нет; сила хвата не рассчитана.',
  } };
}
