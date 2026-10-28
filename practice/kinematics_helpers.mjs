// Redundant arm IK: move the elbow around the shoulder–wrist axis while
// retaining the requested wrist position and complete hand orientation.
// This changes the actual upper-arm/forearm configuration, not clinical labels
// or a hidden axial twist assigned to the wrist.
import { A, V, setSide, computeWorld, armIK, orientHand } from '../cycle_lib.mjs';
import { handBasis, fingers } from '../seg5_util.mjs';

export function rotateElbowPlane(pose, side, degrees) {
  const W = computeWorld(pose.local, pose.rootQ, pose.rootP);
  const shoulder = W['hum_' + side].p, wrist = W['hand_' + side].p;
  const axis = wrist.clone().sub(shoulder).normalize();
  const pole = W['fore_' + side].p.clone().sub(shoulder)
    .applyAxisAngle(axis, degrees * Math.PI / 180).add(shoulder);
  const flex = armIK(pose.local, pose.rootQ, pose.rootP, side, wrist, pole);
  orientHand(pose.local, pose.rootQ, pose.rootP, side, W['hand_' + side].q, flex);
}

// A closed wrist lock grips the distal forearm, with the palm and curled
// fingers outside its bones. Build that contact from the actual posed axis.
export function graspDistalForearm(pose, { pole, elbowPlaneDeg = 0, radius = .032, azimuthDeg = 15 } = {}) {
  const W = computeWorld(pose.local, pose.rootQ, pose.rootP);
  const axis = W.hand_R.p.clone().sub(W.fore_R.p).normalize();
  const center = W.hand_R.p.clone().addScaledVector(axis, -.030);
  const tangent = V(-1, 0, 0).addScaledVector(axis, axis.x).normalize();
  const normal = axis.clone().cross(tangent).normalize();
  tangent.applyAxisAngle(axis, azimuthDeg * Math.PI / 180);
  normal.applyAxisAngle(axis, azimuthDeg * Math.PI / 180);
  const Q = handBasis(tangent, normal), palm = center.clone().addScaledVector(normal, -radius);
  const offset = A.lmkTag('palm', 'L').pc.clone().sub(A.S.hand_L.J).applyQuaternion(Q);
  const flex = armIK(pose.local, pose.rootQ, pose.rootP, 'L', palm.sub(offset), pole || W.fore_L.p);
  orientHand(pose.local, pose.rootQ, pose.rootP, 'L', Q, flex);
  if (elbowPlaneDeg) rotateElbowPlane(pose, 'L', elbowPlaneDeg);
  const digits = {};
  fingers(digits, { flex: [[20, 55, 30]], thumb: { abd: 30, flex: 20, mcp: 20, ip: 10 } });
  for (const [name, q] of Object.entries(digits)) setSide(pose.local, 'L', name, q);
}
