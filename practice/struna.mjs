// Static photographic reconstructions of FYSM «Струна», PDF page 4.
// Angles are construction parameters, not measured subject kinematics.  The
// existing rig, anatomical attachments and material parameters stay unchanged.
import {
  A, V, X, Y, Z, IDQ, qAx, blankLocal, setSide,
  computeWorld, pt, rootPlaced, armIK, orientHand, legIK, orientFoot, LANDMARKS,
  scapulaPose, mirrorQ,
} from '../cycle_lib.mjs';
import { fingers, SPREAD, SOFT, GRIP, handBasis } from '../seg5_util.mjs';
import { rotateElbowPlane, graspDistalForearm } from './kinematics_helpers.mjs';

function pair(L, name, q) { setSide(L, 'L', name, q); setSide(L, 'R', name, q); }
function digits(L, style = SOFT) {
  const left = {}; fingers(left, style);
  for (const [name, q] of Object.entries(left)) pair(L, name, q);
}
function shoulders(L, up = 12, post = 4, prot = 0) {
  const sp = scapulaPose(up, post, prot, null, up * .45, 0);
  pair(L, 'clav', sp.clav); pair(L, 'scap', sp.scap);
}
const world = c => computeWorld(c.local, c.rootQ, c.rootP);
function base(pitch = 0) {
  const c = { local: blankLocal(), rootQ: qAx(X, pitch), rootP: V(0, 0, 0) };
  shoulders(c.local); digits(c.local);
  return c;
}
function place(c, view = [-1, .28, .55], notes = []) {
  c.rootP = rootPlaced(c.local, c.rootQ);
  for (const q of Object.values(c.local)) q.normalize();
  c.rootQ.normalize();
  return { ...c, view, notes };
}
function bodyPoint(c, point) { return pt(world(c), 'pelvis', V(...point)); }
function arm(c, side, target, pole, handQ) {
  const flex = armIK(c.local, c.rootQ, c.rootP, side, target, pole);
  if (handQ) orientHand(c.local, c.rootQ, c.rootP, side, handQ, flex);
}
function symmetricArm(c, target, pole, handQ) {
  arm(c, 'L', target, pole, handQ);
  arm(c, 'R', V(-target.x, target.y, target.z), V(-pole.x, pole.y, pole.z), handQ ? mirrorQ(handQ) : null);
}
function palmWrist(side, center, Q) {
  const landmark = A.LANDMARKS.find(l => l.tag === 'palm' && l.side === side);
  const offset = landmark.pc.clone().sub(A.S[landmark.seg].J).applyQuaternion(Q);
  return center.clone().sub(offset);
}
function feet(c, plantar = 0) {
  for (const side of ['L', 'R']) {
    orientFoot(c.local, c.rootQ, c.rootP, side, qAx(X, plantar));
    setSide(c.local, side, 'toes', qAx(X, -plantar));
  }
}
function balanceFeet(c, plantar = 0) {
  // Keep the intended sole/ball stance while fitting the whole-body pitch to
  // its support. Translation alone cannot bring a COM over its own feet.
  for (let i = 0; i < 14; i++) {
    feet(c, plantar); c.rootP = rootPlaced(c.local, c.rootQ);
    const W = world(c), com = A.comOf(W);
    const ps = A.LANDMARKS.filter(l => l.side === 'L' && (plantar ? l.seg === 'toes_L' : ['heel', 'ball'].includes(l.tag))).map(l => pt(W, l.seg, l.pc));
    const z = ps.reduce((n, p) => n + p.z, 0) / ps.length;
    const error = z - com.z;
    if (Math.abs(error) < .0002) break;
    const step = Math.max(-4, Math.min(4, error / Math.max(.3, com.y) * 180 / Math.PI));
    c.rootQ.premultiply(qAx(X, step));
  }
}
function lockBehind(c, palmRoll = 0) {
  shoulders(c.local, 5, 7, -12); digits(c.local, GRIP);
  const target = bodyPoint(c, [.145, 1.12, -.49]);
  const pole = bodyPoint(c, [.29, 1.20, -.24]);
  // Hands approach beside one another, preserving separate palm volumes.
  const fingerDirection = V(-1, -.12, 0).normalize();
  const palmNormal = V(0, 0, -1).applyAxisAngle(fingerDirection, palmRoll * Math.PI / 180);
  const Q = handBasis(fingerDirection, palmNormal);
  symmetricArm(c, target, pole, Q);
}
function closeFingerLock(c, { width, dy, dz = 0, flex, bend, spread }) {
  const W = world(c);
  digits(c.local, { spread: [0, spread, spread / 2, spread / 2, spread], flex: [[flex, bend, 35]], thumb: { abd: 25, flex: 25, mcp: 15, ip: 10 } });
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    const target = W['hand_' + side].p.clone();
    target.x = sign * width; target.y += sign * dy / 2; target.z += sign * dz / 2;
    arm(c, side, target, W['fore_' + side].p, W['hand_' + side].q);
  }
}
function closeWristLock(c) {
  const W = world(c);
  // The free right hand continues inward instead of folding back toward its
  // own elbow. Its palm roll and elbow branch keep pronation and wrist motion
  // inside the model ranges; no range or tissue stiffness is enlarged.
  const rightTarget = W.hand_R.p.clone(); rightTarget.x = -.05;
  const rightQ = qAx(X, -180).multiply(mirrorQ(handBasis(V(-1, 0, 0), V(0, 0, -1))));
  arm(c, 'R', rightTarget, W.fore_R.p, rightQ);
  rotateElbowPlane(c, 'R', 45);
  // This is an explicit wrist grip around the distal forearm, rather than
  // fingers entering the carpal bones of a reversed free hand.
  graspDistalForearm(c, { pole: W.fore_L.p, elbowPlaneDeg: -30 });
  const free = {}; fingers(free, SOFT);
  for (const [name, q] of Object.entries(free)) setSide(c.local, 'R', name, q);
}
function handsBehindHead(c) {
  shoulders(c.local, 28, 8, 2); digits(c.local, GRIP);
  const W = world(c), h = W.head.p;
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    const target = h.clone().add(V(sign * .15, .030 + sign * .015, -.14 + sign * .007).applyQuaternion(W.head.q));
    const pole = bodyPoint(c, [sign * .45, 1.63, -.07]);
    // Fingers run across the occiput; pointing them along the neck produced
    // almost ninety degrees of ulnar wrist deviation in the old reconstruction.
    const Ql = handBasis(V(-.95, .30, 0), V(0, 0, 1));
    const Q = W.head.q.clone().multiply(side === 'L' ? Ql : mirrorQ(Ql));
    arm(c, side, target, pole, Q);
    rotateElbowPlane(c, side, side === 'L' ? 30 : -30);
  }
}

function upwardLock() {
  const c = base(-2);
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) c.local[k] = qAx(X, -2);
  shoulders(c.local, 45, 16, -2); digits(c.local, GRIP);
  pair(c.local, 'foot', qAx(X, 28)); pair(c.local, 'toes', qAx(X, -37));
  const W = world(c), sh = W.hum_L.p;
  symmetricArm(c, V(.145, sh.y + .52, sh.z - .19), V(.22, sh.y + .25, sh.z - .08), handBasis(V(-1, 1.7, 0), V(0, .2, 1)));
  balanceFeet(c, 28);
  closeFingerLock(c, { width: .09, dy: .012, flex: 25, bend: 45, spread: 30 });
  return place(c, [-1, .20, .55], ['Пальцевый замок и подъём пяток восстановлены по фотографии; индивидуальное переплетение пальцев приближённое.']);
}
function backwardLock() {
  const c = base(-1);
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) c.local[k] = qAx(X, -1.2);
  // A palm roll and the matching elbow branches keep the backward lock from
  // demanding 120 degrees of humeral internal rotation. The finger phases
  // follow the rolled palms so the interlace retains separate bone volumes.
  lockBehind(c, -120);
  balanceFeet(c);
  closeFingerLock(c, { width: .155, dy: .01, dz: -.01, flex: 0, bend: 45, spread: 30 });
  rotateElbowPlane(c, 'L', -60); rotateElbowPlane(c, 'R', 60);
  return place(c, [-1, .24, .6], ['Пальцевый замок за спиной показан геометрической реконструкцией.']);
}
function chair(wings = false) {
  const c = base(wings ? 72 : 35);
  const hip = wings ? 145 : 104, knee = wings ? 85 : 90;
  pair(c.local, 'thigh', qAx(X, -hip)); pair(c.local, 'shank', qAx(X, knee));
  feet(c, wings ? 22 : 0);
  if (wings) {
    shoulders(c.local, 4, 6, -10);
    const W = world(c), sh = W.hum_L.p;
    const target = sh.clone().add(V(.07, -.13, -.51));
    symmetricArm(c, target, sh.clone().add(V(.13, -.05, -.25)), handBasis(V(0, 0, -1), V(0, 1, 0)));
  } else lockBehind(c);
  balanceFeet(c, wings ? 22 : 0);
  if (!wings) closeWristLock(c);
  return place(c, [-1, .23, .5], wings ? ['Пятки подняты, ладони обращены вверх, как на исходной фотографии.'] : ['Запястный замок за спиной восстановлен по фотографии.']);
}
function spider() {
  // Keep shoulders closer to the supports instead of putting almost straight
  // forearms behind the palms and demanding 114 degrees of wrist extension.
  const c = base(15); c.rootP = V(0, .115, 0);
  shoulders(c.local, 5, 5, 12); digits(c.local, SPREAD);
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    legIK(c.local, c.rootQ, c.rootP, side, V(sign * .025, side === 'L' ? .34 : .41, .45), V(sign * .45, .65, .30));
    const Q = handBasis(V(0, 0, 1), V(0, -1, 0));
    const wl = palmWrist('L', V(.285, .003, .14), Q);
    arm(c, side, V(sign * wl.x, wl.y, wl.z), V(sign * .33, .36, -.04), side === 'L' ? Q : mirrorQ(Q));
  }
  feet(c, -15);
  return place(c, [-1, .28, .75], ['Контакт бёдер с руками — геометрическая реконструкция; давление тканей не вычислено.']);
}
function sumoTwist() {
  const c = base(40); c.rootP = V(0, .575, 0);
  // Retain the reconstructed total trunk rotation, but share it across all
  // five model joints. Concentrating 13 degrees at T8–T9 exceeded that joint's
  // 10-degree range and generated an artificial exponential passive moment.
  const trunkRotation = qAx(Y, -13).multiply(qAx(Z, -5))
    .multiply(qAx(Y, -13).multiply(qAx(Z, -5)))
    .multiply(qAx(Y, -10).multiply(qAx(Z, -5)));
  const trunkStep = IDQ.clone().slerp(trunkRotation, 1 / 5);
  for (const segment of ['lumA', 'lumB', 'thA', 'thB', 'thC']) c.local[segment] = trunkStep.clone();
  c.local.neckA = qAx(Y, -8); c.local.head = qAx(Y, -10);
  shoulders(c.local, 8, 5, 8); digits(c.local, SOFT);
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    legIK(c.local, c.rootQ, c.rootP, side, V(sign * .56, .082, .10), V(sign * .7, .55, .33));
    orientFoot(c.local, c.rootQ, c.rootP, side, qAx(Y, sign * 45));
    const W = world(c), knee = W['shank_' + side].p;
    const yaw = (side === 'L' ? 20 : 30) * Math.PI / 180;
    const Ql = handBasis(V(.45, 0, .8).applyAxisAngle(Y, yaw), V(0, -1, 0));
    const Q = side === 'L' ? Ql : mirrorQ(Ql);
    arm(c, side, palmWrist(side, knee.clone().add(V(0, .055, .015)), Q), knee.clone().add(V(sign * -.05, .22, .12)), Q);
    rotateElbowPlane(c, side, side === 'L' ? -105 : 135);
  }
  return place(c, [0.2, .35, 1], ['Сторона скрутки восстановлена по фотографии; поза асимметричная.']);
}
function singleLegLowPlank() {
  const c = base(84); c.rootP = V(0, .245, 0);
  shoulders(c.local, 8, 4, 12); digits(c.local, SPREAD);
  pair(c.local, 'foot', qAx(X, -15)); pair(c.local, 'toes', qAx(X, -65));
  // Right leg stays extended on the ball of its foot. Left knee approaches
  // the ipsilateral upper arm, rather than crossing through the other arm.
  legIK(c.local, c.rootQ, c.rootP, 'L', V(.17, .27, .10), V(.50, .44, .63));
  for (let it = 0; it < 8; it++) {
    for (const side of ['L', 'R']) {
      const sign = side === 'L' ? 1 : -1;
      const Q = handBasis(V(0, 0, 1), V(0, -1, 0));
      const wl = palmWrist('L', V(.25, .003, .48), Q);
      arm(c, side, V(sign * wl.x, wl.y, wl.z), V(sign * .26, .36, .15), side === 'L' ? Q : mirrorQ(Q));
    }
    const W = world(c);
    const min = pred => Math.min(...LANDMARKS.filter(pred).map(l => pt(W, l.seg, l.pc).y));
    const h = min(l => /^(hand|f\d)/.test(l.seg)), f = min(l => l.side === 'R' && /^(foot|toes)/.test(l.seg));
    if (Math.abs(f - h) < .0001) break;
    c.rootQ.premultiply(qAx(X, (h - f) * 57.3 / .88));
  }
  // Fit the bent knee to the posed upper arm, rather than hoping that an
  // arbitrary ankle target and pole will produce the photographed contact.
  // Keep both segment lengths and use the old ankle only for the free shin's
  // direction; neither joint is translated independently of its parent.
  const W = world(c), hip = W.thigh_L.p;
  const upper = W.hum_L.p.clone().lerp(W.fore_L.p, .25);
  const thighLength = A.S.shank_L.J.distanceTo(A.S.thigh_L.J);
  const shankLength = A.S.foot_L.J.distanceTo(A.S.shank_L.J);
  const knee = upper.add(V(.035, .095, 0)).sub(hip).normalize().multiplyScalar(thighLength).add(hip);
  const ankle = W.foot_L.p.clone().sub(knee).normalize().multiplyScalar(shankLength).add(knee);
  legIK(c.local, c.rootQ, c.rootP, 'L', ankle, knee);
  // The airborne shin can open without moving the supporting knee or hip.
  // The original arbitrary ankle direction folded this knee to 173 degrees;
  // 147.5 is a photographic reconstruction, not a measured subject angle.
  c.local.shank_L = qAx(X, 147.5);
  c.local.foot_L = qAx(X, 15); c.local.toes_L = IDQ.clone();
  return place(c, [-1, .22, .6], ['Односторонний вариант: левое колено к плечу, правая нога вытянута.']);
}
function seatedBoat() {
  const c = base(-54);
  pair(c.local, 'thigh', qAx(X, -61)); pair(c.local, 'shank', IDQ);
  pair(c.local, 'foot', qAx(X, -12));
  c.local.lumA = qAx(X, 3); c.local.lumB = qAx(X, 3);
  handsBehindHead(c);
  for (let i = 0; i < 8; i++) {
    c.rootP = rootPlaced(c.local, c.rootQ);
    const W = world(c), com = A.comOf(W);
    const butt = A.LANDMARKS.find(l => l.region === 'ягодицы');
    const p = pt(W, butt.seg, butt.pc), error = p.z - com.z;
    if (Math.abs(error) < .0001) break;
    c.rootQ.premultiply(qAx(X, error / Math.max(.1, com.y - p.y) * 57.3));
  }
  return place(c, [-1, .22, .55], ['Руки за головой и прямые поднятые ноги восстановлены по фотографии.']);
}

export function buildStruna() {
  return {
    'struna-01': upwardLock(),
    'struna-02': backwardLock(),
    'struna-03': chair(false),
    'struna-04': chair(true),
    'struna-05': spider(),
    'struna-06': sumoTwist(),
    'struna-07': singleLegLowPlank(),
    'struna-08': seatedBoat(),
  };
}
