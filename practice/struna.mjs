// Static photographic reconstructions of FYSM «Струна», PDF page 4.
// Angles are construction parameters, not measured subject kinematics.  The
// existing rig, anatomical attachments and material parameters stay unchanged.
import {
  A, V, X, Y, Z, IDQ, qAx, blankLocal, setSide,
  computeWorld, pt, rootPlaced, armIK, orientHand, legIK, orientFoot, LANDMARKS,
  scapulaPose, mirrorQ,
} from '../cycle_lib.mjs';
import { fingers, SPREAD, SOFT, GRIP, handBasis } from '../seg5_util.mjs';

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
function lockBehind(c) {
  shoulders(c.local, 5, 7, -12); digits(c.local, GRIP);
  const target = bodyPoint(c, [.145, 1.12, -.49]);
  const pole = bodyPoint(c, [.29, 1.20, -.24]);
  // Hands approach beside one another, preserving separate palm volumes.
  const Q = handBasis(V(-1, -.12, 0), V(0, 0, -1));
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
  for (const side of ['L', 'R']) {
    const f = {};
    fingers(f, side === 'L' ? { flex: [[0, 55, 35]], thumb: { abd: 35, flex: 25, mcp: 25, ip: 20 } } : SOFT);
    for (const [name, q] of Object.entries(f)) setSide(c.local, side, name, q);
    const target = W['hand_' + side].p.clone(); target.x = side === 'L' ? .11 : -.05;
    if (side === 'L') { target.y += .012; target.z += .025; }
    const Q = side === 'L' ? W.hand_L.q : mirrorQ(handBasis(V(1, 0, 0), V(0, 0, -1)));
    arm(c, side, target, W['fore_' + side].p, Q);
  }
}
function handsBehindHead(c) {
  shoulders(c.local, 28, 8, 2); digits(c.local, SOFT);
  const W = world(c), h = W.head.p;
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    const target = h.clone().add(V(sign * .115, .030, -.090).applyQuaternion(W.head.q));
    const pole = bodyPoint(c, [sign * .45, 1.63, -.07]);
    const Ql = handBasis(V(-.20, 1, .05), V(0, 0, 1));
    const Q = W.head.q.clone().multiply(side === 'L' ? Ql : mirrorQ(Ql));
    arm(c, side, target, pole, Q);
  }
}

function upwardLock() {
  const c = base(-2);
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) c.local[k] = qAx(X, -2);
  shoulders(c.local, 45, 16, -2); digits(c.local, GRIP);
  pair(c.local, 'foot', qAx(X, 28)); pair(c.local, 'toes', qAx(X, -37));
  const W = world(c), sh = W.hum_L.p;
  symmetricArm(c, V(.145, sh.y + .49, sh.z - .19), V(.22, sh.y + .25, sh.z - .08), handBasis(V(-1, .2, 0), V(0, .2, 1)));
  balanceFeet(c, 28);
  closeFingerLock(c, { width: .125, dy: .012, flex: 25, bend: 45, spread: 30 });
  return place(c, [-1, .20, .55], ['Пальцевый замок и подъём пяток восстановлены по фотографии; индивидуальное переплетение пальцев приближённое.']);
}
function backwardLock() {
  const c = base(-1);
  for (const k of ['lumA', 'lumB', 'thA', 'thB', 'thC']) c.local[k] = qAx(X, -1.2);
  lockBehind(c);
  balanceFeet(c);
  closeFingerLock(c, { width: .155, dy: .008, dz: .005, flex: 0, bend: 45, spread: 30 });
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
  const c = base(45); c.rootP = V(0, .235, 0);
  shoulders(c.local, 5, 5, 12); digits(c.local, SPREAD);
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    legIK(c.local, c.rootQ, c.rootP, side, V(sign * .025, side === 'L' ? .34 : .41, .45), V(sign * .45, .65, .30));
    const Q = handBasis(V(0, 0, 1), V(0, -1, 0));
    const wl = palmWrist('L', V(.285, .003, .17), Q);
    arm(c, side, V(sign * wl.x, wl.y, wl.z), V(sign * .33, .36, -.04), side === 'L' ? Q : mirrorQ(Q));
  }
  feet(c, -15);
  return place(c, [-1, .28, .75], ['Контакт бёдер с руками — геометрическая реконструкция; давление тканей не вычислено.']);
}
function sumoTwist() {
  const c = base(40); c.rootP = V(0, .575, 0);
  c.local.thA = qAx(Y, -13).multiply(qAx(Z, -5));
  c.local.thB = qAx(Y, -13).multiply(qAx(Z, -5));
  c.local.thC = qAx(Y, -10).multiply(qAx(Z, -5));
  c.local.neckA = qAx(Y, -8); c.local.head = qAx(Y, -10);
  shoulders(c.local, 8, 5, 8); digits(c.local, SOFT);
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1;
    legIK(c.local, c.rootQ, c.rootP, side, V(sign * .56, .082, .10), V(sign * .7, .55, .33));
    orientFoot(c.local, c.rootQ, c.rootP, side, qAx(Y, sign * 45));
    const W = world(c), knee = W['shank_' + side].p;
    const Ql = handBasis(V(.45, 0, .8), V(0, -1, 0));
    const Q = side === 'L' ? Ql : mirrorQ(Ql);
    arm(c, side, palmWrist(side, knee.clone().add(V(0, .055, .015)), Q), knee.clone().add(V(sign * -.05, .22, .12)), Q);
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
