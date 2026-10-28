// Static photographic reconstructions of FYSM «Детали», PDF page 5.
// Angles are reconstruction parameters, not measurements or training advice.
// This module does not modify the anatomy or infer recruitment from a photograph.
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import {
  A, THREE, V, S, SEG, qAx, X, Y, Z, IDQ, blankLocal, setSide,
  armIK, legIK, orientHand, orientFoot, computeWorld, pt,
  rootPlaced, scapulaPose,
} from '../cycle_lib.mjs';

const ZERO = V(0, 0, 0);
const SIDES = ['L', 'R'];
const sign = s => s === 'L' ? 1 : -1;
let sourceBoneParts = null;
function boneParts() {
  if (sourceBoneParts) return sourceBoneParts;
  const data = JSON.parse(readFileSync(new URL('../skull/bones_bp3d.json', import.meta.url)));
  const payload = gunzipSync(Buffer.from(data.data, 'base64'));
  let offset = 0;
  sourceBoneParts = data.parts.map(part => {
    const points = [], accumulator = [0, 0, 0];
    for (let i = 0; i < part.n; i++) {
      const point = [];
      for (let axis = 0; axis < 3; axis++) {
        accumulator[axis] = (accumulator[axis] + payload.readUInt16LE(offset)) & 65535;
        offset += 2;
        point.push(part.lo[axis] + (part.hi[axis] - part.lo[axis]) * accumulator[axis] / 65535);
      }
      points.push(V(...point));
    }
    offset += part.m * 6;
    return { seg: part.seg, points };
  });
  if (offset !== payload.length) throw new Error('Practice bone-floor payload length mismatch');
  return sourceBoneParts;
}
function boneFloor(W, predicate) {
  let lowest = Infinity;
  for (const part of boneParts()) if (predicate(part.seg)) {
    const transform = W[part.seg];
    for (const point of part.points)
      lowest = Math.min(lowest, point.clone().applyQuaternion(transform.q).add(transform.p).y);
  }
  return lowest;
}
function world(k) { return computeWorld(k.local, k.rootQ, ZERO); }
function base(pitch = 0) { return { local: blankLocal(), rootQ: qAx(X, pitch), rootP: null }; }
function both(k, bone, rotation) { for (const side of SIDES) setSide(k.local, side, bone, rotation); }
function spine(k, angles, neck = [0, 0, 0]) {
  ['lumA', 'lumB', 'thA', 'thB', 'thC'].forEach((name, i) => { k.local[name] = qAx(X, angles[i] || 0); });
  ['neckA', 'neckB', 'head'].forEach((name, i) => { k.local[name] = qAx(X, neck[i] || 0); });
}
function scap(k, up = 0, post = 0, protraction = 0, lift = 0) {
  const p = scapulaPose(up, post, protraction, null, 0, lift);
  both(k, 'clav', p.clav); both(k, 'scap', p.scap);
}
function fingers(k, side, grip = false) {
  for (let n = 2; n <= 5; n++) {
    setSide(k.local, side, 'f' + n + 'a', qAx(Z, grip ? -42 : -3));
    setSide(k.local, side, 'f' + n + 'b', qAx(Z, grip ? -48 : 0));
    setSide(k.local, side, 'f' + n + 'c', qAx(Z, grip ? -25 : 0));
  }
  setSide(k.local, side, 'f1m', qAx(X, grip ? 18 : 30).multiply(qAx(Z, grip ? -20 : 0)));
  setSide(k.local, side, 'f1a', qAx(Z, grip ? -15 : 0));
  setSide(k.local, side, 'f1b', qAx(Z, grip ? -12 : 0));
}
function handBasis(direction, normal, side = 'L') {
  const g = direction.clone().normalize(), n = normal.clone();
  n.addScaledVector(g, -n.dot(g)).normalize();
  const xx = side === 'L' ? n.negate() : n, yy = g.negate(), zz = xx.clone().cross(yy).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(xx, yy, zz));
}
function arm(k, side, wrist, pole, direction = null, normal = null, grip = false) {
  const flex = armIK(k.local, k.rootQ, ZERO, side, wrist, pole);
  if (direction && normal) orientHand(k.local, k.rootQ, ZERO, side, handBasis(direction, normal, side), flex);
  fingers(k, side, grip);
}
function palmAt(k, side, center, pole, direction = V(0, 0, 1)) {
  const q = handBasis(direction, V(0, -1, 0), side);
  const raw = A.lmkTag('palm', side);
  const localOff = raw.p.clone().sub(S['hand_' + side].J);
  const wrist = center.clone().sub(localOff.applyQuaternion(q));
  const flex = armIK(k.local, k.rootQ, ZERO, side, wrist, pole);
  orientHand(k.local, k.rootQ, ZERO, side, q, flex);
  fingers(k, side);
}
// Reconstruct the elbow plane and hand roll together. An unconstrained
// swing/twist split can leave a plausible palm position with an impossible
// wrist deviation or forearm turn. These fixed photographic-fit corrections
// preserve the wrist point for free hands, or the palm contact for a grip.
// They do not change the rig, joint limits or force parameters.
function refitHand(k, side, { yaw = 0, roll = 0, pole = 0 } = {}) {
  const W = world(k), hand = 'hand_' + side;
  const shoulder = W['hum_' + side].p;
  const direction = V(0, -1, 0).applyQuaternion(W[hand].q);
  const normal = V(side === 'L' ? -1 : 1, 0, 0).applyQuaternion(W[hand].q);
  let target = W[hand].p.clone(), Q = W[hand].q.clone();
  if (yaw) {
    const landmark = A.lmkTag('palm', side);
    const contact = pt(W, hand, landmark.p);
    direction.applyAxisAngle(normal, yaw * Math.PI / 180);
    Q = handBasis(direction, normal, side);
    target = contact.sub(landmark.p.clone().sub(S[hand].J).applyQuaternion(Q));
  }
  if (roll) Q.premultiply(new THREE.Quaternion().setFromAxisAngle(direction, roll * Math.PI / 180));
  const axis = target.clone().sub(shoulder).normalize();
  const polePoint = W['fore_' + side].p.clone().sub(shoulder)
    .applyAxisAngle(axis, pole * Math.PI / 180).add(shoulder);
  const flex = armIK(k.local, k.rootQ, ZERO, side, target, polePoint);
  orientHand(k.local, k.rootQ, ZERO, side, Q, flex);
}
function fingertipSupport(k, side) {
  // The hand under the trunk in the source photograph is arched and stands on
  // bent fingers. Forcing its entire palm flat used to rotate the wrist past
  // 160 degrees. Fit the actual finger markers to the same support plane with
  // a 30-degree wrist and a modest forearm turn instead.
  const W = world(k), shoulder = W['hum_' + side].p;
  const elbow = W['fore_' + side].p;
  const wrist = W['hand_' + side].p.clone();
  const floor = Math.min(...A.LANDMARKS.filter(l => l.region === 'таз' && l.side === 'R')
    .map(l => pt(W, l.seg, l.pc).y));
  for (let n = 2; n <= 5; n++) {
    setSide(k.local, side, 'f' + n + 'a', qAx(Z, -20));
    setSide(k.local, side, 'f' + n + 'b', qAx(Z, -35));
    setSide(k.local, side, 'f' + n + 'c', qAx(Z, -15));
  }
  const foreAxis = S.hand_L.J.clone().sub(S.fore_L.J).normalize();
  const twist = new THREE.Quaternion().setFromAxisAngle(foreAxis, -20 * Math.PI / 180);
  wrist.y = floor + .08;
  for (let i = 0; i < 16; i++) {
    const axis = wrist.clone().sub(shoulder).normalize();
    const pole = elbow.clone().sub(shoulder).applyAxisAngle(axis, 20 * Math.PI / 180).add(shoulder);
    const flex = armIK(k.local, k.rootQ, ZERO, side, wrist, pole);
    setSide(k.local, side, 'fore', flex.clone().multiply(twist));
    setSide(k.local, side, 'ulna', twist.clone().invert());
    setSide(k.local, side, 'hand', qAx(Z, 30));
    const posed = world(k);
    // The fitted atlas phalanges extend beyond some schematic fingertip
    // markers. Use every original finger-bone vertex, with 2 mm of geometric
    // clearance, instead of placing those actual bones through the floor.
    const gap = boneFloor(posed, seg => /^f[1-5][mabc]_/.test(seg) && seg.endsWith('_' + side)) - (floor + .002);
    if (Math.abs(gap) < 1e-5) break;
    wrist.y -= gap * .65;
  }
}
function feet(k, quaternion) { for (const side of SIDES) orientFoot(k.local, k.rootQ, ZERO, side, quaternion); }
function settleSagittal(k) {
  const W = world(k), com = A.comOf(W);
  const points = A.LANDMARKS.map(l => pt(W, l.seg, l.pc));
  const sorted = points.map(p => [p.z, p.y]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const p of sorted) { while (lower.length > 1 && cross(lower.at(-2), lower.at(-1), p) <= 0) lower.pop(); lower.push(p); }
  for (const p of sorted.slice().reverse()) { while (upper.length > 1 && cross(upper.at(-2), upper.at(-1), p) <= 0) upper.pop(); upper.push(p); }
  lower.pop(); upper.pop(); const hull = lower.concat(upper);
  let best = null;
  for (let i = 0; i < hull.length; i++) {
    const a = hull[i], b = hull[(i + 1) % hull.length];
    const angle = Math.atan2(b[1] - a[1], b[0] - a[0]);
    for (const trial of [angle, angle + Math.PI]) {
      const cc = Math.cos(trial), ss = Math.sin(trial);
      const rotate = p => [p[0] * cc + p[1] * ss, p[1] * cc - p[0] * ss];
      const ra = rotate(a), rb = rotate(b);
      if (hull.some(p => rotate(p)[1] < ra[1] - 1e-7)) continue;
      const c = rotate([com.z, com.y]);
      if (c[0] < Math.min(ra[0], rb[0]) || c[0] > Math.max(ra[0], rb[0])) continue;
      const delta = Math.atan2(Math.sin(trial), Math.cos(trial));
      if (best === null || Math.abs(delta) < Math.abs(best)) best = delta;
    }
  }
  if (best !== null && Math.abs(best) < 20 * Math.PI / 180)
    k.rootQ.premultiply(qAx(X, best * 180 / Math.PI));
}
function finish(k, notes = [], view = [-1, .34, .6], settle = false) {
  for (const q of Object.values(k.local)) q.normalize();
  k.rootQ.normalize();
  if (settle) settleSagittal(k);
  k.rootP = rootPlaced(k.local, k.rootQ);
  k.notes = notes;
  k.view = view;
  return k;
}
function seated(hip = 93) {
  const k = base();
  both(k, 'thigh', qAx(X, -hip));
  both(k, 'shank', qAx(X, 0));
  feet(k, qAx(X, -90));
  return k;
}
function behindHead(k) {
  const W = world(k);
  for (const side of SIDES) {
    const s = sign(side);
    const wrist = pt(W, 'head', V(s * .17, 1.595, -.18));
    const shoulder = W['hum_' + side].p;
    arm(k, side, wrist, shoulder.clone().add(V(s * .7, .1, .05)),
      V(-s * .95, .3, 0), V(0, 0, 1), true);
  }
}

function corner() {
  const k = seated(88);
  scap(k, 0, 6, 0, -.024);
  let W = world(k);
  const heel = A.LANDMARKS.filter(l => /^(пятки|икры)$/.test(l.region))
    .reduce((low, l) => Math.min(low, pt(W, l.seg, l.pc).y), Infinity);
  for (const side of SIDES) {
    const s = sign(side);
    palmAt(k, side, V(s * .245, heel, .025), V(s * .4, -.3, -.2));
  }
  return finish(k, ['Ладони у таза, ноги вытянуты вперёд. Высота таза и угол ног восстановлены по одному боковому снимку.']);
}
function staff() {
  const k = seated(43.5);
  k.rootQ = qAx(X, -43.5);
  spine(k, [21.75, 21.75, 0, 0, 0]);
  feet(k, qAx(X, -90));
  scap(k, 10, 8, 2, 0);
  behindHead(k);
  for (const side of SIDES) refitHand(k, side, { roll: sign(side) * 15, pole: -sign(side) * 35 });
  return finish(k, ['Вариант из пособия: руки за головой, локти разведены.']);
}
function supineBoat() {
  const k = base(-77);
  spine(k, [7, 7, 6, 5, 5], [3, 2, 0]);
  scap(k, 5, 0, 7);
  both(k, 'thigh', qAx(X, -33));
  both(k, 'shank', qAx(X, 1));
  both(k, 'foot', qAx(X, -12));
  for (const side of SIDES) {
    const s = sign(side);
    const W = world(k), sh = W['hum_' + side].p;
    const target = sh.clone().add(V(-s * .025, .12, .53));
    arm(k, side, target, sh.clone().add(V(s * .12, .1, .4)), V(0, .18, 1), V(-s, 0, 0));
  }
  for (const side of SIDES) refitHand(k, side, { pole: -sign(side) * 25 });
  return finish(k, ['Корпус и прямые ноги приподняты; руки направлены к ногам. Углы — реконструкция снимка.'], [-1, .34, .6], true);
}
function singleBridge() {
  const k = base(-125);
  spine(k, [-8, -8, 8, 13, 15], [5, 5, 5]);
  scap(k, 28, 10, 0);
  // Fit the support foot to the upper back instead of placing the entire pose
  // on the lowest toe and leaving the shoulders suspended above the floor.
  let W = world(k);
  const upper = A.LANDMARKS.filter(l => (l.region === 'спина' && l.p.y > 1.28) || l.region === 'голова');
  // A distributed thoracic/cervical curve lets the upper back lie down
  // instead of balancing the bridge on the occiput. The 10 mm offset fits
  // the fixed atlas upper-back surface relative to its schematic markers;
  // it is a geometric reconstruction parameter, not tissue indentation.
  // The unchanged head/neck bones retain at least 2 mm of bare-bone clearance.
  const upperY = Math.min(Math.min(...upper.map(l => pt(W, l.seg, l.pc).y)),
    boneFloor(W, seg => ['head', 'neckA', 'neckB', 'thC'].includes(seg)) - .002) - .010;
  setSide(k.local, 'L', 'thigh', qAx(X, -55));
  setSide(k.local, 'L', 'shank', IDQ);
  setSide(k.local, 'L', 'foot', qAx(X, 25));
  const footQ = qAx(X, 42);
  const ball = A.lmkTag('ball', 'R');
  const ballOff = ball.pc.clone().sub(S.foot_R.J).applyQuaternion(footQ);
  legIK(k.local, k.rootQ, ZERO, 'R', V(-.09, upperY - ballOff.y, .52), V(-.09, .3, .6));
  orientFoot(k.local, k.rootQ, ZERO, 'R', footQ);
  setSide(k.local, 'R', 'toes', qAx(X, -42));
  for (let i = 0; i < 5; i++) {
    W = world(k);
    const floorY = Math.min(...A.LANDMARKS.filter(l => l.side === 'R' && /^(пятки|подушечки стоп|наружный край стопы|пальцы стоп|тыл стоп)$/.test(l.region)).map(l => pt(W, l.seg, l.pc).y));
    const ankle = W.foot_R.p.clone(); ankle.y += upperY - floorY;
    legIK(k.local, k.rootQ, ZERO, 'R', ankle, V(-.09, .3, .6));
    orientFoot(k.local, k.rootQ, ZERO, 'R', footQ);
  }
  // Both arms extend beyond the head along the same support plane.
  for (const side of SIDES) {
    const s = sign(side); W = world(k);
    const sh = W['hum_' + side].p;
    const target = V(sh.x + s * .02, upperY + .01, sh.z - .57);
    for (let i = 0; i < 5; i++) {
      arm(k, side, target, sh.clone().add(V(s * .1, -.02, -.3)), V(0, 0, -1), V(0, 1, 0));
      const height = boneFloor(world(k), seg => /^(hand|f[1-5][mabc])_/.test(seg) && seg.endsWith('_' + side));
      const clearance = upperY + .002 - height;
      if (clearance <= 1e-5) break;
      target.y += clearance;
    }
  }
  // Keep the nearly straight overhead arms in a feasible elbow plane while
  // preserving the fitted world wrist points and hand orientations.
  for (const side of SIDES) refitHand(k, side, { pole: -sign(side) * 20 });
  return finish(k, ['Левая нога поднята, правая опирается передним отделом стопы; пятка поднята, как на вставке в пособии.', 'Опора восстановлена на верхней части спины и стопе; руки лежат за головой. Изгиб грудного отдела и шеи согласован с исходными костями и поверхностями модели.'], [-1, .32, .55]);
}
function lowBoat() {
  const k = base(-85);
  spine(k, [5, 5, 4, 4, 4], [3, 2, 0]);
  scap(k, 35, 12, 3, .012);
  both(k, 'thigh', qAx(X, -9));
  both(k, 'shank', qAx(X, 1));
  both(k, 'foot', qAx(X, 35));
  for (const side of SIDES) {
    const s = sign(side), W = world(k), sh = W['hum_' + side].p;
    arm(k, side, sh.clone().add(V(-s * .045, .39, -.38)),
      sh.clone().add(V(s * .05, .3, -.25)), V(0, .7, -.7), V(s, 0, 0));
  }
  for (const side of SIDES) refitHand(k, side, { roll: -sign(side) * 25 });
  return finish(k, ['Вариант из пособия: руки вытянуты за голову, ноги остаются низко над полом.'], [-1, .34, .6], true);
}
function lightFingerGrip(k, side, bend) {
  for (let n = 2; n <= 5; n++) {
    setSide(k.local, side, 'f' + n + 'a', qAx(Z, -bend));
    setSide(k.local, side, 'f' + n + 'b', qAx(Z, -bend));
    setSide(k.local, side, 'f' + n + 'c', qAx(Z, -bend * .5));
  }
  setSide(k.local, side, 'f1m', qAx(X, 30));
  setSide(k.local, side, 'f1a', qAx(Z, -10));
  setSide(k.local, side, 'f1b', qAx(Z, -5));
}
function wideSupine() {
  const k = base(-90);
  spine(k, [0, 0, 0, 0, 0]);
  scap(k, 0, 5, 3);
  for (const side of SIDES) {
    const s = sign(side);
    // The knees contact the floor on their medial soft-tissue side. The
    // anterior patellar marker does not represent that side of the knee.
    // A roughly 5-degree change of the knee plane keeps
    // the medial thigh surface above the shared knee/back support plane.
    legIK(k.local, k.rootQ, ZERO, side, V(s * .34, .17, -.42), V(s * .85, -.30, -.05));
    orientFoot(k.local, k.rootQ, ZERO, side, qAx(X, -160).multiply(qAx(Z, s * -10)));
  }
  for (const side of SIDES) {
    const s = sign(side), W = world(k), foot = W['foot_' + side];
    const C = pt(W, 'foot_' + side, V(s * .15, .115, .02));
    const direction = V(0, 0, 1).applyQuaternion(foot.q);
    const Q = handBasis(direction, V(-s, 0, 0), side);
    const off = A.lmkTag('palm', side).p.clone().sub(S['hand_' + side].J).applyQuaternion(Q);
    const wrist = C.clone().sub(off);
    const flex = armIK(k.local, k.rootQ, ZERO, side, wrist, W['hum_' + side].p.clone().add(V(0, -1, 0)));
    orientHand(k.local, k.rootQ, ZERO, side, Q, flex);
    lightFingerGrip(k, side, 20);
  }
  for (const side of SIDES) refitHand(k, side, { yaw: -sign(side) * 10, pole: -sign(side) * 40 });
  // Keep the intended supine trunk. A marker-only sagittal hull tilt would
  // rotate the side of the knees below the actual thigh surface; the shared
  // rendered-geometry ground fit places this frozen posture afterwards.
  return finish(k, ['Колени разведены к полу, кисти у наружных краёв стоп. Контакт восстановлен геометрически; давление и сжатие тканей не рассчитываются.'], [-.85, .8, .65]);
}
function supineStretch() {
  const k = base(-90);
  spine(k, [0, 0, 0, 0, 0], [0, 0, 0]);
  scap(k, 25, 10, 5, .02);
  both(k, 'shank', qAx(X, 1));
  both(k, 'foot', qAx(X, 0));
  // Fit the straight legs to an actual reachable plantar contact. The palm is
  // placed against the ball of the foot, rather than merely aiming the wrist
  // beneath the ankle and leaving a visible gap between the fingers and feet.
  let hip = 120;
  function grips() {
    both(k, 'thigh', qAx(X, -hip).multiply(qAx(Z, -2)));
    const W = world(k);
    return SIDES.map(side => {
      const foot = W['foot_' + side];
      const soleNormal = V(0, -1, 0).applyQuaternion(foot.q).normalize();
      const direction = V(0, 0, -1).applyQuaternion(foot.q).normalize();
      const C = pt(W, 'foot_' + side, V(sign(side) * .11, .006, .135));
      const Q = handBasis(direction, soleNormal.clone().negate(), side);
      const off = A.lmkTag('palm', side).p.clone().sub(S['hand_' + side].J).applyQuaternion(Q);
      const wrist = C.clone().sub(off);
      return { side, Q, wrist, reach: wrist.distanceTo(W['hum_' + side].p) };
    });
  }
  while (hip < 155 && grips().some(g => g.reach > .575)) hip += .25;
  for (const g of grips()) {
    const s = sign(g.side);
    const flex = armIK(k.local, k.rootQ, ZERO, g.side, g.wrist, V(s * .32, .4, -.65));
    orientHand(k.local, k.rootQ, ZERO, g.side, g.Q, flex);
    lightFingerGrip(k, g.side, 10);
  }
  for (const side of SIDES) refitHand(k, side, { yaw: sign(side) * 30 });
  return finish(k, ['Прямые ноги направлены вверх и к голове, ладони у подошв. Глубина сгибания подобрана по досягаемости неизменённого рига, а не измерена по фотографии.'], [-1, .34, .6], true);
}
function mermaid() {
  const k = base();
  k.rootQ = qAx(Z, 90);
  // Both legs lift in the same world direction: this is deliberately not a mirror of the hip rotations.
  k.local.thigh_L = qAx(Z, 7.5); k.local.thigh_R = qAx(Z, 16);
  both(k, 'shank', IDQ); both(k, 'foot', qAx(X, 15));
  for (const name of ['lumA', 'lumB', 'thA', 'thB', 'thC']) k.local[name] = qAx(Z, -5);
  k.local.neckA = qAx(Z, -5); k.local.neckB = qAx(Z, -3);
  scap(k, 12, 4, 4);
  let W = world(k);
  palmAt(k, 'L', V(-.86, -.125, .02), V(-.72, -.03, .1), V(-1, 0, 0));
  W = world(k);
  palmAt(k, 'R', V(-.40, -.125, .30), V(-.35, -.1, .5), V(1, 0, .1));
  fingertipSupport(k, 'R');
  return finish(k, ['Опора на правом боку; обе прямые ноги приподняты. Кисть под корпусом опирается кончиками согнутых пальцев, как на фотографии.', 'Положение рук и боковой наклон — реконструкция фронтального снимка.'], [.1, .45, 1]);
}

export function buildDetali() {
  return {
    'detali-01': corner(),
    'detali-02': staff(),
    'detali-03': supineBoat(),
    'detali-04': singleBridge(),
    'detali-05': lowBoat(),
    'detali-06': wideSupine(),
    'detali-07': supineStretch(),
    'detali-08': mermaid(),
  };
}
