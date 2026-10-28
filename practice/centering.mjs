// Static photo reconstructions of FYSM 1. Практика, page 6.
// Angles below are modelling choices; the PDF does not contain measured joints,
// recruitment, timing or transitions. Nothing here changes the shared skeleton.
import {
  A, THREE, V, X, Y, Z, qAx, IDQ, SEG, S, computeWorld, pt, mirrorQ,
  blankLocal, rootPlaced, armIK, legIK, orientHand, orientFoot, scapulaPose,
} from '../cycle_lib.mjs';
import { buildFold, buildPlank } from '../poses_build.mjs';

const ZERO = V(0, 0, 0);
const SIDES = ['L', 'R'];
const sign = side => side === 'L' ? 1 : -1;
const wOf = p => computeWorld(p.local, p.rootQ, p.rootP || ZERO);
function setSym(L, base, q) {
  L[base + '_L'] = q.clone(); L[base + '_R'] = mirrorQ(q);
}
function handBasis(fingers, normal) {
  const y = fingers.clone().normalize().negate();
  const x = normal.clone().addScaledVector(y, -normal.dot(y)).normalize().negate();
  const z = x.clone().cross(y).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
}
function fingers(L, side, mode = 'flat') {
  const flex = mode === 'grip' ? [48, 55, 32] : mode === 'soft' ? [12, 15, 8] : [6, 0, 0];
  for (let i = 2; i <= 5; i++) {
    const a = mode === 'flat' ? [14, 0, 10, 20][i - 2] : 0;
    const firstFlex = mode === 'flat' ? [7.5, 5.7, 4.8, 2.8][i - 2] : flex[0];
    const ab = qAx(X, i <= 3 ? -a : a).multiply(qAx(Z, -firstFlex));
    const q = side === 'L' ? ab : mirrorQ(ab);
    L[`f${i}a_${side}`] = q;
    L[`f${i}b_${side}`] = side === 'L' ? qAx(Z, -flex[1]) : mirrorQ(qAx(Z, -flex[1]));
    L[`f${i}c_${side}`] = side === 'L' ? qAx(Z, -flex[2]) : mirrorQ(qAx(Z, -flex[2]));
  }
  const thumb = qAx(X, mode === 'flat' ? -42 : -16).multiply(qAx(Y, mode === 'grip' ? 18 : 0));
  L[`f1m_${side}`] = side === 'L' ? thumb : mirrorQ(thumb);
  L[`f1a_${side}`] = mode === 'flat' ? IDQ.clone() : side === 'L' ? qAx(Z, 12) : mirrorQ(qAx(Z, 12));
  L[`f1b_${side}`] = IDQ.clone();
}
function shoulder(L, up = 15, post = 10, pro = 3) {
  const sp = scapulaPose(up, post, pro, null, up / 2, 0);
  setSym(L, 'clav', sp.clav); setSym(L, 'scap', sp.scap);
}
function spine(L, angles = [0, 0, 0, 0, 0], twist = [0, 0, 0]) {
  ['lumA', 'lumB', 'thA', 'thB', 'thC'].forEach((name, i) => {
    L[name] = qAx(X, angles[i]).multiply(qAx(Y, i >= 2 ? twist[i - 2] : 0));
  });
}
function arm(p, side, target, pole, orientation = null, fingerMode = 'flat') {
  const fq = armIK(p.local, p.rootQ, p.rootP || ZERO, side, target, pole);
  if (orientation) orientHand(p.local, p.rootQ, p.rootP || ZERO, side, orientation, fq);
  fingers(p.local, side, fingerMode);
}
function floorArm(p, side, target, pole) {
  const q = handBasis(V(0, 0, 1), V(0, -1, 0));
  arm(p, side, target, pole, side === 'L' ? q : mirrorQ(q));
}
function supportedForearm(p, side, floorY, upperDrop = null) {
  const sh = wOf(p)[`hum_${side}`].p;
  const hum = S.fore_L.J.distanceTo(S.hum_L.J), fore = S.hand_L.J.distanceTo(S.fore_L.J);
  let elbowY = upperDrop == null ? floorY + .0525 : sh.y - upperDrop;
  for (let i = 0; i < 4; i++) {
    const drop = sh.y - elbowY;
    const back = Math.sqrt(Math.max(.0001, hum * hum - drop * drop));
    const elbow = V(sh.x, elbowY, sh.z - back);
    const wrist = elbow.clone().add(V(0, 0, fore));
    floorArm(p, side, wrist, elbow);
    if (upperDrop == null) elbowY -= floorMin(p, l => l.side === side && l.region === 'предплечья') - floorY;
  }
}
function newPose(pitch, pelvisY = 0) {
  const local = blankLocal();
  shoulder(local);
  return { local, rootQ: qAx(X, pitch), rootP: V(0, pelvisY, 0) };
}
function floorMin(p, predicate = () => true) {
  const W = wOf(p);
  return Math.min(...A.LANDMARKS.filter(predicate).map(l => pt(W, l.seg, l.pc).y));
}
function finish(p, notes, view = [-1, .28, .48]) {
  // A common translation preserves every solved reach and support relation.
  const placed = rootPlaced(p.local, p.rootQ);
  p.rootP = placed;
  for (const name of Object.keys(p.local)) p.local[name].normalize();
  return { ...p, view, notes };
}
function bisect(f, lo, hi) {
  let fl = f(lo), fh = f(hi);
  if (fl * fh > 0) return Math.abs(fl) < Math.abs(fh) ? lo : hi;
  for (let i = 0; i < 45; i++) {
    const mid = (lo + hi) / 2, fm = f(mid);
    if (fm * fl > 0) { lo = mid; fl = fm; } else hi = mid;
  }
  return (lo + hi) / 2;
}
function kneelingFeet(p) {
  const L = p.local;
  const kneeFloor = floorMin(p, l => l.region === 'колени');
  const apply = angle => {
    for (const sd of SIDES) {
      orientFoot(L, p.rootQ, p.rootP, sd, qAx(X, angle));
      L[`toes_${sd}`] = qAx(X, -75);
    }
    return floorMin(p, l => l.region === 'пальцы стоп') - kneeFloor;
  };
  apply(bisect(apply, 145, 175));
}

function lockedFold() {
  const b = buildFold({ hipFlex: 150, lum: [6, 6], thor: [4, 4, 4] });
  const p = { local: b.L, rootQ: b.rootQ, rootP: rootPlaced(b.L, b.rootQ) };
  // The two wrists remain distinct. The closed lock is represented by curved
  // fingers approaching the other hand rather than coincident palm centres.
  const W = wOf(p), Q = W.thC.q;
  for (const sd of SIDES) {
    const x = sign(sd);
    const target = W.hum_L.p.clone(); target.x *= x;
    target.add(V(-x * .123, -.10, -.53).applyQuaternion(Q));
    const pole = W[`hum_${sd}`].p.clone().add(V(x * .1, -.05, -.7).applyQuaternion(Q));
    const handQ = handBasis(V(0, .2, -1).applyQuaternion(Q), V(x, 0, 0).applyQuaternion(Q));
    arm(p, sd, target, pole, handQ, 'grip');
  }
  return finish(p, ['Закрытый замок за спиной восстановлен по фотографии; положение пальцев — геометрическая реконструкция.']);
}

function lowPlank() {
  const b = buildPlank();
  const p = { local: b.L, rootQ: b.rootQ, rootP: rootPlaced(b.L, b.rootQ) };
  shoulder(p.local, 15, 10, 10);
  const W = wOf(p);
  const feetFloor = floorMin(p, l => l.region === 'пальцы стоп');
  for (const sd of SIDES) {
    const sh = W[`hum_${sd}`].p;
    const target = sh.clone().add(V(sign(sd) * .018, -.364, -.26));
    const pole = sh.clone().add(V(sign(sd) * .03, -.015, -.7));
    for (let i = 0; i < 3; i++) {
      floorArm(p, sd, target, pole);
      target.y -= floorMin(p, l => l.side === sd && l.region === 'ладони') - feetFloor;
    }
  }
  return finish(p, ['Нижний упор показан с согнутыми локтями и опорой на ладони и пальцы обеих стоп.']);
}

function warriorOne() {
  const p = newPose(12, .58), L = p.local;
  spine(L, [-2, -2, -3, -3, -2]); shoulder(L, 43, 15, 3);
  legIK(L, p.rootQ, p.rootP, 'L', V(.115, .067, .36), V(.12, .58, .9));
  orientFoot(L, p.rootQ, p.rootP, 'L', IDQ);
  // Back heel is lifted in the supplied photograph.
  legIK(L, p.rootQ, p.rootP, 'R', V(-.115, .17, -.75), V(-.12, .3, -.25));
  orientFoot(L, p.rootQ, p.rootP, 'R', qAx(X, 64));
  L.toes_R = qAx(X, -64);
  const W = wOf(p);
  for (const sd of SIDES) {
    const sh = W[`hum_${sd}`].p;
    const hq = handBasis(V(0, 1, .1), V(-1, 0, 0));
    arm(p, sd, sh.clone().add(V(sign(sd) * .025, .576, .025)), sh.clone().add(V(sign(sd) * .12, .4, .2)), sd === 'L' ? hq : mirrorQ(hq), 'soft');
  }
  return finish(p, ['Передняя нога согнута в выпаде; задняя пятка поднята, как на фотографии.'], [-1, .22, .65]);
}

function sideWarrior() {
  // Wide stance in the frontal plane, with the torso arcing toward the straight
  // leg. This is the photographed variant, rather than a generic arms-level II.
  const p = newPose(0, .53), L = p.local;
  p.rootQ = qAx(Z, 12);
  L.lumA = qAx(Z, 8); L.lumB = qAx(Z, 9);
  L.thA = qAx(Z, 6); L.thB = qAx(Z, 4); L.thC = qAx(Z, 3);
  shoulder(L, 42, 15, 2);
  legIK(L, p.rootQ, p.rootP, 'L', V(.50, .068, .02), V(.55, .55, .03));
  orientFoot(L, p.rootQ, p.rootP, 'L', qAx(Y, 88));
  const backHip = wOf(p).thigh_R.p;
  const legLength = S.thigh_L.J.distanceTo(S.shank_L.J) + S.shank_L.J.distanceTo(S.foot_L.J) - .0005;
  const dx = Math.sqrt(legLength * legLength - (.068 - backHip.y) ** 2 - (-.02 - backHip.z) ** 2);
  legIK(L, p.rootQ, p.rootP, 'R', V(backHip.x - dx, .068, -.02), V(-.4, .4, .1));
  orientFoot(L, p.rootQ, p.rootP, 'R', qAx(Y, -50));
  const W = wOf(p);
  for (const sd of SIDES) {
    const sh = W[`hum_${sd}`].p;
    arm(p, sd, sh.clone().add(V(-.32, .47, 0)), sh.clone().add(V(-.24, .4, sign(sd) * .08)), handBasis(V(-.5, .85, 0), V(0, 0, sign(sd))), 'soft');
  }
  return finish(p, ['Широкая стойка с боковым наклоном и поднятыми руками показана по варианту на листе FYSM.'], [.15, .30, 1]);
}

function forearmSplit() {
  const p = newPose(154, 0), L = p.local;
  spine(L, [-3, -3, -3, -3, -3]); shoulder(L, 42, 15, 5);
  // Left leg is the elevated straight leg. Right toes are the third support.
  L.thigh_L = p.rootQ.clone().invert().multiply(qAx(X, 181));
  L.shank_L = IDQ.clone(); L.foot_L = qAx(X, 45); L.toes_L = qAx(X, 10);
  const W = wOf(p);
  for (const sd of SIDES) {
    const sh = W[`hum_${sd}`].p;
    floorArm(p, sd, sh.clone().add(V(sign(sd) * .008, -.325, .248)), sh.clone().add(V(0, -.8, -.03)));
  }
  const foreHeight = floorMin(p, l => l.region === 'предплечья');
  const applyLeg = pitch => {
    L.thigh_R = p.rootQ.clone().invert().multiply(qAx(X, pitch));
    L.shank_R = IDQ.clone();
    orientFoot(L, p.rootQ, p.rootP, 'R', qAx(X, 74));
    L.toes_R = qAx(X, -74);
    return floorMin(p, l => l.side === 'R' && /стоп|пальцы стоп/.test(l.region)) - foreHeight;
  };
  applyLeg(bisect(applyLeg, 20, 85));
  return finish(p, ['Одна прямая нога поднята; другая остаётся на пальцах, предплечья и ладони находятся у пола.'], [-1, .22, .35]);
}

function halfCrane() {
  const p = newPose(135, .54), L = p.local;
  spine(L, [-3, -3, -3, -3, -3]); shoulder(L, 32, 12, 12);
  let W = wOf(p);
  const shL = W.hum_L.p, shR = W.hum_R.p;
  supportedForearm(p, 'R', 0, .20);
  let y = shL.y - .292;
  for (let i = 0; i < 4; i++) {
    floorArm(p, 'L', V(shL.x + .015, y, shL.z - .35), shL.clone().add(V(.08, -.12, -.6)));
    const palm = floorMin(p, l => l.side === 'L' && l.region === 'ладони');
    const fore = floorMin(p, l => l.side === 'R' && l.region === 'предплечья');
    y -= palm - fore;
  }
  W = wOf(p);
  const upper = W.hum_L.p.clone().lerp(W.fore_L.p, .55);
  const hip = W.thigh_L.p;
  const thighLength = S.thigh_L.J.distanceTo(S.shank_L.J);
  const shankLength = S.shank_L.J.distanceTo(S.foot_L.J);
  const knee = upper.clone().add(V(.035, .095, 0)).sub(hip).normalize().multiplyScalar(thighLength).add(hip);
  const ankle = hip.clone().add(V(.20, .25, .03)).sub(knee).normalize().multiplyScalar(shankLength).add(knee);
  legIK(L, p.rootQ, p.rootP, 'L', ankle, knee);
  L.foot_L = qAx(X, 38);
  L.thigh_R = p.rootQ.clone().invert().multiply(qAx(X, 130));
  L.shank_R = qAx(X, 90); L.foot_R = qAx(X, 45); L.toes_R = qAx(X, 10);
  return finish(p, ['Асимметричная опора: левая ладонь и правое предплечье. Одна нога собрана у плеча, другая согнута и поднята.', 'Контакт колена с плечом восстановлен геометрически; давление между частями тела не рассчитывается.'], [1, .32, .48]);
}

function kneelingTwist() {
  const p = newPose(90, .46), L = p.local;
  spine(L, [0, 0, -3, -3, -3], [-20, -25, -25]); shoulder(L, 12, 8, 5);
  for (const sd of SIDES) {
    L[`thigh_${sd}`] = p.rootQ.clone().invert();
    L[`shank_${sd}`] = qAx(X, 90);
    orientFoot(L, p.rootQ, p.rootP, sd, qAx(X, 162));
    L[`toes_${sd}`] = qAx(X, -75);
  }
  kneelingFeet(p);
  const kneeFloor = floorMin(p, l => l.region === 'колени');
  supportedForearm(p, 'L', kneeFloor);
  const sr = wOf(p).hum_R.p;
  arm(p, 'R', sr.clone().add(V(0, .565, -.015)), sr.clone().add(V(-.1, .4, -.05)), handBasis(V(0, 1, 0), V(0, 0, 1)), 'soft');
  L.neckA = qAx(Y, -10); L.neckB = qAx(Y, -10);
  return finish(p, ['Колени и пальцы стоп у пола; грудная клетка развёрнута, левая рука опирается предплечьем, правая вытянута вверх.'], [.8, .34, .75]);
}

function puppy() {
  const p = newPose(145, .46), L = p.local;
  spine(L, [-10, -10, -8, -8, -6]); shoulder(L, 43, 18, 3);
  for (const sd of SIDES) {
    L[`thigh_${sd}`] = p.rootQ.clone().invert();
    L[`shank_${sd}`] = qAx(X, 90);
    orientFoot(L, p.rootQ, p.rootP, sd, qAx(X, 162));
    L[`toes_${sd}`] = qAx(X, -75);
  }
  kneelingFeet(p);
  const kneeFloor = floorMin(p, l => l.region === 'колени');
  const pitch = bisect(pitch => {
    p.rootQ = qAx(X, pitch);
    for (const sd of SIDES) {
      L[`thigh_${sd}`] = p.rootQ.clone().invert();
      orientFoot(L, p.rootQ, p.rootP, sd, qAx(X, 162));
    }
    return floorMin(p, l => l.region === 'грудь') - kneeFloor;
  }, 140, 170);
  p.rootQ = qAx(X, pitch);
  for (const sd of SIDES) L[`thigh_${sd}`] = p.rootQ.clone().invert();
  const W = wOf(p);
  for (const sd of SIDES) {
    const sh = W[`hum_${sd}`].p;
    floorArm(p, sd, V(sh.x, kneeFloor + .014, sh.z + .565), sh.clone().add(V(sign(sd) * .06, .15, .8)));
  }
  L.neckA = qAx(X, -55); L.neckB = qAx(X, -10);
  return finish(p, ['Таз находится над коленями, грудная клетка опущена, руки вытянуты вперёд. Пальцы стоп подогнуты, как на фотографии.'], [-1, .25, .3]);
}

export function buildCentering() {
  return {
    'centering-01': lockedFold(), 'centering-02': lowPlank(),
    'centering-03': warriorOne(), 'centering-04': sideWarrior(),
    'centering-05': forearmSplit(), 'centering-06': halfCrane(),
    'centering-07': kneelingTwist(), 'centering-08': puppy(),
  };
}
