// An ordered, display-only release of the corrected inner-shin bow grip.
// Small IK knots preserve the wrist route; one large joint slerp shortcuts
// that route through the shin. This does not solve grasp force or dynamics.
import { armIK, orientHand } from './cycle_lib.mjs';
import { rotateElbowPlane } from './practice/kinematics_helpers.mjs';

export function prepareBowRelease({ A, THREE, conf, collision, clearEdge, openTarget }) {
  if (typeof collision !== 'function' || typeof clearEdge !== 'function' || !conf || !openTarget)
    throw new Error('Bow release requires a source, target and actual geometry guards');
  const clone = c => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, c.local[s.name].clone()])),
    rootQ: c.rootQ.clone(), rootP: c.rootP.clone() });
  const root = new THREE.Quaternion(), position = new THREE.Vector3();
  const world = c => A.computeWorld(c.local, root, position);
  const put = (c, side, base, q) => { c.local[base + '_' + side] = side === 'L' ? q.clone() : A.mirrorQ(q); };
  const qX = degrees => new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), degrees * Math.PI / 180);
  const stages = [];

  // Uncurl the four long fingers and the thumb's terminal hinge first.
  const loose = clone(conf);
  for (const s of A.SEG) if (/^f[2-5][abc]_[LR]$/.test(s.name) || /^f1b_[LR]$/.test(s.name)) loose.local[s.name].identity();
  stages.push(loose);

  // The wrists lie inside the shins. A short medial/headward withdrawal
  // releases the thumb rather than pulling the closed ring through a leg.
  const unhook = clone(loose), looseWorld = world(loose);
  for (const side of ['L', 'R']) {
    const sign = side === 'L' ? 1 : -1, delta = new THREE.Vector3(-sign * .005, .01, .02);
    const target = looseWorld['hand_' + side].p.clone().add(delta);
    const pole = looseWorld['fore_' + side].p.clone().add(delta);
    const flex = armIK(unhook.local, root, position, side, target, pole);
    orientHand(unhook.local, root, position, side, looseWorld['hand_' + side].q, flex);
  }
  stages.push(unhook);
  const free = clone(unhook);
  free.transitionGuide = true;
  for (const s of A.SEG) if (/^f[1-5][mabc]_[LR]$/.test(s.name)) free.local[s.name].identity();
  for (const side of ['L', 'R']) put(free, side, 'f1m', qX(-35));
  stages.push(free);

  // Bring the entire palm and fingertips headward before moving laterally.
  // The elbow plane changes gradually, never by a discontinuous IK branch.
  const freeWorld = world(free);
  for (let i = 1; i <= 8; i++) {
    const u = i / 8, c = clone(free), delta = new THREE.Vector3(0, .03 * u, .16 * u);
    c.transitionGuide = true;
    for (const side of ['L', 'R']) {
      const target = freeWorld['hand_' + side].p.clone().add(delta);
      const pole = freeWorld['fore_' + side].p.clone().add(delta);
      const flex = armIK(c.local, root, position, side, target, pole);
      orientHand(c.local, root, position, side, freeWorld['hand_' + side].q, flex);
      rotateElbowPlane({ local: c.local, rootQ: root, rootP: position }, side, (side === 'L' ? -60 : 60) * u);
    }
    stages.push(c);
  }
  const front = stages.at(-1), frontWorld = world(front);
  for (let i = 1; i <= 8; i++) {
    const u = i / 8, c = clone(front);
    c.transitionGuide = true;
    for (const side of ['L', 'R']) {
      const delta = new THREE.Vector3((side === 'L' ? 1 : -1) * .15 * u, 0, 0);
      const target = frontWorld['hand_' + side].p.clone().add(delta);
      const pole = frontWorld['fore_' + side].p.clone().add(delta);
      const flex = armIK(c.local, root, position, side, target, pole);
      orientHand(c.local, root, position, side, frontWorld['hand_' + side].q, flex);
    }
    stages.push(c);
  }
  // Let the released palms turn with the whole arm on its way to openTarget.
  // Imposing that final WORLD orientation while the elbows are still behind
  // the body folds the wrists and drives the first metacarpals into the forearms.

  let previous = conf;
  if (collision(conf) || collision(openTarget)) return null;
  for (const stage of stages) {
    if (collision(stage) || !clearEdge(previous, stage)) return null;
    previous = stage;
  }
  return clearEdge(previous, openTarget) ? stages : null;
}
