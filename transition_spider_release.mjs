// A display-only exit from Spider: knees rest on the backs of the upper arms
// and the shins cross in front of the forearms, so the arms are enclosed.
// Order of the exit, each stage checked against actual bone surfaces:
// 1. extend the knees: the shins swing forward and outward, away from the
//    forearms, while the knees stay where they rest;
// 2. slide the knees laterally off the upper arms;
// 3. lift the wrists upward and toward the midline. A direct slerp of the
//    shoulder would swing the elbow outward into the spread thigh;
// 4. rise to the open-arm neutral target.
import { armIK, legIK, LT, LS } from './cycle_lib.mjs';
export function prepareSpiderRelease({ A, THREE, conf, collision, clearEdge, openTarget }) {
  if (!conf || !openTarget || typeof collision !== 'function' || typeof clearEdge !== 'function')
    throw new Error('Spider release requires source, target and actual geometry guards');
  const V = (x, y, z) => new THREE.Vector3(x, y, z), mirror = v => V(-v.x, v.y, v.z);
  const copy = input => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, input.local[s.name].clone()])),
    rootQ: input.rootQ.clone(), rootP: input.rootP.clone() });
  // legIK/armIK take targets on each side's own world side.
  const W0 = A.computeWorld(conf.local, conf.rootQ, conf.rootP);
  const hip = W0.thigh_L.p.clone(), knee0 = W0.shank_L.p.clone(), shoulder = W0.hum_L.p.clone();
  const own = (side, v) => side === 'L' ? v : mirror(v);
  // One leg: knee and shin direction are given for the left side and mirrored.
  const leg = (input, side, knee, shin) => {
    const c = copy(input), k = knee.clone().sub(hip).normalize().multiplyScalar(LT).add(hip);
    const ankle = k.clone().addScaledVector(shin.clone().normalize(), LS);
    legIK(c.local, c.rootQ, c.rootP, side, own(side, ankle), own(side, k));
    return c;
  };
  const legs = (input, knee, shin) => leg(leg(input, 'L', knee, shin), 'R', knee, shin);
  const arms = (input, wrist, pole) => {
    const c = copy(input);
    for (const side of ['L', 'R']) armIK(c.local, c.rootQ, c.rootP, side, own(side, wrist), own(side, pole));
    return c;
  };
  const debug = typeof process !== 'undefined' && process.env?.SPIDER_DEBUG;
  const clear = (chain, label = '') => chain.every((c, i) => {
    const hit = collision(c), ok = !hit && (!i || clearEdge(chain[i - 1], c));
    if (!ok && debug) console.log('spider', label, i, JSON.stringify(hit || clearEdge.lastFailure).slice(0, 200));
    return ok;
  });
  // The ankles are crossed, left above right. Raise the left foot, swing
  // the right shin out beneath it, then the left one.
  const shinL = W0.foot_L.p.clone().sub(knee0).normalize();
  const open = V(.3, 0, 1);
  let unfolded = null, prefix = null;
  for (const raise of [.35, .5, .25, .7]) {
    const raised = leg(conf, 'L', knee0, shinL.clone().add(V(0, raise, .1)));
    const rightOut = leg(raised, 'R', knee0, open);
    const both = leg(rightOut, 'L', knee0, open);
    if (clear([conf, raised, rightOut, both], 'unfold')) { unfolded = both; prefix = [raised, rightOut, both]; break; }
  }
  if (!unfolded) return null;
  for (const offset of [V(.25, 0, .05), V(.35, .05, .1), V(.15, 0, 0), V(.3, -.08, 0)]) {
    const off = legs(unfolded, knee0.clone().add(offset), V(.6, 0, 1));
    if (!clear([unfolded, off], 'off')) continue;
    for (const wrist of [V(-.05, -.25, .12), V(-.03, -.32, .05), V(-.08, -.2, .15), V(0, -.4, .02)])
      for (const pole of [V(.08, -.25, -.15), V(.12, -.15, -.1), V(.05, -.3, -.2)]) {
        const lifted = arms(off, shoulder.clone().add(wrist), shoulder.clone().add(pole));
        if (clear([off, lifted, openTarget], 'lift')) return [...prefix, off, lifted];
      }
  }
  return null;
}
