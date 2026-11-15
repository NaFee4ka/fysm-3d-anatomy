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
  const hip = W0.thigh_L.p.clone(), knee0 = W0.shank_L.p.clone();
  const legs = (knee, shin) => {
    const c = copy(conf), k = knee.clone().sub(hip).normalize().multiplyScalar(LT).add(hip);
    const ankle = k.clone().addScaledVector(shin.clone().normalize(), LS);
    for (const side of ['L', 'R'])
      legIK(c.local, c.rootQ, c.rootP, side, side === 'L' ? ankle : mirror(ankle), side === 'L' ? k : mirror(k));
    return c;
  };
  const arms = (input, wrist, pole) => {
    const c = copy(input);
    for (const side of ['L', 'R'])
      armIK(c.local, c.rootQ, c.rootP, side, side === 'L' ? wrist : mirror(wrist), side === 'L' ? pole : mirror(pole));
    return c;
  };
  const clear = chain => chain.every((c, i) => !collision(c) && (!i || clearEdge(chain[i - 1], c)));
  const unfolded = legs(knee0, V(.3, 0, 1));
  if (!clear([conf, unfolded])) return null;
  for (const offset of [V(.25, 0, .05), V(.35, .05, .1), V(.15, 0, 0)]) {
    const off = legs(knee0.clone().add(offset), V(.6, 0, 1));
    if (!clear([unfolded, off])) continue;
    for (const wrist of [V(.15, .35, 0), V(.18, .25, -.1), V(.12, .4, -.05)]) for (const pole of [V(.25, .3, .15), V(.3, .45, 0)]) {
      const lifted = arms(off, wrist, pole);
      if (clear([off, lifted, openTarget])) return [unfolded, off, lifted];
    }
  }
  return null;
}
