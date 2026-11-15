// A display-only arm exit from the tightly folded Spider position.
// Keep the arms behind the trunk while the folded legs unfold. Opening the
// arms laterally before that body change cuts the upper arms through a shin.
// Pelvis-relative leg rotations of the earlier Spider reconstruction, used only
// as an intermediate release waypoint (same trunk and root orientation).
const LEGS_FORWARD = {
  thigh_L: [-0.903196609, 0.299734073, -0.303395253, -0.048442672], shank_L: [0.808809906, 0, 0, 0.588070179],
  foot_L: [0.369192715, -0.042153795, 0.424396338, 0.825716383], toes_L: [0.130526192, 0, 0, 0.991444861],
  thigh_R: [-0.881082244, -0.307204975, 0.352222142, -0.072517202], shank_R: [0.777643201, 0, 0, 0.628705855],
  foot_R: [0.424003092, 0.041135052, -0.465556915, 0.775748699], toes_R: [0.130526192, 0, 0, 0.991444861],
};
export function prepareSpiderRelease({ A, THREE, conf, collision, clearEdge, openTarget }) {
  if (!conf || !openTarget || typeof collision !== 'function' || typeof clearEdge !== 'function')
    throw new Error('Spider release requires source, target and actual geometry guards');
  const x = new THREE.Vector3(1, 0, 0);
  const shoulder = new THREE.Quaternion().setFromAxisAngle(x, Math.PI / 4);
  const elbow = new THREE.Quaternion().setFromAxisAngle(x, -20 * Math.PI / 180);
  const thumb = new THREE.Quaternion().setFromAxisAngle(x, -35 * Math.PI / 180);
  // The knees rest on the backs of the upper arms. First extend the knees,
  // then move only the legs to the earlier reconstruction (shins
  // forward at knee height); withdrawing the arms directly would cut through
  // the thighs.
  const copy = input => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, input.local[s.name].clone()])),
    rootQ: input.rootQ.clone(), rootP: input.rootP.clone() });
  const legsForward = copy(conf);
  for (const [name, q] of Object.entries(LEGS_FORWARD)) legsForward.local[name] = new THREE.Quaternion(...q).normalize();
  // Extend each knee first: the shin, which crosses in front of the forearm,
  // swings forward and up while the knee stays where it rests.
  const rotated = (input, degrees) => {
    const c = copy(input), axis = new THREE.Vector3(1, 0, 0);
    for (const side of ['L', 'R']) {
      const turn = new THREE.Quaternion().setFromAxisAngle(axis, -degrees * Math.PI / 180);
      c.local['shank_' + side].premultiply(side === 'L' ? turn : A.mirrorQ(turn)).normalize();
    }
    return c;
  };
  const turned = (input, axisName, degrees, segment) => {
    const c = copy(input), axis = new THREE.Vector3(...(axisName === 'x' ? [1, 0, 0] : [0, 1, 0]));
    for (const side of ['L', 'R']) {
      const turn = new THREE.Quaternion().setFromAxisAngle(axis, (axisName === 'y' && side === 'R' ? -1 : 1) * degrees * Math.PI / 180);
      const q = axisName === 'x' && side === 'R' ? A.mirrorQ(turn) : turn;
      c.local[segment + '_' + side].multiply(q).normalize();
    }
    return c;
  };
  const clear = chain => {
    for (let i = 0; i < chain.length; i++) {
      const hit = collision(chain[i]);
      if (hit) return { i, hit };
      if (i && !clearEdge(chain[i - 1], chain[i])) return { i, edge: clearEdge.lastFailure };
    }
    return null;
  };
  let legStages = null, armBase = null;
  search: for (const rot of [0, 15, -15, 30, -30]) for (const ext of [20, 35, 50, 65]) {
    const swing = rot ? turned(conf, 'y', rot, 'thigh') : conf;
    const lowered = turned(swing, 'x', ext, 'thigh');
    const legs = rot ? [swing, lowered] : [lowered];
    const failure = clear([conf, ...legs, ...armStages(lowered), openTarget]);
    if (process.env.SPIDER_DEBUG) console.log('SPIDER try', rot, ext, failure ? JSON.stringify(failure).slice(0, 220) : 'OK');
    if (!failure) { legStages = legs; armBase = lowered; break search; }
  }
  if (!legStages) return null;
  function armStages(base) { return [base, openTarget].map(input => {
    const c = { local: Object.fromEntries(A.SEG.map(s => [s.name, input.local[s.name].clone()])),
      rootQ: input.rootQ.clone(), rootP: input.rootP.clone() };
    // A common rigid placement does not change thorax-relative shoulder
    // angles. Solve in the canonical body frame to retain identical locals.
    const W = A.computeWorld(c.local, new THREE.Quaternion(), new THREE.Vector3());
    for (const side of ['L', 'R']) {
      const target = W.thC.q.clone().multiply(shoulder);
      c.local['hum_' + side] = W['scap_' + side].q.clone().invert().multiply(target).normalize();
      c.local['fore_' + side] = side === 'L' ? elbow.clone() : A.mirrorQ(elbow);
      c.local['ulna_' + side].identity();
      c.local['hand_' + side].identity();
      for (const s of A.SEG) if (/^f[1-5]/.test(s.name) && s.name.endsWith('_' + side)) c.local[s.name].identity();
      c.local['f1m_' + side] = side === 'L' ? thumb.clone() : A.mirrorQ(thumb);
    }
    return c;
  }); }
  return [...legStages, ...armStages(armBase)];
}
