// A display-only arm exit from the tightly folded Spider position.
// Keep the arms behind the trunk while the folded legs unfold. Opening the
// arms laterally before that body change cuts the upper arms through a shin.
export function prepareSpiderRelease({ A, THREE, conf, collision, clearEdge, openTarget }) {
  if (!conf || !openTarget || typeof collision !== 'function' || typeof clearEdge !== 'function')
    throw new Error('Spider release requires source, target and actual geometry guards');
  const x = new THREE.Vector3(1, 0, 0);
  const shoulder = new THREE.Quaternion().setFromAxisAngle(x, Math.PI / 4);
  const elbow = new THREE.Quaternion().setFromAxisAngle(x, -20 * Math.PI / 180);
  const thumb = new THREE.Quaternion().setFromAxisAngle(x, -35 * Math.PI / 180);
  const stages = [conf, openTarget].map(input => {
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
  });
  const chain = [conf, ...stages, openTarget];
  for (let i = 0; i < chain.length; i++) {
    if (collision(chain[i]) || i && !clearEdge(chain[i - 1], chain[i])) return null;
  }
  return stages;
}
