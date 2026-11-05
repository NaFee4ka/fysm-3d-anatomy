// Offline floor envelope of exactly the surfaces shown by the page. This is a
// geometric clearance check, not a skin-contact or pressure model.
import { decodeGroundBones } from './practice/ground_fit.mjs';

export function createTransitionFloor(A, THREE, contactFactory, boneData) {
  const surfaces = contactFactory.surfaces;
  const muscles = surfaces.pickables.filter(mesh => !A.UNITS[mesh.userData.unit]?.deep);
  const bones = decodeGroundBones(boneData, A), point = new THREE.Vector3();
  let evaluations = 0;
  const counts = { muscles: muscles.reduce((n, m) => n + m.geometry.getAttribute('position').count, 0),
    bones: bones.reduce((n, b) => n + b.positions.length / 3, 0), markers: A.LANDMARKS.length };
  function minimum(conf, id) {
    evaluations++;
    const W = A.computeWorld(conf.local, conf.rootQ, conf.rootP), proxies = contactFactory.groundProxies(W, { id });
    surfaces.update(W); surfaces.group?.updateMatrixWorld(true);
    let minimumY = Infinity, witness = null;
    const add = (y, record) => { if (y < minimumY) { minimumY = y; witness = record; } };
    for (const mesh of muscles) {
      const count = mesh.geometry.getAttribute('position').count;
      for (let i = 0; i < count; i++) {
        mesh.getVertexPosition(i, point); mesh.localToWorld(point);
        if (point.y < minimumY) { minimumY = point.y; witness = { kind: 'muscle', unit: mesh.userData.unit, side: mesh.userData.side, vertex: i }; }
      }
    }
    for (const bone of bones) {
      const T = W[bone.seg], q = T.q, p = T.p;
      // Only the y row of the exact quaternion transform is needed. The same
      // normalization policy as computeWorld / the shown bone mesh is retained.
      const xx = q.x * 2, yy = q.y * 2, zz = q.z * 2;
      const m10 = q.x * yy + q.w * zz, m11 = 1 - (q.x * xx + q.z * zz), m12 = q.y * zz - q.w * xx;
      for (let i = 0; i < bone.positions.length; i += 3) {
        const y = m10 * bone.positions[i] + m11 * bone.positions[i + 1] + m12 * bone.positions[i + 2] + p.y;
        if (y < minimumY) { minimumY = y; witness = { kind: 'bone', part: bone.id, segment: bone.seg, vertex: i / 3 }; }
      }
    }
    for (const [index, marker] of A.LANDMARKS.entries()) {
      if (id === 'detali-08' && proxies.length && /^(hand|f[1-5][mabc])_[LR]$/.test(marker.seg)
          && ['ладони', 'пальцы рук', 'кулаки', 'кисти'].includes(marker.region)) continue;
      point.copy(marker.pc).sub(A.S[marker.seg].J).applyQuaternion(W[marker.seg].q).add(W[marker.seg].p);
      add(point.y, { kind: 'marker', index, segment: marker.seg });
    }
    for (const proxy of proxies) add(proxy.point.y, { kind: 'proxy', id: proxy.id, segment: proxy.seg });
    if (!Number.isFinite(minimumY)) throw new Error('Invalid transition floor geometry');
    return { minimumY, witness };
  }
  return { minimum, counts, get evaluations() { return evaluations; }, scope: 'all-bones-outer-muscles-skin-markers-and-explicit-proxies' };
}
