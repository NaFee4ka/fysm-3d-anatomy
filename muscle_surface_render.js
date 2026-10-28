/**
 * BodyParts3D muscle surfaces, fitted to the existing rig at build time.
 * The surfaces use bend-aware dual-quaternion skinning from the segments named
 * by each muscle's attachment/via points. A vertex has at most four influences,
 * but a wide muscle can retain all of its anatomical segment supports. Compact
 * weights vanish where an influence leaves that four-slot palette.
 * Force paths supply the attachments and bounded transverse size correction;
 * strandPath, muscle parameters, and static optimisation results never change.
 *
 * Packed positions are rest-world metres, Uint16 within each part's lo/hi box;
 * positions and triangle indices use the same delta/gzip encoding as bones.
 * CPU picking/contact queries and the GPU use the same rigid palette and blend.
 * Axial twists use DQS; strong bends blend toward LBS to limit DQS bulging.
 * Bounded transverse correction acts on the belly and fades at attachments.
 * These remain geometric approximations, not a measured or finite-element
 * model of tissue deformation or a guarantee of tissue/bone contact.
 */

// A relative half-turn has two equally short quaternion representations. Resolve
// that tie in the lower-index bone's local frame, rather than world coordinates.
// This preserves both antipodal invariance and whole-body rotation covariance.
function muscleDQSign(real, offset, reference, referenceOffset, bone, referenceBone) {
  const x = real[offset], y = real[offset + 1], z = real[offset + 2], w = real[offset + 3];
  const a = reference[referenceOffset], b = reference[referenceOffset + 1], c = reference[referenceOffset + 2], d = reference[referenceOffset + 3];
  const dot = x * a + y * b + z * c + w * d;
  if (Math.abs(dot) > 1e-6) return dot < 0 ? -1 : 1;
  const orientation = bone < referenceBone ? -1 : 1;
  const rx = d * x - a * w - b * z + c * y;
  const ry = d * y - b * w - c * x + a * z;
  const rz = d * z - c * w - a * y + b * x;
  if (Math.abs(rx) > 1e-6) return rx * orientation < 0 ? -1 : 1;
  if (Math.abs(ry) > 1e-6) return ry * orientation < 0 ? -1 : 1;
  if (Math.abs(rz) > 1e-6) return rz * orientation < 0 ? -1 : 1;
  return 1;
}

export function blendMuscleDualQuaternion(indices, weights, real, dual, offset = 0, out = new Float64Array(8)) {
  let referenceSlot = 0;
  for (let j = 1; j < 4; j++) if (weights[offset + j] > weights[offset + referenceSlot]) referenceSlot = j;
  const referenceBone = indices[offset + referenceSlot], referenceOffset = referenceBone * 4;
  out.fill(0);
  for (let j = 0; j < 4; j++) {
    const weight = weights[offset + j];
    if (!weight) continue;
    const bone = indices[offset + j], at = bone * 4;
    const signed = weight * muscleDQSign(real, at, real, referenceOffset, bone, referenceBone);
    for (let k = 0; k < 4; k++) { out[k] += signed * real[at + k]; out[k + 4] += signed * dual[at + k]; }
  }
  let norm = Math.hypot(out[0], out[1], out[2], out[3]);
  if (norm < 1e-12) {
    for (let k = 0; k < 4; k++) { out[k] = real[referenceOffset + k]; out[k + 4] = dual[referenceOffset + k]; }
    norm = Math.hypot(out[0], out[1], out[2], out[3]);
  }
  if (!(norm > 1e-12)) throw new Error('Invalid muscle dual-quaternion palette');
  for (let k = 0; k < 8; k++) out[k] /= norm;
  const study = out[0] * out[4] + out[1] * out[5] + out[2] * out[6] + out[3] * out[7];
  for (let k = 0; k < 4; k++) out[k + 4] -= out[k] * study;
  return out;
}

export function transformMuscleDualQuaternion(point, dq, out) {
  const x = dq[0], y = dq[1], z = dq[2], w = dq[3];
  const dx = dq[4], dy = dq[5], dz = dq[6], dw = dq[7];
  const vx = point.x, vy = point.y, vz = point.z;
  const cx = y * vz - z * vy, cy = z * vx - x * vz, cz = x * vy - y * vx;
  return out.set(vx + 2 * (w * cx + y * cz - z * cy) + 2 * (w * dx - dw * x + y * dz - z * dy),
    vy + 2 * (w * cy + z * cx - x * cz) + 2 * (w * dy - dw * y + z * dx - x * dz),
    vz + 2 * (w * cz + x * cy - y * cx) + 2 * (w * dz - dw * z + x * dy - y * dx));
}

export function deformMuscleVertex(point, axis, indices, weights, real, dual, offset, out, scratch = new Float64Array(8)) {
  const px = point.x, py = point.y, pz = point.z;
  const ax = axis.x, ay = axis.y, az = axis.z;
  blendMuscleDualQuaternion(indices, weights, real, dual, offset, scratch);
  transformMuscleDualQuaternion(point, scratch, out);
  let lx = 0, ly = 0, lz = 0, bx = 0, by = 0, bz = 0, sum = 0;
  for (let j = 0; j < 4; j++) {
    const weight = weights[offset + j];
    if (!weight) continue;
    const k = indices[offset + j] * 4;
    const x = real[k], y = real[k + 1], z = real[k + 2], w = real[k + 3];
    const dx = dual[k], dy = dual[k + 1], dz = dual[k + 2], dw = dual[k + 3];
    const cx = y * pz - z * py, cy = z * px - x * pz, cz = x * py - y * px;
    lx += weight * (px + 2 * (w * cx + y * cz - z * cy) + 2 * (w * dx - dw * x + y * dz - z * dy));
    ly += weight * (py + 2 * (w * cy + z * cx - x * cz) + 2 * (w * dy - dw * y + z * dx - x * dz));
    lz += weight * (pz + 2 * (w * cz + x * cy - y * cx) + 2 * (w * dz - dw * z + x * dy - y * dx));
    const ux = y * az - z * ay, uy = z * ax - x * az, uz = x * ay - y * ax;
    bx += weight * (ax + 2 * (w * ux + y * uz - z * uy));
    by += weight * (ay + 2 * (w * uy + z * ux - x * uz));
    bz += weight * (az + 2 * (w * uz + x * uy - y * ux));
    sum += weight;
  }
  if (sum > 1e-12) {
    // Agreement is one for a common rotation or a pure twist about this local
    // fascicle axis. Bending makes its support directions diverge. The bounded
    // blend is a display correction for DQS bulging, not a tissue parameter.
    const agreement = Math.min(1, Math.hypot(bx, by, bz) / sum);
    const u = Math.max(0, Math.min(1, (1 - agreement) / .65));
    const amount = .8 * u * u * (3 - 2 * u);
    out.x += (lx / sum - out.x) * amount;
    out.y += (ly / sum - out.y) * amount;
    out.z += (lz / sum - out.z) * amount;
  }
  return out;
}

// Exported so a browser test can run the actual blend in transform feedback,
// independently of shading, and compare it with the CPU query implementation.
export function muscleDualQuaternionGLSL(bones) {
  if (!Number.isInteger(bones) || bones < 1 || bones > 64) throw new Error('Invalid muscle palette size');
  return `
attribute vec4 muscleRadial;
attribute vec3 muscleAxis;
uniform vec4 muscleDQReal[${bones}];
uniform vec4 muscleDQDual[${bones}];
uniform float muscleBellyScale;
vec4 muscleQMul(vec4 a, vec4 b) {
  return vec4(a.w*b.xyz+b.w*a.xyz+cross(a.xyz,b.xyz),a.w*b.w-dot(a.xyz,b.xyz));
}
vec3 muscleQRotate(vec4 q, vec3 p) { return p+2.0*cross(q.xyz,cross(q.xyz,p)+q.w*p); }
float muscleQSign(vec4 q, vec4 reference, int bone, int referenceBone) {
  float d=dot(q,reference);
  if(abs(d)>0.000001) return d<0.0?-1.0:1.0;
  vec3 relative=muscleQMul(vec4(-reference.xyz,reference.w),q).xyz;
  if(bone<referenceBone) relative=-relative;
  if(abs(relative.x)>0.000001) return relative.x<0.0?-1.0:1.0;
  if(abs(relative.y)>0.000001) return relative.y<0.0?-1.0:1.0;
  if(abs(relative.z)>0.000001) return relative.z<0.0?-1.0:1.0;
  return 1.0;
}
void muscleBlendDQ(vec4 indices, vec4 weights, out vec4 qr, out vec4 qd) {
  int referenceSlot=0;
  for(int k=1;k<4;k++) if(weights[k]>weights[referenceSlot]) referenceSlot=k;
  int referenceBone=int(indices[referenceSlot]);
  vec4 reference=muscleDQReal[referenceBone];
  qr=vec4(0.0); qd=vec4(0.0);
  for(int k=0;k<4;k++) {
    int bone=int(indices[k]);
    float s=weights[k]*muscleQSign(muscleDQReal[bone],reference,bone,referenceBone);
    qr+=s*muscleDQReal[bone]; qd+=s*muscleDQDual[bone];
  }
  float norm=length(qr);
  if(norm<0.000000000001) {qr=reference;qd=muscleDQDual[referenceBone];norm=length(qr);}
  qr/=norm; qd/=norm; qd-=qr*dot(qr,qd);
}
vec3 muscleDQPoint(vec3 p, vec4 qr, vec4 qd) {
  return muscleQRotate(qr,p)+2.0*muscleQMul(qd,vec4(-qr.xyz,qr.w)).xyz;
}
mat3 muscleQMatrix(vec4 q) {
  float x=q.x,y=q.y,z=q.z,w=q.w;
  return mat3(1.0-2.0*(y*y+z*z),2.0*(x*y+z*w),2.0*(x*z-y*w),
    2.0*(x*y-z*w),1.0-2.0*(x*x+z*z),2.0*(y*z+x*w),
    2.0*(x*z+y*w),2.0*(y*z-x*w),1.0-2.0*(x*x+y*y));
}
void muscleLinearTransform(vec4 indices,vec4 weights,out mat3 rotation,out vec3 translation) {
  rotation=mat3(0.0);translation=vec3(0.0);
  float sum=max(dot(weights,vec4(1.0)),0.000000000001);
  for(int k=0;k<4;k++){
    int bone=int(indices[k]);vec4 q=muscleDQReal[bone],d=muscleDQDual[bone];
    rotation+=(weights[k]/sum)*muscleQMatrix(q);
    translation+=(weights[k]/sum)*2.0*muscleQMul(d,vec4(-q.xyz,q.w)).xyz;
  }
}
float muscleBendBlend(vec3 axis,mat3 rotation) {
  float agreement=clamp(length(rotation*axis),0.0,1.0);
  float u=clamp((1.0-agreement)/0.65,0.0,1.0);
  return 0.8*u*u*(3.0-2.0*u);
}
vec3 muscleHybridPoint(vec3 p,vec3 axis,vec4 indices,vec4 weights,vec4 qr,vec4 qd) {
  mat3 rotation;vec3 translation;muscleLinearTransform(indices,weights,rotation,translation);
  return mix(muscleDQPoint(p,qr,qd),rotation*p+translation,muscleBendBlend(axis,rotation));
}
`;
}

export async function createMuscleSurfaces(A, THREE, data, opts = {}) {
  if (!data || data.v !== 1 || !Array.isArray(data.parts) || !data.parts.length || typeof DecompressionStream === 'undefined') return null;
  const raw = Uint8Array.from(atob(data.data), c => c.charCodeAt(0));
  const buffer = await new Response(new Blob([raw]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  const expected = data.parts.reduce((n, p) => n + 6 * (p.n + p.m), 0);
  if (expected !== buffer.byteLength) throw new Error('Muscle surface data length does not match its manifest');
  const rootSeg = A.SEG.find(s => !s.parent);
  const restW = A.computeWorld(A.STAND.local, A.STAND.rootQ, rootSeg.J.clone());
  const instances = (opts.instances || A.strandInstances()).filter(i => i.kind === 'muscle');
  const pathGroups = new Map();
  const keyOf = (unit, side) => unit + '_' + side;
  for (const inst of instances) {
    const key = keyOf(inst.unit, inst.side);
    if (!pathGroups.has(key)) pathGroups.set(key, []);
    pathGroups.get(key).push(inst);
  }
  const decoded = new Map();
  let offset = 0;
  for (const part of data.parts) {
    if (!Number.isInteger(part.n) || !Number.isInteger(part.m) || part.n < 1 || part.n > 65535 || part.m < 1 || !part.lo || !part.hi || !part.lo.concat(part.hi).every(Number.isFinite)) throw new Error('Invalid muscle surface manifest');
    const q = new Uint16Array(buffer, offset, part.n * 3); offset += part.n * 6;
    const d = new Uint16Array(buffer, offset, part.m * 3); offset += part.m * 6;
    const positions = new Float32Array(part.n * 3), indices = new Uint16Array(part.m * 3);
    const acc = [0, 0, 0];
    for (let v = 0; v < part.n; v++) for (let a = 0; a < 3; a++) {
      acc[a] = (acc[a] + q[3 * v + a]) & 65535;
      positions[3 * v + a] = part.lo[a] + (part.hi[a] - part.lo[a]) * acc[a] / 65535;
    }
    let index = 0;
    for (let k = 0; k < indices.length; k++) {
      index = (index + d[k]) & 65535;
      if (index >= part.n) throw new Error('Muscle surface triangle is outside its vertex array');
      indices[k] = index;
    }
    const key = keyOf(part.unit, part.side);
    if (!A.UNITS[part.unit] || !pathGroups.has(key)) continue;
    if (!decoded.has(key)) decoded.set(key, { unit: part.unit, side: part.side, parts: [] });
    decoded.get(key).parts.push({ positions, indices });
  }

  const group = new THREE.Group(); group.name = 'BodyParts3D muscle surfaces';
  const covered = new Set(), records = [], pickables = [];
  const colorOff = new THREE.Color(0xa94d49), colorOn = new THREE.Color(0x2c9a5a), tendon = new THREE.Color(0xdfcbb7);
  const position = new THREE.Vector3(), color = new THREE.Color(), axis = new THREE.Vector3();
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const yieldUI = () => new Promise(resolve => setTimeout(resolve, 0));
  const nearestDistance = (point, supports) => {
    let distance = Infinity;
    for (const p of supports) distance = Math.min(distance, point.distanceToSquared(p));
    return distance;
  };
  const line = new THREE.Vector3(), foot = new THREE.Vector3(), radial = new THREE.Vector3();
  const radialAxis = new THREE.Vector3(), candidateRadial = new THREE.Vector3();
  function bellyFrame(point, paths, fallbackAxis) {
    let nearest = Infinity;
    radial.set(0, 0, 0); radialAxis.copy(fallbackAxis);
    for (const path of paths) for (let j = 1; j < path.length; j++) {
      line.subVectors(path[j], path[j - 1]);
      const lengthSq = line.lengthSq();
      if (lengthSq < 1e-16) continue;
      const t = clamp(foot.subVectors(point, path[j - 1]).dot(line) / lengthSq, 0, 1);
      foot.copy(path[j - 1]).addScaledVector(line, t);
      candidateRadial.subVectors(point, foot);
      const distance = candidateRadial.lengthSq();
      if (distance < nearest) {
        nearest = distance; radial.copy(candidateRadial); radialAxis.copy(line).normalize();
      }
    }
    // A wide fan is not a cylinder around its middle fascicle. Choose the
    // nearest of all model fascicles and bound the display displacement to
    // 6 mm at the permitted 8% transverse adjustment.
    radial.addScaledVector(radialAxis, -radial.dot(radialAxis));
    if (radial.lengthSq() > .075 * .075) radial.setLength(.075);
  }

  for (const [key, item] of decoded) {
    const paths = pathGroups.get(key);
    // Anatomical segments, rather than tangent frames of a folded force path,
    // provide stable rotations. The previous rotating path cages could turn
    // through 180 degrees near the thumb or a wrapping guide, shredding the skin.
    const supports = new Map();
    for (const inst of paths) for (const attachment of inst.pts) {
      if (!supports.has(attachment.seg)) supports.set(attachment.seg, []);
      supports.get(attachment.seg).push(A.pt(restW, attachment.seg, attachment.p));
    }
    const nodes = [];
    const representative = paths[Math.floor(paths.length / 2)];
    const restPaths = paths.map(inst => A.strandPath(restW, inst));
    const restPath = A.strandPath(restW, representative);
    const restLength = Math.max(1e-6, A.pathLength(restPath));
    axis.subVectors(restPath[restPath.length - 1], restPath[0]);
    if (axis.lengthSq() < 1e-10) axis.set(0, 1, 0);
    axis.normalize();
    const acrossAxis = new THREE.Vector3(Math.abs(axis.x) < 0.8 ? 1 : 0, 0, Math.abs(axis.x) < 0.8 ? 0 : 1);
    acrossAxis.addScaledVector(axis, -acrossAxis.dot(axis)).normalize();
    const attachmentSupports = paths.flatMap(inst => [inst.pts[0], inst.pts[inst.pts.length - 1]])
      .map(p => A.pt(restW, p.seg, p.p));
    const attachmentTintReach = clamp(restLength * 0.07, 0.004, 0.026);
    const basisQ = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
    for (const [seg, points] of supports) {
      const center = new THREE.Vector3();
      for (const p of points) center.add(p);
      center.divideScalar(points.length);
      const bone = new THREE.Bone();
      bone.name = seg;
      bone.position.copy(center); bone.quaternion.copy(basisQ); group.add(bone); bone.updateMatrixWorld(true);
      const node = { seg, supports: points, center, bone, frame: basisQ.clone(), maxOffset: 0 };
      nodes.push(node);
    }
    const vertexCount = item.parts.reduce((sum, p) => sum + p.positions.length / 3, 0);
    const indexCount = item.parts.reduce((sum, p) => sum + p.indices.length, 0);
    const positions = new Float32Array(vertexCount * 3);
    const indices = vertexCount < 65536 ? new Uint16Array(indexCount) : new Uint32Array(indexCount);
    const skinIndices = new Uint16Array(vertexCount * 4), skinWeights = new Float32Array(vertexCount * 4);
    const radialValues = new Float32Array(vertexCount * 4), axisValues = new Float32Array(vertexCount * 3);
    const colors = new Float32Array(vertexCount * 3), onColors = new Float32Array(vertexCount * 3), uv = new Float32Array(vertexCount * 2);
    const normalSeams = [];
    const sourceBox = new THREE.Box3();
    for (const part of item.parts) for (let k = 0; k < part.positions.length; k += 3) sourceBox.expandByPoint(position.fromArray(part.positions, k));
    const size = sourceBox.getSize(new THREE.Vector3());
    const softness = clamp(Math.max(size.x, size.y, size.z) * 0.12, 0.006, 0.05) ** 2;
    // Some source tendons extend beyond the simplified first/last force point.
    // Where every fascicle has the same endpoint segment, tissue beyond that
    // anatomical footprint must remain on that segment (not drift toward a hand
    // or finger). Broad fans with origins on several bones do not get this pin.
    const endpointPins = [];
    for (const end of [0, -1]) {
      const ends = paths.map(inst => end === 0 ? inst.pts[0] : inst.pts.at(-1));
      if (!ends.every(p => p.seg === ends[0].seg)) continue;
      const projections = ends.map(p => A.pt(restW, p.seg, p.p).dot(axis));
      endpointPins.push({
        index: nodes.findIndex(node => node.seg === ends[0].seg),
        limit: end === 0 ? Math.min(...projections) : Math.max(...projections),
        direction: end === 0 ? -1 : 1,
        reach: clamp(Math.max(size.x, size.y, size.z) * 0.03, 0.004, 0.015),
      });
    }
    let vOffset = 0, iOffset = 0;
    for (const part of item.parts) {
      positions.set(part.positions, vOffset * 3);
      for (let k = 0; k < part.indices.length; k++) indices[iOffset + k] = vOffset + part.indices[k];
      // Preserve geometric components and exact source seams. Anatomical support
      // weights below do not depend on vertex density or component tessellation.
      const count = part.positions.length / 3, parents = new Int32Array(count);
      for (let k = 0; k < count; k++) parents[k] = k;
      const find = k => { while (parents[k] !== k) { parents[k] = parents[parents[k]]; k = parents[k]; } return k; };
      for (let k = 0; k < part.indices.length; k += 3) {
        const a = find(part.indices[k]), b = find(part.indices[k + 1]), c = find(part.indices[k + 2]);
        parents[b] = a; parents[c] = a;
      }
      // OBJ normals/UV seams may repeat an identical XYZ with a different index.
      // Treat those vertices as one geometric component for skinning; otherwise
      // the two sides of an unwelded seam can choose different bone palettes and
      // visibly pull apart. Never merge merely nearby points or different parts.
      const coincident = new Map();
      for (let k = 0; k < count; k++) {
        const key = part.positions[k * 3] + '|' + part.positions[k * 3 + 1] + '|' + part.positions[k * 3 + 2];
        if (!coincident.has(key)) coincident.set(key, []);
        const same = coincident.get(key);
        if (same.length) parents[find(k)] = find(same[0]);
        same.push(k);
      }
      for (const same of coincident.values()) if (same.length > 1) normalSeams.push(same.map(k => vOffset + k));
      const components = new Map();
      for (let k = 0; k < count; k++) { const component = find(k); if (!components.has(component)) components.set(component, []); components.get(component).push(k); }
      for (const vertices of components.values()) {
        const candidates = nodes.map((node, index) => ({ index, distance: 0 }));
        for (const localIndex of vertices) {
          const vertex = vOffset + localIndex;
          position.fromArray(part.positions, localIndex * 3);
          for (const candidate of candidates) candidate.distance = nearestDistance(position, nodes[candidate.index].supports);
          candidates.sort((a, b) => a.distance - b.distance || a.index - b.index);
          const influenceCount = Math.min(4, candidates.length);
          // At a fourth/fifth-nearest switch their distances are equal, so the
          // outgoing and incoming weight both reach zero. Ordinary nearest-four
          // inverse-distance weights would change abruptly at that boundary.
          const cutoff = candidates.length > 4 ? candidates[4].distance : Infinity;
          let sum = 0;
          for (let j = 0; j < influenceCount; j++) {
            const { index, distance } = candidates[j];
            const compact = Number.isFinite(cutoff) ? Math.max(0, 1 - distance / Math.max(cutoff, 1e-20)) ** 2 : 1;
            const weight = compact / (distance + softness);
            skinIndices[vertex * 4 + j] = index; skinWeights[vertex * 4 + j] = weight; sum += weight;
            nodes[index].maxOffset = Math.max(nodes[index].maxOffset, position.distanceTo(nodes[index].center));
          }
          // A support that is clearly closer than the next anatomical support
          // carries its local tissue rigidly. This retains sparse tendon ends,
          // without mistaking the bounding-box edges of a broad fan for tendons.
          const ratio = candidates.length > 1 ? Math.sqrt(candidates[0].distance / Math.max(candidates[1].distance, 1e-20)) : 0;
          const pin = clamp((0.45 - ratio) / 0.25, 0, 1);
          const pinWeight = pin * pin * (3 - 2 * pin);
          if (sum < 1e-30) { skinWeights[vertex * 4] = 1; sum = 1; }
          for (let j = 0; j < influenceCount; j++) {
            skinWeights[vertex * 4 + j] = (1 - pinWeight) * skinWeights[vertex * 4 + j] / sum + (j === 0 ? pinWeight : 0);
          }
          let endpointStrength = 0;
          for (const endpoint of endpointPins) {
            // Pin only while this anatomical endpoint is the closest support;
            // fade to zero before that nearest-support identity changes. This
            // also keeps an endpoint from bypassing the compact palette cutoff.
            if (candidates[0].index !== endpoint.index) continue;
            const outside = endpoint.direction * (position.dot(axis) - endpoint.limit);
            const u = clamp(outside / endpoint.reach, 0, 1);
            const dominance = clamp((1 - ratio) / 0.3, 0, 1);
            const strength = u * u * (3 - 2 * u) * dominance * dominance * (3 - 2 * dominance);
            endpointStrength = Math.max(endpointStrength, strength);
            for (let j = 0; j < influenceCount; j++) skinWeights[vertex * 4 + j] =
              (1 - strength) * skinWeights[vertex * 4 + j] + (j === 0 ? strength : 0);
          }
          // A short attachment tint is an appearance approximation, not a
          // segmented tendon map. Projecting an entire fan on one strand and
          // clamping its ends used to bleach large angular plates of abdomen
          // and chest. Distance to actual attachment footprints stays local.
          const belly = clamp(Math.sqrt(nearestDistance(position, attachmentSupports)) / attachmentTintReach, 0, 1);
          const soft = belly * belly * (3 - 2 * belly);
          bellyFrame(position, restPaths, axis);
          radial.toArray(radialValues, vertex * 4);
          radialValues[vertex * 4 + 3] = soft * (1 - endpointStrength);
          radialAxis.toArray(axisValues, vertex * 3);
          color.copy(tendon).lerp(colorOff, soft).toArray(colors, vertex * 3);
          color.copy(tendon).lerp(colorOn, soft).toArray(onColors, vertex * 3);
          // Physical rest coordinates give comparable fascicle size on large
          // and small muscles. U is perpendicular to the longitudinal axis;
          // the subtle canvas strands run along V and deform with the surface.
          uv[vertex * 2] = position.dot(acrossAxis) * 20;
          uv[vertex * 2 + 1] = position.dot(axis) * 10;
        }
      }
      vOffset += count; iOffset += part.indices.length;
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('skinIndex', new THREE.BufferAttribute(skinIndices, 4));
    geometry.setAttribute('skinWeight', new THREE.BufferAttribute(skinWeights, 4));
    geometry.setAttribute('muscleRadial', new THREE.BufferAttribute(radialValues, 4));
    geometry.setAttribute('muscleAxis', new THREE.BufferAttribute(axisValues, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1)); geometry.computeVertexNormals();
    // Smooth normals across exact source seams only. Opposing faces of a thin
    // lamina retain their own normal rather than cancelling to a dark zero.
    const normals = geometry.attributes.normal;
    for (const seam of normalSeams) {
      const groups = [];
      for (const vertex of seam) {
        const normal = new THREE.Vector3().fromBufferAttribute(normals, vertex);
        if (normal.lengthSq() < 1e-12) continue;
        let same = groups.find(g => g.direction.dot(normal) >= 0.5);
        if (!same) { same = { direction: normal.clone(), sum: new THREE.Vector3(), vertices: [] }; groups.push(same); }
        same.sum.add(normal); same.vertices.push(vertex);
      }
      for (const same of groups) {
        same.sum.normalize();
        for (const vertex of same.vertices) normals.setXYZ(vertex, same.sum.x, same.sum.y, same.sum.z);
      }
    }
    const material = new THREE.MeshStandardMaterial({ vertexColors: true, map: opts.fiberTexture || null, roughness: 0.7, metalness: 0 });
    // Rectus is a strap muscle with the three intersections already specified
    // in the model. Mark those individual stations in the fragment material:
    // vertex colours alone would smear a 3 mm mark across 9 mm mesh sections.
    let intersectionMarks = null;
    if (item.unit === 'rectAbd' && opts.fiberTexture && representative.dips?.length) {
      const marks = [1e6, 1e6, 1e6];
      representative.dips.slice(0, 3).forEach((fraction, k) => {
        let remaining = clamp(fraction, 0, 1) * restLength;
        for (let i = 1; i < restPath.length; i++) {
          const length = restPath[i - 1].distanceTo(restPath[i]);
          if (remaining <= length || i === restPath.length - 1) {
            marks[k] = restPath[i - 1].clone().lerp(restPath[i], length > 0 ? clamp(remaining / length, 0, 1) : 0).dot(axis) * 10;
            break;
          }
          remaining -= length;
        }
      });
      intersectionMarks = new THREE.Vector3(...marks);
    }
    const deformation = {
      real: new Float32Array(nodes.length * 4), dual: new Float32Array(nodes.length * 4),
      scale: { value: 1 }, radial: radialValues, axis: axisValues,
    };
    material.onBeforeCompile = shader => {
      shader.uniforms.muscleDQReal = { value: deformation.real };
      shader.uniforms.muscleDQDual = { value: deformation.dual };
      shader.uniforms.muscleBellyScale = deformation.scale;
      shader.vertexShader = muscleDualQuaternionGLSL(nodes.length) + shader.vertexShader;
      shader.vertexShader = shader.vertexShader.replace('#include <skinbase_vertex>', `
        vec4 muscleReal, muscleDual;
        muscleBlendDQ(skinIndex,skinWeight,muscleReal,muscleDual);
        mat3 muscleLinearRotation;vec3 muscleLinearTranslation;
        muscleLinearTransform(skinIndex,skinWeight,muscleLinearRotation,muscleLinearTranslation);
        float muscleBend=muscleBendBlend(muscleAxis,muscleLinearRotation);
        float muscleTransverse=1.0+(muscleBellyScale-1.0)*muscleRadial.w;
      `);
      shader.vertexShader = shader.vertexShader.replace('#include <skinnormal_vertex>', `
        // Inverse-transpose of the local transverse stretch, followed by the
        // blended rigid rotation. Weight/attachment gradients remain a small
        // geometric approximation, rather than a constitutive tissue normal.
        vec3 muscleLocalNormal=objectNormal/muscleTransverse
          +(1.0-1.0/muscleTransverse)*muscleAxis*dot(muscleAxis,objectNormal);
        vec3 muscleRigidNormal=muscleQRotate(muscleReal,muscleLocalNormal);
        mat3 muscleCofactor=mat3(cross(muscleLinearRotation[1],muscleLinearRotation[2]),
          cross(muscleLinearRotation[2],muscleLinearRotation[0]),cross(muscleLinearRotation[0],muscleLinearRotation[1]));
        vec3 muscleLinearNormal=muscleCofactor*muscleLocalNormal;
        if(dot(muscleLinearNormal,muscleLinearNormal)>0.000000000001) muscleLinearNormal=normalize(muscleLinearNormal);
        else muscleLinearNormal=muscleRigidNormal;
        objectNormal=mix(normalize(muscleRigidNormal),muscleLinearNormal,muscleBend);
        if(dot(objectNormal,objectNormal)<0.000000000001)objectNormal=muscleRigidNormal;
        #ifdef USE_TANGENT
          vec3 muscleLocalTangent=muscleTransverse*objectTangent
            +(1.0-muscleTransverse)*muscleAxis*dot(muscleAxis,objectTangent);
          objectTangent=mix(muscleQRotate(muscleReal,muscleLocalTangent),muscleLinearRotation*muscleLocalTangent,muscleBend);
        #endif
      `);
      shader.vertexShader = shader.vertexShader.replace('#include <skinning_vertex>', `
        vec3 musclePoint=transformed+(muscleTransverse-1.0)*muscleRadial.xyz;
        transformed=mix(muscleDQPoint(musclePoint,muscleReal,muscleDual),
          muscleLinearRotation*musclePoint+muscleLinearTranslation,muscleBend);
      `);
      if (intersectionMarks) {
        shader.uniforms.muscleIntersections = { value: intersectionMarks };
        shader.uniforms.muscleTendonColor = { value: tendon };
        shader.fragmentShader = 'uniform vec3 muscleIntersections;\nuniform vec3 muscleTendonColor;\n' + shader.fragmentShader;
        shader.fragmentShader = shader.fragmentShader.replace('#include <color_fragment>', `#include <color_fragment>
          float intersectionDistance = min(min(abs(vMapUv.y - muscleIntersections.x), abs(vMapUv.y - muscleIntersections.y)), abs(vMapUv.y - muscleIntersections.z));
          float intersectionTint = 0.35 * exp(-pow(intersectionDistance / 0.018, 2.0));
          diffuseColor.rgb = mix(diffuseColor.rgb, muscleTendonColor, intersectionTint);`);
      }
    };
    material.customProgramCacheKey = () => 'muscle-dqs-bend-v2-' + nodes.length + (intersectionMarks ? '-rectus' : '');
    const mesh = new THREE.SkinnedMesh(geometry, material);
    mesh.userData = { kind: 'muscle', unit: item.unit, side: item.side, anatomicalSurface: true, skinningMethod: 'bend-aware-dual-quaternion' };
    mesh.frustumCulled = false;
    group.add(mesh); group.updateMatrixWorld(true);
    const skeleton = new THREE.Skeleton(nodes.map(node => node.bone));
    mesh.bind(skeleton, new THREE.Matrix4());
    const dq = new Float64Array(8), vertexAxis = new THREE.Vector3();
    // SkinnedMesh.getVertexPosition and the built-in triangle raycaster both
    // dispatch through this method. Contact quadrature must call that public
    // vertex query too, rather than reproducing the former linear matrix sum.
    mesh.applyBoneTransform = (vertex, target) => {
      const offset = vertex * 4;
      const amount = (deformation.scale.value - 1) * radialValues[offset + 3];
      target.x += radialValues[offset] * amount;
      target.y += radialValues[offset + 1] * amount;
      target.z += radialValues[offset + 2] * amount;
      vertexAxis.fromArray(axisValues, vertex * 3);
      return deformMuscleVertex(target, vertexAxis, skinIndices, skinWeights, deformation.real, deformation.dual, offset, target, dq);
    };
    mesh.muscleDeformation = deformation;
    mesh.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 4);
    records.push({ key, unit: item.unit, side: item.side, mesh, skeleton, nodes, representative, restLength,
      deformation, colorOff: colors.slice(), colorOn: onColors, on: false });
    pickables.push(mesh); covered.add(key);
    if (records.length % 5 === 0) await yieldUI();
  }

  const rigidQ = new THREE.Quaternion(), translation = new THREE.Vector3(), rotatedCenter = new THREE.Vector3();
  const boundsCenter = new THREE.Vector3(), boundsSize = new THREE.Vector3();
  function update(W, paths) {
    for (const rec of records) {
      const path = paths && paths.get(rec.representative.id) || A.strandPath(W, rec.representative);
      // Only the muscle belly receives a bounded transverse correction. The
      // anatomical support transforms stay rigid and attachment masks stay at
      // zero; scaling every bone formerly displaced even pinned tendon ends.
      const radius = Math.fround(clamp(Math.sqrt(rec.restLength / Math.max(1e-6, A.pathLength(path))), 0.92, 1.08));
      rec.deformation.scale.value = radius;
      const box = new THREE.Box3();
      for (let index = 0; index < rec.nodes.length; index++) {
        const node = rec.nodes[index];
        const bone = node.bone;
        bone.position.copy(A.pt(W, node.seg, node.center));
        rigidQ.copy(W[node.seg].q);
        if (![rigidQ.x, rigidQ.y, rigidQ.z, rigidQ.w].every(Number.isFinite) || rigidQ.lengthSq() < 1e-16)
          throw new Error('Invalid muscle support rotation: ' + node.seg);
        rigidQ.normalize();
        bone.quaternion.copy(rigidQ).multiply(node.frame);
        bone.scale.set(1, 1, 1); bone.updateMatrixWorld(true);
        translation.copy(bone.position).sub(rotatedCenter.copy(node.center).applyQuaternion(rigidQ));
        const x = rigidQ.x, y = rigidQ.y, z = rigidQ.z, w = rigidQ.w;
        const tx = translation.x, ty = translation.y, tz = translation.z, at = index * 4;
        rec.deformation.real.set([x, y, z, w], at);
        rec.deformation.dual.set([.5 * (tx * w + ty * z - tz * y), .5 * (-tx * z + ty * w + tz * x),
          .5 * (tx * y - ty * x + tz * w), -.5 * (tx * x + ty * y + tz * z)], at);
        box.expandByPoint(bone.position);
      }
      box.getCenter(boundsCenter);
      let reach = 0;
      for (const node of rec.nodes) reach = Math.max(reach, node.bone.position.distanceTo(boundsCenter) + node.maxOffset + .006);
      // With at most four nonnegative weights the largest-weight reference is
      // >=1/4. Hemisphere alignment gives |sum(w*q)|>=1/4 (up to the half-turn
      // tie epsilon). A dual-quaternion point lies within 4.01 times the largest
      // rigid support reach. This conservative bound includes the 6 mm belly
      // correction and is shared by the CPU raycaster and focus queries.
      const bound = Math.max(.001, reach * 4.01 + .001);
      rec.mesh.boundingSphere.set(boundsCenter, bound);
      boundsSize.setScalar(bound);
      rec.bounds = new THREE.Box3(boundsCenter.clone().sub(boundsSize), boundsCenter.clone().add(boundsSize));
      rec.preciseBounds = null;
      rec.skeleton.update();
    }
    group.updateMatrixWorld(true);
  }
  function setVisibility(visible) { for (const rec of records) rec.mesh.visible = !!visible(rec.unit, rec.side); }
  function setAppearance(appearance) {
    for (const rec of records) {
      const a = appearance(rec.unit, rec.side), material = rec.mesh.material;
      if (rec.on !== !!a.active) {
        rec.on = !!a.active;
        rec.mesh.geometry.attributes.color.array.set(rec.on ? rec.colorOn : rec.colorOff);
        rec.mesh.geometry.attributes.color.needsUpdate = true;
      }
      if (a.emissive) material.emissive.copy(a.emissive);
      material.emissiveIntensity = a.intensity || 0;
      const opacity = a.opacity === undefined ? 1 : a.opacity;
      if (material.opacity !== opacity) { material.opacity = opacity; material.transparent = opacity < 1; material.depthWrite = opacity >= 1; material.needsUpdate = true; }
    }
  }
  function getBounds(unit, side) {
    const box = new THREE.Box3();
    for (const rec of records) if (rec.unit === unit && (!side || side === rec.side) && rec.bounds) {
      // The broad-phase DQS bound is intentionally generous. Compute and cache
      // actual vertex bounds only when a card/focus query needs them, so the
      // camera does not back away fourfold from a selected muscle.
      if (!rec.preciseBounds) {
        rec.preciseBounds = new THREE.Box3();
        for (let vertex = 0; vertex < rec.mesh.geometry.attributes.position.count; vertex++)
          rec.preciseBounds.expandByPoint(rec.mesh.getVertexPosition(vertex, position));
      }
      box.union(rec.preciseBounds);
    }
    return box.isEmpty() ? null : box;
  }
  function dispose() {
    for (const rec of records) { rec.mesh.geometry.dispose(); rec.mesh.material.dispose(); rec.skeleton.dispose(); }
    group.removeFromParent();
  }
  update(opts.world || restW);
  return { group, covered, pickables, update, setVisibility, setAppearance, getBounds, dispose, skinningMethod: 'bend-aware-dual-quaternion' };
}
