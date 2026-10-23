/**
 * BodyParts3D muscle surfaces, fitted to the existing rig at build time.
 * The surfaces use GPU linear blend skinning from the anatomical segments named
 * by each muscle's attachment/via points. Every connected source component has
 * one fixed palette of up to four segments and smooth spatial influence weights.
 * This avoids nearest-node switches and tangent-frame flips of folded paths.
 * Force paths supply the attachments and bounded transverse size correction;
 * strandPath, muscle parameters, and static optimisation results never change.
 *
 * Packed positions are rest-world metres, Uint16 within each part's lo/hi box;
 * positions and triangle indices use the same delta/gzip encoding as bones.
 * Skin weights, volume correction, and tissue/bone contact remain visual
 * approximations. Linear skinning can flatten muscles at deep joint flexion;
 * it is not a measured or finite-element model of tissue deformation.
 */
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
  const colorOff = new THREE.Color(0xa8434b), colorOn = new THREE.Color(0x2c9a5a), tendon = new THREE.Color(0xe9dfcd);
  const position = new THREE.Vector3(), color = new THREE.Color(), axis = new THREE.Vector3();
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const yieldUI = () => new Promise(resolve => setTimeout(resolve, 0));
  const nearestDistance = (point, supports) => {
    let distance = Infinity;
    for (const p of supports) distance = Math.min(distance, point.distanceToSquared(p));
    return distance;
  };

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
    const restPath = A.strandPath(restW, representative);
    const restLength = Math.max(1e-6, A.pathLength(restPath));
    axis.subVectors(restPath[restPath.length - 1], restPath[0]);
    if (axis.lengthSq() < 1e-10) axis.set(0, 1, 0);
    axis.normalize();
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
    const colors = new Float32Array(vertexCount * 3), onColors = new Float32Array(vertexCount * 3), uv = new Float32Array(vertexCount * 2);
    const sourceBox = new THREE.Box3();
    for (const part of item.parts) for (let k = 0; k < part.positions.length; k += 3) sourceBox.expandByPoint(position.fromArray(part.positions, k));
    const size = sourceBox.getSize(new THREE.Vector3());
    const softness = clamp(Math.max(size.x, size.y, size.z) * 0.12, 0.006, 0.05) ** 2;
    let vOffset = 0, iOffset = 0;
    for (const part of item.parts) {
      positions.set(part.positions, vOffset * 3);
      for (let k = 0; k < part.indices.length; k++) indices[iOffset + k] = vOffset + part.indices[k];
      // A packed anatomical part may contain disconnected muscles/tendons. Each
      // connected component uses one fixed palette of up to four attachments.
      // Within that component all weights are smooth functions of position;
      // there is no discontinuous nearest-neighbour selection per vertex.
      const count = part.positions.length / 3, parents = new Int32Array(count);
      for (let k = 0; k < count; k++) parents[k] = k;
      const find = k => { while (parents[k] !== k) { parents[k] = parents[parents[k]]; k = parents[k]; } return k; };
      for (let k = 0; k < part.indices.length; k += 3) {
        const a = find(part.indices[k]), b = find(part.indices[k + 1]), c = find(part.indices[k + 2]);
        parents[b] = a; parents[c] = a;
      }
      const components = new Map();
      for (let k = 0; k < count; k++) { const component = find(k); if (!components.has(component)) components.set(component, []); components.get(component).push(k); }
      for (const vertices of components.values()) {
        const scores = nodes.map((node, index) => ({ index, score: 0 }));
        const stride = Math.max(1, Math.floor(vertices.length / 80));
        for (let k = 0; k < vertices.length; k += stride) {
          position.fromArray(part.positions, vertices[k] * 3);
          for (const score of scores) score.score += 1 / (nearestDistance(position, nodes[score.index].supports) + softness) ** 2;
        }
        scores.sort((a, b) => b.score - a.score);
        // Vertex density is not anatomical importance: a finely tessellated
        // distal palm used to fill the entire palette with finger bones and
        // leave a long, sparsely tessellated forearm tendon attached to the hand.
        // Retain the nearest support at both ends of this component's longest
        // axis before filling the remaining palette from its bulk affinities.
        const componentBox = new THREE.Box3();
        for (const index of vertices) componentBox.expandByPoint(position.fromArray(part.positions, index * 3));
        const span = componentBox.getSize(new THREE.Vector3());
        const majorAxis = span.x > span.y && span.x > span.z ? 0 : span.y > span.z ? 1 : 2;
        let low = vertices[0], high = vertices[0];
        for (const index of vertices) {
          if (part.positions[index * 3 + majorAxis] < part.positions[low * 3 + majorAxis]) low = index;
          if (part.positions[index * 3 + majorAxis] > part.positions[high * 3 + majorAxis]) high = index;
        }
        const chosen = [], endpointNodes = [];
        for (const extreme of [low, high]) {
          position.fromArray(part.positions, extreme * 3);
          let best = 0, distance = Infinity;
          for (let index = 0; index < nodes.length; index++) {
            const d = nearestDistance(position, nodes[index].supports);
            if (d < distance) { distance = d; best = index; }
          }
          endpointNodes.push(best);
          if (!chosen.includes(best)) chosen.push(best);
        }
        for (const score of scores) if (chosen.length < 4 && !chosen.includes(score.index)) chosen.push(score.index);
        for (const localIndex of vertices) {
          const vertex = vOffset + localIndex;
          position.fromArray(part.positions, localIndex * 3);
          let sum = 0;
          for (let j = 0; j < chosen.length; j++) {
            const index = chosen[j];
            const weight = 1 / (nearestDistance(position, nodes[index].supports) + softness) ** 2;
            skinIndices[vertex * 4 + j] = index; skinWeights[vertex * 4 + j] = weight; sum += weight;
            nodes[index].maxOffset = Math.max(nodes[index].maxOffset, position.distanceTo(nodes[index].center));
          }
          const extent = span.getComponent(majorAxis);
          const fraction = extent > 1e-7 ? clamp((position.getComponent(majorAxis) - componentBox.min.getComponent(majorAxis)) / extent, 0, 1) : 0.5;
          const end = fraction < 0.5 ? 0 : 1;
          const pin = clamp(1 - Math.min(fraction, 1 - fraction) / 0.12, 0, 1);
          const pinWeight = pin * pin * (3 - 2 * pin);
          // Tendon ends follow their own attachment, not a small residual share
          // of a distant finger/hand transform. Blend the pin smoothly into the
          // belly; no vertices or source geometry are removed.
          for (let j = 0; j < chosen.length; j++) {
            skinWeights[vertex * 4 + j] = (1 - pinWeight) * skinWeights[vertex * 4 + j] / sum + (chosen[j] === endpointNodes[end] ? pinWeight : 0);
          }
          // Longitudinal texture/colour coordinate uses projection on the muscle
          // envelope axis, with a deliberately soft visual tendon transition.
          const u = clamp(position.clone().sub(restPath[0]).dot(axis) / Math.max(1e-6, restPath[0].distanceTo(restPath[restPath.length - 1])), 0, 1);
          const belly = clamp(Math.min(u, 1 - u) / 0.18, 0, 1);
          const soft = belly * belly * (3 - 2 * belly);
          color.copy(tendon).lerp(colorOff, soft).toArray(colors, vertex * 3);
          color.copy(tendon).lerp(colorOn, soft).toArray(onColors, vertex * 3);
          uv[vertex * 2] = position.x * 10; uv[vertex * 2 + 1] = u;
        }
      }
      vOffset += count; iOffset += part.indices.length;
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('skinIndex', new THREE.BufferAttribute(skinIndices, 4));
    geometry.setAttribute('skinWeight', new THREE.BufferAttribute(skinWeights, 4));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1)); geometry.computeVertexNormals();
    const material = new THREE.MeshStandardMaterial({ vertexColors: true, map: opts.fiberTexture || null, roughness: 0.58, metalness: 0 });
    material.onBeforeCompile = shader => {
      const chunk = THREE.ShaderChunk.skinnormal_vertex.replace('objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;', 'mat3 muscleNormal = mat3(cross(skinMatrix[1].xyz, skinMatrix[2].xyz), cross(skinMatrix[2].xyz, skinMatrix[0].xyz), cross(skinMatrix[0].xyz, skinMatrix[1].xyz));\nobjectNormal = muscleNormal * objectNormal;');
      shader.vertexShader = shader.vertexShader.replace('#include <skinnormal_vertex>', chunk);
    };
    material.customProgramCacheKey = () => 'muscle-surface-normal-v2';
    const mesh = new THREE.SkinnedMesh(geometry, material);
    mesh.userData = { kind: 'muscle', unit: item.unit, side: item.side, anatomicalSurface: true };
    mesh.frustumCulled = false;
    group.add(mesh); group.updateMatrixWorld(true);
    const skeleton = new THREE.Skeleton(nodes.map(node => node.bone));
    mesh.bind(skeleton, new THREE.Matrix4());
    mesh.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 4);
    records.push({ key, unit: item.unit, side: item.side, mesh, skeleton, nodes, representative, restLength, colorOff: colors.slice(), colorOn: onColors, on: false });
    pickables.push(mesh); covered.add(key);
    if (records.length % 5 === 0) await yieldUI();
  }

  function update(W, paths) {
    for (const rec of records) {
      const path = paths && paths.get(rec.representative.id) || A.strandPath(W, rec.representative);
      // Small, bounded transverse correction. Skinning is an approximation; it
      // cannot enforce exact tissue volume or muscle/bone contact.
      const radius = clamp(Math.sqrt(rec.restLength / Math.max(1e-6, A.pathLength(path))), 0.92, 1.08);
      const box = new THREE.Box3();
      for (const node of rec.nodes) {
        const bone = node.bone;
        bone.position.copy(A.pt(W, node.seg, node.center));
        bone.quaternion.copy(W[node.seg].q).multiply(node.frame);
        bone.scale.set(radius, 1, radius); bone.updateMatrixWorld(true);
        const offset = node.maxOffset * Math.max(1, radius) + 0.001;
        box.expandByPoint(position.copy(bone.position).addScalar(offset));
        box.expandByPoint(position.copy(bone.position).addScalar(-offset));
      }
      box.getBoundingSphere(rec.mesh.boundingSphere); rec.bounds = box;
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
    for (const rec of records) if (rec.unit === unit && (!side || side === rec.side) && rec.bounds) box.union(rec.bounds);
    return box.isEmpty() ? null : box;
  }
  function dispose() {
    for (const rec of records) { rec.mesh.geometry.dispose(); rec.mesh.material.dispose(); rec.skeleton.dispose(); }
    group.removeFromParent();
  }
  update(opts.world || restW);
  return { group, covered, pickables, update, setVisibility, setAppearance, getBounds, dispose };
}
