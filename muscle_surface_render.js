/**
 * BodyParts3D muscle surfaces, fitted to the existing rig at build time.
 * The surfaces use GPU linear blend skinning from the anatomical segments named
 * by each muscle's attachment/via points. A vertex has at most four influences,
 * but a wide muscle can retain all of its anatomical segment supports. Compact
 * weights vanish where an influence leaves that four-slot palette.
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
  const colorOff = new THREE.Color(0xa94d49), colorOn = new THREE.Color(0x2c9a5a), tendon = new THREE.Color(0xdfcbb7);
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
          for (const endpoint of endpointPins) {
            // Pin only while this anatomical endpoint is the closest support;
            // fade to zero before that nearest-support identity changes. This
            // also keeps an endpoint from bypassing the compact palette cutoff.
            if (candidates[0].index !== endpoint.index) continue;
            const outside = endpoint.direction * (position.dot(axis) - endpoint.limit);
            const u = clamp(outside / endpoint.reach, 0, 1);
            const dominance = clamp((1 - ratio) / 0.3, 0, 1);
            const strength = u * u * (3 - 2 * u) * dominance * dominance * (3 - 2 * dominance);
            for (let j = 0; j < influenceCount; j++) skinWeights[vertex * 4 + j] =
              (1 - strength) * skinWeights[vertex * 4 + j] + (j === 0 ? strength : 0);
          }
          // A short attachment tint is an appearance approximation, not a
          // segmented tendon map. Projecting an entire fan on one strand and
          // clamping its ends used to bleach large angular plates of abdomen
          // and chest. Distance to actual attachment footprints stays local.
          const belly = clamp(Math.sqrt(nearestDistance(position, attachmentSupports)) / attachmentTintReach, 0, 1);
          const soft = belly * belly * (3 - 2 * belly);
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
    material.onBeforeCompile = shader => {
      const chunk = THREE.ShaderChunk.skinnormal_vertex.replace('objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;', 'mat3 muscleNormal = mat3(cross(skinMatrix[1].xyz, skinMatrix[2].xyz), cross(skinMatrix[2].xyz, skinMatrix[0].xyz), cross(skinMatrix[0].xyz, skinMatrix[1].xyz));\nobjectNormal = muscleNormal * objectNormal;');
      shader.vertexShader = shader.vertexShader.replace('#include <skinnormal_vertex>', chunk);
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
    material.customProgramCacheKey = () => intersectionMarks ? 'muscle-surface-normal-v3-rectus' : 'muscle-surface-normal-v3';
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
