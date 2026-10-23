// Exact, rigid BodyParts3D bone checks for opposite arms. This build-time guard
// does not infer soft-tissue contact, pressure, or complete body collision.
import { gunzipSync } from 'node:zlib';
import { segmentClosest } from './collision_geometry.mjs';

const ARM_SEGMENT = /^(hum|fore|ulna|hand|f[1-5][mabc])_[LR]$/;
const finitePoint = p => p && [p.x, p.y, p.z].every(Number.isFinite);
const sub = (a, b) => a.clone().sub(b);

function projectedInside(p, triangle, normal, epsilon) {
  const drop = Math.abs(normal.x) > Math.abs(normal.y)
    ? (Math.abs(normal.x) > Math.abs(normal.z) ? 'x' : 'z')
    : (Math.abs(normal.y) > Math.abs(normal.z) ? 'y' : 'z');
  const [u, v] = drop === 'x' ? ['y', 'z'] : drop === 'y' ? ['x', 'z'] : ['x', 'y'];
  let positive = false, negative = false;
  for (let k = 0; k < 3; k++) {
    const a = triangle[k], b = triangle[(k + 1) % 3];
    const cross = (b[u] - a[u]) * (p[v] - a[v]) - (b[v] - a[v]) * (p[u] - a[u]);
    const tolerance = epsilon * Math.hypot(b[u] - a[u], b[v] - a[v]);
    positive ||= cross > tolerance; negative ||= cross < -tolerance;
  }
  return !(positive && negative);
}

function segmentTriangle(p, q, triangle, normal, epsilon) {
  const a = triangle[0], start = sub(p, a).dot(normal), end = sub(q, a).dot(normal);
  if ((start > epsilon && end > epsilon) || (start < -epsilon && end < -epsilon)) return false;
  if (Math.abs(start) <= epsilon && projectedInside(p, triangle, normal, epsilon)) return true;
  if (Math.abs(end) <= epsilon && projectedInside(q, triangle, normal, epsilon)) return true;
  const denominator = start - end;
  if (Math.abs(denominator) > 1e-16) {
    const fraction = start / denominator;
    if (fraction >= 0 && fraction <= 1
      && projectedInside(p.clone().lerp(q, fraction), triangle, normal, epsilon)) return true;
  }
  if (Math.abs(start) <= epsilon && Math.abs(end) <= epsilon) {
    for (let k = 0; k < 3; k++)
      if (segmentClosest(p, q, triangle[k], triangle[(k + 1) % 3]).distance <= epsilon) return true;
  }
  return false;
}

// Inclusive surface contact: arrays of three THREE.Vector3-like values.
// Coplanar overlap, identical coordinates and degenerate point/line triangles
// are handled explicitly; equality of segment origins alone is insufficient.
export function triangleIntersect(a, b, epsilon = 1e-9) {
  if (!Array.isArray(a) || !Array.isArray(b) || a.length !== 3 || b.length !== 3
    || !a.concat(b).every(finitePoint) || !Number.isFinite(epsilon) || epsilon < 0)
    throw new Error('Invalid triangle collision input');
  const na = sub(a[1], a[0]).cross(sub(a[2], a[0]));
  const nb = sub(b[1], b[0]).cross(sub(b[2], b[0]));
  const lengthA = na.length(), lengthB = nb.length();
  const scaleA = Math.max(sub(a[1], a[0]).length(), sub(a[2], a[0]).length(), sub(a[2], a[1]).length());
  const scaleB = Math.max(sub(b[1], b[0]).length(), sub(b[2], b[0]).length(), sub(b[2], b[1]).length());
  const regularA = lengthA > (epsilon + 16 * Number.EPSILON * scaleA) * scaleA;
  const regularB = lengthB > (epsilon + 16 * Number.EPSILON * scaleB) * scaleB;
  if (regularA) na.divideScalar(lengthA);
  if (regularB) nb.divideScalar(lengthB);
  if (regularA) {
    const distances = b.map(p => sub(p, a[0]).dot(na));
    if (distances.every(d => d > epsilon) || distances.every(d => d < -epsilon)) return false;
  }
  if (regularB) {
    const distances = a.map(p => sub(p, b[0]).dot(nb));
    if (distances.every(d => d > epsilon) || distances.every(d => d < -epsilon)) return false;
  }
  if (regularB) for (let k = 0; k < 3; k++)
    if (segmentTriangle(a[k], a[(k + 1) % 3], b, nb, epsilon)) return true;
  if (regularA) for (let k = 0; k < 3; k++)
    if (segmentTriangle(b[k], b[(k + 1) % 3], a, na, epsilon)) return true;
  if (!regularA && !regularB) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    if (segmentClosest(a[i], a[(i + 1) % 3], b[j], b[(j + 1) % 3]).distance <= epsilon) return true;
  return false;
}

function boxesOverlap(a, b, epsilon) {
  return a.min.x <= b.max.x + epsilon && a.max.x + epsilon >= b.min.x
    && a.min.y <= b.max.y + epsilon && a.max.y + epsilon >= b.min.y
    && a.min.z <= b.max.z + epsilon && a.max.z + epsilon >= b.min.z;
}

function buildTree(triangles, THREE) {
  const box = new THREE.Box3();
  for (const triangle of triangles) box.union(triangle.box);
  if (triangles.length <= 8) return { box, triangles, count: triangles.length };
  const span = box.getSize(new THREE.Vector3());
  const axis = span.x > span.y ? (span.x > span.z ? 'x' : 'z') : (span.y > span.z ? 'y' : 'z');
  triangles.sort((a, b) => a.center[axis] - b.center[axis]);
  const half = triangles.length >> 1;
  return { box, count: triangles.length, left: buildTree(triangles.slice(0, half), THREE), right: buildTree(triangles.slice(half), THREE) };
}

export function createBodyCollision(A, THREE, boneData) {
  if (!boneData || !Array.isArray(boneData.parts) || !boneData.parts.length || typeof boneData.data !== 'string')
    throw new Error('Missing bone data for body collision checks');
  const buffer = gunzipSync(Buffer.from(boneData.data, 'base64'));
  const expected = boneData.parts.reduce((n, part) => n + 6 * (part.n + part.m), 0);
  if (buffer.length !== expected) throw new Error('Bone collision manifest/payload length mismatch');
  const bones = [];
  let offset = 0;
  for (const [partIndex, part] of boneData.parts.entries()) {
    if (!Number.isInteger(part.n) || !Number.isInteger(part.m) || part.n < 1 || part.m < 1
      || !Array.isArray(part.lo) || !Array.isArray(part.hi) || part.lo.length !== 3 || part.hi.length !== 3
      || !part.lo.concat(part.hi).every(Number.isFinite)) throw new Error('Invalid bone collision manifest');
    const vertices = [], acc = [0, 0, 0];
    const use = ARM_SEGMENT.test(part.seg);
    for (let v = 0; v < part.n; v++) {
      const xyz = [];
      for (let k = 0; k < 3; k++) {
        acc[k] = (acc[k] + buffer.readUInt16LE(offset)) & 65535; offset += 2;
        xyz.push(part.lo[k] + (part.hi[k] - part.lo[k]) * acc[k] / 65535);
      }
      if (use) vertices.push(new THREE.Vector3(...xyz));
    }
    const indices = [];
    let index = 0;
    for (let k = 0; k < part.m * 3; k++) {
      index = (index + buffer.readUInt16LE(offset)) & 65535; offset += 2;
      if (index >= part.n) throw new Error('Bone collision triangle index outside source part');
      if (use) indices.push(index);
    }
    if (!use) continue;
    if (!A.S[part.seg]) throw new Error('Unknown bone collision segment ' + part.seg);
    const triangles = [];
    for (let k = 0; k < indices.length; k += 3) {
      const ids = indices.slice(k, k + 3), points = ids.map(i => vertices[i]);
      const box = new THREE.Box3().setFromPoints(points);
      triangles.push({ id: k / 3, indices: ids, box, center: box.getCenter(new THREE.Vector3()) });
    }
    bones.push({ partIndex, seg: part.seg, name: part.name, side: part.seg.slice(-1), vertices, tree: buildTree(triangles, THREE) });
  }
  const left = bones.filter(b => b.side === 'L'), right = bones.filter(b => b.side === 'R');
  if (!left.length || !right.length) throw new Error('Both arms are required for body collision checks');

  function inspect(W, options = {}) {
    const epsilon = options.epsilon ?? 1e-9;
    const maxPairs = options.maxPairs ?? Infinity, triangleLimit = options.maxTriangleIntersections ?? 1;
    if (!Number.isFinite(epsilon) || epsilon < 0 || !(maxPairs > 0) || !(triangleLimit >= 1))
      throw new Error('Invalid bone collision options');
    const matrices = new Map(), states = new Map();
    const matrixOf = seg => {
      if (!matrices.has(seg)) {
        const transform = W[seg];
        if (!transform || !finitePoint(transform.p) || !transform.q
          || ![transform.q.x, transform.q.y, transform.q.z, transform.q.w].every(Number.isFinite)
          || !(transform.q.lengthSq() > 1e-12)) throw new Error('Invalid bone collision transform ' + seg);
        matrices.set(seg, new THREE.Matrix4().compose(transform.p, transform.q.clone().normalize(), new THREE.Vector3(1, 1, 1)));
      }
      return matrices.get(seg);
    };
    const stateOf = bone => {
      if (!states.has(bone)) {
        const matrix = matrixOf(bone.seg), boxCache = new Map(), vertexCache = new Map(), triangleCache = new Map();
        const state = {
          box: node => {
            if (!boxCache.has(node)) boxCache.set(node, node.box.clone().applyMatrix4(matrix));
            return boxCache.get(node);
          },
          vertex: index => {
            if (!vertexCache.has(index)) vertexCache.set(index, bone.vertices[index].clone().applyMatrix4(matrix));
            return vertexCache.get(index);
          },
          triangle: triangle => {
            if (!triangleCache.has(triangle)) triangleCache.set(triangle, triangle.indices.map(i => state.vertex(i)));
            return triangleCache.get(triangle);
          },
        };
        states.set(bone, state);
      }
      return states.get(bone);
    };
    let candidatePairs = 0, triangleTests = 0, containmentTests = 0;
    function surfacePairs(aNode, bNode, aState, bState, hits) {
      if (!boxesOverlap(aState.box(aNode), bState.box(bNode), epsilon)) return;
      if (aNode.triangles && bNode.triangles) {
        for (const a of aNode.triangles) for (const b of bNode.triangles) {
          if (!boxesOverlap(aState.box(a), bState.box(b), epsilon)) continue;
          triangleTests++;
          if (triangleIntersect(aState.triangle(a), bState.triangle(b), epsilon)) {
            hits.push([a.id, b.id]);
            if (hits.length >= triangleLimit) return;
          }
        }
      } else if (aNode.triangles || (!bNode.triangles && aNode.count < bNode.count)) {
        surfacePairs(aNode, bNode.left, aState, bState, hits);
        if (hits.length < triangleLimit) surfacePairs(aNode, bNode.right, aState, bState, hits);
      } else {
        surfacePairs(aNode.left, bNode, aState, bState, hits);
        if (hits.length < triangleLimit) surfacePairs(aNode.right, bNode, aState, bState, hits);
      }
    }
    function pointInside(point, bone, state) {
      // Independent non-axis rays avoid a single edge/vertex hit determining
      // parity. Identical boundary hits are clustered once, without face culling.
      let votes = 0;
      for (const vector of [[1, .371, .529], [.193, 1, .617], [.467, .283, 1]]) {
        const ray = new THREE.Ray(point, new THREE.Vector3(...vector).normalize()), distances = [];
        const visit = node => {
          if (!ray.intersectsBox(state.box(node))) return;
          if (!node.triangles) { visit(node.left); visit(node.right); return; }
          for (const triangle of node.triangles) {
            const at = ray.intersectTriangle(...state.triangle(triangle), false, new THREE.Vector3());
            if (at) { const t = point.distanceTo(at); if (t > epsilon) distances.push(t); }
          }
        };
        visit(bone.tree); distances.sort((a, b) => a - b);
        let count = 0, last = -Infinity;
        for (const distance of distances) if (distance - last > Math.max(1e-10, epsilon * 4)) { count++; last = distance; }
        votes += count % 2;
      }
      return votes >= 2;
    }
    const pairs = [];
    outer: for (const a of left) for (const b of right) {
      const sa = stateOf(a), sb = stateOf(b), ba = sa.box(a.tree), bb = sb.box(b.tree);
      if (!boxesOverlap(ba, bb, epsilon)) continue;
      candidatePairs++;
      const hits = []; surfacePairs(a.tree, b.tree, sa, sb, hits);
      let kind = hits.length ? 'surface' : null;
      if (!kind && (ba.containsBox(bb) || bb.containsBox(ba))) {
        containmentTests++;
        if (pointInside(sb.vertex(0), a, sa) || pointInside(sa.vertex(0), b, sb)) kind = 'containment';
      }
      if (kind) pairs.push({ a: a.seg, b: b.seg, aName: a.name, bName: b.name,
        aPart: a.partIndex, bPart: b.partIndex, kind, triangleIntersections: hits.length,
        triangleCountCapped: hits.length >= triangleLimit, firstTriangles: hits[0] || null });
      if (pairs.length >= maxPairs) break outer;
    }
    return { colliding: pairs.length > 0, scope: 'opposite-arm-bones', pairs,
      boneParts: bones.length, candidatePairs, triangleTests, containmentTests,
      pairsTruncated: pairs.length >= maxPairs, epsilonM: epsilon };
  }
  return { inspect, boneParts: bones.length, segments: [...new Set(bones.map(b => b.seg))], scope: 'opposite-arm-bones' };
}
