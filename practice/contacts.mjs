// Contact quadrature for the photographed FYSM poses. This module identifies
// geometric supports; it does not infer measured tissue stiffness or grip strength.
import { CONTACT_K, FRICTION } from '../physics_params.mjs';
import { createMuscleSurfaces } from '../muscle_surface_render.js';
import { gunzipSync } from 'node:zlib';

export const FLOOR_BAND_M = .004;
export const CONTACT_REFERENCE_AREA_M2 = .0004;
// Two registered surface proxies, each with the existing 4 mm geometric band.
// This is a model tolerance, not measured skin thickness or compliance.
export const INTERNAL_CONTACT_TOLERANCE_M = 2 * FLOOR_BAND_M;
export const FINGERTIP_SOFT_OFFSET_M = .002;
const HAND_REGIONS = new Set(['ладони', 'пальцы рук', 'кулаки', 'кисти']);
const FOOT_REGIONS = new Set(['пятки', 'подушечки стоп', 'пальцы стоп', 'тыл стоп', 'наружный край стопы']);
const FORE_REGIONS = new Set(['предплечья', 'локти']);
// Source-photo intent, not a licence to create a force at a missing contact.
// Alternatives within a group describe one anatomical supporting patch.
const intent = (regions, side = null) => ({ regions, side });
const hand = side => intent(['ладони', 'пальцы рук', 'кисти'], side);
const fingers = side => ({ ...intent(['пальцы рук'], side), sourceKinds: ['distal-phalanx-skin-proxy'] });
const foot = side => intent([...FOOT_REGIONS], side);
const toes = side => intent(['пальцы стоп', 'подушечки стоп'], side);
const fore = side => intent(['предплечья', 'локти'], side);
const knee = side => intent(['колени'], side);
const buttocks = intent(['ягодицы', 'таз']);
const back = intent(['спина', 'плечи']);
export const PRACTICE_SUPPORT_INTENT = {
  'struna-01': [toes('L'), toes('R')], 'struna-02': [foot('L'), foot('R')],
  'struna-03': [foot('L'), foot('R')], 'struna-04': [toes('L'), toes('R')],
  'struna-05': [hand('L'), hand('R')], 'struna-06': [foot('L'), foot('R')],
  'struna-07': [hand('L'), hand('R'), foot('R')], 'struna-08': [buttocks],
  'detali-01': [hand('L'), hand('R')],
  'detali-02': [buttocks, intent(['пятки', 'икры'], 'L'), intent(['пятки', 'икры'], 'R')],
  'detali-03': [buttocks], 'detali-04': [back, foot('R')],
  'detali-05': [intent(['спина', 'ягодицы', 'таз'])],
  'detali-06': [intent(['спина', 'ягодицы', 'таз']), knee('L'), knee('R')],
  'detali-07': [back],
  // The mermaid's lateral pelvic support can lie on proximal TFL/thigh tissue;
  // do not translate a real side-hip support merely to satisfy a region label.
  'detali-08': [intent(['таз', 'ягодицы', 'спина', 'бёдра'], 'R'), fingers('L'), fingers('R')],
  'centering-01': [foot('L'), foot('R')],
  'centering-02': [hand('L'), hand('R'), toes('L'), toes('R')],
  'centering-03': [foot('L'), toes('R')], 'centering-04': [foot('L'), foot('R')],
  'centering-05': [fore('L'), fore('R'), toes('R')],
  'centering-06': [hand('L'), fore('R')],
  'centering-07': [knee('L'), knee('R'), fore('L')],
  'centering-08': [knee('L'), knee('R'), intent(['грудь', 'нижние рёбра']), hand('L'), hand('R')],
};
export function matchesPracticeSupport(intent, contact) {
  return intent.regions.includes(contact.region) && (!intent.side || contact.side === intent.side)
    && (!intent.sourceKinds || intent.sourceKinds.includes(contact.source?.kind));
}
const finitePoint = p => p && [p.x, p.y, p.z].every(Number.isFinite);
const suffix = seg => /_([LR])$/.exec(seg)?.[1] || 'C';
const family = (region, side) => HAND_REGIONS.has(region) ? 'hand_' + side
  : FOOT_REGIONS.has(region) ? 'foot_' + side
    : FORE_REGIONS.has(region) ? 'fore_' + side : region + '_' + side;

export function convexHull(points) {
  const unique = new Map(points.map(p => [p[0] + '|' + p[1], p]));
  const P = [...unique.values()].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (P.length < 3) return P;
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], hi = [];
  for (const p of P) { while (lo.length > 1 && cross(lo.at(-2), lo.at(-1), p) <= 0) lo.pop(); lo.push(p); }
  for (const p of P.slice().reverse()) { while (hi.length > 1 && cross(hi.at(-2), hi.at(-1), p) <= 0) hi.pop(); hi.push(p); }
  lo.pop(); hi.pop(); return lo.concat(hi);
}
export function supportMargin(poly, point) {
  if (poly.length < 3) return null;
  let inside = true, minimum = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length], x = b[0] - a[0], z = b[1] - a[1];
    if (x * (point[1] - a[1]) - z * (point[0] - a[0]) < -1e-12) inside = false;
    const t = Math.max(0, Math.min(1, ((point[0] - a[0]) * x + (point[1] - a[1]) * z) / (x * x + z * z)));
    minimum = Math.min(minimum, Math.hypot(point[0] - a[0] - t * x, point[1] - a[1] - t * z));
  }
  return (inside ? 1 : -1) * minimum;
}
function clipHeight(poly, level, greater) {
  const result = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const ia = greater ? a.y >= level : a.y <= level, ib = greater ? b.y >= level : b.y <= level;
    if (ia) result.push(a);
    if (ia !== ib) result.push(a.clone().lerp(b, (level - a.y) / (b.y - a.y)));
  }
  return result;
}

// Exact projected area/first moment of the portion of a planar rendered
// triangle in the contact band. Subdividing that triangle preserves both.
export function clippedFloorTriangle(a, b, c, THREE, band = FLOOR_BAND_M) {
  if ([a, b, c].every(p => p.y > band) || [a, b, c].every(p => p.y < -band)) return null;
  let polygon = clipHeight([a, b, c], -band, true);
  if (polygon.length < 3) return null;
  polygon = clipHeight(polygon, band, false);
  if (polygon.length < 3) return null;
  return projectedPolygonPatch(polygon, THREE);
}
function projectedPolygonPatch(polygon, THREE) {
  const centroid = new THREE.Vector3(); let area = 0;
  for (let i = 1; i < polygon.length - 1; i++) {
    const x = polygon[0], y = polygon[i], z = polygon[i + 1];
    const projected = Math.abs((y.x - x.x) * (z.z - x.z) - (y.z - x.z) * (z.x - x.x)) / 2;
    if (projected <= 1e-14) continue;
    centroid.addScaledVector(x, projected / 3).addScaledVector(y, projected / 3).addScaledVector(z, projected / 3);
    area += projected;
  }
  return area > 1e-12 ? { areaM2: area, point: centroid.multiplyScalar(1 / area),
    footprint: polygon.map(p => [p.x, p.z]), polygon } : null;
}
function clipFootprintEdge(polygon, a, b, inside) {
  const distance = p => (b[0] - a[0]) * (p.z - a[1]) - (b[1] - a[1]) * (p.x - a[0]);
  const result = [];
  for (let i = 0; i < polygon.length; i++) {
    const p = polygon[i], q = polygon[(i + 1) % polygon.length], dp = distance(p), dq = distance(q);
    const ip = inside ? dp >= 0 : dp <= 0, iq = inside ? dq >= 0 : dq <= 0;
    if (ip) result.push(p);
    if (ip !== iq) result.push(p.clone().lerp(q, dp / (dp - dq)));
  }
  return result;
}

// Exact planar difference from a convex canonical skin footprint. Keeping
// disjoint outside pieces prevents both losing a distant heel/wrist support
// and counting an overlapping pressure area twice. Triangulation density does
// not change the integrated projected area or first moment of the difference.
export function outsideCanonicalFootprint(patch, footprint, THREE) {
  if (!footprint || footprint.length < 3) return [patch];
  let remaining = patch.polygon;
  const result = [];
  for (let i = 0; i < footprint.length && remaining.length >= 3; i++) {
    const a = footprint[i], b = footprint[(i + 1) % footprint.length];
    const outside = clipFootprintEdge(remaining, a, b, false);
    const piece = outside.length >= 3 ? projectedPolygonPatch(outside, THREE) : null;
    if (piece) result.push(piece);
    remaining = clipFootprintEdge(remaining, a, b, true);
  }
  return result;
}

export function practiceSurfaceRegion(unit, seg, point, W) {
  const side = suffix(seg), base = seg.replace(/_[LR]$/, '');
  if (/^f[1-5][abc]$/.test(base)) return 'пальцы рук';
  if (base === 'hand' || /^f[1-5]m$/.test(base)) return 'ладони';
  if (/^(foot|toes)$/.test(base)) return 'подушечки стоп';
  if (/^(fore|ulna)$/.test(base)) return 'предплечья';
  if (/^(hum|scap|clav)$/.test(base)) return 'плечи';
  if (/^(thigh|shank)$/.test(base)) {
    if (point.distanceTo(W['shank_' + side].p) < .12) return 'колени';
    if (base === 'shank') return /^(tibAnt|edl|ehl)$/.test(unit) ? 'голени' : 'икры';
    return /^(bicFem|semi)/.test(unit) ? 'задняя поверхность бёдер' : 'бёдра';
  }
  if (/^(glute|piriformis|obt|gemelli|quadFem)/.test(unit)) return 'ягодицы';
  if (/^(rectAbd|extObl|intObl|transAbd)$/.test(unit)) return 'живот';
  if (/^(pecMaj|pecMin)$/.test(unit)) return 'грудь';
  if (unit === 'serratus') return 'нижние рёбра';
  if (/^(head|neck)/.test(base)) return 'голова';
  return 'спина';
}

// A geometric hypothesis for absent gluteal soft tissue, restricted to the
// photographed boat pose. Curvature comes from the existing landmark spacing;
// compression is bounded by the existing landmark soft-tissue allowance. No
// dimension depends on COM, force, or success of a static solve.
export function glutealPadGeometry(A, THREE, index, W, areaScale = 1) {
  if (![1, .5].includes(areaScale)) throw new Error('Gluteal pad supports only the two declared area hypotheses');
  const l = A.LANDMARKS[index];
  if (!l || l.region !== 'ягодицы') throw new Error('Gluteal pad requires an existing gluteal skin landmark');
  const neighbours = A.LANDMARKS.map((other, i) => ({ other, i }))
    .filter(({ other, i }) => i !== index && other.region === l.region && other.seg === l.seg);
  const same = neighbours.filter(({ other }) => other.side === l.side)
    .sort((a, b) => a.other.pc.distanceTo(l.pc) - b.other.pc.distanceTo(l.pc))[0];
  const opposite = neighbours.filter(({ other }) => other.side !== l.side)
    .sort((a, b) => a.other.pc.distanceTo(l.pc) - b.other.pc.distanceTo(l.pc))[0];
  if (!same || !opposite || !Number.isFinite(l.soft) || l.soft <= 0)
    throw new Error('Gluteal pad lacks geometric spacing or an existing soft-tissue allowance');
  const spacingsM = [l.pc.distanceTo(same.other.pc), l.pc.distanceTo(opposite.other.pc)];
  const curvatureRadiusM = .5 * Math.min(...spacingsM), maximumCompressionM = l.soft;
  if (!(curvatureRadiusM > maximumCompressionM)) throw new Error('Invalid gluteal pad curvature/compression geometry');
  const radiusSquared = (2 * curvatureRadiusM * maximumCompressionM - maximumCompressionM ** 2) * areaScale;
  const contactRadiusM = Math.sqrt(radiusSquared), areaM2 = Math.PI * radiusSquared;
  const compressionM = curvatureRadiusM - Math.sqrt(curvatureRadiusM ** 2 - radiusSquared);
  const center = A.pt(W, l.seg, l.pc);
  const footprint = Array.from({ length: 32 }, (_, i) => {
    const angle = i * Math.PI / 16;
    return [center.x + contactRadiusM * Math.cos(angle), center.z + contactRadiusM * Math.sin(angle)];
  });
  return { id: 'gluteal-soft-pad:' + index, kind: 'gluteal-geometric-soft-pad',
    labelRu: 'Расчётная площадка мягких тканей', estimated: true, seg: l.seg, side: l.side,
    region: l.region, center, areaM2, areaScale, curvatureRadiusM, maximumCompressionM,
    compressionM, contactRadiusM, footprint,
    source: { kind: 'estimated-geometric-soft-pad', markerIndex: index,
      markerLocalPoint: l.pc.toArray(), neighbourIndices: [same.i, opposite.i], spacingsM,
      curvatureRule: 'half-minimum-existing-same-side-and-opposite-side-landmark-spacing',
      contactRule: 'circular-plane-section; radiusSquared=2*R*compression-compressionSquared',
      assumption: 'geometric soft-tissue patch; not measured skin, pressure, Hertz mechanics or patient calibration' } };
}

export function glutealPadQuadrature(pad, THREE) {
  const source = { ...pad.source, padId: pad.id, estimated: true, areaScale: pad.areaScale,
    markerWorldPoint: pad.center.toArray(), normal: [0, 1, 0],
    curvatureRadiusM: pad.curvatureRadiusM, compressionM: pad.compressionM,
    maximumCompressionM: pad.maximumCompressionM, contactRadiusM: pad.contactRadiusM };
  const point = new THREE.Vector3(pad.center.x, 0, pad.center.z);
  const points = [{ point, areaM2: pad.areaM2 / 3 }];
  for (let i = 0; i < 8; i++) {
    const angle = i * Math.PI / 4, radius = Math.sqrt(3) / 2 * pad.contactRadiusM;
    points.push({ point: point.clone().add(new THREE.Vector3(Math.cos(angle) * radius, 0, Math.sin(angle) * radius)),
      areaM2: pad.areaM2 / 12 });
  }
  return points.map((p, index) => ({ ...p, seg: pad.seg, side: pad.side, region: pad.region,
    footprint: pad.footprint, source: { ...source, quadratureIndex: index } }));
}

// Decode the actual distal bones used by the renderer. Bone payload coordinates
// are already relative to the segment joint, unlike anatomy reference points.
function decodeFingertipBones(data, A) {
  if (!data) return [];
  const raw = gunzipSync(Buffer.from(data.data, 'base64'));
  const expected = data.parts.reduce((sum, part) => sum + 6 * (part.n + part.m), 0);
  if (raw.length !== expected) throw new Error('Invalid distal-bone payload length');
  let cursor = 0;
  const result = [];
  data.parts.forEach((part, partIndex) => {
    if (!/^(f1b|f[2-5]c)_[LR]$/.test(part.seg)) { cursor += 6 * (part.n + part.m); return; }
    if (!Object.hasOwn(A.S, part.seg)) throw new Error('Unknown distal-bone segment');
    const positions = new Float64Array(part.n * 3), acc = [0, 0, 0];
    for (let i = 0; i < part.n; i++) for (let axis = 0; axis < 3; axis++) {
      acc[axis] = (acc[axis] + raw.readUInt16LE(cursor)) & 65535; cursor += 2;
      positions[i * 3 + axis] = part.lo[axis] + (part.hi[axis] - part.lo[axis]) * acc[axis] / 65535;
    }
    cursor += part.m * 6;
    result.push({ ...part, partIndex, positions });
  });
  return result;
}

// Aggregate actual quadrature points on one mechanical segment. Their weighted
// centroid represents the same normal resultant/first moment; no airborne
// support is inserted. Keep original landmark boundary points individually.
function clusterSamples(samples, budget, THREE) {
  if (!samples.length) return [];
  const groups = new Map();
  for (const sample of samples) {
    const key = sample.seg + '|' + sample.region + '|' + sample.side;
    if (!groups.has(key)) groups.set(key, []); groups.get(key).push(sample);
  }
  if (groups.size > budget) throw new Error('Contact quadrature has more mechanical regions than the contact budget');
  const entries = [...groups].map(([key, points]) => ({ key, points, count: 1,
    area: points.reduce((sum, p) => sum + p.areaM2, 0) }));
  for (let remaining = budget - entries.length; remaining > 0;) {
    const candidate = entries.filter(e => e.count < e.points.length)
      .sort((a, b) => b.area / b.count - a.area / a.count || a.key.localeCompare(b.key))[0];
    if (!candidate) break;
    candidate.count++; remaining--;
  }
  const output = [];
  for (const entry of entries) {
    const points = entry.points;
    const centres = [points.reduce((a, p) => p.areaM2 > a.areaM2 ? p : a).point.clone()];
    while (centres.length < entry.count) {
      let best = null, score = -1;
      for (const p of points) {
        const distance = Math.min(...centres.map(c => c.distanceToSquared(p.point)));
        if (distance > score) { score = distance; best = p.point; }
      }
      centres.push(best.clone());
    }
    let clusters;
    for (let iteration = 0; iteration < 12; iteration++) {
      clusters = centres.map(() => ({ areaM2: 0, weighted: new THREE.Vector3(), members: [] }));
      for (const p of points) {
        let nearest = 0, distance = Infinity;
        centres.forEach((c, i) => { const d = c.distanceToSquared(p.point); if (d < distance) { distance = d; nearest = i; } });
        const cluster = clusters[nearest]; cluster.areaM2 += p.areaM2;
        cluster.weighted.addScaledVector(p.point, p.areaM2); cluster.members.push(p);
      }
      clusters.forEach((cluster, i) => { if (cluster.areaM2 > 0) centres[i].copy(cluster.weighted).multiplyScalar(1 / cluster.areaM2); });
    }
    clusters.forEach((cluster, index) => {
      if (!cluster.areaM2) return;
      const point = cluster.weighted.multiplyScalar(1 / cluster.areaM2), first = cluster.members[0];
      output.push({ ...first, point, areaM2: cluster.areaM2,
        source: { kind: 'muscle-surface-quadrature', skinning: 'mesh.getVertexPosition',
          sampleCount: cluster.members.length, heightMinM: Math.min(...cluster.members.map(p => p.point.y)),
          heightMaxM: Math.max(...cluster.members.map(p => p.point.y)),
          triangleMinY: Math.min(...cluster.members.map(p => p.source.triangleMinY)),
          triangleMaxY: Math.max(...cluster.members.map(p => p.source.triangleMaxY)),
          meshMinY: Math.min(...cluster.members.map(p => p.source.meshMinY)),
          units: [...new Set(cluster.members.map(p => p.source.unit))],
          ownership: 'dominant-anatomical-skin-influence', patch: entry.key + ':' + index } });
    });
  }
  return output;
}

export async function createPracticeContacts(A, THREE, muscleData, options = {}) {
  const surfaces = await createMuscleSurfaces(A, THREE, muscleData, { instances: A.strandInstances() });
  if (!surfaces) throw new Error('Practice contacts require the displayed muscle surface model');
  const records = surfaces.pickables.map(mesh => {
    const index = mesh.geometry.getIndex(), weights = mesh.geometry.getAttribute('skinWeight');
    const skin = mesh.geometry.getAttribute('skinIndex'), count = weights.count;
    const segments = new Array(count), bySegment = new Map();
    for (let i = 0; i < count; i++) {
      let slot = 0; for (let k = 1; k < 4; k++) if (weights.getComponent(i, k) > weights.getComponent(i, slot)) slot = k;
      const seg = mesh.skeleton.bones[skin.getComponent(i, slot)].name;
      segments[i] = seg; if (!bySegment.has(seg)) bySegment.set(seg, []); bySegment.get(seg).push(i);
    }
    return { mesh, count, indices: index.array, segments, bySegment, skin, weights,
      positions: new Float64Array(count * 3), unit: mesh.userData.unit, side: mesh.userData.side };
  });
  const vector = (array, i) => new THREE.Vector3().fromArray(array, i * 3);
  const distalBones = decodeFingertipBones(options.boneData, A);
  function glutealPads(W, areaScale = options.padAreaScale ?? 1) {
    return ['L', 'R'].map(side => A.LANDMARKS.map((l, i) => ({ l, i }))
      .filter(({ l }) => l.region === 'ягодицы' && l.side === side)
      .sort((a, b) => A.pt(W, a.l.seg, a.l.pc).y - A.pt(W, b.l.seg, b.l.pc).y || a.i - b.i)[0])
      .map(({ i }) => glutealPadGeometry(A, THREE, i, W, areaScale));
  }
  function groundProxies(W, context = {}) {
    if (context.id === 'struna-08') return glutealPads(W, context.areaScale).map(pad => ({
      id: pad.id, seg: pad.seg, side: pad.side, region: pad.region, point: pad.center.clone(), source: pad.source }));
    if (context.id !== 'detali-08') return [];
    return distalBones.map(bone => {
      let minimum = Infinity, vertex = -1, point = null;
      for (let i = 0; i < bone.n; i++) {
        const P = vector(bone.positions, i).applyQuaternion(W[bone.seg].q).add(W[bone.seg].p);
        if (P.y < minimum) { minimum = P.y; vertex = i; point = P; }
      }
      const width = (bone.hi[0] - bone.lo[0]) / 2 + FINGERTIP_SOFT_OFFSET_M;
      const depth = (bone.hi[2] - bone.lo[2]) / 2 + FINGERTIP_SOFT_OFFSET_M;
      return { id: 'distal-phalanx-skin:' + bone.partIndex, seg: bone.seg, side: suffix(bone.seg),
        region: 'пальцы рук', point: point.clone().add(new THREE.Vector3(0, -FINGERTIP_SOFT_OFFSET_M, 0)),
        areaM2: Math.PI * width * depth,
        source: { kind: 'distal-phalanx-skin-proxy', estimated: true, bonePartIndex: bone.partIndex,
          boneId: bone.id || bone.name || bone.seg, boneSegment: bone.seg, boneVertex: vertex,
          boneLocalPoint: vector(bone.positions, vertex).toArray(), boneWorldPoint: point.toArray(),
          normal: [0, -1, 0], offsetM: FINGERTIP_SOFT_OFFSET_M,
          referenceAreaM2: Math.PI * width * depth,
          referenceAreaRule: 'distal-bone-transverse-ellipse-expanded-by-skin-offset',
          assumption: 'estimated absent fingertip tissue; constant small skin offset; no measured pressure or finger mechanics' } };
    });
  }
  function geometry(W) {
    surfaces.update(W); surfaces.group.updateMatrixWorld(true);
    const target = new THREE.Vector3();
    for (const record of records) for (let i = 0; i < record.count; i++) {
      record.mesh.getVertexPosition(i, target); record.mesh.localToWorld(target); target.toArray(record.positions, i * 3);
    }
    return records;
  }
  function landmarks(W) {
    return A.LANDMARKS.map((l, i) => ({ seg: l.seg, region: l.region, side: l.side,
      point: A.pt(W, l.seg, l.pc), areaM2: CONTACT_REFERENCE_AREA_M2,
      source: { kind: 'anatomy-skin-landmark', index: i, tag: l.tag } }));
  }
  function floorContacts(W, records, lmks, context = {}) {
    const band = options.floorBandM ?? FLOOR_BAND_M, maxContacts = options.maxContacts ?? 30;
    const inferredPads = [], proxies = groundProxies(W, context);
    let sourceLandmarks = lmks;
    if (context.id === 'detali-08') {
      // This photograph supports arched hands on the fingers. The old generic
      // fingertip markers are not registered to the distal atlas bones.
      sourceLandmarks = lmks.filter(p => !HAND_REGIONS.has(p.region));
      for (const proxy of proxies) {
        if (proxy.source.boneWorldPoint[1] < -1e-6) continue;
        sourceLandmarks.push(proxy);
        if (Math.abs(proxy.point.y) <= band) {
          const radius = Math.sqrt(proxy.areaM2 / Math.PI);
          const footprint = Array.from({ length: 16 }, (_, i) => [proxy.point.x + radius * Math.cos(i * Math.PI / 8),
            proxy.point.z + radius * Math.sin(i * Math.PI / 8)]);
          proxy.footprint = footprint;
          inferredPads.push({ id: proxy.id, kind: 'distal-phalanx-skin-proxy', estimated: true,
            labelRu: 'Расчётная подушечка пальца', side: proxy.side, areaM2: proxy.areaM2,
            offsetM: FINGERTIP_SOFT_OFFSET_M, source: proxy.source, footprint });
        }
      }
    }
    const nearLandmarks = sourceLandmarks.filter(p => p.point.y >= -band && p.point.y <= band);
    const canonicalFootprints = new Map(), canonicalPressureCells = [], canonicalFamilies = new Map();
    const addFootprint = (key, footprint) => {
      if (!canonicalFootprints.has(key)) canonicalFootprints.set(key, []);
      canonicalFootprints.get(key).push(footprint);
    };
    for (const p of nearLandmarks) {
      const key = family(p.region, p.side);
      if (p.source.kind !== 'anatomy-skin-landmark') { if (p.footprint) addFootprint(key, p.footprint); continue; }
      if (context.id === 'struna-08' && p.region === 'ягодицы') continue;
      if (!canonicalFamilies.has(key)) canonicalFamilies.set(key, []);
      canonicalFamilies.get(key).push(p);
    }
    for (const [key, points] of canonicalFamilies) for (const p of points) {
      // The canonical skin points did not measure a finite pressure area. Keep
      // the existing numerical reference area explicit and local, shared by
      // the points in one family. A broad hull between heel/toe or separate
      // fingertips is not a filled pressure footprint.
      p.areaM2 = CONTACT_REFERENCE_AREA_M2 / points.length;
      const sides = 16, radiusM = Math.sqrt(2 * p.areaM2 / (sides * Math.sin(2 * Math.PI / sides)));
      const footprint = Array.from({ length: sides }, (_, i) => [p.point.x + radiusM * Math.cos(i * Math.PI / 8),
        p.point.z + radiusM * Math.sin(i * Math.PI / 8)]);
      const cell = { kind: 'estimated-canonical-pressure-cell', estimated: true, family: key,
        markerIndex: p.source.index, center: p.point.toArray(), areaM2: p.areaM2,
        familyReferenceAreaM2: CONTACT_REFERENCE_AREA_M2, familyPointCount: points.length,
        sides, radiusM, footprint,
        assumption: 'local numerical reference coverage; not measured skin or pressure area; does not enlarge the anatomical support hull' };
      p.source.pressureCell = cell;
      canonicalPressureCells.push(cell); addFootprint(key, footprint);
    }
    if (context.id === 'struna-08') for (const pad of glutealPads(W, context.areaScale)) {
      if (Math.abs(pad.center.y) <= band) addFootprint(family(pad.region, pad.side), pad.footprint);
    }
    const surfaceSamples = [], penetration = [];
    let originalSurfaceAreaM2 = 0, deduplicatedSurfaceAreaM2 = 0;
    let surfaceMinimum = null, belowFloorVertices = 0;
    for (const rec of records) {
      if (A.UNITS[rec.unit].deep) continue;
      let minY = Infinity, below = 0;
      for (let i = 0; i < rec.count; i++) {
        const y = rec.positions[3 * i + 1]; minY = Math.min(minY, y);
        if (y < 0) belowFloorVertices++;
        if (y < -band) below++;
        if (!surfaceMinimum || y < surfaceMinimum.heightM) surfaceMinimum = {
          unit: rec.unit, side: rec.side, vertex: i, heightM: y, point: vector(rec.positions, i).toArray() };
      }
      if (below) {
        penetration.push({ unit: rec.unit, side: rec.side, minY, vertices: below,
          excludedFromFloorQuadrature: true });
        // A stripe where a deeply penetrating muscle crosses y=0 is not a
        // physical support. Keep the full penetration diagnostic and exclude
        // this entire rendered unit/side from surface contact identification.
        continue;
      }
      for (let i = 0; i < rec.indices.length; i += 3) {
        const ids = [rec.indices[i], rec.indices[i + 1], rec.indices[i + 2]];
        const a = vector(rec.positions, ids[0]), b = vector(rec.positions, ids[1]), c = vector(rec.positions, ids[2]);
        if (b.clone().sub(a).cross(c.clone().sub(a)).y >= -1e-14) continue;
        const patch = clippedFloorTriangle(a, b, c, THREE, band); if (!patch) continue;
        const ownershipOf = point => {
          const barycentric = new THREE.Triangle(a, b, c).getBarycoord(point, new THREE.Vector3());
          const ownership = new Map();
          ids.forEach((vertex, corner) => {
            for (let slot = 0; slot < 4; slot++) {
              const seg = rec.mesh.skeleton.bones[rec.skin.getComponent(vertex, slot)].name;
              ownership.set(seg, (ownership.get(seg) || 0) + rec.weights.getComponent(vertex, slot) * barycentric.getComponent(corner));
            }
          });
          const seg = [...ownership].sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0]))[0][0];
          const side = suffix(seg) === 'C' ? (rec.side || 'C') : suffix(seg);
          return { seg, side, region: practiceSurfaceRegion(rec.unit, seg, point, W) };
        };
        const initial = ownershipOf(patch.point), key = family(initial.region, initial.side);
        const footprints = [...(canonicalFootprints.get(key) || [])];
        if (!HAND_REGIONS.has(initial.region) && !FOOT_REGIONS.has(initial.region)
            && !FORE_REGIONS.has(initial.region) && initial.side !== 'C')
          footprints.push(...(canonicalFootprints.get(family(initial.region, 'C')) || []));
        let pieces = [patch];
        // Sequential subtraction is the union of the local canonical cells:
        // overlapping cells never subtract the same physical area twice.
        for (const footprint of footprints)
          pieces = pieces.flatMap(piece => outsideCanonicalFootprint(piece, footprint, THREE));
        originalSurfaceAreaM2 += patch.areaM2;
        deduplicatedSurfaceAreaM2 += patch.areaM2 - pieces.reduce((sum, piece) => sum + piece.areaM2, 0);
        for (const piece of pieces) surfaceSamples.push({ ...piece, ...ownershipOf(piece.point),
          source: { kind: 'muscle-surface', unit: rec.unit,
            triangle: i / 3, triangleMinY: Math.min(a.y, b.y, c.y), triangleMaxY: Math.max(a.y, b.y, c.y), meshMinY: minY,
            canonicalFootprintDeduplicated: footprints.length > 0 } });
      }
    }
    let canonical = nearLandmarks.slice();
    if (context.id === 'struna-08') {
      canonical = canonical.filter(p => p.region !== 'ягодицы');
      for (const pad of glutealPads(W, context.areaScale)) if (Math.abs(pad.center.y) <= band) {
        canonical.push(...glutealPadQuadrature(pad, THREE));
        inferredPads.push({ ...pad, center: pad.center.toArray() });
      }
    }
    // The budget is a numerical target, not permission to discard a real
    // support or combine points carried by different mechanical segments.
    const mechanicalGroups = new Set(surfaceSamples.map(p => p.seg + '|' + p.region + '|' + p.side)).size;
    const actualBudget = Math.max(maxContacts, canonical.length + mechanicalGroups);
    const clusters = clusterSamples(surfaceSamples, actualBudget - canonical.length, THREE);
    const all = canonical.concat(clusters);
    // Keep the actual clipped area boundary as a separate diagnostic. A
    // finite set of pressure-cell centroids has a smaller support polygon;
    // pose repair must be able to distinguish that numerical approximation
    // from an anatomical support which is genuinely missing.
    const footprint = convexHull(canonical.flatMap(p => p.footprint || [[p.point.x, p.point.z]])
      .concat(surfaceSamples.flatMap(p => p.footprint)));
    const com = A.comOf(W);
    const contacts = all.map((p, i) => ({ id: 'practice-floor:' + i + ':' + p.seg + ':' + p.region,
      li: 'practice-' + i, seg: p.seg, region: p.region, side: p.side,
      P: new THREE.Vector3(p.point.x, 0, p.point.z), h: Math.max(0, p.point.y),
      k: (CONTACT_K[p.region] || 1.5) * p.areaM2 / CONTACT_REFERENCE_AREA_M2,
      mu: FRICTION[p.region] ?? .5, areaM2: p.areaM2,
      source: { ...p.source, rawPoint: p.point.toArray() } }));
    return { contacts, diagnostics: { method: 'surface-area-contact-quadrature', contactBandM: band,
      maxContacts, actualBudget, contactCount: contacts.length,
      landmarkContacts: canonical.filter(p => p.source.kind === 'anatomy-skin-landmark').length,
      inferredContactCount: canonical.filter(p => p.source.estimated).length,
      surfaceTriangles: surfaceSamples.length, surfaceContacts: clusters.length,
      originalSurfaceAreaM2, deduplicatedSurfaceAreaM2, canonicalPressureCells,
      referenceAreaM2: CONTACT_REFERENCE_AREA_M2, surfaceGeometryValid: penetration.length === 0,
      inferredPads, groundProxySource: context.id === 'detali-08' ? 'actual-distal-phalanx-with-estimated-2mm-tissue' : null,
      missingProxyGeometry: context.id === 'detali-08' && distalBones.length === 0,
      minFingertipBoneHeightM: context.id === 'detali-08'
        ? Math.min(...proxies.map(p => p.source.boneWorldPoint[1]), Infinity) === Infinity ? null
          : Math.min(...proxies.map(p => p.source.boneWorldPoint[1])) : null,
      unclusteredSupport: { poly: footprint, margin: supportMargin(footprint, [com.x, com.z]) },
      surfaceMinimum, belowFloorVertices,
      maxSurfacePenetrationM: Math.max(0, -(surfaceMinimum?.heightM ?? 0)),
      penetratingSurfaceRegions: penetration } };
  }
  function cloud(records, lmks, predicate, anchor, radius) {
    const points = [], radiusSq = radius * radius;
    for (const rec of records) for (const [seg, indices] of rec.bySegment) {
      // Internal contacts are identified on exposed tissue proxies, not on
      // a deep muscle/tendon visible only after switching anatomical layers.
      if (A.UNITS[rec.unit].deep) continue;
      if (!predicate(seg)) continue;
      for (const i of indices) {
        const P = vector(rec.positions, i); if (anchor && P.distanceToSquared(anchor) > radiusSq) continue;
        points.push({ seg, P, source: { kind: 'muscle-surface-vertex', unit: rec.unit, side: rec.side, vertex: i } });
      }
    }
    for (const l of lmks) if (predicate(l.seg) && (!anchor || l.point.distanceToSquared(anchor) <= radiusSq))
      points.push({ seg: l.seg, P: l.point.clone(), source: l.source });
    return points;
  }
  function closest(a, b) {
    let best = null, square = Infinity;
    for (const p of a) for (const q of b) { const distance = p.P.distanceToSquared(q.P); if (distance < square) { square = distance; best = { a: p, b: q }; } }
    return best ? { ...best, gapM: Math.sqrt(square) } : null;
  }
  function pairContact(W, records, lmks, descriptor) {
    const candidatesA = cloud(records, lmks, descriptor.a, descriptor.anchorA, descriptor.radiusA ?? .16);
    const candidatesB = cloud(records, lmks, descriptor.b, descriptor.anchorB, descriptor.radiusB ?? .16);
    const found = closest(candidatesA, candidatesB);
    if (!found || found.gapM > (descriptor.toleranceM ?? INTERNAL_CONTACT_TOLERANCE_M))
      throw new Error(`Practice contact ${descriptor.id} has no demonstrated near-surface pair; gap ${found?.gapM ?? 'missing'} m`);
    const P = found.a.P.clone().add(found.b.P).multiplyScalar(.5);
    let normal = descriptor.normal ? descriptor.normal(P) : null;
    if (normal && normal.lengthSq() < 1e-12) throw new Error('Degenerate internal contact normal: ' + descriptor.id);
    if (normal) normal.normalize();
    return { id: descriptor.id, segA: found.a.seg, segB: found.b.seg, P,
      kind: descriptor.kind, side: descriptor.side || 'C', normal, mu: descriptor.mu ?? 0,
      gapM: found.gapM, source: { kind: 'near-surface-contact', a: found.a.source, b: found.b.source,
        pointA: found.a.P.toArray(), pointB: found.b.P.toArray(), toleranceM: descriptor.toleranceM ?? INTERNAL_CONTACT_TOLERANCE_M,
        assumption: descriptor.kind === 'bilateral' ? 'closed grip; finger strength not identified'
          : descriptor.kind === 'pull' ? 'pull cone along the arm; cone width retained from the legacy grip model'
            : 'unilateral normal contact; skin friction not measured and omitted' } };
  }
  function internals(id, W, records, lmks) {
    const result = [], hands = side => seg => new RegExp('^(hand|f[1-5][mabc])_' + side + '$').test(seg);
    const feet = side => seg => seg === 'foot_' + side || seg === 'toes_' + side;
    const lock = (a, b, anchorA, anchorB, key = 'hands') => result.push(pairContact(W, records, lmks,
      { id: id + ':' + key, kind: 'bilateral', a, b, anchorA, anchorB, radiusA: .20, radiusB: .20 }));
    if (['struna-01', 'struna-02'].includes(id))
      lock(hands('L'), hands('R'), W.hand_L.p, W.hand_R.p);
    if (['struna-03', 'centering-01'].includes(id)) {
      // The source pose closes the left fingers around the distal right
      // forearm. The right carpals are not the opposing mechanical segment.
      const distalForearm = W.hand_R.p.clone().addScaledVector(
        W.hand_R.p.clone().sub(W.fore_R.p).normalize(), -.03);
      result.push(pairContact(W, records, lmks, { id: id + ':wrist-lock', kind: 'bilateral',
        a: hands('L'), b: seg => seg === 'fore_R', anchorA: distalForearm, anchorB: distalForearm,
        radiusA: .06, radiusB: .06 }));
    }
    if (['detali-06', 'detali-07'].includes(id)) for (const side of ['L', 'R']) {
      result.push(pairContact(W, records, lmks, { id: id + ':foot-grip:' + side, side, kind: 'pull',
        a: hands(side), b: feet(side), anchorA: W.hand_L.p.clone().copy(W['hand_' + side].p),
        anchorB: W['foot_' + side].p, radiusA: .18, radiusB: .22, mu: 1,
        normal: P => P.clone().sub(W['hum_' + side].p) }));
    }
    if (['struna-05', 'struna-07', 'centering-06'].includes(id)) {
      const sides = id === 'struna-05' ? ['L', 'R'] : ['L'];
      for (const side of sides) {
        const knee = W['shank_' + side].p;
        result.push(pairContact(W, records, lmks, { id: id + ':knee-arm:' + side, side, kind: 'unilateral',
          a: seg => seg === 'thigh_' + side || seg === 'shank_' + side,
          b: seg => seg === 'hum_' + side, anchorA: knee, anchorB: knee,
          radiusA: .14, radiusB: .24, normal: P => {
            const shoulder = W['hum_' + side].p, elbow = W['fore_' + side].p;
            const d = elbow.clone().sub(shoulder), t = Math.max(0, Math.min(1, P.clone().sub(shoulder).dot(d) / d.lengthSq()));
            return knee.clone().sub(shoulder.clone().addScaledVector(d, t));
          } }));
      }
    }
    if (id === 'struna-06') for (const side of ['L', 'R']) {
      const knee = W['shank_' + side].p;
      result.push(pairContact(W, records, lmks, { id: id + ':palm-knee:' + side, side, kind: 'unilateral',
        a: hands(side), b: seg => seg === 'thigh_' + side || seg === 'shank_' + side,
        anchorA: W['hand_' + side].p, anchorB: knee, radiusA: .14, radiusB: .18,
        normal: P => W['hand_' + side].p.clone().sub(knee) }));
    }
    return result;
  }
  function buildFloor(W, context = {}) {
    for (const transform of Object.values(W)) if (!finitePoint(transform.p)) throw new Error('Invalid practice contact world transform');
    const mesh = geometry(W), lmks = landmarks(W), floor = floorContacts(W, mesh, lmks, context);
    const com = A.comOf(W);
    const poly = convexHull(floor.contacts.map(c => [c.P.x, c.P.z]));
    return { contacts: floor.contacts,
      diagnostics: { ...floor.diagnostics, support: { poly, margin: supportMargin(poly, [com.x, com.z]) },
      }, _mesh: mesh, _landmarks: lmks };
  }
  function build(id, W, buildOptions = {}) {
    const floor = buildFloor(W, { id, areaScale: buildOptions.areaScale });
    let internalContacts = [], error = null;
    try { internalContacts = internals(id, W, floor._mesh, floor._landmarks); }
    catch (failure) { if (!buildOptions.audit) throw failure; error = failure.message; }
    const missingPrimarySupports = (PRACTICE_SUPPORT_INTENT[id] || [])
      .filter(intent => !floor.contacts.some(contact => matchesPracticeSupport(intent, contact)));
    return { contacts: floor.contacts, internalContacts,
      diagnostics: { ...floor.diagnostics, contactModelComplete: error === null,
        missingInternalContact: error, missingPrimarySupports,
        internalContacts: internalContacts.map(c => ({ id: c.id, kind: c.kind, segA: c.segA, segB: c.segB, gapM: c.gapM, source: c.source })) } };
  }
  return { build, buildFloor: (W, context = {}) => {
    const floor = buildFloor(W, context); delete floor._mesh; delete floor._landmarks; return floor;
  }, groundProxies, dispose: () => surfaces.dispose(), surfaces };
}
