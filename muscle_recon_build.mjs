// Reconstruct connected rest surfaces only where the reference atlas has no
// mapped element. These are model-derived shapes, never BodyParts3D meshes.
// Usage: node muscle_recon_build.mjs [OUTFILE]
// Default output stays outside the repository for the muscle atlas packer.
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';

const A = createAnatomy(THREE), V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);
const root = A.S[A.SEG.find(s => !s.parent).name].J;
const W = A.computeWorld(A.STAND.local, A.STAND.rootQ, root);
const targetUnits = ['lat', 'multifidus', 'ql', 'rectAbd', 'intObl', 'transAbd'];
const all = A.strandInstances().filter(s => s.kind === 'muscle');
const parts = [], reports = [];
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));

function pathSampler(points) {
  const lengths = [0];
  for (let i = 1; i < points.length; i++) lengths.push(lengths[i - 1] + points[i].distanceTo(points[i - 1]));
  const L = lengths.at(-1);
  return u => {
    const d = clamp(u, 0, 1) * L;
    let i = 1; while (i < lengths.length - 1 && lengths[i] < d) i++;
    const t = (d - lengths[i - 1]) / (lengths[i] - lengths[i - 1] || 1);
    return points[i - 1].clone().lerp(points[i], t);
  };
}

function fanSamplers(insts, matchGuides) {
  const paths = insts.map(inst => A.strandPath(W, inst));
  if (!matchGuides) return paths.map(pathSampler);
  const sections = insts.map((inst, i) => {
    const path = paths[i], anchors = inst.pts.map(p => A.pt(W, p.seg, p.p));
    const indices = []; let from = 0;
    for (const anchor of anchors) {
      let found = -1;
      for (let j = from; j < path.length; j++) if (path[j].distanceToSquared(anchor) < 1e-18) { found = j; break; }
      if (found < 0) throw Error(`${inst.unit}: missing original guide in the routed path`);
      indices.push(found); from = found;
    }
    return indices.slice(1).map((to, k) => path.slice(indices[k], to + 1));
  });
  const stations = [0];
  for (let k = 0; k < sections[0].length; k++) {
    const mean = sections.reduce((s, paths) => s + A.pathLength(paths[k]), 0) / sections.length;
    stations.push(stations.at(-1) + mean);
  }
  const total = stations.at(-1); for (let k = 1; k < stations.length; k++) stations[k] /= total;
  return sections.map(section => {
    const samplers = section.map(pathSampler);
    return u => {
      let k = 0; while (k < samplers.length - 1 && u > stations[k + 1]) k++;
      return samplers[k]((u - stations[k]) / (stations[k + 1] - stations[k] || 1));
    };
  });
}

function across(samples, x) {
  // Catmull-Rom interpolation through the anatomical fan paths retains a wide
  // origin, convergent insertion, and intervening wrapping guides.
  const f = clamp(x, 0, 1) * (samples.length - 1);
  const i = Math.min(samples.length - 2, Math.floor(f)), t = f - i, t2 = t * t, t3 = t2 * t;
  const at = j => samples[clamp(j, 0, samples.length - 1)];
  return at(i).clone().multiplyScalar(2 - 5 * t2 + 3 * t3)
    .addScaledVector(at(i - 1), -t + 2 * t2 - t3)
    .addScaledVector(at(i + 1), t + 4 * t2 - 3 * t3)
    .addScaledVector(at(i + 2), -t2 + t3).multiplyScalar(0.5);
}

function profile(u, inst) {
  const ease = t => { const x = clamp(t, 0, 1); return x * x * (3 - 2 * x); };
  // Broad trunk muscles do not have the long pinched cigar profile of a limb
  // muscle. Keep a broad belly and ease into a thin aponeurosis at either end.
  const inLength = inst.unit === 'rectAbd' ? 0.16 : 0.22;
  const outLength = inst.unit === 'lat' ? 0.24 : 0.17;
  let p = 0.11 + 0.89 * ease(u / inLength) * ease((1 - u) / outLength);
  // The rectus intersections are shallow transverse transitions, not deep cuts
  // that divide a strip into a stack of rigid rectangular blocks.
  for (const d of inst.dips || []) p *= 1 - 0.14 * Math.exp(-Math.pow((u - d) / 0.040, 2));
  return p;
}

function signedVolume(positions, indices) {
  // Centering removes cancellation caused by the absolute standing-body height.
  const c = V();
  for (let i = 0; i < positions.length; i += 3) c.add(new THREE.Vector3().fromArray(positions, i));
  c.divideScalar(positions.length / 3);
  let volume = 0;
  for (let i = 0; i < indices.length; i += 3) {
    const a = new THREE.Vector3().fromArray(positions, indices[i] * 3).sub(c);
    const b = new THREE.Vector3().fromArray(positions, indices[i + 1] * 3).sub(c);
    const d = new THREE.Vector3().fromArray(positions, indices[i + 2] * 3).sub(c);
    volume += a.dot(b.cross(d)) / 6;
  }
  return volume;
}

function solidSheet(rows, columns, centers, thickness) {
  const positions = [], indices = [], around = Math.max(12, columns * 2);
  const normals = [];
  const at = (i, j) => centers[clamp(i, 0, rows)][clamp(j, 0, columns)];
  for (let i = 0; i <= rows; i++) {
    const row = [];
    for (let j = 0; j <= columns; j++) {
      const du = at(i + 1, j).clone().sub(at(i - 1, j));
      const dv = at(i, j + 1).clone().sub(at(i, j - 1));
      let n = du.clone().cross(dv);
      if (n.lengthSq() < 1e-16) n = row.at(-1)?.clone() || normals.at(-1)?.[j].clone() || V(0, 0, 1);
      row.push(n.normalize());
    }
    normals.push(row);
  }
  // Follow the complete fan sheet across each section. Only its thickness uses
  // an elliptical rim: v=(1-cos(theta))/2 and h*sin(theta). Unlike two stitched
  // flat laminae this has no vertical sidewalls, cut edges, or UV seam vertices.
  for (let i = 0; i <= rows; i++) for (let j = 0; j < around; j++) {
    const angle = j / around * Math.PI * 2, v = (1 - Math.cos(angle)) * 0.5;
    const f = v * columns, a = Math.min(columns - 1, Math.floor(f)), t = f - a;
    const normal = normals[i][a].clone().lerp(normals[i][a + 1], t).normalize();
    const p = across(centers[i], v).addScaledVector(normal, thickness(i / rows, v) * Math.sin(angle));
    positions.push(p.x, p.y, p.z);
  }
  for (let i = 0; i < rows; i++) for (let j = 0; j < around; j++) {
    const a = i * around + j, b = i * around + (j + 1) % around;
    const c = a + around, d = b + around;
    indices.push(a, c, b, b, c, d);
  }
  // Close a curved attachment section with narrow upper/lower strips. A fan
  // through the mean of a crescent section would bridge across the torso and
  // create inverted triangles even if the total signed volume stayed positive.
  for (const end of [0, rows]) {
    const offset = end * around;
    const face = (a, b, c) => { if (!end) indices.push(a, b, c); else indices.push(a, c, b); };
    for (let j = 0; j < around / 2; j++) {
      const a = offset + j, b = offset + j + 1;
      const c = offset + (around - j) % around, d = offset + (around - j - 1) % around;
      if (a === c) face(a, b, d);
      else if (b === d) face(a, b, c);
      else { face(a, b, c); face(b, d, c); }
    }
  }
  let volume = signedVolume(positions, indices);
  if (volume < 0) {
    for (let i = 0; i < indices.length; i += 3) [indices[i + 1], indices[i + 2]] = [indices[i + 2], indices[i + 1]];
    volume = -volume;
  }
  return { positions, indices, volume };
}

function conformAcrossWrap(p, samples, v, wraps) {
  const f = v * (samples.length - 1), a = Math.min(samples.length - 2, Math.floor(f)), t = f - a;
  for (const wrap of wraps) {
    // Only the torso's broad wrapping surfaces apply to the inter-fascicle
    // sheet. Joint-sized obstacles remain controlled by the original paths.
    if (wrap.r.x < 0.1 || wrap.r.y < 0.12) continue;
    const center = A.pt(W, wrap.seg, wrap.c);
    const qa = samples[a].clone().sub(center).divide(wrap.r);
    const qb = samples[a + 1].clone().sub(center).divide(wrap.r);
    const ra = qa.length(), rb = qb.length();
    if (Math.min(ra, rb) < 0.92 || Math.max(ra, rb) > 1.15) continue;
    const q = p.clone().sub(center).divide(wrap.r), r = q.length();
    const wanted = ra + (rb - ra) * t;
    // Radial interpolation avoids a chord through the belly wall. It preserves
    // the source path radii (including their small geometric approximation),
    // rather than claiming a newly measured contact surface.
    if (r > 1e-8 && r < wanted) p.copy(q.multiplyScalar(wanted / r).multiply(wrap.r).add(center));
  }
  return p;
}

function fanVolume(unit, side, insts) {
  const rows = unit === 'rectAbd' ? 44 : unit === 'lat' ? 36 : 28;
  const columns = unit === 'lat' ? 14 : 12;
  // The latissimus paths have very different lengths before their common
  // axillary guides. A single arc fraction joins unrelated anatomical stations
  // and folds the broad sheet. Match each original guide interval instead.
  const samplers = fanSamplers(insts, unit === 'lat');
  const centers = [];
  const meanRadius = insts.reduce((s, x) => s + x.r, 0) / insts.length;
  // These are display thickness estimates, documented as such; they are not
  // measured PCSA values and do not enter the force solver or its Fmax weights.
  const halfThickness = { lat: 0.0035, ql: 0.009, rectAbd: 0.0075, intObl: 0.0035, transAbd: 0.0025 }[unit];
  for (let i = 0; i <= rows; i++) {
    const u = i / rows, samples = samplers.map(f => f(u));
    const width = samples.at(-1).clone().sub(samples[0]).normalize();
    const spread = unit === 'ql' ? 0.9 : unit === 'rectAbd' ? 0.75 : unit === 'lat' ? 0.7 : 0.42;
    const extension = meanRadius * spread * Math.pow(Math.sin(Math.PI * u), 0.7);
    const row = [];
    for (let j = 0; j <= columns; j++) {
      const v = j / columns;
      const p = across(samples, v);
      if (i && i < rows) conformAcrossWrap(p, samples, v, insts[0].wraps);
      row.push(p.addScaledVector(width, (2 * v - 1) * extension));
    }
    centers.push(row);
  }
  const mesh = solidSheet(rows, columns, centers, u => halfThickness * profile(u, insts[0]));
  return {
    unit, side, source: 'reconstructed', ids: [], ...mesh,
    method: 'Closed rounded envelope across the existing anatomical fan paths, with torso-conforming inter-fascicle sections, a broad belly, and smooth aponeurotic transitions.',
    limitations: [
      'Model-derived reconstruction; this part is not a segmented BodyParts3D mesh.',
      'Attachment paths and wrapping guides are inherited from the existing rig; their anatomical limitations remain.',
      'Thickness and edge spread are display estimates, not measured tissue geometry, PCSA, or a tissue deformation law.',
      'The right surface mirrors the left because the underlying rest rig and reconstructed attachment paths are symmetric.',
      ...(unit === 'rectAbd' ? ['Shallow transverse transitions follow the existing modeled tendinous intersections.'] : []),
    ],
  };
}

function multifidusVolume(side, insts) {
  const positions = [], indices = [];
  let volume = 0, slips = 0;
  for (const inst of insts) {
    const points = A.strandPath(W, inst), sampler = pathSampler(points);
    // Overlapping short lamellar slips preserve a segmental organization. Their
    // locations follow the three regional model paths; they do not claim a new
    // clinically validated vertebra-by-vertebra attachment atlas.
    const intervals = Math.max(2, points.length - 1);
    for (let s = 0; s < intervals - 1; s++) {
      const rows = 12, columns = 4, centers = [];
      const start = s / intervals, end = Math.min(1, (s + 2) / intervals);
      for (let i = 0; i <= rows; i++) {
        const u = i / rows, p = sampler(start + (end - start) * u), row = [];
        const pennate = 0.22 + 0.78 * Math.pow(Math.sin(Math.PI * u), 0.7);
        const width = inst.r * 0.95 * pennate;
        for (let j = 0; j <= columns; j++) {
          const v = j / columns, x = (2 * v - 1) * width * (side === 'R' ? -1 : 1);
          row.push(p.clone().add(V(x, 0, -Math.sin(Math.PI * u) * inst.r * 0.22)));
        }
        centers.push(row);
      }
      const mesh = solidSheet(rows, columns, centers, u =>
        inst.r * 0.52 * (0.08 + 0.92 * Math.sin(Math.PI * u)));
      const offset = positions.length / 3;
      positions.push(...mesh.positions); indices.push(...mesh.indices.map(i => i + offset));
      volume += mesh.volume; slips++;
    }
  }
  return {
    unit: 'multifidus', side, source: 'reconstructed', ids: [], positions, indices, volume, slips,
    method: 'Overlapping short closed lamellar slips along the existing lumbar, thoracic, and cervical multifidus paths.',
    limitations: [
      'Model-derived segmental reconstruction; this part is not a segmented BodyParts3D mesh.',
      'The regional model paths do not resolve every individual vertebral attachment.',
      'Slip width and thickness are display estimates and do not alter the physical paths or maximum muscle force.',
    ],
  };
}

function validate(part) {
  if (!part.positions.every(Number.isFinite) || part.positions.length % 3) throw Error(`${part.unit}: invalid vertices`);
  if (!(part.volume > 1e-10)) throw Error(`${part.unit}: non-positive volume`);
  const measuredVolume = signedVolume(part.positions, part.indices);
  if (Math.abs(measuredVolume - part.volume) > Math.max(1e-12, part.volume * 1e-8)) {
    throw Error(`${part.unit}: inconsistent surface orientation or enclosed volume`);
  }
  const edges = new Map(), vertexCount = part.positions.length / 3;
  let minArea = Infinity;
  for (let i = 0; i < part.indices.length; i += 3) {
    const face = part.indices.slice(i, i + 3);
    if (face.some(x => !Number.isInteger(x) || x < 0 || x >= vertexCount)) throw Error(`${part.unit}: index out of range`);
    const a = new THREE.Vector3().fromArray(part.positions, face[0] * 3);
    const b = new THREE.Vector3().fromArray(part.positions, face[1] * 3).sub(a);
    const c = new THREE.Vector3().fromArray(part.positions, face[2] * 3).sub(a);
    minArea = Math.min(minArea, b.cross(c).length() / 2);
    for (const [x, y] of [[face[0], face[1]], [face[1], face[2]], [face[2], face[0]]]) {
      const key = x < y ? `${x}/${y}` : `${y}/${x}`;
      const e = edges.get(key) || { count: 0, winding: 0 };
      e.count++; e.winding += x < y ? 1 : -1; edges.set(key, e);
    }
  }
  if ([...edges.values()].some(e => e.count !== 2 || e.winding !== 0)) throw Error(`${part.unit}: open or inconsistently wound mesh`);
  if (!(minArea > 1e-13)) throw Error(`${part.unit}: degenerate face (${minArea})`);
  const ys = part.positions.filter((_, i) => i % 3 === 1);
  if (Math.min(...ys) < 0.75 || Math.max(...ys) > 1.70) throw Error(`${part.unit}: rest-coordinate translation mismatch`);
  reports.push({ unit: part.unit, side: part.side, vertices: vertexCount, triangles: part.indices.length / 3, volumeMl: +(part.volume * 1e6).toFixed(2), minArea });
}

for (const unit of targetUnits) {
  const left = all.filter(s => s.unit === unit && s.side === 'L');
  const right = all.filter(s => s.unit === unit && s.side === 'R');
  if (!left.length || left.length !== right.length) throw Error(`Missing or asymmetric path sets for ${unit}`);
  for (let i = 0; i < left.length; i++) {
    const l = A.strandPath(W, left[i]), r = A.strandPath(W, right[i]);
    if (l.length !== r.length || l.some((p, j) => Math.hypot(p.x + r[j].x, p.y - r[j].y, p.z - r[j].z) > 1e-12)) {
      throw Error(`Cannot mirror an asymmetric reconstructed path: ${unit}`);
    }
  }
  const l = unit === 'multifidus' ? multifidusVolume('L', left) : fanVolume(unit, 'L', left);
  validate(l);
  // Preserve an identical triangulation in the mirror. Re-triangulating a
  // curved nonplanar quad along the opposite diagonal changes its displayed
  // shape and enclosed volume despite sharing the same reflected vertices.
  const r = { ...l, side: 'R', positions: l.positions.map((v, i) => i % 3 ? v : -v), indices: l.indices.slice() };
  for (let i = 0; i < r.indices.length; i += 3) [r.indices[i + 1], r.indices[i + 2]] = [r.indices[i + 2], r.indices[i + 1]];
  validate(r);
  for (const part of [l, r]) {
    // Keep adequate precision for the following uint16 atlas quantization.
    part.positions = part.positions.map(v => +v.toFixed(7));
    parts.push(part);
  }
}
const outfile = process.argv[2] || '/tmp/fysm-bp3d/reconstructed-muscles.json';
mkdirSync(dirname(outfile), { recursive: true });
writeFileSync(outfile, JSON.stringify({
  version: 1, source: 'Model-derived reconstruction, separate from BodyParts3D',
  coordinateSystem: 'Metres in standing rig coordinates, pelvis at its anatomical rest joint',
  parts,
}));
console.log(JSON.stringify({ outfile, parts: reports, totalTriangles: reports.reduce((s, p) => s + p.triangles, 0) }, null, 2));
