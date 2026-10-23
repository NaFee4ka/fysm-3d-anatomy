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
  const belly = inst.belly || 0.5;
  const t = u < belly ? 0.5 * u / belly : 0.5 + 0.5 * (u - belly) / (1 - belly);
  let p = 0.055 + 0.945 * Math.pow(Math.max(0, Math.sin(Math.PI * t)), 0.65);
  for (const d of inst.dips || []) p *= 1 - 0.50 * Math.exp(-Math.pow((u - d) / 0.026, 2));
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
  const positions = [], indices = [], size = (rows + 1) * (columns + 1);
  const normals = [];
  const at = (i, j) => centers[clamp(i, 0, rows)][clamp(j, 0, columns)];
  // Continuous surface normals give the two anatomical laminae of a closed
  // muscle volume, rather than one tube for every numerical fascicle.
  for (let i = 0; i <= rows; i++) for (let j = 0; j <= columns; j++) {
    const du = at(i + 1, j).clone().sub(at(i - 1, j));
    const dv = at(i, j + 1).clone().sub(at(i, j - 1));
    let n = du.clone().cross(dv);
    if (n.lengthSq() < 1e-16) n = normals.at(-1)?.clone() || V(0, 0, 1);
    n.normalize(); normals.push(n);
  }
  for (const sign of [1, -1]) for (let i = 0; i <= rows; i++) for (let j = 0; j <= columns; j++) {
    const k = i * (columns + 1) + j;
    const p = centers[i][j].clone().addScaledVector(normals[k], sign * thickness(i / rows, j / columns));
    positions.push(p.x, p.y, p.z);
  }
  const top = [], boundaries = new Map();
  for (let i = 0; i < rows; i++) for (let j = 0; j < columns; j++) {
    const a = i * (columns + 1) + j, b = a + 1, c = a + columns + 1, d = c + 1;
    top.push(a, c, b, b, c, d);
  }
  for (let i = 0; i < top.length; i += 3) {
    const a = top[i], b = top[i + 1], c = top[i + 2];
    indices.push(a, b, c, a + size, c + size, b + size);
    for (const [x, y] of [[a, b], [b, c], [c, a]]) {
      const key = x < y ? `${x}/${y}` : `${y}/${x}`;
      if (boundaries.has(key)) boundaries.delete(key); else boundaries.set(key, [x, y]);
    }
  }
  // Stitch every boundary to the opposite lamina: watertight volume including
  // the thin tendon/aponeurosis rims, without an open surface or a triangle fan.
  for (const [a, b] of boundaries.values()) indices.push(b, a, a + size, b, a + size, b + size);
  let volume = signedVolume(positions, indices);
  if (volume < 0) {
    for (let i = 0; i < indices.length; i += 3) [indices[i + 1], indices[i + 2]] = [indices[i + 2], indices[i + 1]];
    volume = -volume;
  }
  return { positions, indices, volume };
}

function fanVolume(unit, side, insts) {
  const rows = unit === 'lat' ? 32 : 28, columns = unit === 'lat' ? 12 : 10;
  const samplers = insts.map(inst => pathSampler(A.strandPath(W, inst)));
  const centers = [];
  const meanRadius = insts.reduce((s, x) => s + x.r, 0) / insts.length;
  // These are display thickness estimates, documented as such; they are not
  // measured PCSA values and do not enter the force solver or its Fmax weights.
  const halfThickness = { lat: 0.0035, ql: 0.009, rectAbd: 0.006, intObl: 0.0035, transAbd: 0.0025 }[unit];
  for (let i = 0; i <= rows; i++) {
    const u = i / rows, samples = samplers.map(f => f(u));
    const width = samples.at(-1).clone().sub(samples[0]).normalize();
    const extension = meanRadius * (unit === 'ql' ? 0.9 : 0.42) * Math.pow(Math.sin(Math.PI * u), 0.7);
    const row = [];
    for (let j = 0; j <= columns; j++) {
      const v = j / columns;
      row.push(across(samples, v).addScaledVector(width, (2 * v - 1) * extension));
    }
    centers.push(row);
  }
  const mesh = solidSheet(rows, columns, centers, (u, v) => {
    const edge = 0.12 + 0.88 * Math.pow(Math.sin(Math.PI * v), 0.6);
    return Math.max(0.00018, halfThickness * profile(u, insts[0]) * edge);
  });
  return {
    unit, side, source: 'reconstructed', ids: [], ...mesh,
    method: 'Closed two-lamina surface interpolated through the existing anatomical fan paths, with a tapered belly, broad attachment rims, and thin tendinous ends.',
    limitations: [
      'Model-derived reconstruction; this part is not a segmented BodyParts3D mesh.',
      'Attachment paths and wrapping guides are inherited from the existing rig; their anatomical limitations remain.',
      'Thickness and edge spread are display estimates, not measured tissue geometry, PCSA, or a tissue deformation law.',
      ...(unit === 'rectAbd' ? ['Transverse grooves follow the existing modeled tendinous intersections.'] : []),
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
      const mesh = solidSheet(rows, columns, centers, (u, v) =>
        Math.max(0.00018, inst.r * 0.52 * (0.08 + 0.92 * Math.sin(Math.PI * u)) * (0.15 + 0.85 * Math.sin(Math.PI * v))));
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

for (const unit of targetUnits) for (const side of ['L', 'R']) {
  const insts = all.filter(s => s.unit === unit && s.side === side);
  if (!insts.length) throw Error(`No paths for ${unit}/${side}`);
  const part = unit === 'multifidus' ? multifidusVolume(side, insts) : fanVolume(unit, side, insts);
  validate(part);
  // Keep adequate precision for the following uint16 atlas quantization.
  part.positions = part.positions.map(v => +v.toFixed(7));
  parts.push(part);
}
const outfile = process.argv[2] || '/tmp/fysm-bp3d/reconstructed-muscles.json';
mkdirSync(dirname(outfile), { recursive: true });
writeFileSync(outfile, JSON.stringify({
  version: 1, source: 'Model-derived reconstruction, separate from BodyParts3D',
  coordinateSystem: 'Metres in standing rig coordinates, pelvis at its anatomical rest joint',
  parts,
}));
console.log(JSON.stringify({ outfile, parts: reports, totalTriangles: reports.reduce((s, p) => s + p.triangles, 0) }, null, 2));
