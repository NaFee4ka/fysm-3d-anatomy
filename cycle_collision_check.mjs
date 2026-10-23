// Collision checks for the actual interpolation used by the page. The fast
// arm-core screen is deliberately separate from the surface/containment test.
// node cycle_collision_check.mjs [--input FILE] [--packed FILE] [--preflight]
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { capsuleClearance } from './collision_geometry.mjs';

const A = createAnatomy(THREE);
const ROOT = path.dirname(fileURLToPath(import.meta.url));
export const COLLISION_SUBSTEPS = 20;
const V = (...v) => new THREE.Vector3(...v);
const finite = value => typeof value === 'number' && Number.isFinite(value);
const requireValue = (ok, message) => { if (!ok) throw new Error('Body collision check: ' + message); };
const digest = value => createHash('sha256').update(value).digest('hex');

function quaternion(values) {
  requireValue(Array.isArray(values) && values.length === 4 && values.every(finite)
    && Math.hypot(...values) > 1e-8, 'invalid quaternion');
  return new THREE.Quaternion(...values).normalize();
}
function position(values) {
  requireValue(Array.isArray(values) && values.length === 3 && values.every(finite), 'invalid root position');
  return V(...values);
}
function int16(text, count) {
  requireValue(typeof text === 'string', 'missing packed geometry');
  const buffer = Buffer.from(text, 'base64');
  requireValue(buffer.length === count * 2, 'invalid packed geometry length');
  return Array.from({ length: count }, (_, k) => buffer.readInt16LE(2 * k));
}

export function motionFrames(data) {
  if (typeof data.q === 'string') {
    const { n, fps, segNames } = data;
    requireValue(Number.isInteger(n) && n >= 2 && finite(fps) && fps > 0, 'invalid packed frame grid');
    requireValue(Array.isArray(segNames) && segNames.length === A.SEG.length
      && new Set(segNames).size === segNames.length && A.SEG.every(s => segNames.includes(s.name)), 'incomplete packed rig');
    const ns = segNames.length, qs = int16(data.q, n * ns * 4);
    const roots = int16(data.root, n * 4), ps = int16(data.p, n * 3);
    return Array.from({ length: n }, (_, i) => ({
      time: i / fps,
      local: Object.fromEntries(segNames.map((name, k) => [name, quaternion(qs.slice((i * ns + k) * 4, (i * ns + k + 1) * 4).map(v => v / 32767))])),
      rootQ: quaternion(roots.slice(i * 4, i * 4 + 4).map(v => v / 32767)),
      rootP: position(ps.slice(i * 3, i * 3 + 3).map(v => v * 1e-4)),
    }));
  }
  requireValue(Array.isArray(data.frames) && data.frames.length >= 2, 'missing frames');
  return data.frames.map((frame, i) => {
    const time = frame.T ?? frame.t;
    requireValue(finite(time) && (i === 0 || time > (data.frames[i - 1].T ?? data.frames[i - 1].t)), 'invalid frame times');
    requireValue(frame.local && A.SEG.every(s => frame.local[s.name]), 'incomplete frame rig');
    return { time, local: Object.fromEntries(A.SEG.map(s => [s.name, quaternion(frame.local[s.name])])),
      rootQ: quaternion(frame.root), rootP: position(frame.rootP) };
  });
}

export function interpolateMotion(a, b, u) {
  return {
    time: a.time + (b.time - a.time) * u,
    local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion().slerpQuaternions(a.local[s.name], b.local[s.name], u)])),
    rootQ: new THREE.Quaternion().slerpQuaternions(a.rootQ, b.rootQ, u),
    rootP: a.rootP.clone().lerp(b.rootP, u),
  };
}

export function screenArmCores(W) {
  // Thin screening cores, not an enclosing skin shape or measured body radii.
  // Full surface checks below are still required before publishing the round.
  const parts = side => [
    { id: 'upper_' + side, a: W['hum_' + side].p, b: W['fore_' + side].p, radiusM: .020 },
    { id: 'fore_' + side, a: W['fore_' + side].p, b: W['hand_' + side].p, radiusM: .020 },
    { id: 'palm_' + side, a: W['hand_' + side].p, b: W['f3a_' + side].p, radiusM: .012 },
  ];
  let minimum = Infinity, pair = null;
  for (const left of parts('L')) for (const right of parts('R')) {
    const { clearanceM } = capsuleClearance(left, right);
    if (clearanceM < minimum) { minimum = clearanceM; pair = [left.id, right.id]; }
  }
  return { clearanceM: minimum, pair };
}

export function geometryDigest(data) {
  return digest(JSON.stringify(typeof data.q === 'string'
    ? [data.n, data.fps, data.segNames, data.q, data.root, data.p]
    : data.frames.map(f => [f.T ?? f.t, f.local, f.root, f.rootP])));
}

export async function validateMotionCollisions(data, options = {}) {
  const steps = options.substeps ?? COLLISION_SUBSTEPS;
  requireValue(Number.isInteger(steps) && steps >= 1, 'invalid subdivision count');
  const frames = motionFrames(data), preflight = options.preflight === true;
  let engine = options.engine;
  if (!preflight && !engine) {
    const { createBodyCollision } = await import('./body_collision.mjs');
    const bones = JSON.parse(fs.readFileSync(path.join(ROOT, 'skull/bones_bp3d.json'), 'utf8'));
    engine = createBodyCollision(A, THREE, bones);
  }
  let states = 0, minimumCoreClearanceM = Infinity, minimumAt = null;
  for (let i = 0; i < frames.length; i++) {
    const count = i + 1 < frames.length ? steps : 1;
    for (let k = 0; k < count; k++) {
      const c = i + 1 < frames.length ? interpolateMotion(frames[i], frames[i + 1], k / steps) : frames[i];
      const W = A.computeWorld(c.local, c.rootQ, c.rootP);
      const screen = screenArmCores(W);
      if (screen.clearanceM < minimumCoreClearanceM) { minimumCoreClearanceM = screen.clearanceM; minimumAt = { time: c.time, pair: screen.pair }; }
      requireValue(screen.clearanceM >= -1e-6,
        'arm cores intersect at ' + c.time.toFixed(4) + ' s: ' + screen.pair.join(' / ')
        + ', clearance ' + (screen.clearanceM * 1000).toFixed(3) + ' mm');
      if (!preflight) {
        const result = engine.inspect(W);
        requireValue(result && typeof result.colliding === 'boolean', 'missing surface-check result');
        requireValue(!result.colliding, 'opposite arm bones intersect at ' + c.time.toFixed(4)
          + ' s: ' + JSON.stringify(result.pairs?.slice(0, 4)));
      }
      states++;
    }
  }
  const checkerFiles = ['anatomy3.js', 'collision_geometry.mjs', 'body_collision.mjs', 'cycle_collision_check.mjs', 'skull/bones_bp3d.json'];
  return { v: 1, verified: !preflight, scope: 'opposite-arm-bones', boneParts: preflight ? null : engine.boneParts,
    method: preflight ? 'thin-core-preflight' : 'mesh-intersection-and-containment',
    frameCount: frames.length, states, substeps: steps, maxTimeStepS: Math.max(...frames.slice(1).map((f, i) => (f.time - frames[i].time) / steps)),
    minimumCoreClearanceM, minimumAt, geometrySha256: geometryDigest(data),
    sourceSha256: Object.fromEntries(checkerFiles.filter(f => fs.existsSync(path.join(ROOT, f))).map(f => [f, digest(fs.readFileSync(path.join(ROOT, f)))])),
    limits: 'Sampled rigid bone geometry of opposite arms, including palms and fingers; not continuous collision detection or contact/deformation of every soft tissue.' };
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const arg = (name, fallback) => { const i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : fallback; };
  const filename = arg('--packed', arg('--input', 'cycle_keys/cycle.json'));
  const data = JSON.parse(fs.readFileSync(filename, 'utf8'));
  const result = await validateMotionCollisions(data, { preflight: process.argv.includes('--preflight') });
  console.log(JSON.stringify(result));
}
