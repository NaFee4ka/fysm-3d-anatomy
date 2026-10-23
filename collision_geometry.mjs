// Geometric distances in metres. No anatomy, motion correction or contact force
// law is implied. Inputs are Vector3-compatible objects; they are never changed.
const clamp01 = value => Math.max(0, Math.min(1, value));

function vector(point, name) {
  if (!point || typeof point.clone !== 'function'
    || ![point.x, point.y, point.z].every(Number.isFinite))
    throw new TypeError('Collision geometry needs a finite Vector3: ' + name);
}

// The minimum of a convex quadratic on [0,1]^2 occurs at its interior stationary
// point or on an edge. Test the four edge minima and, when present, the interior.
// Cross products avoid subtracting nearly equal a*e and b*b for almost-parallel
// segments; a small nonzero angle must not erase an actual crossing.
export function segmentClosest(a0, a1, b0, b1) {
  for (const [point, name] of [[a0, 'a0'], [a1, 'a1'], [b0, 'b0'], [b1, 'b1']]) vector(point, name);
  const ux = a1.x - a0.x, uy = a1.y - a0.y, uz = a1.z - a0.z;
  const vx = b1.x - b0.x, vy = b1.y - b0.y, vz = b1.z - b0.z;
  const wx = b0.x - a0.x, wy = b0.y - a0.y, wz = b0.z - a0.z;
  const uu = ux * ux + uy * uy + uz * uz, vv = vx * vx + vy * vy + vz * vz;
  const uv = ux * vx + uy * vy + uz * vz;
  const uw = ux * wx + uy * wy + uz * wz, vw = vx * wx + vy * wy + vz * wz;
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const nn = nx * nx + ny * ny + nz * nz;
  if (![uu, vv, uv, uw, vw, nn].every(Number.isFinite))
    throw new RangeError('Collision geometry exceeds the finite coordinate range');
  let bestS = 0, bestT = 0, distance = Infinity;
  const candidate = (s, t) => {
    const d = Math.hypot(ux * s - wx - vx * t, uy * s - wy - vy * t, uz * s - wz - vz * t);
    if (d < distance) { distance = d; bestS = s; bestT = t; }
  };
  candidate(0, vv > 0 ? clamp01(-vw / vv) : 0);
  candidate(1, vv > 0 ? clamp01((uv - vw) / vv) : 0);
  candidate(uu > 0 ? clamp01(uw / uu) : 0, 0);
  candidate(uu > 0 ? clamp01((uw + uv) / uu) : 0, 1);
  if (nn > 0) {
    const wvx = wy * vz - wz * vy, wvy = wz * vx - wx * vz, wvz = wx * vy - wy * vx;
    const wux = wy * uz - wz * uy, wuy = wz * ux - wx * uz, wuz = wx * uy - wy * ux;
    const s = (wvx * nx + wvy * ny + wvz * nz) / nn;
    const t = (wux * nx + wuy * ny + wuz * nz) / nn;
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) candidate(s, t);
  }
  if (!Number.isFinite(distance)) throw new RangeError('Non-finite collision distance');
  const a = a0.clone().set(a0.x + ux * bestS, a0.y + uy * bestS, a0.z + uz * bestS);
  const b = b0.clone().set(b0.x + vx * bestT, b0.y + vy * bestT, b0.z + vz * bestT);
  return { a, b, distance };
}

export function capsuleClearance(ca, cb) {
  for (const capsule of [ca, cb]) {
    if (!capsule || !Number.isFinite(capsule.radiusM) || capsule.radiusM < 0)
      throw new RangeError('Capsule radius must be finite and nonnegative');
  }
  const radii = ca.radiusM + cb.radiusM;
  if (!Number.isFinite(radii)) throw new RangeError('Capsule radii exceed the finite range');
  const { a, b, distance } = segmentClosest(ca.a, ca.b, cb.a, cb.b);
  return { a, b, distanceM: distance, clearanceM: distance - radii };
}
