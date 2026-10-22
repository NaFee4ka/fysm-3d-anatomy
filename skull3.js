// ---------- skull: signed distance model, polygonised with surface nets ----------
// skullData() is self-contained (no closures, no THREE) so it can run in a Web Worker: it returns plain typed arrays
// (positions relative to the head joint J, gradient normals, cavity shading) for the cranium, the mandible and the teeth.
// Coordinates: standing rest pose, metres, +z forward, +y up, +x = left side.
function skullData(Jx, Jy, Jz) {
  const sq = Math.sqrt, abs = Math.abs, max = Math.max, min = Math.min, PI = Math.PI;
  const smin = (a, b, k) => { const h = max(k - abs(a - b), 0) / k; return min(a, b) - h * h * k * 0.25; };
  const smax = (a, b, k) => -smin(-a, -b, k);
  const ell = (x, y, z, c, r) => {
    const px = (x - c[0]) / r[0], py = (y - c[1]) / r[1], pz = (z - c[2]) / r[2];
    const k0 = sq(px * px + py * py + pz * pz);
    const qx = px / r[0], qy = py / r[1], qz = pz / r[2];
    return k0 * (k0 - 1) / (sq(qx * qx + qy * qy + qz * qz) || 1e-9);
  };
  const cap = (x, y, z, a, b, r) => {
    const bx = b[0] - a[0], by = b[1] - a[1], bz = b[2] - a[2], px = x - a[0], py = y - a[1], pz = z - a[2];
    const t = max(0, min(1, (px * bx + py * by + pz * bz) / (bx * bx + by * by + bz * bz)));
    const dx = px - bx * t, dy = py - by * t, dz = pz - bz * t;
    return sq(dx * dx + dy * dy + dz * dz) - r;
  };
  const cone = (x, y, z, a, b, ra, rb) => {          // capsule with linearly changing radius
    const bx = b[0] - a[0], by = b[1] - a[1], bz = b[2] - a[2], px = x - a[0], py = y - a[1], pz = z - a[2];
    const t = max(0, min(1, (px * bx + py * by + pz * bz) / (bx * bx + by * by + bz * bz)));
    const dx = px - bx * t, dy = py - by * t, dz = pz - bz * t;
    return sq(dx * dx + dy * dy + dz * dz) - (ra + (rb - ra) * t);
  };
  const sph = (x, y, z, c, r) => { const dx = x - c[0], dy = y - c[1], dz = z - c[2]; return sq(dx * dx + dy * dy + dz * dz) - r; };

  // ---- curves lying on the vault: sutures (zig-zag grooves) and muscle lines (low ridges) ----
  const VC = [0, 1.675, -0.004], VR = [0.071, 0.079, 0.092];
  const onVault = (d, lift) => { const k = sq((d[0] / VR[0]) ** 2 + (d[1] / VR[1]) ** 2 + (d[2] / VR[2]) ** 2); return [VC[0] + d[0] / k * (1 + lift), VC[1] + d[1] / k * (1 + lift), VC[2] + d[2] / k * (1 + lift)]; };
  function polyline(f, n, zig) {
    const P = [];
    for (let i = 0; i <= n; i++) P.push(onVault(f(i / n), 0));
    if (!zig) return P;
    const Z = [];
    for (let i = 0; i < P.length - 1; i++) for (let s = 0; s < 4; s++) {
      const u = s / 4, a = P[i], b = P[i + 1];
      const p = [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
      const t = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], c = [p[0] - VC[0], p[1] - VC[1], p[2] - VC[2]];
      const sx = t[1] * c[2] - t[2] * c[1], sy = t[2] * c[0] - t[0] * c[2], sz = t[0] * c[1] - t[1] * c[0], sl = sq(sx * sx + sy * sy + sz * sz) || 1;
      const amp = zig * Math.sin((i * 4 + s) * PI * 0.5 + 0.7) * (0.6 + 0.4 * Math.sin(i * 1.7));
      Z.push([p[0] + sx / sl * amp, p[1] + sy / sl * amp, p[2] + sz / sl * amp]);
    }
    Z.push(P[P.length - 1]);
    return Z;
  }
  const segs = P => { const S = []; for (let i = 1; i < P.length; i++) { const a = P[i - 1], b = P[i]; const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2]; const hl = sq((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) / 2 + 0.004; S.push([a, b, m, hl * hl]); } return S; };
  const SUT = [].concat(
    segs(polyline(t => { const a = PI * (0.16 + 0.68 * t); return [Math.cos(a), Math.sin(a), 0.42]; }, 22, 0.0011)),        // coronal
    segs(polyline(t => { const a = 0.4 - 1.45 * t; return [0, Math.cos(a), Math.sin(a)]; }, 22, 0.0013)),                       // sagittal
    segs(polyline(t => { const a = PI * (0.1 + 0.8 * t); return [Math.cos(a), 0.55 * Math.sin(a) + 0.35, -1]; }, 22, 0.0015)), // lambdoid
    segs(polyline(t => [1, 0.12 + 0.3 * Math.sin(PI * t), 0.35 - 0.95 * t], 12, 0.0004)),                                       // squamous L
    segs(polyline(t => [-1, 0.12 + 0.3 * Math.sin(PI * t), 0.35 - 0.95 * t], 12, 0.0004)));                                     // squamous R
  const RID = [].concat(
    segs(polyline(t => [1, 0.55 - 0.45 * t + 0.25 * Math.sin(PI * t), 0.75 - 1.5 * t], 14, 0)),                                // temporal line L
    segs(polyline(t => [-1, 0.55 - 0.45 * t + 0.25 * Math.sin(PI * t), 0.75 - 1.5 * t], 14, 0)),                               // temporal line R
    segs(polyline(t => { const a = PI * (0.18 + 0.64 * t); return [Math.cos(a), -0.42 + 0.12 * Math.sin(a), -1]; }, 16, 0))); // superior nuchal line
  const curveDist = (x, y, z, S, r) => { let g = 1; for (const [a, b, m, R2] of S) { const dx = x - m[0], dy = y - m[1], dz = z - m[2]; if (dx * dx + dy * dy + dz * dz > R2) continue; const v = cap(x, y, z, a, b, r); if (v < g) g = v; } return g; };

  function cranium(x, y, z) {
    const X = abs(x);
    let vault = ell(X, y, z, VC, VR);
    vault = smin(vault, ell(X, y, z, [0, 1.69, 0.045], [0.063, 0.062, 0.05]), 0.012);              // frontal squama
    vault = smin(vault, ell(X, y, z, [0.035, 1.7, -0.02], [0.034, 0.045, 0.055]), 0.03);            // parietal eminences
    vault = smin(vault, sph(X, y, z, [0.026, 1.708, 0.07], 0.022), 0.03);                            // frontal eminences
    let d = vault;
    d = smin(d, ell(X, y, z, [0, 1.606, -0.012], [0.05, 0.022, 0.066]), 0.012);                    // cranial base
    if (z > 0.015 && y < 1.7 && X < 0.07) {
    d = smin(d, cap(X, y, z, [0.012, 1.671, 0.096], [0.047, 1.668, 0.083], 0.0068), 0.006);       // superciliary arch
    d = smin(d, sph(X, y, z, [0, 1.671, 0.099], 0.0082), 0.008);                                   // glabella
    d = smin(d, ell(X, y, z, [0, 1.642, 0.08], [0.036, 0.028, 0.028]), 0.012);                     // midface
    d = smin(d, ell(X, y, z, [0, 1.612, 0.08], [0.031, 0.027, 0.026]), 0.01);                      // maxilla
    d = smin(d, cap(X, y, z, [0.0, 1.592, 0.091], [0.027, 1.594, 0.05], 0.0068), 0.008);           // alveolar arch
    for (let i = 0; i < 8; i++) {                                                                    // alveolar yokes over the roots
      const s = (i + 0.5) / 8, tx = 0.0035 + 0.0245 * Math.pow(s, 0.85), tz = 0.094 - 0.049 * Math.pow(s, 1.5);
      d = smin(d, cap(X, y, z, [tx, 1.592, tz + 0.002], [tx * 0.95, 1.603 + (i === 2 ? 0.004 : 0), tz + 0.0005], i === 2 ? 0.0022 : 0.0016), 0.006);
    }
    d = smin(d, cap(X, y, z, [0.017, 1.596, 0.097], [0.018, 1.614, 0.096], 0.0018), 0.007);       // canine eminence
    d = smin(d, ell(X, y, z, [0.044, 1.637, 0.074], [0.009, 0.013, 0.008]), 0.008);                // zygomatic body
    d = smin(d, cap(X, y, z, [0.045, 1.645, 0.08], [0.046, 1.667, 0.084], 0.0036), 0.008);       // frontal process of zygomatic
    d = smin(d, cap(X, y, z, [0.046, 1.667, 0.084], [0.044, 1.676, 0.083], 0.0035), 0.008);        // zygomatic process of frontal
    }
    if (z < 0.01) {
    d = smin(d, cone(X, y, z, [0.057, 1.607, -0.022], [0.054, 1.587, -0.017], 0.0075, 0.003), 0.008); // mastoid process
    d = smin(d, sph(X, y, z, [0, 1.636, -0.083], 0.0055), 0.012);                                  // external occipital protuberance
    }
    if (z > -0.02) {
    d = smax(d, -ell(X, y, z, [0.08, 1.648, 0.03], [0.016, 0.03, 0.042]), 0.012);                 // temporal fossa
    d = smax(d, -ell(X, y, z, [0.065, 1.632, 0.035], [0.008, 0.012, 0.022]), 0.006);              // infratemporal space under the arch
    d = smax(d, -ell(X, y, z, [0.031, 1.655, 0.106], [0.0175, 0.0165, 0.032]), 0.003);            // orbit
    d = smax(d, -cone(X, y, z, [0.03, 1.655, 0.095], [0.016, 1.65, 0.062], 0.0125, 0.003), 0.004); // orbital cone to the optic canal
    d = smax(d, -cap(X, y, z, [0.021, 1.648, 0.072], [0.037, 1.641, 0.08], 0.0016), 0.001);       // inferior orbital fissure
    d = smax(d, -cap(X, y, z, [0.022, 1.664, 0.074], [0.035, 1.669, 0.078], 0.0013), 0.001);      // superior orbital fissure
    d = smax(d, -sph(X, y, z, [0.021, 1.672, 0.104], 0.0026), 0.0015);                             // supraorbital notch
    d = smax(d, -ell(X, y, z, [0, 1.627, 0.108], [0.0105, 0.017, 0.021]), 0.003);                  // piriform aperture
    d = smax(d, -cap(X, y, z, [0.004, 1.628, 0.1], [0.005, 1.627, 0.07], 0.0085), 0.003);          // nasal cavity
    d = smax(d, -ell(X, y, z, [0.022, 1.619, 0.104], [0.008, 0.008, 0.004]), 0.004);               // canine fossa
    d = smax(d, -sph(X, y, z, [0.029, 1.633, 0.103], 0.0021), 0.0012);                             // infraorbital foramen
    d = smax(d, -sph(X, y, z, [0.049, 1.637, 0.082], 0.0013), 0.0008);                             // zygomaticofacial foramen
    }
    if (z < 0.01) {
    d = smax(d, -cap(X, y, z, [0, 1.57, -0.033], [0, 1.618, -0.033], 0.0145), 0.004);              // foramen magnum
    d = smax(d, -ell(X, y, z, [0.051, 1.613, -0.002], [0.01, 0.006, 0.009]), 0.003);               // mandibular fossa
    d = smax(d, -cap(X, y, z, [0.064, 1.611, -0.016], [0.046, 1.611, -0.016], 0.0034), 0.0015);    // external acoustic meatus
    }
    if (z > -0.02 && y < 1.66) {
    d = smin(d, cap(X, y, z, [0.053, 1.633, 0.066], [0.061, 1.628, 0.006], 0.003), 0.007);        // zygomatic arch, lower edge
    d = smin(d, cap(X, y, z, [0.054, 1.638, 0.06], [0.062, 1.633, 0.012], 0.0025), 0.007);        // zygomatic arch, upper edge
    d = smin(d, cap(X, y, z, [0.0, 1.664, 0.102], [0.0, 1.645, 0.109], 0.0055), 0.008);          // nasal bones (bridge)
    d = smin(d, cap(X, y, z, [0, 1.612, 0.103], [0, 1.611, 0.112], 0.0016), 0.002);               // anterior nasal spine
    d = smin(d, cap(x, y, z, [0, 1.64, 0.098], [0, 1.615, 0.078], 0.001), 0.0015);                // nasal septum (vomer, ethmoid plate)
    d = smin(d, ell(X, y, z, [0.0075, 1.623, 0.09], [0.002, 0.0045, 0.012]), 0.002);               // inferior nasal concha
    d = smin(d, ell(X, y, z, [0.0065, 1.633, 0.085], [0.0018, 0.003, 0.009]), 0.002);              // middle nasal concha
    d = smin(d, cap(X, y, z, [0.045, 1.598, -0.011], [0.041, 1.577, 0.001], 0.0013), 0.002);       // styloid process
    }
    if (abs(vault) < 0.005) {
      d = smin(d, curveDist(x, y, z, RID, 0.0011), 0.0025);                                         // temporal and nuchal lines
      d = smax(d, -curveDist(x, y, z, SUT, 0.0007), 0.0008);                                        // sutures
    }
    return d;
  }

  const MAND = [[0.045, 1.587, 0.014], [0.042, 1.577, 0.04], [0.032, 1.569, 0.064], [0.017, 1.563, 0.082], [0, 1.561, 0.088]];
  function mandible(x, y, z) {
    const X = abs(x);
    let d = 1;
    for (let i = 1; i < MAND.length; i++) {
      const a = MAND[i - 1], b = MAND[i];
      d = smin(d, cap(X, y, z, a, b, 0.0055), 0.006);                                              // base
      d = smin(d, cap(X, y, z, [a[0] - 0.0015, a[1] + 0.0105, a[2]], [b[0] - 0.0015, b[1] + 0.0105, b[2]], 0.0048), 0.006); // alveolar part
    }
    for (let i = 0; i < 8; i++) {                                                                    // alveolar yokes
      const s = (i + 0.5) / 8, tx = 0.0035 + 0.0245 * Math.pow(s, 0.85), tz = 0.092 - 0.049 * Math.pow(s, 1.5);
      d = smin(d, cap(X, y, z, [tx, 1.576, tz + 0.0015], [tx, 1.568, tz + 0.0005], 0.0015), 0.006);
    }
    d = smin(d, cone(X, y, z, [0, 1.557, 0.091], [0.012, 1.557, 0.087], 0.0055, 0.0035), 0.004);  // mental protuberance and tubercle
    d = smin(d, cap(X, y, z, [0.03, 1.571, 0.064], [0.043, 1.596, 0.03], 0.0022), 0.004);          // oblique line
    d = smin(d, ell(X, y, z, [0.046, 1.602, 0.009], [0.0034, 0.019, 0.013]), 0.006);               // ramus
    d = smin(d, cap(X, y, z, [0.045, 1.585, 0.005], [0.05, 1.611, -0.002], 0.0046), 0.005);       // posterior border
    d = smin(d, sph(X, y, z, [0.046, 1.583, 0.007], 0.0052), 0.006);                               // angle of the mandible
    d = smin(d, cone(X, y, z, [0.043, 1.596, 0.022], [0.044, 1.626, 0.026], 0.004, 0.0016), 0.004); // coronoid process
    d = smin(d, cap(X, y, z, [0.049, 1.608, -0.001], [0.05, 1.615, -0.002], 0.0032), 0.003);      // condylar neck
    d = smin(d, ell(X, y, z, [0.05, 1.618, -0.002], [0.0095, 0.0042, 0.0048]), 0.002);             // condylar head
    d = smax(d, -ell(X, y, z, [0.046, 1.619, 0.012], [0.006, 0.0085, 0.008]), 0.003);              // mandibular notch
    d = smax(d, -sph(X, y, z, [0.022, 1.568, 0.074], 0.0018), 0.0011);                             // mental foramen
    d = smax(d, -sph(X, y, z, [0.042, 1.6, 0.012], 0.002), 0.001);                                 // mandibular foramen (inner side)
    return d;
  }

  // ---- teeth: incisors (chisel), canines (cone), premolars (2 cusps), molars (4 cusps) ----
  const TEETH = [];
  for (const up of [1, 0]) for (let i = 0; i < 8; i++) {
    const s = (i + 0.5) / 8, tx = 0.0035 + 0.0245 * Math.pow(s, 0.85), tz = 0.094 - 0.049 * Math.pow(s, 1.5) - (up ? 0 : 0.002);
    const sgn = up ? -1 : 1, base = up ? 1.5905 : 1.5745;                 // crowns point down (upper) or up (lower)
    const tip = base + sgn * (i < 3 ? 0.0115 : 0.0088);
    TEETH.push({ i, up, tx, tz, base, tip, sgn });
  }
  function teeth(x, y, z) {
    const X = abs(x);
    let d = 1;
    for (const t of TEETH) {
      if (abs(X - t.tx) > 0.01 || abs(z - t.tz) > 0.01 || abs(y - (t.base + t.tip) / 2) > 0.014) continue;
      const c = [t.tx, (t.base + t.tip) / 2, t.tz], cy = c[1];
      let v;
      if (t.i < 2) {                                                                                 // incisor
        const w = t.up ? (t.i === 0 ? 0.0043 : 0.0034) : 0.0027;
        v = ell(X, y, z, c, [w, abs(t.tip - t.base) / 2, 0.0023]);
      } else if (t.i === 2) {                                                                        // canine
        v = smin(ell(X, y, z, [c[0], cy - t.sgn * 0.001, c[2]], [0.0036, 0.0042, 0.0034]), cone(X, y, z, [c[0], cy, c[2]], [c[0], t.tip, c[2]], 0.0032, 0.0008), 0.002);
      } else if (t.i < 5) {                                                                          // premolar: buccal and lingual cusps
        v = ell(X, y, z, [c[0], cy - t.sgn * 0.001, c[2]], [0.0038, 0.0036, 0.0034]);
        for (const off of [0.0019, -0.0019]) v = smin(v, sph(X, y, z, [c[0] + off * 0.8, t.tip - t.sgn * 0.0018, c[2] + off * 0.2], 0.0019), 0.0012);
      } else {                                                                                       // molar: four cusps and a fissure
        v = ell(X, y, z, [c[0], cy - t.sgn * 0.001, c[2]], [0.0052, 0.0034, 0.0049]);
        for (const [ox, oz] of [[0.0023, 0.0022], [-0.0023, 0.0022], [0.0023, -0.0022], [-0.0023, -0.0022]]) v = smin(v, sph(X, y, z, [c[0] + ox, t.tip - t.sgn * 0.0019, c[2] + oz], 0.0019), 0.0011);
        v = smax(v, -cap(X, y, z, [c[0], t.tip + t.sgn * 0.0004, c[2] - 0.004], [c[0], t.tip + t.sgn * 0.0004, c[2] + 0.004], 0.0005), 0.0006);
      }
      d = min(d, v);
    }
    return d;
  }

  function surfaceNets(f, lo, hi, h, aoScale) {
    const nx = Math.ceil((hi[0] - lo[0]) / h) + 1, ny = Math.ceil((hi[1] - lo[1]) / h) + 1, nz = Math.ceil((hi[2] - lo[2]) / h) + 1;
    const F = new Float32Array(nx * ny * nz);
    const id = (i, j, k) => i + nx * (j + ny * k);
    // coarse pass every 4 cells; fine samples far from the surface keep the coarse value
    const S = 4, cx = Math.ceil((nx - 1) / S) + 1, cy = Math.ceil((ny - 1) / S) + 1, cz = Math.ceil((nz - 1) / S) + 1;
    const G = new Float32Array(cx * cy * cz);
    for (let k = 0; k < cz; k++) for (let j = 0; j < cy; j++) for (let i = 0; i < cx; i++) G[i + cx * (j + cy * k)] = f(lo[0] + i * S * h, lo[1] + j * S * h, lo[2] + k * S * h);
    const far = S * h * 1.05;
    for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
      const g = G[Math.round(i / S) + cx * (Math.round(j / S) + cy * Math.round(k / S))];
      F[id(i, j, k)] = abs(g) > far ? g : f(lo[0] + i * h, lo[1] + j * h, lo[2] + k * h);
    }
    const V = new Int32Array(nx * ny * nz).fill(-1), pos = [];
    const E = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
    const C = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]];
    const val = new Float32Array(8);
    for (let k = 0; k < nz - 1; k++) for (let j = 0; j < ny - 1; j++) for (let i = 0; i < nx - 1; i++) {
      let neg = 0;
      for (let c = 0; c < 8; c++) { val[c] = F[id(i + C[c][0], j + C[c][1], k + C[c][2])]; if (val[c] < 0) neg++; }
      if (neg === 0 || neg === 8) continue;
      let sx = 0, sy = 0, sz = 0, n = 0;
      for (const [a, b] of E) {
        if ((val[a] < 0) === (val[b] < 0)) continue;
        const t = val[a] / (val[a] - val[b]);
        sx += C[a][0] + (C[b][0] - C[a][0]) * t; sy += C[a][1] + (C[b][1] - C[a][1]) * t; sz += C[a][2] + (C[b][2] - C[a][2]) * t; n++;
      }
      V[id(i, j, k)] = pos.length / 3;
      pos.push(lo[0] + (i + sx / n) * h, lo[1] + (j + sy / n) * h, lo[2] + (k + sz / n) * h);
    }
    // normals from the field gradient; vertices pulled onto the surface; cavity shading from the field along the normal
    const e = h * 0.3, nv = pos.length / 3;
    const nor = new Float32Array(nv * 3), col = new Float32Array(nv);
    for (let v = 0; v < nv; v++) {
      let x = pos[3 * v], y = pos[3 * v + 1], z = pos[3 * v + 2];
      const f0 = f(x, y, z);
      let gx = f(x + e, y, z) - f0, gy = f(x, y + e, z) - f0, gz = f(x, y, z + e) - f0;
      let gl = sq(gx * gx + gy * gy + gz * gz) || 1;
      gx /= gl; gy /= gl; gz /= gl;
      const d0 = max(-h * 0.5, min(h * 0.5, f0));
      x -= gx * d0; y -= gy * d0; z -= gz * d0;
      pos[3 * v] = x; pos[3 * v + 1] = y; pos[3 * v + 2] = z;
      nor[3 * v] = gx; nor[3 * v + 1] = gy; nor[3 * v + 2] = gz;
      const dd = 1.8 * aoScale;
      col[v] = max(0, min(1, f(x + gx * dd, y + gy * dd, z + gz * dd) / dd));
    }
    const idx = [];
    const quad = (a, b, c, d, flip) => { if (a < 0 || b < 0 || c < 0 || d < 0) return; if (flip) idx.push(a, c, b, a, d, c); else idx.push(a, b, c, a, c, d); };
    for (let k = 1; k < nz - 1; k++) for (let j = 1; j < ny - 1; j++) for (let i = 1; i < nx - 1; i++) {
      const p0 = F[id(i, j, k)] < 0;
      if (p0 !== (F[id(i + 1, j, k)] < 0)) quad(V[id(i, j - 1, k - 1)], V[id(i, j, k - 1)], V[id(i, j, k)], V[id(i, j - 1, k)], !p0);
      if (p0 !== (F[id(i, j + 1, k)] < 0)) quad(V[id(i - 1, j, k - 1)], V[id(i - 1, j, k)], V[id(i, j, k)], V[id(i, j, k - 1)], !p0);
      if (p0 !== (F[id(i, j, k + 1)] < 0)) quad(V[id(i - 1, j - 1, k)], V[id(i, j - 1, k)], V[id(i, j, k)], V[id(i - 1, j, k)], !p0);
    }
    const P = new Float32Array(pos.length);
    for (let v = 0; v < pos.length; v += 3) { P[v] = pos[v] - Jx; P[v + 1] = pos[v + 1] - Jy; P[v + 2] = pos[v + 2] - Jz; }
    return { pos: P, nor, col, idx: new Uint32Array(idx) };
  }
  return {
    cranium: surfaceNets(cranium, [-0.083, 1.566, -0.106], [0.083, 1.763, 0.123], 0.0018, 0.004),
    mandible: surfaceNets(mandible, [-0.064, 1.546, -0.012], [0.064, 1.632, 0.1], 0.0014, 0.003),
    teeth: surfaceNets(teeth, [-0.034, 1.563, 0.036], [0.034, 1.6, 0.1], 0.0008, 0.0015),
  };
}
