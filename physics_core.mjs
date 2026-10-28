// Static equilibrium of a posed body.
//  * vertical floor reactions: the body, held rigid by its muscles, sinks a little into a soft floor
//    (stiffness by tissue under each contact) — the classic elastic-foundation solution;
//  * friction at the contacts, hand grips, the scapula pressing on the ribcage, muscle activations and
//    small "reserve" torques (ligaments, joint surfaces) come from one quadratic program:
//    every joint axis and the whole body must be in equilibrium, total muscle effort is minimal;
//  * passive tissues: spinal discs and ligaments as rotational springs with stiff end-range stops,
//    and the passive pull of the long muscles that a pose stretches.
import * as THREE from 'three';
import { MUSCLE, PASSIVE, SPINE_PASSIVE, JOINT_STOPS, CONTACT_K, FRICTION } from './physics_params.mjs';

export const G = 9.81;

// Rigid-tendon approximation: the supplied phi is the fibre fraction at the
// standing reference. It is not silently replaced by a different parameter.
export function fiberLengthRatio(length, referenceLength, phi) {
  if (![length, referenceLength, phi].every(Number.isFinite) || length < 0
      || referenceLength <= 0 || phi <= 0 || phi > 1) throw new Error('Invalid muscle length/reference/fibre fraction');
  return 1 + (length / referenceLength - 1) / phi;
}
export function activeForceLengthFactor(length) {
  if (!Number.isFinite(length)) throw new Error('Nonfinite normalized fibre length');
  // A shortened unit can be slack under the rigid-tendon approximation. A
  // negative fibre length is not a small positive length with finite capacity.
  return length <= 0 ? 0 : Math.exp(-((length - 1) ** 2) / 0.45);
}

// The 20 face normals of an icosahedron. Its vertices lie on the requested
// force-radius sphere; the inscribed approximation never exceeds that bound.
const PHI = (1 + Math.sqrt(5)) / 2;
const FORCE_BALL_INRADIUS = Math.sqrt((5 + 2 * Math.sqrt(5)) / 15);
export function forceBallFacets(radius) {
  if (!Number.isFinite(radius) || radius < 0) throw new Error('Invalid force capacity');
  const normals = [];
  for (const a of [-1, 1]) for (const b of [-1, 1]) {
    for (const c of [-1, 1]) normals.push([a, b, c]);
    normals.push([0, a * PHI, b / PHI], [a * PHI, b / PHI, 0], [a / PHI, 0, b * PHI]);
  }
  return normals.map(v => ({ normal: new THREE.Vector3(...v).normalize(), bound: radius * FORCE_BALL_INRADIUS }));
}

// A regular polygon inscribed in the isotropic Coulomb circle. The former
// four-face diamond reduced available traction by 29% along a world diagonal.
// Sixteen faces keep every admissible force inside the same unchanged mu*N
// circle, with at most 1-cos(pi/16) = 1.92% directional loss.
export const COULOMB_FACETS = 16;
export function frictionFacets(mu) {
  const bound = -mu * Math.cos(Math.PI / COULOMB_FACETS);
  return Array.from({ length: COULOMB_FACETS }, (_, k) => {
    const angle = (k + 0.5) * 2 * Math.PI / COULOMB_FACETS;
    return { normal: bound, x: Math.cos(angle), z: Math.sin(angle) };
  });
}

// Energy of one existing spinal spring, in joules. Parameters retain their
// original units (N*m/degree and degrees); no stiffness or range is retuned.
export function spineJointEnergy(q, parameter) {
  const D = Math.PI / 180, e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
  const angles = [e.x, e.y, e.z];
  let energy = 0;
  for (let a = 0; a < 3; a++) {
    const angle = angles[a];
    energy += 0.5 * parameter.k[a] * angle * angle / D;
    const limit = (a === 0 ? (angle >= 0 ? parameter.flex : parameter.ext) : parameter.side) * D;
    const span = 0.4 * limit, u = (Math.abs(angle) - 0.6 * limit) / span;
    if (u > 0) energy += parameter.Mlim * span * (Math.expm1(4 * u) / 4 - u) / Math.expm1(4);
  }
  return energy;
}

export function spineJointTorque(q, parameter) {
  const delta = 1e-6, torque = new Float64Array(3);
  for (let a = 0; a < 3; a++) {
    const axis = new THREE.Vector3().setComponent(a, 1);
    const plus = new THREE.Quaternion().setFromAxisAngle(axis, delta).multiply(q);
    const minus = new THREE.Quaternion().setFromAxisAngle(axis, -delta).multiply(q);
    // The solver's axes are fixed in the parent frame. Euler-angle derivatives
    // cannot be assigned directly to these torques in a mixed rotation.
    torque[a] = -(spineJointEnergy(plus, parameter) - spineJointEnergy(minus, parameter)) / (2 * delta);
  }
  return torque;
}

export function makePhysics(A, opts = {}) {
  const { V, S, SEG, pt, LANDMARKS, MASS, strandPath, pathLength } = A;
  const BODY = opts.mass ?? 70;
  if (!Number.isFinite(BODY) || BODY <= 0) throw new Error('Body mass must be finite and positive');
  const BW = BODY * G;
  const children = {};
  for (const s of SEG) children[s.name] = [];
  for (const s of SEG) if (s.parent) children[s.parent].push(s.name);
  const subtree = name => { const out = new Set(); const st = [name]; while (st.length) { const n = st.pop(); out.add(n); for (const c of children[n]) st.push(c); } return out; };
  const ALL = new Set(SEG.map(s => s.name));
  const basis = (q, i) => [V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)][i].applyQuaternion(q);

  // ---------- degrees of freedom that must be in equilibrium ----------
  const DOF = [];
  for (let i = 0; i < 3; i++) DOF.push({ id: 'root_t' + 'xyz'[i], joint: 'root', kind: 'trans', axis: () => [V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)][i], moved: ALL });
  for (let i = 0; i < 3; i++) DOF.push({ id: 'root_r' + 'xyz'[i], joint: 'root', kind: 'rot', center: W => W.pelvis.p, axis: () => [V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)][i], moved: ALL });
  const SPINE = [['ls', 'lumA', 'pelvis'], ['l34', 'lumB', 'lumA'], ['tl', 'thA', 'lumB'], ['t89', 'thB', 'thA'], ['t45', 'thC', 'thB'], ['ct', 'neckA', 'thC'], ['c45', 'neckB', 'neckA'], ['ao', 'head', 'neckB']];
  // spine: 0 = about the parent's X (+ flexion), 1 = about Y (axial rotation), 2 = about Z (side bend)
  for (const [j, child, parent] of SPINE) for (let a = 0; a < 3; a++) DOF.push({ id: j + '_' + 'xyz'[a], joint: j, comp: a, kind: 'rot', child, center: W => W[child].p, axis: W => basis(W[parent].q, a), moved: subtree(child) });
  for (const sd of ['L', 'R']) {
    const n = b => b + '_' + sd;
    for (let a = 0; a < 3; a++) DOF.push({ id: 'sc_' + sd + '_' + 'xyz'[a], joint: 'sc_' + sd, comp: a, kind: 'rot', center: W => W[n('clav')].p, axis: W => basis(W.thC.q, a), moved: subtree(n('clav')) });
    // shoulder: about the scapula's X (+ extension), Y (rotation), Z (+ abduction on the left)
    for (let a = 0; a < 3; a++) DOF.push({ id: 'gh_' + sd + '_' + 'xyz'[a], joint: 'gh_' + sd, comp: a, kind: 'rot', center: W => W[n('hum')].p, axis: W => basis(W[n('scap')].q, a), moved: subtree(n('hum')) });
    // elbow: + extension; forearm: + pronation (radius turns round the ulna)
    DOF.push({ id: 'el_' + sd + '_x', joint: 'el_' + sd, comp: 0, kind: 'rot', center: W => W[n('fore')].p, axis: W => basis(W[n('hum')].q, 0), moved: subtree(n('fore')) });
    { const mv = subtree(n('fore')); for (const s of subtree(n('ulna'))) mv.delete(s);
      DOF.push({ id: 'el_' + sd + '_p', joint: 'el_' + sd, comp: 1, kind: 'rot', center: W => W[n('fore')].p, axis: W => W[n('hand')].p.clone().sub(W[n('fore')].p).normalize(), moved: mv }); }
    // wrist: about the forearm's X (deviation) and Z (flexion/extension)
    DOF.push({ id: 'wr_' + sd + '_x', joint: 'wr_' + sd, comp: 0, kind: 'rot', center: W => W[n('hand')].p, axis: W => basis(W[n('fore')].q, 0), moved: subtree(n('hand')) });
    DOF.push({ id: 'wr_' + sd + '_z', joint: 'wr_' + sd, comp: 2, kind: 'rot', center: W => W[n('hand')].p, axis: W => basis(W[n('fore')].q, 2), moved: subtree(n('hand')) });
    // hip: about the pelvis X (+ extension), Y (+ outward rotation on the left), Z (+ abduction on the left)
    for (let a = 0; a < 3; a++) DOF.push({ id: 'hip_' + sd + '_' + 'xyz'[a], joint: 'hip_' + sd, comp: a, kind: 'rot', center: W => W[n('thigh')].p, axis: W => basis(W.pelvis.q, a), moved: subtree(n('thigh')) });
    // knee: + flexion; ankle: + plantar flexion, and inversion/eversion about the foot's long axis
    DOF.push({ id: 'kn_' + sd + '_x', joint: 'kn_' + sd, comp: 0, kind: 'rot', center: W => W[n('shank')].p, axis: W => basis(W[n('thigh')].q, 0), moved: subtree(n('shank')) });
    DOF.push({ id: 'an_' + sd + '_x', joint: 'an_' + sd, comp: 0, kind: 'rot', center: W => W[n('foot')].p, axis: W => basis(W[n('shank')].q, 0), moved: subtree(n('foot')) });
    DOF.push({ id: 'an_' + sd + '_z', joint: 'an_' + sd, comp: 2, kind: 'rot', center: W => W[n('foot')].p, axis: W => basis(W[n('foot')].q, 2), moved: subtree(n('foot')) });
  }
  const dofIndex = Object.fromEntries(DOF.map((d, k) => [d.id, k]));

  // virtual displacement of a point attached to segment seg for dof k
  const dpdq = (k, seg, P, ax, c) => {
    const d = DOF[k];
    if (!d.moved.has(seg)) return null;
    if (d.kind === 'trans') return ax.clone();
    return ax.clone().cross(P.clone().sub(c));
  };
  const pairedDisplacement = (k, segA, segB, P, ax, c) => {
    const a = dpdq(k, segA, P, ax, c), b = dpdq(k, segB, P, ax, c);
    return V(0, 0, 0).add(a || V(0, 0, 0)).sub(b || V(0, 0, 0));
  };
  const rotW = (W, k, ang, ax, c) => {
    const d = DOF[k];
    const R = new THREE.Quaternion().setFromAxisAngle(ax, ang);
    const W2 = {};
    for (const nme in W) {
      if (!d.moved.has(nme)) { W2[nme] = W[nme]; continue; }
      if (d.kind === 'trans') { W2[nme] = { q: W[nme].q, p: W[nme].p.clone().addScaledVector(ax, ang) }; continue; }
      W2[nme] = { q: R.clone().multiply(W[nme].q), p: W[nme].p.clone().sub(c).applyQuaternion(R).add(c) };
    }
    return W2;
  };

  // ---------- muscles ----------
  const INST = A.strandInstances().filter(i => i.kind === 'muscle');
  // expected sign of the lever on the left side ('+' / '-'), for hinge dofs; the right side is checked against a reference muscle
  const GUARD_DEF = {
    el_x: { ref: 'tricLat', '+': ['tricLong', 'tricLat', 'tricMed', 'anconeus'], '-': ['bicLong', 'bicShort', 'brachialis', 'brachiorad', 'pronTeres'] },
    kn_x: { ref: 'vastLat', '+': ['bicFemL', 'bicFemS', 'semiT', 'semiM', 'gracilis', 'sartorius', 'gastroM', 'gastroL', 'popliteus', 'plantaris'], '-': ['rectFem', 'vastLat', 'vastMed', 'vastInt'] },
    wr_z: { ref: 'wristExt', '+': ['wristExt'], '-': ['wristFlex', 'fingerFlex', 'palmaris'] },
    an_x: { ref: 'soleus', '+': ['gastroM', 'gastroL', 'soleus', 'plantaris', 'tibPost', 'fibL', 'fibB', 'fhl', 'fdl'], '-': ['tibAnt', 'edl', 'ehl'] },
    // hip: only muscles that keep their action at any flexion (the adductors genuinely switch from flexion to extension)
    hip_x: { ref: 'gluteMax', '+': ['gluteMax', 'bicFemL', 'semiT', 'semiM'], '-': ['psoas', 'iliacus', 'rectFem', 'sartorius', 'tfl'] },
  };
  const GUARDS = [];
  const unitStrands = {};
  for (const i of INST) (unitStrands[i.unit + '_' + i.side] ||= []).push(i);
  const TOUCH = INST.map(i => new Set(i.pts.map(p => p.seg).concat(i.wraps.map(w => w.seg))));
  const FMAX = INST.map(i => {
    const P = MUSCLE[i.unit];
    if (!P) throw new Error('no muscle params for ' + i.unit);
    const arr = unitStrands[i.unit + '_' + i.side];
    const w = s => (s.r || 0.005) ** 2;
    return P.F * w(i) / arr.reduce((a, s) => a + w(s), 0);
  });
  const W_STAND = A.placeW(A.STAND.local, A.STAND.rootQ);
  const L_STAND = INST.map(i => pathLength(strandPath(W_STAND, i)));
  // fibre length relative to standing; force-length factor (Gaussian, width as in Thelen 2003)
  const fiberLen = (j, L) => fiberLengthRatio(L, L_STAND[j], MUSCLE[INST[j].unit].phi);
  const expRise = (u, k) => (Math.exp(k * u) - 1) / (Math.exp(k) - 1);
  // passive pull of stretched long muscles: starts at slack strain e0 of the muscle-tendon unit,
  // reaches frac*Fmax at the strain eLim (end of a flexible person's range)
  const passiveForce = (j, L) => {
    const p = PASSIVE[INST[j].unit];
    if (!p) return 0;
    const e = L / L_STAND[j] - 1;
    if (e <= p.e0) return 0;
    return Math.min(0.8, p.frac * expRise((e - p.e0) / (p.eLim - p.e0), 3)) * FMAX[j];
  };

  // Use the anatomy module's trace for both lever lengths and joint reactions.
  // A wrap contact belongs to the bone it wraps; there is no second geometry
  // algorithm here that could drift when an attachment or pulley is improved.
  const ownedPath = A.ownedStrandPath;
  // net pull of a strand of tension f on the set of segments `set`
  function pullOn(path, set, f) {
    const F = V(0, 0, 0);
    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1], b = path[i];
      const ia = set.has(a.seg), ib = set.has(b.seg);
      if (ia === ib) continue;
      const u = (ia ? b.p.clone().sub(a.p) : a.p.clone().sub(b.p)).normalize();
      F.addScaledVector(u, f);
    }
    return F;
  }

  {
    const W0 = A.placeW(A.STAND.local, A.STAND.rootQ);
    const lever = (j, k) => { const d = DOF[k], ax = d.axis(W0), c = d.center(W0), e = 0.3 * Math.PI / 180; return -(pathLength(strandPath(rotW(W0, k, e, ax, c), INST[j])) - pathLength(strandPath(rotW(W0, k, -e, ax, c), INST[j]))) / (2 * e); };
    for (const [key, def] of Object.entries(GUARD_DEF)) for (const sd of ['L', 'R']) {
      const [jn, ax] = key.split('_'); const k = dofIndex[jn + '_' + sd + '_' + ax];
      const refJ = INST.findIndex(m => m.unit === def.ref && m.side === sd);
      const expRef = def['+'].includes(def.ref) ? 1 : -1;
      const flip = Math.sign(lever(refJ, k)) === expRef ? 1 : -1;
      for (const sg of ['+', '-']) { const js = INST.map((m, j) => j).filter(j => INST[j].side === sd && def[sg].includes(INST[j].unit)); GUARDS.push({ k, sign: (sg === '+' ? 1 : -1) * flip, js }); }
    }
  }

  // ---------- contacts ----------
  function contactsOf(W, thr = 0.004) {
    if (!Number.isFinite(thr) || thr < 0) throw new Error('Contact threshold must be finite and nonnegative');
    const out = [];
    LANDMARKS.forEach((l, li) => {
      const P = pt(W, l.seg, l.pc);
      if (P.y < thr) out.push({ id: 'landmark:' + li, source: 'landmark', region: l.region, side: l.side, seg: l.seg, tag: l.tag, li, P: V(P.x, 0, P.z), h: Math.max(0, P.y), k: CONTACT_K[l.region] || 1.5, mu: FRICTION[l.region] ?? 0.6 });
    });
    return out;
  }
  const point = (value, label) => {
    const xyz = Array.isArray(value) ? value : value && [value.x, value.y, value.z];
    if (!xyz || xyz.length !== 3 || !xyz.every(Number.isFinite)) throw new Error('Invalid contact point/vector: ' + label);
    return V(...xyz);
  };
  const cap = (value, label) => {
    if (value === undefined) return undefined;
    if (!Number.isFinite(value) || value < 0) throw new Error('Invalid contact capacity: ' + label);
    return value;
  };
  function floorContacts(W, override, threshold) {
    if (override === undefined) return contactsOf(W, threshold);
    if (!Array.isArray(override)) throw new Error('Floor contacts override must be an array');
    const maxHeight = threshold ?? .004;
    if (!Number.isFinite(maxHeight) || maxHeight < 0) throw new Error('Contact threshold must be finite and nonnegative');
    const ids = new Set();
    return override.map(c => {
      if (!c || typeof c.id !== 'string' || !c.id.length || ids.has(c.id) || typeof c.seg !== 'string' || !Object.hasOwn(S, c.seg))
        throw new Error('Floor contact needs a unique id and known segment');
      ids.add(c.id);
      const P = point(c.P, c.id);
      if (Math.abs(P.y) > 1e-6) throw new Error('Floor contact point must lie on y=0: ' + c.id);
      const h = c.h ?? 0, k = c.k ?? CONTACT_K[c.region] ?? 1.5, mu = c.mu ?? FRICTION[c.region] ?? .6;
      if (!Number.isFinite(h) || h < 0 || h > maxHeight + 1e-6 || !Number.isFinite(k) || k <= 0 || !Number.isFinite(mu) || mu < 0)
        throw new Error('Invalid floor contact height, stiffness or friction: ' + c.id);
      return { ...c, P: V(P.x, 0, P.z), h, k, mu, maxNormalN: cap(c.maxNormalN, c.id), li: c.id };
    });
  }
  function internalContacts(W, legacy, additional) {
    if (!Array.isArray(legacy) || !Array.isArray(additional)) throw new Error('Internal contacts/grips must be arrays');
    const ids = new Set();
    return legacy.map((g, index) => ({ ...g, id: g.id ?? 'legacy-grip:' + (g.side ?? index),
      segA: g.hand, segB: g.shank, kind: g.from ? 'pull' : 'bilateral',
      mu: g.cone ?? 1, maxComponentN: (g.max ?? .8) * BW, legacy: true }))
      .concat(additional.map(g => ({ ...g, legacy: false }))).map(g => {
        if (!g || typeof g.id !== 'string' || !g.id.length || ids.has(g.id)
            || typeof g.segA !== 'string' || typeof g.segB !== 'string'
            || !Object.hasOwn(S, g.segA) || !Object.hasOwn(S, g.segB) || g.segA === g.segB)
          throw new Error('Internal contact needs a unique id and two distinct known segments');
        ids.add(g.id);
        if (!['bilateral', 'pull', 'unilateral'].includes(g.kind)) throw new Error('Invalid internal contact kind: ' + g.id);
        const P = point(g.P, g.id), out = { ...g, P, hand: g.segA, shank: g.segB };
        for (const key of ['maxForceN', 'maxNormalN', 'maxComponentN']) out[key] = cap(g[key], g.id + ':' + key);
        if (g.kind === 'bilateral' && g.maxNormalN !== undefined)
          throw new Error('Bilateral contact has no signed normal capacity: ' + g.id);
        if (g.kind !== 'bilateral') {
          if (g.normal !== undefined) out.normal = point(g.normal, g.id + ':normal');
          else if (typeof g.from === 'string' && Object.hasOwn(S, g.from)) out.normal = P.clone().sub(W[g.from].p);
          else throw new Error('Unilateral/pull contact needs a normal or known from segment: ' + g.id);
          if (out.normal.lengthSq() < 1e-16) throw new Error('Internal contact normal must be nonzero: ' + g.id);
          out.normal.normalize();
          out.mu = g.mu ?? 0;
          if (!Number.isFinite(out.mu) || out.mu < 0) throw new Error('Invalid internal contact friction: ' + g.id);
          const helper = Math.abs(out.normal.y) < .9 ? V(0, 1, 0) : V(1, 0, 0);
          out.tangent1 = helper.cross(out.normal).normalize();
          out.tangent2 = out.normal.clone().cross(out.tangent1).normalize();
        }
        return out;
      });
  }
  // elastic foundation: compression d0 + a*x + b*z - h under a rigid body; balance of weight and its moments
  function verticalLoads(cs, com) {
    let act = cs.map((c, i) => i);
    for (let it = 0; it < 40 && act.length; it++) {
      const M = [[1e-9, 0, 0], [0, 1e-9, 0], [0, 0, 1e-9]], rhs = [BW, BW * com.x, BW * com.z];
      for (const i of act) { const c = cs[i], v = [1, c.P.x, c.P.z]; for (let r = 0; r < 3; r++) { for (let s = 0; s < 3; s++) M[r][s] += c.k * v[r] * v[s]; rhs[r] += c.k * c.h * v[r]; } }
      const x = solve3(M, rhs);
      const N = cs.map((c, i) => (act.includes(i) ? c.k * (x[0] + x[1] * c.P.x + x[2] * c.P.z - c.h) : 0));
      const nxt = act.filter(i => N[i] > 1e-9);
      if (nxt.length === act.length) return N.map(v => Math.max(0, v));
      act = nxt;
    }
    return cs.map(() => 0);
  }
  // scapula pressing on the ribcage: medial border near the spine of the scapula, and the inferior angle
  const ST_PTS = [[0.078, 1.385, -0.106], [0.091, 1.252, -0.104]];
  const THORAX_C = V(0, 1.30, -0.005), THORAX_R = V(0.15, 0.2, 0.115);
  function stContacts(W) {
    const out = [];
    for (const sd of ['L', 'R']) for (const p0 of ST_PTS) {
      const P = pt(W, 'scap_' + sd, V(sd === 'R' ? -p0[0] : p0[0], p0[1], p0[2]));
      const local = P.clone().sub(W.thB.p).applyQuaternion(W.thB.q.clone().invert()).add(S.thB.J);
      const n = local.clone().sub(THORAX_C).divide(THORAX_R).divide(THORAX_R).normalize().applyQuaternion(W.thB.q).normalize();
      out.push({ side: sd, P, n, segA: 'scap_' + sd, segB: 'thB' });
    }
    return out;
  }

  // ---------- passive spine: discs and ligaments as springs with end-range stops ----------
  function spinePassive(local) {
    const out = {};
    for (const [j, child] of SPINE) {
      const torque = spineJointTorque(local[child], SPINE_PASSIVE[j]);
      for (let a = 0; a < 3; a++) out[j + '_' + 'xyz'[a]] = torque[a];
    }
    return out;
  }

  // ---------- end of range of the limb joints: capsule, ligaments, bone or soft tissue meeting soft tissue ----------
  // each stop is a potential U(angle); its generalized force on any dof is -dU/dq by finite differences, so no sign conventions are needed
  const unitV = v => v.clone().normalize();
  const angleOf = {
    // plantar flexion of the ankle: angle between the shank and the foot, relative to standing
    ankle: (W, sd) => { const s = W['shank_' + sd].p.clone().sub(W['foot_' + sd].p), f = W['toes_' + sd].p.clone().sub(W['foot_' + sd].p); return s.angleTo(f); },
    // flexion of a hinge, signed by the side the distal segment bends to
    knee: (W, sd) => { const a = unitV(W['shank_' + sd].p.clone().sub(W['thigh_' + sd].p)), b = unitV(W['foot_' + sd].p.clone().sub(W['shank_' + sd].p)); const ax = V(1, 0, 0).applyQuaternion(W['thigh_' + sd].q); return Math.atan2(a.clone().cross(b).dot(ax), a.dot(b)); },
    elbow: (W, sd) => { const a = unitV(W['fore_' + sd].p.clone().sub(W['hum_' + sd].p)), b = unitV(W['hand_' + sd].p.clone().sub(W['fore_' + sd].p)); const ax = V(1, 0, 0).applyQuaternion(W['hum_' + sd].q); return Math.atan2(a.clone().cross(b).dot(ax), a.dot(b)); },
    wrist: (W, sd) => { const a = unitV(W['hand_' + sd].p.clone().sub(W['fore_' + sd].p)), b = unitV(W['f3a_' + sd].p.clone().sub(W['hand_' + sd].p)); const ax = V(0, 0, 1).applyQuaternion(W['fore_' + sd].q); return Math.atan2(a.clone().cross(b).dot(ax), a.dot(b)); },
  };
  const ANG0 = {}, SGN = {};
  {
    const W0 = W_STAND;
    for (const sd of ['L', 'R']) for (const j of Object.keys(angleOf)) {
      ANG0[j + sd] = angleOf[j](W0, sd);
      // which way is "positive" (flexion, extension, plantar flexion as named in JOINT_STOPS): test a small known motion
      const L = {}; for (const k in A.STAND.local) L[k] = A.STAND.local[k].clone();
      const seg = { ankle: 'foot', knee: 'shank', elbow: 'fore', wrist: 'hand' }[j] + '_' + sd;
      const R = sd === 'R' ? A.mirrorQ : (q => q);
      const test = { ankle: A.qAx(A.AX.X, 10), knee: A.qAx(A.AX.X, 10), elbow: A.qAx(A.AX.X, -10), wrist: A.qAx(A.AX.Z, 10) }[j];
      L[seg] = R(test);
      SGN[j + sd] = Math.sign(angleOf[j](A.placeW(L, A.STAND.rootQ), sd) - ANG0[j + sd]) || 1;
    }
  }
  const D2R = Math.PI / 180;
  // energy of one stop: moment rises like expRise(u, 4) from the start of the stop to its end, where it reaches M
  const stopEnergy = (th, st) => { const a = st.from * D2R, b = st.to * D2R; if (th <= a) return 0; const u = (th - a) / (b - a), k = 4; return st.M * (b - a) * ((Math.exp(k * u) - 1) / k - u) / (Math.exp(k) - 1); };
  let STOPS = JOINT_STOPS; // o.stops of the running solve replaces the stops of the named joints (a pose that needs a larger range)
  const stopsEnergy = W => {
    let U = 0;
    for (const sd of ['L', 'R']) for (const [j, list] of Object.entries(STOPS)) {
      const th = SGN[j + sd] * (angleOf[j](W, sd) - ANG0[j + sd]);
      for (const st of list) U += stopEnergy(st.dir > 0 ? th : -th, st);
    }
    return U;
  };
  const STOP_DOFS = DOF.map((d, k) => k).filter(k => /^(el|wr|kn|an)_/.test(DOF[k].joint));
  function jointStops(W) {
    const out = {}, e = 0.2 * D2R;
    if (stopsEnergy(W) === 0) return out;
    for (const k of STOP_DOFS) {
      const d = DOF[k], ax = d.axis(W), c = d.center(W);
      const q = -(stopsEnergy(rotW(W, k, e, ax, c)) - stopsEnergy(rotW(W, k, -e, ax, c))) / (2 * e);
      if (Math.abs(q) > 1e-6) out[d.id] = q;
    }
    return out;
  }

  // ---------- assemble and solve ----------
  // o.rootP (or conf.rootP): world position of the pelvis; without it the pose is placed as the page places it (placeW)
  // o.stops: {joint: [stops]} replaces JOINT_STOPS of those joints for this solve (the handstand needs the wrist beyond 85°)
  // o.warm: the .warm of a previous result (a neighbouring frame) — starting point and duals for the QP, matched by identity
  // o.smooth, o.prevAct: small extra cost w·Fmax/1000·(a − a_prev)²/2 per strand, pulling activations towards the previous
  //   frame (only the sharing between redundant muscles is affected; equilibrium is still exact). Off unless o.smooth > 0.
  function solve(conf, o = {}) {
    const rootP = o.rootP || conf.rootP;
    const W = rootP ? A.computeWorld(conf.local, conf.rootQ, rootP) : A.placeW(conf.local, conf.rootQ);
    const ND = DOF.length;
    const frames = DOF.map(d => ({ ax: d.axis(W), c: d.center ? d.center(W) : null }));
    // muscle moment arms: central differences of path length (wrapping included)
    const delta = 0.3 * Math.PI / 180;
    const Lm = INST.map(i => pathLength(strandPath(W, i)));
    const RLength = INST.map(() => new Float64Array(ND));
    for (let k = 0; k < ND; k++) {
      const d = DOF[k];
      if (d.joint === 'root') continue;
      const cross = [];
      INST.forEach((m, j) => { let inn = false, out = false; for (const s of TOUCH[j]) { if (d.moved.has(s)) inn = true; else out = true; } if (inn && out) cross.push(j); });
      if (!cross.length) continue;
      const Wp = rotW(W, k, delta, frames[k].ax, frames[k].c), Wm = rotW(W, k, -delta, frames[k].ax, frames[k].c);
      for (const j of cross) RLength[j][k] = -(pathLength(strandPath(Wp, INST[j])) - pathLength(strandPath(Wm, INST[j]))) / (2 * delta);
    }
    // Derive the actual moment of unit tension from the same owned force path
    // used for joint reactions. A heuristic wrap is not an exact geodesic, so
    // -dL/dq can differ from this force moment; retain that difference explicitly.
    const paths = INST.map(i => ownedPath(W, i));
    const unitLoads = paths.map(path => {
      const loads = new Map();
      const add = (node, force) => {
        if (!loads.has(node.seg)) loads.set(node.seg, { F: V(0, 0, 0), M: V(0, 0, 0) });
        const load = loads.get(node.seg); load.F.add(force); load.M.add(node.p.clone().cross(force));
      };
      for (let i = 1; i < path.length; i++) {
        const force = path[i].p.clone().sub(path[i - 1].p).normalize();
        add(path[i - 1], force); add(path[i], force.clone().negate());
      }
      return loads;
    });
    const R = unitLoads.map(loads => Float64Array.from(DOF, (d, k) => {
      if (d.joint === 'root') return 0;
      let work = 0;
      for (const [segment, load] of loads) if (d.moved.has(segment))
        work += (d.kind === 'trans' ? load.F : load.M.clone().sub(frames[k].c.clone().cross(load.F))).dot(frames[k].ax);
      return work;
    }));
    const violations = INST.map(() => []);
    for (const guard of GUARDS) for (const j of guard.js) {
      const lever = R[j][guard.k];
      if (Math.abs(lever) > 1e-6 && Math.sign(lever) !== guard.sign)
        violations[j].push({ dof: DOF[guard.k].id, expectedSign: guard.sign, momentArmM: lever,
          geometricMomentArmM: RLength[j][guard.k] });
    }
    const lt = Lm.map((L, j) => fiberLen(j, L));
    const capacity = FMAX.map((F, j) => F * activeForceLengthFactor(lt[j]));
    const muscleQuality = INST.map((instance, j) => ({ id: instance.id, unit: instance.unit, side: instance.side,
      status: violations[j].length ? 'quarantined' : lt[j] <= 0 ? 'slack' : 'estimated',
      quarantined: violations[j].length > 0, available: !violations[j].length && capacity[j] > 0,
      violations: violations[j], normalizedFiberLength: lt[j],
      activeForceLengthFactor: activeForceLengthFactor(lt[j]), slackFiber: lt[j] <= 0 }));
    // Invalid anatomical paths are quarantined as a whole, including passive
    // tension. Zeroing one moment while retaining its actual force would break
    // mechanical equilibrium. The mesh and Fmax remain visible and unchanged.
    const FE = capacity.map((force, j) => muscleQuality[j].quarantined ? 0 : force);
    const FP = Lm.map((L, j) => muscleQuality[j].quarantined ? 0 : passiveForce(j, L));
    let maxDerivativeMismatchM = 0, maxDerivativeMismatch = null;
    for (let j = 0; j < INST.length; j++) {
      let strandMismatch = 0;
      for (let k = 0; k < ND; k++) {
        const differenceM = Math.abs(R[j][k] - RLength[j][k]);
        strandMismatch = Math.max(strandMismatch, differenceM);
        if (differenceM > maxDerivativeMismatchM) {
          maxDerivativeMismatchM = differenceM;
          maxDerivativeMismatch = { id: INST[j].id, unit: INST[j].unit, side: INST[j].side,
            dof: DOF[k].id, forceMomentArmM: R[j][k], lengthDerivativeMomentArmM: RLength[j][k], differenceM };
        }
      }
      muscleQuality[j].derivativeMismatchM = strandMismatch;
    }
    const invalidPaths = muscleQuality.filter(p => p.quarantined);
    const pathQuality = { invalidPaths, quarantinedStrands: invalidPaths.length,
      quarantinedUnits: [...new Set(invalidPaths.map(p => p.unit))].sort(),
      disabledNominalCapacityN: FMAX.reduce((sum, force, j) => sum + (muscleQuality[j].quarantined ? force : 0), 0),
      disabledActiveCapacityN: capacity.reduce((sum, force, j) => sum + (muscleQuality[j].quarantined ? force : 0), 0),
      maxDerivativeMismatchM, maxDerivativeMismatch, momentArmDefinition: 'owned-path-unit-tension-virtual-work',
      maxQuarantinedMomentArmM: Math.max(0, ...invalidPaths.flatMap(p => p.violations.map(v => Math.abs(v.momentArmM)))) };
    const unidentifiedUnits = ['thenar', 'hypothenar', 'interossei'];
    const constitutive = { assumption: 'rigid-tendon-relative-to-standing', fibreFractionFloorApplied: false,
      forceLengthClamped: false, slackFiberStrands: muscleQuality.filter(p => p.slackFiber).length,
      slackFiberUnits: [...new Set(muscleQuality.filter(p => p.slackFiber).map(p => p.unit))].sort(),
      negligibleActiveCapacityStrands: capacity.filter(force => force < 1e-6).length,
      minNormalizedFiberLength: Math.min(...lt), maxNormalizedFiberLength: Math.max(...lt) };
    const spP = spinePassive(conf.local);
    STOPS = o.stops ? Object.assign({}, JOINT_STOPS, o.stops) : JOINT_STOPS;
    for (const [id, q] of Object.entries(jointStops(W))) spP[id] = (spP[id] || 0) + q;

    const com = A.comOf(W);
    const contacts = floorContacts(W, o.contacts, o.contactThr);
    if (!contacts.length) throw new Error('Static equilibrium requires external floor support');
    const Nv = verticalLoads(contacts, com);
    const st = stContacts(W);
    const grips = internalContacts(W, o.grips ?? [], o.internalContacts ?? []);

    const NM = INST.length, NC = contacts.length, NG = grips.length, NS = st.length;
    const iM = 0, iC = NM, iG = iC + 3 * NC, iS = iG + 3 * NG, iR = iS + NS;
    const jointDofs = DOF.map((d, k) => k).filter(k => DOF[k].joint !== 'root');
    const resIdx = new Map(jointDofs.map((k, r) => [k, iR + r]));
    // passive stiffness of a muscle group is not known for a given person: one scale per group and side,
    // kept near 1 by a small cost (the practitioner is as flexible as the pose needs, within limits)
    const pgroups = [];
    if (o.passiveGroups) for (const [name, units] of Object.entries(o.passiveGroups)) for (const sd of ['L', 'R']) {
      const js = []; INST.forEach((m, j) => { if (m.side === sd && units.includes(m.unit) && FP[j] > 0.5) js.push(j); });
      if (js.length) pgroups.push({ name, side: sd, js });
    }
    const iP = iR + jointDofs.length;
    const NX = iP + pgroups.length;

    // known generalized forces: gravity, vertical floor reactions, passive muscle pull, passive spine
    const known = new Float64Array(ND);
    const Aeq = [], beq = [];
    const MOMS = 0.1 * BW;
    for (let k = 0; k < ND; k++) {
      const d = DOF[k], ax = frames[k].ax, c = frames[k].c;
      const sc = d.kind === 'trans' ? BW : MOMS;
      const row = new Float64Array(NX);
      for (let j = 0; j < NM; j++) if (R[j][k]) row[iM + j] = R[j][k] * FE[j] / sc;
      contacts.forEach((ct, i) => { const v = dpdq(k, ct.seg, ct.P, ax, c); if (!v) return; row[iC + 3 * i] += v.y * BW / sc; row[iC + 3 * i + 1] += v.x * BW / sc; row[iC + 3 * i + 2] += v.z * BW / sc; });
      grips.forEach((g, i) => {
        const v = pairedDisplacement(k, g.segA, g.segB, g.P, ax, c);
        row[iG + 3 * i] += v.x * BW / sc; row[iG + 3 * i + 1] += v.y * BW / sc; row[iG + 3 * i + 2] += v.z * BW / sc;
      });
      st.forEach((s, i) => {
        const va = dpdq(k, s.segA, s.P, ax, c), vb = dpdq(k, s.segB, s.P, ax, c);
        let v = 0; if (va) v += va.dot(s.n); if (vb) v -= vb.dot(s.n);
        row[iS + i] += v * BW / sc;
      });
      if (resIdx.has(k)) row[resIdx.get(k)] = 10 / sc;
      pgroups.forEach((g, i) => { let v = 0; for (const j of g.js) if (R[j][k]) v += R[j][k] * FP[j]; row[iP + i] = v / sc; });
      let b = 0;
      for (const m of MASS) { const P = pt(W, m.seg, m.p); const v = dpdq(k, m.seg, P, ax, c); if (v) b += v.y * (-m.m * BW); }
      for (let j = 0; j < NM; j++) if (FP[j] && R[j][k]) b += R[j][k] * FP[j];
      if (spP[d.id] != null) b += spP[d.id];
      known[k] = b;
      Aeq.push(row); beq.push(-b / sc);
    }

    const lo = new Float64Array(NX).fill(-1e3), hi = new Float64Array(NX).fill(1e3);
    const tone = o.tone ?? {};
    if (!tone || typeof tone !== 'object' || Object.values(tone).some(t => !Number.isFinite(t) || t < 0 || t > 1))
      throw new Error('Prescribed activation bounds must lie in [0,1]');
    for (let j = 0; j < NM; j++) {
      lo[iM + j] = muscleQuality[j].quarantined || lt[j] <= 0 ? 0 : tone[INST[j].unit] ?? 0;
      hi[iM + j] = muscleQuality[j].quarantined || lt[j] <= 0 ? 0 : 1;
    }
    for (let i = 0; i < NC; i++) { lo[iC + 3 * i] = 0; hi[iC + 3 * i] = contacts[i].maxNormalN === undefined ? 3 : contacts[i].maxNormalN / BW; }
    for (let i = 0; i < NG; i++) for (let a = 0; a < 3; a++) {
      const bound = grips[i].maxComponentN === undefined ? 1e3 : grips[i].maxComponentN / BW;
      lo[iG + 3 * i + a] = -bound; hi[iG + 3 * i + a] = bound;
    }
    for (let i = 0; i < NS; i++) { lo[iS + i] = 0; hi[iS + i] = o.noST ? 0 : 1; }
    for (let i = 0; i < pgroups.length; i++) { lo[iP + i] = (o.passiveLo ?? 0.5) - 1; hi[iP + i] = (o.passiveHi ?? 2) - 1; }
    // Isotropic Coulomb friction, approximated by an inscribed 16-sided cone.
    const Ain = [], bin = [], inKeys = [];
    for (let i = 0; i < NC; i++) for (const [facetIndex, facet] of frictionFacets(contacts[i].mu).entries()) {
      const r = new Map(); r.set(iC + 3 * i, facet.normal); r.set(iC + 3 * i + 1, facet.x); r.set(iC + 3 * i + 2, facet.z);
      Ain.push(r); bin.push(0); inKeys.push('floor:' + contacts[i].id + ':coulomb:' + facetIndex);
    }
    // A common contact point gives the two bodies equal/opposite forces with
    // zero net whole-body force AND moment. Normal is the force on segment A.
    grips.forEach((g, i) => {
      const add = (direction, bound, key) => {
        const row = new Map();
        for (let a = 0; a < 3; a++) row.set(iG + 3 * i + a, direction.getComponent(a));
        Ain.push(row); bin.push(bound); inKeys.push('internal:' + g.id + ':' + key);
      };
      if (g.kind !== 'bilateral') {
        add(g.normal.clone().negate(), 0, 'nonnegative-normal');
        for (const [facetIndex, facet] of frictionFacets(g.mu).entries())
          add(g.normal.clone().multiplyScalar(facet.normal).addScaledVector(g.tangent1, facet.x).addScaledVector(g.tangent2, facet.z), 0, 'coulomb:' + facetIndex);
        if (g.maxNormalN !== undefined) add(g.normal, g.maxNormalN / BW, 'normal-capacity');
      }
      if (g.maxForceN !== undefined) for (const [face, facet] of forceBallFacets(g.maxForceN / BW).entries())
        add(facet.normal, facet.bound, 'force-capacity:' + face);
    });

    const P = new Float64Array(NX), qlin = new Float64Array(NX);
    for (let j = 0; j < NM; j++) P[iM + j] = FMAX[j] / 1000;
    // floor: elastic energy of the compressed tissue (N^2/k) for the normal force, a small price for friction
    for (let i = 0; i < NC; i++) { P[iC + 3 * i] = (o.wE ?? 0.6) / contacts[i].k; P[iC + 3 * i + 1] = o.wT ?? 0.3; P[iC + 3 * i + 2] = o.wT ?? 0.3; }
    for (let i = 0; i < 3 * NG; i++) P[iG + i] = 0.05;
    for (let i = 0; i < NS; i++) P[iS + i] = o.wS ?? 0.2;
    for (const [k, ix] of resIdx) P[ix] = o.wR || 100;
    for (let i = 0; i < pgroups.length; i++) P[iP + i] = o.wP ?? 0.01;

    const ws = o.smooth > 0 && o.prevAct ? o.smooth : 0;
    if (ws) for (let j = 0; j < NM; j++) { const f = FMAX[j] / 1000; P[iM + j] = f * (1 + ws); qlin[iM + j] = -ws * f * (o.prevAct[j] || 0); }
    // identity of variables and rows, so that a neighbouring frame can seed the solver
    const varKeys = new Array(NX);
    for (let j = 0; j < NM; j++) varKeys[iM + j] = 'm' + j;
    contacts.forEach((ct, i) => { for (let a = 0; a < 3; a++) varKeys[iC + 3 * i + a] = 'floor:' + ct.id + ':' + a; });
    grips.forEach((g, i) => { for (let a = 0; a < 3; a++) varKeys[iG + 3 * i + a] = 'internal:' + g.id + ':' + a; });
    for (let i = 0; i < NS; i++) varKeys[iS + i] = 's' + i;
    for (const [k, ix] of resIdx) varKeys[ix] = 'r' + k;
    pgroups.forEach((g, i) => { varKeys[iP + i] = 'p' + g.name + g.side; });
    const eqKeys = DOF.map((d, k) => 'e' + k);
    // without a warm start the vertical loads of the rigid-body solution are a good first guess for the floor
    let warm = o.warm || null;
    if (warm) {
      warm = { x: new Map(warm.x), y: warm.y, rho: warm.rho };
      contacts.forEach((ct, i) => { const k = 'floor:' + ct.id + ':0'; if (!warm.x.has(k)) warm.x.set(k, Nv[i] / BW); });
    }
    const equilibriumResiduals = value => {
      let forceResidualN = 0, momentResidualNm = 0;
      Aeq.forEach((row, r) => {
        let sum = 0; for (let c = 0; c < NX; c++) sum += row[c] * value[c];
        const error = Math.abs(sum - beq[r]) * (DOF[r].kind === 'trans' ? BW : MOMS);
        if (DOF[r].kind === 'trans') forceResidualN = Math.max(forceResidualN, error);
        else momentResidualNm = Math.max(momentResidualNm, error);
      });
      return { forceResidualN, momentResidualNm };
    };
    let x;
    try {
      x = qpSolveVerified(P, Aeq, beq, Ain, bin, lo, hi, { iters: o.iters || 20000, tol: o.tol || 1e-6, epsAbs: o.epsAbs, epsRel: o.epsRel, q: qlin, warm, varKeys, eqKeys, inKeys, retries: o.retries, allowUnconverged: o.allowUnconverged });
    } catch (error) {
      if (error.quality && error.solution) {
        Object.assign(error.quality, equilibriumResiduals(error.solution));
        error.message += '; force residual ' + error.quality.forceResidualN + ' N, moment residual ' + error.quality.momentResidualNm + ' N*m';
      }
      throw error;
    }

    // ---------- results ----------
    const pscale = {};
    pgroups.forEach((g, i) => { const k = 1 + x[iP + i]; pscale[g.name + '_' + g.side] = k; for (const j of g.js) FP[j] *= k; });
    const act = Array.from(x.slice(iM, iM + NM));
    const force = act.map((a, j) => a * FE[j]);
    const geometricMomentDifferenceNm = DOF.map((d, k) => ({ id: d.id,
      differenceNm: R.reduce((sum, row, j) => sum + (row[k] - RLength[j][k]) * (force[j] + FP[j]), 0) }));
    pathQuality.geometricForceMomentMismatchNm = Math.max(0, ...geometricMomentDifferenceNm.map(d => Math.abs(d.differenceNm)));
    pathQuality.geometricMomentDifferenceNm = geometricMomentDifferenceNm;
    const contactF = contacts.map((ct, i) => ({ ...ct, F: V(x[iC + 3 * i + 1], x[iC + 3 * i], x[iC + 3 * i + 2]).multiplyScalar(BW), Nrigid: Nv[i] }));
    const pairedF = grips.map((g, i) => ({ ...g, F: V(x[iG + 3 * i], x[iG + 3 * i + 1], x[iG + 3 * i + 2]).multiplyScalar(BW) }));
    const gripF = pairedF.filter(g => g.legacy), internalContactF = pairedF.filter(g => !g.legacy);
    const stF = st.map((s, i) => ({ ...s, F: s.n.clone().multiplyScalar(x[iS + i] * BW) }));
    const reserve = {};
    for (const [k, ix] of resIdx) reserve[DOF[k].id] = x[ix] * 10;
    const quality = { ...x.quality, ...equilibriumResiduals(x) };
    const maxRes = Math.max(quality.forceResidualN, quality.momentResidualNm);
    // per dof: load from gravity and contacts, passive share, active muscles, reserve, active capacity both ways
    const dofOut = DOF.map((d, k) => {
      const ax = frames[k].ax, c = frames[k].c;
      let ext = 0;
      for (const m of MASS) { const Pm = pt(W, m.seg, m.p); const v = dpdq(k, m.seg, Pm, ax, c); if (v) ext += v.y * (-m.m * BW); }
      for (const ct of contactF) { const v = dpdq(k, ct.seg, ct.P, ax, c); if (v) ext += v.dot(ct.F); }
      for (const g of pairedF) { const vh = dpdq(k, g.hand, g.P, ax, c), vs = dpdq(k, g.shank, g.P, ax, c); if (vh) ext += vh.dot(g.F); if (vs) ext -= vs.dot(g.F); }
      let stm = 0;
      for (const s of stF) { const va = dpdq(k, s.segA, s.P, ax, c), vb = dpdq(k, s.segB, s.P, ax, c); if (va) stm += va.dot(s.F); if (vb) stm -= vb.dot(s.F); }
      let mus = 0, pas = spP[d.id] || 0, capPos = 0, capNeg = 0;
      for (let j = 0; j < NM; j++) { const r = R[j][k]; if (!r) continue; mus += r * force[j]; pas += r * FP[j]; if (r > 0) capPos += r * FE[j]; else capNeg -= r * FE[j]; }
      return { id: d.id, joint: d.joint, comp: d.comp, ext, st: stm, mus, pas, res: reserve[d.id] || 0, capPos, capNeg };
    });
    // joint reaction forces: what the proximal bone pushes on the rest of the limb/trunk across the joint
    const reaction = (childSeg, exclude) => {
      const set = subtree(childSeg); if (exclude) for (const s of subtree(exclude)) set.delete(s);
      const F = V(0, 0, 0);
      for (const m of MASS) if (set.has(m.seg)) F.y -= m.m * BW;
      for (const ct of contactF) if (set.has(ct.seg)) F.add(ct.F);
      for (const g of pairedF) { if (set.has(g.hand)) F.add(g.F); if (set.has(g.shank)) F.sub(g.F); }
      for (const s of stF) { if (set.has(s.segA)) F.add(s.F); if (set.has(s.segB)) F.sub(s.F); }
      for (let j = 0; j < NM; j++) { const f = force[j] + FP[j]; if (f > 0.5) F.add(pullOn(paths[j], set, f)); }
      return F.negate();
    };
    const jr = {};
    const along = (F, seg, axis) => F.dot(basis(W[seg].q, axis));
    { const F = reaction('lumA'); const comp = along(F, 'lumA', 1); jr.L5S1 = { F, compression: comp, shear: Math.sqrt(Math.max(0, F.lengthSq() - comp * comp)) }; }
    { const F = reaction('thA'); const comp = along(F, 'thA', 1); jr.T12L1 = { F, compression: comp, shear: Math.sqrt(Math.max(0, F.lengthSq() - comp * comp)) }; }
    for (const sd of ['L', 'R']) {
      jr['hip_' + sd] = { F: reaction('thigh_' + sd) };
      jr['knee_' + sd] = { F: reaction('shank_' + sd) };
      jr['ankle_' + sd] = { F: reaction('foot_' + sd) };
      jr['gh_' + sd] = { F: reaction('hum_' + sd) };
      jr['elbow_' + sd] = { F: reaction('fore_' + sd) };
      jr['wrist_' + sd] = { F: reaction('hand_' + sd) };
    }
    // Assemble all real point loads independently of the QP moment-arm matrix.
    // This catches a lost action/reaction pair, a sign/scaling error, and the
    // former combination of a clipped lever with an unclipped physical force.
    const bodyLoads = Object.fromEntries(SEG.map(s => [s.name, { F: V(0, 0, 0), M: V(0, 0, 0) }]));
    const addLoad = (segment, P, F) => {
      bodyLoads[segment].F.add(F); bodyLoads[segment].M.add(P.clone().cross(F));
    };
    for (const m of MASS) addLoad(m.seg, pt(W, m.seg, m.p), V(0, -m.m * BW, 0));
    for (const c of contactF) addLoad(c.seg, c.P, c.F);
    for (const g of pairedF) { addLoad(g.segA, g.P, g.F); addLoad(g.segB, g.P, g.F.clone().negate()); }
    for (const s of stF) { addLoad(s.segA, s.P, s.F); addLoad(s.segB, s.P, s.F.clone().negate()); }
    for (let j = 0; j < NM; j++) {
      const tension = force[j] + FP[j];
      for (let i = 1; i < paths[j].length; i++) {
        const a = paths[j][i - 1], b = paths[j][i], pull = b.p.clone().sub(a.p).normalize().multiplyScalar(tension);
        addLoad(a.seg, a.p, pull); addLoad(b.seg, b.p, pull.clone().negate());
      }
    }
    let forceResidualN = 0, momentResidualNm = 0, maxAssemblyDifference = 0;
    const byDof = DOF.map((d, k) => {
      let physical = 0;
      for (const name of d.moved) {
        const load = bodyLoads[name];
        physical += (d.kind === 'trans' ? load.F : load.M.clone().sub(frames[k].c.clone().cross(load.F))).dot(frames[k].ax);
      }
      physical += (spP[d.id] || 0) + (reserve[d.id] || 0);
      const scaledError = Aeq[k].reduce((sum, value, index) => sum + value * x[index], 0) - beq[k];
      const qpError = scaledError * (d.kind === 'trans' ? BW : MOMS);
      maxAssemblyDifference = Math.max(maxAssemblyDifference, Math.abs(physical - qpError));
      if (d.kind === 'trans') forceResidualN = Math.max(forceResidualN, Math.abs(physical));
      else momentResidualNm = Math.max(momentResidualNm, Math.abs(physical));
      return { id: d.id, kind: d.kind, residual: physical };
    });
    const forceToleranceN = quality.primalTolerance * BW + 1e-7;
    const momentToleranceNm = quality.primalTolerance * MOMS + 1e-7;
    const assemblyVerified = maxAssemblyDifference <= 1e-7;
    const physicalBalance = { verified: quality.converged && assemblyVerified
        && forceResidualN <= forceToleranceN && momentResidualNm <= momentToleranceNm,
      assemblyVerified, forceResidualN, momentResidualNm, forceToleranceN, momentToleranceNm,
      maxAssemblyDifference, byDof };
    if (!assemblyVerified) throw new Error('Physical force assembly disagrees with the QP equilibrium: ' + maxAssemblyDifference);
    if (quality.converged && !physicalBalance.verified) throw new Error('Physical force balance failed despite numerical QP convergence');
    const contactQuality = { floorContacts: contacts.length, internalContacts: internalContactF.length,
      floorSource: o.contacts === undefined ? 'landmarks' : 'explicit-surface-patches',
      missingStrengthLimitIds: internalContactF.filter(g => g.maxForceN === undefined && g.maxNormalN === undefined && g.maxComponentN === undefined).map(g => g.id),
      numericalInternalForceGuardN: 1e3 * BW,
      numericalForceGuardActiveIds: pairedF.filter(g => g.maxComponentN === undefined && Math.max(Math.abs(g.F.x), Math.abs(g.F.y), Math.abs(g.F.z)) >= .999e3 * BW).map(g => g.id),
      optionalForceCapacityApproximation: 'inscribed-icosahedron-20-faces' };
    Object.assign(quality, { physicalBalanceVerified: physicalBalance.verified,
      physicalForceResidualN: forceResidualN, physicalMomentResidualNm: momentResidualNm });
    return { W, com, act, force, FE, FP, lt, R, RLength, contactF, gripF, internalContactF, stF, reserve, maxRes, quality, dofOut, jr, BW, BODY, INST, FMAX, Lm, L_STAND, paths, spP, pscale,
      muscleQuality, pathQuality, constitutive, physicalBalance, contactQuality, unidentifiedUnits,
      warm: x.warm, iters: x.iters };
  }

  return { solve, DOF, dofIndex, INST, FMAX, BW, BODY, subtree, contactsOf, verticalLoads, stContacts, L_STAND, W_STAND, rotW, dpdq, pairedDisplacement, ownedPath, pullOn };
}

function solve3(M, b) {
  const a = M.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c; for (let r = c + 1; r < 3; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    for (let r = 0; r < 3; r++) if (r !== c) { const f = a[r][c] / a[c][c]; for (let k = c; k < 4; k++) a[r][k] -= f * a[c][k]; }
  }
  return [a[0][3] / a[0][0], a[1][3] / a[1][1], a[2][3] / a[2][2]];
}

// ---------- quadratic program: min 1/2 x'Px + q'x  s.t. Aeq x = beq, Ain x <= bin, lo <= x <= hi  (ADMM, OSQP-style) ----------
// o.q: linear term (default 0). o.warm = { x: Map(key → value), y: Map(rowKey → dual), rho }: starting point from a similar
// problem, matched through o.varKeys (per variable), o.eqKeys (per equality row) and o.inKeys (per inequality row); bound rows
// are keyed 'b' + variable key. The Float64Array result carries .warm, .iters
// and .quality. Residuals describe the returned, bound-clamped vector, not an
// earlier ADMM iterate. epsAbs/epsRel (or tol for both) set stopping tolerances.
export function qpSolve(Pd, Aeq, beq, Ain, bin, lo, hi, o = {}) {
  const n = Pd.length;
  const rows = [];
  for (let r = 0; r < Aeq.length; r++) { const idx = [], val = []; const row = Aeq[r]; for (let c = 0; c < n; c++) if (row[c] !== 0) { idx.push(c); val.push(row[c]); } rows.push({ idx, val, l: beq[r], u: beq[r], eq: true }); }
  Ain.forEach((m, r) => rows.push({ idx: [...m.keys()], val: [...m.values()], l: -1e20, u: bin[r], eq: false }));
  for (let c = 0; c < n; c++) rows.push({ idx: [c], val: [1], l: lo[c], u: hi[c], eq: lo[c] === hi[c] });
  const m = rows.length;
  let rho = o.rho || 0.1;
  const sigma = 1e-6, alpha = 1.6;
  const rhoV = new Float64Array(m);
  const setRho = () => { for (let r = 0; r < m; r++) rhoV[r] = rows[r].eq ? 1e3 * rho : rho; };
  setRho();
  let L = null;
  const factor = () => {
    const M = new Float64Array(n * n);
    for (let c = 0; c < n; c++) M[c * n + c] = Pd[c] + sigma;
    for (let r = 0; r < m; r++) {
      const { idx, val } = rows[r], w = rhoV[r];
      for (let a = 0; a < idx.length; a++) { const ia = idx[a] * n, va = val[a] * w; for (let b = 0; b < idx.length; b++) M[ia + idx[b]] += va * val[b]; }
    }
    for (let j = 0; j < n; j++) {
      const rj = j * n;
      let s = M[rj + j];
      for (let k = 0; k < j; k++) s -= M[rj + k] * M[rj + k];
      const d = Math.sqrt(Math.max(s, 1e-12));
      M[rj + j] = d;
      for (let i = j + 1; i < n; i++) {
        const ri = i * n;
        let t = M[ri + j];
        for (let k = 0; k < j; k++) t -= M[ri + k] * M[rj + k];
        M[ri + j] = t / d;
      }
    }
    L = M;
  };
  const cholSolve = b => {
    const y = new Float64Array(n);
    for (let i = 0; i < n; i++) { let s = b[i]; const ri = i * n; for (let k = 0; k < i; k++) s -= L[ri + k] * y[k]; y[i] = s / L[ri + i]; }
    for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * y[k]; y[i] = s / L[i * n + i]; }
    return y;
  };
  const ql = o.q || null;
  const rowKey = r => { const ne = Aeq.length, ni = Ain.length; if (r < ne) return o.eqKeys ? o.eqKeys[r] : null; if (r < ne + ni) return o.inKeys ? o.inKeys[r - ne] : null; return o.varKeys ? 'b' + o.varKeys[r - ne - ni] : null; };
  if (o.warm && o.warm.rho) { rho = o.warm.rho; setRho(); }
  factor();
  let x = new Float64Array(n), z = new Float64Array(m), y = new Float64Array(m);
  const Ax = v => { const out = new Float64Array(m); for (let r = 0; r < m; r++) { const { idx, val } = rows[r]; let s = 0; for (let a = 0; a < idx.length; a++) s += val[a] * v[idx[a]]; out[r] = s; } return out; };
  const ATv = v => { const out = new Float64Array(n); for (let r = 0; r < m; r++) { const vr = v[r]; if (!vr) continue; const { idx, val } = rows[r]; for (let a = 0; a < idx.length; a++) out[idx[a]] += val[a] * vr; } return out; };
  if (o.warm && o.varKeys) {
    for (let c = 0; c < n; c++) { const v = o.warm.x.get(o.varKeys[c]); if (v !== undefined) x[c] = Math.min(hi[c], Math.max(lo[c], v)); }
    const ax0 = Ax(x);
    for (let r = 0; r < m; r++) { z[r] = Math.min(rows[r].u, Math.max(rows[r].l, ax0[r])); const k = rowKey(r); if (k && o.warm.y) { const v = o.warm.y.get(k); if (v !== undefined) y[r] = v; } }
  }
  const iters = o.iters || 20000;
  const epsAbs = o.epsAbs ?? o.tol ?? 1e-6, epsRel = o.epsRel ?? o.tol ?? 1e-6;
  if (!(Number.isFinite(epsAbs) && epsAbs >= 0 && Number.isFinite(epsRel) && epsRel >= 0)) throw new Error('QP tolerances must be finite and nonnegative');
  const clampResult = () => Float64Array.from(x, (v, c) => Math.min(hi[c], Math.max(lo[c], v)));
  const diagnose = value => {
    const ax = Ax(value), aty = ATv(y);
    let primalResidual = 0, dualResidual = 0, equalityViolation = 0, inequalityViolation = 0, boundViolation = 0;
    let maxAx = 0, maxZ = 0, maxPx = 0, maxATy = 0, maxQ = 0;
    let finite = value.every(Number.isFinite) && y.every(Number.isFinite) && z.every(Number.isFinite);
    for (let r = 0; r < m; r++) {
      primalResidual = Math.max(primalResidual, Math.abs(ax[r] - z[r]));
      maxAx = Math.max(maxAx, Math.abs(ax[r])); maxZ = Math.max(maxZ, Math.abs(z[r]));
      if (r < Aeq.length) equalityViolation = Math.max(equalityViolation, Math.abs(ax[r] - beq[r]));
      else if (r < Aeq.length + Ain.length) inequalityViolation = Math.max(inequalityViolation, Math.max(0, ax[r] - bin[r - Aeq.length]));
    }
    for (let c = 0; c < n; c++) {
      const px = Pd[c] * value[c], q = ql ? ql[c] : 0;
      dualResidual = Math.max(dualResidual, Math.abs(px + q + aty[c]));
      boundViolation = Math.max(boundViolation, Math.max(0, lo[c] - value[c], value[c] - hi[c]));
      maxPx = Math.max(maxPx, Math.abs(px)); maxATy = Math.max(maxATy, Math.abs(aty[c])); maxQ = Math.max(maxQ, Math.abs(q));
    }
    const primalTolerance = epsAbs + epsRel * Math.max(maxAx, maxZ);
    const dualTolerance = epsAbs + epsRel * Math.max(maxPx, maxATy, maxQ);
    finite &&= [primalResidual, dualResidual, equalityViolation, inequalityViolation, boundViolation, primalTolerance, dualTolerance].every(Number.isFinite);
    const converged = finite && primalResidual <= primalTolerance && dualResidual <= dualTolerance && equalityViolation <= primalTolerance && inequalityViolation <= primalTolerance && boundViolation <= primalTolerance;
    return { status: !finite ? 'nonfinite' : converged ? 'solved' : 'max_iterations', converged, finite, iterations: 0, primalResidual, dualResidual, equalityViolation, inequalityViolation, boundViolation, primalTolerance, dualTolerance };
  };
  const t = new Float64Array(m);
  let it = 0;
  for (; it < iters; it++) {
    for (let r = 0; r < m; r++) t[r] = rhoV[r] * z[r] - y[r];
    const rhs = ATv(t);
    for (let c = 0; c < n; c++) rhs[c] += sigma * x[c];
    if (ql) for (let c = 0; c < n; c++) rhs[c] -= ql[c];
    const xt = cholSolve(rhs);
    const zt = Ax(xt);
    for (let c = 0; c < n; c++) x[c] = alpha * xt[c] + (1 - alpha) * x[c];
    for (let r = 0; r < m; r++) {
      const zr = alpha * zt[r] + (1 - alpha) * z[r];
      const v = zr + y[r] / rhoV[r];
      const zn = Math.min(rows[r].u, Math.max(rows[r].l, v));
      y[r] += rhoV[r] * (zr - zn);
      z[r] = zn;
    }
    if (it % 50 === 49) {
      const axv = Ax(x);
      let rp = 0; for (let r = 0; r < m; r++) rp = Math.max(rp, Math.abs(axv[r] - z[r]));
      const g = ATv(y); let rd = 0; for (let c = 0; c < n; c++) rd = Math.max(rd, Math.abs(Pd[c] * x[c] + g[c] + (ql ? ql[c] : 0)));
      if (diagnose(clampResult()).converged) break;
      if (it % 400 === 399) {
        let ax = 0, zz = 0, px = 0, aty = 0;
        for (let r = 0; r < m; r++) { ax = Math.max(ax, Math.abs(axv[r])); zz = Math.max(zz, Math.abs(z[r])); }
        for (let c = 0; c < n; c++) { px = Math.max(px, Math.abs(Pd[c] * x[c] + (ql ? ql[c] : 0))); aty = Math.max(aty, Math.abs(g[c])); }
        const ratio = Math.sqrt((rp / Math.max(ax, zz, 1e-12)) / (rd / Math.max(px, aty, 1e-12) + 1e-12));
        if (ratio > 5 || ratio < 0.2) { rho = Math.min(1e4, Math.max(1e-4, rho * ratio)); setRho(); factor(); }
      }
    }
  }
  const xr = clampResult();
  xr.iters = Math.min(it + 1, iters);
  xr.quality = diagnose(xr); xr.quality.iterations = xr.iters;
  if (o.varKeys) {
    const wx = new Map(), wy = new Map();
    for (let c = 0; c < n; c++) wx.set(o.varKeys[c], xr[c]);
    for (let r = 0; r < m; r++) { const k = rowKey(r); if (k && y[r] !== 0) wy.set(k, y[r]); }
    xr.warm = { x: wx, y: wy, rho };
  }
  return xr;
}

// Retry the same mathematical problem by continuing its primal/dual state.
// Tolerances, objective and constraints stay fixed. Unverified values are only
// returned when a caller explicitly requests diagnostic allowUnconverged mode.
export function qpSolveVerified(Pd, Aeq, beq, Ain, bin, lo, hi, o = {}) {
  const retries = o.retries ?? 1, baseIters = o.iters || 20000;
  if (!Number.isInteger(retries) || retries < 0) throw new Error('QP retries must be a nonnegative integer');
  const keyed = {
    ...o,
    varKeys: o.varKeys || Array.from({ length: Pd.length }, (_, c) => 'v' + c),
    eqKeys: o.eqKeys || Array.from({ length: Aeq.length }, (_, r) => 'e' + r),
    inKeys: o.inKeys || Array.from({ length: Ain.length }, (_, r) => 'i' + r),
  };
  let warm = o.warm, result, totalIterations = 0;
  const attemptQuality = [];
  for (let attempt = 0; attempt <= retries; attempt++) {
    result = qpSolve(Pd, Aeq, beq, Ain, bin, lo, hi, { ...keyed, warm, iters: baseIters * 2 ** attempt });
    totalIterations += result.iters;
    attemptQuality.push({ ...result.quality });
    if (result.quality.converged || !result.quality.finite) break;
    warm = result.warm;
  }
  Object.assign(result.quality, { attempts: attemptQuality.length, totalIterations, attemptQuality });
  if (!result.quality.converged && !o.allowUnconverged) {
    const error = new Error('QP solution was not verified (' + result.quality.status + ', primal ' + result.quality.primalResidual + ', dual ' + result.quality.dualResidual + ')');
    error.quality = result.quality;
    error.solution = result;
    throw error;
  }
  return result;
}
