// Build-time physics for the five poses and plain standing: writes physics.json for the page.
import * as THREE from 'three';
import fs from 'fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE, MUSCLE, POSE_STOPS } from './physics_params.mjs';

const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const r1 = v => Math.round(v * 10) / 10, r3 = v => Math.round(v * 1000) / 1000;
const arr = v => [r3(v.x), r3(v.y), r3(v.z)];
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));

function hull(P) {
  const Q = P.map(p => [p[0], p[1]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (Q.length < 3) return Q;
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of Q) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = Q.length - 1; i >= 0; i--) { const p = Q[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
// signed distance from point to polygon boundary (positive inside)
function margin(poly, p) {
  if (poly.length < 3) return -1;
  let inside = true, dmin = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const ex = b[0] - a[0], ez = b[1] - a[1];
    const cr = ex * (p[1] - a[1]) - ez * (p[0] - a[0]);
    if (cr < 0) inside = false;
    const L2 = ex * ex + ez * ez;
    const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ez) / (L2 || 1)));
    dmin = Math.min(dmin, Math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ez));
  }
  return inside ? dmin : -dmin;
}

// muscle moment the joint needs, by anatomical component, with the group that provides it
// sign +: [label for +, label for -]
const COMP = {
  spine: { x: ['сгибатели', 'разгибатели'], y: ['вращают', 'вращают'], z: ['наклоняют вбок', 'наклоняют вбок'] },
  sc: { x: ['наклоняют лопатку', 'наклоняют лопатку'], y: ['отводят плечо назад', 'выводят плечо вперёд'], z: ['поднимают плечо', 'опускают плечо'] },
  el: { x: ['разгибатели', 'сгибатели'], p: ['пронаторы', 'супинаторы'] },
  wr: { z: ['разгибатели', 'сгибатели'], x: ['отводят к мизинцу', 'отводят к большому пальцу'] },
  hip: { x: ['разгибатели', 'сгибатели'], y: ['наружные ротаторы', 'внутренние ротаторы'], z: ['отводящие', 'приводящие'] },
  kn: { x: ['сгибатели', 'разгибатели'] },
  an: { x: ['подошвенные сгибатели', 'тыльные сгибатели'], z: ['выворачивают стопу наружу', 'подворачивают стопу внутрь'] },
};
const typeOf = j => (/^(ls|l34|tl|t89|t45|ct|c45|ao)$/.test(j) ? 'spine' : j.replace(/_[LR]$/, ''));

export function buildPose(id, options = {}) {
  const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
  const W = A.placeW(conf.local, conf.rootQ);
  const grips = (id === 'bow' || id === 'fold') ? ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: 'hum_' + sd, cone: 1 }; }) : [];
  const t0 = Date.now();
  const tone = options.cues === false ? {} : (CUE_TONE[id] || {});
  const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone, grips, stops: POSE_STOPS[id], iters: 16000 });
  const BW = r.BW;
  // floor
  const groups = {};
  for (const c of r.contactF) {
    const k = c.region + '|' + c.side;
    const g = (groups[k] ||= { region: c.region, side: c.side, F: V(0, 0, 0), m: V(0, 0, 0), n: 0 });
    g.F.add(c.F); g.m.addScaledVector(c.P, c.F.y); g.n++;
  }
  const floor = Object.values(groups).filter(g => g.F.y > 0.002 * BW).map(g => ({ region: g.region, side: g.side, F: arr(g.F), cop: arr(g.m.clone().multiplyScalar(1 / g.F.y)), pct: r1(g.F.y / BW * 100), tang: r1(Math.hypot(g.F.x, g.F.z) / BW * 100) }));
  const pts = r.contactF.filter(c => c.F.y > 0.002 * BW).map(c => [c.P.x, c.P.z]);
  const poly = hull(pts);
  const marg = margin(poly, [r.com.x, r.com.z]);
  // muscles, left side (the poses are symmetric)
  const mus = {};
  r.INST.forEach((s, j) => {
    if (s.side !== 'L') return;
    const m = (mus[s.unit] ||= { F: 0, Fp: 0, Fmax: 0, Fe: 0, activationNumerator: 0 });
    m.F += r.force[j]; m.Fp += r.FP[j]; m.Fmax += r.FMAX[j]; m.Fe += r.FE[j];
    m.activationNumerator += r.act[j] * r.FMAX[j];
  });
  for (const [u, m] of Object.entries(mus)) {
    m.a = m.F / m.Fmax;
    // Recruitment activation and active force/Fmax differ when force-length
    // capacity is reduced. Keep the existing displayed force fraction in `a`.
    m.activation = r3(m.activationNumerator / m.Fmax);
    delete m.activationNumerator;
    // Presence of a prescribed minimum, not a binding constraint or a causal
    // share of active force. Tone bounds activation, whereas `a` is force/Fmax.
    m.cue = (tone[u] || 0) > 0;
    m.tone = tone[u] || 0;
    for (const k of ['F', 'Fp', 'Fmax', 'Fe']) m[k] = Math.round(m[k]);
    m.a = r3(m.a);
  }
  // joints
  const byJoint = {};
  for (const d of r.dofOut) {
    if (d.joint === 'root' || /_R$/.test(d.joint)) continue;
    const key = 'xyzp'[d.comp] === undefined ? 'x' : (d.id.split('_').pop());
    const cap = d.mus >= 0 ? d.capPos : d.capNeg;
    (byJoint[d.joint] ||= []).push({ c: key, need: r1(d.mus), load: r1(d.ext + d.st), passive: r1(d.pas), res: r1(d.res), cap: Math.round(cap), eff: r3(Math.abs(d.mus) / Math.max(1, cap)) });
  }
  const joints = {};
  for (const [j, comps] of Object.entries(byJoint)) {
    const t = typeOf(j), lab = COMP[t] || {};
    for (const c of comps) { const L = lab[c.c]; c.group = L ? (c.need >= 0 ? L[0] : L[1]) : null; }
    joints[j] = { comps };
  }
  // shoulder: elevation and axial rotation of the arm
  {
    const sk = ['x', 'y', 'z'].map(a => PH.dofIndex['gh_L_' + a]);
    const ax = sk.map(k => PH.DOF[k].axis(W));
    const vec = f => ax.reduce((acc, e, i) => acc.addScaledVector(e, f(r.dofOut[sk[i]])), V(0, 0, 0));
    const Mm = vec(d => d.mus), Me = vec(d => d.ext), Mp = vec(d => d.pas);
    const h = W.fore_L.p.clone().sub(W.hum_L.p).normalize();
    const down = V(0, -1, 0).applyQuaternion(W.thC.q);
    let eEl = down.clone().cross(h); if (eEl.lengthSq() < 1e-6) eEl = ax[0].clone(); eEl.normalize();
    // capacity along the two directions
    let cElP = 0, cElN = 0, cRP = 0, cRN = 0;
    r.INST.forEach((s, j) => { if (s.side !== 'L') return; let me = 0, mr = 0; for (let i = 0; i < 3; i++) { const rr = r.R[j][sk[i]]; if (!rr) continue; me += rr * ax[i].dot(eEl); mr += rr * ax[i].dot(h); } if (me > 0) cElP += me * r.FE[j]; else cElN -= me * r.FE[j]; if (mr > 0) cRP += mr * r.FE[j]; else cRN -= mr * r.FE[j]; });
    const el = Mm.dot(eEl), rot = Mm.dot(h);
    // past the line of the trunk (arm behind the ear) "raising" means bringing the arm back towards that line
    const chest = V(0, 0, 1).applyQuaternion(W.thC.q), upT = V(0, 1, 0).applyQuaternion(W.thC.q);
    const behind = h.dot(upT) > 0.7 && h.dot(chest) < 0;
    const elevGroup = el >= 0 ? (behind ? 'не дают руке уйти за голову' : 'поднимают руку') : (behind ? 'уводят руку за голову' : 'опускают руку');
    joints.gh_L.arm = {
      elev: { need: r1(el), load: r1(Me.dot(eEl)), passive: r1(Mp.dot(eEl)), cap: Math.round(el >= 0 ? cElP : cElN), eff: r3(Math.abs(el) / Math.max(1, el >= 0 ? cElP : cElN)), group: elevGroup, behind },
      rot: { need: r1(rot), load: r1(Me.dot(h)), passive: r1(Mp.dot(h)), cap: Math.round(rot >= 0 ? cRP : cRN), eff: r3(Math.abs(rot) / Math.max(1, rot >= 0 ? cRP : cRN)), group: rot >= 0 ? 'вращают плечо внутрь' : 'вращают плечо наружу' },
    };
  }
  // joint reaction forces
  const jrMap = { gh_L: 'gh_L', el_L: 'elbow_L', wr_L: 'wrist_L', hip_L: 'hip_L', kn_L: 'knee_L', an_L: 'ankle_L' };
  for (const [j, k] of Object.entries(jrMap)) if (joints[j]) { const F = r.jr[k].F; joints[j].jr = { F: Math.round(F.length()), bw: r3(F.length() / BW) }; }
  joints.ls = joints.ls || {}; joints.ls.jr = { F: Math.round(r.jr.L5S1.F.length()), bw: r3(r.jr.L5S1.F.length() / BW), comp: Math.round(r.jr.L5S1.compression), shear: Math.round(r.jr.L5S1.shear) };
  // how much of the disc compression comes from muscles pulling across L5–S1, and which ones
  {
    const set = PH.subtree('lumA'), ax = V(0, 1, 0).applyQuaternion(r.W.lumA.q), byU = {};
    r.INST.forEach((s, j) => { const f = r.force[j] + r.FP[j]; if (f < 0.5) return; const c = -PH.pullOn(r.paths[j], set, f).dot(ax); if (Math.abs(c) > 0.5) byU[s.unit] = (byU[s.unit] || 0) + c; });
    joints.ls.jr.mus = Math.round(Object.values(byU).reduce((a, b) => a + b, 0));
    joints.ls.jr.top = Object.entries(byU).sort((a, b) => b[1] - a[1]).slice(0, 2).map(([u]) => u);
  }
  joints.tl = joints.tl || {}; joints.tl.jr = { F: Math.round(r.jr.T12L1.F.length()), bw: r3(r.jr.T12L1.F.length() / BW), comp: Math.round(r.jr.T12L1.compression), shear: Math.round(r.jr.T12L1.shear) };
  // patellofemoral force: quadriceps tension turned round the patella
  {
    let Fq = 0; const dir = V(0, 0, 0);
    r.INST.forEach((s, j) => {
      if (s.side !== 'L' || !/^(rectFem|vastLat|vastMed|vastInt)$/.test(s.unit)) return;
      const f = r.force[j] + r.FP[j]; if (f < 1) return;
      const path = r.paths[j]; const set = PH.subtree('shank_L');
      for (let i = 1; i < path.length; i++) { const a = path[i - 1], b = path[i]; if (!set.has(a.seg) && set.has(b.seg)) { dir.addScaledVector(a.p.clone().sub(b.p).normalize(), f); Fq += f; } }
    });
    const pat = A.pt(W, 'shank_L', V(0.098, 0.528, 0.04)), tub = A.pt(W, 'shank_L', V(0.098, 0.452, 0.03));
    const pf = dir.clone().add(tub.clone().sub(pat).normalize().multiplyScalar(Fq));
    joints.kn_L.pf = { F: Math.round(pf.length()), bw: r3(pf.length() / BW), quad: Math.round(Fq) };
  }
  const st = r.stF.filter(s => s.side === 'L').reduce((a, s) => a + s.F.length(), 0);
  joints.st_L = { st: { F: Math.round(st), bw: r3(st / BW) } };
  // Use every unrounded external contact for the independent whole-body check.
  const supportForce = r.contactF.reduce((sum, c) => sum.add(c.F), V(0, 0, 0));
  const supportMoment = r.contactF.reduce((sum, c) => sum.add(c.P.clone().cross(c.F)), V(0, 0, 0));
  supportMoment.add(r.com.clone().cross(V(0, -BW, 0)));
  const outsideCurve = r.lt.map((length, i) => ({ length, unit: r.INST[i].unit }))
    .filter(({ length }) => length < 0.4 || length > 1.7);
  const out = {
    bw: Math.round(BW), mass: PH.BODY, com: arr(r.com),
    support: { poly: poly.map(p => [r3(p[0]), r3(p[1])]), margin: r3(marg) },
    floor, grips: r.gripF.map(g => ({ side: g.side, P: arr(g.P), F: arr(g.F), N: Math.round(g.F.length()) })),
    muscles: mus, joints,
    balance: { forceN: supportForce.toArray(), weightN: BW,
      forceErrorN: supportForce.clone().add(V(0, -BW, 0)).toArray(), momentErrorNm: supportMoment.toArray() },
    modelLimits: { forceLengthClampedStrands: outsideCurve.length,
      forceLengthClampedUnits: [...new Set(outsideCurve.map(s => s.unit))].sort(),
      fibreFractionFloorUnits: Object.keys(MUSCLE).filter(u => MUSCLE[u].phi < 0.3).sort() },
    diag: { resid: r.maxRes, reserves: r1(Object.values(r.reserve).reduce((a, x) => a + Math.abs(x), 0)), ms: Date.now() - t0,
      quality: r.quality, forceResidualN: r.quality?.forceResidualN, momentResidualNm: r.quality?.momentResidualNm },
  };
  return out;
}

export const POSE_IDS = ['stand', 'dog', 'plank', 'half', 'bow', 'fold', 'handstand'];
export function buildAllPoses(options = {}) {
  return Object.fromEntries(POSE_IDS.map(id => [id, buildPose(id, options)]));
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
const only = process.argv[2];
const ids = only ? only.split(',') : POSE_IDS;
const res = {};
for (const id of ids) {
  res[id] = buildPose(id);
  const o = res[id];
  const top = Object.entries(o.muscles).sort((a, b) => b[1].a - a[1].a).slice(0, 8).map(([u, m]) => `${u} ${Math.round(m.a * 100)}%${m.cue ? '*' : ''}`).join(', ');
  console.log(id.padEnd(6), o.diag.ms + 'ms', 'resid', o.diag.resid.toExponential(1), 'reserves', o.diag.reserves, 'N·m | L5S1', o.joints.ls.jr.comp, 'N (' + (o.joints.ls.jr.comp / o.bw).toFixed(2) + ' BW) | margin', (o.support.margin * 100).toFixed(1), 'cm\n   ', top);
}
if (!only) fs.writeFileSync('physics.json', JSON.stringify(res));
else fs.writeFileSync('physics_dbg.json', JSON.stringify(res));
}
