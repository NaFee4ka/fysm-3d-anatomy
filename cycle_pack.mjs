// Packs the round for the page: frames (int16), per-frame muscle activation (uint8, both sides), per-frame strain of
// ligaments and capsules (int8, from geometry), floor forces if physics has them, and the statistics for one round.
// usage: node cycle_pack.mjs   →  cycle_data.json   (inputs: cycle_keys/cycle.json, cycle_phys.json?, cycle_stats.json?)
//
// cycle_data.json
//   v, fps, n (frames), T (seconds of motion), segs [{id, from, to, fromName, toName, t0, t1, phases:[{t0,t1,label}]}],
//   holds [{pose, T}], segNames [59, order of q], provisional (any segment provisional), placeholder {act: bool}
//   q:    base64 Int16  n×59×4  local quaternions ×32767        root: base64 Int16 n×4 (×32767)
//   p:    base64 Int16  n×3     rootP in 0.1 mm                  grip: base64 Uint8 n (bit 0 left, bit 1 right)
//   units [91]  aL, aR: base64 Uint8 n×91, activation ×255 (zeros when physics is missing)
//   ligs [27 ligament units + 'cap:<joint>' ×6]  sL, sR: base64 Int16 n×33, strain (L/L0−1, max over the unit's strands) ×4000
//   floor (optional): base64 Int16 n×10×6 [region, side, Fx, Fy, Fz (N), cop x, cop z (mm)] packed as [code, Fx, Fy, Fz, cx, cz],
//         regions [names]; com (optional): base64 Int16 n×3 (mm)
//   jload (optional): {ids:[ls, tl, hip_L, hip_R, kn_…, an_…, gh_…, el_…, wr_…], v: base64 Uint8 n×J, force in the joint × body weight ×50}
//   grips (optional): base64 Int16 n×2×6 [P x,y,z (mm), F x,y,z (N)]
//   stats: { T, thresholds:{act:[.08,.2,.4], lig:[.02,.05]}, muscles:{u:{trans:{L,R}, pose:{p:{L,R}}, stretch:{peak,peakT,pose}}},
//            ligs:{id:{trans:{L,R}, pose:{p:{L,R}}}}, joints:{...} }
import fs from 'fs';
import * as THREE from 'three';
import { createAnatomy } from './anatomy3.js';
import { validateCyclePhysics, packJointLoads } from './cycle_quality.mjs';

const A = createAnatomy(THREE);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
// options (for tests): --cycle FILE --phys FILE --stats FILE --out FILE
const arg = (k, d) => { const i = process.argv.indexOf(k); return i > 0 ? process.argv[i + 1] : d; };
const CYCLE_F = arg('--cycle', 'cycle_keys/cycle.json'), PHYS_F = arg('--phys', 'cycle_phys.json'), STATS_F = arg('--stats', 'cycle_stats.json'), OUT_F = arg('--out', 'cycle_data.json');
const cyc = JSON.parse(fs.readFileSync(CYCLE_F, 'utf8'));
const phys = fs.existsSync(PHYS_F) ? JSON.parse(fs.readFileSync(PHYS_F, 'utf8')) : null;
const statsIn = fs.existsSync(STATS_F) ? JSON.parse(fs.readFileSync(STATS_F, 'utf8')) : null;
const modelQuality = validateCyclePhysics(cyc, phys, {
  units: Object.keys(A.UNITS), ligs: [...Object.keys(A.LIGS), ...Object.keys(A.CAP_NAMES).map(id => 'cap_' + id)],
});
const fps = cyc.fps || 10, n = cyc.frames.length, dt = 1 / fps;
const SEGN = A.SEG.map(s => s.name);
const UNITS = Object.keys(A.UNITS);
const LIGU = Object.keys(A.LIGS), CAPJ = Object.keys(A.CAP_NAMES);
const LIGS = [...LIGU, ...CAPJ.map(j => 'cap:' + j)];
const b64 = ta => Buffer.from(ta.buffer, ta.byteOffset, ta.byteLength).toString('base64');
const r3 = v => Math.round(v * 1000) / 1000, r2 = v => Math.round(v * 100) / 100;

// ---------- frames ----------
const q16 = new Int16Array(n * SEGN.length * 4), root16 = new Int16Array(n * 4), p16 = new Int16Array(n * 3), grip = new Uint8Array(n);
const confs = [];
cyc.frames.forEach((f, i) => {
  const local = {};
  SEGN.forEach((s, k) => {
    let q = f.local[s] || [0, 0, 0, 1];
    const len = Math.hypot(...q) || 1;
    // keep w ≥ 0 so neighbouring frames slerp along the short way after quantisation
    const sg = q[3] < 0 ? -1 : 1;
    for (let c = 0; c < 4; c++) q16[(i * SEGN.length + k) * 4 + c] = Math.round(sg * q[c] / len * 32767);
    local[s] = new THREE.Quaternion(q[0], q[1], q[2], q[3]).normalize();
  });
  const rq = f.root, rl = Math.hypot(...rq) || 1, rs = rq[3] < 0 ? -1 : 1;
  for (let c = 0; c < 4; c++) root16[i * 4 + c] = Math.round(rs * rq[c] / rl * 32767);
  for (let c = 0; c < 3; c++) p16[i * 3 + c] = Math.round(f.rootP[c] * 10000);
  grip[i] = ((f.grip || []).includes('L') ? 1 : 0) | ((f.grip || []).includes('R') ? 2 : 0);
  confs.push({ local, rootQ: new THREE.Quaternion(...rq).normalize(), rootP: V(...f.rootP) });
});
if (cyc.frames.some((f, i) => Math.abs((f.T ?? i * dt) - i * dt) > 1e-4)) console.warn('warning: frames are not evenly spaced at', fps, 'fps');

// ---------- geometry: ligament/capsule strain and muscle length, both sides ----------
const insts = A.strandInstances();
const W0 = A.poseAt(A.STAND, 0).W;
const L0 = insts.map(i => A.pathLength(A.strandPath(W0, i)));
const ligIdx = id => LIGS.indexOf(id);
const NL = LIGS.length;
const sL = new Float32Array(n * NL).fill(-1), sR = new Float32Array(n * NL).fill(-1);
const stretch = { L: new Float32Array(n * UNITS.length), R: new Float32Array(n * UNITS.length) };
const cntU = { L: new Float32Array(UNITS.length), R: new Float32Array(UNITS.length) };
insts.forEach(i => { if (i.kind === 'muscle') cntU[i.side][UNITS.indexOf(i.unit)]++; });
const t0 = Date.now();
confs.forEach((c, f) => {
  const W = A.computeWorld(c.local, c.rootQ, c.rootP);
  insts.forEach((inst, j) => {
    const e = A.pathLength(A.strandPath(W, inst)) / L0[j] - 1;
    if (inst.kind === 'muscle') { const u = UNITS.indexOf(inst.unit); stretch[inst.side][f * UNITS.length + u] += e / cntU[inst.side][u]; return; }
    const id = inst.kind === 'lig' ? inst.unit : 'cap:' + inst.unit;
    const k = ligIdx(id); if (k < 0) return;
    const arr = inst.side === 'L' ? sL : sR;
    arr[f * NL + k] = Math.max(arr[f * NL + k], e);
  });
});
console.log('geometry', n, 'frames in', Date.now() - t0, 'ms');
const s16 = arr => { const o = new Int16Array(arr.length); for (let k = 0; k < arr.length; k++) o[k] = Math.max(-32767, Math.min(32767, Math.round(arr[k] * 4000))); return o; };

// ---------- activation from physics (interpolated onto the frame times) ----------
const NU = UNITS.length;
const aL = new Float32Array(n * NU), aR = new Float32Array(n * NU);
let placeholderAct = true, floor16 = null, com16 = null, regions = null, jeff = null, grips16 = null;
if (phys && Array.isArray(phys.frames) && phys.frames.length) {
  const pu = phys.units || UNITS;
  const map = UNITS.map(u => pu.indexOf(u));
  const PT = phys.frames.map((f, i) => f.T ?? f.t ?? i * dt);
  const at = (f, side) => { const v = f['a' + side] || (f.act && f.act[side]); return v; };
  const get = (v, k, u) => (Array.isArray(v) ? (map[k] >= 0 ? v[map[k]] : 0) : (v && v[u]) || 0);
  if (at(phys.frames[0], 'L')) {
    placeholderAct = false;
    for (let i = 0; i < n; i++) {
      const T = i * dt;
      let j = 0; while (j < PT.length - 2 && PT[j + 1] < T) j++;
      const ta = PT[j], tb = PT[Math.min(PT.length - 1, j + 1)], w = tb > ta ? Math.max(0, Math.min(1, (T - ta) / (tb - ta))) : 0;
      const fa = phys.frames[j], fb = phys.frames[Math.min(PT.length - 1, j + 1)];
      for (const [side, out] of [['L', aL], ['R', aR]]) {
        const va = at(fa, side), vb = at(fb, side);
        UNITS.forEach((u, k) => { out[i * NU + k] = (1 - w) * get(va, k, u) + w * get(vb, k, u); });
      }
    }
  }
  // floor forces and centre of mass, nearest physics frame
  const near = T => { let best = 0; PT.forEach((t, k) => { if (Math.abs(t - T) < Math.abs(PT[best] - T)) best = k; }); return phys.frames[best]; };
  if (phys.frames[0].floor) {
    regions = [];
    floor16 = new Int16Array(n * 10 * 6);
    for (let i = 0; i < n; i++) {
      const f = near(i * dt);
      (f.floor || []).slice(0, 10).forEach((g, k) => {
        let ri = regions.indexOf(g.region); if (ri < 0) { regions.push(g.region); ri = regions.length - 1; }
        const cop = g.cop || [0, 0, 0], cz = cop.length === 3 ? cop[2] : cop[1];
        const o = (i * 10 + k) * 6;
        floor16[o] = 1 + ri * 4 + (g.side === 'L' ? 1 : g.side === 'R' ? 2 : 0); floor16[o + 1] = Math.round(g.F[0]); floor16[o + 2] = Math.round(g.F[1]); floor16[o + 3] = Math.round(g.F[2]);
        floor16[o + 4] = Math.round(cop[0] * 1000); floor16[o + 5] = Math.round(cz * 1000);
      });
    }
  }
  if (phys.frames[0].com) { com16 = new Int16Array(n * 3); for (let i = 0; i < n; i++) { const c = near(i * dt).com; for (let k = 0; k < 3; k++) com16[i * 3 + k] = Math.round(c[k] * 1000); } }
  // force with which the bones press on each other in a joint (× body weight), interpolated like the activation
  if (phys.frames[0].jr) {
    const MAP = [['ls', 'L5S1', 0], ['tl', 'T12L1', 0]];
    for (const [j, k] of [['hip', 'hip'], ['kn', 'knee'], ['an', 'ankle'], ['gh', 'gh'], ['el', 'elbow'], ['wr', 'wrist']]) { MAP.push([j + '_L', k, 0]); MAP.push([j + '_R', k, 1]); }
    const ids = MAP.map(m => m[0]);
    const values = new Float64Array(n * ids.length);
    for (let i = 0; i < n; i++) {
      const T = i * dt;
      let j = 0; while (j < PT.length - 2 && PT[j + 1] < T) j++;
      const ta = PT[j], tb = PT[Math.min(PT.length - 1, j + 1)], w = tb > ta ? Math.max(0, Math.min(1, (T - ta) / (tb - ta))) : 0;
      const ja = phys.frames[j].jr || {}, jb = phys.frames[Math.min(PT.length - 1, j + 1)].jr || {};
      MAP.forEach(([id, key, c], k) => { const a = ja[key][c], b = jb[key][c]; values[i * ids.length + k] = (1 - w) * a + w * b; });
    }
    const packed = packJointLoads(values);
    jeff = { ids, v: b64(packed.values), scale: packed.scale, unit: 'BW' };
  }
  if (phys.frames.some(f => f.grips && f.grips.length)) {
    grips16 = new Int16Array(n * 2 * 6);
    for (let i = 0; i < n; i++) {
      const f = near(i * dt);
      (f.grips || []).slice(0, 2).forEach((g, k) => { const o = (i * 2 + k) * 6; for (let c = 0; c < 3; c++) { grips16[o + c] = Math.round(g.P[c] * 1000); grips16[o + 3 + c] = Math.round(g.F[c]); } });
    }
  }
}
// any physics frame in the source time that is provisional is reported, not hidden
const provisional = cyc.segs.some(s => s.provisional);

// ---------- Myers' lines over the round (fascia3.js): stretch from geometry, engagement from the activations ----------
let flines = null, flS16 = null, flE8 = null;
if (fs.existsSync('fascia3.js') && fs.existsSync('books/fascia_model.json')) {
  const { createFascia } = await import('./fascia3.js');
  const F = createFascia(A, THREE, JSON.parse(fs.readFileSync('books/fascia_model.json', 'utf8')));
  const ids = F.LINES.map(l => l.id), K = ids.length;
  const pst = JSON.parse(fs.readFileSync('physics.json', 'utf8')).stand.muscles;
  const Fmax = Object.fromEntries(Object.entries(pst).map(([u, m]) => [u, m.Fmax]));
  const S = new Float32Array(n * K), E = new Float32Array(n * K);
  const tf = Date.now();
  confs.forEach((c, f) => {
    const W = A.computeWorld(c.local, c.rootQ, c.rootP);
    const a = {}; UNITS.forEach((u, k) => { a[u] = { L: aL[f * NU + k], R: aR[f * NU + k] }; });
    const m = F.withPhysics(F.measure(W), placeholderAct ? null : { a, Fmax, Fp: null });
    ids.forEach((id, k) => { const L = m.lines[id + '_L'], R = m.lines[id + '_R']; S[f * K + k] = R ? (L.strain + R.strain) / 2 : L.strain; E[f * K + k] = Math.max(L.engage || 0, R ? R.engage || 0 : 0); });
  });
  flines = { ids, lines: {} };
  ids.forEach((id, k) => {
    let sMax = -Infinity, sMaxT = 0, sMin = Infinity, sMinT = 0, eMax = 0, eMaxT = 0, eInt = 0;
    for (let i = 0; i < n; i++) {
      const v = S[i * K + k], e = E[i * K + k];
      if (v > sMax) { sMax = v; sMaxT = i * dt; } if (v < sMin) { sMin = v; sMinT = i * dt; } if (e > eMax) { eMax = e; eMaxT = i * dt; }
      if (i > 0) eInt += (e + E[(i - 1) * K + k]) / 2 * dt;
    }
    flines.lines[id] = { sMax: r3(sMax), sMaxT: r2(sMaxT), sMin: r3(sMin), sMinT: r2(sMinT), eMax: r3(eMax), eMaxT: r2(eMaxT), eInt: r3(eInt), pose: Object.fromEntries(cyc.holds.map(h => { const i = Math.max(0, Math.min(n - 1, Math.round(h.T * fps))); return [h.pose, { s: r3(S[i * K + k]), e: r3(E[i * K + k]) }]; })) };
  });
  flS16 = Int16Array.from(S, v => Math.max(-32767, Math.min(32767, Math.round(v * 4000))));
  flE8 = Uint8Array.from(E, v => Math.round(Math.max(0, Math.min(1, v)) * 255));
  console.log('myers lines', K, 'over', n, 'frames in', Date.now() - tf, 'ms');
}

// ---------- statistics for one round (physics file wins; the rest from the packed series) ----------
const holdIdx = cyc.holds.map(h => ({ pose: h.pose, i: Math.max(0, Math.min(n - 1, Math.round(h.T * fps))) }));
function series(arr, stride, k) { const o = new Float32Array(n); for (let i = 0; i < n; i++) o[i] = arr[i * stride + k]; return o; }
function transStats(s, ths) {
  let peak = -Infinity, peakT = 0, int = 0;
  const tAbove = ths.map(() => 0);
  for (let i = 0; i < n; i++) {
    if (s[i] > peak) { peak = s[i]; peakT = i * dt; }
    if (i > 0) { const m = (s[i] + s[i - 1]) / 2; int += Math.max(0, m) * dt; ths.forEach((t, k) => { if (m >= t) tAbove[k] += dt; }); }
  }
  return { peak, peakT, int, tAbove };
}
const stats = { T: r2(cyc.T), thresholds: { act: [0.08, 0.2, 0.4], lig: [0.02, 0.05] }, muscles: {}, ligs: {}, joints: {}, source: {} };
const sideStats = (arr, stride, k, kind) => {
  const s = series(arr, stride, k);
  const ths = kind === 'act' ? [0.08, 0.2, 0.4] : [0.02, 0.05];
  const t = transStats(s, ths);
  if (kind === 'act') return { peak: r3(t.peak), peakT: r2(t.peakT), int: r3(t.int), t08: r2(t.tAbove[0]), t20: r2(t.tAbove[1]), t40: r2(t.tAbove[2]) };
  let over = 0; for (let i = 1; i < n; i++) over += Math.max(0, (s[i] + s[i - 1]) / 2 - 0.02) * dt;
  return { peak: r3(t.peak), peakT: r2(t.peakT), t02: r2(t.tAbove[0]), t05: r2(t.tAbove[1]), int: r3(over) };
};
UNITS.forEach((u, k) => {
  const m = { trans: { L: sideStats(aL, NU, k, 'act'), R: sideStats(aR, NU, k, 'act') }, pose: {} };
  for (const h of holdIdx) m.pose[h.pose] = { L: r3(aL[h.i * NU + k]), R: r3(aR[h.i * NU + k]) };
  const st = series(stretch.L, NU, k), sr = series(stretch.R, NU, k);
  let peak = -Infinity, peakT = 0; for (let i = 0; i < n; i++) { const v = Math.max(st[i], sr[i]); if (v > peak) { peak = v; peakT = i * dt; } }
  m.stretch = { peak: r3(peak), peakT: r2(peakT), pose: Object.fromEntries(holdIdx.map(h => [h.pose, r3(Math.max(st[h.i], sr[h.i]))])) };
  stats.muscles[u] = m;
});
LIGS.forEach((id, k) => {
  const o = { trans: { L: sideStats(sL, NL, k, 'lig'), R: sideStats(sR, NL, k, 'lig') }, pose: {} };
  for (const h of holdIdx) o.pose[h.pose] = { L: r3(sL[h.i * NL + k]), R: r3(sR[h.i * NL + k]) };
  stats.ligs[id] = o;
});
stats.source = { act: placeholderAct ? 'placeholder' : 'cycle_phys.json', ligs: 'geometry', stretch: 'geometry', joints: 'none' };
if (statsIn) {
  // the physics agent's numbers replace the ones computed here wherever they exist
  if (statsIn.thresholds) stats.thresholds = { ...stats.thresholds, ...statsIn.thresholds };
  if (statsIn.muscles) for (const [u, m] of Object.entries(statsIn.muscles)) if (stats.muscles[u]) {
    if (m.trans) stats.muscles[u].trans = m.trans;
    if (m.pose) stats.muscles[u].pose = m.pose;
    if (m.stretch) stats.muscles[u].stretch = m.stretch;
  }
  if (statsIn.ligs) for (const [id0, o] of Object.entries(statsIn.ligs)) {
    const id = stats.ligs[id0] ? id0 : (A.CAP_NAMES[id0] ? 'cap:' + id0 : id0.replace(/^cap_/, 'cap:'));
    if (stats.ligs[id]) { if (o.trans) stats.ligs[id].trans = o.trans; if (o.pose) stats.ligs[id].pose = o.pose; }
  }
  if (statsIn.joints) stats.joints = statsIn.joints;
  if (statsIn.T) stats.T = statsIn.T;
  stats.source = { ...stats.source, file: 'cycle_stats.json', act: statsIn.muscles ? 'cycle_stats.json' : stats.source.act, ligs: statsIn.ligs ? 'cycle_stats.json' : 'geometry', stretch: statsIn.muscles ? 'cycle_stats.json' : 'geometry', joints: statsIn.joints ? 'cycle_stats.json' : 'none' };
}

const out = {
  v: 1, fps, n, T: r2(cyc.T), segs: cyc.segs, holds: cyc.holds, segNames: SEGN, provisional, placeholder: { act: placeholderAct && !(statsIn && statsIn.muscles) },
  q: b64(q16), root: b64(root16), p: b64(p16), grip: b64(grip),
  units: UNITS, aL: b64(Uint8Array.from(aL, v => Math.round(Math.max(0, Math.min(1, v)) * 255))), aR: b64(Uint8Array.from(aR, v => Math.round(Math.max(0, Math.min(1, v)) * 255))),
  ligs: LIGS, sL: b64(s16(sL)), sR: b64(s16(sR)),
  floor: floor16 ? b64(floor16) : null, regions, com: com16 ? b64(com16) : null, grips: grips16 ? b64(grips16) : null, jload: jeff,
  stats,
  modelQuality,
  flines: flines ? { ...flines, s: b64(flS16), e: b64(flE8), placeholder: placeholderAct } : null,
};
const txt = JSON.stringify(out);
fs.writeFileSync(OUT_F, txt);
console.log(OUT_F, (txt.length / 1024).toFixed(0), 'KB,', n, 'frames,', r2(cyc.T), 's,', 'activation:', out.placeholder.act ? 'PLACEHOLDER (zeros)' : 'physics', provisional ? '| provisional frames' : '| real frames', '| floor', !!floor16, '| stats', statsIn ? 'cycle_stats.json' : 'computed here');
