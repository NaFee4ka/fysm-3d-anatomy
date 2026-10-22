// High plank on the palms (anchor between seg4 and seg5): writes cycle_keys/highplank.json.
// Palms flat, middle finger forward (turned out a little), wrists nearly under the shoulders, elbows a few degrees short of
// straight, one straight line ankles–pelvis–shoulders, neck neutral, feet hip-width on tucked toes, toes at z = -0.83.
import fs from 'fs';
import { A, V, THREE, S, SEG, qAx, X, Y, Z, computeWorld, pt, LANDMARKS, D, IDQ, scapulaPose, comOf, contactsOf } from './cycle_lib.mjs';
import { setLR, bisect, LP, wpt, fingers, SPREAD, handBasis, orientHandL, blankLocal, toData, handLmL } from './seg5_util.mjs';

export const HP = { lum: 2, thor: [1, 1, 0], neck: [0, 0, 0], ankleDorsi: 10, scapUp: 14, scapPost: 2, scapProt: 12, scapElev: 0, clavRot: 6, elbowFlex: 5, turnOut: 8, shoulderAhead: 0.01, handX: 0.215, ankleX: 0.09, toeZ: -0.83 };

export function buildHighPlank(o) {
  const p = Object.assign({}, HP, o || {});
  const L = blankLocal();
  L.lumA = qAx(X, p.lum); L.lumB = qAx(X, p.lum);
  L.thA = qAx(X, p.thor[0]); L.thB = qAx(X, p.thor[1]); L.thC = qAx(X, p.thor[2]);
  L.neckA = qAx(X, p.neck[0]); L.neckB = qAx(X, p.neck[1]); L.head = qAx(X, p.neck[2]);
  const left = {};
  const sp = scapulaPose(p.scapUp, p.scapPost, p.scapProt, null, p.clavRot, p.scapElev);
  left.clav = sp.clav; left.scap = sp.scap;
  fingers(left, SPREAD);
  const st = { sh: 65, abd: 4, toes: 70, hipExt: 3, add: 0, phi: 68 };
  const toeC = LP('toes', 0.1, 0.004, 0.178), toeB = LP('toes', 0.1, 0.006, 0.145);
  const toeLm = LANDMARKS.filter(l => l.side === 'L' && (l.seg === 'toes_L' || l.seg === 'foot_L'));
  const fdir = V(Math.sin(p.turnOut * D), 0, Math.cos(p.turnOut * D));
  const apply = () => {
    const RTH = qAx(X, -st.sh).multiply(qAx(Z, st.abd));
    left.hum = left.clav.clone().multiply(left.scap).invert().multiply(RTH);
    left.thigh = qAx(X, st.hipExt).multiply(qAx(Z, st.add));
    left.shank = IDQ.clone(); left.foot = qAx(X, -p.ankleDorsi); left.toes = qAx(X, -st.toes);
    const rootQ = qAx(X, st.phi);
    orientHandL(L, left, rootQ, V(0, 0, 0), handBasis(fdir, V(0, -1, 0)), qAx(X, -p.elbowFlex));
    const W0 = computeWorld(L, rootQ, V(0, 0, 0));
    let mn = Infinity; for (const l of handLmL) mn = Math.min(mn, pt(W0, l.seg, l.pc).y);
    const rootP = V(0, -mn, 0);
    return { rootQ, rootP, W: computeWorld(L, rootQ, rootP) };
  };
  const minToe = W => { let m = Infinity; for (const l of toeLm) m = Math.min(m, pt(W, l.seg, l.pc).y); return m; };
  // pelvis on the line from the ankle to the shoulder (sagittal plane): + when the pelvis is above the line
  const sag = W => { const a = W.foot_L.p, s = W.hum_L.p, h = W.pelvis.p; const u = (h.z - a.z) / (s.z - a.z); return h.y - (a.y + u * (s.y - a.y)); };
  for (let it = 0; it < 10; it++) {
    st.phi = bisect(v => { st.phi = v; return minToe(apply().W); }, 40, 100);
    st.sh = bisect(v => { st.sh = v; const { W } = apply(); return (W.hum_L.p.z - W.hand_L.p.z) - p.shoulderAhead; }, 30, 110);
    st.abd = bisect(v => { st.abd = v; return apply().W.hand_L.p.x - p.handX; }, -15, 25);
    st.toes = bisect(v => { st.toes = v; const { W } = apply(); return wpt(W, toeC).y - wpt(W, toeB).y; }, -10, 110);
    st.hipExt = bisect(v => { st.hipExt = v; return sag(apply().W); }, -20, 25);
    st.add = bisect(v => { st.add = v; return apply().W.foot_L.p.x - p.ankleX; }, -12, 12);
  }
  const { rootQ, rootP } = apply();
  // toes on the mark along the mat
  const W1 = computeWorld(L, rootQ, rootP);
  const tc = contactsOf(W1).filter(c => c.l.region === 'пальцы стоп');
  const zt = tc.reduce((a, c) => a + c.p.z, 0) / tc.length;
  rootP.z += p.toeZ - zt;
  return { L, rootQ, rootP, st, p };
}

if (process.argv[1] && process.argv[1].endsWith('cycle_hp.mjs')) {
  const c = buildHighPlank();
  const W = computeWorld(c.L, c.rootQ, c.rootP);
  const cs = contactsOf(W);
  const g = {};
  for (const ct of cs) { const k = ct.l.region + ':' + ct.l.side; (g[k] ||= []).push(ct.p); }
  const avg = ps => ps.reduce((a, q) => a.add(q), V(0, 0, 0)).multiplyScalar(1 / ps.length);
  const r3 = v => +v.toFixed(3);
  const contacts = Object.fromEntries(Object.entries(g).map(([k, ps]) => { const m = avg(ps); return [k, { x: r3(m.x), z: r3(m.z), n: ps.length }]; }));
  const com = comOf(W);
  const elbowAng = sd => { const h = W['fore_' + sd].p.clone().sub(W['hum_' + sd].p).normalize(), f = W['hand_' + sd].p.clone().sub(W['fore_' + sd].p).normalize(); return Math.acos(Math.min(1, h.dot(f))) / D; };
  const info = {
    wrist: { x: r3(W.hand_L.p.x), y: r3(W.hand_L.p.y), z: r3(W.hand_L.p.z) },
    shoulder: { x: r3(W.hum_L.p.x), y: r3(W.hum_L.p.y), z: r3(W.hum_L.p.z) },
    elbow: { x: r3(W.fore_L.p.x), y: r3(W.fore_L.p.y), z: r3(W.fore_L.p.z), flexDeg: +elbowAng('L').toFixed(1) },
    pelvis: { y: r3(W.pelvis.p.y), z: r3(W.pelvis.p.z) },
    ankle: { x: r3(W.foot_L.p.x), y: r3(W.foot_L.p.y), z: r3(W.foot_L.p.z) },
    com: { x: r3(com.x), y: r3(com.y), z: r3(com.z) },
    bodyPitchDeg: +(90 - c.st.phi).toFixed(1),
    solved: Object.fromEntries(Object.entries(c.st).map(([k, v]) => [k, +v.toFixed(2)])),
    contacts,
  };
  const out = { ...toData(c.L, c.rootQ, c.rootP), info };
  const dry = process.argv.includes('--dry');
  const file = dry ? '/tmp/claude-0/-home-claude/e37a82bc-007b-5e07-be95-11ef82530e9e/scratchpad/hp_try.json' : 'cycle_keys/highplank.json';
  fs.writeFileSync(file, JSON.stringify(out));
  fs.writeFileSync('/tmp/claude-0/-home-claude/e37a82bc-007b-5e07-be95-11ef82530e9e/scratchpad/hp_seg.json', JSON.stringify({ id: 'hp', from: 'highplank', to: 'highplank', fps: 10, duration: 0, frames: [{ t: 0, label: 'highplank', ...toData(c.L, c.rootQ, c.rootP) }] }));
  console.log(file, JSON.stringify(info, null, 1));
}
