import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

/*__ANATOMY__*/
/*__FASCIA_LIB__*/
/*__MUSCLE_SURFACES_LIB__*/
/*__CONNECTIVE_SURFACES_LIB__*/
/*__JOINT_SURFACES_LIB__*/

// ---------- typography: keep short words and dashes off line ends ----------
const SHORT = 'в|во|и|не|ни|до|на|с|со|к|ко|о|об|от|у|по|за|из|а|но|же|для|при|без|над|под|про|или|как|что|это|то|их|его|её|ее|чем|чём|уже';
const RE_SHORT = new RegExp('(^|[\\s(«„"])(' + SHORT + ')[\\t\\r\\n ]+', 'giu');
function nb(s) {
  let result = String(s), previous;
  // Match only breakable suffix spaces. Each pass protects another word in
  // chains such as "и не в"; an existing NBSP must not consume that next match.
  do {
    previous = result;
    result = result.replace(RE_SHORT, (m, a, w) => a + w + '\u00a0');
  } while (result !== previous);
  return result.replace(/\s—/g, ' —').replace(/\s–/g, ' –').replace(/—\s/g, '— ').replace(/–\s/g, '– ').replace(/ (\S{1,4})$/, '\u00a0$1');
}
function typesetStatic(root) {
  const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
  const nodes = [];
  while (walker.nextNode()) nodes.push(walker.currentNode);
  for (const n of nodes) if (n.parentElement && !/SCRIPT|STYLE/.test(n.parentElement.tagName)) n.nodeValue = nb(n.nodeValue);
}
typesetStatic(document.body);

// ---------- model data ----------
const A = createAnatomy(THREE);
const POSE_DATA = /*__POSES__*/null;
const PHYS = /*__PHYSICS__*/null;
const RESEARCH_SENSITIVITY = /*__RESEARCH_SENSITIVITY__*/null;
const CYC_DATA = /*__CYCLE__*/null;
const FASCIA_MODEL = /*__FASCIA__*/null;
const FASCIA_NOTES = /*__FASCIA_NOTES__*/null;
const MUSCLE_SURFACE_DATA = /*__MUSCLE_SURFACES__*/null;
const JOINT_SURFACE_DATA = /*__JOINT_SURFACES__*/null;
const CAPSULE_RIM_DATA = /*__CAPSULE_RIMS__*/null;
const PRACTICE_DATA = /*__PRACTICE__*/null;
const PRACTICE_ALGORITHMS = PRACTICE_DATA?.algorithms || [];
const PRACTICE_BY_ID = new Map(PRACTICE_ALGORITHMS.map(a => [a.id, a]));
const practiceConfs = Object.fromEntries(Object.entries(PRACTICE_DATA?.poses || {}).map(([id, data]) => [id, {
  local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion().fromArray(data.local[s.name]).normalize()])),
  rootQ: new THREE.Quaternion().fromArray(data.root).normalize(),
  rootP: new THREE.Vector3().fromArray(data.rootP),
}]));
const EXTRAS = POSE_DATA.extras || {};
const POSES = A.POSES;
const POSE = Object.fromEntries(POSES.map(p => [p.id, p]));
const confs = Object.fromEntries(POSES.map(p => [p.id, A.poseFromData(POSE_DATA[p.id])]));
const insts = A.strandInstances();
const reduceMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

// ---------- the round: five poses with the transitions between them (packed by cycle_pack.mjs) ----------
function b64bytes(s) { const bin = atob(s); const u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return u; }
const CYC = (() => {
  const d = CYC_DATA;
  if (!d || !d.n || d.n < 2) return null;
  const i16 = s => new Int16Array(b64bytes(s).buffer);
  const c = { ...d, qa: i16(d.q), ra: i16(d.root), pa: i16(d.p), ga: b64bytes(d.grip), aLa: b64bytes(d.aL), aRa: b64bytes(d.aR), sLa: i16(d.sL), sRa: i16(d.sR) };
  c.NS = d.segNames.length; c.NU = d.units.length; c.NL = d.ligs.length;
  c.unitIdx = Object.fromEntries(d.units.map((u, k) => [u, k]));
  c.ligIdx = Object.fromEntries(d.ligs.map((u, k) => [u, k]));
  c.floorA = d.floor ? i16(d.floor) : null;
  c.comA = d.com ? i16(d.com) : null;
  c.jloadA = d.jload ? b64bytes(d.jload.v) : null;
  c.jloadIdx = d.jload ? Object.fromEntries(d.jload.ids.map((id, k) => [id, k])) : null;
  c.gripsA = d.grips ? i16(d.grips) : null;
  c.phases = d.segs.flatMap((s, k) => s.phases.map(p => ({ ...p, seg: k })));
  if (d.flines) { c.flS = i16(d.flines.s); c.flE = b64bytes(d.flines.e); c.NF = d.flines.ids.length; c.flIdx = Object.fromEntries(d.flines.ids.map((id, k) => [id, k])); }
  return c;
})();
const newConf = () => ({ local: Object.fromEntries(A.SEG.map(s => [s.name, new THREE.Quaternion()])), rootQ: new THREE.Quaternion(), rootP: new THREE.Vector3() });
const cloneConf = c => ({ local: Object.fromEntries(Object.entries(c.local).map(([k, q]) => [k, q.clone()])), rootQ: c.rootQ.clone(), rootP: c.rootP ? c.rootP.clone() : null });
// root position that puts the lowest point on the floor and the body in the middle of the mat (what placeW does)
function placedRootP(local, rootQ) {
  const W0 = A.computeWorld(local, rootQ, new THREE.Vector3());
  let minY = Infinity, zmin = Infinity, zmax = -Infinity;
  for (const l of A.LANDMARKS) { const p = A.pt(W0, l.seg, l.pc); if (p.y < minY) minY = p.y; if (p.z < zmin) zmin = p.z; if (p.z > zmax) zmax = p.z; }
  return new THREE.Vector3(0, -minY, -(zmin + zmax) / 2);
}
// joints, root and place on the mat blended together
function blendFull(a, b, u) {
  const L = {};
  for (const s of A.SEG) L[s.name] = a.local[s.name].clone().slerp(b.local[s.name], u);
  const pa = a.rootP || placedRootP(a.local, a.rootQ), pb = b.rootP || placedRootP(b.local, b.rootQ);
  return { local: L, rootQ: a.rootQ.clone().slerp(b.rootQ, u), rootP: pa.clone().lerp(pb, u) };
}
const cycNow = { i: 0, u: 0 };
function cycIndex(t) {
  const x = Math.max(0, Math.min(CYC.n - 1, t * CYC.fps));
  const i = Math.min(CYC.n - 2, Math.floor(x));
  return { i, u: x - i };
}
const _qa = new THREE.Quaternion(), _qb = new THREE.Quaternion();
const QK = 1 / 32767;
function readQ(arr, o, q) { return q.set(arr[o] * QK, arr[o + 1] * QK, arr[o + 2] * QK, arr[o + 3] * QK).normalize(); }
// configuration at time t of the round: slerp between the stored 10 fps frames
function fillCycConf(t, out, idx) {
  const { i, u } = cycIndex(t);
  if (idx) { idx.i = i; idx.u = u; }
  const NS = CYC.NS, qa = CYC.qa;
  for (let s = 0; s < NS; s++) {
    const q = out.local[CYC.segNames[s]]; if (!q) continue;
    readQ(qa, (i * NS + s) * 4, _qa); readQ(qa, ((i + 1) * NS + s) * 4, _qb);
    q.slerpQuaternions(_qa, _qb, u);
  }
  readQ(CYC.ra, i * 4, _qa); readQ(CYC.ra, (i + 1) * 4, _qb);
  out.rootQ.slerpQuaternions(_qa, _qb, u);
  const pa = CYC.pa, o0 = i * 3, o1 = (i + 1) * 3;
  out.rootP.set((pa[o0] + (pa[o1] - pa[o0]) * u) * 1e-4, (pa[o0 + 1] + (pa[o1 + 1] - pa[o0 + 1]) * u) * 1e-4, (pa[o0 + 2] + (pa[o1 + 2] - pa[o0 + 2]) * u) * 1e-4);
  return out;
}
// activation of a muscle (one side) at the current frame of the round
function cycAct(unit, side, at) {
  const k = CYC.unitIdx[unit]; if (k === undefined) return 0;
  const { i, u } = at || cycNow, a = side === 'R' ? CYC.aRa : CYC.aLa, NU = CYC.NU;
  return (a[i * NU + k] * (1 - u) + a[(i + 1) * NU + k] * u) / 255;
}
function cycStrain(id, side, at) {
  const k = CYC.ligIdx[id]; if (k === undefined) return 0;
  const { i, u } = at || cycNow, a = side === 'R' ? CYC.sRa : CYC.sLa, NL = CYC.NL;
  return (a[i * NL + k] * (1 - u) + a[(i + 1) * NL + k] * u) / 4000;
}
// force with which the bones press on each other in a joint, in body weights (physics of the round)
function cycJointLoad(id, at) {
  if (!CYC.jloadA) return 0;
  const k = CYC.jloadIdx[id]; if (k === undefined) return 0;
  const { i, u } = at || cycNow, J = CYC.jload.ids.length, sc = CYC.jload.scale || 50;
  return (CYC.jloadA[i * J + k] * (1 - u) + CYC.jloadA[(i + 1) * J + k] * u) / sc;
}
const cycConf = CYC ? newConf() : null;
let cycMorph = null;

// ---------- renderer ----------
const stage = document.getElementById('stage');
const canvas = document.getElementById('view');
let renderer;
try { renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' }); }
catch (err) {
  const l = stage.querySelector('.loading');
  if (l) l.textContent = nb('Этот браузер не поддерживает WebGL, поэтому 3D-модель не отрисуется.');
  throw err;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
const BG = 0x0d1215;
const scene = new THREE.Scene();
scene.background = new THREE.Color(BG);
scene.fog = new THREE.Fog(BG, 4.2, 10);
const camera = new THREE.PerspectiveCamera(32, 1, 0.03, 40);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.25;
controls.maxDistance = 7;
controls.maxPolarAngle = Math.PI * 0.8;
controls.rotateSpeed = 0.8;

scene.add(new THREE.HemisphereLight(0xc7d5e0, 0x2b221d, 1.2));
const key = new THREE.DirectionalLight(0xfff1de, 2.2); key.position.set(-1.8, 3.2, 1.6); scene.add(key);
const rim = new THREE.DirectionalLight(0x86b8ff, 1.4); rim.position.set(1.6, 1.2, -2.4); scene.add(rim);
const fill = new THREE.DirectionalLight(0xffd9b8, 0.5); fill.position.set(1.2, 0.4, 2.2); scene.add(fill);

const composer = new EffectComposer(renderer);
composer.addPass(new RenderPass(scene, camera));
const bloom = new UnrealBloomPass(new THREE.Vector2(512, 512), 0.5, 0.4, 0.9);
composer.addPass(bloom);
composer.addPass(new OutputPass());

// ---------- floor ----------
function floorTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 1024;
  const g = c.getContext('2d');
  const grad = g.createRadialGradient(512, 512, 40, 512, 512, 512);
  grad.addColorStop(0, 'rgba(32,42,48,1)'); grad.addColorStop(1, 'rgba(13,18,21,1)');
  g.fillStyle = grad; g.fillRect(0, 0, 1024, 1024);
  const step = 1024 / 32;
  for (let i = 0; i <= 32; i++) {
    const major = i % 5 === 0;
    g.strokeStyle = major ? 'rgba(160,180,190,0.18)' : 'rgba(160,180,190,0.07)';
    g.lineWidth = major ? 2 : 1;
    g.beginPath(); g.moveTo(i * step, 0); g.lineTo(i * step, 1024); g.stroke();
    g.beginPath(); g.moveTo(0, i * step); g.lineTo(1024, i * step); g.stroke();
  }
  const m = g.createRadialGradient(512, 512, 300, 512, 512, 512);
  m.addColorStop(0, 'rgba(13,18,21,0)'); m.addColorStop(1, 'rgba(13,18,21,1)');
  g.fillStyle = m; g.fillRect(0, 0, 1024, 1024);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 4;
  return t;
}
const floor = new THREE.Mesh(new THREE.CircleGeometry(1.6, 64), new THREE.MeshBasicMaterial({ map: floorTexture(), transparent: true, opacity: 0.95, side: THREE.DoubleSide, depthWrite: false }));
floor.rotation.x = -Math.PI / 2;
scene.add(floor);
function shadowTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256;
  const g = c.getContext('2d');
  const grad = g.createRadialGradient(128, 128, 10, 128, 128, 128);
  grad.addColorStop(0, 'rgba(0,0,0,0.55)'); grad.addColorStop(0.6, 'rgba(0,0,0,0.25)'); grad.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = grad; g.fillRect(0, 0, 256, 256);
  return new THREE.CanvasTexture(c);
}
const shadow = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ map: shadowTexture(), transparent: true, depthWrite: false }));
shadow.rotation.x = -Math.PI / 2; shadow.position.y = 0.002;
scene.add(shadow);

// ---------- forces: floor reactions, hand grips, centre of mass and the support area ----------
const forcesGroup = new THREE.Group(); scene.add(forcesGroup);
const FORCE_COL = 0xfff0cf, GRIP_COL = 0x7fd3ff, COM_COL = 0xffcf5c;
const ARROW_SHAFT = new THREE.CylinderGeometry(1, 1, 1, 12, 1).translate(0, 0.5, 0), ARROW_HEAD = new THREE.ConeGeometry(1, 1, 18).translate(0, 0.5, 0);
const Y_UP = new THREE.Vector3(0, 1, 0);
function overlayMat(color, opacity) { return new THREE.MeshBasicMaterial({ color, transparent: true, opacity: opacity ?? 0.95, depthTest: false, depthWrite: false }); }
function arrow(from, vec, scale, color) {
  const g = new THREE.Group(), mat = overlayMat(color);
  const L = Math.max(0.012, vec.length() * scale), head = Math.min(0.04, L * 0.38);
  const shaft = new THREE.Mesh(ARROW_SHAFT, mat), tip = new THREE.Mesh(ARROW_HEAD, mat);
  shaft.scale.set(0.0042, L - head, 0.0042); tip.position.y = L - head; tip.scale.set(0.011, head, 0.011);
  shaft.renderOrder = tip.renderOrder = 30;
  g.add(shaft, tip); g.position.copy(from); g.quaternion.setFromUnitVectors(Y_UP, vec.clone().normalize());
  return g;
}
const FORCE_SCALE = 0.0045 / (PHYS.dog.bw / 100); // 0.45 cm of arrow per 1% of body weight
function buildForceSet(ph) {
  const g = new THREE.Group();
  const v = a => new THREE.Vector3(a[0], a[1], a[2]);
  for (const f of ph.floor) { if (f.pct < 0.8) continue; const from = v(f.cop); from.y = 0.002; g.add(arrow(from, v(f.F), FORCE_SCALE, FORCE_COL)); }
  for (const gr of ph.grips) if (gr.N > 5) g.add(arrow(v(gr.P), v(gr.F), FORCE_SCALE, GRIP_COL));
  // centre of mass, plumb line and its point on the floor
  const com = v(ph.com);
  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.016, 20, 14), overlayMat(COM_COL, 1)); ball.position.copy(com); ball.renderOrder = 31; g.add(ball);
  const lineGeo = new THREE.BufferGeometry().setFromPoints([com, new THREE.Vector3(com.x, 0.002, com.z)]);
  const line = new THREE.Line(lineGeo, new THREE.LineDashedMaterial({ color: COM_COL, dashSize: 0.02, gapSize: 0.014, depthTest: false, transparent: true, opacity: 0.9 }));
  line.computeLineDistances(); line.renderOrder = 31; g.add(line);
  const dot = new THREE.Mesh(new THREE.RingGeometry(0.012, 0.02, 28).rotateX(-Math.PI / 2), overlayMat(COM_COL, 0.95)); dot.position.set(com.x, 0.003, com.z); dot.renderOrder = 31; g.add(dot);
  if (ph.support.poly.length >= 3) {
    const pts = ph.support.poly.map(q => new THREE.Vector3(q[0], 0.003, q[1]));
    const loop = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: FORCE_COL, transparent: true, opacity: 0.75, depthTest: false }));
    loop.renderOrder = 30; g.add(loop);
    const shape = new THREE.Shape(ph.support.poly.map(q => new THREE.Vector2(q[0], -q[1])));
    const fillM = new THREE.Mesh(new THREE.ShapeGeometry(shape).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ color: FORCE_COL, transparent: true, opacity: 0.08, depthWrite: false, side: THREE.DoubleSide }));
    fillM.position.y = 0.0025; g.add(fillM);
  }
  g.visible = false;
  return g;
}
function disposeGroup(g) { g.traverse(o => { if (o.geometry && o.geometry !== ARROW_SHAFT && o.geometry !== ARROW_HEAD) o.geometry.dispose(); if (o.material) o.material.dispose(); }); }
let forceSets = null;
function buildForces(id) {
  if (forceSets) { for (const k in forceSets) { forcesGroup.remove(forceSets[k]); disposeGroup(forceSets[k]); } }
  forceSets = { pose: buildForceSet(PHYS[id]), stand: buildForceSet(PHYS.stand) };
  forcesGroup.add(forceSets.pose, forceSets.stand);
}
// in the round: arrows of the nearest computed frame (floor forces come from physics; the centre of mass from geometry)
let cycForce = null, cycForceI = -1;
function cycForceSet(i) {
  const ph = { floor: [], grips: [], com: [0, 0, 0], support: { poly: [] } };
  const BW = PHYS.dog.bw;
  if (CYC.floorA) for (let k = 0; k < 10; k++) {
    const o = (i * 10 + k) * 6, a = CYC.floorA;
    if (!a[o]) continue;
    ph.floor.push({ F: [a[o + 1], a[o + 2], a[o + 3]], cop: [a[o + 4] / 1000, 0, a[o + 5] / 1000], pct: a[o + 2] / BW * 100 });
  }
  if (CYC.gripsA) for (let k = 0; k < 2; k++) {
    const o = (i * 2 + k) * 6, a = CYC.gripsA, F = [a[o + 3], a[o + 4], a[o + 5]];
    const N = Math.hypot(...F); if (N > 0) ph.grips.push({ P: [a[o] / 1000, a[o + 1] / 1000, a[o + 2] / 1000], F, N });
  }
  if (CYC.comA) ph.com = [CYC.comA[i * 3] / 1000, CYC.comA[i * 3 + 1] / 1000, CYC.comA[i * 3 + 2] / 1000];
  else if (poseW) { const c = A.comOf(poseW); ph.com = [c.x, c.y, c.z]; }
  // support area: hull of the points of pressure
  const P = ph.floor.map(f => [f.cop[0], f.cop[2]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (P.length >= 3) {
    const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lo = [], up = [];
    for (const p of P) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
    for (let k = P.length - 1; k >= 0; k--) { const p = P[k]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
    lo.pop(); up.pop(); ph.support.poly = lo.concat(up);
  }
  return buildForceSet(ph);
}
function updateForces() {
  if (!forceSets) return;
  if (state.mode === 'practice') {
    forceSets.pose.visible = forceSets.stand.visible = false;
    if (cycForce) cycForce.visible = false;
    return;
  }
  if (state.mode === 'cycle' && CYC) {
    forceSets.pose.visible = forceSets.stand.visible = false;
    const on = state.forces && !cycMorph;
    const i = Math.round(state.cycT * CYC.fps);
    if (on && i !== cycForceI) {
      if (cycForce) { forcesGroup.remove(cycForce); disposeGroup(cycForce); }
      cycForce = cycForceSet(i); forcesGroup.add(cycForce); cycForceI = i;
    }
    if (cycForce) cycForce.visible = on;
    return;
  }
  if (cycForce) cycForce.visible = false;
  const on = state.forces && !morph;
  forceSets.pose.visible = on && state.t > 0.985;
  forceSets.stand.visible = on && state.t < 0.015;
}

// ---------- model groups ----------
const model = new THREE.Group(); scene.add(model);
// ---------- fascia and Myers' lines (fascia3.js / fascia_render.js, built from the atlases and Anatomy Trains) ----------
const FASC = typeof FASCIA_LIB !== 'undefined' && FASCIA_MODEL ? FASCIA_LIB.createFascia(A, THREE, FASCIA_MODEL) : null;
const FR = FASC ? createFasciaRender(FASC, A, THREE, { xray: true }) : null;
if (FR) { FR.setLayer('sheets', false); FR.setLayer('bands', false); FR.setLayer('lines', false); FR.setLayer('stations', false); model.add(FR.group); }
let fasciaM = null, fasciaAt = 0;
const segGroup = {};
for (const s of A.SEG) { const g = new THREE.Group(); model.add(g); segGroup[s.name] = g; }

// ---------- bone geometry ----------
const V3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
function profAt(prof, t) {
  for (let i = 1; i < prof.length; i++) {
    if (t <= prof[i][0] + 1e-9) {
      const [t0, r0] = prof[i - 1], [t1, r1] = prof[i];
      const u = (t - t0) / ((t1 - t0) || 1), s = u * u * (3 - 2 * u);
      return r0 + (r1 - r0) * s;
    }
  }
  return prof[prof.length - 1][1];
}
function latheGeo(a, b, prof, radial) {
  const dir = b.clone().sub(a); const len = dir.length();
  const pts = [new THREE.Vector2(0, 0)];
  const n = 10;
  for (let i = 0; i <= n; i++) { const t = i / n; pts.push(new THREE.Vector2(Math.max(profAt(prof, t), 0.0006), t * len)); }
  pts.push(new THREE.Vector2(0, len));
  const g = new THREE.LatheGeometry(pts, radial || 12);
  g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize()));
  g.translate(a.x, a.y, a.z);
  return g;
}
function ellGeo(c, r) { const g = new THREE.SphereGeometry(1, 20, 14); g.scale(r[0], r[1], r[2]); g.translate(c.x, c.y, c.z); return g; }
function tubeGeo(pts, r) { const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal'); return new THREE.TubeGeometry(curve, Math.max(8, pts.length * 5), r, 7, false); }
function torusGeo(c, R, r, n, arc, rot) {
  const g = new THREE.TorusGeometry(R, r, 8, 28, (arc || 360) * Math.PI / 180);
  if (rot) g.rotateZ(rot * Math.PI / 180);
  g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), n.clone().normalize()));
  g.translate(c.x, c.y, c.z);
  return g;
}
function plateGeo(pts, hub, th) {
  const n = new THREE.Vector3();
  for (let i = 0; i < pts.length; i++) { const a = pts[i].clone().sub(hub), b = pts[(i + 1) % pts.length].clone().sub(hub); n.add(a.cross(b)); }
  n.normalize();
  const off = n.clone().multiplyScalar(th / 2);
  const pos = [];
  const P = v => pos.push(v.x, v.y, v.z);
  const T = p => p.clone().add(off), Bt = p => p.clone().sub(off);
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i], b = pts[(i + 1) % pts.length];
    P(T(hub)); P(T(a)); P(T(b));
    P(Bt(hub)); P(Bt(b)); P(Bt(a));
    P(T(a)); P(Bt(a)); P(Bt(b));
    P(T(a)); P(Bt(b)); P(T(b));
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(pos.length / 3 * 2), 2));
  g.computeVertexNormals();
  return g;
}
function norm(g) {
  const ng = g.index ? g.toNonIndexed() : g;
  for (const k of Object.keys(ng.attributes)) if (!['position', 'normal', 'uv'].includes(k)) ng.deleteAttribute(k);
  if (!ng.attributes.normal) ng.computeVertexNormals();
  if (!ng.attributes.uv) ng.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(ng.attributes.position.count * 2), 2));
  return ng;
}

const GOLD = new THREE.Color(0xffc04a);
const MAT = {
  bone: new THREE.MeshStandardMaterial({ color: 0xd8ccb3, roughness: 0.66, metalness: 0 }),
  dark: new THREE.MeshStandardMaterial({ color: 0x2c2723, roughness: 0.95, metalness: 0 }),
};
const cartMat = () => new THREE.MeshPhysicalMaterial({ color: 0xcbd8d8, roughness: 0.36, metalness: 0, clearcoat: 0.35, clearcoatRoughness: 0.3, emissive: GOLD, emissiveIntensity: 0 });
const discMat = () => new THREE.MeshStandardMaterial({ color: 0x7b9fb3, roughness: 0.4, metalness: 0, transparent: true, opacity: 0.9, emissive: GOLD, emissiveIntensity: 0 });
const buckets = new Map();
/*__SKULL__*/
const SKULL_OLD = new Set(['Череп', 'Нижняя челюсть', 'Скуловая дуга', 'Сосцевидный отросток']);
for (const b of A.BONES) {
  const J = A.S[b.seg].J;
  const L = arr => V3(arr).sub(J);
  let g;
  if (b.kind === 'ell') g = ellGeo(L(b.c), b.r);
  else if (b.kind === 'lathe') g = latheGeo(L(b.a), L(b.b), b.prof);
  else if (b.kind === 'tube') g = tubeGeo(b.pts.map(L), b.r);
  else if (b.kind === 'plate') g = plateGeo(b.pts.map(L), L(b.hub), b.th);
  else if (b.kind === 'torus') g = torusGeo(L(b.c), b.R, b.r, V3(b.n), b.arc, b.rot);
  const mat = b.mat || 'bone';
  const jointKey = b.joint ? (b.joint + '|' + (b.vert || '') + '|' + (b.side || '')) : '';
  const oldSkull = b.group === 'skull' && SKULL_OLD.has(b.name);
  const keyName = (oldSkull ? 'OLDSKULL|' : '') + b.seg + '|' + mat + '|' + b.name + '|' + jointKey;
  if (!buckets.has(keyName)) buckets.set(keyName, { oldSkull, seg: b.seg, mat, name: b.name, latin: b.latin, joint: b.joint, vert: b.vert, side: b.side, geos: [] });
  buckets.get(keyName).geos.push(norm(g));
}
const boneMeshes = [], cartMeshes = [], discMeshes = [];
const glowByJoint = {};   // joint id (with side) -> meshes
const vertToJoint = {};   // vertebra name -> spine joint id
{
  const order = ['L5', 'L4', 'L3', 'L2', 'L1', 'Th12', 'Th11', 'Th10', 'Th9', 'Th8', 'Th7', 'Th6', 'Th5', 'Th4', 'Th3', 'Th2', 'Th1', 'C7', 'C6', 'C5', 'C4', 'C3', 'C2', 'C1'];
  const map = { L5: 'ls', L4: 'ls', L3: 'l34', L2: 'l34', L1: 'tl', Th12: 'tl', Th11: 'tl', Th10: 't89', Th9: 't89', Th8: 't89', Th7: 't89', Th6: 't45', Th5: 't45', Th4: 't45', Th3: 't45', Th2: 'ct', Th1: 'ct', C7: 'ct', C6: 'c45', C5: 'c45', C4: 'c45', C3: 'ao', C2: 'ao', C1: 'ao' };
  for (const n of order) vertToJoint[n] = map[n];
}
for (const bk of buckets.values()) {
  const geo = mergeGeometries(bk.geos, false);
  let mesh;
  if (bk.mat === 'cart') { mesh = new THREE.Mesh(geo, cartMat()); mesh.renderOrder = 2; cartMeshes.push(mesh); }
  else if (bk.mat === 'disc') { mesh = new THREE.Mesh(geo, discMat()); discMeshes.push(mesh); }
  else { mesh = new THREE.Mesh(geo, MAT[bk.mat]); boneMeshes.push(mesh); }
  let jid = null;
  if (bk.joint === 'facet' || bk.joint === 'disc') jid = vertToJoint[bk.vert] || null;
  else if (bk.joint) jid = bk.side ? bk.joint + '_' + bk.side : bk.joint;
  mesh.userData = { kind: bk.mat === 'cart' || bk.mat === 'disc' ? 'cart' : 'bone', name: bk.name, latin: bk.latin, joint: jid, seg: bk.seg, baseOpacity: mesh.material.opacity };
  if (bk.mat === 'bone' || bk.mat === 'dark') mesh.userData.procBone = true;
  if (jid) (glowByJoint[jid] ||= []).push(mesh);
  segGroup[bk.seg].add(mesh);
}

// Retired geometry must leave both the scene and the picking/glow registries.
// Otherwise a detached schematic surface can still intercept a tap.
function removeModelMesh(mesh) {
  mesh.removeFromParent();
  for (const list of [boneMeshes, cartMeshes, discMeshes, ...Object.values(glowByJoint)]) {
    const i = list.indexOf(mesh);
    if (i >= 0) list.splice(i, 1);
  }
  mesh.geometry.dispose();
  if (mesh.userData.kind === 'cart') mesh.material.dispose();
}

// real bones: BodyParts3D (DBCLS, CC BY 4.0), decimated and fitted to the rig segments at build time.
// The schematic bones of a segment stay until its real bones are unpacked, then they are hidden.
const BONE_DATA = /*__BONEDATA__*/null;
async function addRealBones() {
  const D = BONE_DATA;
  if (!D || typeof DecompressionStream === 'undefined') return;
  const bin = Uint8Array.from(atob(D.data), c => c.charCodeAt(0));
  const buf = await new Response(new Blob([bin]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
  const boneMat = new THREE.MeshStandardMaterial({ color: 0xd8ccb3, roughness: 0.62, metalness: 0 });
  const toothMat = new THREE.MeshStandardMaterial({ color: 0xf1ead8, roughness: 0.35, metalness: 0 });
  const covered = new Set(D.parts.map(p => p.seg));
  // schematic cartilage that the real bones replace or that no longer sits on a bone surface
  const CART_GONE = new Set(['Лобковый симфиз', 'Мечевидный отросток', 'Рёберный хрящ', 'Хрящ головки плюсневой кости', 'Дугоотростчатый сустав']);
  for (const seg of covered) for (const g of segGroup[seg].children.slice()) if (g.userData && g.userData.kind === 'cart' && CART_GONE.has(g.userData.name)) removeModelMesh(g);
  for (const seg of covered) for (const g of segGroup[seg].children.slice()) if (g.userData && g.userData.procBone) removeModelMesh(g);
  let off = 0;
  for (const p of D.parts) {
    const dq = new Uint16Array(buf, off, p.n * 3); off += p.n * 6;
    const di = new Uint16Array(buf, off, p.m * 3); off += p.m * 6;
    const pos = new Float32Array(p.n * 3), idx = new Uint16Array(p.m * 3);
    const acc = [0, 0, 0];
    for (let i = 0; i < p.n; i++) for (let a = 0; a < 3; a++) { acc[a] = (acc[a] + dq[3 * i + a]) & 0xFFFF; pos[3 * i + a] = p.lo[a] + (p.hi[a] - p.lo[a]) * acc[a] / 65535; }
    let ia = 0;
    for (let i = 0; i < p.m * 3; i++) { ia = (ia + di[i]) & 0xFFFF; idx[i] = ia; }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setIndex(new THREE.BufferAttribute(idx, 1));
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, p.latin === 'dentes' ? toothMat : boneMat);
    mesh.userData = { kind: 'bone', name: p.name, latin: p.latin, joint: null };
    mesh.visible = state.bones;
    boneMeshes.push(mesh);
    segGroup[p.seg].add(mesh);
  }
  window.__skullReady = true;
}
setTimeout(() => { addRealBones().catch(() => { window.__skullReady = true; }); }, 0);

// ---------- joints (pick targets) ----------
const jointPick = [], jointById = {};
const pickMat = new THREE.MeshBasicMaterial({ visible: false });
for (const j of A.JOINTS) {
  const J = A.S[j.seg].J;
  const c = V3(j.at).sub(J);
  const pick = new THREE.Mesh(new THREE.SphereGeometry(j.type === 'st' ? 0.045 : Math.max(j.r * 1.6, 0.022), 10, 8), pickMat);
  pick.position.copy(c); pick.userData = { kind: 'joint', id: j.id };
  segGroup[j.seg].add(pick);
  jointPick.push(pick);
  jointById[j.id] = { def: j, pick, k: 0, res: null };
}

// ---------- strands ----------
const NSEG = 26, NRAD = 10;
const MUSCLE = new THREE.Color(0xa8434b), TENDON = new THREE.Color(0xe9dfcd);
const LIG = new THREE.Color(0xe6dfd0), CAP = new THREE.Color(0xd9d2c4);
const WARM = new THREE.Color(0xff8a2a), COOL = new THREE.Color(0x36b9ff);
// working muscles are green: the belly takes the green colour, the glow leans to green
const WORK = new THREE.Color(0x3fdc86), MUSCLE_ON = new THREE.Color(0x2c9a5a);
function fiberTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256;
  const g = c.getContext('2d'), pixels = g.createImageData(c.width, c.height);
  // Fascicles run along V. The previous eight-pixel texture varied only in V,
  // making every muscle look like a corrugated hose at phone zoom levels.
  // Periodic, low-contrast strands avoid both transverse bars and repeat seams;
  // mipmaps keep the fine detail from shimmering when the camera pulls back.
  for (let y = 0; y < c.height; y++) for (let x = 0; x < c.width; x++) {
    const u = 2 * Math.PI * x / c.width, v = 2 * Math.PI * y / c.height;
    const warp = 0.32 * Math.sin(v) + 0.12 * Math.sin(3 * v + 0.8);
    const fiber = Math.sin(23 * u + warp) + 0.45 * Math.sin(47 * u + 0.3 * Math.sin(2 * v));
    const grain = Math.sin(79 * u + 37 * v) * Math.sin(31 * u - 53 * v);
    const value = Math.round(246 + 3.4 * fiber + 0.8 * grain), k = 4 * (y * c.width + x);
    pixels.data[k] = pixels.data[k + 1] = pixels.data[k + 2] = value;
    pixels.data[k + 3] = 255;
  }
  g.putImageData(pixels, 0, 0);
  const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
  return t;
}
const fibers = fiberTexture();
const smooth = (a, b, x) => { const t = Math.max(0, Math.min(1, (x - a) / (b - a))); return t * t * (3 - 2 * t); };
function shapeAt(u, belly, dips) {
  const g = u < belly ? 0.5 * u / belly : 0.5 + 0.5 * (u - belly) / (1 - belly);
  let s = Math.pow(Math.max(0, Math.sin(Math.PI * Math.max(0, Math.min(1, g)))), 0.8);
  if (dips) for (const d of dips) s *= 1 - 0.45 * Math.exp(-Math.pow((u - d) / 0.03, 2));
  return s;
}
class Strand {
  constructor(inst) {
    this.inst = inst;
    const nV = (NSEG + 1) * NRAD;
    this.pos = new Float32Array(nV * 3); this.nor = new Float32Array(nV * 3);
    const uv = new Float32Array(nV * 2), col = new Float32Array(nV * 3), colOn = new Float32Array(nV * 3);
    this.prof = [];
    const isMuscle = inst.kind === 'muscle';
    for (let i = 0; i <= NSEG; i++) {
      const u = i / NSEG;
      const sh = isMuscle ? shapeAt(u, inst.belly, inst.dips) : 1;
      this.prof.push(isMuscle ? 0.17 + 0.83 * sh : 1);
      const c = isMuscle ? TENDON.clone().lerp(MUSCLE, smooth(0.1, 0.45, sh)) : (inst.kind === 'lig' ? LIG : CAP);
      const cOn = isMuscle ? TENDON.clone().lerp(MUSCLE_ON, smooth(0.1, 0.45, sh)) : c;
      for (let j = 0; j < NRAD; j++) { const k = i * NRAD + j; uv[k * 2] = j / NRAD; uv[k * 2 + 1] = u; col[k * 3] = c.r; col[k * 3 + 1] = c.g; col[k * 3 + 2] = c.b; colOn[k * 3] = cOn.r; colOn[k * 3 + 1] = cOn.g; colOn[k * 3 + 2] = cOn.b; }
    }
    const idx = [];
    for (let i = 0; i < NSEG; i++) for (let j = 0; j < NRAD; j++) {
      const a = i * NRAD + j, b = i * NRAD + (j + 1) % NRAD, c = (i + 1) * NRAD + j, d = (i + 1) * NRAD + (j + 1) % NRAD;
      idx.push(a, b, c, b, d, c);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
    g.setAttribute('normal', new THREE.BufferAttribute(this.nor, 3));
    g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    g.setIndex(idx);
    this.geo = g;
    this.colOff = col.slice(); this.colOn = colOn; this.on = false;
    if (isMuscle) this.mat = new THREE.MeshStandardMaterial({ vertexColors: true, map: fibers, roughness: 0.7, metalness: 0, emissive: 0x000000, emissiveIntensity: 0 });
    else if (inst.kind === 'lig') this.mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.35, metalness: 0, emissive: 0x000000, emissiveIntensity: 0 });
    else this.mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.45, metalness: 0, transparent: true, opacity: 0.5, depthWrite: false, emissive: 0x000000, emissiveIntensity: 0 });
    this.mesh = new THREE.Mesh(g, this.mat);
    this.mesh.frustumCulled = false;
    if (inst.kind === 'cap') this.mesh.renderOrder = 1;
    this.mesh.userData = { kind: inst.kind, unit: inst.unit, side: inst.side };
    this.center = [];
    this.L0 = 1; this.strain = 0;
    this.flatDir = new THREE.Vector3();
  }
  setOn(on) {
    if (on === this.on) return;
    this.on = on;
    const a = this.geo.attributes.color; a.array.set(on ? this.colOn : this.colOff); a.needsUpdate = true;
  }
  update(pts, rScale, flatDir, widths) {
    const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');
    curve.arcLengthDivisions = 60;
    const P = curve.getSpacedPoints(NSEG);
    this.center = P;
    const T = [], N = new THREE.Vector3(), B = new THREE.Vector3(), q = new THREE.Quaternion();
    for (let i = 0; i <= NSEG; i++) {
      const a = P[Math.max(0, i - 1)], b = P[Math.min(NSEG, i + 1)];
      const t = b.clone().sub(a); if (t.lengthSq() < 1e-12) t.set(0, 1, 0); T.push(t.normalize());
    }
    const t0 = T[0];
    const useFlat = flatDir && flatDir.lengthSq() > 1e-8;
    if (useFlat) { N.copy(flatDir).addScaledVector(t0, -flatDir.dot(t0)); if (N.lengthSq() < 1e-8) N.crossVectors(t0, Math.abs(t0.x) < 0.6 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0)); N.normalize(); }
    else { const ax = Math.abs(t0.x) < 0.6 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0); N.crossVectors(t0, ax).normalize(); }
    const r0 = this.inst.r * rScale, flat = this.inst.flat;
    const sheet = !!widths;
    for (let i = 0; i <= NSEG; i++) {
      if (i > 0) {
        if (useFlat) { N.copy(flatDir).addScaledVector(T[i], -flatDir.dot(T[i])); if (N.lengthSq() < 1e-8) { q.setFromUnitVectors(T[i - 1], T[i]); N.applyQuaternion(q); } N.normalize(); }
        else { q.setFromUnitVectors(T[i - 1], T[i]); N.applyQuaternion(q); N.addScaledVector(T[i], -N.dot(T[i])).normalize(); }
      }
      B.crossVectors(T[i], N);
      const r = r0 * this.prof[i] + 0.0007;
      let rN = r, rB = r * flat;
      if (sheet) {
        const u = i / NSEG;
        const w = widths[0] + (widths[1] - widths[0]) * u;
        rN = Math.max(w, r * 0.6); rB = r;
      }
      for (let j = 0; j < NRAD; j++) {
        const th = j / NRAD * Math.PI * 2, cs = Math.cos(th), sn = Math.sin(th);
        const k = (i * NRAD + j) * 3;
        this.pos[k] = P[i].x + N.x * cs * rN + B.x * sn * rB;
        this.pos[k + 1] = P[i].y + N.y * cs * rN + B.y * sn * rB;
        this.pos[k + 2] = P[i].z + N.z * cs * rN + B.z * sn * rB;
        const nx = N.x * cs * rB + B.x * sn * rN, ny = N.y * cs * rB + B.y * sn * rN, nz = N.z * cs * rB + B.z * sn * rN;
        const il = 1 / (Math.hypot(nx, ny, nz) || 1);
        this.nor[k] = nx * il; this.nor[k + 1] = ny * il; this.nor[k + 2] = nz * il;
      }
    }
    this.geo.attributes.position.needsUpdate = true;
    this.geo.attributes.normal.needsUpdate = true;
    this.geo.computeBoundingSphere();
  }
}
const strands = insts.map(i => new Strand(i));
for (const s of strands) model.add(s.mesh);
{
  const P0 = A.poseAt(confs.dog, 0);
  for (const s of strands) s.L0 = A.pathLength(A.strandPath(P0.W, s.inst));
}
const muscles = strands.filter(s => s.inst.kind === 'muscle');
const ligs = strands.filter(s => s.inst.kind === 'lig');
const caps = strands.filter(s => s.inst.kind === 'cap');
const UNIT_IDS = Object.keys(A.UNITS);
const unitStrands = {}, unitSideStrands = {}, ligStrands = {}, capStrands = {};
for (const s of muscles) {
  (unitStrands[s.inst.unit] ||= []).push(s);
  (unitSideStrands[s.inst.unit + '_' + s.inst.side] ||= []).push(s);
}
for (const s of ligs) (ligStrands[s.inst.unit] ||= []).push(s);
for (const s of caps) (capStrands[s.inst.unit + '_' + s.inst.side] ||= []).push(s);

// ---------- state ----------
const state = { t: 1, poseId: 'dog', bones: true, joints: true, ligs: true, musclesOuter: true, musclesDeep: true, light: true, forces: false, lightMode: 'length', sel: null, playing: null, mode: 'pose', cycT: 0, fascia: false, flines: false, algorithmId: 'struna', practiceStep: 0 };
let muscleSurfaces = null, connectiveSurfaces = null, jointSurfaces = null;
// physics: muscle effort from the static solution, at the full pose and in plain standing
const ACTIVE_MIN = 0.08;
const activeOf = id => new Set(Object.entries(PHYS[id].muscles).filter(([, m]) => m.a >= ACTIVE_MIN).map(([u]) => u));
let activeSet = activeOf('dog');
const lerpN = (a, b, t) => a + (b - a) * t;
function effortOf(unit, id) { if (state.mode === 'practice') return 0; const p = PHYS[id || state.poseId].muscles[unit], s0 = PHYS.stand.muscles[unit]; return lerpN(s0 ? s0.a : 0, p ? p.a : 0, state.t); }
function jointPhys(id, pose) { if (state.mode === 'practice') return null; return PHYS[pose || state.poseId].joints[id.replace(/_R$/, '_L')] || null; }
function jointEffortOf(id) {
  // loads under 1 N·m are ignored: a small moment against a weak muscle group would light the joint for nothing
  const big = c => (Math.abs(c.need) >= 1 ? c.eff : 0);
  const f = pose => { const J = jointPhys(id, pose); if (!J) return 0; let e = 0; if (J.comps) for (const c of J.comps) e = Math.max(e, big(c)); if (J.arm) e = Math.max(e, big(J.arm.elev), big(J.arm.rot)); return e; };
  return lerpN(f('stand'), f(state.poseId), state.t);
}
let morph = null, displayConf = null;
const isDeep = s => !!A.UNITS[s.inst.unit].deep;
const muscleVisible = s => isDeep(s) ? state.musclesDeep : state.musclesOuter;
const surfaceReplaces = s => s.inst.kind === 'muscle'
  ? !!(muscleSurfaces && muscleSurfaces.covered.has(s.inst.unit + '_' + s.inst.side))
  : !!(connectiveSurfaces && connectiveSurfaces.covered.has(s.inst.kind + '_' + s.inst.unit + '_' + s.inst.side));
function syncSurfaceVisibility() {
  for (const s of strands) {
    const on = s.inst.kind === 'muscle' ? muscleVisible(s) : s.inst.kind === 'lig' ? state.ligs : state.joints;
    s.mesh.visible = !!on && !surfaceReplaces(s);
  }
  if (muscleSurfaces) muscleSurfaces.setVisibility(unit => A.UNITS[unit].deep ? state.musclesDeep : state.musclesOuter);
  if (connectiveSurfaces) connectiveSurfaces.setVisibility(part => part.kind === 'lig' ? state.ligs : state.joints);
  if (jointSurfaces) jointSurfaces.setVisibility(state.joints);
}
function updateJointSourceNote(status) {
  const note = document.getElementById('joint-source-note');
  if (!note) return;
  const capParts = connectiveSurfaces ? connectiveSurfaces.parts.filter(part => part.kind === 'cap') : [];
  const fittedCaps = capParts.filter(part => part.displayFit === 'bone-rim').length;
  note.textContent = nb(status === 'ready'
    ? 'Хрящи, мениски и суставные губы основных суставов — геометрические реконструкции по готовым костям. ' + (fittedCaps === capParts.length && fittedCaps ? 'Края капсул подогнаны к поверхности костей. ' : 'Часть капсул показана условной схемой. ') + 'Контакт и сжатие тканей не моделируются.'
    : 'Показана схема суставных поверхностей: уточнённые формы в этом браузере недоступны. Связки и капсулы — геометрические реконструкции.');
}
function jointSourceHTML(id) {
  if (!jointSurfaces || !jointSurfaces.parts.some(part => part.joint === id)) return '';
  return `<p class="meter-foot">${nb('Суставные поверхности — геометрические реконструкции по костям BodyParts3D. Подсветка капсулы показывает изменение длины отдельного расчётного пути. Сжатие, скольжение менисков и контакт тканей не рассчитываются.')}</p>`;
}
function loadedMuscleParts(unit) {
  if (!muscleSurfaces) return [];
  return (MUSCLE_SURFACE_DATA.parts || []).filter(part => (!unit || part.unit === unit) && muscleSurfaces.covered.has(part.unit + '_' + part.side));
}
function updateMuscleSourceNote(status) {
  const note = document.getElementById('muscle-source-note');
  if (!note) return;
  if (status === 'ready') {
    const real = new Set(), reconstructed = new Set();
    for (const part of loadedMuscleParts()) (part.source === 'reconstructed' ? reconstructed : real).add(part.unit);
    note.textContent = nb('Объёмные формы: ' + real.size + ' мышечных юнитов по BodyParts3D; ' + reconstructed.size + ' — геометрические реконструкции. Движение тканей — приближение модели.');
  } else if (status === 'fallback') note.textContent = nb('Показана схема мышечных пучков: объёмные формы в этом браузере недоступны.');
}
function muscleSourceHTML(unit) {
  const parts = loadedMuscleParts(unit);
  if (!parts.length) return '';
  const reconstructed = parts.some(part => part.source === 'reconstructed');
  const real = parts.some(part => part.source !== 'reconstructed');
  const source = reconstructed ? (real ? 'BodyParts3D и геометрическая реконструкция' : 'геометрическая реконструкция') : 'BodyParts3D';
  return `<p class="meter-foot">${nb('Объёмная форма: ' + source + '. Движение тканей — приближение модели.')}</p>`;
}
let poseW = null, poseLocal = null, regions = null;
const centerTarget = new THREE.Vector3(0, 0.4, 0);
const FIT_PTS = [['f3c_L', [0.21, 0.63, -0.02]], ['f3c_R', [-0.21, 0.63, -0.02]], ['ulna_L', [0.2, 1.09, -0.04]], ['ulna_R', [-0.2, 1.09, -0.04]], ['head', [0, 1.75, 0.0]], ['head', [0, 1.62, 0.1]], ['pelvis', [0, 0.93, 0]], ['hand_L', [0.21, 0.63, -0.02]], ['hand_R', [-0.21, 0.63, -0.02]], ['toes_L', [0.09, 0.003, 0.2]], ['toes_R', [-0.09, 0.003, 0.2]], ['foot_L', [0.094, 0.01, -0.06]], ['foot_R', [-0.094, 0.01, -0.06]], ['hum_L', [0.21, 1.4, -0.03]], ['hum_R', [-0.21, 1.4, -0.03]], ['pelvis', [0.15, 1.07, -0.03]], ['pelvis', [-0.15, 1.07, -0.03]], ['pelvis', [0, 1.02, -0.135]], ['pelvis', [0.08, 0.93, -0.14]], ['pelvis', [-0.08, 0.93, -0.14]], ['shank_L', [0.098, 0.52, 0.03]], ['shank_R', [-0.098, 0.52, 0.03]]].map(([s, p]) => ({ s, p: new THREE.Vector3(p[0], p[1], p[2]) }));

// a configuration shown instead of the pose (debug hook; a conf with rootP keeps its own place on the mat)
let overrideConf = null;
function applyPose(t) {
  let conf;
  if (state.mode === 'practice') {
    conf = practiceConfs[practiceCurrent().step.id];
  } else if (state.mode === 'cycle' && CYC) {
    conf = fillCycConf(state.cycT, cycConf, cycNow);
    if (cycMorph) conf = blendFull(cycMorph.from, conf, cycMorph.u);
  } else {
    conf = A.blendConf(A.STAND, confs[state.poseId], t);
    // leaving the round the body also slides from its place on the mat to the middle
    if (morph) conf = morph.from.rootP ? blendFull(morph.from, conf, morph.u) : A.blendConf(morph.from, conf, morph.u);
  }
  if (overrideConf) conf = overrideConf;
  displayConf = conf;
  const P = { local: conf.local, W: conf.rootP ? A.computeWorld(conf.local, conf.rootQ, conf.rootP) : A.placeW(conf.local, conf.rootQ) };
  poseW = P.W; poseLocal = P.local;
  for (const s of A.SEG) { const w = P.W[s.name]; segGroup[s.name].position.copy(w.p); segGroup[s.name].quaternion.copy(w.q); }
  const fd = new THREE.Vector3();
  const pathCache = new Map();
  for (const s of strands) {
    const pts = A.strandPath(P.W, s.inst);
    pathCache.set(s.inst.id, pts);
    const L = A.pathLength(pts);
    s.strain = L / s.L0 - 1;
    // Keep the analytic paths for cards and forces. Loaded surfaces replace only
    // the tube geometry, so there is no duplicate rendering or duplicated work.
    if (surfaceReplaces(s)) { s.center = pts; continue; }
    let flatDir = null, widths = null;
    if (s.inst.flatRef) {
      const nO = A.pt(P.W, s.inst.flatRef.seg, s.inst.flatRef.p);
      fd.copy(nO).sub(pts[0]); flatDir = fd;
      if (s.inst.sheet && s.inst.flatRefIns) {
        const nI = A.pt(P.W, s.inst.flatRefIns.seg, s.inst.flatRefIns.p);
        const perp = (nb, p, a, b) => { const tg = b.clone().sub(a).normalize(); const off = nb.clone().sub(p); off.addScaledVector(tg, -off.dot(tg)); return off.length() * 0.56; };
        const n = pts.length;
        widths = [Math.min(0.04, perp(nO, pts[0], pts[0], pts[1])), Math.min(0.04, perp(nI, pts[n - 1], pts[n - 2], pts[n - 1]))];
      }
    }
    else if (s.inst.flat < 1) { fd.copy(pts[0]).sub(centerTarget); fd.y = 0; flatDir = fd; }
    s.update(pts, s.inst.kind === 'muscle' ? Math.sqrt(s.L0 / L) : 1, flatDir, widths);
  }
  if (muscleSurfaces) muscleSurfaces.update(P.W, pathCache);
  if (connectiveSurfaces) connectiveSurfaces.update(P.W, pathCache);
  if (jointSurfaces) jointSurfaces.update(P.W);
  for (const id in jointById) { const jj = jointById[id]; jj.res = A.analyzeJoint(jj.def, P.local, P.W, state.mode === 'pose' && STATS[state.poseId] ? STATS[state.poseId].joint[id].mode : undefined); jj.k = Math.max(0, Math.min(1, Math.abs(jj.res.primary || 0) / (jj.res.comps[0] ? jj.res.comps[0].rom : 90))); }
  regions = A.analyzeRegions(P.local);
  const box = new THREE.Box3();
  for (const f of FIT_PTS) box.expandByPoint(A.pt(P.W, f.s, f.p));
  box.getCenter(centerTarget);
  const size = box.getSize(new THREE.Vector3());
  shadow.scale.set(Math.max(0.5, size.x + 0.45), Math.max(0.5, size.z + 0.45), 1);
  shadow.position.x = centerTarget.x; shadow.position.z = centerTarget.z;
  updateFascia(P.W);
}
// fascia follows the body; while the round plays it is refreshed about 15 times a second (≈ 18 ms per update)
function fasciaPhys() {
  if (state.mode === 'practice') return null;
  if (state.mode === 'cycle' && CYC) {
    if (!CYC.NU) return null;
    const a = {}, Fmax = {};
    for (const u of UNIT_IDS) { a[u] = { L: cycAct(u, 'L'), R: cycAct(u, 'R') }; const m = PHYS.stand.muscles[u]; Fmax[u] = m ? m.Fmax : 0; }
    return { a, Fmax, Fp: null };
  }
  const ph = PHYS[state.t > 0.5 ? state.poseId : 'stand'];
  return ph ? FASC.physFromStatic(ph) : null;
}
function updateFascia(W, force) {
  if (!FR || !(state.fascia || state.flines)) return;
  const now = performance.now();
  const moving = (state.mode === 'cycle' && cyc && cyc.playing) || morph || cycMorph || state.playing;
  if (!force && moving && now - fasciaAt < 66) return;
  fasciaAt = now;
  fasciaM = FASC.withPhysics(FASC.measure(W || poseW, { paths: true }), fasciaPhys());
  FR.update(W || poseW, fasciaM, { mode: state.lightMode === 'effort' ? 'effort' : 'length', light: state.light, reduceMotion });
  // all twelve lines at once are a tangle: without a line in focus they are drawn fainter
  if (!(state.sel && (state.sel.kind === 'fline' || state.sel.kind === 'fascia'))) for (const l of FR.lines) if (l.mat.opacity !== 0.5) { l.mat.opacity = 0.5; l.mat.needsUpdate = true; }
}

// ---------- glow ----------
const avg = arr => arr.reduce((a, s) => a + s.strain, 0) / arr.length;
const unitAvg = u => avg(unitStrands[u]);
function glowK(strain) { return Math.pow(Math.min(1, Math.abs(strain) / 0.4), 0.8); }
function setOpacity(mat, want, base) {
  const o = Math.max(0, Math.min(1, want * base));
  if (mat.opacity !== o) { mat.opacity = o; mat.transparent = o < 1; mat.depthWrite = o >= 1 && base >= 1; mat.needsUpdate = true; }
}
const surfaceEmissive = new THREE.Color();
function updateMaterials(time) {
  const pulse = reduceMotion ? 1 : 0.72 + 0.28 * Math.sin(time * 0.0042);
  const sel = state.sel;
  const selUnit = sel && sel.kind === 'muscle' ? sel.unit : null;
  const selLig = sel && sel.kind === 'lig' ? sel.unit : null;
  const selJoint = sel && sel.kind === 'joint' ? sel.id : null;
  const selJointBase = selJoint ? selJoint.replace(/_[LR]$/, '') : null;
  const effMode = state.lightMode === 'effort';
  const cyc = state.mode === 'cycle' && CYC;
  for (const s of muscles) {
    // in the round each side has its own effort (stepping and reaching are not symmetric)
    const ac = cyc ? cycAct(s.inst.unit, s.inst.side) : 0;
    // a muscle works when its effort reaches the threshold: in the round by the frame, in a pose by the slider (full pose = the working set)
    const aNow = cyc ? ac : effortOf(s.inst.unit);
    const active = aNow >= ACTIVE_MIN;
    s.setOn(active);
    if (state.light) {
      if (effMode) { s.mat.emissive.copy(WORK); s.mat.emissiveIntensity = 1.3 * Math.pow(Math.min(1, aNow / 0.5), 0.75) * (active ? pulse : 1); }
      else if (active) { s.mat.emissive.copy(s.strain >= 0 ? WARM : COOL).lerp(WORK, 0.9); s.mat.emissiveIntensity = Math.max(1.15 * glowK(s.strain), 0.8) * pulse; }
      else { s.mat.emissive.copy(s.strain >= 0 ? WARM : COOL); s.mat.emissiveIntensity = 1.15 * glowK(s.strain); }
    }
    else s.mat.emissiveIntensity = 0;
    const faded = (selUnit && s.inst.unit !== selUnit) || (sel && !selUnit);
    setOpacity(s.mat, faded ? (selUnit ? 0.14 : 0.25) : 1, 1);
  }
  if (muscleSurfaces) muscleSurfaces.setAppearance((unit, side) => {
    const a = cyc ? cycAct(unit, side) : effortOf(unit);
    const active = a >= ACTIVE_MIN;
    const sameSide = unitSideStrands[unit + '_' + side];
    const strain = sameSide.reduce((sum, s) => sum + s.strain, 0) / sameSide.length;
    const emissive = surfaceEmissive.copy(effMode ? WORK : strain >= 0 ? WARM : COOL);
    let intensity = 0;
    if (state.light) {
      if (effMode) intensity = 1.3 * Math.pow(Math.min(1, a / 0.5), 0.75) * (active ? pulse : 1);
      else if (active) { emissive.lerp(WORK, 0.9); intensity = Math.max(1.15 * glowK(strain), 0.8) * pulse; }
      else intensity = 1.15 * glowK(strain);
    }
    const faded = (selUnit && unit !== selUnit) || (sel && !selUnit);
    return { active, emissive, intensity, opacity: faded ? (selUnit ? 0.14 : 0.25) : 1 };
  });
  for (const s of ligs) {
    const k = s.strain > 0.02 ? glowK(s.strain) : 0;
    if (state.light) { s.mat.emissive.copy(WARM); s.mat.emissiveIntensity = 0.7 * k; } else s.mat.emissiveIntensity = 0;
    const own = selLig === s.inst.unit || (selJointBase && A.LIGS[s.inst.unit].joint === selJointBase);
    const faded = sel && !own;
    setOpacity(s.mat, faded ? 0.25 : 1, 1);
  }
  for (const s of caps) {
    const k = s.strain > 0.02 ? glowK(s.strain) : 0;
    if (state.light) { s.mat.emissive.copy(WARM); s.mat.emissiveIntensity = 1.0 * k; } else s.mat.emissiveIntensity = 0;
    const own = selJoint && (s.inst.unit + '_' + s.inst.side) === selJoint;
    setOpacity(s.mat, sel && !own ? 0.35 : 1, own ? 0.75 : 0.5);
  }
  if (connectiveSurfaces) connectiveSurfaces.setAppearance(part => {
    const k = part.strain > 0.02 ? glowK(part.strain) : 0;
    part.mat.emissive.copy(WARM);
    part.mat.emissiveIntensity = state.light ? (part.kind === 'cap' ? 1 : 0.7) * k : 0;
    if (part.kind === 'lig') {
      const own = selLig === part.unit || (selJointBase && A.LIGS[part.unit].joint === selJointBase);
      setOpacity(part.mat, sel && !own ? 0.25 : 1, 1);
    } else {
      const own = selJoint && part.unit + '_' + part.side === selJoint;
      setOpacity(part.mat, sel && !own ? 0.35 : 1, own ? 0.75 : 0.5);
    }
  });
  for (const id in jointById) {
    const jj = jointById[id];
    const meshes = glowByJoint[id] || [];
    const selected = selJoint === id;
    // in the round the joints glow by the force in the joint (full light at 3 body weights)
    const kj = effMode ? (cyc ? Math.min(1, cycJointLoad(id) / 3) : Math.min(1, jointEffortOf(id) / 0.5)) : jj.k;
    for (const m of meshes) {
      m.material.emissiveIntensity = state.light ? 0.9 * Math.pow(kj, 0.85) : 0;
      const base = m.userData.baseOpacity ?? 0.9;
      setOpacity(m.material, selected ? 1.3 : (sel && !selected ? 0.6 : 1), base);
    }
  }
  bloom.enabled = state.light;
}

// ---------- camera ----------
let VIEW_DIR = new THREE.Vector3(...POSE.dog.view).normalize();
// in the round the camera keeps one distance that fits every frame, and follows the middle of the body
const CYCLE_VIEW = new THREE.Vector3(-1, 0.3, 0.24).normalize();
let cycDist = { key: '', d: 0 };
function cycleFitDistance() {
  const key = camera.aspect.toFixed(3) + '|' + VIEW_DIR.toArray().map(v => v.toFixed(3)).join(',');
  if (cycDist.key === key) return cycDist.d;
  const vf = camera.fov * Math.PI / 180, tv = Math.tan(vf / 2), th = tv * camera.aspect;
  const fwd = VIEW_DIR.clone().negate();
  const right = new THREE.Vector3().crossVectors(fwd, new THREE.Vector3(0, 1, 0)).normalize();
  const up = new THREE.Vector3().crossVectors(right, fwd).normalize();
  const tmp = newConf(), box = new THREE.Box3(), ctr = new THREE.Vector3();
  let need = 0.6;
  for (let f = 0; f < CYC.n; f += 3) {
    const c = fillCycConf(f / CYC.fps, tmp);
    const W = A.computeWorld(c.local, c.rootQ, c.rootP);
    box.makeEmpty();
    const pts = FIT_PTS.map(fp => A.pt(W, fp.s, fp.p));
    for (const p of pts) box.expandByPoint(p);
    box.getCenter(ctr);
    for (const p0 of pts) { const p = p0.clone().sub(ctr); need = Math.max(need, (Math.abs(p.dot(right)) + 0.1) / th + p.dot(VIEW_DIR), (Math.abs(p.dot(up)) + 0.1) / tv + p.dot(VIEW_DIR)); }
  }
  cycDist = { key, d: need * 1.03 };
  return cycDist.d;
}
function fitDistance() {
  if (state.mode === 'cycle' && CYC) return cycleFitDistance();
  const vf = camera.fov * Math.PI / 180;
  const tv = Math.tan(vf / 2), th = tv * camera.aspect;
  const fwd = VIEW_DIR.clone().negate();
  const right = new THREE.Vector3().crossVectors(fwd, new THREE.Vector3(0, 1, 0)).normalize();
  const up = new THREE.Vector3().crossVectors(right, fwd).normalize();
  let need = 0.6;
  const pad = 0.1;
  for (const f of FIT_PTS) {
    const p = A.pt(poseW, f.s, f.p).sub(centerTarget);
    const x = Math.abs(p.dot(right)) + pad, y = Math.abs(p.dot(up)) + pad, z = p.dot(VIEW_DIR);
    need = Math.max(need, x / th + z, y / tv + z);
  }
  return need * 1.03;
}
function resetView(animate) {
  const target = centerTarget.clone();
  const pos = target.clone().addScaledVector(VIEW_DIR, fitDistance());
  if (!animate || reduceMotion) { controls.target.copy(target); camera.position.copy(pos); controls.update(); return; }
  const fromP = camera.position.clone(), fromT = controls.target.clone(), t0 = performance.now();
  state.camAnim = now => {
    const u = Math.min(1, (now - t0) / 650), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
    camera.position.lerpVectors(fromP, pos, e); controls.target.lerpVectors(fromT, target, e);
    if (u >= 1) state.camAnim = null;
  };
}
function focusOn(worldPos, dist) {
  state.userMoved = true;
  const dir = camera.position.clone().sub(controls.target).normalize();
  const target = worldPos.clone(), pos = target.clone().addScaledVector(dir, dist);
  if (reduceMotion) { controls.target.copy(target); camera.position.copy(pos); return; }
  const fromP = camera.position.clone(), fromT = controls.target.clone(), t0 = performance.now();
  state.camAnim = now => {
    const u = Math.min(1, (now - t0) / 600), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
    camera.position.lerpVectors(fromP, pos, e); controls.target.lerpVectors(fromT, target, e);
    if (u >= 1) state.camAnim = null;
  };
}
function resize() {
  const w = stage.clientWidth, h = stage.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  composer.setSize(w, h);
  bloom.resolution.set(w / 2, h / 2);
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
new ResizeObserver(() => { resize(); if (poseW && !state.userMoved && !state.camAnim) resetView(false); }).observe(stage);

// ---------- picking ----------
const ray = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let downAt = null;
canvas.addEventListener('pointerdown', e => { downAt = { x: e.clientX, y: e.clientY, t: performance.now() }; hideHint(); });
canvas.addEventListener('pointerup', e => {
  if (!downAt) return;
  const moved = Math.hypot(e.clientX - downAt.x, e.clientY - downAt.y), dt = performance.now() - downAt.t;
  downAt = null;
  if (moved > 8 || dt > 600) return;
  pick(e.clientX, e.clientY);
});
controls.addEventListener('start', () => { state.userMoved = true; hideHint(); });
function pick(cx, cy) {
  const r = canvas.getBoundingClientRect();
  ndc.set(((cx - r.left) / r.width) * 2 - 1, -((cy - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(ndc, camera);
  const objs = [];
  for (const s of muscles) if (s.mesh.visible) objs.push(s.mesh);
  if (state.ligs) for (const s of ligs) if (s.mesh.visible) objs.push(s.mesh);
  if (state.joints) { objs.push(...jointPick); for (const mesh of [...cartMeshes, ...discMeshes]) if (mesh.visible && mesh.parent) objs.push(mesh); for (const s of caps) if (s.mesh.visible) objs.push(s.mesh); }
  if (muscleSurfaces) for (const mesh of muscleSurfaces.pickables) if (mesh.visible) objs.push(mesh);
  if (connectiveSurfaces) for (const mesh of connectiveSurfaces.pickables) if (mesh.visible) objs.push(mesh);
  if (state.bones) objs.push(...boneMeshes);
  if (FR && (state.fascia || state.flines)) for (const m of FR.pickables) if (m.visible) objs.push(m);
  const hits = ray.intersectObjects(objs, false);
  const first = hits[0];
  if (first) {
    const d = first.object.userData;
    if (d.kind === 'fascia') return select({ kind: 'fascia', id: d.id, side: d.side });
    if (d.kind === 'line' || d.kind === 'station') return select({ kind: 'fline', id: d.id || d.line, side: d.side });
    if (d.kind === 'muscle' || d.kind === 'lig' || d.kind === 'joint') return select(d);
    if (d.kind === 'cap') return select({ kind: 'joint', id: d.unit + '_' + d.side });
    if (d.kind === 'cart' && d.joint) return select({ kind: 'joint', id: d.joint });
  }
  const hitDepth = first ? first.distance : Infinity;
  let best = null;
  const p = new THREE.Vector3();
  const scan = (list) => {
    for (const s of list) {
      for (let i = 2; i < s.center.length - 2; i += 2) {
        p.copy(s.center[i]); const d = p.distanceTo(camera.position);
        if (d > hitDepth + 0.02) continue;
        p.project(camera);
        const sx = (p.x + 1) / 2 * r.width + r.left, sy = (1 - p.y) / 2 * r.height + r.top;
        const dd = Math.hypot(sx - cx, sy - cy);
        if (dd < 16 && (!best || dd < best.dd)) best = { dd, data: s.mesh.userData };
      }
    }
  };
  scan(muscles.filter(s => s.mesh.visible));
  if (state.ligs) scan(ligs.filter(s => s.mesh.visible));
  if (best) return select(best.data.kind === 'cap' ? { kind: 'joint', id: best.data.unit + '_' + best.data.side } : best.data);
  if (first) return select(first.object.userData);
  select(null);
}

// ---------- UI ----------
const $ = id => document.getElementById(id);
const info = $('info');
const cuesEl = $('cues');
const fmtPct = s => { const v = Math.round(s * 100); return (v > 0 ? '+' : v < 0 ? '−' : '±') + Math.abs(v) + '%'; };
const fmtDeg = a => Math.round(Math.abs(a)) + '°';
const sideWord = id => /_L$/.test(id) ? 'слева' : /_R$/.test(id) ? 'справа' : '';
function strainWord(s) {
  const a = Math.abs(s);
  if (a < 0.04) return 'почти не меняется';
  if (s > 0) return a > 0.25 ? 'сильно удлинена' : 'удлинена';
  return a > 0.25 ? 'сильно укорочена' : 'укорочена';
}
function tensionWord(s) {
  if (s < 0.02) return 'расслаблена';
  if (s < 0.12) return 'слегка натянута';
  if (s < 0.35) return 'натянута';
  return 'сильно натянута';
}
function barHTML(s, scale) {
  const sc = scale || 0.4;
  const clamp = Math.max(-sc, Math.min(sc, s));
  const x = 50 + clamp / sc * 50;
  const left = Math.min(50, x), width = Math.abs(x - 50);
  const cls = s >= 0 ? 'warm' : 'cool';
  return `<div class="bar" aria-hidden="true"><span class="bar-zero"></span><span class="bar-fill ${cls}" style="left:${left}%;width:${width}%"></span></div>`;
}
function romBar(v, rom) {
  const k = Math.max(0, Math.min(1.25, Math.abs(v) / rom));
  return `<div class="bar rom" aria-hidden="true"><span class="bar-fill ${k > 1 ? 'over' : 'gold'}" style="left:0;width:${Math.min(100, k * 80)}%"></span><span class="bar-mark" style="left:80%"></span></div>`;
}

const kgf = n => Math.round(n / 9.81);
const fBW = v => (Math.round(v * 10) / 10).toFixed(1).replace('.', ',');
const effBar = (e, cls) => `<div class="bar" aria-hidden="true"><span class="bar-fill ${cls || 'warm'}" style="left:0;width:${Math.min(100, Math.round(e * 100))}%"></span></div>`;
const pctTxt = e => Math.round(e * 100) + '%';

// ---------- per-pose statistics at the full pose ----------
const STATS = {};
function computeStats(id) {
  const c = confs[id];
  const W = A.placeW(c.local, c.rootQ);
  const unit = {}, lig = {}, cu = {}, cl = {};
  for (const s of strands) {
    if (s.inst.kind === 'cap') continue;
    const e = A.pathLength(A.strandPath(W, s.inst)) / s.L0 - 1;
    if (s.inst.kind === 'muscle') { unit[s.inst.unit] = (unit[s.inst.unit] || 0) + e; cu[s.inst.unit] = (cu[s.inst.unit] || 0) + 1; }
    else { lig[s.inst.unit] = (lig[s.inst.unit] || 0) + e; cl[s.inst.unit] = (cl[s.inst.unit] || 0) + 1; }
  }
  for (const u in unit) unit[u] /= cu[u];
  for (const u in lig) lig[u] /= cl[u];
  const joint = {};
  for (const jid in jointById) joint[jid] = A.analyzeJoint(jointById[jid].def, c.local, W);
  const m = A.measure(id, c.local, W);
  Object.assign(m, EXTRAS[id] || {});
  return { unit, lig, joint, regions: A.analyzeRegions(c.local), m };
}

// ---------- cues card ----------
function cueList(list, m, numbered) {
  const tag = numbered ? 'ol' : 'ul';
  return `<${tag} class="cues${numbered ? ' cues-num' : ''}">${list.map(c => {
    let v = null;
    try { v = c.m ? c.m(m) : null; } catch (e) { v = null; }
    return `<li><span class="cue-t">${nb(c.t)}</span>${v ? `<span class="cue-m num-soft">${nb(v)}</span>` : ''}</li>`;
  }).join('')}</${tag}>`;
}
const REGION_GROUP = { 'ладони': 'Кисти', 'пальцы рук': 'Кисти', 'кулаки': 'Кисти', 'кисти': 'Кисти', 'предплечья': 'Предплечья', 'локти': 'Предплечья', 'пятки': 'Пятки', 'подушечки стоп': 'Передний отдел стоп', 'пальцы стоп': 'Пальцы стоп', 'тыл стоп': 'Тыл стоп', 'наружный край стопы': 'Передний отдел стоп', 'нижние рёбра': 'Нижние рёбра', 'грудь': 'Грудь', 'живот': 'Живот', 'таз': 'Таз', 'бёдра': 'Бёдра', 'задняя поверхность бёдер': 'Бёдра', 'колени': 'Колени', 'голени': 'Голени', 'ягодицы': 'Ягодицы', 'икры': 'Икры', 'спина': 'Спина', 'голова': 'Голова', 'плечи': 'Плечи' };
const FOOT_GROUPS = new Set(['Пятки', 'Передний отдел стоп', 'Пальцы стоп', 'Тыл стоп']);
function floorGroups(ph) {
  const g = {};
  const feetOnly = ph.floor.every(f => FOOT_GROUPS.has(REGION_GROUP[f.region]));
  for (const f of ph.floor) {
    let k = REGION_GROUP[f.region] || f.region;
    if (!feetOnly && FOOT_GROUPS.has(k)) k = 'Стопы';
    const o = (g[k] ||= { name: k, pct: 0, fx: 0, fz: 0 });
    o.pct += f.pct; o.fx += f.F[0]; o.fz += f.F[2];
  }
  return Object.values(g).filter(o => o.pct >= 0.5).sort((a, b) => b.pct - a.pct);
}
function physicsLoadHTML(P, ph) {
  const gs = floorGroups(ph);
  const tot = gs.reduce((a, o) => a + o.pct, 0) || 1;
  const segs = gs.map((o, i) => `<span class="fl-seg fl-${i % 4}" style="width:${(o.pct / tot * 100).toFixed(1)}%"></span>`).join('');
  const legend = gs.map((o, i) => `<li><i class="fl-dot fl-${i % 4}" aria-hidden="true"></i><span>${nb(o.name)}</span><b class="num">${Math.round(o.pct)}%</b><span class="num-soft">${kgf(o.pct / 100 * ph.bw)}\u00a0кг</span></li>`).join('');
  let html = `<div class="load" role="img" aria-label="${nb(gs.map(o => o.name + ' ' + Math.round(o.pct) + ' процентов веса').join(', '))}"><div class="fl-bar">${segs}</div><ul class="fl-legend">${legend}</ul></div>`;
  // friction: supports pushing the mat in opposite directions
  const hz = gs.map(o => ({ o, h: Math.hypot(o.fx, o.fz) })).filter(x => x.h > 0.03 * ph.bw).sort((a, b) => b.h - a.h);
  if (hz.length >= 2) {
    // friction close to the load on the support: the mat is at the limit of its grip (static friction about 1)
    const GEN = { 'Кисти': 'кистей', 'Стопы': 'стоп', 'Предплечья': 'предплечий', 'Пальцы стоп': 'пальцев стоп', 'Передний отдел стоп': 'переднего отдела стоп', 'Пятки': 'пяток', 'Тыл стоп': 'тыла стоп' };
    const lim = hz.find(x => GEN[x.o.name] && x.h >= 0.85 * x.o.pct / 100 * ph.bw);
    const ratio = Math.max(...hz.map(x => x.h / Math.max(1, x.o.pct / 100 * ph.bw)));
    let t = hz[0].o.name + ' и ' + hz[1].o.name.toLowerCase() + ' упираются в коврик в разные стороны, сила трения около ' + kgf(hz[0].h) + '\u00a0кг.';
    if (lim) t += ' У ' + GEN[lim.o.name] + ' трение почти равно давлению на коврик: это предел сцепления хорошего коврика, на скользком полу опоры разъехались бы.';
    else if (ratio < 0.3) t += ' Это немного: такое усилие удержит и гладкий пол.';
    else t += ' Без цепкого коврика они бы разъехались.';
    html += `<p class="meter-foot">${nb(t)}</p>`;
  }
  const mg = ph.support.margin;
  html += `<p class="meter-foot">${nb(mg > 0 ? 'Центр масс над площадью опоры, до её края ' + fcmUI(mg) + '.' : 'Центр масс на краю опоры.')}</p>`;
  return html;
}
const fcmUI = v => { const c = Math.abs(v) * 100; return (c < 10 ? c.toFixed(1).replace('.', ',') : String(Math.round(c))) + '\u00a0см'; };
function supportHTML(P, S) {
  const m = S.m, sup = m.support;
  let html = physicsLoadHTML(P, PHYS[P.id]);
  if (P.id === 'fold') {
    const k = Math.round(m.comPct * 100);
    html += `<div class="load" role="img" aria-label="Центр масс над точкой ${k} процентов длины стопы от пятки"><div class="foot-bar"><span class="foot-mid"></span><span class="foot-mark" style="left:${k}%"></span></div><div class="load-legend"><span>пятка</span><span>центр масс <b class="num">${k}%</b></span><span>пальцы</span></div></div>`;
  }
  if (P.extra) html += `<p class="meter-foot">${nb(P.extra(m) + '.')}</p>`;
  return html;
}
function renderCues() {
  if (state.mode === 'practice') return renderPracticeCues();
  const P = POSE[state.poseId], S = STATS[state.poseId];
  cuesEl.innerHTML = `
    <p class="card-eyebrow">${nb('Подсказки к позе ' + P.num)}</p>
    ${P.steps ? `<h3 class="card-sub first">${nb(P.stepsTitle)}</h3>${cueList(P.steps, S.m, true)}` : ''}
    <h3 class="card-sub${P.steps ? '' : ' first'}">${nb(P.cuesTitle)}</h3>${cueList(P.cues, S.m, false)}
    <h3 class="card-sub">${nb('Нагрузка')}</h3>${supportHTML(P, S)}`;
}

// ---------- summary and detail cards ----------
const ZONES = [['shoulder', 'Плечи'], ['arm', 'Руки'], ['trunk', 'Корпус'], ['leg', 'Таз и ноги']];
const KEY_JOINTS = ['gh_L', 'st_L', 'el_L', 'wr_L', 'hip_L', 'kn_L', 'an_L', 'mtp_L'];
function mainComp(res) { return res.comps.slice().sort((a, b) => Math.abs(b.value) / (b.rom || 1) - Math.abs(a.value) / (a.rom || 1))[0]; }
// The two scenario solves cover endpoints only. They do not describe a
// confidence interval, an intermediate slider frame, or a frame of the round.
function researchPoseKey() {
  if (state.mode !== 'pose' || morph || overrideConf) return null;
  if (state.t <= 1e-8) return 'stand';
  return state.t >= 1 - 1e-8 ? state.poseId : null;
}
let lastResearchKey = null;
const researchNumber = (value, digits = 1) => Number(value).toFixed(digits).replace('.', ',');
function researchCase() {
  const key = researchPoseKey(), data = key && RESEARCH_SENSITIVITY && RESEARCH_SENSITIVITY.poses[key];
  const solved = q => q && q.status === 'solved' && q.converged !== false && q.finite !== false;
  return data && solved(data.quality && data.quality.cue) && solved(data.quality && data.quality.noCue) ? { key, data } : null;
}
function researchComparisonHTML(kind, id, compact = false) {
  const current = researchCase();
  if (!current) return '';
  const record = kind === 'muscle' ? current.data.muscles[id] : current.data.joints[id.replace(/_R$/, '_L')];
  if (!record) return '';
  const percent = n => researchNumber(n * 100) + '%';
  const force = n => researchNumber(n, 0) + '\u00a0Н';
  const bw = n => researchNumber(n, 2);
  const specs = kind === 'muscle'
    ? [['a', 'Доля максимальной силы', percent], ['forceN', 'Активная сила', force]]
    : record.compressionN ? [['compressionN', compact ? 'L5–S1: сжатие' : 'Сжатие', force], ...(!compact ? [['shearN', 'Сдвигающая сила', force]] : [])]
      : [['bw', 'Реакция сустава, в весах тела', bw], ...(!compact ? [['forceN', 'Сила реакции', force]] : [])];
  const rows = specs.map(([key, title, format]) => {
    const metric = record[key];
    if (!metric || !Array.isArray(metric.range) || metric.range.length !== 2 || ![metric.cue, metric.noCue, ...metric.range].every(Number.isFinite)) return '';
    return `<div class="research-row" data-research-metric="${key}" data-cue="${metric.cue}" data-no-cue="${metric.noCue}" data-low="${metric.range[0]}" data-high="${metric.range[1]}">
      <dt>${nb(title)}</dt><dd>${nb(format(metric.cue))}</dd><dd>${nb(format(metric.noCue))}</dd>
      <dd class="research-range">${nb('Диапазон двух сценариев: ' + format(metric.range[0]) + '–' + format(metric.range[1]))}</dd></div>`;
  }).filter(Boolean).join('');
  if (!rows) return '';
  return `<section class="research-comparison" data-research-pose="${current.key}" data-research-kind="${kind}" data-research-id="${id}">
    <h3 class="card-sub">${nb('Как подсказки меняют оценку')}</h3>
    <p class="meter-foot">${nb(current.key === 'stand' ? 'Обычная стойка. Два расчёта одной позы:' : 'Полная поза. Два расчёта одного положения:')}</p>
    <div class="research-columns" aria-hidden="true"><span></span><span>${nb('С подсказками')}</span><span>${nb('Без подсказок')}</span></div>
    <dl class="research-values">${rows}</dl>
    <p class="meter-foot">${nb('Меняется только минимальный тонус, заданный подсказками. Это диапазон этих двух сценариев, не доверительный интервал и не оценка персональной точности.')}</p>
    ${kind === 'muscle' ? `<p class="meter-foot">${nb('Доля максимальной силы — не внутренняя активация решателя и не измеренная ЭМГ.')}</p>` : ''}
  </section>`;
}
function updateResearchQuality() {
  const node = document.getElementById('model-quality');
  if (!node) return;
  const lines = [];
  if (state.mode === 'practice') {
    node.textContent = nb('3D-реконструкция по фото. Нагрузка для этих поз пока не рассчитана.');
    return;
  }
  const small = (n, unit) => Math.abs(n) < 0.01 ? 'менее 0,01 ' + unit : researchNumber(Math.abs(n), 2) + ' ' + unit;
  if (state.mode === 'cycle') {
    const q = CYC && CYC.modelQuality;
    lines.push('Круг рассчитан по кадрам как последовательность статических равновесий. Инерция не учитывается; диапазон двух сценариев для круга не рассчитывался.');
    if (q && q.verified && Number.isFinite(q.frameCount)) lines.push('Численное равновесие проверено для ' + q.frameCount + ' расчётных кадров.');
    if (q && Number.isFinite(q.maxForceResidualN) && Number.isFinite(q.maxMomentResidualNm)) lines.push('Наибольший остаток баланса сил: ' + small(q.maxForceResidualN, 'Н') + '; моментов: ' + small(q.maxMomentResidualNm, 'Н·м') + '.');
    if (q && Number.isFinite(q.maxReserveTotalNm)) lines.push('Наибольшая сумма резервных моментов за кадр: ' + researchNumber(q.maxReserveTotalNm, 1) + ' Н·м. Резервы показывают, где одних рассчитанных мышц недостаточно для равновесия.');
    if (q && q.forceLengthLimitedFrames > 0) lines.push('В части кадров мышцы выходят за диапазон упрощённой кривой «сила–длина»; это ограничивает оценку их усилия.');
  } else {
    const key = researchPoseKey();
    if (!key) lines.push('В промежуточном положении усилия интерполированы. Отдельное равновесие и диапазон двух сценариев здесь не рассчитывались.');
    else {
      const ph = PHYS[key], diag = ph && ph.diag, q = diag && diag.quality;
      lines.push(q ? (q.status === 'solved' && q.converged && q.finite ? 'Численное равновесие прошло проверки решения.' : 'Численное решение не прошло все проверки; это ограничение показанных оценок.') : 'Диагностика численного решения для этой позы недоступна.');
      if (q && Number.isFinite(q.forceResidualN) && Number.isFinite(q.momentResidualNm)) lines.push('Остаток баланса сил: ' + small(q.forceResidualN, 'Н') + '; моментов: ' + small(q.momentResidualNm, 'Н·м') + '.');
      if (diag && Number.isFinite(diag.reserves)) lines.push('Сумма резервных моментов: ' + researchNumber(diag.reserves, 1) + ' Н·м. Резервы дополняют мышцы и отражают ограничение модели.');
      const limits = ph && ph.modelLimits;
      if (limits && limits.forceLengthClampedUnits && limits.forceLengthClampedUnits.length) lines.push('У части мышц длина выходит за диапазон упрощённой кривой «сила–длина». Это ограничение модели мышц, даже когда равновесие решено.');
    }
  }
  node.innerHTML = lines.map(line => `<p class="meter-foot">${nb(line)}</p>`).join('');
}
function renderSummary() {
  const P = POSE[state.poseId], S = STATS[state.poseId];
  const byZone = z => UNIT_IDS.filter(u => (z === 'shoulder' ? ['shoulder', 'arm'].includes(A.UNITS[u].zone) : A.UNITS[u].zone === z));
  const stretchGroups = [['shoulder', 'Плечи и руки'], ['trunk', 'Корпус'], ['leg', 'Таз и ноги']].map(([z, label]) => ({
    label, items: byZone(z).map(u => ({ u, s: S.unit[u] })).filter(o => o.s > 0.05).sort((a, b) => b.s - a.s).slice(0, 4),
  })).filter(g => g.items.length);
  const ph = PHYS[P.id];
  // small muscles (under ~15 kg of force) reach high percentages with little force; they stay in their own cards
  const working = Object.entries(ph.muscles).filter(([, m]) => m.a >= ACTIVE_MIN && m.Fmax >= 150).sort((a, b) => b[1].a - a[1].a).slice(0, 9);
  const mainLoad = J => { if (J.arm) return J.arm.elev.eff >= J.arm.rot.eff ? J.arm.elev : J.arm.rot; return (J.comps || []).slice().sort((a, b) => b.eff - a.eff)[0]; };
  const jointLoad = [...KEY_JOINTS, 'ls', 'tl', 't89', 'ct'].map(id => ({ id, e: jointEffortOf(id), J: jointPhys(id) })).filter(o => o.J && o.e >= 0.05 && mainLoad(o.J) && Math.abs(mainLoad(o.J).need) >= 1).sort((a, b) => b.e - a.e).slice(0, 6);
  const spine = ph.joints.ls && ph.joints.ls.jr, spine0 = PHYS.stand.joints.ls.jr;
  const ligTop = Object.keys(A.LIGS).map(u => ({ u, s: S.lig[u] })).filter(o => o.s > 0.08).sort((a, b) => b.s - a.s).slice(0, 6);
  info.innerHTML = `
    <p class="card-eyebrow">${nb('Что меняется относительно стойки')}</p>
    <h2 class="card-title">${nb('Сильнее всего удлиняются')}</h2>
    ${stretchGroups.map(g => `<h3 class="card-sub">${nb(g.label)}</h3><ul class="rank">${g.items.map(o => `<li><button class="rank-row" data-unit="${o.u}"><span class="rank-name">${nb(A.UNITS[o.u].name)}</span><span class="num warm-t">${fmtPct(o.s)}</span>${barHTML(o.s)}</button></li>`).join('')}</ul>`).join('')}
    <h3 class="card-sub">${nb('Работают сильнее всего в полной позе, доля максимальной силы')}</h3>
    <ul class="rank">${working.map(([u, m]) => `<li><button class="rank-row" data-unit="${u}"><span class="rank-name">${nb(A.UNITS[u].name)}</span><span class="num work-t">${pctTxt(m.a)}</span>${effBar(m.a, 'work')}${m.tone > 0 ? `<span class="rank-sub">${nb('Для мышцы задан минимальный тонус по подсказке; итоговая сила определяется совместным расчётом.')}</span>` : ''}</button></li>`).join('')}</ul>
    <h3 class="card-sub">${nb('Нагрузка на суставы, доля силы окружающих мышц')}</h3>
    <ul class="rank">${jointLoad.map(o => { const c = mainLoad(o.J); return `<li><button class="rank-row" data-joint="${o.id}"><span class="rank-name">${nb(jointById[o.id].def.name)}</span><span class="num gold-t">${pctTxt(o.e)}</span>${effBar(o.e)}<span class="rank-sub">${nb((c.group ? c.group + ', ' : '') + Math.round(Math.abs(c.need)) + ' Н·м')}</span></button></li>`; }).join('')}</ul>
    ${spine ? `<p class="meter-foot">${nb('В полной позе диск L5–S1 сжат силой ' + fBW(spine.comp / ph.bw) + ' веса тела, около ' + kgf(spine.comp) + '\u00a0кг. В стойке ' + fBW(spine0.comp / PHYS.stand.bw) + '.')}</p>` : ''}
    ${researchComparisonHTML('joint', 'ls', true)}
    ${ligTop.length ? `<h3 class="card-sub">${nb('Связки под натяжением')}</h3><ul class="rank">${ligTop.map(o => `<li><button class="rank-row" data-lig="${o.u}"><span class="rank-name">${nb(A.LIGS[o.u].name)}</span><span class="word warm-t">${nb(tensionWord(o.s))}</span></button></li>`).join('')}</ul>` : ''}
    <h3 class="card-sub">${nb('Суставы')}</h3>
    <ul class="joints">${KEY_JOINTS.map(id => { const j = jointById[id], c = mainComp(S.joint[id]); return `<li><button class="rank-row" data-joint="${id}"><span class="rank-name">${nb(j.def.name)}</span><span class="num gold-t">${fmtDeg(c.value)}</span><span class="rank-sub">${nb(c.label.toLowerCase() + ', норма до ' + c.rom + '°')}</span></button></li>`; }).join('')}</ul>
    <h3 class="card-sub">${nb('Позвоночник, сумма по отделам')}</h3>
    <ul class="joints">${Object.keys(S.regions).map(k => { const r = S.regions[k]; return `<li><button class="rank-row" data-joint="${A.REGION[k].ids[0]}"><span class="rank-name">${nb(r.name)}</span><span class="num gold-t">${fmtDeg(r.signed)}</span><span class="rank-sub">${nb((r.signed < -0.2 ? 'разгибание' : r.signed > 0.2 ? 'сгибание' : 'нейтраль') + ', норма до ' + r.rom + '°')}</span></button></li>`; }).join('')}</ul>
    <p class="card-hint">${nb('Нажми на мышцу, связку, хрящ или капсулу на модели, чтобы открыть карточку.')}</p>`;
  bindCardButtons();
}
function bindCardButtons() {
  info.querySelectorAll('[data-unit]').forEach(b => b.addEventListener('click', () => select({ kind: 'muscle', unit: b.dataset.unit, side: 'L' })));
  info.querySelectorAll('[data-lig]').forEach(b => b.addEventListener('click', () => select({ kind: 'lig', unit: b.dataset.lig, side: 'L' })));
  info.querySelectorAll('[data-joint]').forEach(b => b.addEventListener('click', () => select({ kind: 'joint', id: b.dataset.joint })));
  const c = info.querySelector('.card-close'); if (c) c.addEventListener('click', () => select(null));
  const f = info.querySelector('.card-focus'); if (f) f.addEventListener('click', () => focusSelection());
}
function closeBtn() { return `<button class="card-close" aria-label="Закрыть карточку"><svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg></button>`; }
function focusBtn() { return `<button class="btn btn-sm card-focus"><svg viewBox="0 0 24 24" width="14" height="14" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="11" cy="11" r="6"/><path d="M20 20l-4.5-4.5"/></svg><span>Показать на модели</span></button>`; }
function poseNote(text) { const P = POSE[state.poseId]; return text ? `<h3 class="card-sub">${nb('В позе «' + P.name + '»')}</h3><p class="role">${nb(text)}</p>` : ''; }
function effortBlock(unit) {
  const m = PHYS[state.poseId].muscles[unit];
  if (!m) return '';
  const a = effortOf(unit);
  let foot;
  if (m.a >= 0.01) {
    foot = 'В полной позе ' + pctTxt(m.a) + ' максимальной силы, тянет примерно на ' + kgf(m.F) + '\u00a0кг';
    foot += m.Fp >= 20 ? '. Растянутые волокна добавляют пассивное натяжение ещё ' + kgf(m.Fp) + '\u00a0кг.' : '.';
  } else foot = m.Fp >= 20 ? 'Сама почти не работает, но растянутые волокна тянут пассивно примерно на ' + kgf(m.Fp) + '\u00a0кг.' : 'В полной позе по расчёту почти не работает.';
  if (m.tone > 0) foot += ' Для мышцы задан минимальный тонус по подсказке; итоговая сила определяется совместным расчётом.';
  return `<h3 class="card-sub">${nb('Доля максимальной силы по расчёту')}</h3>
    <div class="meter"><span class="meter-label">${nb('Сейчас')}</span><span class="num work-t big">${pctTxt(a)}</span></div>
    ${effBar(a, 'work')}
    <p class="meter-foot">${nb(foot)}</p>`;
}
function renderMuscle(unit) {
  const U = A.UNITS[unit], P = POSE[state.poseId];
  const s = unitAvg(unit), sf = STATS[state.poseId].unit[unit];
  const cls = s >= 0 ? 'warm' : 'cool';
  const parts = unitStrands[unit].filter(x => x.inst.side === 'L');
  const partRows = parts.length > 1 ? `<ul class="parts">${parts.map((p, i) => `<li><span>${nb(p.inst.label || ('пучок ' + (i + 1)))}</span><span class="num ${p.strain >= 0 ? 'warm' : 'cool'}-t">${fmtPct(p.strain)}</span>${barHTML(p.strain)}</li>`).join('')}</ul>` : '';
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Мышца')}</p>
    <h2 class="card-title">${nb(U.name)}</h2>
    <p class="latin">${U.latin}</p>
    ${muscleSourceHTML(unit)}
    <div class="tags"><span class="tag">${U.deep ? 'глубокая' : 'внешняя'}</span>${effortOf(unit) >= ACTIVE_MIN ? '<span class="tag tag-active">работает</span>' : ''}<span class="tag tag-${cls}">${nb(strainWord(s))}</span></div>
    <div class="meter"><span class="meter-label">${nb('Длина сейчас')}</span><span class="num ${cls}-t big">${fmtPct(s)}</span></div>
    ${barHTML(s)}
    <p class="meter-foot">${nb('В полной позе: ' + fmtPct(sf) + ' к длине в стойке' + (parts.length > 1 ? ', среднее по ' + parts.length + ' пучкам' : ''))}</p>
    ${partRows}
    ${effortBlock(unit)}
    ${researchComparisonHTML('muscle', unit)}
    ${poseNote(P.notes[unit])}
    <h3 class="card-sub">${nb('Что делает')}</h3><p class="role">${nb(U.fn)}</p>
    ${focusBtn()}`;
  bindCardButtons();
}
function renderLig(unit) {
  const Lg = A.LIGS[unit];
  const parts = ligStrands[unit].filter(x => x.inst.side === 'L');
  const s = avg(parts), sf = STATS[state.poseId].lig[unit];
  const partRows = parts.length > 1 ? `<ul class="parts">${parts.map((p, i) => `<li><span>${nb(p.inst.label || ('пучок ' + (i + 1)))}</span><span class="num ${p.strain > 0.02 ? 'warm' : 'muted'}-t">${fmtPct(p.strain)}</span>${barHTML(p.strain)}</li>`).join('')}</ul>` : '';
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Связка')}</p>
    <h2 class="card-title">${nb(Lg.name)}</h2>
    <p class="latin">${Lg.latin}</p>
    <div class="tags"><span class="tag ${s > 0.02 ? 'tag-warm' : ''}">${nb(tensionWord(s))}</span></div>
    <div class="meter"><span class="meter-label">${nb('Расстояние между точками крепления')}</span><span class="num ${s > 0.02 ? 'warm' : 'muted'}-t big">${fmtPct(s)}</span></div>
    ${barHTML(s)}
    <p class="meter-foot">${nb('В полной позе: ' + fmtPct(sf) + '. Связка не растягивается как мышца: в стойке она сложена, а при натяжении распрямляется, поэтому проценты показывают только, где она берёт нагрузку.')}</p>
    ${partRows}
    <h3 class="card-sub">${nb('Что делает')}</h3><p class="role">${nb(Lg.role)}</p>
    ${focusBtn()}`;
  bindCardButtons();
}
// title of a group of muscles: "Мышцы-сгибатели", "Приводящие мышцы", "Наружные ротаторы", "Мышцы вращают плечо внутрь"
function groupTitle(g) {
  const cap = t => t.charAt(0).toUpperCase() + t.slice(1);
  if (/^(сгибатели|разгибатели|пронаторы|супинаторы)$/.test(g)) return 'Мышцы-' + g;
  if (/^(отводящие|приводящие)$/.test(g)) return cap(g) + ' мышцы';
  if (/ротаторы$|сгибатели$/.test(g)) return cap(g);
  return 'Мышцы ' + g;
}
const SPINE_NAMES = { erector: 'разгибатели спины', multifidus: 'многораздельная', psoas: 'большая поясничная', iliacus: 'подвздошная', rectAbd: 'прямая мышца живота', intObl: 'внутренняя косая', extObl: 'наружная косая', ql: 'квадратная мышца поясницы', transAbd: 'поперечная мышца живота' };
function loadRow(c, title) {
  const need = Math.abs(c.need);
  const pasSame = Math.sign(c.passive) === Math.sign(c.need);
  let note = pctTxt(c.eff) + ' их силы';
  if (Math.abs(c.load) >= 1) note += ', вес тела и опора дают ' + Math.round(Math.abs(c.load)) + ' Н·м';
  if (Math.abs(c.passive) >= 1) note += pasSame ? ', пассивные ткани берут на себя ' + Math.round(Math.abs(c.passive)) + ' Н·м' : ', пассивные ткани сопротивляются ещё на ' + Math.round(Math.abs(c.passive)) + ' Н·м';
  return `<li><span>${nb(title)}</span><span class="num warm-t">${Math.round(need)}\u00a0Н·м</span>${effBar(c.eff)}<span class="part-note">${nb(note)}</span></li>`;
}
function jointLoadBlock(id, j) {
  const J = jointPhys(id);
  if (!J) return '';
  const ph = PHYS[state.poseId];
  const rows = [];
  if (J.arm) {
    for (const c of [J.arm.elev, J.arm.rot]) if (Math.abs(c.need) >= 1 || c.eff >= 0.03) rows.push(loadRow(c, groupTitle(c.group)));
  } else if (J.comps) {
    for (const c of J.comps.slice().sort((a, b) => b.eff - a.eff)) if (c.group && (Math.abs(c.need) >= 1 || c.eff >= 0.03) && rows.length < 2) rows.push(loadRow(c, groupTitle(c.group)));
  }
  const foot = [];
  if (J.jr && id !== 'ls' && id !== 'tl') foot.push('Сила, с которой кости давят друг на друга в суставе: ' + fBW(J.jr.bw) + ' веса тела, около ' + kgf(J.jr.F) + '\u00a0кг.');
  if (id === 'ls' && J.jr) {
    foot.push('Диск L5–S1 сжат силой ' + fBW(J.jr.comp / ph.bw) + ' веса тела (около ' + kgf(J.jr.comp) + '\u00a0кг), сдвигающая сила ' + kgf(J.jr.shear) + '\u00a0кг. В стойке ' + fBW(PHYS.stand.joints.ls.jr.comp / PHYS.stand.bw) + '.');
    const top = (J.jr.top || []).map(u => SPINE_NAMES[u] || A.UNITS[u].name.toLowerCase());
    if (J.jr.mus >= 0.5 * J.jr.comp && top.length) foot.push('Мышцы добавляют к сжатию около ' + kgf(J.jr.mus) + '\u00a0кг, больше всего ' + top.join(' и ') + '. Чем сильнее мышцы держат позвоночник, тем сильнее они сжимают диски.');
  }
  if (id === 'tl' && J.jr) foot.push('Диск Th12–L1 сжат силой ' + fBW(J.jr.comp / ph.bw) + ' веса тела.');
  if (J.pf && J.pf.F > 20) foot.push('Надколенник прижат к бедру силой ' + fBW(J.pf.bw) + ' веса тела, около ' + kgf(J.pf.F) + '\u00a0кг.');
  if (J.st) foot.push('Лопатка давит на рёбра силой ' + fBW(J.st.bw) + ' веса тела, около ' + kgf(J.st.F) + '\u00a0кг.');
  if (!rows.length && !foot.length) return '';
  return `<h3 class="card-sub">${nb('Нагрузка в полной позе по расчёту статики')}</h3>${rows.length ? `<ul class="parts comps">${rows.join('')}</ul>` : ''}${foot.map(t => `<p class="meter-foot">${nb(t)}</p>`).join('')}`;
}
function renderJoint(id) {
  const jj = jointById[id], j = jj.def, r = jj.res, rf = STATS[state.poseId].joint[id], P = POSE[state.poseId];
  const comps = r.comps.map((c, i) => `<li><span>${nb(c.label)}</span><span class="num gold-t">${fmtDeg(c.value)}</span>${romBar(c.value, c.rom)}<span class="part-note">${nb('в полной позе ' + fmtDeg(rf.comps[i].value) + (rf.comps[i].label !== c.label ? ' (' + rf.comps[i].label.toLowerCase() + ')' : '') + ', норма до ' + c.rom + '°' + (c.note ? '. ' + c.note.charAt(0).toUpperCase() + c.note.slice(1) : ''))}</span></li>`).join('');
  const reg = j.region ? regions[j.region] : null;
  const regRow = reg ? `<p class="meter-foot">${nb(reg.name + ' целиком: ' + fmtDeg(reg.signed) + ' ' + (reg.signed < -0.2 ? 'разгибания' : reg.signed > 0.2 ? 'сгибания' : '') + ', норма до ' + reg.rom + '°')}</p>` : '';
  const ligList = Object.keys(A.LIGS).filter(u => A.LIGS[u].joint === j.base || (j.type === 'spine' && A.LIGS[u].joint === 'spine'));
  const ligRows = ligList.length ? `<h3 class="card-sub">${nb('Связки этого сустава')}</h3><ul class="rank">${ligList.map(u => { const s = avg(ligStrands[u].filter(x => x.inst.side === (j.side || 'L'))); return `<li><button class="rank-row" data-lig="${u}"><span class="rank-name">${nb(A.LIGS[u].name)}</span><span class="word ${s > 0.02 ? 'warm' : 'muted'}-t">${nb(tensionWord(s))}</span></button></li>`; }).join('')}</ul>` : '';
  const capW = capStrands[id] ? tensionWord(Math.max(...capStrands[id].map(x => x.strain))) : null;
  const capRow = capW ? `<p class="meter-foot">${nb(A.CAP_NAMES[j.base] + ': ' + (capW === 'расслаблена' ? 'расслаблена со всех сторон' : capW + ' с одной стороны, расслаблена с другой'))}</p>` : '';
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb(j.type === 'spine' ? 'Позвоночник' : j.type === 'st' ? 'Лопаточно-грудное скольжение' : 'Сустав')} ${sideWord(id)}</p>
    <h2 class="card-title">${nb(j.name)}</h2>
    <ul class="parts comps">${comps}</ul>
    ${regRow}${capRow}${jointSourceHTML(id)}
    ${jointLoadBlock(id, j)}
    ${researchComparisonHTML('joint', id)}
    ${poseNote(P.joints[j.type === 'st' ? 'st' : j.type])}
    ${ligRows}
    ${focusBtn()}`;
  bindCardButtons();
}
function renderBone(d) {
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Кость')}</p>
    <h2 class="card-title">${nb(d.name)}</h2>
    <p class="latin">${d.latin || ''}</p>
    <p class="role">${nb('Форма кости не меняется. Меняется её положение: смотри соседние суставы, их хрящи подсвечены по углу.')}</p>`;
  bindCardButtons();
}
function renderInfo() {
  renderInfoBase();
  if (state.mode !== 'practice') appendLinesOf();
  lastResearchKey = researchPoseKey();
  updateResearchQuality();
}
function renderInfoBase() {
  const s = state.sel;
  if (state.mode === 'practice') return renderPracticeInfo();
  if (s && s.kind === 'fline') return renderLine(s.id);
  if (s && s.kind === 'fascia') return renderFasciaEl(s.id);
  if (s && s.kind === 'flines') return renderLinesOverview();
  if (state.mode === 'cycle' && CYC) {
    if (!s) return renderCycleNow();
    if (s.kind === 'muscle') return renderMuscleCycle(s.unit);
    if (s.kind === 'lig') return renderLigCycle(s.unit);
    if (s.kind === 'joint') return renderJointCycle(s.id);
    return renderBone(s);
  }
  if (!s) return renderSummary();
  if (s.kind === 'muscle') return renderMuscle(s.unit);
  if (s.kind === 'lig') return renderLig(s.unit);
  if (s.kind === 'joint') return renderJoint(s.id);
  if (s.kind === 'bone' || s.kind === 'cart') return renderBone(s);
}
function select(d) {
  state.sel = d;
  if (FR) {
    if (d && d.kind === 'fline' && !state.flines) setLayer('flines', true);
    if (d && d.kind === 'fascia' && !state.fascia) setLayer('fascia', true);
    FR.focus(d && d.kind === 'fline' ? { line: d.id } : d && d.kind === 'fascia' ? { el: d.id } : null);
    updateFascia(null, true);
  }
  if (d && d.kind === 'muscle') { const deep = !!A.UNITS[d.unit].deep; if (deep && !state.musclesDeep) setLayer('musclesDeep', true); if (!deep && !state.musclesOuter) setLayer('musclesOuter', true); }
  if (d && d.kind === 'lig' && !state.ligs) setLayer('ligs', true);
  if (d && d.kind === 'joint' && !state.joints) setLayer('joints', true);
  renderInfo();
}
function focusSelection() {
  const s = state.sel; if (!s) return;
  const surfaceBounds = s.kind === 'muscle' && muscleSurfaces ? muscleSurfaces.getBounds(s.unit, s.side || 'L')
    : s.kind === 'lig' && connectiveSurfaces ? connectiveSurfaces.getBounds('lig', s.unit, s.side || 'L') : null;
  if (surfaceBounds) {
    const sphere = surfaceBounds.getBoundingSphere(new THREE.Sphere());
    return focusOn(sphere.center, Math.max(s.kind === 'muscle' ? 0.5 : 0.3, sphere.radius * 2.6));
  }
  if ((s.kind === 'fline' || s.kind === 'fascia') && fasciaM) {
    let pts = [];
    const side = s.side === 'R' ? 'R' : 'L';
    if (s.kind === 'fline') {
      const lm = fasciaM.lines[s.id + '_' + side];
      pts = lm?.path || FASC.linePath(poseW, s.id, side);
    } else {
      const e = FASC.EL[s.id], elementSide = e.central ? 'L' : side;
      // Use the resolved anatomical instances, including their reflected points
      // and wraps. Raw source strands describe only the left reference side.
      for (const inst of e.inst[elementSide] || []) pts.push(...A.strandPath(poseW, inst));
      if (!pts.length) for (const m of FR.pickables) {
        if (m.userData.kind !== 'fascia' || m.userData.id !== s.id || m.userData.side !== elementSide) continue;
        m.geometry.computeBoundingSphere();
        pts.push(m.localToWorld(m.geometry.boundingSphere.center.clone()));
      }
    }
    if (pts.length) { const c = new THREE.Vector3(); for (const q of pts) c.add(q); c.divideScalar(pts.length); let rad = 0; for (const q of pts) rad = Math.max(rad, q.distanceTo(c)); return focusOn(c, Math.max(0.45, rad * 2.4)); }
    return;
  }
  let list = null, dist = 0.5;
  if (s.kind === 'muscle') list = unitStrands[s.unit].filter(x => x.inst.side === (s.side || 'L'));
  if (s.kind === 'lig') { list = ligStrands[s.unit].filter(x => x.inst.side === (s.side || 'L')); dist = 0.3; }
  if (list) {
    const c = new THREE.Vector3();
    let n = 0;
    for (const st of list) for (const p of st.center) { c.add(p); n++; }
    c.divideScalar(n || 1);
    let rad = 0; for (const st of list) for (const p of st.center) rad = Math.max(rad, p.distanceTo(c));
    return focusOn(c, Math.max(dist, rad * 2.6));
  }
  if (s.kind === 'joint') { const jj = jointById[s.id]; return focusOn(jj.pick.getWorldPosition(new THREE.Vector3()), jj.def.type === 'spine' ? 0.4 : 0.32); }
}

function setLayer(name, on) {
  if (state.mode === 'practice' && name === 'forces') on = false;
  state[name] = on;
  const btn = document.querySelector(`[data-layer="${name}"]`);
  if (btn) btn.setAttribute('aria-pressed', on ? 'true' : 'false');
  if (name === 'light') document.body.classList.toggle('no-light', !on);
  if (name === 'bones') for (const m of boneMeshes) m.visible = on;
  if (name === 'joints') { for (const m of cartMeshes) m.visible = on; for (const m of discMeshes) m.visible = on; }
  if (name === 'joints' || name === 'ligs' || name === 'musclesOuter' || name === 'musclesDeep') syncSurfaceVisibility();
  if (name === 'forces') updateForces();
  if (FR && name === 'fascia') { FR.setLayer('sheets', on); FR.setLayer('bands', on); updateFascia(null, true); }
  if (FR && name === 'flines') { FR.setLayer('lines', on); updateFascia(null, true); if (on && !state.sel) select({ kind: 'flines' }); }
  if (FR && name === 'light') updateFascia(null, true);
  if (state.sel && ((state.sel.kind === 'fline' || state.sel.kind === 'flines') && !state.flines || state.sel.kind === 'fascia' && !state.fascia)) select(null);
  if (state.sel && ((state.sel.kind === 'muscle' && !muscleVisible(unitStrands[state.sel.unit][0])) || (state.sel.kind === 'lig' && !state.ligs) || (state.sel.kind === 'joint' && !state.joints) || (state.sel.kind === 'bone' && !state.bones))) select(null);
}
document.querySelectorAll('[data-layer]').forEach(b => b.addEventListener('click', () => setLayer(b.dataset.layer, b.getAttribute('aria-pressed') !== 'true')));
function setLightMode(mode) {
  if (state.mode === 'practice' && mode !== 'length') return;
  state.lightMode = mode;
  document.querySelectorAll('[data-lightmode]').forEach(b => b.setAttribute('aria-pressed', b.dataset.lightmode === mode ? 'true' : 'false'));
  document.body.classList.toggle('mode-effort', mode === 'effort');
  const cap = $('pose-caption'); if (cap) cap.textContent = nb(mode === 'effort' ? 'Свет показывает усилие мышц и нагрузку на суставы' : 'Свет показывает отличие от обычной стойки');
  if (!state.light) setLayer('light', true);
  updateFascia(null, true);
  if (!state.sel || /^(fline|flines|fascia)$/.test(state.sel.kind)) renderInfo();
}
document.querySelectorAll('[data-lightmode]').forEach(b => b.addEventListener('click', () => setLightMode(b.dataset.lightmode)));

const slider = $('pose'), poseOut = $('pose-out'), playBtn = $('play');
let poseDirty = true, morphDone = false;
function setT(t, fromSlider) {
  if (state.mode === 'practice') t = 1;
  state.t = Math.max(0, Math.min(1, t));
  if (!fromSlider) slider.value = Math.round(state.t * 100);
  poseOut.textContent = Math.round(state.t * 100) + '%';
  slider.style.setProperty('--fill', state.t * 100 + '%');
  poseDirty = true;
}
slider.addEventListener('input', () => { state.playing = null; setPlayLabel(); setT(slider.value / 100, true); });
function setPlayLabel() { playBtn.setAttribute('aria-pressed', state.playing ? 'true' : 'false'); playBtn.querySelector('span').textContent = state.playing ? 'Стоп' : 'Переход'; }
playBtn.addEventListener('click', () => {
  if (state.mode === 'practice') return;
  if (state.playing) { state.playing = null; setPlayLabel(); return; }
  if (reduceMotion) { setT(state.t > 0.5 ? 0 : 1); return; }
  const from = state.t >= 0.999 ? 0 : state.t;
  if (state.t >= 0.999) setT(0);
  state.playing = { t0: performance.now(), from, dur: 3200 * (1 - from) + 400 };
  setPlayLabel();
});
$('reset').addEventListener('click', () => { state.userMoved = false; resetView(true); });
const hint = $('hint');
function hideHint() { if (hint && !hint.hidden) { hint.classList.add('gone'); setTimeout(() => { hint.hidden = true; }, 400); } }

// ---------- FYSM practice: static reconstructions, without borrowed physics ----------
let practiceReturn = null;
const practiceText = value => nb(value).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
function practiceCurrent() {
  const algorithm = PRACTICE_BY_ID.get(state.algorithmId);
  const step = Array.isArray(algorithm?.steps) ? algorithm.steps[state.practiceStep] : null;
  return algorithm && step ? { algorithm, step, geometry: PRACTICE_DATA.poses[step.id] } : null;
}
function setPageHash(hash, replace = false) {
  const target = '#' + hash;
  if (location.hash !== target) history[replace ? 'replaceState' : 'pushState'](null, '', target);
}
function syncPracticeControls() {
  const on = state.mode === 'practice';
  document.body.classList.toggle('practice-mode', on);
  $('main-mode').setAttribute('aria-pressed', on ? 'false' : 'true');
  $('practice-mode').setAttribute('aria-pressed', on ? 'true' : 'false');
  $('base-poses').hidden = on;
  $('practice-navigation').hidden = !on;
  posePlayer.hidden = on || state.mode === 'cycle';
  cyclePlayer.hidden = on || state.mode !== 'cycle';
  for (const b of document.querySelectorAll('[data-lightmode]')) b.setAttribute('aria-pressed', b.dataset.lightmode === state.lightMode ? 'true' : 'false');
  document.body.classList.toggle('mode-effort', state.lightMode === 'effort');
  const forceButton = document.querySelector('[data-layer="forces"]');
  forceButton.disabled = on;
  forceButton.setAttribute('aria-pressed', state.forces ? 'true' : 'false');
  document.querySelector('[data-lightmode="effort"]').disabled = on;
  $('legend-lig-length').textContent = nb(on
    ? 'Тёплый свет на связке или капсуле — её путь длиннее, чем в стойке'
    : 'Связки и капсулы светятся тёплым там, где берут натяжение');
}
function updatePracticeHeader() {
  const { algorithm, step } = practiceCurrent();
  $('pose-sanskrit').textContent = nb(step.sanskrit);
  $('pose-name').textContent = nb(step.name);
  $('practice-progress').textContent = nb('Поза ' + step.number + ' из ' + algorithm.steps.length);
  $('practice-prev').disabled = state.practiceStep === 0;
  $('practice-next').disabled = state.practiceStep === algorithm.steps.length - 1;
  document.querySelectorAll('[data-algorithm]').forEach(b => b.setAttribute('aria-pressed', b.dataset.algorithm === algorithm.id ? 'true' : 'false'));
  $('practice-steps').innerHTML = algorithm.steps.map((s, i) => `<button class="btn practice-step" data-practice-step="${i}" aria-label="${practiceText('Поза ' + s.number + '. ' + s.name)}" aria-pressed="${state.practiceStep === i}"${state.practiceStep === i ? ' aria-current="step"' : ''}>${s.number}</button>`).join('');
  $('practice-steps').querySelectorAll('[data-practice-step]').forEach(b => b.addEventListener('click', () => enterPractice(algorithm.id, Number(b.dataset.practiceStep))));
  document.querySelectorAll('[data-pose]').forEach(b => b.setAttribute('aria-pressed', 'false'));
  cycleTab.setAttribute('aria-pressed', 'false');
  canvas.setAttribute('aria-label', nb('3D-реконструкция позы «' + step.name + '». Вращай, приближай и выбирай мышцы, связки и суставы.'));
}
function enterPractice(id = state.algorithmId, index = state.practiceStep, writeHash = true) {
  const algorithm = PRACTICE_BY_ID.get(id);
  if (!Array.isArray(algorithm?.steps) || !Number.isInteger(index) || index < 0 || index >= algorithm.steps.length || !Object.hasOwn(practiceConfs, algorithm.steps[index].id)) return false;
  if (state.mode !== 'practice') {
    practiceReturn = { mode: state.mode, poseId: state.poseId, t: state.t, cycT: state.cycT, lightMode: state.lightMode, forces: state.forces };
    if (state.mode === 'cycle') exitCycle();
  }
  state.mode = 'practice'; state.algorithmId = id; state.practiceStep = index;
  state.playing = null; setPlayLabel(); cycPause();
  morph = null; cycMorph = null; overrideConf = null;
  state.camAnim = null; state.sel = null; state.t = 1;
  state.lightMode = 'length'; state.forces = false;
  if (FR) FR.focus(null);
  fasciaM = null;
  VIEW_DIR = new THREE.Vector3(...(PRACTICE_DATA.poses[algorithm.steps[index].id].view || [-1, .3, .6])).normalize();
  state.userMoved = false;
  syncPracticeControls();
  applyPose(1); poseDirty = false;
  updateHeader(); renderCues(); renderInfo(); updateMaterials(performance.now()); updateForces();
  resetView(false);
  if (writeHash) setPageHash('practice/' + id + '/' + (index + 1));
  return true;
}
function exitPractice(restore = true, writeHash = true) {
  if (state.mode !== 'practice') return;
  const previous = practiceReturn || { mode: 'pose', poseId: state.poseId, t: 1, lightMode: 'length', forces: false };
  state.mode = 'pose'; state.playing = null; state.sel = null;
  morph = null; cycMorph = null; overrideConf = null; state.camAnim = null;
  state.poseId = previous.poseId; state.lightMode = previous.lightMode; state.forces = previous.forces;
  activeSet = activeOf(state.poseId); fasciaM = null;
  canvas.setAttribute('aria-label', nb('3D-модель выбранной позы. Вращай, приближай и нажимай на мышцы, связки и суставы.'));
  syncPracticeControls();
  if (!restore) return;
  VIEW_DIR = new THREE.Vector3(...POSE[state.poseId].view).normalize();
  buildForces(state.poseId); setT(previous.t); setPlayLabel();
  if (previous.mode === 'cycle' && CYC) {
    enterCycle(previous.cycT, false); cycPause(); cycMorph = null;
  }
  state.userMoved = false;
  applyPose(state.t); poseDirty = false; resetView(false);
  updateHeader();
  if (state.mode === 'cycle') renderCycleStats(); else renderCues();
  renderInfo(); updateForces();
  if (writeHash) setPageHash(state.mode === 'cycle' ? 'cycle' : state.poseId);
}
function practiceLengths(left, right, central = false) {
  const row = (label, value) => `<div><dt>${practiceText(label)}</dt><dd class="${value >= 0 ? 'warm' : 'cool'}-t">${Number.isFinite(value) ? fmtPct(value) : '—'}</dd></div>`;
  return `<dl class="practice-lengths">${row(central ? 'По средней линии' : 'Слева', left)}${central ? '' : row('Справа', right)}</dl>`;
}
function practiceStrain(unit, side, kind = 'muscle') {
  const parts = (kind === 'muscle' ? unitStrands[unit] : ligStrands[unit])?.filter(s => s.inst.side === side) || [];
  return parts.length ? avg(parts) : null;
}
function renderPracticeCues() {
  const { algorithm, step, geometry } = practiceCurrent();
  const notes = (geometry.notes || []).filter(n => typeof n === 'string');
  const image = algorithm.referenceImage;
  cuesEl.innerHTML = `<p class="card-eyebrow">${practiceText(algorithm.name + ' · поза ' + step.number + ' из ' + algorithm.steps.length)}</p>
    <h2 class="card-title">${practiceText('О положении')}</h2>
    <p class="role">${practiceText('3D-реконструкция по фото. Нагрузка для этих поз пока не рассчитана.')}</p>
    ${notes.length ? `<ul class="plain">${notes.map(n => `<li>${practiceText(n)}</li>`).join('')}</ul>` : ''}
    <p class="meter-foot">${practiceText('Источник: «' + PRACTICE_DATA.source.title + '», раздел «' + PRACTICE_DATA.section + '», страница PDF ' + algorithm.page + '. Названия и порядок поз сохранены.')}</p>
    <p class="meter-foot">${practiceText('Показано статическое положение. Переходы между позами не реконструированы.')}</p>
    ${image ? `<details class="practice-reference"><summary>${practiceText('Фото из методички')}</summary><img src="${image}" alt="${practiceText(algorithm.referenceAlt || 'Восемь поз алгоритма «' + algorithm.name + '» из методички FYSM')}" loading="lazy"><p class="meter-foot">${practiceText('«' + PRACTICE_DATA.source.title + '», страница PDF ' + algorithm.page + '.')}</p></details>` : ''}`;
}
function bindPracticeSides() {
  info.querySelectorAll('[data-practice-side]').forEach(b => b.addEventListener('click', () => select({ ...state.sel, side: b.dataset.practiceSide })));
}
function renderPracticeInfo() {
  const s = state.sel;
  const noLoad = `<p class="meter-foot">${practiceText('Изменение длины пути — геометрическая оценка относительно стойки. Оно не показывает усилие мышцы, нагрузку или растяжение ткани.')}</p>`;
  if (s?.kind === 'bone' || s?.kind === 'cart') return renderBone(s);
  if (s?.kind === 'muscle' || s?.kind === 'lig') {
    const muscle = s.kind === 'muscle', definition = muscle ? A.UNITS[s.unit] : A.LIGS[s.unit], side = s.side || 'L';
    const parts = (muscle ? unitStrands[s.unit] : ligStrands[s.unit]).filter(p => p.inst.side === side);
    const left = practiceStrain(s.unit, 'L', s.kind), right = practiceStrain(s.unit, 'R', s.kind);
    info.innerHTML = `${closeBtn()}<p class="card-eyebrow">${practiceText(muscle ? 'Мышца · длина пути' : 'Связка · длина пути')}</p>
      <h2 class="card-title">${practiceText(definition.name)}</h2><p class="latin">${practiceText(definition.latin || '')}</p>
      <h3 class="card-sub">${practiceText('Изменение относительно стойки')}</h3>${practiceLengths(left, right)}${noLoad}
      <div class="practice-side">${['L', 'R'].map(sd => `<button class="btn" data-practice-side="${sd}" aria-pressed="${side === sd}">${practiceText(sd === 'L' ? 'Слева' : 'Справа')}</button>`).join('')}</div>
      <h3 class="card-sub">${practiceText('Пучки ' + (side === 'L' ? 'слева' : 'справа'))}</h3>
      <ul class="parts">${parts.map((p, i) => `<li><span>${practiceText(p.inst.label || 'Пучок ' + (i + 1))}</span><span class="num ${p.strain >= 0 ? 'warm' : 'cool'}-t">${fmtPct(p.strain)}</span>${barHTML(p.strain)}</li>`).join('')}</ul>
      ${muscle ? muscleSourceHTML(s.unit) + `<h3 class="card-sub">${practiceText('Что делает мышца')}</h3><p class="role">${practiceText(definition.fn || '')}</p>` : ''}${focusBtn()}`;
    bindCardButtons(); bindPracticeSides(); return;
  }
  if (s?.kind === 'joint') {
    const j = jointById[s.id], r = j.res;
    info.innerHTML = `${closeBtn()}<p class="card-eyebrow">${practiceText('Сустав · ' + sideWord(s.id))}</p><h2 class="card-title">${practiceText(j.def.name)}</h2>
      <h3 class="card-sub">${practiceText('Углы показанного положения')}</h3><ul class="parts comps">${r.comps.map(c => `<li><span>${practiceText(c.label)}</span><span class="num gold-t">${fmtDeg(c.value)}</span>${romBar(c.value, c.rom)}<span class="part-note">${practiceText('Диапазон модели до ' + c.rom + '°. ' + (c.note || ''))}</span></li>`).join('')}</ul>
      <p class="meter-foot">${practiceText('Углы — геометрическая оценка реконструкции по фото. Силы и контакт в суставе для этой позы не рассчитаны.')}</p>${jointSourceHTML(s.id)}${focusBtn()}`;
    bindCardButtons(); return;
  }
  if (FASC && s && /^(fline|fascia|flines)$/.test(s.kind)) {
    const m = ensureFasciaM();
    if (s.kind === 'flines') {
      info.innerHTML = `${closeBtn()}<p class="card-eyebrow">${practiceText('Геометрия линий Майерса')}</p><h2 class="card-title">${practiceText('Изменение длины к стойке')}</h2>${noLoad}<ul class="rank">${Object.keys(FASC.LINE).map(id => `<li><button class="rank-row" data-fline="${id}"><span class="rank-name">${lineDot(id)} ${practiceText(FASC.LINE[id].ru)}</span><span class="practice-pair">${practiceText('Слева ' + fmtPct(m.lines[id + '_L']?.strain || 0) + ' · справа ' + fmtPct(m.lines[id + '_R']?.strain || 0))}</span></button></li>`).join('')}</ul>`;
    } else {
      const line = s.kind === 'fline', def = line ? FASC.lineInfo(s.id) : FASC.EL[s.id];
      const values = line ? m.lines : m.el, left = values[s.id + '_L'] || values[s.id + '_C'] || values[s.id], right = values[s.id + '_R'];
      const central = !values[s.id + '_L'] && !right;
      info.innerHTML = `${closeBtn()}<p class="card-eyebrow">${practiceText(line ? 'Линия Майерса · геометрия' : 'Фасция · геометрия')}</p><h2 class="card-title">${practiceText(def.ru)}</h2>
        <h3 class="card-sub">${practiceText('Изменение длины к стойке')}</h3>${practiceLengths(left?.strain ?? null, right?.strain ?? null, central)}${noLoad}
        <h3 class="card-sub">${practiceText('Описание структуры')}</h3><p class="role">${practiceText(line ? def.fn || '' : def.role || '')}</p>${focusBtn()}`;
    }
    bindCardButtons(); bindFasciaButtons(); return;
  }
  const changes = UNIT_IDS.map(unit => ({ unit, L: practiceStrain(unit, 'L'), R: practiceStrain(unit, 'R') }))
    .filter(p => Number.isFinite(p.L) && Number.isFinite(p.R)).sort((a, b) => Math.max(Math.abs(b.L), Math.abs(b.R)) - Math.max(Math.abs(a.L), Math.abs(a.R))).slice(0, 6);
  info.innerHTML = `<p class="card-eyebrow">${practiceText('Геометрия позы')}</p><h2 class="card-title">${practiceText('Длина мышечных путей')}</h2>${noLoad}
    <ul class="rank">${changes.map(p => `<li><button class="rank-row" data-unit="${p.unit}"><span class="rank-name">${practiceText(A.UNITS[p.unit].name)}</span><span class="practice-pair">${practiceText('Слева ' + fmtPct(p.L) + ' · справа ' + fmtPct(p.R))}</span></button></li>`).join('')}</ul>
    <p class="card-hint">${practiceText('Нажми на мышцу, связку или сустав. Карточка покажет геометрию выбранной стороны.')}</p>`;
  bindCardButtons();
}

// ---------- pose switching ----------
function updateHeader() {
  if (state.mode === 'practice') return updatePracticeHeader();
  const cycMode = state.mode === 'cycle';
  document.querySelectorAll('[data-pose]').forEach(b => b.setAttribute('aria-pressed', !cycMode && b.dataset.pose === state.poseId ? 'true' : 'false'));
  if (cycleTab) cycleTab.setAttribute('aria-pressed', cycMode ? 'true' : 'false');
  if (cycMode) { cyc.phaseKey = ''; updateCycleText(); return; }
  const P = POSE[state.poseId];
  $('pose-sanskrit').textContent = P.sanskrit;
  $('pose-name').textContent = nb(P.name);
  $('pose-end').textContent = P.short || P.name.split(' ')[0];
  slider.setAttribute('aria-label', nb('Из стойки в позу «' + P.name + '»'));
  document.querySelectorAll('[data-pose]').forEach(b => b.setAttribute('aria-pressed', b.dataset.pose === state.poseId ? 'true' : 'false'));
}
function switchPose(id) {
  const wasCycle = state.mode === 'cycle';
  const wasPractice = state.mode === 'practice';
  if (!Object.hasOwn(POSE, id) || (id === state.poseId && !wasCycle && !wasPractice)) return;
  // the frame of the round is a reused object: the morph starts from a copy (with its place on the mat)
  const from = !wasPractice && displayConf ? cloneConf(displayConf) : null;
  if (wasPractice) exitPractice(false, false);
  if (wasCycle) exitCycle();
  state.poseId = id;
  activeSet = activeOf(id);
  VIEW_DIR = new THREE.Vector3(...POSE[id].view).normalize();
  buildForces(id);
  state.playing = null; setPlayLabel();
  setT(1);
  morph = reduceMotion || !from ? null : { from, t0: performance.now(), dur: 1200, u: 0 };
  state.userMoved = false;
  if (state.sel && state.sel.kind === 'bone') state.sel = null;
  updateHeader();
  renderCues();
  renderInfo();
  setPageHash(id);
}
document.querySelectorAll('[data-pose]').forEach(b => b.addEventListener('click', () => switchPose(b.dataset.pose)));

// ---------- the round: player ----------
const cycleTab = $('cycle-tab'), cyclePlayer = $('cycle-player'), posePlayer = $('pose-player');
const cycSlider = $('cyc-t'), cycOut = $('cyc-out'), cycPhaseEl = $('cyc-phase'), cycPlayBtn = $('cyc-play'), cycMarks = $('cyc-marks');
const cyc = { playing: false, speed: 1, last: 0, pauseUntil: 0, fade: null, hold: 15, zone: 'all', allMus: false, allLig: false, lastCard: 0, lastLive: 0, phaseKey: '', infoBusy: 0, cardDirty: false };
const HOLDS = [0, 5, 15, 30, 60];
const fmtSec = s => {
  const v = Math.max(0, s);
  if (v < 9.95) return (Math.round(v * 10) / 10).toFixed(1).replace('.', ',') + ' с';
  if (v < 59.5) return Math.round(v) + ' с';
  const m = Math.floor(Math.round(v) / 60), r = Math.round(v) - m * 60;
  return m + ' мин' + (r ? ' ' + r + ' с' : '');
};
// "из 91 мышцы", "из 27 связок": genitive after "из"
const genOf = (n, sing, pl) => (n % 10 === 1 && n % 100 !== 11 ? sing : pl);
const plural = (n, one, few, many) => { const a = n % 100, b = n % 10; if (a > 10 && a < 20) return many; if (b === 1) return one; if (b > 1 && b < 5) return few; return many; };
const actPlaceholder = () => !!(CYC && CYC.placeholder && CYC.placeholder.act);
function cycPhaseAt(t) { const ph = CYC.phases; for (let k = 0; k < ph.length; k++) if (t < ph[k].t1 - 1e-6) return ph[k]; return ph[ph.length - 1]; }
function cycSegAt(t) { const s = CYC.segs; for (let k = 0; k < s.length; k++) if (t < s[k].t1 - 1e-6) return k; return s.length - 1; }
function cycHoldAt(t) { return CYC.holds.findIndex(h => Math.abs(h.T - t) <= 0.051); }
const posPct = t => `calc(11px + (100% - 22px) * ${(t / CYC.T).toFixed(5)})`;
function buildCycMarks() {
  let html = '';
  for (const p of CYC.phases) if (p.t0 > 1e-6 && cycHoldAt(p.t0) < 0) html += `<span class="cyc-tick" style="left:${posPct(p.t0)}" aria-hidden="true"></span>`;
  CYC.holds.forEach((h, k) => { html += `<button class="cyc-pose" data-hold="${k}" style="left:${posPct(h.T)}" aria-label="${nb('Поза ' + (k + 1) + ': ' + POSE[h.pose].name)}">${k + 1}</button>`; });
  cycMarks.innerHTML = html;
  cycMarks.querySelectorAll('[data-hold]').forEach(b => b.addEventListener('click', () => { cycPause(); setCycT(CYC.holds[+b.dataset.hold].T); }));
}
function setCycT(t, fromSlider) {
  state.cycT = Math.max(0, Math.min(CYC.T, t));
  const f = state.cycT / CYC.T;
  if (!fromSlider) cycSlider.value = Math.round(f * 1000);
  cycSlider.style.setProperty('--fill', (f * 100).toFixed(2) + '%');
  cycOut.textContent = fmtSec(state.cycT) + ' из ' + fmtSec(CYC.T);
  updateCycleText();
  poseDirty = true;
}
function updateCycleText() {
  const t = state.cycT, h = cycHoldAt(t), k = cycSegAt(t), S = CYC.segs[k], ph = cycPhaseAt(t);
  const key = h >= 0 ? 'h' + h : 'p' + CYC.phases.indexOf(ph);
  if (key === cyc.phaseKey) return;
  cyc.phaseKey = key;
  let text;
  if (h >= 0) {
    const P = POSE[CYC.holds[h].pose];
    $('pose-sanskrit').textContent = nb('Круг · поза ' + (h + 1) + ' из ' + CYC.holds.length);
    $('pose-name').textContent = nb(P.name);
    text = 'Поза «' + P.name + '»';
  } else {
    $('pose-sanskrit').textContent = nb('Круг · переход ' + (k + 1) + ' из ' + CYC.segs.length);
    $('pose-name').textContent = nb(S.fromName) + ' → ' + nb(S.toName);
    text = ph.label;
  }
  cycPhaseEl.textContent = nb(text);
  cycSlider.setAttribute('aria-valuetext', nb(fmtSec(t) + ': ' + text));
  cycMarks.querySelectorAll('[data-hold]').forEach(b => b.setAttribute('aria-current', +b.dataset.hold === h ? 'true' : 'false'));
}
function setCycPlayLabel() { cycPlayBtn.setAttribute('aria-pressed', cyc.playing ? 'true' : 'false'); cycPlayBtn.querySelector('span').textContent = cyc.playing ? 'Пауза' : 'Пуск'; }
function cycPlay() {
  if (!CYC) return;
  if (state.cycT >= CYC.T - 1e-3) setCycT(0);
  cyc.playing = true; cyc.last = performance.now(); cyc.pauseUntil = 0; cyc.fade = null;
  setCycPlayLabel();
}
function cycPause() {
  cyc.playing = false; cyc.pauseUntil = 0;
  if (cyc.fade) { cyc.fade = null; canvas.style.opacity = ''; }
  setCycPlayLabel();
}
// playback: real time × speed; a short stop at each of the five poses; at the end a fade back to the start
function cycStep(now) {
  if (!cyc.playing) return;
  // slow devices skip frames instead of slowing the round down
  const dt = Math.min(0.25, Math.max(0, (now - cyc.last) / 1000)); cyc.last = now;
  if (cyc.fade) {
    if (now >= cyc.fade.until) { cyc.fade = null; setCycT(0); canvas.style.opacity = ''; cyc.pauseUntil = now + 1000; }
    return;
  }
  if (cyc.pauseUntil) {
    if (now < cyc.pauseUntil) return;
    cyc.pauseUntil = 0;
    if (state.cycT >= CYC.T - 1e-6) { cyc.fade = { until: now + 320 }; canvas.style.opacity = '0'; return; }
  }
  const t0 = state.cycT, t1 = t0 + dt * cyc.speed;
  const h = CYC.holds.find(x => x.T > t0 + 1e-6 && x.T <= t1 + 1e-9);
  if (h) { setCycT(h.T); cyc.pauseUntil = now + (h.T >= CYC.T - 1e-6 ? 1500 : 1000); return; }
  setCycT(Math.min(CYC.T, t1));
}
function enterCycle(startT, writeHash = true) {
  if (!CYC || state.mode === 'cycle') return;
  const wasPractice = state.mode === 'practice';
  if (wasPractice) exitPractice(false, false);
  const from = !wasPractice && displayConf ? cloneConf(displayConf) : null;
  if (from && !from.rootP) from.rootP = placedRootP(from.local, from.rootQ);
  state.mode = 'cycle';
  state.playing = null; setPlayLabel(); morph = null;
  const h = CYC.holds.find(x => x.pose === state.poseId);
  cycMorph = reduceMotion || !from ? null : { from, t0: performance.now(), dur: 900, u: 0 };
  VIEW_DIR = CYCLE_VIEW.clone();
  state.userMoved = false;
  if (state.sel && (state.sel.kind === 'bone' || state.sel.kind === 'cart')) state.sel = null;
  cyclePlayer.hidden = false; posePlayer.hidden = true;
  cyc.phaseKey = ''; cycForceI = -1;
  const lj = $('legend-joint-effort'); if (lj) lj.textContent = nb('Золото на хряще — сила, с которой кости давят друг на друга, ярче всего от 3 весов тела');
  setCycT(startT ?? (h ? h.T : 0));
  updateHeader();
  renderCycleStats();
  renderInfo();
  if (!reduceMotion) { cycPlay(); cyc.pauseUntil = performance.now() + 1500; }
  if (writeHash) setPageHash('cycle');
}
function exitCycle() {
  if (state.mode !== 'cycle') return;
  cycPause(); cycMorph = null;
  state.mode = 'pose';
  cyclePlayer.hidden = true; posePlayer.hidden = false;
  canvas.style.opacity = '';
  const lj = $('legend-joint-effort'); if (lj) lj.textContent = nb('Золото на хряще — нагрузка на сустав, доля силы мышц вокруг него');
  if (cycForce) cycForce.visible = false;
}
if (cycleTab) cycleTab.addEventListener('click', () => { if (state.mode === 'cycle') return; enterCycle(); });
cycSlider.addEventListener('input', () => { cycPause(); setCycT(cycSlider.value / 1000 * CYC.T, true); });
cycPlayBtn.addEventListener('click', () => (cyc.playing ? cycPause() : cycPlay()));
document.querySelectorAll('[data-speed]').forEach(b => b.addEventListener('click', () => {
  cyc.speed = +b.dataset.speed;
  document.querySelectorAll('[data-speed]').forEach(x => x.setAttribute('aria-pressed', x === b ? 'true' : 'false'));
}));
info.addEventListener('pointerdown', () => { cyc.infoBusy = performance.now(); });
info.addEventListener('pointermove', () => { cyc.infoBusy = performance.now(); });

// ---------- the round: statistics ----------
// one round with a hold of H seconds in each of the five poses:
// peak = max(transitions, poses), integral = transitions + H·Σ poses, time over a threshold = transitions + H·(poses over it), mean = integral / (T + 5H)
function roundStats(H) {
  const S = CYC.stats, poses = CYC.holds.map(h => h.pose), np = poses.length;
  const total = S.T + np * H;
  const th = S.thresholds || { act: [0.08, 0.2, 0.4], lig: [0.02, 0.05] };
  const mus = Object.entries(S.muscles).filter(([u]) => A.UNITS[u]).map(([u, m]) => {
    const side = sd => {
      const tr = (m.trans && m.trans[sd]) || {}, pv = poses.map(p => (m.pose && m.pose[p] && m.pose[p][sd]) || 0);
      const peak = Math.max(tr.peak || 0, H > 0 ? Math.max(...pv) : 0);
      const int = (tr.int || 0) + H * pv.reduce((a, b) => a + b, 0);
      const cnt = thr => pv.filter(v => v >= thr).length;
      return { peak, int, mean: int / total, t08: (tr.t08 || 0) + H * cnt(th.act[0]), t20: (tr.t20 || 0) + H * cnt(th.act[1]), t40: (tr.t40 || 0) + H * cnt(th.act[2]) };
    };
    const L = side('L'), R = side('R');
    const st = m.stretch || {}, spv = poses.map(p => (st.pose && st.pose[p]) || 0);
    return { u, L, R, peak: Math.max(L.peak, R.peak), mean: (L.mean + R.mean) / 2, t08: Math.max(L.t08, R.t08), t40: Math.max(L.t40, R.t40), stretch: Math.max(st.peak || 0, H > 0 ? Math.max(...spv) : -1) };
  });
  const ligs = Object.entries(S.ligs).map(([id, o]) => {
    const side = sd => {
      const tr = (o.trans && o.trans[sd]) || {}, pv = poses.map(p => (o.pose && o.pose[p] && o.pose[p][sd]) ?? -1);
      return { peak: Math.max(tr.peak ?? -1, H > 0 ? Math.max(...pv) : -1), t02: (tr.t02 || 0) + H * pv.filter(v => v >= th.lig[0]).length, t05: (tr.t05 || 0) + H * pv.filter(v => v >= th.lig[1]).length };
    };
    const L = side('L'), R = side('R');
    return { id, cap: id.startsWith('cap:'), L, R, peak: Math.max(L.peak, R.peak), t02: Math.max(L.t02, R.t02), t05: Math.max(L.t05, R.t05) };
  }).filter(l => (l.cap ? A.CAP_NAMES[l.id.slice(4)] : A.LIGS[l.id]));
  return { total, H, np, mus, ligs, th };
}
const ligName = id => (id.startsWith('cap:') ? A.CAP_NAMES[id.slice(4)] : A.LIGS[id].name);
const ligAttr = id => (id.startsWith('cap:') ? `data-joint="${id.slice(4)}_L"` : `data-lig="${id}"`);
function peakBar(mean, peak) {
  const w = v => Math.max(0, Math.min(100, v * 100)).toFixed(1);
  return `<div class="bar" aria-hidden="true"><span class="bar-fill warm" style="left:0;width:${w(mean)}%"></span><span class="bar-peak" style="left:${w(peak)}%"></span></div>`;
}
const ZONE_CHIPS = [['all', 'Все'], ['shoulder', 'Плечи и руки'], ['trunk', 'Корпус'], ['leg', 'Таз и ноги']];
const inZone = u => cyc.zone === 'all' || (cyc.zone === 'shoulder' ? ['shoulder', 'arm'].includes(A.UNITS[u].zone) : A.UNITS[u].zone === cyc.zone);
function renderCycleStats() {
  const R = roundStats(cyc.hold), th = R.th, ph = actPlaceholder();
  const nMus = R.mus.length;
  const on = R.mus.filter(m => m.peak >= th.act[0]).length, strong = R.mus.filter(m => m.peak >= th.act[2]).length;
  const ligOnly = R.ligs.filter(l => !l.cap), capOnly = R.ligs.filter(l => l.cap);
  const taut = ligOnly.filter(l => l.peak >= th.lig[0]).length, capT = capOnly.filter(l => l.peak >= th.lig[0]).length;
  const mus = R.mus.filter(m => inZone(m.u)).sort((a, b) => b.mean - a.mean || b.peak - a.peak);
  const musShown = cyc.allMus ? mus : mus.slice(0, 12);
  const ligs = ligOnly.slice().sort((a, b) => b.peak - a.peak), caps = capOnly.slice().sort((a, b) => b.peak - a.peak);
  const ligShown = cyc.allLig ? ligs : ligs.slice(0, 8);
  const stretchTop = R.mus.filter(m => m.stretch > 0.05 && inZone(m.u)).sort((a, b) => b.stretch - a.stretch).slice(0, 6);
  const tot = fmtSec(R.total);
  const musRow = m => {
    const asym = Math.abs(m.L.peak - m.R.peak) >= 0.05;
    const sub = 'в работе ' + fmtSec(m.t08) + ' из ' + tot + (asym ? ', пик слева ' + pctTxt(m.L.peak) + ', справа ' + pctTxt(m.R.peak) : '');
    return `<li><button class="rank-row" data-unit="${m.u}"><span class="rank-name">${nb(A.UNITS[m.u].name)}</span><span class="num-pair warm-t">${pctTxt(m.mean)} · пик ${pctTxt(m.peak)}</span>${peakBar(m.mean, m.peak)}<span class="rank-sub">${nb(sub)}</span></button></li>`;
  };
  const ligRow = l => `<li><button class="rank-row" ${ligAttr(l.id)}><span class="rank-name">${nb(ligName(l.id))}</span><span class="num ${l.peak >= th.lig[0] ? 'warm' : 'muted'}-t">${fmtPct(l.peak)}</span>${barHTML(l.peak)}<span class="rank-sub">${nb(l.peak >= th.lig[0] ? 'длиннее стойки ' + fmtSec(l.t02) + ' из ' + tot + (l.t05 > 0.05 ? ', больше чем на 5% — ' + fmtSec(l.t05) : '') : 'не длиннее, чем в стойке')}</span></button></li>`;
  const holdLine = cyc.hold ? R.np + ' × ' + cyc.hold + ' с в позах' : 'без задержек';
  cuesEl.innerHTML = `
    <p class="card-eyebrow">${nb('За один круг')}</p>
    <h2 class="card-title">${nb('Как включаются мышцы и связки')}</h2>
    <div class="hold-row"><p class="block-label">${nb('Задержка в каждой позе')}</p>
      <div class="seg seg-hold" role="group" aria-label="${nb('Задержка в каждой позе')}">${HOLDS.map(h => `<button data-hold-s="${h}" aria-pressed="${h === cyc.hold}">${h} с</button>`).join('')}</div></div>
    <div class="kpis">
      <div class="kpi"><div class="kpi-v">${ph ? '—' : on}<small> из ${nMus}</small></div><div class="kpi-l">${nb(genOf(nMus, 'мышцы', 'мышц') + ' включились сильнее 8% своей силы')}</div></div>
      <div class="kpi"><div class="kpi-v">${ph ? '—' : strong}</div><div class="kpi-l">${nb(ph ? 'работали сильно' : plural(strong, 'мышца работала', 'мышцы работали', 'мышц работали') + ' сильно, больше 40%')}</div></div>
      <div class="kpi"><div class="kpi-v">${taut}<small> из ${ligOnly.length}</small></div><div class="kpi-l">${nb(genOf(ligOnly.length, 'связки', 'связок') + ' удлинились относительно стойки' + (capOnly.length ? '; капсулы суставов: ' + capT + ' из ' + capOnly.length : ''))}</div></div>
      <div class="kpi"><div class="kpi-v">${tot}</div><div class="kpi-l">${nb('длится круг: переходы ' + fmtSec(CYC.stats.T) + ', ' + holdLine)}</div></div>
    </div>
    ${ph ? `<p class="callout">${nb('Усилия мышц по кадрам ещё считаются. Пока видны удлинение связок и длина мышц: их даёт сама геометрия движения.')}</p>` : ''}
    ${CYC.provisional ? `<p class="callout">${nb('Черновые переходы: тело местами проходит сквозь пол, цифры предварительные.')}</p>` : ''}
    <div class="filter-chips" role="group" aria-label="${nb('Часть тела')}" style="margin-top:14px">${ZONE_CHIPS.map(([z, l]) => `<button class="chip-sm" data-zone="${z}" aria-pressed="${z === cyc.zone}">${nb(l)}</button>`).join('')}</div>
    ${ph ? '' : `<h3 class="card-sub">${nb('Мышцы: среднее усилие за круг и пик')}</h3>
    <ul class="rank">${musShown.map(musRow).join('')}</ul>
    ${mus.length > 12 ? `<button class="btn btn-sm more-btn" data-more="mus">${nb(cyc.allMus ? 'Свернуть список' : 'Показать все ' + mus.length)}</button>` : ''}`}
    ${stretchTop.length ? `<h3 class="card-sub">${nb('Сильнее всего удлиняются за круг')}</h3><ul class="rank">${stretchTop.map(m => `<li><button class="rank-row" data-unit="${m.u}"><span class="rank-name">${nb(A.UNITS[m.u].name)}</span><span class="num warm-t">${fmtPct(m.stretch)}</span>${barHTML(m.stretch)}</button></li>`).join('')}</ul>` : ''}
    ${CYC.flines ? roundLinesHTML() : ''}
    <h3 class="card-sub">${nb('Связки: пик удлинения относительно стойки')}</h3>
    <ul class="rank">${ligShown.map(ligRow).join('')}</ul>
    ${ligs.length > 8 ? `<button class="btn btn-sm more-btn" data-more="lig">${nb(cyc.allLig ? 'Свернуть список' : 'Показать все ' + ligs.length)}</button>` : ''}
    ${caps.length ? `<h3 class="card-sub">${nb('Капсулы суставов: пик удлинения относительно стойки')}</h3><ul class="rank">${caps.map(ligRow).join('')}</ul>` : ''}
    <p class="meter-foot">${nb('Среднее — усилие мышцы, усреднённое по всему кругу, пик — самое большое за круг, «в работе» — сколько секунд она тянула сильнее 8% своей силы. Для связок и капсул показано удлинение относительно стойки: насколько путь их волокон между точками крепления длиннее, чем в стойке. Это не растяжение ткани: часть капсул в стойке провисает, поэтому у них проценты большие. Задержка добавляется в каждой из пяти поз, переходы идут в спокойном темпе.')}</p>`;
  cuesEl.querySelectorAll('[data-hold-s]').forEach(b => b.addEventListener('click', () => { cyc.hold = +b.dataset.holdS; renderCycleStats(); if (state.sel) renderInfo(); }));
  cuesEl.querySelectorAll('[data-zone]').forEach(b => b.addEventListener('click', () => { cyc.zone = b.dataset.zone; renderCycleStats(); }));
  cuesEl.querySelectorAll('[data-more]').forEach(b => b.addEventListener('click', () => { if (b.dataset.more === 'mus') cyc.allMus = !cyc.allMus; else cyc.allLig = !cyc.allLig; renderCycleStats(); }));
  const toCard = d => { select(d); requestAnimationFrame(() => info.scrollIntoView({ block: 'start', behavior: reduceMotion ? 'auto' : 'smooth' })); };
  cuesEl.querySelectorAll('[data-unit]').forEach(b => b.addEventListener('click', () => toCard({ kind: 'muscle', unit: b.dataset.unit, side: 'L' })));
  cuesEl.querySelectorAll('[data-lig]').forEach(b => b.addEventListener('click', () => toCard({ kind: 'lig', unit: b.dataset.lig, side: 'L' })));
  cuesEl.querySelectorAll('[data-joint]').forEach(b => b.addEventListener('click', () => toCard({ kind: 'joint', id: b.dataset.joint })));
  cuesEl.querySelectorAll('[data-fline]').forEach(b => b.addEventListener('click', () => toCard({ kind: 'fline', id: b.dataset.fline })));
}
function roundLinesHTML() {
  if (!FASC) return '';
  const L = CYC.flines, ph = L.placeholder;
  const rows = L.ids.filter(id => FASC.LINE[id]).map(id => ({ id, ...L.lines[id] })).sort((a, b) => (ph ? b.sMax - a.sMax : b.eMax - a.eMax));
  return `<h3 class="card-sub">${nb('Миофасциальные линии Майерса за круг')}</h3>
    <ul class="rank">${rows.map(r => `<li><button class="rank-row" data-fline="${r.id}"><span class="rank-name">${lineDot(r.id)} ${nb(FASC.LINE[r.id].ru)}</span><span class="num warm-t">${ph ? fmtPct(r.sMax) : pctTxt(r.eMax)}</span>${ph ? barHTML(r.sMax, 0.2) : effBar(r.eMax)}<span class="rank-sub">${nb('длиннее всего ' + fmtPct(r.sMax) + ', короче всего ' + fmtPct(r.sMin) + (ph ? '' : '; пик включённости ' + pctTxt(r.eMax)))}</span></button></li>`).join('')}</ul>`;
}

// ---------- the round: cards ----------
function seriesOf(kind, id, side) {
  const out = new Float32Array(CYC.n);
  if (kind === 'act') { const k = CYC.unitIdx[id], a = side === 'R' ? CYC.aRa : CYC.aLa; if (k !== undefined) for (let i = 0; i < CYC.n; i++) out[i] = a[i * CYC.NU + k] / 255; }
  else if (kind === 'flS' || kind === 'flE') { if (CYC.flS) { const k = CYC.flIdx[id]; if (k !== undefined) for (let i = 0; i < CYC.n; i++) out[i] = kind === 'flS' ? CYC.flS[i * CYC.NF + k] / 4000 : CYC.flE[i * CYC.NF + k] / 255; } }
  else if (kind === 'jload') { if (CYC.jloadA) { const k = CYC.jloadIdx[id], J = CYC.jload.ids.length, sc = CYC.jload.scale || 50; if (k !== undefined) for (let i = 0; i < CYC.n; i++) out[i] = CYC.jloadA[i * J + k] / sc; } }
  else { const k = CYC.ligIdx[id], a = side === 'R' ? CYC.sRa : CYC.sLa; if (k !== undefined) for (let i = 0; i < CYC.n; i++) out[i] = a[i * CYC.NL + k] / 4000; }
  return out;
}
const CX0 = 34, CX1 = 314;
const curveX = t => CX0 + (CX1 - CX0) * t / CYC.T;
// a value over the whole round: left side solid, right side dashed, poses numbered, phases as bands, a marker for now
function curveSVG(series, o) {
  const W = 320, H = 108, y0 = 14, y1 = 88;
  let lo = 0, hi = 0;
  for (const s of series) for (const v of s) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (o.kind === 'act') { lo = 0; hi = Math.max(0.1, Math.ceil(hi * 10 - 1e-6) / 10); }
  else if (o.kind === 'jload') { lo = 0; hi = Math.max(1, Math.ceil(hi - 1e-6)); }
  else { lo = Math.floor(lo * 20 + 1e-6) / 20; hi = Math.max(0.05, Math.ceil(hi * 20 - 1e-6) / 20); }
  const Y = v => y1 - (y1 - y0) * (v - lo) / ((hi - lo) || 1);
  const n = CYC.n, dtf = 1 / CYC.fps;
  const path = s => { let d = ''; for (let i = 0; i < n; i++) d += (i ? 'L' : 'M') + curveX(i * dtf).toFixed(1) + ',' + Y(s[i]).toFixed(1); return d; };
  const bands = CYC.segs.map((s, k) => (k % 2 ? `<rect x="${curveX(s.t0).toFixed(1)}" y="${y0}" width="${(curveX(s.t1) - curveX(s.t0)).toFixed(1)}" height="${y1 - y0}" style="fill:#fff;opacity:.035"/>` : '')).join('');
  const holds = CYC.holds.map((h, k) => `<line x1="${curveX(h.T).toFixed(1)}" x2="${curveX(h.T).toFixed(1)}" y1="${y0 - 2}" y2="${y1}" style="stroke:#4b5a63;stroke-width:1;stroke-dasharray:2 2"/><text x="${Math.min(W - 6, Math.max(6, curveX(h.T))).toFixed(1)}" y="${y0 - 5}" text-anchor="middle" style="fill:#8f9ca3;font:9px var(--font-num)">${k + 1}</text>`).join('');
  const ref = o.kind === 'act' ? 0.08 : o.kind === 'jload' ? 1 : 0;
  const refLine = ref > lo + (hi - lo) * 0.12 && ref < hi - (hi - lo) * 0.12 ? `<line x1="${CX0}" x2="${CX1}" y1="${Y(ref).toFixed(1)}" y2="${Y(ref).toFixed(1)}" style="stroke:#6c7a82;stroke-width:1;stroke-dasharray:1 3"/><text x="${CX0 - 4}" y="${(Y(ref) + 3).toFixed(1)}" text-anchor="end" style="fill:#8f9ca3;font:8.5px var(--font-num)">${o.kind === 'act' ? '8%' : o.kind === 'jload' ? '1×' : '0'}</text>` : '';
  const lab = v => (o.kind === 'act' ? Math.round(v * 100) + '%' : o.kind === 'jload' ? String(Math.round(v * 10) / 10).replace('.', ',') + '×' : (v > 0 ? '+' : v < 0 ? '−' : '') + Math.abs(Math.round(v * 100)) + '%');
  const top = `<text x="${CX0 - 4}" y="${y0 + 3}" text-anchor="end" style="fill:#8f9ca3;font:8.5px var(--font-num)">${lab(hi)}</text><text x="${CX0 - 4}" y="${y1 + 3}" text-anchor="end" style="fill:#8f9ca3;font:8.5px var(--font-num)">${lab(lo)}</text>`;
  const axis = `<line x1="${CX0}" x2="${CX1}" y1="${y1}" y2="${y1}" style="stroke:#26333b;stroke-width:1"/><text x="${CX0}" y="${y1 + 12}" style="fill:#8f9ca3;font:8.5px var(--font-num)">0 с</text><text x="${CX1}" y="${y1 + 12}" text-anchor="end" style="fill:#8f9ca3;font:8.5px var(--font-num)">${String(Math.round(CYC.T)).replace('.', ',')} с</text>`;
  const lines = `<path d="${path(series[0])}" style="fill:none;stroke:var(--warm);stroke-width:1.6;stroke-linejoin:round"/>` + (series[1] ? `<path d="${path(series[1])}" style="fill:none;stroke:#ffd29a;stroke-width:1.3;stroke-dasharray:3 2;stroke-linejoin:round"/>` : '');
  const x = curveX(state.cycT).toFixed(1);
  const mark = `<line id="cyc-live-mark" x1="${x}" x2="${x}" y1="${y0 - 2}" y2="${y1 + 2}" style="stroke:var(--gold);stroke-width:2"/>`;
  return `<div class="cyc-curve"><svg viewBox="0 0 ${W} ${H}" role="img" aria-label="${nb(o.label)}" data-curve>${bands}${holds}${refLine}${axis}${top}${lines}${mark}</svg>
    <div class="curve-legend"><span><i class="lg-line"></i>${nb('слева')}</span>${series[1] ? `<span><i class="lg-line r"></i>${nb('справа')}</span>` : ''}<span><i class="lg-line now"></i>${nb('сейчас, нажми на график, чтобы перейти')}</span></div></div>`;
}
function bindCurve() {
  const svg = info.querySelector('[data-curve]'); if (!svg) return;
  svg.style.cursor = 'pointer';
  svg.addEventListener('click', e => {
    const r = svg.getBoundingClientRect();
    const xv = (e.clientX - r.left) / r.width * 320;
    cycPause(); setCycT((xv - CX0) / (CX1 - CX0) * CYC.T);
  });
}
function roundLine(m, R) {
  return `<ul class="parts">
    <li><span>${nb('Пик за круг')}</span><span class="num work-t">${pctTxt(m.peak)}</span>${effBar(m.peak, 'work')}<span class="part-note">${nb('слева ' + pctTxt(m.L.peak) + ', справа ' + pctTxt(m.R.peak))}</span></li>
    <li><span>${nb('Среднее за круг')}</span><span class="num work-t">${pctTxt(m.mean)}</span>${effBar(m.mean, 'work')}</li>
    <li><span>${nb('В работе, сильнее 8%')}</span><span class="num warm-t">${fmtSec(m.t08)}</span><span class="part-note">${nb('из ' + fmtSec(R.total) + (m.t40 > 0.05 ? ', сильнее 40% — ' + fmtSec(m.t40) : '') + (R.H ? ', с задержкой ' + R.H + ' с в каждой позе' : ', без задержек в позах'))}</span></li>
  </ul>`;
}
function renderMuscleCycle(unit) {
  const U = A.UNITS[unit], ph = actPlaceholder();
  const R = roundStats(cyc.hold), m = R.mus.find(x => x.u === unit);
  const s = unitAvg(unit), cls = s >= 0 ? 'warm' : 'cool';
  const aL = cycAct(unit, 'L'), aR = cycAct(unit, 'R'), a = Math.max(aL, aR);
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Мышца · круг')}</p>
    <h2 class="card-title">${nb(U.name)}</h2>
    <p class="latin">${U.latin}</p>
    ${muscleSourceHTML(unit)}
    <div class="tags"><span class="tag">${U.deep ? 'глубокая' : 'внешняя'}</span>${ph ? '' : `<span class="tag tag-active" id="cyc-live-tag"${a >= ACTIVE_MIN ? '' : ' hidden'}>работает</span>`}</div>
    ${ph ? `<p class="callout">${nb('Усилия по кадрам ещё считаются. Ниже — длина мышцы по ходу круга.')}</p>` : `
    <div class="meter"><span class="meter-label">${nb('Усилие сейчас')}</span><span class="num warm-t big" id="cyc-live-a">${pctTxt(a)}</span></div>
    <div class="bar" aria-hidden="true"><span class="bar-fill warm" id="cyc-live-abar" style="left:0;width:${Math.min(100, a * 100).toFixed(1)}%"></span></div>
    <p class="meter-foot" id="cyc-live-foot">${nb('слева ' + pctTxt(aL) + ', справа ' + pctTxt(aR))}</p>
    <h3 class="card-sub">${nb('Усилие по ходу круга, доля максимальной силы')}</h3>
    ${curveSVG([seriesOf('act', unit, 'L'), seriesOf('act', unit, 'R')], { kind: 'act', label: 'Усилие мышцы ' + U.name + ' по ходу круга' })}
    ${roundLine(m, R)}`}
    <div class="meter"><span class="meter-label">${nb('Длина сейчас')}</span><span class="num ${cls}-t big" id="cyc-live-len">${fmtPct(s)}</span></div>
    <div id="cyc-live-lenbar">${barHTML(s)}</div>
    <p class="meter-foot">${nb('К длине в стойке. Самая большая за круг: ' + fmtPct(m.stretch) + '.')}</p>
    <h3 class="card-sub">${nb('Что делает')}</h3><p class="role">${nb(U.fn)}</p>
    ${focusBtn()}`;
  bindCardButtons(); bindCurve();
}
function ligRoundList(o, R) {
  return `<ul class="parts">
    <li><span>${nb('Пик за круг')}</span><span class="num warm-t">${fmtPct(o.peak)}</span>${barHTML(o.peak)}<span class="part-note">${nb('слева ' + fmtPct(o.L.peak) + ', справа ' + fmtPct(o.R.peak))}</span></li>
    <li><span>${nb('Натянута сильнее, чем в стойке')}</span><span class="num warm-t">${fmtSec(o.t02)}</span><span class="part-note">${nb('из ' + fmtSec(R.total) + (o.t05 > 0.05 ? ', сильнее 5% — ' + fmtSec(o.t05) : ''))}</span></li>
  </ul>`;
}
function renderLigCycle(unit) {
  const Lg = A.LIGS[unit];
  const R = roundStats(cyc.hold), o = R.ligs.find(x => x.id === unit);
  const now = Math.max(cycStrain(unit, 'L'), cycStrain(unit, 'R'));
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Связка · круг')}</p>
    <h2 class="card-title">${nb(Lg.name)}</h2>
    <p class="latin">${Lg.latin}</p>
    <div class="meter"><span class="meter-label">${nb('Удлинение относительно стойки сейчас, самый длинный пучок')}</span><span class="num ${now > 0.02 ? 'warm' : 'muted'}-t big" id="cyc-live-len">${fmtPct(now)}</span></div>
    <div id="cyc-live-lenbar">${barHTML(now)}</div>
    <h3 class="card-sub">${nb('Расстояние между точками крепления по ходу круга, к стойке')}</h3>
    ${curveSVG([seriesOf('lig', unit, 'L'), seriesOf('lig', unit, 'R')], { kind: 'lig', label: 'Натяжение связки ' + Lg.name + ' по ходу круга' })}
    ${o ? ligRoundList(o, R) : ''}
    <p class="meter-foot">${nb('Связка не растягивается как мышца: в стойке она сложена, а при натяжении распрямляется, поэтому проценты показывают только, где она берёт нагрузку.')}</p>
    <h3 class="card-sub">${nb('Что делает')}</h3><p class="role">${nb(Lg.role)}</p>
    ${focusBtn()}`;
  bindCardButtons(); bindCurve();
}
function renderJointCycle(id) {
  const jj = jointById[id], j = jj.def, r = jj.res;
  const comps = r.comps.map(c => `<li><span>${nb(c.label)}</span><span class="num gold-t">${fmtDeg(c.value)}</span>${romBar(c.value, c.rom)}<span class="part-note">${nb('норма до ' + c.rom + '°' + (c.note ? '. ' + c.note.charAt(0).toUpperCase() + c.note.slice(1) : ''))}</span></li>`).join('');
  const reg = j.region ? regions[j.region] : null;
  const regRow = reg ? `<p class="meter-foot">${nb(reg.name + ' целиком: ' + fmtDeg(reg.signed) + ' ' + (reg.signed < -0.2 ? 'разгибания' : reg.signed > 0.2 ? 'сгибания' : '') + ', норма до ' + reg.rom + '°')}</p>` : '';
  const capId = A.CAP_NAMES[j.base] ? 'cap:' + j.base : null, side = j.side || 'L';
  const R = roundStats(cyc.hold);
  const capO = capId ? R.ligs.find(x => x.id === capId) : null;
  const capBlock = capO ? `<h3 class="card-sub">${nb(A.CAP_NAMES[j.base] + ' по ходу круга')}</h3>${curveSVG([seriesOf('lig', capId, side)], { kind: 'lig', label: A.CAP_NAMES[j.base] + ' по ходу круга' })}${ligRoundList(capO, R)}` : '';
  const loadBlock = jointLoadCycle(id);
  const ligList = Object.keys(A.LIGS).filter(u => A.LIGS[u].joint === j.base || (j.type === 'spine' && A.LIGS[u].joint === 'spine'));
  const ligRows = ligList.length ? `<h3 class="card-sub">${nb('Связки этого сустава сейчас')}</h3><ul class="rank">${ligList.map(u => { const s = avg(ligStrands[u].filter(x => x.inst.side === side)); return `<li><button class="rank-row" data-lig="${u}"><span class="rank-name">${nb(A.LIGS[u].name)}</span><span class="word ${s > 0.02 ? 'warm' : 'muted'}-t">${nb(tensionWord(s))}</span></button></li>`; }).join('')}</ul>` : '';
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb((j.type === 'spine' ? 'Позвоночник' : j.type === 'st' ? 'Лопаточно-грудное скольжение' : 'Сустав') + ' ' + sideWord(id) + ' · сейчас')}</p>
    <h2 class="card-title">${nb(j.name)}</h2>
    <ul class="parts comps">${comps}</ul>
    ${regRow}${jointSourceHTML(id)}
    ${loadBlock}
    ${capBlock}
    ${ligRows}
    ${focusBtn()}`;
  bindCardButtons(); bindCurve();
}
// force in the joint during the round (physics): now, over the round, peak
const JLOAD_KEY = id => ({ ls: 'L5S1', tl: 'T12L1' }[id] || ({ hip: 'hip', kn: 'knee', an: 'ankle', gh: 'gh', el: 'elbow', wr: 'wrist' }[id.replace(/_[LR]$/, '')]));
function jointLoadCycle(id) {
  if (!CYC.jloadA || CYC.jloadIdx[id] === undefined) return '';
  const key = JLOAD_KEY(id), J = (CYC.stats.joints || {})[key];
  const now = cycJointLoad(id);
  const H = cyc.hold;
  let peak = null;
  if (J && J.trans) { peak = J.trans.peak; if (H > 0 && J.pose) peak = Math.max(peak, ...Object.values(J.pose)); }
  const what = id === 'ls' ? 'Диск L5–S1 сжат' : id === 'tl' ? 'Диск Th12–L1 сжат' : 'Кости давят друг на друга';
  return `<h3 class="card-sub">${nb('Сила в суставе по расчёту статики')}</h3>
    <div class="meter"><span class="meter-label">${nb(what + ' сейчас')}</span><span class="num warm-t big" id="cyc-live-load">${fBW(now)}×</span></div>
    <p class="meter-foot" id="cyc-live-loadfoot">${nb('веса тела, около ' + kgf(now * PHYS.dog.bw) + '\u00a0кг')}</p>
    ${curveSVG([seriesOf('jload', id)], { kind: 'jload', label: what + ' по ходу круга, в весах тела' })}
    ${peak != null ? `<p class="meter-foot">${nb('Больше всего за круг: ' + fBW(peak) + ' веса тела, около ' + kgf(peak * PHYS.dog.bw) + '\u00a0кг' + (J.trans.peakT != null ? ', на ' + fmtSec(J.trans.peakT) : '') + '.')}</p>` : ''}`;
}
function renderCycleNow() {
  const t = state.cycT, h = cycHoldAt(t), ph = cycPhaseAt(t), noAct = actPlaceholder();
  const big = u => !PHYS.stand.muscles[u] || PHYS.stand.muscles[u].Fmax >= 150;
  const top = noAct ? [] : UNIT_IDS.filter(big).map(u => { const L = cycAct(u, 'L'), R = cycAct(u, 'R'); return { u, L, R, a: Math.max(L, R) }; }).filter(o => o.a >= ACTIVE_MIN).sort((a, b) => b.a - a.a).slice(0, 8);
  const stretchNow = UNIT_IDS.map(u => ({ u, s: unitAvg(u) })).filter(o => o.s > 0.05).sort((a, b) => b.s - a.s).slice(0, 5);
  const ligNow = Object.keys(A.LIGS).map(u => ({ u, s: Math.max(cycStrain(u, 'L'), cycStrain(u, 'R')) })).filter(o => o.s > 0.02).sort((a, b) => b.s - a.s).slice(0, 5);
  const title = h >= 0 ? 'Поза «' + POSE[CYC.holds[h].pose].name + '»' : ph.label;
  info.innerHTML = `
    <p class="card-eyebrow">${nb('Сейчас · ' + fmtSec(t))}</p>
    <h2 class="card-title">${nb(title)}</h2>
    ${noAct ? '' : top.length ? `<h3 class="card-sub">${nb('Работают сильнее всего, доля максимальной силы')}</h3><ul class="rank">${top.map(o => `<li><button class="rank-row" data-unit="${o.u}"><span class="rank-name">${nb(A.UNITS[o.u].name)}</span><span class="num work-t">${pctTxt(o.a)}</span>${effBar(o.a, 'work')}${Math.abs(o.L - o.R) >= 0.03 ? `<span class="rank-sub">${nb('слева ' + pctTxt(o.L) + ', справа ' + pctTxt(o.R))}</span>` : ''}</button></li>`).join('')}</ul>` : `<p class="meter-foot">${nb('Сейчас ни одна крупная мышца не работает сильнее 8% своей силы.')}</p>`}
    ${stretchNow.length ? `<h3 class="card-sub">${nb('Сильнее всего удлинены')}</h3><ul class="rank">${stretchNow.map(o => `<li><button class="rank-row" data-unit="${o.u}"><span class="rank-name">${nb(A.UNITS[o.u].name)}</span><span class="num warm-t">${fmtPct(o.s)}</span>${barHTML(o.s)}</button></li>`).join('')}</ul>` : ''}
    ${ligNow.length ? `<h3 class="card-sub">${nb('Связки под натяжением')}</h3><ul class="rank">${ligNow.map(o => `<li><button class="rank-row" data-lig="${o.u}"><span class="rank-name">${nb(A.LIGS[o.u].name)}</span><span class="num warm-t">${fmtPct(o.s)}</span></button></li>`).join('')}</ul>` : ''}
    <p class="card-hint">${nb('Нажми на мышцу, связку или сустав на модели, чтобы увидеть, как они работают по ходу круга.')}</p>`;
  bindCardButtons();
}
// live numbers of the open card while the round plays
function updateCycleLive() {
  const s = state.sel; if (!s) return;
  const mark = document.getElementById('cyc-live-mark');
  if (mark) { const x = curveX(state.cycT).toFixed(1); mark.setAttribute('x1', x); mark.setAttribute('x2', x); }
  if (s.kind === 'muscle') {
    const aL = cycAct(s.unit, 'L'), aR = cycAct(s.unit, 'R'), a = Math.max(aL, aR);
    const el = document.getElementById('cyc-live-a');
    if (el) {
      el.textContent = pctTxt(a);
      document.getElementById('cyc-live-abar').style.width = Math.min(100, a * 100).toFixed(1) + '%';
      document.getElementById('cyc-live-foot').textContent = nb('слева ' + pctTxt(aL) + ', справа ' + pctTxt(aR));
      const tag = document.getElementById('cyc-live-tag'); if (tag) tag.hidden = a < ACTIVE_MIN;
    }
    const len = document.getElementById('cyc-live-len');
    if (len) { const v = unitAvg(s.unit); len.textContent = fmtPct(v); len.className = 'num ' + (v >= 0 ? 'warm' : 'cool') + '-t big'; document.getElementById('cyc-live-lenbar').innerHTML = barHTML(v); }
  } else if (s.kind === 'joint') {
    const el = document.getElementById('cyc-live-load');
    if (el) { const v = cycJointLoad(s.id); el.textContent = fBW(v) + '×'; document.getElementById('cyc-live-loadfoot').textContent = nb('веса тела, около ' + kgf(v * PHYS.dog.bw) + '\u00a0кг'); }
  } else if (s.kind === 'lig') {
    const v = Math.max(cycStrain(s.unit, 'L'), cycStrain(s.unit, 'R'));
    const len = document.getElementById('cyc-live-len');
    if (len) { len.textContent = fmtPct(v); len.className = 'num ' + (v > 0.02 ? 'warm' : 'muted') + '-t big'; document.getElementById('cyc-live-lenbar').innerHTML = barHTML(v); }
  }
}
function cycCardTick(now) {
  const busy = now - cyc.infoBusy < 900;
  const sel = state.sel;
  if (!sel || sel.kind === 'joint' || /^(fline|flines|fascia)$/.test(sel.kind)) {
    if (busy || now - cyc.lastCard < (cyc.playing ? (sel ? 300 : 600) : 120)) return;
    renderInfo(); cyc.lastCard = now; cyc.cardDirty = false;
  } else {
    if (now - cyc.lastLive < 90) return;
    updateCycleLive(); cyc.lastLive = now; cyc.cardDirty = false;
  }
}

// ---------- fascia and Myers' lines: cards ----------
const capFirst = t => t ? t.charAt(0).toUpperCase() + t.slice(1) : '';
const POSE_KEY = { dog: 'dog', plank: 'plank', half: 'half', bow: 'bow', fold: 'fold', handstand: 'handstand' };
const TYPE_RU = { fascia: 'Фасция', aponeurosis: 'Апоневроз', band: 'Фасциальный тяж', ligament: 'Связка', retinaculum: 'Удерживатель', septum: 'Межмышечная перегородка', sheet: 'Фасциальный листок', tendon: 'Сухожилие', membrane: 'Мембрана' };
const lineDot = id => { const c = FASC.LINE[id].color; return `<i class="dot" style="background:#${(c >>> 0).toString(16).padStart(6, '0')}"></i>`; };
const lineChip = id => `<button class="chip chip-sm" data-fline="${id}" aria-pressed="${state.sel && state.sel.id === id ? 'true' : 'false'}">${lineDot(id)}${nb(FASC.LINE[id].abbr || id)}</button>`;
function memberName(mb) {
  if (mb.kind === 'muscle') return A.UNITS[mb.unit] ? A.UNITS[mb.unit].name : mb.unit;
  if (mb.kind === 'lig') return A.LIGS[mb.unit] ? A.LIGS[mb.unit].name : mb.unit;
  return FASC.EL[mb.unit] ? FASC.EL[mb.unit].ru : mb.unit;
}
function memberBtn(mb) {
  const nm = nb(memberName(mb));
  if (mb.kind === 'muscle') return `<button class="link-btn" data-unit="${mb.unit}">${nm}</button>`;
  if (mb.kind === 'lig' && A.LIGS[mb.unit]) return `<button class="link-btn" data-lig="${mb.unit}">${nm}</button>`;
  if (FASC.EL[mb.unit]) return `<button class="link-btn" data-fel="${mb.unit}">${nm}</button>`;
  return `<span>${nm}</span>`;
}
function whereNow() { return state.mode === 'cycle' && CYC ? 'сейчас' : (state.t > 0.5 ? 'в позе «' + POSE[state.poseId].name + '»' : 'в стойке'); }
function bookNote(id) {
  if (!FASCIA_NOTES || !FASCIA_NOTES[id]) return '';
  if (state.mode === 'cycle' && CYC) {
    const seg = CYC.segs[cycSegAt(state.cycT)];
    const pose = seg ? seg.to : null;
    const n = pose && FASCIA_NOTES[id][pose];
    return n ? `<h3 class="card-sub">${nb('Майерс: к позе «' + (POSE[pose] ? POSE[pose].name : pose) + '»')}</h3><p class="role">${nb(capFirst(n))}</p>` : '';
  }
  const n = FASCIA_NOTES[id][POSE_KEY[state.poseId]];
  return n ? `<h3 class="card-sub">${nb('Майерс о позе «' + POSE[state.poseId].name + '»')}</h3><p class="role">${nb(capFirst(n))}</p>` : '';
}
function bindFasciaButtons() {
  info.querySelectorAll('[data-fline]').forEach(b => b.addEventListener('click', () => select({ kind: 'fline', id: b.dataset.fline })));
  info.querySelectorAll('[data-fel]').forEach(b => b.addEventListener('click', () => select({ kind: 'fascia', id: b.dataset.fel })));
}
function ensureFasciaM() { if (!fasciaM && FASC) fasciaM = FASC.withPhysics(FASC.measure(poseW, { paths: true }), fasciaPhys()); return fasciaM; }
function renderLine(id) {
  const m = ensureFasciaM(), li = FASC.lineInfo(id), L = m.lines[id + '_L'], R = m.lines[id + '_R'];
  const sL = L ? L.strain : 0, sR = R ? R.strain : sL, eng = Math.max(L ? L.engage || 0 : 0, R ? R.engage || 0 : 0);
  const asym = Math.abs(sL - sR) > 0.01;
  const top = L && L.actMaxUnit && L.actMax >= 0.05 ? 'Больше всех работает ' + A.UNITS[L.actMaxUnit].name.toLowerCase() + ': ' + pctTxt(L.actMax) + '.' : 'Мышцы линии почти не работают.';
  const groups = (L ? L.groups : []).map((g, i) => {
    const gi = li.groups[i] || {};
    const mem = [...new Map(g.members.map(mb => [mb.kind + mb.unit, mb])).values()];
    return `<li><span>${nb(g.ru)}</span><span class="num ${g.strain > 0.02 ? 'warm' : g.strain < -0.02 ? 'cool' : 'muted'}-t">${fmtPct(g.strain)}</span>${barHTML(g.strain, 0.3)}<span class="part-note">${mem.map(memberBtn).join(', ')}${gi.cov && gi.cov !== 'full' ? ' · ' + nb(gi.cov === 'static' ? 'в модели не растягивается' : gi.cov === 'missing' ? 'в модели нет' : 'в модели частично') : ''}</span></li>`;
  }).join('');
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Миофасциальная линия · Майерс')}</p>
    <h2 class="card-title">${lineDot(id)} ${nb(li.ru)}</h2>
    <p class="latin">${li.en}</p>
    <div class="meter"><span class="meter-label">${nb('Длина линии по сравнению со стойкой, ' + whereNow())}</span><span class="num ${sL > 0.02 ? 'warm' : sL < -0.02 ? 'cool' : 'muted'}-t big" id="fl-strain">${fmtPct(sL)}</span></div>
    ${barHTML(sL, 0.2)}
    ${asym ? `<p class="meter-foot">${nb('Слева ' + fmtPct(sL) + ', справа ' + fmtPct(sR) + '.')}</p>` : ''}
    <div class="meter"><span class="meter-label">${nb('Включённость: доля силы мышц линии, которая сейчас натянута')}</span><span class="num warm-t big" id="fl-eng">${pctTxt(eng)}</span></div>
    ${effBar(eng)}
    <p class="meter-foot">${nb(top)}</p>
    ${state.mode === 'cycle' && CYC && CYC.flS ? lineRoundBlock(id) : ''}
    <h3 class="card-sub">${nb('Участки линии')}</h3>
    <ul class="parts">${groups}</ul>
    <h3 class="card-sub">${nb('Что делает')}</h3><p class="role">${nb(capFirst(li.fn || ''))}</p>
    ${bookNote(id)}
    ${li.evidence ? `<p class="meter-foot">${nb('Доказательность: ' + li.evidence)}</p>` : ''}
    <h3 class="card-sub">${nb('Другие линии')}</h3>
    <div class="chips">${Object.keys(FASC.LINE).map(lineChip).join('')}</div>
    ${focusBtn()}`;
  bindCardButtons(); bindFasciaButtons(); if (state.mode === 'cycle' && CYC) bindCurve();
}
function renderFasciaEl(id) {
  const m = ensureFasciaM(), e = FASC.EL[id];
  const vL = m.el[id + '_L'] || m.el[id + '_C'] || m.el[id], vR = m.el[id + '_R'];
  const sL = vL ? vL.strain : 0, sR = vR ? vR.strain : sL;
  const mus = e.muscles || {};
  const muRow = (title, list) => list && list.length ? `<p class="role"><b>${nb(title)}:</b> ${list.map(u => A.UNITS[u] ? `<button class="link-btn" data-unit="${u}">${nb(A.UNITS[u].name)}</button>` : nb(u)).join(', ')}</p>` : '';
  const lines = (e.myers || []).filter(l => FASC.LINE[l]);
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb((TYPE_RU[e.type] || 'Фасция') + ' · Неттер, Синельников')}</p>
    <h2 class="card-title">${nb(e.ru)}</h2>
    <p class="latin">${e.lat || ''}</p>
    ${e.rigid ? `<p class="meter-foot">${nb('Оба конца на одной кости модели, поэтому её длина не меняется: показана для связей.')}</p>` : `
    <div class="meter"><span class="meter-label">${nb(e.pathProxy ? 'Путь между точками крепления, ' + whereNow() : 'Длина волокон по сравнению со стойкой, ' + whereNow())}</span><span class="num ${sL > 0.02 ? 'warm' : sL < -0.02 ? 'cool' : 'muted'}-t big">${fmtPct(sL)}</span></div>
    ${barHTML(sL, 0.3)}
    ${Math.abs(sL - sR) > 0.01 ? `<p class="meter-foot">${nb('Слева ' + fmtPct(sL) + ', справа ' + fmtPct(sR) + '.')}</p>` : ''}`}
    <h3 class="card-sub">${nb('Что делает')}</h3><p class="role">${nb(e.role || '')}</p>
    ${muRow('Прикрепляются', mus.inserts)}${muRow('Окутывает', mus.invested)}${muRow('Натягивают', mus.tensors)}
    ${e.attachments && e.attachments.length ? `<h3 class="card-sub">${nb('Крепится')}</h3><ul class="plain">${e.attachments.map(a => `<li>${nb(a)}</li>`).join('')}</ul>` : ''}
    ${lines.length ? `<h3 class="card-sub">${nb('Входит в линии Майерса')}</h3><div class="chips">${lines.map(lineChip).join('')}</div>` : ''}
    ${e.netter || e.sinelnikov ? `<p class="meter-foot">${nb([e.netter ? 'Неттер: ' + e.netter : '', e.sinelnikov ? 'Синельников: ' + e.sinelnikov : ''].filter(Boolean).join('. ') + '.')}</p>` : ''}
    ${focusBtn()}`;
  bindCardButtons(); bindFasciaButtons();
}
function lineRoundBlock(id) {
  const st = CYC.flines.lines[id];
  if (!st) return '';
  const R = roundStats(cyc.hold), H = R.H || 0;
  return `<h3 class="card-sub">${nb('По ходу круга')}</h3>
    ${curveSVG([seriesOf('flS', id)], { kind: 'lig', label: 'Длина линии по ходу круга' })}
    <ul class="parts">
      <li><span>${nb('Длиннее всего')}</span><span class="num warm-t">${fmtPct(st.sMax)}</span>${barHTML(st.sMax, 0.2)}<span class="part-note">${nb('на ' + fmtSec(st.sMaxT) + ' круга')}</span></li>
      <li><span>${nb('Короче всего')}</span><span class="num cool-t">${fmtPct(st.sMin)}</span>${barHTML(st.sMin, 0.2)}<span class="part-note">${nb('на ' + fmtSec(st.sMinT) + ' круга')}</span></li>
      ${CYC.flines.placeholder ? '' : `<li><span>${nb('Пик включённости')}</span><span class="num warm-t">${pctTxt(st.eMax)}</span>${effBar(st.eMax)}<span class="part-note">${nb('на ' + fmtSec(st.eMaxT) + ' круга')}</span></li>`}
    </ul>`;
}
function renderLinesOverview() {
  const m = ensureFasciaM();
  const eff = state.lightMode === 'effort';
  const rows = Object.keys(FASC.LINE).map(id => {
    const L = m.lines[id + '_L'], R = m.lines[id + '_R'];
    const s = L ? (R ? (L.strain + R.strain) / 2 : L.strain) : 0, e = Math.max(L ? L.engage || 0 : 0, R ? R.engage || 0 : 0);
    return `<li><button class="rank-row" data-fline="${id}"><span class="rank-name">${lineDot(id)} ${nb(FASC.LINE[id].ru)}</span><span class="num ${eff ? 'warm' : s > 0.02 ? 'warm' : s < -0.02 ? 'cool' : 'muted'}-t">${eff ? pctTxt(e) : fmtPct(s)}</span>${eff ? effBar(e) : barHTML(s, 0.2)}</button></li>`;
  }).join('');
  info.innerHTML = `${closeBtn()}
    <p class="card-eyebrow">${nb('Анатомические поезда · Майерс')}</p>
    <h2 class="card-title">${nb('Миофасциальные линии')}</h2>
    <p class="role">${nb('Двенадцать непрерывных цепочек мышц и фасций, по которым натяжение передаётся через всё тело. ' + (eff ? 'Цифра — включённость: какая доля силы мышц линии сейчас натянута, ' + whereNow() + '.' : 'Цифра — насколько линия длиннее или короче, чем в стойке, ' + whereNow() + '.'))}</p>
    <ul class="rank">${rows}</ul>
    <p class="meter-foot">${nb('Нажми на линию, чтобы увидеть её участки и что о ней говорят книги. Фасции из атласов Неттера и Синельникова включаются отдельным слоем.')}</p>`;
  bindCardButtons(); bindFasciaButtons();
}
// muscle and ligament cards also say which lines they belong to
function appendLinesOf() {
  const s = state.sel;
  if (!FASC || !s || !(s.kind === 'muscle' || s.kind === 'lig')) return;
  if (info.querySelector('[data-lines-of]')) return;
  const ids = FASC.linesOf(s.kind === 'muscle' ? 'muscle' : 'lig', s.unit).map(x => x.line).filter(id => FASC.LINE[id]);
  if (!ids.length) return;
  const div = document.createElement('div');
  div.setAttribute('data-lines-of', '1');
  div.innerHTML = `<h3 class="card-sub">${nb('Входит в линии Майерса')}</h3><div class="chips">${[...new Set(ids)].map(lineChip).join('')}</div>`;
  const fb = info.querySelector('.card-focus');
  if (fb) info.insertBefore(div, fb); else info.appendChild(div);
  bindFasciaButtons();
}

// ---------- boot ----------
if (PRACTICE_ALGORITHMS.length) {
  $('practice-mode').hidden = false;
  $('practice-algorithms').innerHTML = PRACTICE_ALGORITHMS.map(a => `<button class="btn" data-algorithm="${a.id}" aria-pressed="false">${practiceText(a.name)}</button>`).join('');
  $('practice-algorithms').querySelectorAll('[data-algorithm]').forEach(b => b.addEventListener('click', () => enterPractice(b.dataset.algorithm, 0)));
  $('practice-mode').addEventListener('click', () => enterPractice());
  $('main-mode').addEventListener('click', () => exitPractice());
  $('practice-prev').addEventListener('click', () => enterPractice(state.algorithmId, state.practiceStep - 1));
  $('practice-next').addEventListener('click', () => enterPractice(state.algorithmId, state.practiceStep + 1));
}
function applyPracticeHash() {
  const hash = location.hash.slice(1);
  if (hash === 'practice' || hash.startsWith('practice/')) {
    const match = /^practice\/([a-z]+)\/([1-8])$/.exec(hash);
    if (match && enterPractice(match[1], Number(match[2]) - 1, false)) return;
    const first = PRACTICE_ALGORITHMS[0];
    if (first && enterPractice(first.id, 0, false)) setPageHash('practice/' + first.id + '/1', true);
    return;
  }
  if (state.mode === 'practice') exitPractice(true, false);
  if (Object.hasOwn(POSE, hash)) switchPose(hash);
  else if (hash === 'cycle' || hash === 'krug') { enterCycle(0, false); cycPause(); cycMorph = null; }
}
window.addEventListener('hashchange', applyPracticeHash);
for (const P of POSES) STATS[P.id] = computeStats(P.id);
{
  const h = (location.hash || '').replace('#', '');
  if (Object.hasOwn(POSE, h)) { state.poseId = h; activeSet = activeOf(h); VIEW_DIR = new THREE.Vector3(...POSE[h].view).normalize(); }
}
buildForces(state.poseId);
if (typeof createConnectiveSurfaces === 'function') {
  connectiveSurfaces = createConnectiveSurfaces(A, THREE, { capsuleRims: CAPSULE_RIM_DATA && CAPSULE_RIM_DATA.capsuleRims });
  if (connectiveSurfaces) {
    model.add(connectiveSurfaces.group); syncSurfaceVisibility();
    stage.dataset.capsuleSurfaces = connectiveSurfaces.parts.filter(part => part.kind === 'cap').every(part => part.displayFit === 'bone-rim') ? 'fitted' : 'schematic';
  }
}
applyPose(1);
resize();
resetView(false);
setT(1);
updateHeader();
renderCues();
renderInfo();
$('stage').classList.add('ready');
stage.dataset.muscleSurfaces = 'loading';
stage.dataset.jointSurfaces = 'loading';
// Replace only the exact schematic structures covered by a decoded package.
// Minor joints and spine retain their existing surfaces; failure retains all
// schematic joint surfaces and reports that state in the source note.
setTimeout(async () => {
  try {
    const surfaces = typeof createJointSurfaces === 'function'
      ? await createJointSurfaces(A, THREE, JOINT_SURFACE_DATA) : null;
    if (!surfaces || !surfaces.covered.size) {
      if (surfaces) surfaces.dispose();
      stage.dataset.jointSurfaces = 'fallback'; updateJointSourceNote('fallback'); return;
    }
    surfaces.update(poseW);
    jointSurfaces = surfaces;
    for (const mesh of cartMeshes.slice()) {
      if (surfaces.covered.has(mesh.userData.seg + '|' + mesh.userData.name)) removeModelMesh(mesh);
    }
    model.add(surfaces.group);
    for (const part of surfaces.parts) {
      part.mesh.userData.baseOpacity = part.baseOpacity;
      cartMeshes.push(part.mesh);
      (glowByJoint[part.joint] ||= []).push(part.mesh);
    }
    syncSurfaceVisibility(); poseDirty = true;
    stage.dataset.jointSurfaces = 'ready'; updateJointSourceNote('ready');
    if (state.sel && state.sel.kind === 'joint') renderInfo();
  } catch (error) {
    stage.dataset.jointSurfaces = 'fallback'; updateJointSourceNote('fallback');
    console.warn('Joint surface reconstructions could not load; schematic surfaces remain visible.', error);
  }
}, 0);
// Keep the procedural muscles visible while the anatomical meshes decode and
// acquire their visual skin weights. A failed import retains that complete view.
setTimeout(async () => {
  try {
    const surfaces = typeof createMuscleSurfaces === 'function'
      ? await createMuscleSurfaces(A, THREE, MUSCLE_SURFACE_DATA, { instances: insts, fiberTexture: fibers, world: poseW }) : null;
    if (!surfaces || !surfaces.covered.size) { if (surfaces) surfaces.dispose(); stage.dataset.muscleSurfaces = 'fallback'; updateMuscleSourceNote('fallback'); return; }
    muscleSurfaces = surfaces;
    model.add(surfaces.group);
    syncSurfaceVisibility();
    poseDirty = true;
    stage.dataset.muscleSurfaces = 'ready';
    updateMuscleSourceNote('ready');
    if (state.sel && state.sel.kind === 'muscle') renderInfo();
  } catch (error) {
    stage.dataset.muscleSurfaces = 'fallback';
    updateMuscleSourceNote('fallback');
    console.warn('Anatomical muscle surfaces could not load; procedural muscles remain visible.', error);
  }
}, 0);
if (CYC) {
  cycleTab.hidden = false;
  buildCycMarks();
  // reduced motion: the round is not played by itself, only scrubbed
  if (reduceMotion) { cycPlayBtn.hidden = true; document.querySelector('.cyc .seg-inline').hidden = true; }
  const h0 = (location.hash || '').replace('#', '');
  if (h0 === 'cycle' || h0 === 'krug') enterCycle(0, false);
}
if (location.hash === '#practice' || location.hash.startsWith('#practice/')) applyPracticeHash();

let lastInfoT = -1;
const cycFollow = new THREE.Vector3(), _fd = new THREE.Vector3();
function frame(now) {
  requestAnimationFrame(frame);
  if (state.mode === 'cycle') cycStep(now);
  else if (state.playing) {
    const p = state.playing, u = Math.min(1, (now - p.t0) / p.dur);
    const e = u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
    setT(p.from + (1 - p.from) * e);
    if (u >= 1) { state.playing = null; setPlayLabel(); }
  }
  if (morph) {
    const u = Math.min(1, (now - morph.t0) / morph.dur);
    morph.u = u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
    poseDirty = true;
    if (u >= 1) { morph = null; morphDone = true; }
  }
  if (cycMorph) {
    const u = Math.min(1, (now - cycMorph.t0) / cycMorph.dur);
    cycMorph.u = u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2;
    poseDirty = true;
    if (u >= 1) cycMorph = null;
  }
  if (poseDirty) {
    applyPose(state.t); poseDirty = false;
    if (state.mode === 'cycle') cyc.cardDirty = true;
    // the open card follows the slider, and is redrawn once the switch to another pose has finished
    else if (researchPoseKey() !== lastResearchKey || (state.sel && (morphDone || Math.abs(state.t - lastInfoT) > 0.004))) { renderInfo(); lastInfoT = state.t; }
    morphDone = false;
  }
  if (state.mode === 'cycle' && cyc.cardDirty) cycCardTick(now);
  // in the round the middle of the body moves along the mat: the camera goes with it, also after the user turned it
  _fd.copy(cycFollow); cycFollow.lerp(centerTarget, 0.12); _fd.subVectors(cycFollow, _fd);
  if (state.camAnim) state.camAnim(now);
  else if (!state.userMoved) {
    controls.target.lerp(centerTarget, state.mode === 'cycle' ? 0.08 : 0.1);
    const want = controls.target.clone().addScaledVector(VIEW_DIR, fitDistance());
    camera.position.lerp(want, 0.1);
  } else if (state.mode === 'cycle') { controls.target.add(_fd); camera.position.add(_fd); }
  updateMaterials(now);
  updateForces();
  controls.update();
  composer.render();
}
requestAnimationFrame(frame);
/*__DEBUG__*/
