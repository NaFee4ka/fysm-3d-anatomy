// Provisional round for developing physics and the player before the real segments exist:
// plain slerp of every joint between the anchors (bodies pass through the floor; only the data format is real).
import fs from 'fs';
import { A, V, THREE, computeWorld, rootPlaced, ease } from './cycle_lib.mjs';

const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const anchors = JSON.parse(fs.readFileSync('cycle_keys/anchors.json', 'utf8'));
const fold = (() => { const c = A.poseFromData(data.fold); const rp = rootPlaced(c.local, c.rootQ); rp.z += 0.45; return { local: data.fold.local, root: data.fold.root, rootP: [rp.x, rp.y, rp.z] }; })();
const K = { ...anchors, fold };
const SEGS = [
  { id: 'seg1', from: 'dog', to: 'plank', duration: 4.0, phases: [[2.0, 'Сгибаю локти и опускаю предплечья'], [4.0, 'Опускаю таз в планку']] },
  { id: 'seg2', from: 'plank', to: 'half', duration: 5.0, phases: [[2.4, 'Опускаюсь на живот'], [3.4, 'Вытягиваю рёбра вперёд'], [5.0, 'Свожу лопатки — полупрогиб']] },
  { id: 'seg3', from: 'half', to: 'bow', duration: 4.0, phases: [[1.0, 'Сгибаю колени'], [2.5, 'Беру голени изнутри'], [4.0, 'Поднимаюсь в корзинку']] },
  { id: 'seg4', from: 'bow', to: 'highplank', duration: 5.0, phases: [[2.0, 'Отпускаю руки и ноги'], [3.2, 'Ставлю ладони под плечи'], [5.0, 'Выжимаюсь в планку']] },
  { id: 'seg5', from: 'highplank', to: 'fold', duration: 5.0, phases: [[1.6, 'Подшагиваю правой ногой'], [3.2, 'Подшагиваю левой'], [5.0, 'Поднимаюсь в наклон']] },
];
// provisional high plank: the forearm plank lifted on straight arms is not built here; the forearm plank stands in for it
K.highplank = { ...anchors.plank, rootP: [anchors.plank.rootP[0], anchors.plank.rootP[1], anchors.plank.rootP[2] - 0.1] };
const q = a => new THREE.Quaternion(...a);
const r6 = v => +v.toFixed(6);
for (const s of SEGS) {
  const a = K[s.from], b = K[s.to];
  const n = Math.round(s.duration * 10);
  const frames = [];
  for (let i = 0; i <= n; i++) {
    const u = ease(i / n);
    const local = {};
    for (const seg of A.SEG) local[seg.name] = new THREE.Quaternion().slerpQuaternions(q(a.local[seg.name]), q(b.local[seg.name]), u).toArray().map(r6);
    const root = new THREE.Quaternion().slerpQuaternions(q(a.root), q(b.root), u).toArray().map(r6);
    const rootP = a.rootP.map((v, k) => r6(v + (b.rootP[k] - v) * u));
    const grip = (s.id === 'seg3' && i / n > 0.6) || (s.id === 'seg4' && i / n < 0.3) ? ['L', 'R'] : [];
    frames.push({ t: +(i / 10).toFixed(2), local, root, rootP, grip });
  }
  let t0 = 0;
  const phases = s.phases.map(([t1, label]) => { const p = { t0, t1, label }; t0 = t1; return p; });
  fs.writeFileSync(`cycle_keys/prov_${s.id}.json`, JSON.stringify({ id: s.id, from: s.from, to: s.to, fps: 10, duration: s.duration, phases, provisional: true, frames }));
  console.log(s.id, frames.length, 'frames');
}
