import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const file = process.argv[2] || './poses.json';
const data = JSON.parse(fs.readFileSync(file, 'utf8'));
const ids = (process.argv[3] || 'bow,half').split(',');
for (const id of ids) {
  if (!data[id]) continue;
  const c = A.poseFromData(data[id]);
  const W = A.placeW(c.local, c.rootQ);
  console.log('==', id);
  for (const j of A.JOINTS) {
    if (!/_L$/.test(j.id)) continue;
    if (!/^(gh|st|sc|el|wr|hip|kn|an)_/.test(j.id)) continue;
    const r = A.analyzeJoint(j, c.local, W);
    console.log(j.id, r.comps.map(x => `${x.key}:${x.label}=${x.signed.toFixed(1)}(rom ${x.rom})`).join(' | '));
  }
}
