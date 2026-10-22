import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const PHYS = JSON.parse(fs.readFileSync('physics.json', 'utf8'));
for (const P of A.POSES) {
  const ph = PHYS[P.id];
  console.log('=====', P.id);
  for (const [u, t] of Object.entries(P.notes || {})) {
    const m = ph.muscles[u];
    console.log((u.padEnd(12) + (m ? (Math.round(m.a * 100) + '%').padStart(5) + (m.Fp ? ' p' + m.Fp + 'N' : '').padEnd(8) + (m.cue ? '*' : ' ') : '  n/a        ')) + ' ' + t);
  }
}
