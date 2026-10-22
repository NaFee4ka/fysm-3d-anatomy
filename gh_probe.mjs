import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
for (const id of (process.argv[2] || 'half,plank').split(',')) {
  const c = A.poseFromData(data[id]); const W = A.placeW(c.local, c.rootQ);
  for (const jid of ['gh_L', 'st_L', 'el_L', 'wr_L', 'hip_L']) {
    const r = A.analyzeJoint(A.JOINTS.find(j => j.id === jid), c.local, W);
    console.log(id, jid, r.mode || '', r.comps.map(x => x.label + ' ' + Math.round(x.value)).join(', '));
  }
}
