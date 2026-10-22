// Writes cycle-style frames (local, root, rootP) for the poses of a poses json, for shot_conf.py.
// usage: node hs_frame.mjs poses_dbg.json handstand OUT.json
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const [file, id, out] = process.argv.slice(2);
const d = JSON.parse(fs.readFileSync(file, 'utf8'))[id];
const c = A.poseFromData(d);
const W0 = A.computeWorld(c.local, c.rootQ, new THREE.Vector3(0, 0, 0));
let minY = Infinity, zmin = Infinity, zmax = -Infinity;
for (const l of A.LANDMARKS) { const p = A.pt(W0, l.seg, l.pc); minY = Math.min(minY, p.y); zmin = Math.min(zmin, p.z); zmax = Math.max(zmax, p.z); }
const rootP = [0, -minY, -(zmin + zmax) / 2].map(v => +v.toFixed(6));
fs.writeFileSync(out, JSON.stringify([{ local: d.local, root: d.root, rootP, label: id }]));
console.log('frame', id, 'rootP', rootP);
