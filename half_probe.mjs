import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
const A = createAnatomy(THREE);
const data = JSON.parse(fs.readFileSync(process.argv[2] || 'poses_dbg.json', 'utf8'));
const c = A.poseFromData(data.half);
const W = A.placeW(c.local, c.rootQ);
const E = W.fore_L.p, G = W.hum_L.p;
console.log('elbow above shoulder', (E.y - G.y).toFixed(3), 'elbow toward head', (E.z - G.z).toFixed(3), 'elbow lateral', (E.x - G.x).toFixed(3), 'elbow height', E.y.toFixed(3), 'shoulder height', G.y.toFixed(3));
