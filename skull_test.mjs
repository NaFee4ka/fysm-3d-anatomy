import fs from 'fs';
const src = fs.readFileSync('skull3.js', 'utf8');
const f = new Function(src + '\nreturn skullData(0, 1.595, -0.036);');
const t0 = Date.now(); const D = f(); console.log('ms', Date.now() - t0);
for (const [k, g] of Object.entries(D)) { let mn=1,mx=0; for (const c of g.col){mn=Math.min(mn,c);mx=Math.max(mx,c);} console.log(k, 'verts', g.pos.length / 3, 'tris', g.idx.length / 3, 'ao', mn.toFixed(2), mx.toFixed(2)); }
