import { execSync } from 'child_process';
import fs from 'fs';
const src = fs.readFileSync('poses_build.mjs', 'utf8');
for (const c of JSON.parse(process.argv[2])) {
  const extra = Object.entries(c).map(([k, v]) => `${k}: ${JSON.stringify(v)}`).join(', ');
  const s = src.replace("humDir: [0.55, -0.7, -0.45], foreDir: [0.45, 0.75, 0.5], wristExt: 50", "humDir: [0.55, -0.7, -0.45], foreDir: [0.45, 0.75, 0.5], wristExt: 50, " + extra);
  fs.writeFileSync('poses_scan_tmp.mjs', s);
  const out = execSync('node poses_scan_tmp.mjs half && node jcheck.mjs poses_dbg.json half', { encoding: 'utf8' });
  const g = l => (out.split('\n').find(x => x.startsWith(l)) || '').replace(/\(rom \d+\)/g, '');
  const mm = JSON.parse((out.split('\n').find(x => x.includes('"elbowBack"')) || '{}').trim());
  console.log(JSON.stringify(c), '\n   ', g('gh_L'), '\n   ', g('el_L'), '|', g('wr_L'), '\n    elbowBack', mm.elbowBack, 'gap', mm.elbowGap, 'handH', mm.handH, 'chest', mm.chest);
}
fs.unlinkSync('poses_scan_tmp.mjs');
