import { execSync } from 'child_process';
import fs from 'fs';
const src = fs.readFileSync('poses_build.mjs', 'utf8');
for (const c of JSON.parse(process.argv[2])) {
  let s = src;
  for (const [k, v] of Object.entries(c)) s = s.replace(new RegExp('(const HALF = \\{[^\\n]*?\\b' + k + ': )(\\[[^\\]]*\\]|[-\\d.]+)'), '$1' + JSON.stringify(v));
  fs.writeFileSync('poses_scan_tmp.mjs', s);
  const out = execSync('node poses_scan_tmp.mjs half && node jcheck.mjs poses_dbg.json half', { encoding: 'utf8' });
  const g = l => (out.split('\n').find(x => x.startsWith(l)) || '').replace(/\(rom \d+\)/g, '');
  const mm = JSON.parse((out.split('\n').find(x => x.includes('"elbowBack"')) || '{}').trim());
  console.log(JSON.stringify(c), '\n   ', g('gh_L'), '\n   ', g('el_L'), '|', g('wr_L'), '|', g('st_L'), '\n    elbowBack', mm.elbowBack, 'gap', mm.elbowGap, 'handH', mm.handH, 'chest', mm.chest);
}
fs.unlinkSync('poses_scan_tmp.mjs');
