// scan W-arm geometry for the half pose: report shoulder rotation, elbow, wrist, elbow position
import { execSync } from 'child_process';
import fs from 'fs';
const src = fs.readFileSync('poses_build.mjs', 'utf8');
const combos = JSON.parse(process.argv[2]);
for (const c of combos) {
  const patched = src.replace(/humDir: \[[^\]]*\], foreDir: \[[^\]]*\], wristExt: [\d.]+/, `humDir: ${JSON.stringify(c.h)}, foreDir: ${JSON.stringify(c.f)}, wristExt: ${c.e || 50}`)
    .replace(/scapProt: -20/, `scapProt: ${c.prot ?? -20}`);
  fs.writeFileSync('poses_scan_tmp.mjs', patched);
  const out = execSync('node poses_scan_tmp.mjs half && node jcheck.mjs poses_dbg.json half', { encoding: 'utf8' });
  const g = l => (out.split('\n').find(x => x.startsWith(l)) || '').replace(/\(rom \d+\)/g, '');
  const meas = out.split('\n').find(x => x.includes('"elbowBack"')) || '';
  const mm = JSON.parse(meas.trim());
  console.log(JSON.stringify(c), '\n   ', g('gh_L'), '\n   ', g('el_L'), '|', g('wr_L'), '\n    elbowBack', mm.elbowBack, 'gap', mm.elbowGap, 'handH', mm.handH, 'chest', mm.chest, 'knee', mm.knee);
}
fs.unlinkSync('poses_scan_tmp.mjs');
