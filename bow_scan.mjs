import { execSync } from 'child_process';
import fs from 'fs';
const src = fs.readFileSync('poses_build.mjs', 'utf8');
for (const c of JSON.parse(process.argv[2])) {
  let s = src.replace(/ghER: 45, supMax: 75, gripX: [-\d.]+, gripY: [-\d.]+, gripZ: [-\d.]+/, `ghER: ${c.er ?? 90}, supMax: ${c.sup ?? 90}, gripX: ${c.x ?? 0.07}, gripY: ${c.y ?? 0.16}, gripZ: ${c.z ?? -0.012}`);
  s = s.replace('4 * Math.max(0, r.er - 85)', `4 * Math.max(0, r.er - ${c.erCap ?? 90})`);
  fs.writeFileSync('poses_scan_tmp.mjs', s);
  const out = execSync('node poses_scan_tmp.mjs bow', { encoding: 'utf8' });
  const info = JSON.parse(out.split('\n')[0].replace(/^bow \d+ms /, ''));
  const meas = JSON.parse(out.split('\n')[1].trim());
  console.log(JSON.stringify(c), 'knee', info.knee.toFixed(1), 'er', info.er.toFixed(1), 'pron', info.pron.toFixed(1), 'sum', (info.er - info.pron).toFixed(1), 'wrist', info.wristExt.toFixed(1), info.wristDev.toFixed(1), 'feetH', meas.feetH, 'kneeGap', meas.kneeGap);
}
fs.unlinkSync('poses_scan_tmp.mjs');
