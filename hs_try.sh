#!/bin/bash
# usage: ./hs_try.sh '{"thC":-5}'  -> builds the handstand with these overrides, merges it into poses.json, runs physics, prints a summary
cd "$(dirname "$0")"
HS="$1" node poses_build.mjs handstand > /tmp/hs_build.log 2>&1 || { cat /tmp/hs_build.log; exit 1; }
node -e "
const fs=require('fs');
const a=JSON.parse(fs.readFileSync('poses.json','utf8')), b=JSON.parse(fs.readFileSync('poses_dbg.json','utf8'));
a.handstand=b.handstand; fs.writeFileSync('poses.json', JSON.stringify(a));
"
node physics_build.mjs handstand > /tmp/hs_phys.log 2>&1 || { cat /tmp/hs_phys.log; exit 1; }
node -e "
const o=JSON.parse(require('fs').readFileSync('physics_dbg.json','utf8')).handstand;
const J=o.joints, e=(j,c)=>{const x=J[j]&&J[j].comps.find(k=>k.c===c); return x? x.need+'/'+x.cap+' ('+x.eff+(x.res?' RES '+x.res:'')+')':'-'};
const jr=J.ls.jr;
console.log('L5S1',jr.comp,'N',(jr.comp/o.bw).toFixed(2),'BW | margin',(o.support.margin*100).toFixed(1),'cm | reserves',o.diag.reserves,'| floor',o.floor.map(f=>f.region[0]+f.side+' '+f.pct).join(' '));
console.log('need/cap  t45',e('t45','x'),' t89',e('t89','x'),' tl',e('tl','x'),' sc.y',e('sc_L','y'),' gh.x',e('gh_L','x'),' gh.z',e('gh_L','z'),' el.x',e('el_L','x'),' el.p',e('el_L','p'),' wr.x',e('wr_L','x'),' wr.z',e('wr_L','z'));
console.log(Object.entries(o.muscles).sort((a,b)=>b[1].a-a[1].a).slice(0,14).map(([u,m])=>u+' '+Math.round(m.a*100)).join(', '));
"
grep -o '"cm z rel. shoulder".*' /tmp/hs_build.log | head -1
