// usage: node phys_try.mjs pose '{"tone":{...},"pg":true,"cone":1,"wP":0.01}'
import * as THREE from 'three';
import fs from 'fs';
import { createAnatomy } from './anatomy3.js';
import { makePhysics } from './physics_core.mjs';
import { CUE_TONE, PASSIVE_GROUPS } from './physics_params.mjs';
const A = createAnatomy(THREE);
const PH = makePhysics(A, { mass: 70 });
const data = JSON.parse(fs.readFileSync('poses.json', 'utf8'));
const id = process.argv[2] || 'fold';
const opt = process.argv[3] ? JSON.parse(process.argv[3]) : {};
const conf = id === 'stand' ? A.STAND : A.poseFromData(data[id]);
const W = A.placeW(conf.local, conf.rootQ);
const grips = (id === 'bow' || id === 'fold') ? ['L', 'R'].map(sd => { const l = A.LANDMARKS.find(x => x.tag === 'palm' && x.side === sd); return { hand: 'hand_' + sd, shank: 'shank_' + sd, P: A.pt(W, l.seg, l.p), side: sd, from: opt.cone != null ? 'hum_' + sd : undefined, cone: opt.cone }; }) : [];
const tone = Object.assign({}, CUE_TONE[id] || {}, opt.tone || {});
const r = PH.solve({ local: conf.local, rootQ: conf.rootQ }, { tone, grips, iters: 16000, passiveGroups: opt.pg ? PASSIVE_GROUPS : null, wP: opt.wP, passiveLo: opt.lo, passiveHi: opt.hi });
const f1 = v => v.toFixed(1);
for (const g of r.gripF) if (g.side === 'L') { const u = g.P.clone().sub(r.W.hum_L.p).normalize(); console.log('grip L |F|', f1(g.F.length()), 'along arm (tension +)', f1(g.F.dot(u))); }
const show = re => r.dofOut.filter(d => re.test(d.id)).forEach(d => console.log(d.id.padEnd(10), 'ext', f1(d.ext), 'st', f1(d.st), 'mus', f1(d.mus), 'pas', f1(d.pas), 'res', f1(d.res)));
if (opt.dofs) show(new RegExp(opt.dofs));
const mus = {};
r.INST.forEach((s, j) => { if (s.side !== 'L') return; const m = (mus[s.unit] ||= { F: 0, Fmax: 0, Fp: 0 }); m.F += r.force[j]; m.Fmax += r.FMAX[j]; m.Fp += r.FP[j]; });
const top = Object.entries(mus).map(([u, m]) => [u, m.F / m.Fmax, m.F, m.Fp]).sort((a, b) => b[1] - a[1]).slice(0, opt.n || 22);
console.log(top.map(([u, a, F, Fp]) => `${u} ${Math.round(a * 100)}% ${Math.round(F)}N${Fp > 5 ? ' p' + Math.round(Fp) : ''}`).join(' | '));
console.log('passive:', Object.entries(mus).filter(([, m]) => m.Fp > 5).map(([u, m]) => u + ' ' + Math.round(m.Fp)).join(', '));
console.log('pscale:', Object.entries(r.pscale).filter(([k]) => k.endsWith('_L')).map(([k, v]) => k + ' ' + v.toFixed(2)).join(', '));
console.log('resid', r.maxRes.toExponential(1), 'reserves', Object.entries(r.reserve).filter(([, v]) => Math.abs(v) > 0.3).map(([k, v]) => k + ' ' + f1(v)).join(', '), '| L5S1 comp', Math.round(r.jr.L5S1.compression), (r.jr.L5S1.compression / r.BW).toFixed(2) + 'BW');
const fl = {}; for (const c of r.contactF) { const k = c.region; fl[k] = (fl[k] || 0) + c.F.y; } console.log('floor %:', Object.entries(fl).filter(([, v]) => v > 1).map(([k, v]) => k + ' ' + Math.round(v / r.BW * 100)).join(', '));
console.log('limb stops:', Object.entries(r.spP).filter(([k, v]) => /^(el|wr|kn|an)_L/.test(k) && Math.abs(v) > 0.05).map(([k, v]) => k + ' ' + v.toFixed(2)).join(', '));
