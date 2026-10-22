// Transitions of the round against what Coulter and Long's principles predict (agent yoga).
// usage: node books/yoga_round.mjs   → books/yoga_round.md + books/yoga_round.json
// Reads cycle_phys.json (per-frame activations aL/aR) and cycle_stats.json; marks the output provisional when the round is.
import fs from 'fs';
const root = new URL('../', import.meta.url);
const R = f => JSON.parse(fs.readFileSync(new URL(f, root), 'utf8'));
const phys = R('cycle_phys.json'), stats = R('cycle_stats.json');
const names = JSON.parse(fs.readFileSync(new URL('books/yoga_model_data.json', root), 'utf8')).names;
const U = phys.units;
const prov = !!stats.provisional || (phys.segs || []).some(s => s.provisional);
// what the books' principles predict for each segment (prime movers / controllers); "ecc" = works while lengthening
const EXPECT = {
  seg1: { title: 'Горка → Планка на предплечьях', why: 'локти сгибаются под весом — трицепс тормозит (уступающая работа, Коултер гл. 1); плечо удерживают передняя дельта и передняя зубчатая (Лонг); таз опускается — сгибатели бедра и пресс держат «доску» (Коултер гл. 3, Лонг)',
    units: { tricLat: 'ecc', tricMed: 'ecc', tricLong: 'ecc', deltAnt: 'iso', serratus: 'iso', rectAbd: 'iso', extObl: 'iso', psoas: 'ecc', iliacus: 'ecc', rectFem: 'iso', vastLat: 'iso' } },
  seg2: { title: 'Планка → лёг на живот → Полупрогиб', why: 'опускание — передняя цепь (пресс, поясничная, зубчатая) тормозит; затем разгибатели спины поднимают грудь, ромбовидные и трапеции сводят лопатки, задняя дельта уводит локти назад, ягодичные стабилизируют таз (Лонг, саранча; Коултер гл. 5)',
    units: { rectAbd: 'ecc', psoas: 'ecc', serratus: 'ecc', erector: 'conc', multifidus: 'conc', semispinalis: 'conc', splenius: 'conc', rhomb: 'conc', trapMid: 'conc', trapLow: 'conc', deltPost: 'conc', gluteMax: 'iso' } },
  seg3: { title: 'Полупрогиб → Корзинка', why: 'сгибатели колена поднимают голени (подколенные, икроножные); руки уходят назад — разгибатели плеча (широчайшая, большая круглая, задняя дельта, длинная головка трицепса); захват — сгибатели пальцев; затем четырёхглавые «отталкивают» голени, ягодичные и подколенные поднимают бёдра, разгибатели спины выгибают корпус (Лонг, лук; Коултер гл. 5)',
    units: { bicFemL: 'conc', semiT: 'conc', semiM: 'conc', gastroM: 'conc', lat: 'conc', teresMaj: 'conc', deltPost: 'conc', tricLong: 'conc', fingerFlex: 'iso', vastLat: 'conc', vastInt: 'conc', gluteMax: 'conc', erector: 'conc' } },
  seg4: { title: 'Корзинка → Планка на ладонях', why: 'выход из лука уступающий; затем отжимание — трицепс, большая грудная, передняя зубчатая, передняя дельта; корпус держат пресс, поясничная, четырёхглавые (Лонг, планка/чатуранга; Коултер гл. 3)',
    units: { tricLat: 'conc', tricMed: 'conc', tricLong: 'conc', pecMaj: 'conc', serratus: 'conc', deltAnt: 'conc', rectAbd: 'iso', extObl: 'iso', psoas: 'iso', vastLat: 'iso' } },
  seg5: { title: 'Планка → подшагнул → Наклон', why: 'колено к груди — поясничная, подвздошная, прямая бедра, пресс; вес на руках — трицепс, передняя зубчатая, передняя дельта; опорная нога — четырёхглавая и ягодичная; выпрямление колен в наклоне — четырёхглавые, подколенные удлиняются (Лонг, наклон; Коултер гл. 6)',
    units: { psoas: 'conc', iliacus: 'conc', rectFem: 'conc', rectAbd: 'conc', tricLat: 'iso', serratus: 'iso', deltAnt: 'iso', vastLat: 'conc', gluteMax: 'iso' } },
};
const ix = Object.fromEntries(U.map((u, i) => [u, i]));
const out = { provisional: prov, segs: {} };
let md = `## Переходы круга: что предсказывают книги и что показала модель${prov ? ' (ПРЕДВАРИТЕЛЬНО: провизорные кадры)' : ''}\n\n`;
md += 'Пик — наибольшая активация за переход (доля максимальной силы мышцы), среднее — за время перехода; L/R — левая/правая сторона. «Работает» = пик ≥ 15 % или среднее ≥ 8 %.\n\n';
for (const s of phys.segs) {
  const E = EXPECT[s.id]; if (!E) continue;
  const fr = phys.frames.filter(f => f.seg === s.id || (f.T >= s.t0 && f.T <= s.t1 && !f.seg));
  if (!fr.length) continue;
  const val = (u, side) => fr.map(f => f[side][ix[u]] || 0);
  const peak = (u) => Math.max(...val(u, 'aL'), ...val(u, 'aR'));
  const mean = (u) => (val(u, 'aL').reduce((a, b) => a + b, 0) + val(u, 'aR').reduce((a, b) => a + b, 0)) / (2 * fr.length);
  const rows = Object.entries(E.units).map(([u, mode]) => { const p = peak(u), m = mean(u); return { u, name: names[u], mode, peak: +p.toFixed(3), mean: +m.toFixed(3), works: p >= 0.15 || m >= 0.08 }; });
  const top = U.map(u => ({ u, p: peak(u), m: mean(u) })).sort((a, b) => b.p - a.p).slice(0, 10).map(x => ({ u: x.u, name: names[x.u], peak: +x.p.toFixed(3), mean: +x.m.toFixed(3) }));
  const hit = rows.filter(r => r.works).length;
  out.segs[s.id] = { title: E.title, why: E.why, rows, top, hit, of: rows.length, frames: fr.length };
  md += `### ${E.title}\nКниги: ${E.why}.\n\n| Мышца | Работа по книгам | Пик | Среднее | Работает в модели? |\n|---|---|---|---|---|\n`;
  for (const r of rows) md += `| ${r.name} | ${{ ecc: 'уступающая', conc: 'преодолевающая', iso: 'удерживающая' }[r.mode]} | ${Math.round(r.peak * 100)} % | ${Math.round(r.mean * 100)} % | ${r.works ? 'да' : 'нет'} |\n`;
  md += `\nСовпало ${hit} из ${rows.length}. Самые активные в модели: ${top.map(t => `${t.name} ${Math.round(t.peak * 100)} %`).join(', ')}.\n\n`;
}
// the whole round (transitions only, no holds): how many muscles worked and which ones carried the round
{
  const all = U.map((u, i) => {
    const pk = Math.max(...phys.frames.map(f => Math.max(f.aL[i] || 0, f.aR[i] || 0)));
    const st = stats.muscles && stats.muscles[u] && stats.muscles[u].trans;
    const integ = st ? (st.L.int + st.R.int) / 2 : 0;
    return { u, name: names[u], peak: pk, int: integ };
  });
  const n08 = all.filter(x => x.peak >= 0.08).length, n40 = all.filter(x => x.peak >= 0.4).length;
  const byInt = [...all].sort((a, b) => b.int - a.int).slice(0, 12);
  out.round = { muscles: U.length, peak08: n08, peak40: n40, topByIntegral: byInt.map(x => ({ unit: x.u, name: x.name, int: +x.int.toFixed(2), peak: +x.peak.toFixed(3) })), T: stats.T };
  md += `### Весь круг (только переходы, без задержек в позах)\nЗа ${stats.T} с переходов хотя бы раз включились на 8 % и больше ${n08} мышц из ${U.length}, на 40 % и больше — ${n40}.\n` +
    `Больше всего работы (интеграл активации, %·с) у: ${byInt.map(x => `${x.name} ${Math.round(x.int * 100)}`).join(', ')}.\n\n`;
}
fs.writeFileSync(new URL('books/yoga_round.md', root), md);
if (process.argv.includes('--into')) {
  const target = new URL(process.argv.includes('--test') ? 'books/yoga_analysis.test.md' : 'books/yoga_analysis.md', root);
  const src = fs.readFileSync(new URL('books/yoga_analysis.md', root), 'utf8');
  if (prov && !process.argv.includes('--force')) console.log('round is provisional: not written into yoga_analysis.md (use --force to test)');
  else {
    const body = md.replace(/^## [^\n]*\n\n/, '');
    const res = src.replace(/<!-- ROUND:BEGIN -->[\s\S]*?<!-- ROUND:END -->/, `<!-- ROUND:BEGIN -->\n${body.trim()}\n<!-- ROUND:END -->`);
    fs.writeFileSync(target, res);
    console.log('written into', target.pathname);
  }
}
fs.writeFileSync(new URL('books/yoga_round.json', root), JSON.stringify(out, null, 1));
console.log(md.slice(0, 1500));
