// books/yoga_analysis.md → books/yoga_analysis.json (structured; the markdown is the source of truth) + model numbers per pose.
import fs from 'fs';
const md = fs.readFileSync(new URL(process.argv[2] || './yoga_analysis.md', import.meta.url), 'utf8');
const D = JSON.parse(fs.readFileSync(new URL('./yoga_model_data.json', import.meta.url), 'utf8'));
const round = fs.existsSync(new URL('./yoga_round.json', import.meta.url)) ? JSON.parse(fs.readFileSync(new URL('./yoga_round.json', import.meta.url), 'utf8')) : null;
const POSE_IDS = { 'Горка': 'dog', 'Планка на предплечьях': 'plank', 'Полупрогиб': 'half', 'Корзинка': 'bow', 'Наклон': 'fold' };
const MATCH = { '✅': 'yes', '🟡': 'partial', '❌': 'no', '—': 'n/a', '': 'n/a' };
const sections = md.split(/\n## /).slice(1);
const out = { title: 'Модель против йога-анатомии: Коултер и Лонг', sources: [], poses: {}, transitions: null, summary: {}, method: '' };
for (const sec of sections) {
  const head = sec.split('\n')[0].trim();
  const m = head.match(/^\d+\.\s+(.*)$/);
  const name = m ? m[1] : head;
  const pid = Object.entries(POSE_IDS).find(([k]) => name.startsWith(k));
  if (pid) {
    const rows = [];
    for (const line of sec.split('\n')) {
      if (!line.startsWith('|') || /^\|\s*(Тема|---)/.test(line)) continue;
      const c = line.split('|').slice(1, -1).map(s => s.trim());
      if (c.length < 5) continue;
      rows.push({ topic: c[0], books: c[1], model: c[2], match: MATCH[c[3]] ?? c[3], why: c[4] === '—' ? '' : c[4] });
    }
    const id = pid[1];
    const units = Object.entries(D.units).filter(([u, v]) => v[id]);
    const top = units.filter(([, v]) => v[id].a >= 0.05).sort((a, b) => b[1][id].a - a[1][id].a).map(([u, v]) => ({ unit: u, name: D.names[u], a: v[id].a, cueTone: v[id].tone || 0 }));
    const stretched = units.filter(([, v]) => v[id].stretchMean > 0.08).sort((a, b) => b[1][id].stretchMean - a[1][id].stretchMean).map(([u, v]) => ({ unit: u, name: D.names[u], stretch: v[id].stretchMean, passiveN: v[id].Fp || 0 }));
    const j = D.joints[id];
    out.poses[id] = {
      name, variant: sec.split('\n')[0].includes('(') ? head.replace(/^\d+\.\s+/, '') : name, rows,
      score: { yes: rows.filter(r => r.match === 'yes').length, partial: rows.filter(r => r.match === 'partial').length, no: rows.filter(r => r.match === 'no').length },
      model: { topActivations: top, stretched, L5S1: j.ls && j.ls.jr, floor: D.floor[id], jointReactionsBW: Object.fromEntries(['gh_L', 'el_L', 'wr_L', 'hip_L', 'kn_L', 'an_L'].filter(k => j[k] && j[k].jr).map(k => [k.replace('_L', ''), j[k].jr.bw])), patellofemoralBW: j.kn_L && j.kn_L.pf ? j.kn_L.pf.bw : null },
    };
  } else if (/^Переходы/.test(name)) {
    out.transitions = { note: 'Книги описывают в основном удержание поз; для переходов — принципы (Коултер, гл. 1; Лонг: мягкий вход и выход). Таблица книги ↔ модель — books/yoga_round.mjs → yoga_round.md/json.', expected: [], model: round };
    for (const line of sec.split('<!-- ROUND:BEGIN -->')[0].split('\n')) { if (!line.startsWith('|') || /^\|\s*(Переход|---)/.test(line)) continue; const c = line.split('|').slice(1, -1).map(s => s.trim()); out.transitions.expected.push({ transition: c[0], books: c[1] }); }
  } else if (/^Итог/.test(name)) {
    const parts = sec.split('**').map(s => s.trim());
    const grab = title => { const i = parts.findIndex(p => p.startsWith(title)); if (i < 0) return []; return parts[i + 1].split(/\n(?=- |\d+\. )/).map(s => s.replace(/^(- |\d+\. )/, '').replace(/\s*\n\s*/g, ' ').trim()).filter(Boolean); };
    out.summary = { agree: grab('Где модель и книги сходятся'), disagreeText: sec.split('**Где расходятся и почему**')[1]?.split('**Что стоит добавить')[0].trim(), improve: grab('Что стоит добавить в модель по книгам') };
  } else if (/^Практические выводы/.test(name)) {
    out.practice = sec.split(/\n(?=- )/).slice(1).map(x => x.replace(/^- /, '').replace(/\*\*/g, '').replace(/\s*\n\s*/g, ' ').trim()).filter(Boolean);
  } else if (/^Источники/.test(name)) {
    out.sources = sec.split('\n').filter(l => l.startsWith('- ')).map(l => l.slice(2).trim());
  } else if (/^Как читать/.test(name)) out.method = sec.split('\n').slice(1).join('\n').trim();
}
out.scoreTotal = Object.values(out.poses).reduce((a, p) => ({ yes: a.yes + p.score.yes, partial: a.partial + p.score.partial, no: a.no + p.score.no }), { yes: 0, partial: 0, no: 0 });
fs.writeFileSync(new URL('./yoga_analysis.json', import.meta.url), JSON.stringify(out, null, 1));
console.log(Object.entries(out.poses).map(([k, p]) => `${k}: ${p.rows.length} rows, yes ${p.score.yes} partial ${p.score.partial} no ${p.score.no}`).join('\n'), '\ntotal', out.scoreTotal, '\nsummary agree', out.summary.agree.length, 'improve', out.summary.improve.length, 'sources', out.sources.length, 'transitions', out.transitions && out.transitions.expected.length);
