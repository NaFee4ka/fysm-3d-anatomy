# Фасции и линии Майерса — модуль для страницы (агент fascia)

Файлы: `fascia3.js` (данные и расчёт), `fascia_render.js` (three.js), `books/fascia_model.json` (74 КБ, собирается
`node fascia_model_build.mjs` из `books/fascia_structures.json` — атлас Неттера/Синельникова, и `books/myers_lines.json` —
линии Майерса; id обоих файлов сохранены, id фасций Майерса → структуры атласа через `aliases`).
Проверки: `node fascia_test.mjs` (позы → `books/fascia_pose_table.json`), `node fascia_round.mjs` (круг →
`books/fascia_round.json`; перезапустить после настоящих cycle.json и cycle_phys.json), вид без страницы —
`books/fascia_view.html` + `python3 books/fascia_shots.py "pose:view:mode:lines:layers;…"`.

## Встраивание в однофайловую страницу
- `fascia3.js` инлайнить внутри IIFE (у него верхнеуровневые `row`, `G`, `GRIDS`… — могут совпасть с именами app3.js):
  `const FASCIA_LIB = (() => { <код без слова export>; return { createFascia, LINE_COLORS }; })();`
- `fascia_render.js` — одна функция `createFasciaRender`, можно просто убрать `export`.
- `books/fascia_model.json` — как physics.json: `/*__FASCIA__*/null`.

## API
```js
const F = FASCIA_LIB.createFascia(A, THREE, FASCIA_MODEL);       // A = createAnatomy(THREE)
const FR = createFasciaRender(F, A, THREE, { xray: true });       // линии видны сквозь мышцы
model.add(FR.group);
// при смене позы (P.W — мировые трансформы):
const phys = F.physFromStatic(PHYS[poseId]);                       // или кадр круга: F.physFromFrame({ aL, aR }, units, FmaxByUnit)
const fm = F.withPhysics(F.measure(P.W, { paths: true }), phys);   // ≈ 3 мс
FR.update(P.W, fm, { mode: 'length' | 'effort', light: state.light, time: now, reduceMotion });
// стоимость (node, V8): всё сразу ≈ 18 мс, только фасции ≈ 10 мс, 24 линии ≈ 4 мс, скрытые слои — 0;
// при проигрывании круга достаточно обновлять раз в 2 кадра
FR.setLayer('sheets' | 'bands' | 'lines' | 'stations', on);       // листки, тяжи, линии, станции (костные точки)
FR.showLines(['SBL', 'SFL'] | null);                               // какие линии видны (null — все)
FR.focus({ line: 'SBL' } | { el: 'tlfPost' } | null);              // выделить, остальное приглушить; станции линии видны
FR.pickables                                                       // userData { kind: 'fascia'|'line'|'station', id|line, side }
```
Данные для карточек:
- `F.EL[id]`: `ru, lat, type, role, attachments[], muscles{inserts, invested, tensors}, netter, sinelnikov, myers[], link`
  (link — связка, которую модель уже рисует: supraspinousLig→ssl, nuchalLig→nuchal, inguinalLig, sacrotuberousLig, itTract→itb,
  plantarAponeurosis→plantar); живое: `fm.el[id + '_L'].strain / .max / .fibres`.
- `F.lineInfo(id)`: `ru, en, abbr, color, fn, evidence, crossing, sideNote, groups[{n, ru, lat, side A|B|C|X, cov, from, to, members}]`;
  живое: `fm.lines[id + '_L']`: `strain` (геометрия, к стойке), `chainStrain`, `engage` ((Tact+Tpas)/ΣFmax), `actMean`,
  `actMax`, `actMaxUnit`, `Tact`, `Tpas` (Н), `groups[].strain`.
- `F.linesOf('muscle'|'lig'|'fascia', unit)` → в какие линии входит (для карточек мышц и связок).
- Цвета линий: `F.LINE[id].color` (ПЗЛ синяя, ПФЛ красная, ЛЛ жёлтая, СЛ фиолетовая, ГФЛ зелёная, руки — оранжевые/голубые,
  функциональные — розовые/фиолетовые).

## Что считается
- strain элемента — среднее по волокнам L/L0 − 1 (путь с обтеканием, как у мышц), к стойке; у сухожилий (`pathProxy`) это
  изменение пути «голень — пятка», а не растяжение ткани.
- strain линии — длина цепи из представителей участков + промежутки между ними (костные станции) к стойке.
- engage — доля мышечной ёмкости линии под натяжением: (Σ a·Fmax + Σ Fp) / Σ Fmax.
