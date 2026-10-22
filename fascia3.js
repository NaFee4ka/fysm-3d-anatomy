// Fascia and myofascial lines (Т. Майерс «Анатомические поезда») on top of the anatomy model.
//
//   import { createFascia } from './fascia3.js';
//   const F = createFascia(A, THREE, MODEL);   // A = createAnatomy(THREE); MODEL = books/fascia_model.json (atlas + Myers),
//                                              // without it the small built-in set below is used
//
// ELEMENTS  fascial sheets, aponeuroses, retinacula, septa. Geometry in the anatomy model's raw form: points
//           [seg|'auto', x, y, z] (left side, standing; 'auto' = trunk segment by height; arm points get armZ) or already
//           resolved {seg, p: [x, y, z]}; the right side is the mirror. strands = fibres (strain, drawing as bands),
//           surfaces = grids rows × cols drawn as translucent sheets (a grid without strands gives the fibres itself),
//           link = a ligament the anatomy model already has (no own geometry: its strands are used).
// LINES     Myers' lines as ordered chains of groups (tracks). A group = parallel members at one level of the chain:
//           {mus|lig|fas: id, idx?: [fibre indices]}. Group side: 'A' this side, 'B' the other side (spiral and functional
//           lines cross), 'C' midline, 'X' the crossing itself. Stations (bony attachments) are kept for markers and text.
//
// Metrics, all relative to plain standing (A.STAND):
//   element strain   mean over its fibres of L/L0 − 1 (L = path length with wrapping, as for muscle strands)
//   group strain     mean strain of the group's members (member = mean over its strands)
//   line strain      (Σ representative length of the groups + Σ straight gaps between consecutive groups) / same at
//                    standing − 1; gaps stand for bony stations (constant on one bone, changing across joints)
//   chainStrain      the same without the gaps
//   Tact             Σ over muscle members of a · Fmax (N), a = activation of that side (0..1)
//   Tpas             Σ over muscle members of Fp (N), passive force of the stretched muscle, when the physics gives it
//   engage           (Tact + Tpas) / Σ Fmax — the share of the line's muscular capacity under tension, active + passive
//   actMean          activation averaged over the muscle members weighted by their standing length; actMax and its unit
//   (members with idx subsets get Fmax and Fp scaled by their share of the unit's fibres)

// ---------- built-in set (used when no model file is given; also the grids the model builder borrows) ----------
const row = (y, cols, seg) => cols.map(([x, z]) => [seg || 'auto', x, y, z]);
export const GRIDS = {
  tlf: [
    row(0.95, [[0, -0.112], [0.018, -0.116], [0.034, -0.113]]), row(1.0, [[0, -0.11], [0.03, -0.118], [0.055, -0.108]]),
    row(1.05, [[0, -0.112], [0.04, -0.122], [0.085, -0.092]]), row(1.1, [[0, -0.113], [0.045, -0.123], [0.102, -0.082]]),
    row(1.15, [[0, -0.116], [0.045, -0.126], [0.106, -0.082]]), row(1.2, [[0, -0.121], [0.045, -0.129], [0.1, -0.09]]),
    row(1.25, [[0, -0.126], [0.035, -0.132], [0.07, -0.115]]),
  ],
  fasciaLata: [
    row(1.05, [[0.13, 0.04], [0.14, 0.0], [0.125, -0.045]], 'pelvis'), row(0.95, [[0.145, 0.05], [0.165, 0.0], [0.145, -0.06]], 'pelvis'),
    row(0.86, [[0.15, 0.055], [0.172, 0.0], [0.15, -0.06]], 'thigh'), row(0.72, [[0.145, 0.05], [0.168, 0.005], [0.145, -0.055]], 'thigh'),
    row(0.6, [[0.13, 0.05], [0.152, 0.01], [0.135, -0.05]], 'thigh'), row(0.49, [[0.115, 0.03], [0.13, 0.012], [0.13, -0.025]], 'shank'),
  ],
  cruralPost: [
    row(0.05, [[0.085, -0.068], [0.094, -0.074], [0.105, -0.068]], 'foot'), row(0.15, [[0.08, -0.056], [0.095, -0.063], [0.112, -0.056]], 'shank'),
    row(0.28, [[0.075, -0.08], [0.097, -0.088], [0.12, -0.08]], 'shank'), row(0.4, [[0.072, -0.079], [0.097, -0.086], [0.123, -0.078]], 'shank'),
    row(0.5, [[0.075, -0.052], [0.097, -0.058], [0.12, -0.052]], 'shank'), row(0.58, [[0.075, -0.056], [0.097, -0.062], [0.12, -0.056]], 'thigh'),
  ],
  cruralAnt: [
    row(0.035, [[0.08, 0.09], [0.098, 0.095], [0.116, 0.08]], 'foot'), row(0.06, [[0.082, 0.06], [0.098, 0.07], [0.115, 0.055]], 'foot'),
    row(0.1, [[0.09, 0.045], [0.105, 0.043], [0.118, 0.035]], 'shank'), row(0.22, [[0.098, 0.043], [0.112, 0.041], [0.126, 0.031]], 'shank'),
    row(0.34, [[0.105, 0.036], [0.118, 0.035], [0.13, 0.026]], 'shank'), row(0.44, [[0.105, 0.031], [0.118, 0.027], [0.128, 0.017]], 'shank'),
  ],
  rectusSheath: [
    row(0.9, [[0.008, 0.097], [0.03, 0.095], [0.05, 0.09]], 'pelvis'), row(0.97, [[0.008, 0.107], [0.04, 0.106], [0.07, 0.1]], 'pelvis'),
    row(1.05, [[0.008, 0.115], [0.045, 0.114], [0.085, 0.106]]), row(1.13, [[0.008, 0.116], [0.05, 0.115], [0.09, 0.106]]),
    row(1.21, [[0.008, 0.117], [0.055, 0.113], [0.095, 0.098]]), row(1.28, [[0.008, 0.112], [0.06, 0.105], [0.1, 0.088]]),
  ],
};
export const WRAPS = {
  HIP_BIG: [['thigh', 0.1, 0.925, 0.0], 0.062],
  KNEE_BACK: [['thigh', 0.097, 0.52, -0.012], [0.04, 0.034, 0.045]],
  ANKLE_FRONT: [['foot', 0.095, 0.06, -0.006], [0.028, 0.03, 0.036]],
};
export const BUILTIN = {
  elements: [
    { id: 'tlf', ru: 'Грудопоясничная фасция, задний листок', lat: 'fascia thoracolumbalis, lamina posterior', type: 'fascia', side: 'LR', surfaces: [GRIDS.tlf], gridFibres: 'both', rowFrom: 2, rowTo: 6 },
    { id: 'tlfCross', ru: 'Грудопоясничная фасция, перекрёст', lat: 'fascia thoracolumbalis, lamina superficialis', type: 'band', side: 'LR', strands: [{ pts: [['auto', 0.1, 1.17, -0.098], ['auto', 0.05, 1.08, -0.12], ['pelvis', 0.0, 1.0, -0.113], ['pelvis', -0.036, 0.975, -0.117]], cross: true }] },
    { id: 'fasciaLata', ru: 'Широкая фасция бедра', lat: 'fascia lata', type: 'fascia', side: 'LR', surfaces: [GRIDS.fasciaLata], gridFibres: 'cols', wraps: [WRAPS.HIP_BIG] },
    { id: 'calcanealTendon', ru: 'Пяточное (ахиллово) сухожилие', lat: 'tendo calcaneus', type: 'band', side: 'LR', pathProxy: true, r: 0.0045, strands: [[['shank', 0.093, 0.16, -0.05], ['shank', 0.094, 0.09, -0.053], ['foot', 0.094, 0.04, -0.071]]] },
    { id: 'cruralFascia', ru: 'Фасция голени', lat: 'fascia cruris', type: 'fascia', side: 'LR', surfaces: [GRIDS.cruralPost, GRIDS.cruralAnt], gridFibres: 'cols', wraps: [WRAPS.KNEE_BACK] },
    { id: 'galeaAponeurotica', ru: 'Надчерепной апоневроз', lat: 'galea aponeurotica', type: 'aponeurosis', side: 'LR', strands: [{ pts: [['head', 0, 1.625, -0.098], ['head', 0, 1.7, -0.087], ['head', 0, 1.757, -0.02], ['head', 0, 1.738, 0.05], ['head', 0, 1.692, 0.099]], central: true }, [['head', 0.05, 1.63, -0.089], ['head', 0.055, 1.71, -0.062], ['head', 0.05, 1.745, 0.0], ['head', 0.045, 1.72, 0.06], ['head', 0.04, 1.676, 0.099]]] },
    { id: 'lineaAlba', ru: 'Белая линия живота', lat: 'linea alba', type: 'aponeurosis', side: 'C', r: 0.0024, strands: [[['auto', 0, 1.255, 0.114], ['auto', 0, 1.2, 0.117], ['auto', 0, 1.15, 0.116], ['auto', 0, 1.1, 0.116], ['auto', 0, 1.05, 0.115], ['pelvis', 0, 1.0, 0.112], ['pelvis', 0, 0.95, 0.106], ['pelvis', 0, 0.885, 0.096]]] },
    { id: 'rectusSheath', ru: 'Влагалище прямой мышцы живота', lat: 'vagina musculi recti abdominis', type: 'aponeurosis', side: 'LR', surfaces: [GRIDS.rectusSheath], gridFibres: 'cols' },
    { id: 'sternalFascia', ru: 'Грудинная фасция', lat: 'fascia sternalis', type: 'fascia', side: 'C', r: 0.0024, strands: [[['thB', 0, 1.255, 0.113], ['thB', 0, 1.3, 0.116], ['thB', 0, 1.35, 0.108], ['thB', 0, 1.4, 0.083]]] },
    { id: 'palmarAponeurosis', ru: 'Ладонный апоневроз', lat: 'aponeurosis palmaris', type: 'aponeurosis', side: 'LR', r: 0.0014, strands: [[0.026, 0.72, 'f2a'], [0.008, 0.715, 'f3a'], [-0.01, 0.72, 'f4a'], [-0.026, 0.728, 'f5a']].map(([z, y, fs]) => [['hand', 0.198, 0.795, 0.005], ['hand', 0.199, 0.758, z * 0.75], [fs, 0.201, y - 0.014, z]]) },
    { id: 'brachialFascia', ru: 'Фасция плеча с межмышечными перегородками', lat: 'fascia brachii', type: 'septum', side: 'LR', strands: [[['hum', 0.19, 1.33, 0.02], ['hum', 0.183, 1.2, 0.01], ['hum', 0.178, 1.085, -0.002]], [['hum', 0.212, 1.27, 0.0], ['hum', 0.222, 1.17, -0.01], ['hum', 0.232, 1.085, 0.0]]] },
    { id: 'clavipectoralFascia', ru: 'Ключично-грудная фасция', lat: 'fascia clavipectoralis', type: 'fascia', side: 'LR', strands: [[['clav', 0.1, 1.388, 0.052], ['scap', 0.154, 1.407, 0.031]], [['scap', 0.153, 1.404, 0.03], ['auto', 0.085, 1.31, 0.09]]] },
  ],
  lines: null, // filled below
};
const G = (ru, members, side, rep) => ({ ru, members, side: side || 'A', rep: rep || null });
BUILTIN.lines = [
  { id: 'SBL', ru: 'Поверхностная задняя линия', groups: [G('Подошвенный апоневроз и короткий сгибатель пальцев', [{ lig: 'plantar' }, { mus: 'fdb' }]), G('Икроножная мышца и ахиллово сухожилие', [{ mus: 'gastroM' }, { mus: 'gastroL' }, { fas: 'calcanealTendon' }]), G('Мышцы задней поверхности бедра', [{ mus: 'bicFemL' }, { mus: 'semiT' }, { mus: 'semiM' }]), G('Крестцово-бугорная связка', [{ lig: 'sacrotub' }]), G('Крестцово-поясничная фасция и выпрямитель позвоночника', [{ mus: 'erector' }, { fas: 'tlf' }]), G('Полуостистая мышца головы', [{ mus: 'semispinalis' }]), G('Надчерепной апоневроз', [{ fas: 'galeaAponeurotica' }], 'C')] },
  { id: 'SFL', ru: 'Поверхностная фронтальная линия', groups: [G('Передняя большеберцовая и разгибатели пальцев', [{ mus: 'tibAnt' }, { mus: 'edl' }, { mus: 'ehl' }, { fas: 'cruralFascia', idx: [3, 4, 5] }]), G('Связка надколенника', [{ lig: 'patTend' }]), G('Четырёхглавая мышца', [{ mus: 'rectFem' }, { mus: 'vastLat' }, { mus: 'vastMed' }, { mus: 'vastInt' }]), G('Прямая мышца живота и её влагалище', [{ mus: 'rectAbd' }, { fas: 'rectusSheath' }]), G('Грудинная фасция', [{ fas: 'sternalFascia' }], 'C'), G('Грудино-ключично-сосцевидная', [{ mus: 'scm' }]), G('Фасция скальпа', [{ fas: 'galeaAponeurotica' }])] },
  { id: 'LL', ru: 'Латеральная линия', groups: [G('Малоберцовые мышцы', [{ mus: 'fibL' }, { mus: 'fibB' }]), G('Латеральная коллатеральная связка', [{ lig: 'lcl' }]), G('Подвздошно-большеберцовый тракт и напрягатель широкой фасции', [{ lig: 'itb' }, { mus: 'tfl' }, { mus: 'gluteMax', idx: [0] }, { fas: 'fasciaLata' }]), G('Косые мышцы живота', [{ mus: 'extObl' }, { mus: 'intObl' }]), G('Ременная и грудино-ключично-сосцевидная', [{ mus: 'splenius' }, { mus: 'scm' }])] },
  { id: 'SPL', ru: 'Спиральная линия', groups: [G('Ременная мышца', [{ mus: 'splenius' }]), G('Ромбовидные', [{ mus: 'rhomb' }], 'B'), G('Передняя зубчатая', [{ mus: 'serratus' }], 'B'), G('Наружная косая', [{ mus: 'extObl' }], 'B'), G('Белая линия', [{ fas: 'lineaAlba' }], 'C'), G('Внутренняя косая', [{ mus: 'intObl' }]), G('Напрягатель широкой фасции и тракт', [{ mus: 'tfl' }, { lig: 'itb' }]), G('Передняя большеберцовая', [{ mus: 'tibAnt' }]), G('Длинная малоберцовая', [{ mus: 'fibL' }]), G('Двуглавая мышца бедра', [{ mus: 'bicFemL' }]), G('Крестцово-бугорная связка', [{ lig: 'sacrotub' }]), G('Выпрямитель позвоночника', [{ mus: 'erector' }, { fas: 'tlf' }])] },
  { id: 'DFL', ru: 'Глубокая фронтальная линия', groups: [G('Задняя большеберцовая и длинные сгибатели пальцев', [{ mus: 'tibPost' }, { mus: 'fdl' }, { mus: 'fhl' }]), G('Подколенная', [{ mus: 'popliteus' }]), G('Приводящие', [{ mus: 'addMag' }, { mus: 'addLong' }, { mus: 'addBrev' }]), G('Поясничная, подвздошная, гребенчатая', [{ mus: 'psoas' }, { mus: 'iliacus' }, { mus: 'pectineus' }]), G('Передняя продольная связка', [{ lig: 'all' }], 'C'), G('Длинная мышца шеи и лестничные', [{ mus: 'longusColli' }, { mus: 'scalene' }])] },
  { id: 'SFAL', ru: 'Поверхностная фронтальная линия руки', groups: [G('Большая грудная и широчайшая', [{ mus: 'pecMaj' }, { mus: 'lat' }]), G('Медиальная межмышечная перегородка', [{ fas: 'brachialFascia', idx: [0] }]), G('Сгибатели запястья и пальцев', [{ mus: 'wristFlex' }, { mus: 'palmaris' }, { mus: 'fingerFlex' }]), G('Ладонный апоневроз', [{ fas: 'palmarAponeurosis' }])] },
  { id: 'DFAL', ru: 'Глубокая фронтальная линия руки', groups: [G('Малая грудная и ключично-грудная фасция', [{ mus: 'pecMin' }, { fas: 'clavipectoralFascia' }]), G('Двуглавая и клювовидно-плечевая', [{ mus: 'bicShort' }, { mus: 'bicLong' }, { mus: 'coracobr' }]), G('Лучевая коллатеральная связка', [{ lig: 'rcl' }]), G('Возвышение большого пальца', [{ mus: 'thenar' }])] },
  { id: 'SBAL', ru: 'Поверхностная задняя линия руки', groups: [G('Трапециевидная', [{ mus: 'trapUp' }, { mus: 'trapMid' }, { mus: 'trapLow' }]), G('Дельтовидная', [{ mus: 'deltMid' }, { mus: 'deltPost' }, { mus: 'deltAnt' }]), G('Латеральная межмышечная перегородка', [{ fas: 'brachialFascia', idx: [1] }]), G('Разгибатели запястья и пальцев', [{ mus: 'wristExt' }])] },
  { id: 'DBAL', ru: 'Глубокая задняя линия руки', groups: [G('Ромбовидные и поднимающая лопатку', [{ mus: 'rhomb' }, { mus: 'levScap' }]), G('Вращательная манжета', [{ mus: 'infra' }, { mus: 'teresMin' }, { mus: 'supra' }, { mus: 'subscap' }]), G('Трёхглавая', [{ mus: 'tricLong' }, { mus: 'tricLat' }, { mus: 'tricMed' }]), G('Локтевая коллатеральная связка', [{ lig: 'ucl' }]), G('Возвышение мизинца', [{ mus: 'hypothenar' }])] },
  { id: 'BFL', ru: 'Задняя функциональная линия', groups: [G('Широчайшая', [{ mus: 'lat' }]), G('Перекрёст грудопоясничной фасции', [{ fas: 'tlfCross' }], 'X'), G('Большая ягодичная', [{ mus: 'gluteMax' }], 'B'), G('Латеральная широкая', [{ mus: 'vastLat' }], 'B'), G('Связка надколенника', [{ lig: 'patTend' }], 'B')] },
  { id: 'FFL', ru: 'Передняя функциональная линия', groups: [G('Большая грудная, нижние волокна', [{ mus: 'pecMaj', idx: [3, 4] }]), G('Край влагалища прямой мышцы', [{ mus: 'rectAbd' }, { fas: 'rectusSheath' }]), G('Длинная приводящая', [{ mus: 'addLong' }], 'B')] },
  { id: 'IFL', ru: 'Ипсилатеральная функциональная линия', groups: [G('Широчайшая, наружные волокна', [{ mus: 'lat', idx: [4] }]), G('Наружная косая', [{ mus: 'extObl' }]), G('Портняжная', [{ mus: 'sartorius' }])] },
];
export const LINE_COLORS = { SBL: 0x4fa3ff, SFL: 0xff6b6b, LL: 0xffd166, SPL: 0xc77dff, DFL: 0x06d6a0, SFAL: 0xff9f1c, DFAL: 0xffbf69, SBAL: 0x4cc9f0, DBAL: 0x4895ef, BFL: 0xf72585, FFL: 0xe85d9a, IFL: 0x9d4edd };
const R_BY_TYPE = { aponeurosis: 0.0024, fascia: 0.0019, band: 0.003, ligament: 0.0028, retinaculum: 0.0022, septum: 0.0016, sheet: 0.0019 };

export function createFascia(A, THREE, model) {
  const M = model && model.elements ? model : BUILTIN;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);
  const { sideName, AZ, armZ } = A;
  const segForY = y => (y < 1.095 ? 'lumA' : y < 1.2 ? 'lumB' : y < 1.287 ? 'thA' : y < 1.372 ? 'thB' : y < 1.458 ? 'thC' : y < 1.522 ? 'neckA' : 'neckB');
  const trunkSeg = y => (y < 1.02 ? 'pelvis' : y >= 1.595 ? 'head' : segForY(y));
  // raw [seg|'auto', x, y, z] (armZ applied) or resolved {seg, p}
  const resolvePt = r => {
    if (Array.isArray(r)) { const base = r[0] === 'auto' ? trunkSeg(r[2]) : r[0]; return { base, p: V(...AZ(base, [r[1], r[2], r[3]])) }; }
    return { base: r.seg.replace(/_[LR]$/, ''), p: V(r.p[0], r.p[1], r.p[2]) };
  };
  const mirP = (q, side) => ({ seg: sideName(q.base, side), p: V(side === 'R' ? -q.p.x : q.p.x, q.p.y, q.p.z) });
  const mkWraps = (ws, side) => (ws || []).map(([[n, x, y, z], r, depth]) => ({ seg: sideName(n, side), c: V(side === 'R' ? -x : x, y, armZ(n, x, z)), r: Array.isArray(r) ? V(r[0], r[1], r[2]) : V(r, r, r), depth: depth || 1 }));
  const AINST = A.strandInstances();

  // ---------- elements ----------
  const ELEMENTS = [], EL = {}, instances = [];
  for (const d of M.elements) {
    const e = Object.assign({}, d);
    e.central = e.side === 'C';
    e.r = e.r || R_BY_TYPE[e.type] || 0.002; e.flat = e.flat ?? 0.42;
    const grids = e.surfaces || (e.rows ? [e.rows] : []);
    // fibres
    let fibres = (e.strands || []).map(s => (Array.isArray(s) ? { pts: s } : s));
    if (!fibres.length && grids.length && !e.link) {
      for (const gr of grids) {
        const nr = gr.length, nc = gr[0].length;
        if (e.gridFibres !== 'rows') for (let c = 0; c < nc; c++) fibres.push({ pts: gr.map(r => r[c]), label: 'вертикальные волокна ' + (c + 1) });
        if (e.gridFibres === 'rows' || e.gridFibres === 'both') for (let r = (e.rowFrom || 0); r < (e.rowTo ?? nr); r++) fibres.push({ pts: gr[r], label: 'поперечные волокна ' + (r + 1), wraps: [] });
      }
    }
    e.fibres = fibres.map(f => ({ label: f.label || null, cross: !!f.cross, central: !!f.central || (e.central && (Array.isArray(f.pts[0]) ? f.pts.every(p => p[1] === 0) : f.pts.every(p => p.p[0] === 0))), pts: f.pts.map(resolvePt), wraps: f.wraps || e.wraps || [] }));
    e.inst = { L: [], R: [] };
    for (const side of ['L', 'R']) {
      if (e.link) { e.inst[side] = AINST.filter(i => i.kind === 'lig' && i.unit === e.link && i.side === side); continue; }
      e.fibres.forEach((f, k) => {
        const inst = { id: 'fascia_' + e.id + '_' + k + '_' + side, kind: 'fascia', unit: e.id, side, idx: k, n: e.fibres.length, r: e.r, flat: e.flat, pts: f.pts.map(q => mirP(q, side)), wraps: mkWraps(f.wraps, side), central: f.central, cross: f.cross, label: f.label };
        e.inst[side].push(inst); instances.push(inst);
      });
    }
    e.gridInst = grids.length ? { L: grids.map(gr => gr.map(r => r.map(q => mirP(resolvePt(q), 'L')))), R: grids.map(gr => gr.map(r => r.map(q => mirP(resolvePt(q), 'R')))) } : null;
    // drawing surfaces as columns that slide over the element's obstacles like its fibres (instance format of A.strandPath)
    e.gridCols = e.gridInst ? Object.fromEntries(['L', 'R'].map(side => [side, e.gridInst[side].map(gr => gr[0].map((_, c) => ({ pts: gr.map(r => r[c]), wraps: mkWraps(e.wraps, side) })))])) : null;
    ELEMENTS.push(e); EL[e.id] = e;
  }
  const ALIAS = M.aliases || {};

  // ---------- members ----------
  const bySel = {};
  const instOf = (kind, unit, side) => (bySel[kind + ':' + unit + ':' + side] ||= AINST.filter(i => i.kind === kind && i.unit === unit && i.side === side));
  const memberInst = (m, side) => {
    if (m.mus) return instOf('muscle', m.mus, side).filter((i, k) => !m.idx || m.idx.includes(i.idx));
    if (m.lig) return instOf('lig', m.lig, side);
    if (m.fas) { const e = EL[m.fas]; if (!e) return []; return e.inst[side].filter((i, k) => !m.idx || m.idx.includes(k)); }
    return [];
  };
  const memberKey = m => (m.mus ? 'mus:' + m.mus : m.lig ? 'lig:' + m.lig : 'fas:' + m.fas) + (m.idx ? '[' + m.idx.join(',') + ']' : '');
  const otherSide = s => (s === 'L' ? 'R' : 'L');

  // ---------- lines ----------
  const LINES = M.lines.map(l => Object.assign({ color: LINE_COLORS[l.id] ?? 0xffffff }, l));
  const LINE = Object.fromEntries(LINES.map(l => [l.id, l]));
  const W_STAND = A.placeW(A.STAND.local, A.STAND.rootQ);
  const lenOf = (W, inst) => A.pathLength(A.strandPath(W, inst));
  const L0 = new Map();
  for (const i of instances) L0.set(i.id, lenOf(W_STAND, i));
  for (const i of AINST) L0.set(i.id, lenOf(W_STAND, i));
  for (const i of instances) i.L0 = L0.get(i.id);
  const resolved = {};
  function resolveLine(l, side) {
    const key = l.id + '_' + side;
    if (resolved[key]) return resolved[key];
    const groups = l.groups.map((gr, gi) => {
      const gs = gr.side === 'B' ? otherSide(side) : side;
      const members = gr.members.map(m => {
        const inst = memberInst(m, gs);
        const kind = m.mus ? 'muscle' : m.lig ? 'lig' : 'fascia', unit = m.mus || m.lig || m.fas;
        const all = m.mus ? instOf('muscle', m.mus, gs).length : 1;
        return { spec: m, key: memberKey(m), side: gs, inst, kind, unit, share: m.mus && m.idx ? inst.length / Math.max(1, all) : 1 };
      }).filter(m => m.inst.length);
      let rep = null;
      if (gr.rep) { const ri = memberInst(gr.rep, gs); if (ri.length) rep = ri[Math.floor((ri.length - 1) / 2)]; }
      if (!rep && members.length) { const ri = members[0].inst; rep = ri[Math.floor((ri.length - 1) / 2)]; }
      return { n: gr.n ?? gi + 1, ru: gr.ru, lat: gr.lat, sideCode: gr.side || 'A', side: gs, cov: gr.cov, note: gr.note, members, rep, from: gr.from || null, to: gr.to || null, related: gr.related || [] };
    }).filter(gr => gr.rep);
    const P = groups.map(gr => A.strandPath(W_STAND, gr.rep));
    const flip = groups.map(() => false);
    for (let k = 0; k < groups.length; k++) {
      const a = P[k][0], b = P[k][P[k].length - 1];
      if (k === 0) { if (groups.length > 1) { const n = P[1]; const d = x => Math.min(x.distanceTo(n[0]), x.distanceTo(n[n.length - 1])); flip[k] = d(a) < d(b); } }
      else { const prev = flip[k - 1] ? P[k - 1][0] : P[k - 1][P[k - 1].length - 1]; flip[k] = b.distanceTo(prev) < a.distanceTo(prev); }
    }
    // stations: given for the left version of a line; the right version is the mirror image
    const stations = [];
    for (const gr of groups) for (const st of [gr.from, gr.to]) if (st && st.at) stations.push({ ru: st.ru, seg: side === 'L' ? st.at.seg : swapSide(st.at.seg), p: V(side === 'L' ? st.at.p[0] : -st.at.p[0], st.at.p[1], st.at.p[2]) });
    const res = { line: l, side, groups, flip, stations };
    resolved[key] = res;
    const m0 = measureLineRaw(res, W_STAND);
    res.L0 = m0.L; res.chainL0 = m0.chainL;
    return res;
  }
  const swapSide = s => (/_L$/.test(s) ? s.replace(/_L$/, '_R') : /_R$/.test(s) ? s.replace(/_R$/, '_L') : s);
  function measureLineRaw(res, W) {
    let L = 0, chainL = 0;
    const paths = res.groups.map((gr, k) => { const p = A.strandPath(W, gr.rep); return res.flip[k] ? p.slice().reverse() : p; });
    for (let k = 0; k < paths.length; k++) {
      const lk = A.pathLength(paths[k]); L += lk; chainL += lk;
      if (k > 0) L += paths[k - 1][paths[k - 1].length - 1].distanceTo(paths[k][0]);
    }
    return { L, chainL, paths };
  }

  // ---------- measuring a configuration (W = world transforms) ----------
  function measure(W, opt = {}) {
    const len = new Map();
    const Lget = inst => { let v = len.get(inst.id); if (v === undefined) { v = opt.strandLen ? opt.strandLen(inst) : lenOf(W, inst); len.set(inst.id, v); } return v; };
    const strainOf = inst => Lget(inst) / L0.get(inst.id) - 1;
    const el = {};
    for (const e of ELEMENTS) for (const side of ['L', 'R']) {
      const fs = e.inst[side].map(strainOf);
      el[e.id + '_' + side] = fs.length ? { strain: fs.reduce((a, b) => a + b, 0) / fs.length, max: Math.max(...fs), min: Math.min(...fs), fibres: fs } : { strain: 0, max: 0, min: 0, fibres: [] };
    }
    const lines = {};
    for (const l of LINES) for (const side of ['L', 'R']) {
      const res = resolveLine(l, side);
      const m = measureLineRaw(res, W);
      const groups = res.groups.map(gr => {
        const ms = gr.members.map(mb => { const s = mb.inst.map(strainOf); return { key: mb.key, kind: mb.kind, unit: mb.unit, side: mb.side, share: mb.share, L0: mb.inst.reduce((a, i) => a + L0.get(i.id), 0) / mb.inst.length, strain: s.reduce((a, b) => a + b, 0) / s.length }; });
        return { n: gr.n, ru: gr.ru, sideCode: gr.sideCode, strain: ms.reduce((a, b) => a + b.strain, 0) / ms.length, members: ms };
      });
      lines[l.id + '_' + side] = { id: l.id, side, L: m.L, L0: res.L0, strain: m.L / res.L0 - 1, chainStrain: m.chainL / res.chainL0 - 1, groups, path: opt.paths ? [].concat(...m.paths) : undefined };
    }
    return { el, lines };
  }
  // physics on top: phys = { a: {unit: {L, R}}, Fmax: {unit: N per side}, Fp?: {unit: {L, R}} }
  function withPhysics(m, phys) {
    if (!phys) return m;
    for (const lm of Object.values(m.lines)) {
      let Tact = 0, Tpas = 0, Fm = 0, aw = 0, w = 0, amax = 0, amaxU = null;
      for (const gr of lm.groups) for (const mb of gr.members) {
        if (mb.kind !== 'muscle') continue;
        const a = phys.a[mb.unit] ? phys.a[mb.unit][mb.side] || 0 : 0;
        const Fmax = (phys.Fmax[mb.unit] || 0) * mb.share;
        const Fp = phys.Fp && phys.Fp[mb.unit] ? (phys.Fp[mb.unit][mb.side] || 0) * mb.share : 0;
        mb.a = a; mb.F = a * Fmax; mb.Fp = Fp;
        Tact += a * Fmax; Tpas += Fp; Fm += Fmax;
        const l0 = mb.L0 || 1; aw += a * l0; w += l0;
        if (a > amax) { amax = a; amaxU = mb.unit; }
      }
      lm.Tact = Tact; lm.Tpas = Tpas; lm.Fmax = Fm; lm.engage = Fm > 0 ? (Tact + Tpas) / Fm : 0; lm.actMean = w ? aw / w : 0; lm.actMax = amax; lm.actMaxUnit = amaxU;
    }
    return m;
  }
  // adapters: a static pose of physics.json (left side, the poses are symmetric) and a frame of cycle_phys.json
  function physFromStatic(ph) {
    const a = {}, Fmax = {}, Fp = {};
    for (const [u, mm] of Object.entries(ph.muscles)) { a[u] = { L: mm.a, R: mm.a }; Fmax[u] = mm.Fmax; Fp[u] = { L: mm.Fp, R: mm.Fp }; }
    return { a, Fmax, Fp };
  }
  function physFromFrame(fr, units, FmaxByUnit) {
    const a = {};
    units.forEach((u, k) => { a[u] = { L: fr.aL ? fr.aL[k] : 0, R: fr.aR ? fr.aR[k] : 0 }; });
    let Fp = null;
    if (fr.pL && fr.pR) { Fp = {}; units.forEach((u, k) => { Fp[u] = { L: fr.pL[k], R: fr.pR[k] }; }); }
    return { a, Fmax: FmaxByUnit, Fp };
  }
  const linePath = (W, id, side) => [].concat(...measureLineRaw(resolveLine(LINE[id], side), W).paths);
  const lineGroupsPaths = (W, id, side) => measureLineRaw(resolveLine(LINE[id], side), W).paths;
  const stationsOf = (W, id, side) => resolveLine(LINE[id], side).stations.map(s => ({ ru: s.ru, P: A.pt(W, s.seg, s.p) }));
  // which lines a muscle / ligament / fascia element belongs to (for cards: «входит в линии …»)
  function linesOf(kind, unit) {
    const out = [];
    for (const l of LINES) l.groups.forEach((gr, k) => { for (const m of gr.members) { const u = m.mus || m.lig || m.fas; const kd = m.mus ? 'muscle' : m.lig ? 'lig' : 'fascia'; if (kd === kind && u === unit && !out.some(o => o.line === l.id && o.group === k)) out.push({ line: l.id, ru: l.ru, group: k, groupRu: gr.ru, cross: gr.side === 'B' }); } });
    return out;
  }
  // text for a line's card: name, function, evidence, tracks with their stations (from Myers' research file)
  function lineInfo(id) {
    const l = LINE[id]; if (!l) return null;
    return { id, ru: l.ru, en: l.en, abbr: l.abbr, color: l.color, fn: l.fn || l.role || null, evidence: l.evidence || null, crossing: !!l.crossing, sideNote: l.sideNote || null,
      groups: l.groups.map((gr, k) => ({ n: gr.n ?? k + 1, ru: gr.ru, lat: gr.lat || null, side: gr.side || 'A', cov: gr.cov || null, note: gr.note || null, from: gr.from ? gr.from.ru : null, to: gr.to ? gr.to.ru : null, members: gr.members.map(m => m.mus || m.lig || m.fas) })) };
  }
  for (const l of LINES) for (const side of ['L', 'R']) resolveLine(l, side);
  return { lineInfo, ELEMENTS, EL, LINES, LINE, ALIAS, instances, measure, withPhysics, physFromStatic, physFromFrame, linePath, lineGroupsPaths, stationsOf, linesOf, resolveLine, L0, W_STAND };
}
