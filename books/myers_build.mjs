// Anatomy Trains (Т. Майерс, «Анатомические поезда») mapped onto the model: 12 myofascial lines, track by track,
// with stations as rig points (segment + point in standing-pose coordinates, as everywhere in anatomy3.js),
// model coverage, new fascia elements the model lacks, and what the model itself says about each line in the
// five poses (length change of every track against standing, activation of the track's muscles from physics.json)
// and — when the segment files exist — over the transitions of the round.
// usage: node books/myers_build.mjs   → books/myers_lines.json
// Texts paraphrase the book's map of lines (no quotations); track lists follow Myers' line maps, checked against
// anatomytrains.com materials and the dissection review Wilke et al. 2016 (Arch Phys Med Rehabil 97:454–461).
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import * as THREE from 'three';
const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const { createAnatomy } = await import(path.join(ROOT, 'anatomy3.js'));
const A = createAnatomy(THREE);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const INST = A.strandInstances();
const r3 = v => +(+v).toFixed(3);

// ---------- references to the model ----------
const instOf = (kind, unit, side, idx) => INST.find(i => i.kind === kind && i.unit === unit && i.side === side && i.idx === (idx ?? 0)) || INST.find(i => i.kind === kind && i.unit === unit && i.side === side);
const firstIdx = (kind, unit) => INST.filter(i => i.kind === kind && i.unit === unit && i.side === 'L').map(i => i.idx);
const P = q => ({ seg: q.seg, p: [r3(q.p.x), r3(q.p.y), r3(q.p.z)] });
const end = (kind, unit, k, which) => { const ks = firstIdx(kind, unit); const i = instOf(kind, unit, 'L', k ?? ks[0]); if (!i) throw new Error('no ' + kind + ' ' + unit); return P(which === 'o' ? i.pts[0] : i.pts[i.pts.length - 1]); };
const o = (u, k) => end('muscle', u, k, 'o'), i = (u, k) => end('muscle', u, k, 'i');
const lo = (u, k) => end('lig', u, k, 'o'), li = (u, k) => end('lig', u, k, 'i');
const p = (seg, xyz) => ({ seg, p: xyz });
const lm = (region, tag) => { const l = A.LANDMARKS.find(x => x.region === region && (tag ? x.tag === tag : true) && x.side !== 'R'); return P({ seg: l.seg, p: l.pc }); };

// ---------- fascia elements the model does not have (rest coordinates, left side unless central) ----------
// type band: a polyline whose length the model can follow; sheet: an outline (first ring) for drawing, with a
// representative band (`band`) for length; static: every point on one rigid segment of the rig — drawn, never strained
const FASCIA = {
  thoracolumbarFascia: { ru: 'Грудопоясничная фасция (задний листок), крестцовая фасция', lat: 'fascia thoracolumbalis, lamina posterior; fascia sacralis', type: 'sheet', side: 'C',
    outline: [p('pelvis', [0, 0.93, -0.118]), p('pelvis', [0.05, 0.97, -0.118]), p('pelvis', [0.1, 1.06, -0.085]), p('lumB', [0.11, 1.15, -0.095]), p('thA', [0.09, 1.21, -0.128]), p('thA', [0, 1.2, -0.152]), p('thA', [-0.09, 1.21, -0.128]), p('lumB', [-0.11, 1.15, -0.095]), p('pelvis', [-0.1, 1.06, -0.085]), p('pelvis', [-0.05, 0.97, -0.118])],
    band: [p('pelvis', [0.015, 0.97, -0.12]), p('lumA', [0.015, 1.05, -0.122]), p('lumB', [0.015, 1.12, -0.12]), p('thA', [0.015, 1.2, -0.15])],
    links: 'широчайшая (обе стороны), большая ягодичная (обе стороны), выпрямитель и многораздельная под ней, внутренняя косая и поперечная через латеральный шов, задняя нижняя зубчатая', lines: ['SBL', 'SPL', 'BFL'] },
  tlfCross: { ru: 'Перекрёст грудопоясничной фасции: широчайшая одной стороны → большая ягодичная другой', lat: 'fascia thoracolumbalis (decussatio)', type: 'band', side: 'X',
    band: [p('lumA', [0.026, 1.057, -0.102]), p('pelvis', [0, 0.975, -0.121]), p('pelvis', [-0.042, 1.01, -0.112])], lines: ['BFL'] },
  lineaAlba: { ru: 'Белая линия живота и апоневроз косых мышц', lat: 'linea alba; aponeuroses mm. obliquorum', type: 'band', side: 'C',
    band: [p('pelvis', [0, 0.887, 0.095]), p('pelvis', [0, 0.95, 0.094]), p('pelvis', [0, 1.0, 0.1]), p('lumA', [0, 1.05, 0.103]), p('lumB', [0, 1.1, 0.104]), p('lumB', [0, 1.15, 0.104]), p('thA', [0, 1.2, 0.118])], lines: ['SPL', 'SFL'] },
  rectusSheath: { ru: 'Влагалище прямой мышцы живота, полулунная линия', lat: 'vagina m. recti abdominis; linea semilunaris', type: 'sheet', side: 'L',
    outline: [p('pelvis', [0.012, 0.888, 0.093]), p('pelvis', [0.05, 0.9, 0.087]), p('pelvis', [0.077, 0.95, 0.084]), p('pelvis', [0.08, 1.0, 0.088]), p('lumA', [0.082, 1.05, 0.091]), p('lumB', [0.085, 1.1, 0.092]), p('lumB', [0.085, 1.15, 0.092]), p('thA', [0.085, 1.2, 0.105]), p('thB', [0.08, 1.3, 0.1]), p('thB', [0.01, 1.3, 0.112]), p('thA', [0.008, 1.2, 0.118]), p('lumB', [0.008, 1.12, 0.104]), p('pelvis', [0.008, 0.95, 0.095])],
    band: [p('pelvis', [0.05, 0.9, 0.087]), p('pelvis', [0.08, 1.0, 0.088]), p('lumB', [0.085, 1.1, 0.092]), p('thA', [0.085, 1.2, 0.105]), p('thB', [0.08, 1.3, 0.1])], lines: ['SFL', 'FFL'] },
  sternalFascia: { ru: 'Грудинная (грудино-хрящевая) фасция', lat: 'fascia sternalis; membrana sterni', type: 'band', side: 'C',
    band: [p('thB', [0, 1.3, 0.112]), p('thB', [0, 1.35, 0.104]), p('thC', [0, 1.4, 0.079])], lines: ['SFL'] },
  galea: { ru: 'Сухожильный шлем (надчерепной апоневроз) и височная фасция', lat: 'galea aponeurotica; aponeurosis epicranialis; fascia temporalis', type: 'sheet', side: 'C', static: true,
    outline: [p('head', [0, 1.6, -0.092]), p('head', [0.05, 1.65, -0.085]), p('head', [0.075, 1.7, -0.02]), p('head', [0.065, 1.71, 0.06]), p('head', [0, 1.702, 0.097]), p('head', [-0.065, 1.71, 0.06]), p('head', [-0.075, 1.7, -0.02]), p('head', [-0.05, 1.65, -0.085])],
    band: [p('head', [0, 1.6, -0.092]), p('head', [0, 1.69, -0.097]), p('head', [0, 1.757, 0]), p('head', [0, 1.702, 0.097])], lines: ['SBL', 'SFL', 'LL'] },
  intercostals: { ru: 'Межрёберные мышцы и фасция боковой стенки груди', lat: 'mm. intercostales externi et interni', type: 'band', side: 'L',
    band: [p('thA', [0.137, 1.21, 0.035]), p('thB', [0.142, 1.29, 0.04]), p('thB', [0.137, 1.33, 0.036]), p('thC', [0.125, 1.38, 0.025])], lines: ['LL'] },
  cruralFasciaAnt: { ru: 'Фасция голени, передний отдел, удерживатели разгибателей', lat: 'fascia cruris (compartimentum anterius); retinacula mm. extensorum', type: 'band', side: 'L',
    band: [p('shank_L', [0.11, 0.43, 0.042]), p('shank_L', [0.105, 0.3, 0.038]), p('shank_L', [0.1, 0.12, 0.035]), p('foot_L', [0.095, 0.075, 0.05]), p('foot_L', [0.088, 0.05, 0.1])], lines: ['SFL'] },
  cruralFasciaLat: { ru: 'Фасция голени, латеральный отдел, удерживатели малоберцовых', lat: 'fascia cruris (compartimentum laterale); retinacula mm. fibularium', type: 'band', side: 'L',
    band: [p('shank_L', [0.135, 0.45, -0.012]), p('shank_L', [0.136, 0.3, -0.012]), p('shank_L', [0.13, 0.12, -0.016]), p('shank_L', [0.127, 0.075, -0.022]), p('foot_L', [0.122, 0.03, 0.04])], lines: ['LL'] },
  fasciaLata: { ru: 'Широкая фасция бедра (вместе с подвздошно-большеберцовым трактом)', lat: 'fascia lata; tractus iliotibialis', type: 'sheet', side: 'L',
    outline: [p('pelvis', [0.125, 1.03, 0.06]), p('thigh_L', [0.155, 0.9, 0.02]), p('thigh_L', [0.15, 0.7, 0.01]), p('shank_L', [0.118, 0.468, 0.018]), p('shank_L', [0.098, 0.5, 0.05]), p('thigh_L', [0.11, 0.7, 0.085]), p('thigh_L', [0.11, 0.88, 0.082])],
    band: [p('pelvis', [0.125, 1.03, 0.06]), p('thigh_L', [0.155, 0.9, 0.02]), p('thigh_L', [0.15, 0.7, 0.01]), p('shank_L', [0.118, 0.468, 0.018])], lines: ['LL', 'SPL'] },
  lateralSeptumArm: { ru: 'Латеральная межмышечная перегородка плеча', lat: 'septum intermusculare brachii laterale', type: 'band', side: 'L', static: true,
    band: [p('hum_L', [0.213, 1.268, -0.026]), p('hum_L', [0.226, 1.17, -0.03]), p('hum_L', [0.232, 1.08, -0.026])], lines: ['SBAL'] },
  medialSeptumArm: { ru: 'Медиальная межмышечная перегородка плеча', lat: 'septum intermusculare brachii mediale', type: 'band', side: 'L', static: true,
    band: [p('hum_L', [0.195, 1.33, -0.008]), p('hum_L', [0.185, 1.2, -0.02]), p('hum_L', [0.18, 1.078, -0.032])], lines: ['SFAL'] },
  palmarAponeurosis: { ru: 'Ладонный апоневроз и удерживатель сгибателей (запястный канал)', lat: 'aponeurosis palmaris; retinaculum mm. flexorum', type: 'sheet', side: 'L',
    outline: [p('hand_L', [0.199, 0.78, -0.026]), p('f2a_L', [0.206, 0.705, -0.003]), p('f3a_L', [0.205, 0.698, -0.022]), p('f4a_L', [0.203, 0.703, -0.041]), p('f5a_L', [0.2, 0.715, -0.057])],
    band: [p('hand_L', [0.199, 0.78, -0.026]), p('hand_L', [0.197, 0.745, -0.024]), p('f3a_L', [0.205, 0.698, -0.022])], lines: ['SFAL'] },
  clavipectoral: { ru: 'Ключично-грудная фасция', lat: 'fascia clavipectoralis', type: 'sheet', side: 'L',
    outline: [p('clav_L', [0.06, 1.392, 0.06]), p('clav_L', [0.118, 1.398, 0.022]), p('scap_L', [0.155, 1.405, -0.002]), p('thB', [0.07, 1.33, 0.09])],
    band: [p('clav_L', [0.09, 1.395, 0.04]), p('scap_L', [0.155, 1.405, -0.002]), p('thB', [0.07, 1.33, 0.09])], lines: ['DFAL'] },
  pelvicFloor: { ru: 'Тазовое дно (мышца, поднимающая задний проход, и её фасция)', lat: 'diaphragma pelvis; m. levator ani; fascia diaphragmatis pelvis', type: 'sheet', side: 'C', static: true,
    outline: [p('pelvis', [0, 0.885, -0.06]), p('pelvis', [0.045, 0.875, -0.01]), p('pelvis', [0.03, 0.88, 0.05]), p('pelvis', [0, 0.885, 0.07]), p('pelvis', [-0.03, 0.88, 0.05]), p('pelvis', [-0.045, 0.875, -0.01])],
    band: [p('pelvis', [0, 0.885, -0.06]), p('pelvis', [0, 0.878, 0.0]), p('pelvis', [0, 0.885, 0.07])], lines: ['DFL'] },
  diaphragm: { ru: 'Диафрагма (ножки, сухожильный центр)', lat: 'diaphragma; crura; centrum tendineum', type: 'sheet', side: 'C', planned: false,
    outline: [p('lumA', [0, 1.06, -0.03]), p('thA', [0.09, 1.22, 0.02]), p('thB', [0, 1.27, 0.07]), p('thA', [-0.09, 1.22, 0.02])],
    band: [p('lumA', [0, 1.06, -0.03]), p('thA', [0, 1.24, 0.0]), p('thB', [0, 1.27, 0.07])], lines: ['DFL'], note: 'дыхание в модели не считается — только как форма' },
  ulnarPeriosteum: { ru: 'Фасция вдоль надкостницы локтевой кости', lat: 'periosteum ulnae', type: 'band', side: 'L', static: true,
    band: [p('ulna_L', [0.203, 1.093, -0.062]), p('ulna_L', [0.208, 0.95, -0.065]), p('ulna_L', [0.21, 0.865, -0.058])], lines: ['DBAL'] },
  radialPeriosteum: { ru: 'Надкостница лучевой кости (передний край)', lat: 'periosteum radii', type: 'band', side: 'L', static: true,
    band: [p('fore_L', [0.212, 1.037, -0.014]), p('fore_L', [0.216, 0.95, -0.006]), p('fore_L', [0.216, 0.84, 0.0])], lines: ['DFAL'] },
  fibularHeadLig: { ru: 'Передняя связка головки малоберцовой кости', lat: 'lig. capitis fibulae anterius', type: 'band', side: 'L', static: true,
    band: [p('shank_L', [0.13, 0.455, -0.02]), p('shank_L', [0.124, 0.47, -0.004])], lines: ['LL'] },
};
// structures the lines name that the model would need as new muscles, not fascia (listed, not built)
const MISSING_MUSCLES = {
  suboccipitals: { ru: 'Подзатылочные мышцы', lat: 'mm. recti capitis posteriores, obliqui capitis', lines: ['SBL'] },
  longusCapitis: { ru: 'Длинная мышца головы', lat: 'm. longus capitis', lines: ['DFL'] },
  hyoids: { ru: 'Подъязычные и надподъязычные мышцы', lat: 'mm. infrahyoidei et suprahyoidei', lines: ['DFL'] },
  transversusThoracis: { ru: 'Поперечная мышца груди, внутригрудная фасция', lat: 'm. transversus thoracis; fascia endothoracica', lines: ['DFL'] },
  edb: { ru: 'Короткий разгибатель пальцев стопы', lat: 'm. extensor digitorum brevis', lines: ['SFL'] },
  sternalis: { ru: 'Грудинная мышца (непостоянная)', lat: 'm. sternalis', lines: ['SFL'] },
  fingerExtTendons: { ru: 'Сухожилия разгибателей на тыле пальцев (разгибательный аппарат)', lat: 'aponeurosis dorsalis digitorum', lines: ['SBAL'] },
};

// ---------- the lines ----------
// side of a track: A — the line's own side, B — the opposite side (crossing lines), C — central
const T = (o) => o;
const LINES = [
  { id: 'SBL', ru: 'Поверхностная задняя линия', abbr: 'ПЗЛ', en: 'Superficial Back Line', crossing: false,
    fn: 'держит тело выпрямленным против сгибания вперёд, разгибает (кроме колена — его сгибает); в стойке — постоянная «осанка» сзади; растягивается в наклонах вперёд, укорачивается в прогибах',
    evidence: 'сильная: все три перехода (подошвенная фасция → икроножная → задняя поверхность бедра → поясничная фасция/выпрямитель) подтверждены на вскрытиях (Wilke 2016)',
    tracks: [
      T({ ru: 'Подошвенная фасция и короткий сгибатель пальцев', lat: 'aponeurosis plantaris; m. flexor digitorum brevis', units: ['fdb'], ligs: ['plantar'], side: 'A', cov: 'full', path: [{ lig: 'plantar', k: 1 }],
        from: { ru: 'подошвенная поверхность фаланг пальцев стопы', at: i('fdb', 1) }, to: { ru: 'пяточный бугор снизу', at: lo('plantar', 0) } }),
      T({ ru: 'Икроножная мышца и ахиллово сухожилие (камбаловидная — глубже, в общем сухожилии)', lat: 'm. gastrocnemius; tendo calcaneus', units: ['gastroM', 'gastroL'], related: ['soleus', 'plantaris'], side: 'A', cov: 'full', path: [{ unit: 'gastroM' }],
        from: { ru: 'пяточный бугор сзади', at: i('gastroM') }, to: { ru: 'мыщелки бедра сзади', at: o('gastroM') } }),
      T({ ru: 'Мышцы задней поверхности бедра', lat: 'm. biceps femoris (caput longum), m. semitendinosus, m. semimembranosus', units: ['bicFemL', 'semiT', 'semiM'], related: ['bicFemS'], side: 'A', cov: 'full', path: [{ unit: 'bicFemL' }],
        from: { ru: 'мыщелки большеберцовой, головка малоберцовой (головки икроножной и сухожилия задней группы перекрываются у колена)', at: i('bicFemL') }, to: { ru: 'седалищный бугор', at: o('bicFemL') } }),
      T({ ru: 'Крестцово-бугорная связка', lat: 'lig. sacrotuberale', ligs: ['sacrotub'], side: 'A', cov: 'static', path: [{ lig: 'sacrotub' }],
        from: { ru: 'седалищный бугор', at: li('sacrotub') }, to: { ru: 'крестец', at: lo('sacrotub') }, note: 'в модели таз и крестец — одно жёсткое тело, длина связки не меняется' }),
      T({ ru: 'Пояснично-крестцовая фасция и мышца, выпрямляющая позвоночник', lat: 'fascia thoracolumbalis; m. erector spinae', units: ['erector'], related: ['multifidus'], fascia: ['thoracolumbarFascia'], side: 'A', cov: 'partial', path: [{ unit: 'erector', k: 0 }],
        from: { ru: 'крестец', at: o('erector', 0) }, to: { ru: 'нижнешейный отдел (в модели выпрямитель заканчивается на C7–T1)', at: i('erector', 0) }, note: 'нет фасции; выпрямитель доходит до шеи, а не до затылка' }),
      T({ ru: 'Шейный отрезок: полуостистая и ременная мышцы головы, выйная связка', lat: 'm. semispinalis capitis; m. splenius capitis; lig. nuchae', units: ['semispinalis'], related: ['splenius'], ligs: ['nuchal'], side: 'A', cov: 'partial', path: [{ unit: 'semispinalis' }],
        from: { ru: 'остистые и поперечные отростки верхнегрудных позвонков', at: o('semispinalis') }, to: { ru: 'затылочный гребень', at: i('semispinalis') }, note: 'нет подзатылочных мышц («глубокий экспресс» шеи)' }),
      T({ ru: 'Сухожильный шлем (надчерепной апоневроз)', lat: 'galea aponeurotica', fascia: ['galea'], side: 'C', cov: 'missing', path: [{ fascia: 'galea' }],
        from: { ru: 'затылочный гребень', at: p('head', [0, 1.6, -0.092]) }, to: { ru: 'лобная кость, надбровная дуга', at: lm('голова', null) }, note: 'голова — одно тело, фасция статична' }),
    ],
    poses: {
      dog: { myers: 'lengthened', ru: 'главная растягиваемая линия: подошва, икры, задняя поверхность бедра; с прогибом в спине верхний отрезок (выпрямитель) укорочен и работает' },
      plank: { myers: 'neutral', ru: 'почти нейтральна по длине; держит не она, а передние линии' },
      half: { myers: 'shortened-active', ru: 'укорачивается и работает: выпрямитель, полуостистая поднимают грудь, ягодичные и задняя группа бедра держат таз и ноги' },
      bow: { myers: 'shortened-active', ru: 'сильно укорочена и работает по всей длине: спина, ягодичные, задняя группа бедра; колено согнуто — нижний отрезок свободен' },
      fold: { myers: 'lengthened', ru: 'растягивается целиком, от подошвы до затылка — классический наклон для ПЗЛ' },
    } },
  { id: 'SFL', ru: 'Поверхностная фронтальная линия', abbr: 'ПФЛ', en: 'Superficial Front Line', crossing: false,
    fn: 'антагонист ПЗЛ по сагиттали: сгибает туловище и бедро, разгибает колено, тянет стопу вверх; защищает переднюю поверхность; растягивается в прогибах',
    evidence: 'не подтверждена: переходы прямая бедра → прямая живота не найдены на вскрытиях (Wilke 2016) — линия функциональная, не сплошная',
    tracks: [
      T({ ru: 'Длинный и короткий разгибатели пальцев, передняя большеберцовая, передний отдел фасции голени', lat: 'm. extensor digitorum longus et brevis, m. extensor hallucis longus, m. tibialis anterior; fascia cruris', units: ['edl', 'ehl', 'tibAnt'], fascia: ['cruralFasciaAnt'], side: 'A', cov: 'partial', path: [{ unit: 'tibAnt' }],
        from: { ru: 'тыльная поверхность фаланг пальцев', at: i('edl', 1) }, to: { ru: 'бугристость большеберцовой кости', at: li('patTend') }, note: 'нет короткого разгибателя и фасции голени' }),
      T({ ru: 'Связка надколенника', lat: 'lig. patellae', ligs: ['patTend'], side: 'A', cov: 'static', path: [{ lig: 'patTend' }],
        from: { ru: 'бугристость большеберцовой', at: li('patTend') }, to: { ru: 'надколенник', at: lo('patTend') }, note: 'надколенник в модели жёстко связан с голенью' }),
      T({ ru: 'Прямая мышца бедра и вся четырёхглавая', lat: 'm. rectus femoris; m. quadriceps femoris', units: ['rectFem'], related: ['vastLat', 'vastMed', 'vastInt'], side: 'A', cov: 'full', path: [{ unit: 'rectFem' }],
        from: { ru: 'надколенник', at: i('rectFem') }, to: { ru: 'передняя нижняя подвздошная ость', at: o('rectFem') } }),
      T({ ru: 'Прямая мышца живота и её влагалище', lat: 'm. rectus abdominis; vagina m. recti abdominis', units: ['rectAbd'], fascia: ['rectusSheath'], side: 'A', cov: 'partial', path: [{ unit: 'rectAbd', k: 2 }],
        from: { ru: 'лобковый бугорок (переход с ПНПО — «соединение» линии)', at: o('rectAbd', 2) }, to: { ru: '5-е ребро', at: i('rectAbd', 2) }, note: 'нет влагалища прямой мышцы' }),
      T({ ru: 'Грудинная мышца и грудино-хрящевая фасция', lat: 'm. sternalis; fascia sternalis', fascia: ['sternalFascia'], side: 'C', cov: 'missing', path: [{ fascia: 'sternalFascia' }],
        from: { ru: '5-е ребро', at: p('thB', [0, 1.3, 0.112]) }, to: { ru: 'рукоятка грудины', at: lm('грудь', 'chestTop') } }),
      T({ ru: 'Грудино-ключично-сосцевидная мышца', lat: 'm. sternocleidomastoideus', units: ['scm'], side: 'A', cov: 'full', path: [{ unit: 'scm', k: 0 }],
        from: { ru: 'рукоятка грудины', at: i('scm', 0) }, to: { ru: 'сосцевидный отросток', at: o('scm', 0) } }),
      T({ ru: 'Фасция волосистой части головы', lat: 'fascia epicranialis', fascia: ['galea'], side: 'A', cov: 'missing', path: [],
        from: { ru: 'сосцевидный отросток', at: o('scm', 0) }, to: { ru: 'височная область', at: p('head', [0.075, 1.7, -0.02]) }, note: 'статична' }),
    ],
    poses: {
      dog: { myers: 'mixed', ru: 'нижний отрезок работает (четырёхглавая выпрямляет колени, передняя большеберцовая — тыльное сгибание); живот и грудь слегка удлинены прогибом спины' },
      plank: { myers: 'loaded', ru: 'главная несущая линия планки: держит живот и таз от провисания (прямая живота, четырёхглавая, передняя большеберцовая), ГКС держит голову' },
      half: { myers: 'lengthened', ru: 'удлиняется: передняя поверхность бёдер, живота, груди, шеи' },
      bow: { myers: 'lengthened', ru: 'растягивается сильнее всего в круге: от голени через бедро и живот до горла; четырёхглавая при этом работает, выжимая голени в руки' },
      fold: { myers: 'shortened', ru: 'укорочена; четырёхглавая работает, чтобы держать колени прямыми (и по Лонгу — расслабляет заднюю поверхность бедра)' },
    } },
  { id: 'LL', ru: 'Латеральная линия', abbr: 'ЛЛ', en: 'Lateral Line', crossing: false,
    fn: 'держит баланс справа–слева, тормозит наклоны и скручивания вбок, стабилизирует туловище над ногами при ходьбе и на одной ноге',
    evidence: 'умеренная: подтверждены 2 из 5 переходов (Wilke 2016)',
    tracks: [
      T({ ru: 'Длинная и короткая малоберцовые, латеральный отдел фасции голени', lat: 'mm. fibulares longus et brevis; fascia cruris', units: ['fibL', 'fibB'], fascia: ['cruralFasciaLat'], side: 'A', cov: 'partial', path: [{ unit: 'fibL' }],
        from: { ru: 'основания 1-й и 5-й плюсневых', at: i('fibL') }, to: { ru: 'головка малоберцовой кости', at: o('fibL') }, note: 'нет фасции голени' }),
      T({ ru: 'Передняя связка головки малоберцовой кости', lat: 'lig. capitis fibulae anterius', fascia: ['fibularHeadLig'], side: 'A', cov: 'missing', path: [],
        from: { ru: 'головка малоберцовой', at: o('fibL') }, to: { ru: 'латеральный мыщелок большеберцовой', at: li('itb') }, note: 'обе кости голени — одно тело модели' }),
      T({ ru: 'Подвздошно-большеберцовый тракт, напрягатель широкой фасции, большая (верхние волокна) и средняя ягодичные', lat: 'tractus iliotibialis; m. tensor fasciae latae; m. gluteus maximus (pars superior); m. gluteus medius', units: ['tfl', 'gluteMed'], related: ['gluteMax'], ligs: ['itb'], fascia: ['fasciaLata'], side: 'A', cov: 'full', path: [{ lig: 'itb' }, { unit: 'tfl' }],
        from: { ru: 'латеральный мыщелок большеберцовой', at: li('itb') }, to: { ru: 'гребень подвздошной кости, ПВПО, ЗВПО', at: o('tfl') } }),
      T({ ru: 'Наружная и внутренняя косые мышцы живота («латеральный X»)', lat: 'mm. obliqui externus et internus abdominis', units: ['intObl', 'extObl'], side: 'A', cov: 'full', path: [{ unit: 'intObl', k: 0 }],
        from: { ru: 'гребень подвздошной кости', at: o('intObl', 0) }, to: { ru: 'нижние рёбра', at: i('intObl', 0) } }),
      T({ ru: 'Наружные и внутренние межрёберные мышцы', lat: 'mm. intercostales', fascia: ['intercostals'], side: 'A', cov: 'missing', path: [{ fascia: 'intercostals' }],
        from: { ru: 'нижние рёбра', at: p('thA', [0.137, 1.21, 0.035]) }, to: { ru: '1-е и 2-е рёбра', at: p('thC', [0.125, 1.38, 0.025]) } }),
      T({ ru: 'Ременная мышца головы и грудино-ключично-сосцевидная (шейный «X»)', lat: 'm. splenius capitis; m. sternocleidomastoideus', units: ['splenius', 'scm'], side: 'A', cov: 'full', path: [{ unit: 'scm', k: 1 }],
        from: { ru: '1-е и 2-е рёбра, ключица', at: i('scm', 1) }, to: { ru: 'затылочный гребень, сосцевидный отросток', at: o('scm', 1) } }),
    ],
    poses: {
      dog: { myers: 'stabilizing', ru: 'обе стороны в равновесии; удерживает таз и колени от заваливания, бока груди немного удлинены руками над головой' },
      plank: { myers: 'stabilizing', ru: 'обе стороны работают одинаково, не давая туловищу провернуться или наклониться' },
      half: { myers: 'neutral', ru: 'сагиттальная поза, линия почти нейтральна; межрёберные удлиняются с прогибом и вдохом' },
      bow: { myers: 'stabilizing', ru: 'отводящие бедра и тракт держат колени, чтобы не разъезжались; бока груди удлинены' },
      fold: { myers: 'neutral', ru: 'почти нейтральна; малоберцовые и тракт держат стопу и колено ровно' },
    } },
  { id: 'SPL', ru: 'Спиральная линия', abbr: 'СЛ', en: 'Spiral Line', crossing: true,
    fn: 'двойная спираль вокруг тела: создаёт и тормозит скручивание, держит баланс между сторонами, «стремя» стопы (передняя большеберцовая + длинная малоберцовая) держит свод и колено',
    evidence: 'умеренная: подтверждены 5 из 9 переходов (Wilke 2016)',
    sideNote: 'название по стороне головы: СЛ левая начинается левой ременной мышцей, идёт через правые ромбовидные, переднюю зубчатую и наружную косую, пересекает живот к левой внутренней косой и спускается по левой ноге обратно к затылку слева',
    tracks: [
      T({ ru: 'Ременные мышцы головы и шеи', lat: 'mm. splenius capitis et cervicis', units: ['splenius'], side: 'A', cov: 'full', path: [{ unit: 'splenius', k: 1 }],
        from: { ru: 'затылочный гребень, сосцевидный отросток, поперечные отростки C1–C3', at: i('splenius', 1) }, to: { ru: 'остистые отростки C7–T5 (переход на другую сторону)', at: o('splenius', 1) } }),
      T({ ru: 'Большая и малая ромбовидные (другая сторона)', lat: 'mm. rhomboidei', units: ['rhomb'], side: 'B', cov: 'full', path: [{ unit: 'rhomb', k: 2 }],
        from: { ru: 'остистые отростки C7–T5', at: o('rhomb', 2) }, to: { ru: 'медиальный край лопатки', at: i('rhomb', 2) } }),
      T({ ru: 'Передняя зубчатая', lat: 'm. serratus anterior', units: ['serratus'], side: 'B', cov: 'full', path: [{ unit: 'serratus', k: 4 }],
        from: { ru: 'медиальный край лопатки', at: i('serratus', 4) }, to: { ru: 'боковые поверхности рёбер', at: o('serratus', 4) } }),
      T({ ru: 'Наружная косая мышца живота', lat: 'm. obliquus externus abdominis', units: ['extObl'], side: 'B', cov: 'full', path: [{ unit: 'extObl', k: 1 }],
        from: { ru: 'рёбра', at: o('extObl', 1) }, to: { ru: 'апоневроз живота', at: i('extObl', 1) } }),
      T({ ru: 'Апоневроз живота и белая линия (перекрёст)', lat: 'aponeurosis abdominis; linea alba', fascia: ['lineaAlba'], side: 'C', cov: 'missing', path: [{ fascia: 'lineaAlba' }],
        from: { ru: 'апоневроз другой стороны', at: p('pelvis', [0, 1.0, 0.1]) }, to: { ru: 'апоневроз своей стороны', at: p('lumA', [0, 1.05, 0.103]) } }),
      T({ ru: 'Внутренняя косая мышца живота', lat: 'm. obliquus internus abdominis', units: ['intObl'], side: 'A', cov: 'full', path: [{ unit: 'intObl', k: 3 }],
        from: { ru: 'белая линия', at: i('intObl', 3) }, to: { ru: 'гребень подвздошной кости, ПВПО', at: o('intObl', 3) } }),
      T({ ru: 'Напрягатель широкой фасции и подвздошно-большеберцовый тракт', lat: 'm. tensor fasciae latae; tractus iliotibialis', units: ['tfl'], ligs: ['itb'], side: 'A', cov: 'full', path: [{ unit: 'tfl' }, { lig: 'itb' }],
        from: { ru: 'ПВПО', at: o('tfl') }, to: { ru: 'латеральный мыщелок большеберцовой', at: li('itb') } }),
      T({ ru: 'Передняя большеберцовая', lat: 'm. tibialis anterior', units: ['tibAnt'], side: 'A', cov: 'full', path: [{ unit: 'tibAnt' }],
        from: { ru: 'латеральный мыщелок большеберцовой', at: o('tibAnt') }, to: { ru: 'основание 1-й плюсневой, медиальная клиновидная', at: i('tibAnt') } }),
      T({ ru: 'Длинная малоберцовая («стремя» под сводом стопы)', lat: 'm. fibularis longus', units: ['fibL'], side: 'A', cov: 'full', path: [{ unit: 'fibL' }],
        from: { ru: 'основание 1-й плюсневой', at: i('fibL') }, to: { ru: 'головка малоберцовой', at: o('fibL') } }),
      T({ ru: 'Двуглавая мышца бедра, длинная головка', lat: 'm. biceps femoris, caput longum', units: ['bicFemL'], side: 'A', cov: 'full', path: [{ unit: 'bicFemL' }],
        from: { ru: 'головка малоберцовой', at: i('bicFemL') }, to: { ru: 'седалищный бугор', at: o('bicFemL') } }),
      T({ ru: 'Крестцово-бугорная связка', lat: 'lig. sacrotuberale', ligs: ['sacrotub'], side: 'A', cov: 'static', path: [{ lig: 'sacrotub' }],
        from: { ru: 'седалищный бугор', at: li('sacrotub') }, to: { ru: 'крестец', at: lo('sacrotub') } }),
      T({ ru: 'Крестцово-поясничная фасция, мышца, выпрямляющая позвоночник', lat: 'fascia thoracolumbalis; m. erector spinae', units: ['erector'], fascia: ['thoracolumbarFascia'], side: 'A', cov: 'partial', path: [{ unit: 'erector', k: 1 }],
        from: { ru: 'крестец', at: o('erector', 1) }, to: { ru: 'затылочный гребень (в модели — до верхнегрудного отдела)', at: i('erector', 1) } }),
    ],
    poses: {
      dog: { myers: 'balanced', ru: 'обе спирали уравновешены; задний отрезок (двуглавая бедра) растянут вместе с ПЗЛ, передняя зубчатая держит лопатки' },
      plank: { myers: 'loaded', ru: 'передний отрезок (передняя зубчатая → косые) работает изометрически, держит рёбра и таз; «стремя» не нагружено' },
      half: { myers: 'mixed', ru: 'ромбовидные укорачиваются (сводим лопатки), передняя зубчатая и косые удлиняются прогибом' },
      bow: { myers: 'mixed', ru: 'ромбовидные укорочены, косые и передняя зубчатая удлинены; задний отрезок ноги свободен (колено согнуто)' },
      fold: { myers: 'balanced', ru: 'задний отрезок ноги удлинён, «стремя» стопы держит свод и колено' },
    } },
  { id: 'DFL', ru: 'Глубокая фронтальная линия', abbr: 'ГФЛ', en: 'Deep Front Line', crossing: false,
    fn: '«кор» в трёхмерном смысле: от свода стопы через внутреннюю поверхность ноги, тазовое дно, поясничную мышцу, диафрагму и средостение до основания черепа; стабилизирует, держит свод, связывает дыхание с ходьбой',
    evidence: 'отдельно не проверялась (Wilke 2016 — нет данных)',
    tracks: [
      T({ ru: 'Задняя большеберцовая, длинные сгибатели пальцев', lat: 'm. tibialis posterior; mm. flexores digitorum et hallucis longi', units: ['tibPost', 'fdl', 'fhl'], side: 'A', cov: 'full', path: [{ unit: 'tibPost' }],
        from: { ru: 'подошвенные кости предплюсны, подошвенная поверхность пальцев', at: i('tibPost') }, to: { ru: 'задняя поверхность большеберцовой и малоберцовой', at: o('tibPost') } }),
      T({ ru: 'Фасция подколенной мышцы, капсула коленного сустава', lat: 'm. popliteus; capsula articularis genus', units: ['popliteus'], caps: ['kn'], side: 'A', cov: 'full', path: [{ unit: 'popliteus' }],
        from: { ru: 'верхняя задняя часть голени', at: i('popliteus') }, to: { ru: 'медиальный надмыщелок бедра', at: o('popliteus') } }),
      T({ ru: 'Задняя межмышечная перегородка, большая и малая приводящие', lat: 'septum intermusculare posterius; mm. adductores magnus et minimus', units: ['addMag'], side: 'A', cov: 'partial', path: [{ unit: 'addMag', k: 2 }],
        from: { ru: 'медиальный надмыщелок бедра', at: i('addMag', 2) }, to: { ru: 'ветвь седалищной кости', at: o('addMag', 2) }, note: 'перегородки нет' }),
      T({ ru: 'Медиальная межмышечная перегородка, короткая и длинная приводящие (передний вариант)', lat: 'septum intermusculare mediale; mm. adductores brevis et longus', units: ['addLong', 'addBrev'], side: 'A', cov: 'partial', path: [{ unit: 'addLong' }],
        from: { ru: 'шероховатая линия бедра', at: i('addLong') }, to: { ru: 'лобковая ветвь', at: o('addLong') } }),
      T({ ru: 'Тазовое дно, фасция внутренней запирательной мышцы', lat: 'diaphragma pelvis; m. levator ani; fascia obturatoria', units: ['obtInt'], fascia: ['pelvicFloor'], side: 'A', cov: 'partial', path: [{ unit: 'obtInt' }],
        from: { ru: 'седалищная ветвь', at: o('obtInt') }, to: { ru: 'копчик', at: p('pelvis', [0, 0.885, -0.06]) }, note: 'нет мышц тазового дна' }),
      T({ ru: 'Большая поясничная, подвздошная, гребенчатая (бедренный треугольник)', lat: 'm. psoas major; m. iliacus; m. pectineus', units: ['psoas', 'iliacus', 'pectineus'], side: 'A', cov: 'full', path: [{ unit: 'psoas', k: 0 }],
        from: { ru: 'малый вертел', at: i('psoas', 0) }, to: { ru: 'тела и поперечные отростки поясничных позвонков', at: o('psoas', 0) } }),
      T({ ru: 'Квадратная мышца поясницы, подвздошно-поясничная связка (задняя сторона ГФЛ)', lat: 'm. quadratus lumborum; lig. iliolumbale', units: ['ql'], ligs: ['iliolum'], side: 'A', cov: 'full', path: [{ unit: 'ql', k: 0 }],
        from: { ru: 'гребень подвздошной кости', at: o('ql', 0) }, to: { ru: '12-е ребро, поперечные отростки L1–L4', at: i('ql', 0) } }),
      T({ ru: 'Передняя крестцовая фасция и передняя продольная связка', lat: 'fascia presacralis; lig. longitudinale anterius', ligs: ['all'], side: 'C', cov: 'full', path: [{ lig: 'all' }],
        from: { ru: 'копчик, крестец', at: lo('all') }, to: { ru: 'основание затылочной кости', at: li('all') } }),
      T({ ru: 'Длинные мышцы шеи и головы', lat: 'm. longus colli; m. longus capitis', units: ['longusColli'], side: 'A', cov: 'partial', path: [{ unit: 'longusColli' }],
        from: { ru: 'тела верхнегрудных позвонков', at: o('longusColli') }, to: { ru: 'основание затылочной кости', at: i('longusColli') }, note: 'нет длинной мышцы головы' }),
      T({ ru: 'Диафрагма, перикард, средостение, предпозвоночная фасция, лестничные (средний путь)', lat: 'diaphragma; pericardium; fascia prevertebralis; mm. scaleni', units: ['scalene'], fascia: ['diaphragm'], side: 'A', cov: 'partial', path: [{ unit: 'scalene', k: 0 }],
        from: { ru: 'поясничные позвонки (ножки диафрагмы)', at: p('lumA', [0, 1.06, -0.03]) }, to: { ru: 'поперечные отростки шейных позвонков, основание черепа', at: o('scalene', 0) }, note: 'есть только лестничные; диафрагмы и средостения нет' }),
      T({ ru: 'Передняя диафрагма, внутригрудная фасция, поперечная мышца груди, подъязычные (передний путь)', lat: 'fascia endothoracica; m. transversus thoracis; mm. infra- et suprahyoidei', side: 'A', cov: 'missing', path: [],
        from: { ru: 'мечевидный отросток', at: p('thA', [0, 1.22, 0.117]) }, to: { ru: 'подъязычная кость, нижняя челюсть', at: lm('голова', 'chin') } }),
    ],
    poses: {
      dog: { myers: 'mixed', ru: 'поясничная укорочена (бедро согнуто), приводящие и задняя большеберцовая удлинены; глубокий кор держит поясницу в прогибе' },
      plank: { myers: 'loaded', ru: 'работает как стабилизатор: поясничная и подвздошная не дают тазу провиснуть, вместе с поперечной живота' },
      half: { myers: 'neutral', ru: 'в нашем полупрогибе бёдра лежат на полу (разгибания бедра нет) — поясничная почти нейтральна; удлиняется передняя продольная связка с прогибом груди' },
      bow: { myers: 'lengthened', ru: 'растягивается сильнее всего: передняя капсула бедра, поясничная, передняя продольная связка; живот на полу ограничивает дыхание' },
      fold: { myers: 'mixed', ru: 'поясничная укорочена, задние волокна большой приводящей удлинены вместе с задней поверхностью бедра' },
    } },
  { id: 'SFAL', ru: 'Поверхностная фронтальная линия руки', abbr: 'ПФЛР', en: 'Superficial Front Arm Line', crossing: false,
    fn: 'приводит руку и сгибает кисть, держит руку при опоре и хвате; связывает грудь и спину (широчайшая) с ладонью',
    evidence: 'отдельно не проверялась',
    tracks: [
      T({ ru: 'Большая грудная и широчайшая мышцы', lat: 'm. pectoralis major; m. latissimus dorsi', units: ['pecMaj', 'lat'], side: 'A', cov: 'full', path: [{ unit: 'pecMaj', k: 2 }],
        from: { ru: 'медиальная треть ключицы, рёберные хрящи, грудопоясничная фасция, гребень подвздошной', at: o('pecMaj', 2) }, to: { ru: 'медиальная линия плечевой кости', at: i('pecMaj', 2) } }),
      T({ ru: 'Медиальная межмышечная перегородка плеча', lat: 'septum intermusculare brachii mediale', fascia: ['medialSeptumArm'], side: 'A', cov: 'missing', path: [{ fascia: 'medialSeptumArm' }],
        from: { ru: 'медиальная линия плеча', at: p('hum_L', [0.195, 1.33, -0.008]) }, to: { ru: 'медиальный надмыщелок', at: o('wristFlex', 1) }, note: 'статична — вся на плечевой кости' }),
      T({ ru: 'Сгибатели запястья и пальцев (общее сгибательное начало), длинная ладонная', lat: 'mm. flexores carpi et digitorum; m. palmaris longus', units: ['wristFlex', 'fingerFlex', 'palmaris'], side: 'A', cov: 'full', path: [{ unit: 'wristFlex', k: 1 }],
        from: { ru: 'медиальный надмыщелок', at: o('wristFlex', 1) }, to: { ru: 'ладонная сторона запястья', at: i('wristFlex', 1) } }),
      T({ ru: 'Запястный канал, ладонный апоневроз — до ладонной поверхности пальцев', lat: 'canalis carpi; aponeurosis palmaris', fascia: ['palmarAponeurosis'], side: 'A', cov: 'partial', path: [{ fascia: 'palmarAponeurosis' }],
        from: { ru: 'запястье', at: i('palmaris') }, to: { ru: 'ладонная поверхность пальцев', at: i('fingerFlex', 1) }, note: 'сухожилия сгибателей пальцев есть, апоневроза нет' }),
    ],
    poses: {
      dog: { myers: 'lengthened-loaded', ru: 'руки над головой — грудная и широчайшая удлинены; кисть разогнута на 90° — сгибатели предплечья растянуты и одновременно держат вес' },
      plank: { myers: 'loaded', ru: 'грудная и широчайшая держат плечо, сгибатели пальцев сжимают кулаки' },
      half: { myers: 'lengthened', ru: 'руки назад в W — большая грудная удлинена' },
      bow: { myers: 'lengthened-loaded', ru: 'руки назад к голеням — грудная удлинена, сгибатели пальцев держат хват' },
      fold: { myers: 'loaded', ru: 'хват за голени: работают сгибатели пальцев' },
    } },
  { id: 'DFAL', ru: 'Глубокая фронтальная линия руки', abbr: 'ГФЛР', en: 'Deep Front Arm Line', crossing: false,
    fn: 'от рёбер через малую грудную и двуглавую к большому пальцу: тянет и стабилизирует плечо спереди, держит «лучевую» сторону кисти при опоре',
    evidence: 'отдельно не проверялась',
    tracks: [
      T({ ru: 'Малая грудная, ключично-грудная фасция (и подключичная)', lat: 'm. pectoralis minor; fascia clavipectoralis; m. subclavius', units: ['pecMin'], related: ['subclavius'], fascia: ['clavipectoral'], side: 'A', cov: 'partial', path: [{ unit: 'pecMin', k: 1 }],
        from: { ru: '3–5-е рёбра', at: o('pecMin', 1) }, to: { ru: 'клювовидный отросток', at: i('pecMin', 1) }, note: 'нет фасции' }),
      T({ ru: 'Двуглавая мышца плеча и клювовидно-плечевая', lat: 'm. biceps brachii; m. coracobrachialis', units: ['bicShort', 'bicLong'], related: ['coracobr'], side: 'A', cov: 'full', path: [{ unit: 'bicShort' }],
        from: { ru: 'клювовидный отросток', at: o('bicShort') }, to: { ru: 'бугристость лучевой кости', at: i('bicShort') } }),
      T({ ru: 'Надкостница лучевой кости', lat: 'periosteum radii', fascia: ['radialPeriosteum'], side: 'A', cov: 'missing', path: [],
        from: { ru: 'бугристость лучевой', at: i('bicShort') }, to: { ru: 'шиловидный отросток лучевой', at: i('brachiorad') }, note: 'статична' }),
      T({ ru: 'Лучевые коллатеральные связки запястья, мышцы возвышения большого пальца', lat: 'lig. collaterale carpi radiale; mm. thenaris', units: ['thenar'], related: ['apl'], caps: ['wr'], side: 'A', cov: 'partial', path: [{ unit: 'thenar' }],
        from: { ru: 'ладьевидная, трапеция', at: o('thenar') }, to: { ru: 'наружная сторона большого пальца', at: i('thenar') }, note: 'связки запястья — только общей капсулой' }),
    ],
    poses: {
      dog: { myers: 'loaded', ru: 'держит опору через основание указательного и большого пальцев; малая грудная удлинена поднятыми руками' },
      plank: { myers: 'loaded', ru: 'двуглавая держит локоть под 90°, малая грудная — лопатку' },
      half: { myers: 'mixed', ru: 'малая грудная удлинена (плечо назад), двуглавая укорочена согнутым локтем' },
      bow: { myers: 'lengthened', ru: 'прямые руки назад — двуглавая растянута сильнее всего в круге' },
      fold: { myers: 'mixed', ru: 'руки держат голени согнутыми локтями — двуглавая укорочена' },
    } },
  { id: 'SBAL', ru: 'Поверхностная задняя линия руки', abbr: 'ПЗЛР', en: 'Superficial Back Arm Line', crossing: false,
    fn: 'поднимает и отводит руку, разгибает кисть и пальцы; от затылка и грудных остистых к тылу пальцев',
    evidence: 'отдельно не проверялась',
    tracks: [
      T({ ru: 'Трапециевидная мышца', lat: 'm. trapezius', units: ['trapUp', 'trapMid', 'trapLow'], ligs: ['nuchal'], side: 'A', cov: 'full', path: [{ unit: 'trapUp', k: 0 }],
        from: { ru: 'затылочный гребень, выйная связка, грудные остистые', at: o('trapUp', 0) }, to: { ru: 'ость лопатки, акромион, латеральная треть ключицы', at: i('trapUp', 0) } }),
      T({ ru: 'Дельтовидная мышца', lat: 'm. deltoideus', units: ['deltAnt', 'deltMid', 'deltPost'], side: 'A', cov: 'full', path: [{ unit: 'deltMid', k: 1 }],
        from: { ru: 'ость лопатки, акромион, ключица', at: o('deltMid', 1) }, to: { ru: 'дельтовидная бугристость', at: i('deltMid', 1) } }),
      T({ ru: 'Латеральная межмышечная перегородка плеча', lat: 'septum intermusculare brachii laterale', fascia: ['lateralSeptumArm'], side: 'A', cov: 'missing', path: [{ fascia: 'lateralSeptumArm' }],
        from: { ru: 'дельтовидная бугристость', at: i('deltMid', 1) }, to: { ru: 'латеральный надмыщелок', at: o('wristExt', 1) }, note: 'статична' }),
      T({ ru: 'Разгибатели запястья и пальцев (общее разгибательное начало)', lat: 'mm. extensores carpi et digitorum', units: ['wristExt'], related: ['brachiorad'], side: 'A', cov: 'partial', path: [{ unit: 'wristExt', k: 1 }],
        from: { ru: 'латеральный надмыщелок', at: o('wristExt', 1) }, to: { ru: 'тыльная поверхность пальцев', at: i('wristExt', 1) }, note: 'в модели — до пясти, сухожилий на пальцах нет' }),
    ],
    poses: {
      dog: { myers: 'loaded', ru: 'нижняя и средняя трапеции поворачивают лопатки вверх, дельтовидные держат руки над головой под весом; разгибатели кисти укорочены' },
      plank: { myers: 'loaded', ru: 'дельтовидные и трапеции держат плечо над локтем' },
      half: { myers: 'shortened-active', ru: 'главная рабочая линия W-рук: средняя и нижняя трапеции сводят лопатки, задняя дельта уводит руки назад' },
      bow: { myers: 'shortened', ru: 'трапеции держат плечи раскрытыми; руки тянет хват, а не мышцы' },
      fold: { myers: 'neutral', ru: 'руки держат голени — линия почти нейтральна' },
    } },
  { id: 'DBAL', ru: 'Глубокая задняя линия руки', abbr: 'ГЗЛР', en: 'Deep Back Arm Line', crossing: false,
    fn: 'от остистых отростков через ромбовидные, вращательную манжету и трёхглавую к мизинцу: держит лопатку и плечо сзади, разгибает локоть, держит «локтевую» сторону кисти',
    evidence: 'отдельно не проверялась',
    tracks: [
      T({ ru: 'Ромбовидные и мышца, поднимающая лопатку', lat: 'mm. rhomboidei; m. levator scapulae', units: ['rhomb', 'levScap'], side: 'A', cov: 'full', path: [{ unit: 'rhomb', k: 1 }],
        from: { ru: 'остистые отростки C7–T5, поперечные отростки C1–C4', at: o('rhomb', 1) }, to: { ru: 'медиальный край лопатки', at: i('rhomb', 1) } }),
      T({ ru: 'Вращательная манжета: подостная, малая круглая, надостная, подлопаточная', lat: 'mm. infraspinatus, teres minor, supraspinatus, subscapularis', units: ['infra', 'teresMin', 'supra', 'subscap'], related: ['teresMaj'], side: 'A', cov: 'full', path: [{ unit: 'infra', k: 1 }],
        from: { ru: 'медиальный край и ямки лопатки', at: o('infra', 1) }, to: { ru: 'головка плечевой кости', at: i('infra', 1) } }),
      T({ ru: 'Трёхглавая мышца плеча', lat: 'm. triceps brachii', units: ['tricLong', 'tricLat', 'tricMed'], related: ['anconeus'], side: 'A', cov: 'full', path: [{ unit: 'tricLong' }],
        from: { ru: 'головка плечевой (подсуставной бугорок лопатки)', at: o('tricLong') }, to: { ru: 'локтевой отросток', at: i('tricLong') } }),
      T({ ru: 'Фасция вдоль надкостницы локтевой кости', lat: 'periosteum ulnae', fascia: ['ulnarPeriosteum'], side: 'A', cov: 'missing', path: [],
        from: { ru: 'локтевой отросток', at: i('tricLong') }, to: { ru: 'шиловидный отросток локтевой', at: lm('предплечья', 'foreWrist') }, note: 'статична' }),
      T({ ru: 'Локтевые коллатеральные связки запястья, мышцы возвышения мизинца', lat: 'lig. collaterale carpi ulnare; mm. hypothenaris', units: ['hypothenar'], caps: ['wr'], side: 'A', cov: 'partial', path: [{ unit: 'hypothenar' }],
        from: { ru: 'трёхгранная, крючковидная', at: o('hypothenar') }, to: { ru: 'наружная сторона мизинца', at: i('hypothenar') }, note: 'связки — только общей капсулой' }),
    ],
    poses: {
      dog: { myers: 'loaded', ru: 'трёхглавая выпрямляет локти под весом, подостная и малая круглая разворачивают плечи наружу; ромбовидные удлинены (лопатки разведены и повёрнуты вверх)' },
      plank: { myers: 'loaded', ru: 'ромбовидные с передней зубчатой держат лопатку на грудной клетке' },
      half: { myers: 'active', ru: '«свёл лопатки»: ромбовидные работают, подостная разворачивает плечи наружу; трёхглавая удлинена согнутым локтем' },
      bow: { myers: 'shortened-active', ru: 'ромбовидные укорочены, трёхглавая держит локти прямыми в тяге' },
      fold: { myers: 'lengthened', ru: 'руки держат голени: лопатки разведены, локти согнуты — ромбовидные и трёхглавая удлинены' },
    } },
  { id: 'BFL', ru: 'Задняя функциональная линия', abbr: 'ЗФЛ', en: 'Back Functional Line', crossing: true,
    fn: 'диагональ спины: широчайшая одной стороны через грудопоясничную фасцию к большой ягодичной и латеральной широкой другой — передаёт силу рук ногам и наоборот (бросок, шаг, толчок)',
    evidence: 'сильная: подтверждены все 3 перехода (Wilke 2016)',
    sideNote: 'название по стороне широчайшей: ЗФЛ левая = левая широчайшая → правая большая ягодичная → правая латеральная широкая',
    tracks: [
      T({ ru: 'Широчайшая мышца спины', lat: 'm. latissimus dorsi', units: ['lat'], side: 'A', cov: 'full', path: [{ unit: 'lat', k: 3 }],
        from: { ru: 'тело плечевой кости', at: i('lat', 3) }, to: { ru: 'грудопоясничная фасция', at: o('lat', 3) } }),
      T({ ru: 'Грудопоясничная и крестцовая фасции — переход через срединную линию', lat: 'fascia thoracolumbalis; fascia sacralis', fascia: ['tlfCross', 'thoracolumbarFascia'], side: 'X', cov: 'missing', path: [{ fascia: 'tlfCross' }],
        from: { ru: 'поясничная часть широчайшей', at: o('lat', 3) }, to: { ru: 'крестец другой стороны', at: p('pelvis', [-0.042, 1.01, -0.112]) } }),
      T({ ru: 'Большая ягодичная мышца (другая сторона)', lat: 'm. gluteus maximus', units: ['gluteMax'], side: 'B', cov: 'full', path: [{ unit: 'gluteMax', k: 2 }],
        from: { ru: 'крестец', at: o('gluteMax', 2) }, to: { ru: 'ягодичная бугристость бедра', at: i('gluteMax', 2) }, note: 'берём средние (крестцовые) волокна: верхние в глубоком сгибании бедра модель укорачивает — путь перешагивает ось сустава' }),
      T({ ru: 'Латеральная широкая мышца бедра', lat: 'm. vastus lateralis', units: ['vastLat'], side: 'B', cov: 'full', path: [{ unit: 'vastLat' }],
        from: { ru: 'тело бедра', at: o('vastLat') }, to: { ru: 'надколенник', at: i('vastLat') } }),
      T({ ru: 'Связка надколенника', lat: 'lig. patellae', ligs: ['patTend'], side: 'B', cov: 'static', path: [{ lig: 'patTend' }],
        from: { ru: 'надколенник', at: lo('patTend') }, to: { ru: 'бугристость большеберцовой', at: li('patTend') } }),
    ],
    poses: {
      dog: { myers: 'lengthened', ru: 'обе диагонали удлинены: руки над головой тянут широчайшие, согнутые бёдра — большие ягодичные' },
      plank: { myers: 'loaded', ru: 'диагонали держат таз и плечи как единое целое — изометрия' },
      half: { myers: 'active', ru: 'большая ягодичная и латеральная широкая держат таз и ноги на полу, широчайшая уводит руки назад' },
      bow: { myers: 'shortened-active', ru: 'большая ягодичная работает сильно; широчайшую тянет хват' },
      fold: { myers: 'lengthened', ru: 'большая ягодичная растянута глубоким сгибанием бедра' },
    } },
  { id: 'FFL', ru: 'Фронтальная функциональная линия', abbr: 'ФФЛ', en: 'Front Functional Line', crossing: true,
    fn: 'диагональ спереди: нижний край большой грудной через влагалище прямой мышцы живота и лобковый симфиз к длинной приводящей другой стороны',
    evidence: 'сильная: подтверждены оба перехода (Wilke 2016)',
    sideNote: 'название по стороне большой грудной: ФФЛ левая = левая грудная → правая длинная приводящая',
    tracks: [
      T({ ru: 'Нижний край большой грудной мышцы', lat: 'm. pectoralis major (pars abdominalis)', units: ['pecMaj'], side: 'A', cov: 'full', path: [{ unit: 'pecMaj', k: 4 }],
        from: { ru: 'тело плечевой кости', at: i('pecMaj', 4) }, to: { ru: 'хрящи 5–6-го рёбер', at: o('pecMaj', 4) } }),
      T({ ru: 'Полулунная линия, латеральный край влагалища прямой мышцы живота', lat: 'linea semilunaris; vagina m. recti abdominis', units: ['rectAbd'], fascia: ['rectusSheath'], side: 'A', cov: 'partial', path: [{ unit: 'rectAbd', k: 2 }],
        from: { ru: '5–6-е рёбра', at: i('rectAbd', 2) }, to: { ru: 'лобковый бугорок и симфиз (переход на другую сторону)', at: o('rectAbd', 2) } }),
      T({ ru: 'Длинная приводящая мышца (другая сторона)', lat: 'm. adductor longus', units: ['addLong'], side: 'B', cov: 'full', path: [{ unit: 'addLong' }],
        from: { ru: 'лобковый бугорок', at: o('addLong') }, to: { ru: 'шероховатая линия бедра', at: i('addLong') } }),
    ],
    poses: {
      dog: { myers: 'lengthened', ru: 'руки над головой удлиняют нижний край грудной; линия натянута по диагонали спереди' },
      plank: { myers: 'loaded', ru: 'передняя диагональ держит опору руки с противоположной ногой — изометрия' },
      half: { myers: 'lengthened', ru: 'прогиб удлиняет переднюю диагональ' },
      bow: { myers: 'lengthened', ru: 'растянута вместе с ПФЛ' },
      fold: { myers: 'shortened', ru: 'укорочена' },
    } },
  { id: 'IFL', ru: 'Ипсилатеральная функциональная линия', abbr: 'ИФЛ', en: 'Ipsilateral Functional Line', crossing: false,
    fn: 'боковая диагональ одной стороны: наружный край широчайшей → наружная косая → портняжная; стабилизирует при опоре на руку и при подтягивании',
    evidence: 'отдельно не проверялась',
    tracks: [
      T({ ru: 'Наружный край широчайшей мышцы спины', lat: 'm. latissimus dorsi (margo lateralis)', units: ['lat'], side: 'A', cov: 'full', path: [{ unit: 'lat', k: 4 }],
        from: { ru: 'тело плечевой кости', at: i('lat', 4) }, to: { ru: '10–12-е рёбра, гребень подвздошной', at: o('lat', 4) } }),
      T({ ru: 'Наружная косая мышца живота', lat: 'm. obliquus externus abdominis', units: ['extObl'], side: 'A', cov: 'full', path: [{ unit: 'extObl', k: 3 }],
        from: { ru: 'нижние рёбра', at: o('extObl', 3) }, to: { ru: 'ПВПО', at: i('extObl', 3) } }),
      T({ ru: 'Портняжная мышца', lat: 'm. sartorius', units: ['sartorius'], side: 'A', cov: 'full', path: [{ unit: 'sartorius' }],
        from: { ru: 'ПВПО', at: o('sartorius') }, to: { ru: '«гусиная лапка», медиальный мыщелок большеберцовой', at: i('sartorius') } }),
    ],
    poses: {
      dog: { myers: 'mixed', ru: 'широчайшая удлинена руками над головой, портняжная укорочена согнутым бедром' },
      plank: { myers: 'loaded', ru: 'держит бок туловища от провисания' },
      half: { myers: 'neutral', ru: 'бёдра на полу, линия почти нейтральна; наружная косая держит рёбра' },
      bow: { myers: 'lengthened', ru: 'портняжная слегка удлинена (бедро разогнуто, колено согнуто), наружная косая — прогибом' },
      fold: { myers: 'shortened', ru: 'портняжная укорочена сгибанием бедра' },
    } },
];

// ---------- model side: lengths and activations ----------
const other = s => (s === 'L' ? 'R' : 'L');
const mirSeg = s => s.replace(/_L$/, '_R');
const mirPt = q => ({ seg: mirSeg(q.seg), p: [-q.p[0], q.p[1], q.p[2]] });
function trackSide(lineSide, t) { return t.side === 'B' ? other(lineSide) : lineSide; }
function polyLen(W, pts) { const P3 = pts.map(q => A.pt(W, q.seg, V(...q.p))); let L = 0; for (let k = 1; k < P3.length; k++) L += P3[k].distanceTo(P3[k - 1]); return L; }
function pathLen(W, el, side, crossingSide) {
  if (el.unit || el.lig) {
    const kind = el.unit ? 'muscle' : 'lig', unit = el.unit || el.lig;
    const ks = INST.filter(i => i.kind === kind && i.unit === unit && i.side === side).map(i => i.idx);
    const inst = instOf(kind, unit, side, el.k ?? ks[0]);
    return A.pathLength(A.strandPath(W, inst));
  }
  if (el.fascia) {
    const f = FASCIA[el.fascia];
    let pts = f.band;
    if (f.side === 'L' && side === 'R') pts = pts.map(mirPt);
    if (f.side === 'X' && crossingSide === 'R') pts = pts.map(mirPt);
    return polyLen(W, pts);
  }
  return 0;
}
const data = JSON.parse(fs.readFileSync(path.join(ROOT, 'poses.json'), 'utf8'));
const PHYS = JSON.parse(fs.readFileSync(path.join(ROOT, 'physics.json'), 'utf8'));
const POSE_IDS = ['dog', 'plank', 'half', 'bow', 'fold'];
const confs = { stand: A.STAND };
for (const id of POSE_IDS) confs[id] = A.poseFromData(data[id]);
const Ws = Object.fromEntries(Object.entries(confs).map(([k, c]) => [k, A.placeW(c.local, c.rootQ)]));
function lineLengths(line, W, side) {
  return line.tracks.map(t => t.path.reduce((s, el) => s + pathLen(W, el, trackSide(side, t), side), 0));
}
const pct = (a, b) => (b > 1e-6 ? +((a / b - 1) * 100).toFixed(1) : null);
function actOf(units, pose) { const m = PHYS[pose] && PHYS[pose].muscles; if (!m) return null; const v = units.filter(u => m[u]).map(u => ({ u, a: m[u].a })); return v; }

const out = { source: 'Т. Майерс «Анатомические поезда» — карта линий; переходы — Wilke et al. 2016; точки — модель (координаты стойки)', generated: new Date().toISOString(), poses: POSE_IDS, lines: [], fascia: {}, missingMuscles: MISSING_MUSCLES, coverage: {} };
const covW = { full: 1, static: 1, partial: 0.5, missing: 0 };
for (const line of LINES) {
  const L0 = lineLengths(line, Ws.stand, 'L');
  const model = { length: {}, activation: {} };
  for (const pose of POSE_IDS) {
    const Lp = lineLengths(line, Ws[pose], 'L');
    const tot0 = L0.reduce((a, b) => a + b, 0), tot = Lp.reduce((a, b) => a + b, 0);
    model.length[pose] = { totalPct: pct(tot, tot0), tracksPct: Lp.map((l, k) => (L0[k] > 1e-6 ? pct(l, L0[k]) : null)) };
    const units = [...new Set(line.tracks.flatMap(t => t.units || []))];
    const acts = actOf(units, pose) || [];
    const mean = acts.length ? acts.reduce((s, x) => s + x.a, 0) / acts.length : 0;
    model.activation[pose] = { mean: +mean.toFixed(3), max: acts.length ? +Math.max(...acts.map(x => x.a)).toFixed(3) : 0, top: acts.sort((a, b) => b.a - a.a).slice(0, 3).map(x => x.u + ' ' + Math.round(x.a * 100) + '%') };
  }
  const counts = { full: 0, static: 0, partial: 0, missing: 0 };
  for (const t of line.tracks) counts[t.cov]++;
  const n = line.tracks.length;
  const coverage = { ...counts, tracks: n, structuralPct: Math.round(line.tracks.reduce((s, t) => s + covW[t.cov], 0) / n * 100), dynamicPct: Math.round(line.tracks.reduce((s, t) => s + (t.cov === 'full' ? 1 : t.cov === 'partial' ? 0.5 : 0), 0) / n * 100) };
  const missing = [...new Set(line.tracks.flatMap(t => (t.fascia || [])).concat(Object.entries(MISSING_MUSCLES).filter(([, m]) => m.lines.includes(line.id)).map(([k]) => k)))];
  out.lines.push({
    id: line.id, ru: line.ru, abbr: line.abbr, en: line.en, crossing: line.crossing, sideNote: line.sideNote || null, fn: line.fn, evidence: line.evidence,
    tracks: line.tracks.map((t, k) => ({ n: k + 1, ru: t.ru, lat: t.lat, side: t.side, model: { units: t.units || [], related: t.related || [], ligs: t.ligs || [], caps: t.caps || [], fascia: t.fascia || [] }, cov: t.cov, note: t.note || null, path: t.path, from: t.from, to: t.to, standLength: +L0[k].toFixed(3) })),
    coverage, missing, poses: line.poses, model,
  });
  out.coverage[line.id] = coverage;
}
for (const [k, f] of Object.entries(FASCIA)) out.fascia[k] = { ...f, static: !!f.static || (new Set((f.outline || f.band).map(q => q.seg.replace(/_[LR]$/, ''))).size === 1) };
// transitions: line length range over each segment of the round, both sides (if the segment files exist)
out.transitions = {};
for (const sid of ['seg1', 'seg2', 'seg3', 'seg4', 'seg5']) {
  const f = path.join(ROOT, 'cycle_keys', sid + '.json');
  if (!fs.existsSync(f)) continue;
  const seg = JSON.parse(fs.readFileSync(f, 'utf8'));
  const q = a => new THREE.Quaternion(...a);
  const res = {};
  const frames = seg.frames.filter((_, k) => k % 2 === 0 || k === seg.frames.length - 1);
  for (const line of LINES) {
    const r = {};
    for (const sd of ['L', 'R']) {
      const L0 = lineLengths(line, Ws.stand, sd).reduce((a, b) => a + b, 0);
      let mn = Infinity, mx = -Infinity, tmn = 0, tmx = 0;
      for (const fr of frames) {
        const Lc = {}; for (const [s, v] of Object.entries(fr.local)) Lc[s] = q(v);
        const W = A.computeWorld(Lc, q(fr.root), V(...fr.rootP));
        const v = pct(lineLengths(line, W, sd).reduce((a, b) => a + b, 0), L0);
        if (v < mn) { mn = v; tmn = fr.t; } if (v > mx) { mx = v; tmx = fr.t; }
      }
      r[sd] = { minPct: mn, minT: tmn, maxPct: mx, maxT: tmx };
    }
    res[line.id] = r;
  }
  out.transitions[sid] = { from: seg.from, to: seg.to, duration: seg.duration, lines: res };
}
fs.mkdirSync(path.join(ROOT, 'books'), { recursive: true });
fs.writeFileSync(path.join(ROOT, 'books', 'myers_lines.json'), JSON.stringify(out, null, 1));
for (const l of out.lines) console.log(l.id.padEnd(5), 'tracks', l.coverage.tracks, 'full', l.coverage.full, 'static', l.coverage.static, 'partial', l.coverage.partial, 'missing', l.coverage.missing, '| structural', l.coverage.structuralPct + '%', 'dynamic', l.coverage.dynamicPct + '%', '| length vs stand:', POSE_IDS.map(p => p + ' ' + l.model.length[p].totalPct + '%').join(', '), '| act mean:', POSE_IDS.map(p => p + ' ' + Math.round(l.model.activation[p].mean * 100) + '%').join(', '));
console.log('fascia elements', Object.keys(out.fascia).length, '| transitions', Object.keys(out.transitions).join(','));
