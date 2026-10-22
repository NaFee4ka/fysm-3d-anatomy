// Muscle parameters for the static model (one side).
// F   — maximal isometric force of the whole muscle, N. Lower limb after Delp's Gait2392 model,
//       arm and rotator cuff after Holzbaur 2005, trunk after Christophy 2012, neck after Vasavada 1998,
//       shoulder girdle after van der Helm 1994; small muscles from cross-sections (about 35 N per cm²).
// phi — share of the muscle-tendon length taken by muscle fibres (the rest is tendon);
//       a stretch of the whole muscle-tendon unit stretches the fibres 1/phi times more.
export const MUSCLE = {
  // shoulder girdle
  trapUp: { F: 400, phi: 0.85 }, trapMid: { F: 600, phi: 0.9 }, trapLow: { F: 350, phi: 0.9 },
  rhomb: { F: 450, phi: 0.9 }, levScap: { F: 280, phi: 0.8 }, serratus: { F: 900, phi: 0.9 },
  pecMin: { F: 400, phi: 0.8 }, subclavius: { F: 150, phi: 0.7 },
  // shoulder
  pecMaj: { F: 1730, phi: 0.75 }, lat: { F: 1060, phi: 0.7 },
  deltAnt: { F: 1140, phi: 0.5 }, deltMid: { F: 1140, phi: 0.5 }, deltPost: { F: 260, phi: 0.78 },
  supra: { F: 490, phi: 0.63 }, infra: { F: 1210, phi: 0.7 }, teresMin: { F: 350, phi: 0.5 }, subscap: { F: 1380, phi: 0.72 },
  teresMaj: { F: 425, phi: 0.85 }, coracobr: { F: 240, phi: 0.5 },
  // arm and forearm
  tricLong: { F: 800, phi: 0.48 }, tricLat: { F: 620, phi: 0.53 }, tricMed: { F: 620, phi: 0.53 }, anconeus: { F: 350, phi: 0.6 },
  bicLong: { F: 625, phi: 0.3 }, bicShort: { F: 435, phi: 0.4 }, brachialis: { F: 990, phi: 0.6 }, brachiorad: { F: 260, phi: 0.56 },
  pronTeres: { F: 560, phi: 0.27 }, pronQuad: { F: 75, phi: 0.85 }, supinator: { F: 475, phi: 0.54 },
  wristFlex: { F: 350, phi: 0.2 }, wristExt: { F: 500, phi: 0.25 }, palmaris: { F: 27, phi: 0.19 },
  fingerFlex: { F: 1000, phi: 0.2 }, apl: { F: 60, phi: 0.33 },
  thenar: { F: 150, phi: 0.6 }, hypothenar: { F: 100, phi: 0.6 }, interossei: { F: 240, phi: 0.5 },
  // neck
  scm: { F: 300, phi: 0.7 }, scalene: { F: 220, phi: 0.7 }, splenius: { F: 250, phi: 0.8 }, semispinalis: { F: 280, phi: 0.7 }, longusColli: { F: 100, phi: 0.6 },
  // trunk
  erector: { F: 2400, phi: 0.55 }, multifidus: { F: 500, phi: 0.8 }, ql: { F: 300, phi: 0.7 },
  rectAbd: { F: 450, phi: 0.9 }, extObl: { F: 900, phi: 0.85 }, intObl: { F: 800, phi: 0.85 }, transAbd: { F: 500, phi: 0.85 },
  // hip
  psoas: { F: 800, phi: 0.5 }, iliacus: { F: 800, phi: 0.5 },
  gluteMax: { F: 1300, phi: 0.65 }, gluteMed: { F: 1360, phi: 0.5 }, gluteMin: { F: 585, phi: 0.8 },
  piriformis: { F: 300, phi: 0.25 }, obtInt: { F: 250, phi: 0.4 }, gemelli: { F: 110, phi: 0.4 }, quadFem: { F: 250, phi: 0.7 }, obtExt: { F: 300, phi: 0.4 },
  tfl: { F: 230, phi: 0.2 }, sartorius: { F: 160, phi: 0.78 }, pectineus: { F: 260, phi: 0.75 },
  addLong: { F: 420, phi: 0.5 }, addBrev: { F: 290, phi: 0.8 }, addMag: { F: 1210, phi: 0.6 }, gracilis: { F: 160, phi: 0.57 },
  // thigh
  bicFemL: { F: 900, phi: 0.23 }, bicFemS: { F: 800, phi: 0.5 }, semiT: { F: 410, phi: 0.44 }, semiM: { F: 1290, phi: 0.17 },
  rectFem: { F: 780, phi: 0.18 }, vastLat: { F: 1870, phi: 0.4 }, vastMed: { F: 1290, phi: 0.45 }, vastInt: { F: 1365, phi: 0.45 },
  // leg and foot
  popliteus: { F: 120, phi: 0.6 }, plantaris: { F: 60, phi: 0.1 },
  gastroM: { F: 1560, phi: 0.11 }, gastroL: { F: 680, phi: 0.14 }, soleus: { F: 3550, phi: 0.13 },
  tibAnt: { F: 905, phi: 0.22 }, edl: { F: 510, phi: 0.17 }, ehl: { F: 160, phi: 0.2 },
  fibL: { F: 940, phi: 0.13 }, fibB: { F: 435, phi: 0.23 }, tibPost: { F: 1590, phi: 0.12 },
  fhl: { F: 320, phi: 0.13 }, fdl: { F: 310, phi: 0.1 },
  abdHall: { F: 150, phi: 0.45 }, fdb: { F: 150, phi: 0.45 }, adm: { F: 80, phi: 0.45 },
};

// Muscles a cue switches on on purpose (fraction of maximal activation).
// The rest of the work is left to the solver.
const QUADS = ['vastLat', 'vastMed', 'vastInt', 'rectFem'];
const ABS = ['rectAbd', 'extObl', 'intObl', 'transAbd'];
const ofList = (list, v) => Object.fromEntries(list.map(u => [u, v]));
export const CUE_TONE = {
  dog: Object.assign(ofList(['rhomb', 'trapMid'], 0.12), { trapLow: 0.15, erector: 0.08, multifidus: 0.1 }, ofList(['interossei', 'thenar', 'hypothenar'], 0.15)),
  plank: Object.assign(ofList(QUADS, 0.15), { rectAbd: 0.15, gluteMax: 0.12, fingerFlex: 0.05 }),
  half: Object.assign(ofList(QUADS, 0.12), ofList(ABS, 0.1), { gluteMax: 0.12, wristExt: 0.08, trapLow: 0.15, rhomb: 0.2, trapMid: 0.2, deltPost: 0.15, gastroM: 0.04, gastroL: 0.04, soleus: 0.04 }),
  // the heel push is not forced: the legs pulling the trunk up through the arms is what the least-effort solution finds
  bow: Object.assign(ofList(['addLong', 'addBrev', 'addMag', 'gracilis'], 0.15), { tibAnt: 0.2, edl: 0.12, ehl: 0.12, rhomb: 0.15, trapMid: 0.15, fingerFlex: 0.15 }),
  // the hands only rest round the shins; the pelvis is tilted forward by the hip flexors on purpose
  fold: Object.assign({ psoas: 0.08, iliacus: 0.08, fingerFlex: 0.03 }, ofList(['tfl', 'gluteMin', 'addLong'], 0.05)),
  // the fingertips and the heels of the hands are pressed on purpose; the rest is left to the solver
  handstand: { fingerFlex: 0.05 },
  stand: {},
};

// Passive pull of long muscles a pose stretches: zero below strain e0 of the muscle-tendon unit,
// frac*Fmax at strain eLim, steeper beyond (exponential, like the passive force of muscle fibres).
// The ranges describe a flexible practitioner who can hold these poses.
export const PASSIVE = {
  // hamstrings: at the very end of a straight-leg raise the passive hip moment reaches some 40–60 N·m (Magnusson, Halbertsma);
  // the folds here stop a little short of that
  bicFemL: { e0: 0.08, eLim: 0.45, frac: 0.25 }, semiT: { e0: 0.08, eLim: 0.45, frac: 0.25 }, semiM: { e0: 0.08, eLim: 0.45, frac: 0.25 },
  bicFemS: { e0: 0.1, eLim: 0.4, frac: 0.1 },
  // calves: the end of dorsiflexion (some 30° with the knee a little bent) is reached at about 7% stretch of the soleus
  gastroM: { e0: 0.01, eLim: 0.07, frac: 0.15 }, gastroL: { e0: 0.01, eLim: 0.07, frac: 0.15 }, soleus: { e0: 0.01, eLim: 0.07, frac: 0.15 },
  rectFem: { e0: 0.05, eLim: 0.3, frac: 0.2 }, psoas: { e0: 0.05, eLim: 0.3, frac: 0.15 }, iliacus: { e0: 0.05, eLim: 0.3, frac: 0.15 },
  addMag: { e0: 0.1, eLim: 0.5, frac: 0.15 }, addLong: { e0: 0.08, eLim: 0.4, frac: 0.12 }, addBrev: { e0: 0.08, eLim: 0.4, frac: 0.12 }, gracilis: { e0: 0.08, eLim: 0.4, frac: 0.12 },
  gluteMax: { e0: 0.1, eLim: 0.4, frac: 0.12 },
  pecMaj: { e0: 0.1, eLim: 0.5, frac: 0.12 }, lat: { e0: 0.1, eLim: 0.45, frac: 0.12 }, teresMaj: { e0: 0.1, eLim: 0.55, frac: 0.12 },
  deltAnt: { e0: 0.1, eLim: 0.4, frac: 0.1 }, tricLong: { e0: 0.1, eLim: 0.4, frac: 0.1 },
  rectAbd: { e0: 0.08, eLim: 0.3, frac: 0.12 }, extObl: { e0: 0.08, eLim: 0.3, frac: 0.1 }, erector: { e0: 0.08, eLim: 0.25, frac: 0.1 },
  // front of the shin: the end of plantar flexion (50–60°) stretches them by some 12%
  tibAnt: { e0: 0.03, eLim: 0.13, frac: 0.15 }, edl: { e0: 0.03, eLim: 0.13, frac: 0.15 }, ehl: { e0: 0.03, eLim: 0.13, frac: 0.15 },
};

// Spine: each model joint stands for 2–4 motion segments. k — rotational stiffness of discs and
// ligaments, N·m per degree (flexion, axial rotation, side bend); beyond 60% of the range a stiff
// end-range stop adds up to Mlim (facet joints, spinous processes, ligaments pulled taut).
// Ranges per model joint, degrees, for a flexible person.
export const SPINE_PASSIVE = {
  ls: { k: [1.2, 1.8, 1.2], flex: 35, ext: 20, side: 15, Mlim: 20 },
  l34: { k: [1.0, 1.5, 1.0], flex: 35, ext: 20, side: 15, Mlim: 20 },
  tl: { k: [1.0, 1.5, 1.0], flex: 20, ext: 14, side: 12, Mlim: 20 },
  t89: { k: [0.8, 1.2, 0.8], flex: 18, ext: 14, side: 10, Mlim: 20 },
  t45: { k: [0.8, 1.2, 0.8], flex: 18, ext: 14, side: 10, Mlim: 20 },
  ct: { k: [0.3, 0.45, 0.3], flex: 25, ext: 30, side: 20, Mlim: 5 },
  c45: { k: [0.25, 0.4, 0.25], flex: 25, ext: 30, side: 20, Mlim: 5 },
  ao: { k: [0.2, 0.3, 0.2], flex: 20, ext: 25, side: 15, Mlim: 4 },
};

// Floor: relative stiffness of the tissue under each contact (bony = stiff) and friction on a yoga mat.
export const CONTACT_K = {
  'ладони': 3, 'пальцы рук': 1.5, 'кулаки': 2, 'кисти': 2, 'предплечья': 2, 'локти': 3,
  'пятки': 3, 'подушечки стоп': 3, 'пальцы стоп': 1.5, 'тыл стоп': 1.5, 'наружный край стопы': 2,
  'колени': 2, 'голени': 2, 'икры': 1, 'бёдра': 1.5, 'задняя поверхность бёдер': 1, 'ягодицы': 1,
  'таз': 2.5, 'живот': 0.7, 'нижние рёбра': 2, 'грудь': 2, 'спина': 2, 'голова': 2.5, 'плечи': 2,
};
export const FRICTION = Object.assign(
  Object.fromEntries(['ладони', 'пальцы рук', 'кулаки', 'кисти', 'предплечья', 'локти', 'пятки', 'подушечки стоп', 'пальцы стоп', 'наружный край стопы', 'тыл стоп'].map(r => [r, 1.0])),
  Object.fromEntries(['живот', 'нижние рёбра', 'грудь', 'таз', 'бёдра', 'колени', 'голени', 'ягодицы', 'спина', 'голова', 'плечи', 'задняя поверхность бёдер', 'икры'].map(r => [r, 0.5])),
);

// How stiff the stretched muscles are differs from person to person and from region to region.
// The solver may scale the passive pull of each group between passiveLo and passiveHi times the curve above,
// at a small price: the practitioner is taken to be as flexible as the pose needs, within these limits.
export const PASSIVE_GROUPS = {
  hamstrings: ['bicFemL', 'bicFemS', 'semiT', 'semiM', 'addMag', 'gluteMax'],
  adductors: ['addLong', 'addBrev', 'gracilis'],
  calves: ['gastroM', 'gastroL', 'soleus'],
  hipFlexors: ['rectFem', 'psoas', 'iliacus'],
  abs: ['rectAbd', 'extObl'],
  back: ['erector'],
  shoulders: ['pecMaj', 'lat', 'teresMaj', 'deltAnt', 'tricLong'],
  shinFront: ['tibAnt', 'edl', 'ehl'],
};

// End of range of the limb joints (capsule, ligaments, bone, or soft tissue of the forearm meeting the upper arm):
// angle measured from standing, degrees; dir +1 acts beyond +from, dir -1 beyond -from; the resisting moment
// rises steeply and reaches M (N·m) at 'to'. Positive: ankle plantar flexion, knee and elbow flexion, wrist extension.
// Per-pose replacements: a flexible practitioner who can hold the pose has a larger range at the joint it needs.
// Handstand: palms flat with vertical forearms need the wrist 90–95° back (gymnasts and yogis who hold it have 95–105°).
export const POSE_STOPS = {
  // (the stretched tendons of the finger and wrist flexors and the palmar capsule carry part of the load: M is larger than for the other poses)
  handstand: { wrist: [{ dir: 1, from: 70, to: 100, M: 20 }, { dir: -1, from: 50, to: 85, M: 10 }] },
};
export const JOINT_STOPS = {
  ankle: [{ dir: 1, from: 40, to: 65, M: 15 }],
  knee: [{ dir: -1, from: 2, to: 10, M: 40 }],
  elbow: [{ dir: 1, from: 135, to: 155, M: 10 }, { dir: -1, from: 2, to: 10, M: 30 }],
  wrist: [{ dir: 1, from: 50, to: 85, M: 10 }, { dir: -1, from: 50, to: 85, M: 10 }],
};
