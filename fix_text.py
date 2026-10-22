m = open('t3_meta.js').read()
def rep(old, new, cnt=1):
    global m
    assert m.count(old) == cnt, (m.count(old), old[:80])
    m = m.replace(old, new)
rep("""  const fcm = v => { const c = Math.abs(v) * 100; return (c < 10 ? c.toFixed(1).replace('.', ',') : String(Math.round(c))) + ' см'; };""",
    """  const fcm = v => { const c = Math.abs(v) * 100; return (c < 10 ? c.toFixed(1).replace('.', ',') : String(Math.round(c))) + '\\u00a0см'; };
  const neckText = v => (Math.abs(v) < 2 ? 'шея ровно по линии позвоночника' : v < 0 ? 'шея разогнута на ' + fd(v) : 'шея согнута на ' + fd(v));
  const clavText = (v, ref) => 'ключица ' + (v < 0 ? 'опущена на ' : 'поднята на ') + fd(v) + (ref != null ? ' вместо ' + fd(ref) + ' без этого действия' : '');""")
rep("""        { t: 'Сводим лопатки, опускаем плечи к тазу', m: m => 'лопатки сведены на ' + fd(m.retract) + ', ключица поднята на ' + fd(m.clavElev) + (m.clavElevNoCue != null ? ' вместо ' + fd(m.clavElevNoCue) + ' без этого действия' : '') },""",
    """        { t: 'Сводим лопатки, опускаем плечи к тазу', m: m => 'лопатки сведены на ' + fd(m.retract) + ', ' + clavText(m.clavElev, m.clavElevNoCue) },""")
rep("""        { t: 'Плечи к тазу', m: m => 'ключица ' + (m.clavElev < 0 ? 'опущена на ' : 'поднята на ') + fd(m.clavElev) },""",
    """        { t: 'Плечи к тазу', m: m => clavText(m.clavElev, m.clavElevNoCue) },""")
rep("""m: m => 'шея: ' + fd(m.neck) }""", """m: m => neckText(m.neck) }""", 4)
rep("""        { t: 'Вес в центре стопы', m: m => 'центр масс над точкой ' + Math.round(m.comPct * 100) + ' % длины стопы от пятки' },""",
    """        { t: 'Вес в центре стопы', m: m => 'центр масс над точкой ' + Math.round(m.comPct * 100) + '\\u00a0% длины стопы от пятки' },""")
pairs = [
 ("notes(['vastLat', 'vastMed', 'vastInt', 'rectFem'], 'Держат подсогнутые колени. Когда пятки опустятся, выпрямляют ноги.')", "notes(['vastLat', 'vastMed', 'vastInt', 'rectFem'], 'Четырёхглавая мышца держит подсогнутые колени. Когда пятки опустятся, она выпрямит ноги.')"),
 ("notes(['gastroM', 'gastroL', 'soleus'], 'Из-за их натяжения пятки не достают пола.')", "notes(['gastroM', 'gastroL', 'soleus'], 'Из-за натяжения икроножной и камбаловидной пятки не достают пола.')"),
 ("notes(['extObl', 'intObl', 'transAbd'], 'Держат живот плоским, а тело в одну линию.')", "notes(['extObl', 'intObl', 'transAbd'], 'Вместе с другими мышцами живота держит живот плоским, а тело в одну линию.')"),
 ("notes(['vastLat', 'vastMed', 'vastInt'], 'Колени в тонусе: держат ноги идеально прямыми.')", "notes(['vastLat', 'vastMed', 'vastInt'], 'Колени в тонусе: четырёхглавая держит ноги идеально прямыми.')"),
 ("notes(['splenius', 'semispinalis'], 'Держат голову в продолжение позвоночника.')", "notes(['splenius', 'semispinalis'], 'Держит голову в продолжение позвоночника.')"),
 ("notes(['psoas', 'iliacus'], 'Держат ноги на одной линии с корпусом.')", "notes(['psoas', 'iliacus'], 'Держит ноги на одной линии с корпусом.')"),
 ("notes(['semispinalis', 'splenius'], 'Держат голову в продолжение позвоночника.')", "notes(['semispinalis', 'splenius'], 'Держит голову в продолжение позвоночника.')"),
 ("notes(['trapMid', 'rhomb'], 'Сводят лопатки.')", "{ trapMid: 'Сводит лопатки.', rhomb: 'Сводят лопатки.' }"),
 ("notes(['deltPost', 'deltMid', 'infra'], 'Держат локти поднятыми и разведёнными.')", "notes(['deltPost', 'deltMid', 'infra'], 'Держит локти поднятыми и разведёнными.')"),
 ("notes(['bicFemL', 'semiT', 'semiM'], 'Помогают ягодицам держать прямые ноги на полу.')", "notes(['bicFemL', 'semiT', 'semiM'], 'Задняя поверхность бедра помогает ягодицам держать прямые ноги на полу.')"),
 ("notes(['vastLat', 'vastMed', 'vastInt'], 'Выпрямляют колени и подтягивают надколенники, поэтому колени не касаются пола.')", "notes(['vastLat', 'vastMed', 'vastInt'], 'Четырёхглавая выпрямляет колени и подтягивает надколенники, поэтому колени не касаются пола.')"),
 ("notes(['transAbd', 'intObl', 'extObl'], 'Не дают пояснице провалиться в прогиб.')", "notes(['transAbd', 'intObl', 'extObl'], 'Пресс включён: не даёт пояснице провалиться в прогиб.')"),
 ("notes(['gastroM', 'gastroL', 'soleus'], 'Тянут носки от себя.')", "notes(['gastroM', 'gastroL', 'soleus'], 'Тянет носки от себя.')"),
 ("notes(['vastLat', 'vastMed', 'vastInt'], 'Толкают пятки вверх и назад в руки. Именно это поднимает грудь.')", "notes(['vastLat', 'vastMed', 'vastInt'], 'Четырёхглавая толкает пятки вверх и назад в руки. Именно это поднимает грудь.')"),
 ("notes(['gluteMax', 'bicFemL', 'semiT', 'semiM'], 'Разгибают бедро и поднимают колени.')", "notes(['gluteMax', 'bicFemL', 'semiT', 'semiM'], 'Разгибает бедро и поднимает колени.')"),
 ("notes(['addLong', 'addBrev', 'addMag', 'gracilis'], 'Ноги толкаем вовнутрь: не дают коленям разъехаться.')", "notes(['addLong', 'addBrev', 'addMag', 'gracilis'], 'Ноги толкаем вовнутрь: приводящие мышцы не дают коленям разъехаться.')"),
 ("notes(['tibAnt', 'edl', 'ehl'], 'Держат носки на себя.')", "notes(['tibAnt', 'edl', 'ehl'], 'Держит носки на себя.')"),
 ("notes(['erector', 'multifidus'], 'Держат прогиб.')", "notes(['erector', 'multifidus'], 'Держит прогиб.')"),
 ("notes(['trapMid', 'rhomb', 'trapLow'], 'Отводят плечи назад и раскрывают грудь.')", "{ trapMid: 'Отводит плечи назад и раскрывает грудь.', trapLow: 'Опускает плечи и раскрывает грудь.', rhomb: 'Отводят плечи назад и раскрывают грудь.' }"),
 ("notes(['pecMaj', 'deltAnt', 'bicLong', 'coracobr'], 'Сильно растянуты: руки отведены назад.')", "notes(['pecMaj', 'deltAnt', 'bicLong', 'coracobr'], 'Сильно растянута: руки отведены назад.')"),
 ("notes(['rectAbd', 'psoas', 'iliacus'], 'Растянуты прогибом и разгибанием бедра.')", "notes(['rectAbd', 'psoas', 'iliacus'], 'Растянута прогибом и разгибанием бедра.')"),
 ("notes(['infra', 'teresMin'], 'Вращают плечо наружу, поэтому локти смотрят внутрь.')", "notes(['infra', 'teresMin'], 'Вращает плечо наружу, поэтому локти смотрят внутрь.')"),
 ("notes(['bicFemL', 'semiT', 'semiM'], 'Главное растяжение позы. Чем они короче, тем сильнее округляется спина.')", "notes(['bicFemL', 'semiT', 'semiM'], 'Задняя поверхность бедра даёт главное растяжение позы. Чем она короче, тем сильнее округляется спина.')"),
 ("notes(['erector', 'multifidus'], 'Держат спину прямой против силы тяжести.')", "notes(['erector', 'multifidus'], 'Держит спину прямой против силы тяжести.')"),
 ("notes(['psoas', 'iliacus'], 'Тянут корпус к ногам и наклоняют таз вперёд.')", "notes(['psoas', 'iliacus'], 'Тянет корпус к ногам и наклоняет таз вперёд.')"),
 ("notes(['rectFem', 'vastLat', 'vastMed', 'vastInt'], 'Держат колени прямыми.')", "notes(['rectFem', 'vastLat', 'vastMed', 'vastInt'], 'Четырёхглавая держит колени прямыми.')"),
 ("notes(['bicLong', 'bicShort', 'brachialis'], 'Сгибают локти и подтягивают корпус к ногам.')", "notes(['bicLong', 'bicShort', 'brachialis'], 'Сгибает локоть и подтягивает корпус к ногам.')"),
 ("notes(['tibAnt', 'fibL', 'tibPost', 'abdHall', 'fdb', 'fhl'], 'Держат вес в центре стопы и своды.')", "notes(['tibAnt', 'fibL', 'tibPost', 'abdHall', 'fdb', 'fhl'], 'Держит вес в центре стопы и своды.')"),
 ("notes(['tfl', 'gluteMin', 'addLong'], 'Сводят большие пальцы стоп: вращают бедро внутрь и приводят его.')", "notes(['tfl', 'gluteMin', 'addLong'], 'Сводит большие пальцы стоп: вращает бедро внутрь и приводит его.')"),
 ("notes(['splenius', 'semispinalis'], 'Держат шею в продолжение позвоночника.')", "notes(['splenius', 'semispinalis'], 'Держит шею в продолжение позвоночника.')"),
 ("notes(['trapLow', 'trapMid'], 'Держат плечи от ушей.')", "notes(['trapLow', 'trapMid'], 'Держит плечи от ушей.')"),
]
for a, b in pairs: rep(a, b)
rep("        thenar: 'Отводит большой палец.',\n        hypothenar: 'Отводит мизинец.',", "        thenar: 'Отводят большой палец.',\n        hypothenar: 'Отводят мизинец.',")
open('t3_meta.js', 'w').write(m)

u = open('ui3.js').read()
def repu(old, new, cnt=1):
    global u
    assert u.count(old) == cnt, (u.count(old), old[:80])
    u = u.replace(old, new)
repu("""    <p class="card-eyebrow">${nb('Как делать и что показывает модель')}</p>""", """    <p class="card-eyebrow">${nb('Подсказки к позе ' + P.num)}</p>""")
repu("""${romBar(c.value, c.rom)}<span class="part-note">${nb('в полной позе ' + fmtDeg(rf.comps[i].value) + ' (' + rf.comps[i].label.toLowerCase() + '), норма до ' + c.rom + '°' + (c.note ? '. ' + c.note.charAt(0).toUpperCase() + c.note.slice(1) : ''))}</span></li>`).join('');""",
     """${romBar(c.value, c.rom)}<span class="part-note">${nb('в полной позе ' + fmtDeg(rf.comps[i].value) + (rf.comps[i].label !== c.label ? ' (' + rf.comps[i].label.toLowerCase() + ')' : '') + ', норма до ' + c.rom + '°' + (c.note ? '. ' + c.note.charAt(0).toUpperCase() + c.note.slice(1) : ''))}</span></li>`).join('');""")
repu("""<span class="num warm-t">${nb(tensionWord(o.s))}</span>""", """<span class="word warm-t">${nb(tensionWord(o.s))}</span>""")
repu("""<span class="num ${s > 0.02 ? 'warm' : 'muted'}-t">${nb(tensionWord(s))}</span>""", """<span class="word ${s > 0.02 ? 'warm' : 'muted'}-t">${nb(tensionWord(s))}</span>""")
open('ui3.js', 'w').write(u)

t = open('template3.html').read()
old = ".card-sub.first { margin-top: 8px; }"
assert t.count(old) == 1
t = t.replace(old, ".card-sub.first { margin-top: 8px; }\n.card-sub + .role { margin-top: 0; }\n.word { font-size: 13.5px; text-align: right; }")
open('template3.html', 'w').write(t)

b = open('poses_build.mjs').read()
old = "  out.extras = { dog: { clavElevNoCue: +m.clavElev.toFixed(2), retractNoCue: +m.retract.toFixed(2) } };"
assert b.count(old) == 1
b = b.replace(old, """  out.extras = { dog: { clavElevNoCue: +m.clavElev.toFixed(2), retractNoCue: +m.retract.toFixed(2) } };
  const h = buildHalf({ scapElev: 0 });
  const mh = measure('half', h.L, placeW(h.L, h.rootQ));
  out.extras.half = { clavElevNoCue: +mh.clavElev.toFixed(2) };
  console.log('half without the shoulders-down cue: clavElev', mh.clavElev.toFixed(1));""")
open('poses_build.mjs', 'w').write(b)
print('text fixes applied')
