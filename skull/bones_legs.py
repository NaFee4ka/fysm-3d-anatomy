# Real leg bones from BodyParts3D (DBCLS, CC BY 4.0) fitted to the rig segments: thigh, shank (tibia, fibula,
# patella), foot (tarsals, metatarsals), toes (phalanges). Writes skull/parts_legs.{npz,json}.
import numpy as np, sys
sys.path.insert(0, './skull')
from bp3d_lib import find, load_many, frame_fit, umeyama, apply, decimate, J, save_parts

def bone(name):
    hits = find('^' + name + '$')
    assert name in hits, name
    return load_many(hits[name])

def axial(v, s, R, t, a0, a1, thick=0.98):
    # rotate/translate like frame_fit, but stretch only along the bone axis (thickness keeps its natural size)
    a0 = np.asarray(a0); e = (np.asarray(a1) - a0); L = np.linalg.norm(e); e /= L
    d = np.asarray(v) - a0
    along = d @ e
    d2 = d * thick + np.outer(along, e) * (s - thick)
    return (R @ d2.T).T + (R @ a0) * s + t
def keep_for(f): return 1.0 if len(f) < 700 else 0.72
def sphere_fit(P):
    A = np.c_[2 * P, np.ones(len(P))]; b = (P ** 2).sum(1)
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    return c[:3], np.sqrt(c[3] + (c[:3] ** 2).sum())

RU = {
    'femur': ('Бедренная кость', 'femur'), 'tibia': ('Большеберцовая кость', 'tibia'), 'fibula': ('Малоберцовая кость', 'fibula'),
    'patella': ('Надколенник', 'patella'), 'talus': ('Таранная кость', 'talus'), 'calcaneus': ('Пяточная кость', 'calcaneus'),
    'navicular': ('Ладьевидная кость стопы', 'os naviculare'), 'cuboid': ('Кубовидная кость', 'os cuboideum'),
    'medial cuneiform': ('Медиальная клиновидная кость', 'os cuneiforme mediale'),
    'intermediate cuneiform': ('Промежуточная клиновидная кость', 'os cuneiforme intermedium'),
    'lateral cuneiform': ('Латеральная клиновидная кость', 'os cuneiforme laterale'),
}
MT = ['first', 'second', 'third', 'fourth', 'fifth']
TOE = ['big toe', 'second toe', 'third toe', 'fourth toe', 'little toe']
TOE_RU = ['большого пальца стопы', 'II пальца стопы', 'III пальца стопы', 'IV пальца стопы', 'мизинца стопы']
PH_RU = {'proximal': 'Проксимальная фаланга', 'middle': 'Средняя фаланга', 'distal': 'Дистальная фаланга'}
PH_LAT = {'proximal': 'phalanx proximalis', 'middle': 'phalanx media', 'distal': 'phalanx distalis'}

parts = []
for side, sd, lat in (('left', 'L', 1.0), ('right', 'R', -1.0)):
    # ---------- thigh: femoral head centre -> hip joint, condyle centres -> knee joint ----------
    fv, ff = bone(f'{side} femur')
    top = fv[:, 1].max()
    prox = fv[fv[:, 1] > top - 0.07]
    medial = prox[np.argsort(prox[:, 0] * lat)[:max(30, len(prox) // 6)]]       # most medial proximal points = head
    seed = medial.mean(0)
    hp = fv[np.linalg.norm(fv - seed, axis=1) < 0.03]
    H, hr = sphere_fit(hp)
    for _ in range(3):
        hp = fv[np.abs(np.linalg.norm(fv - H, axis=1) - hr) < 0.004]; H, hr = sphere_fit(hp)
    dist = fv[fv[:, 1] < fv[:, 1].min() + 0.03]
    mx = np.median(dist[:, 0])
    cm = dist[dist[:, 0] * lat < mx * lat]; cl = dist[dist[:, 0] * lat >= mx * lat]   # medial, lateral condyle
    def condyle(c):
        low = c[c[:, 1] < c[:, 1].min() + 0.012]; p = low.mean(0); p[1] = c[:, 1].min() + 0.021; return p
    Cm, Cl = condyle(cm), condyle(cl)
    K = (Cm + Cl) / 2
    s, R, t = frame_fit(H, K, (Cl - Cm), J(f'thigh_{sd}'), J(f'shank_{sd}'), np.array([lat, 0, 0]))
    print(sd, 'femur head r %.1f mm, length %.1f mm -> scale %.3f' % (hr * 1000, np.linalg.norm(K - H) * 1000, s))
    v, f = decimate(axial(fv, s, R, t, H, K), ff, keep_for(ff))
    parts.append({'seg': f'thigh_{sd}', 'name': RU['femur'][0], 'latin': RU['femur'][1], 'pos': v, 'idx': f})
    Kmap = apply([K], s, R, t)[0]
    # ---------- shank: tibia, fibula, patella; knee centre (femoral, in BodyParts3D) -> knee joint, plafond -> ankle ----------
    tv, tf = bone(f'{side} tibia')
    low = tv[tv[:, 1] < tv[:, 1].min() + 0.02]
    # plafond: lowest points away from the medial malleolus (which is the most medial low part)
    cx = np.percentile(low[:, 0] * lat, 60)
    plaf = low[(low[:, 0] * lat > cx - 0.012)]
    plaf = plaf[plaf[:, 1] < plaf[:, 1].min() + 0.006]
    Pl = plaf.mean(0)
    fbv, fbf = bone(f'{side} fibula')
    latref = fbv.mean(0) - tv.mean(0); latref[1] = 0
    s2, R2, t2 = frame_fit(K, Pl, latref, J(f'shank_{sd}'), J(f'foot_{sd}'), np.array([lat, 0, 0]))
    print(sd, 'shank length %.1f mm -> scale %.3f' % (np.linalg.norm(Pl - K) * 1000, s2))
    for key, (v0, f0) in (('tibia', (tv, tf)), ('fibula', (fbv, fbf)), ('patella', bone(f'{side} patella'))):
        v, f = decimate(axial(v0, s2, R2, t2, K, Pl), f0, keep_for(f0))
        parts.append({'seg': f'shank_{sd}', 'name': RU[key][0], 'latin': RU[key][1], 'pos': v, 'idx': f})
    # ---------- foot: similarity fit of talus dome, heel, first and fifth metatarsal heads ----------
    tal_v, tal_f = bone(f'{side} talus')
    dome = tal_v[tal_v[:, 1] > tal_v[:, 1].max() - 0.004].mean(0)
    cal_v, cal_f = bone(f'{side} calcaneus')
    heel = cal_v[cal_v[:, 1] < cal_v[:, 1].min() + 0.003]; heel = heel[heel[:, 2] < np.percentile(heel[:, 2], 50)].mean(0)
    mt = [bone(f'{side} {o} metatarsal bone') for o in MT]
    def head_bottom(v):
        front = v[v[:, 2] > v[:, 2].max() - 0.012]; return front[front[:, 1] < front[:, 1].min() + 0.003].mean(0)
    m1, m5 = head_bottom(mt[0][0]), head_bottom(mt[4][0])
    jf = J(f'foot_{sd}')
    # centres: talus, calcaneus and the five metatarsal heads against the rig's own foot bones
    def head_c(v): return v[v[:, 2] > v[:, 2].max() - 0.010].mean(0)
    src = np.array([tal_v.mean(0), cal_v.mean(0)] + [head_c(m[0]) for m in mt])
    tgt = np.array([[0.095, 0.058, -0.004], [0.094, 0.033, -0.035], [0.074, 0.02, 0.135], [0.087, 0.018, 0.142], [0.1, 0.017, 0.137], [0.113, 0.016, 0.128], [0.126, 0.015, 0.118]])
    tgt[:, 0] *= lat
    W = [2, 2, 3, 1, 1, 1, 3]
    # length from the skin landmarks: heel back -0.064, second-toe tip 0.19 -> about 0.24 m of bone
    d2 = bone(f'distal phalanx of {side} second toe')[0]
    Lb = d2[:, 2].max() - cal_v[:, 2].min()
    k = 0.24 / Lb
    _, R3, t3 = umeyama(np.repeat(src * k, W, 0), np.repeat(tgt, W, 0), with_scale=False)
    s3, t3 = k * 1.0, t3
    err = np.linalg.norm(apply(src, s3, R3, t3) - tgt, axis=1) * 1000
    # sole: lowest bone of heel and forefoot about 9 mm above the floor (skin and fat pad below)
    lowest = min(apply(cal_v, s3, R3, t3)[:, 1].min(), min(apply(m[0], s3, R3, t3)[:, 1].min() for m in mt))
    t3 = t3 + np.array([0, 0.009 - lowest, 0])
    print(sd, 'foot fit scale %.3f, errors mm' % s3, err.round(1))
    foot_bones = [('talus', (tal_v, tal_f)), ('calcaneus', (cal_v, cal_f)), ('navicular', bone(f'navicular bone of {side} foot')),
                  ('cuboid', bone(f'{side} cuboid bone'))] + [(c, bone(f'{side} {c} bone')) for c in ('medial cuneiform', 'intermediate cuneiform', 'lateral cuneiform')]
    for key, (v0, f0) in foot_bones:
        v, f = decimate(apply(v0, s3, R3, t3), f0, keep_for(f0))
        parts.append({'seg': f'foot_{sd}', 'name': RU[key][0], 'latin': RU[key][1], 'pos': v, 'idx': f})
    for i, (v0, f0) in enumerate(mt):
        v, f = decimate(apply(v0, s3, R3, t3), f0, keep_for(f0))
        parts.append({'seg': f'foot_{sd}', 'name': f'{["I", "II", "III", "IV", "V"][i]} плюсневая кость', 'latin': 'os metatarsi ' + ['I', 'II', 'III', 'IV', 'V'][i], 'pos': v, 'idx': f})
    # ---------- toes: phalanges with the foot's transform (rest pose), on the toes segment ----------
    for i, toe in enumerate(TOE):
        for ph in ('proximal', 'middle', 'distal'):
            nm = f'{ph} phalanx of {side} {toe}'
            if not find('^' + nm + '$'): continue
            v0, f0 = bone(nm)
            v, f = decimate(apply(v0, s3, R3, t3), f0, keep_for(f0))
            parts.append({'seg': f'toes_{sd}', 'name': f'{PH_RU[ph]} {TOE_RU[i]}', 'latin': PH_LAT[ph], 'pos': v, 'idx': f})

# report: lowest points of the foot bones in the rest pose (sole should be ~1 cm above the floor)
for sd in 'LR':
    fb = np.concatenate([p['pos'] for p in parts if p['seg'] in (f'foot_{sd}', f'toes_{sd}')])
    print(sd, 'foot bones lowest y %.1f mm, z range %.3f..%.3f' % (fb[:, 1].min() * 1000, fb[:, 2].min(), fb[:, 2].max()))
save_parts('legs', parts)
