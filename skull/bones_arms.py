# Real shoulder-girdle and arm bones (BodyParts3D, DBCLS, CC BY 4.0) fitted onto the rig segments in the rest pose.
# BodyParts3D stands in the anatomical position (palms forward); the rig's rest arm hangs with the thumb forward,
# so every bone is placed by its own joint landmarks, and the finger bones inherit the roll of the whole hand.
import sys, numpy as np
sys.path.insert(0, './skull')
from bp3d_lib import find, load_many, frame_fit, umeyama, apply, decimate, J, proc, save_parts

def bone(name):
    d = find('^' + name + '$')
    if not d: raise SystemExit('missing ' + name)
    return load_many(d[name])

def ends(v, frac=0.08):
    """proximal (higher) and distal end centres along the principal axis"""
    c = v.mean(0); u = np.linalg.svd(v - c, full_matrices=False)[2][0]
    if u[1] < 0: u = -u                                  # u points up (proximal)
    t = (v - c) @ u; L = t.max() - t.min()
    return v[t > t.max() - frac * L].mean(0), v[t < t.min() + frac * L].mean(0)

def sphere_fit(p):
    A = np.c_[2 * p, np.ones(len(p))]; b = (p ** 2).sum(1)
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    return x[:3], np.sqrt(x[3] + (x[:3] ** 2).sum())

def rot_between(a, b):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b); v = np.cross(a, b); c = np.dot(a, b)
    if np.linalg.norm(v) < 1e-9: return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1 / (1 + c))

def lathe_ab(seg, name):
    b = proc(seg, name)[0]; return np.array(b['a']), np.array(b['b'])

KEEP = {'big': 0.7, 'scap': 0.13}
parts, report = [], []

def add(seg, ru, la, v, f, keep=1.0):
    vv, ff = decimate(v, f, keep)
    parts.append({'seg': seg, 'name': ru, 'latin': la, 'pos': vv, 'idx': ff})

for side, S, sx in (('left', 'L', 1.0), ('right', 'R', -1.0)):
    lat = np.array([sx, 0, 0])            # lateral direction for this side
    # ---------- clavicle: sternal end -> clav joint, acromial end -> scap joint, superior side up ----------
    v, f = bone(f'{side} clavicle')
    med = v[np.argsort(sx * v[:, 0])[:30]].mean(0); latp = v[np.argsort(-sx * v[:, 0])[:30]].mean(0)
    s, R, t = frame_fit(med, latp, [0, 1, 0], J(f'clav_{S}'), J(f'scap_{S}') + np.array([-0.012 * sx, 0, 0.004]), [0, 1, 0])
    add(f'clav_{S}', 'Ключица', 'clavicula', apply(v, s, R, t), f)
    report.append((S, 'clavicle', round(s, 3)))
    # ---------- scapula: landmarks to the schematic scapula ----------
    v, f = bone(f'{side} scapula')
    inf = v[v[:, 1].argmin()]                                             # inferior angle
    medial = v[sx * v[:, 0] < (sx * v[:, 0]).min() + 0.025]
    sup = medial[medial[:, 1].argmax()]                                   # superior angle
    acr = v[(sx * v[:, 0] + v[:, 1]).argmax()]                            # acromion tip (lateral-superior)
    hi = v[v[:, 1] < acr[1] - 0.02]
    glen_band = hi[sx * hi[:, 0] > (sx * hi[:, 0]).max() - 0.006]
    glen = glen_band.mean(0)                                              # glenoid (most lateral below the acromion)
    pl = proc(f'scap_{S}', 'Лопатка')[0]['pts']; pl = np.array(pl)
    tgt_inf = pl[np.argmin(pl[:, 1])]; tgt_sup = np.array(pl[0])
    tgt_acr = np.array(proc(f'scap_{S}', 'Акромион')[0]['c']) + np.array([0.012 * sx, 0.004, 0.0])
    tgt_glen = J(f'hum_{S}') - lat * 0.026
    src = np.array([inf, sup, acr, glen]); dst = np.array([tgt_inf, tgt_sup, tgt_acr, tgt_glen])
    s, R, t = umeyama(src, dst)
    err = np.linalg.norm(apply(src, s, R, t) - dst, axis=1)
    add(f'scap_{S}', 'Лопатка', 'scapula', apply(v, s, R, t), f, KEEP['scap'])
    report.append((S, 'scapula', round(s, 3), err.round(3).tolist()))
    # ---------- humerus: head centre -> shoulder joint, elbow centre -> elbow joint, epicondyles across x ----------
    v, f = bone(f'{side} humerus')
    top = v[v[:, 1] > v[:, 1].max() - 0.04]
    head_pts = top[sx * top[:, 0] < np.median(sx * top[:, 0])]            # medial half of the proximal end
    hc, hr = sphere_fit(head_pts)
    dist = v[v[:, 1] < v[:, 1].min() + 0.04]
    epiL = dist[(sx * dist[:, 0]).argmax()]; epiM = dist[(sx * dist[:, 0]).argmin()]
    elbow = (epiL + epiM) / 2 + np.array([0, -0.006, 0.004])
    tgt_el = J(f'fore_{S}') + np.array([0, 0.004, 0.0])
    s, R, t = frame_fit(hc, elbow, (epiL - epiM), J(f'hum_{S}'), tgt_el, lat)
    add(f'hum_{S}', 'Плечевая кость', 'humerus', apply(v, s, R, t), f, KEEP['big'])
    report.append((S, 'humerus', round(s, 3), 'head r', round(hr * s, 4)))
    # ---------- radius and ulna: onto the schematic forearm bones; radius lies on the thumb side ----------
    vr, fr_ = bone(f'{side} radius'); vu, fu = bone(f'{side} ulna')
    r0, r1 = ends(vr); u0, u1 = ends(vu)
    ra, rb = lathe_ab(f'fore_{S}', 'Лучевая кость'); ua, ub = lathe_ab(f'ulna_{S}', 'Локтевая кость')
    s, R, t = frame_fit(r0, r1, (u0 + u1) / 2 - (r0 + r1) / 2, ra + np.array([0, 0.006, 0]), rb, (ua + ub) / 2 - (ra + rb) / 2)
    add(f'fore_{S}', 'Лучевая кость', 'radius', apply(vr, s, R, t), fr_)
    s, R, t = frame_fit(u0, u1, (r0 + r1) / 2 - (u0 + u1) / 2, ua, ub, (ra + rb) / 2 - (ua + ub) / 2)
    add(f'ulna_{S}', 'Локтевая кость', 'ulna', apply(vu, s, R, t), fu)
    # ---------- hand: carpals + metacarpals by one similarity fit ----------
    CARP = [('scaphoid', 'Ладьевидная кость', 'os scaphoideum'), ('lunate', 'Полулунная кость', 'os lunatum'), ('triquetral', 'Трёхгранная кость', 'os triquetrum'),
            ('pisiform', 'Гороховидная кость', 'os pisiforme'), ('trapezium', 'Кость-трапеция', 'os trapezium'), ('trapezoid', 'Трапециевидная кость', 'os trapezoideum'),
            ('capitate', 'Головчатая кость', 'os capitatum'), ('hamate', 'Крючковидная кость', 'os hamatum')]
    MC = [('first', 'Первая пястная кость', 'os metacarpale I'), ('second', 'Вторая пястная кость', 'os metacarpale II'), ('third', 'Третья пястная кость', 'os metacarpale III'),
          ('fourth', 'Четвёртая пястная кость', 'os metacarpale IV'), ('fifth', 'Пятая пястная кость', 'os metacarpale V')]
    carp = {k: bone(f'{side} {k}') for k, _, _ in CARP}
    mc = {k: bone(f'{side} {k} metacarpal bone') for k, _, _ in MC}
    src, dst = [], []
    pa = [p for p in proc(f'hand_{S}', 'Пястные кости')]
    for i, k in enumerate(['second', 'third', 'fourth', 'fifth']):
        b0, b1 = ends(mc[k][0])
        src += [b0, b1]; dst += [np.array(pa[i]['a']), J(f'f{i + 2}a_{S}')]
    for k in ('pisiform', 'trapezium', 'lunate'):
        c = np.array(proc(f'hand_{S}', dict((a, b) for a, b, _ in CARP)[k])[0]['c'])
        src.append(carp[k][0].mean(0)); dst.append(c)
    src, dst = np.array(src), np.array(dst)
    sH, RH, tH = umeyama(src, dst)
    errH = np.linalg.norm(apply(src, sH, RH, tH) - dst, axis=1)
    report.append((S, 'hand', round(sH, 3), errH.round(3).tolist()))
    for k, ru, la in CARP: add(f'hand_{S}', ru, la, apply(carp[k][0], sH, RH, tH), carp[k][1])
    for i, (k, ru, la) in enumerate(MC):
        v = apply(mc[k][0], sH, RH, tH)
        if k == 'first':   # thumb metacarpal is its own segment: base -> f1m joint, head -> f1a joint
            b0, b1 = ends(v); seg = f'f1m_{S}'; a, b = J(seg), J(f'f1a_{S}')
        else:
            b0, b1 = ends(v); seg = f'hand_{S}'; a, b = np.array(pa[i - 1]['a']), J(f'f{i + 1}a_{S}')
        Rm = rot_between(b1 - b0, b - a); sc = np.linalg.norm(b - a) / np.linalg.norm(b1 - b0)
        v = (sc * (Rm @ (v - b0).T)).T + a
        add(seg, ru, la, v, mc[k][1])
    # ---------- phalanges: hand roll, axis onto the segment, base on the joint ----------
    FING = [('thumb', 1, ['proximal', 'distal'], ['a', 'b']), ('index finger', 2, ['proximal', 'middle', 'distal'], ['a', 'b', 'c']),
            ('middle finger', 3, ['proximal', 'middle', 'distal'], ['a', 'b', 'c']), ('ring finger', 4, ['proximal', 'middle', 'distal'], ['a', 'b', 'c']),
            ('little finger', 5, ['proximal', 'middle', 'distal'], ['a', 'b', 'c'])]
    RU = {'proximal': 'Проксимальная фаланга', 'middle': 'Средняя фаланга', 'distal': 'Дистальная фаланга'}
    LA = {'proximal': 'phalanx proximalis', 'middle': 'phalanx media', 'distal': 'phalanx distalis'}
    FN = {1: 'большого пальца', 2: 'указательного пальца', 3: 'среднего пальца', 4: 'безымянного пальца', 5: 'мизинца'}
    for fname, n, kinds, segs in FING:
        for j, (kind, sg) in enumerate(zip(kinds, segs)):
            v, f = bone(f'{kind} phalanx of {side} {fname}')
            v = apply(v, sH, RH, tH)
            b0, b1 = ends(v, 0.1)
            seg = f'f{n}{sg}_{S}'
            a = J(seg)
            if j + 1 < len(segs): b = J(f'f{n}{segs[j + 1]}_{S}')
            else: b = np.array(proc(seg)[0]['b'])
            Rm = rot_between(b1 - b0, b - a); sc = np.linalg.norm(b - a) / np.linalg.norm(b1 - b0)
            v = (sc * (Rm @ (v - b0).T)).T + a
            add(seg, RU[kind] + ' ' + FN[n], LA[kind] + (' pollicis' if n == 1 else ' digiti ' + ['', '', 'II', 'III', 'IV', 'V'][n] + ' manus'), v, f)

for r in report: print(*r)
save_parts('arms', parts)
