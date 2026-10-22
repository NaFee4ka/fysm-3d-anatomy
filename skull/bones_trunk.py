# Real trunk bones from BodyParts3D (DBCLS, CC BY 4.0): pelvis, spine, ribs, costal cartilages, sternum.
# One similarity transform per rig segment (vertebral bodies, spinous and transverse tips onto the schematic
# vertebrae of that segment); ribs, cartilages and sternum ride with the transform of their thoracic segment.
import sys, re, numpy as np
sys.path.insert(0, './skull')
from bp3d_lib import find, load, load_many, umeyama, apply, decimate, J, proc, save_parts, RIG

ORD = ['first', 'second', 'third', 'fourth', 'fifth', 'sixth', 'seventh', 'eighth', 'ninth', 'tenth', 'eleventh', 'twelfth']
RU_ORD = ['первое', 'второе', 'третье', 'четвёртое', 'пятое', 'шестое', 'седьмое', 'восьмое', 'девятое', 'десятое', 'одиннадцатое', 'двенадцатое']
def fj(name):
    r = find('^' + re.escape(name) + '$')
    assert r, name
    return list(r.values())[0]

# ---- vertebrae: (BodyParts3D concept, rig name, segment, latin) ----
VERT = []
for i in range(5):
    VERT.append((f'{ORD[i]} lumbar vertebra', f'Поясничный позвонок L{i + 1}', 'lumA' if i >= 3 else 'lumB', f'vertebra lumbalis L{i + 1}'))
for i in range(12):
    seg = 'thA' if i >= 8 else 'thB' if i >= 4 else 'thC'
    VERT.append((f'{ORD[i]} thoracic vertebra', f'Грудной позвонок Th{i + 1}', seg, f'vertebra thoracica Th{i + 1}'))
for i in range(2, 7):
    VERT.append((f'{ORD[i]} cervical vertebra', f'Шейный позвонок C{i + 1}', 'neckA' if i >= 4 else 'neckB', f'vertebra cervicalis C{i + 1}'))
VERT.append(('axis', 'Шейный позвонок C2 (осевой)', 'neckB', 'axis, vertebra cervicalis C2'))
VERT.append(('atlas', 'Шейный позвонок C1 (атлант)', 'neckB', 'atlas, vertebra cervicalis C1'))

def rig_vert(name):
    """schematic vertebra: body centre, spinous tip, left/right transverse tips"""
    key = re.sub(r' \(.*\)', '', name)
    bs = [b for b in RIG['bones'] if b['name'] == key and b['kind'] == 'lathe']
    body = next(b for b in bs if abs(b['a'][0]) < 1e-6 and abs(b['b'][1] - b['a'][1]) > abs(b['b'][2] - b['a'][2]))
    spin = next(b for b in bs if abs(b['a'][0]) < 1e-6 and b is not body)
    tl = next(b for b in bs if b['b'][0] > 0.01); tr = next(b for b in bs if b['b'][0] < -0.01)
    c = (np.array(body['a']) + np.array(body['b'])) / 2
    return c, np.array(spin['b']), np.array(tl['b']), np.array(tr['b'])

def bp_vert(v, region):
    """BodyParts3D vertebra landmarks in the same order"""
    z0, z1 = v[:, 2].min(), v[:, 2].max()
    frac, w = {'lumbar': (0.36, 0.016), 'thoracic': (0.33, 0.012), 'cervical': (0.32, 0.008)}[region]
    sel = v[(v[:, 2] > z1 - frac * (z1 - z0)) & (np.abs(v[:, 0] - v[:, 0].mean()) < w)]
    c = sel.mean(0)
    mid = v[np.abs(v[:, 0] - v[:, 0].mean()) < 0.008]
    spin = mid[mid[:, 2].argmin()]
    return c, spin, v[v[:, 0].argmax()], v[v[:, 0].argmin()]

# landmarks per vertebra; fits are in the sagittal plane (BodyParts3D is already upright and symmetric):
# one spine-wide scale from the L5..C3 body-centre path length, then per segment a rotation about x + shift
meshes, LM = {}, {}
for concept, name, seg, lat in VERT:
    v, f = load_many(fj(concept))
    meshes[name] = (v, f, seg, lat)
    if concept == 'atlas': continue
    region = 'lumbar' if 'lumbar' in concept else 'thoracic' if 'thoracic' in concept else 'cervical'
    LM[name] = (bp_vert(v, region), rig_vert(name), seg)
chain = [n for n in [f'Поясничный позвонок L{i}' for i in range(5, 0, -1)] + [f'Грудной позвонок Th{i}' for i in range(12, 0, -1)] + [f'Шейный позвонок C{i}' for i in range(7, 2, -1)]]
pl = lambda P: sum(np.linalg.norm(P[i + 1] - P[i]) for i in range(len(P) - 1))
s_sp = pl([LM[n][1][0] for n in chain]) / pl([LM[n][0][0] for n in chain])
print('spine scale', round(s_sp, 3))
def fit2d(src, dst, w):
    src, dst, w = np.array(src)[:, 1:], np.array(dst)[:, 1:], np.array(w, float)
    ms, md = (src * w[:, None]).sum(0) / w.sum(), (dst * w[:, None]).sum(0) / w.sum()
    A, B = (src - ms) * s_sp, dst - md
    num = (w * (A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0])).sum(); den = (w * (A * B).sum(1)).sum()
    th = np.arctan2(num, den)       # rotation in the (y, z) plane
    c, sn = np.cos(th), np.sin(th)
    R = np.array([[1, 0, 0], [0, c, -sn], [0, sn, c]])
    m3s = np.r_[0, ms]; m3d = np.r_[0, md]
    t = m3d - s_sp * R @ m3s
    return s_sp, R, t, np.degrees(th)
T, segpts = {}, {}
for n, (src, dst, seg) in LM.items():
    sp = segpts.setdefault(seg, ([], [], []))
    for k, w in zip(range(4), (4, 1, 0, 0)):
        sp[0].append(src[k]); sp[1].append(dst[k]); sp[2].append(w)
for seg, (src, dst, w) in segpts.items():
    s, R, t, th = fit2d(src, dst, w)
    T[seg] = (s, R, t)
    e = [np.linalg.norm(apply([a], s, R, t)[0] - b) for a, b, ww in zip(src, dst, w) if ww == 4]
    print(f'{seg:6s} pitch {th:6.1f}° body-centre error mean {np.mean(e) * 1000:.1f} max {np.max(e) * 1000:.1f} mm')
for n, (src, dst, seg) in LM.items():
    s, R, t = T[seg]
    print('   ', n.ljust(24), 'body', (1000 * (apply([src[0]], s, R, t)[0] - dst[0])).round(1), 'spin', (1000 * (apply([src[1]], s, R, t)[0] - dst[1])).round(1))

# ---- pelvis: hip bones + sacrum, fitted on hip centres, S1 endplate, ASIS and pubic symphysis ----
hipL, fhL = load_many(fj('left hip bone')); hipR, fhR = load_many(fj('right hip bone'))
sac, fsac = load_many(fj('sacrum'))
def femhead(concept):
    v, _ = load_many(fj(concept))
    top = v[v[:, 1] > v[:, 1].max() - 0.05]
    side = np.sign(top[:, 0].mean())
    head = top[np.abs(top[:, 0]) < np.abs(top[:, 0]).min() + 0.045] if True else top
    # sphere fit to the medial-upper cap
    A_ = np.c_[2 * head, np.ones(len(head))]; b_ = (head ** 2).sum(1)
    sol = np.linalg.lstsq(A_, b_, rcond=None)[0]
    return sol[:3]
hcL, hcR = femhead('left femur'), femhead('right femur')
s1 = sac[sac[:, 1] > sac[:, 1].max() - 0.012]
s1 = s1[np.abs(s1[:, 0]) < 0.02].mean(0)
asisL = hipL[hipL[:, 2].argmax()] if False else None
def asis(v):
    upper = v[v[:, 1] > v[:, 1].max() - 0.06]
    return upper[upper[:, 2].argmax()]
sym = np.concatenate([hipL[np.abs(hipL[:, 0]) < 0.006], hipR[np.abs(hipR[:, 0]) < 0.006]]).mean(0)
src = [hcL, hcR, s1, asis(hipL), asis(hipR), sym]
dst = [J('thigh_L'), J('thigh_R'), J('lumA') + np.array([0, -0.004, -0.002]), np.array([0.125, 1.035, 0.058]), np.array([-0.125, 1.035, 0.058]), np.array([0, 0.886, 0.07])]
Wp = [4, 4, 3, 0, 0, 0]
S_, D_ = [], []
for a, b, w in zip(src, dst, Wp): S_ += [a] * w; D_ += [b] * w
s, R, t = umeyama(np.array(S_), np.array(D_))
err = np.linalg.norm(apply(np.array(src), s, R, t) - np.array(dst), axis=1)
T['pelvis'] = (s, R, t)
print('pelvis scale %.3f; errors mm: hips %.1f %.1f, S1 %.1f, ASIS %.1f %.1f, symphysis %.1f' % ((s,) + tuple(err * 1000)))

# ---- assemble parts ----
parts = []
def add(v, f, seg, name, lat, keep):
    s, R, t = T[seg]
    vv, ff = decimate(apply(v, s, R, t), f, keep)
    parts.append({'seg': seg, 'name': name, 'latin': lat, 'pos': vv, 'idx': ff})
KEEP_V = 0.16
# each vertebra: its segment's transform, then shifted so that its body centre sits exactly on the rig's
# (the atlas follows the axis)
SHIFT = {}
for n, (src, dst, seg) in LM.items():
    s, R, t = T[seg]
    SHIFT[n] = dst[0] - apply([src[0]], s, R, t)[0]
SHIFT['Шейный позвонок C1 (атлант)'] = SHIFT['Шейный позвонок C2 (осевой)']
for name, (v, f, seg, lat) in meshes.items():
    s, R, t = T[seg]
    vv, ff = decimate(apply(v, s, R, t) + SHIFT[name], f, KEEP_V)
    parts.append({'seg': seg, 'name': name, 'latin': lat, 'pos': vv, 'idx': ff})
add(hipL, fhL, 'pelvis', 'Тазовая кость', 'os coxae', 0.12)
add(hipR, fhR, 'pelvis', 'Тазовая кость', 'os coxae', 0.12)
add(sac, fsac, 'pelvis', 'Крестец', 'os sacrum', 0.14)
for i in range(12):
    seg = 'thA' if i >= 8 else 'thB' if i >= 4 else 'thC'
    for side, sw in (('left', 'L'), ('right', 'R')):
        v, f = load_many(fj(f'{side} {ORD[i]} rib'))
        add(v, f, seg, f'Ребро {i + 1}', f'costa {["I","II","III","IV","V","VI","VII","VIII","IX","X","XI","XII"][i]}', 0.12)
        if i < 7:
            v, f = load_many(fj(f'{side} {ORD[i]} costal cartilage'))
            add(v, f, seg, f'Рёберный хрящ {i + 1}', 'cartilago costalis', 0.25)
v, f = load_many(fj('manubrium')); add(v, f, 'thC', 'Рукоятка грудины', 'manubrium sterni', 0.2)
v, f = load_many(fj('body of sternum')); add(v, f, 'thB', 'Тело грудины', 'corpus sterni', 0.2)
v, f = load_many(fj('xiphoid process')); add(v, f, 'thB', 'Мечевидный отросток', 'processus xiphoideus', 0.4)

# continuity: gap between consecutive vertebral bodies across segment borders (rest pose)
def bodyc(name):
    p = next(p for p in parts if p['name'] == name)
    return p['pos']
order = [f'Поясничный позвонок L{i}' for i in range(5, 0, -1)] + [f'Грудной позвонок Th{i}' for i in range(12, 0, -1)] + [f'Шейный позвонок C{i}' for i in range(7, 2, -1)]
for a, b in zip(order[:-1], order[1:]):
    pa, pb = bodyc(a), bodyc(b)
    sa = next(p['seg'] for p in parts if p['name'] == a); sb = next(p['seg'] for p in parts if p['name'] == b)
    if sa != sb:
        # vertical gap between top of a's body and bottom of b's body near the midline front
        fa = pa[(np.abs(pa[:, 0]) < 0.01)]; fb = pb[(np.abs(pb[:, 0]) < 0.01)]
        za = fa[:, 2].max(); zb = fb[:, 2].max()
        ta = fa[fa[:, 2] > za - 0.015][:, 1].max(); bb = fb[fb[:, 2] > zb - 0.015][:, 1].min()
        print(f'border {a} ({sa}) -> {b} ({sb}): disc space {1000 * (bb - ta):.1f} mm, front offset {1000 * (zb - za):.1f} mm')
save_parts('trunk', parts)
