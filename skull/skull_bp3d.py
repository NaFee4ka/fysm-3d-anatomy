# Real skull from BodyParts3D (DBCLS, CC BY 4.0): decimate, align to the rig's head, pack for the page.
import numpy as np, trimesh, fast_simplification, json, base64, gzip, glob, re, sys
sys.path.insert(0, './skull')
from bp3d_lib import save_parts
SRC = '/home/claude/skullsrc/skull_obj'
BONES = {  # file id -> (russian, latin)
 'FJ3200': ('Лобная кость', 'os frontale'), 'FJ3274': ('Теменная кость', 'os parietale'), 'FJ3380': ('Теменная кость', 'os parietale'),
 'FJ3309': ('Затылочная кость', 'os occipitale'), 'FJ3281': ('Височная кость', 'os temporale'), 'FJ3386': ('Височная кость', 'os temporale'),
 'FJ3394': ('Клиновидная кость', 'os sphenoidale'), 'FJ3199': ('Решётчатая кость', 'os ethmoidale'),
 'FJ3287': ('Скуловая кость', 'os zygomaticum'), 'FJ3392': ('Скуловая кость', 'os zygomaticum'),
 'FJ3269': ('Верхняя челюсть', 'maxilla'), 'FJ3375': ('Верхняя челюсть', 'maxilla'),
 'FJ3272': ('Носовая кость', 'os nasale'), 'FJ3378': ('Носовая кость', 'os nasale'),
 'FJ3265': ('Слёзная кость', 'os lacrimale'), 'FJ3371': ('Слёзная кость', 'os lacrimale'),
 'FJ3273': ('Нёбная кость', 'os palatinum'), 'FJ3379': ('Нёбная кость', 'os palatinum'),
 'FJ3395': ('Сошник', 'vomer'), 'FJ3263': ('Нижняя носовая раковина', 'concha nasalis inferior'), 'FJ3369': ('Нижняя носовая раковина', 'concha nasalis inferior'),
 'FJ3289': ('Нижняя челюсть', 'mandibula'), 'FJ2772': ('Подъязычная кость', 'os hyoideum'),
}
TARGET = float(sys.argv[1]) if len(sys.argv) > 1 else 0.22   # fraction of triangles kept
def load(fid):
    m = trimesh.load(f'{SRC}/{fid}.obj', force='mesh', process=True)
    v = np.asarray(m.vertices, dtype=np.float64)
    # BodyParts3D: mm, x = left, y = back(-front), z = up  ->  rig: m, x = left, y = up, z = front
    return np.stack([v[:, 0], v[:, 2], -v[:, 1]], 1) * 0.001, np.asarray(m.faces)
parts = {fid: load(fid) for fid in BONES}
# alignment: occipital condyles onto the atlanto-occipital joint, scale from condyles to vertex (crown)
occ = parts['FJ3309'][0]
low = occ[occ[:, 1] < occ[:, 1].min() + 0.006]
cond = low.mean(0)
top = max(p[0][:, 1].max() for p in parts.values())
S = (1.755 - 1.603) / (top - cond[1])
T = np.array([0, 1.603, -0.03]) - cond * S
print('scale', round(S, 4), 'condyles', cond.round(4))
out, tris = [], 0
for fid, (v, f) in parts.items():
    v = v * S + T
    keep = TARGET if len(f) > 3000 else min(1, TARGET * 2.0)
    vv, ff = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_reduction=1 - keep)
    tris += len(ff)
    out.append((fid, vv, ff))

# ---- teeth along the alveolar arches (BodyParts3D has none), then close the jaw ----
UPW = [8.5, 6.5, 7.5, 7.0, 6.5, 10.0, 9.0, 8.5]     # mesiodistal widths, mm: central, lateral, canine, PM1, PM2, M1, M2, M3
LOW = [5.0, 5.5, 7.0, 7.0, 7.0, 11.0, 10.5, 10.0]
def ellipsoid(c, ax, r, sub=2):
    m = trimesh.creation.icosphere(subdivisions=sub)
    m.vertices = m.vertices @ np.stack([ax[0] * r[0], ax[1] * r[1], ax[2] * r[2]], 0) + c
    return m
def tooth(c, i, up, along, out):
    vert = np.array([0, -1.0, 0]) if up else np.array([0, 1.0, 0])     # from the gum towards the bite
    ax = (along, vert, out)
    P = lambda a, v, o: c + along * a + vert * v + out * o
    k = S * 0.001                                                      # mm -> m with the skull scale
    W = (UPW if up else LOW)[i] * 0.5 * k * 0.98
    if i < 2:   # incisor: flat blade with an incisal edge
        return [ellipsoid(P(0, 4.6 * k, 0), ax, [W, 5.4 * k, 2.6 * k])]
    if i == 2:  # canine: long crown with a cusp
        return [ellipsoid(P(0, 4.4 * k, 0), ax, [W, 5.6 * k, 3.6 * k]), ellipsoid(P(0, 8.6 * k, 0), ax, [W * 0.45, 2.0 * k, 1.8 * k], 1)]
    if i < 5:   # premolar: buccal and lingual cusps
        return [ellipsoid(P(0, 3.6 * k, 0), ax, [W, 3.9 * k, 4.2 * k]), ellipsoid(P(0, 6.6 * k, 1.8 * k), ax, [W * 0.6, 1.6 * k, 1.8 * k], 1), ellipsoid(P(0, 6.4 * k, -1.8 * k), ax, [W * 0.6, 1.5 * k, 1.8 * k], 1)]
    parts = [ellipsoid(P(0, 3.4 * k, 0), ax, [W, 3.6 * k, 5.0 * k])]   # molar: four cusps
    for oa, oo in ((0.45, 0.45), (-0.45, 0.45), (0.45, -0.45), (-0.45, -0.45)):
        parts.append(ellipsoid(P(oa * W, 6.2 * k, oo * 4.6 * k), ax, [W * 0.42, 1.5 * k, 2.0 * k], 1))
    return parts
def arch_fit(v, upper):
    # alveolar margin: lowest (upper jaw) / highest (lower jaw) points of the outer band in thin slices across x
    pts = []
    for x0 in np.arange(-0.03, 0.0301, 0.002):
        sl = v[np.abs(v[:, 0] - x0) < 0.0012]
        if len(sl) < 5: continue
        front = sl[sl[:, 2] > sl[:, 2].max() - 0.008]
        pts.append(front[front[:, 1].argmin()] if upper else front[front[:, 1].argmax()])
    pts = np.array(pts)
    X = pts[:, 0]
    cz = np.polyfit(X, pts[:, 2], 2); cy = np.polyfit(X, pts[:, 1], 2)
    return cz, cy
def arch_teeth(cz, cy, up, widths, inset):
    xs = np.linspace(0, 0.034, 400)
    zs, ys = np.polyval(cz, xs), np.polyval(cy, xs)
    seg = np.hypot(np.diff(xs), np.diff(zs)); s_ = np.concatenate([[0], np.cumsum(seg)])
    parts = []
    for side in (1, -1):
        acc = 0
        for i, wmm in enumerate(widths):
            w = wmm * S * 0.001
            mid = acc + w / 2; acc += w
            j = min(len(xs) - 2, np.searchsorted(s_, mid))
            x, z, y = xs[j] * side, zs[j], ys[j]
            t = np.array([(xs[j + 1] - xs[j]) * side, 0, zs[j + 1] - zs[j]]); t /= np.linalg.norm(t)
            o = np.array([-t[2] * side, 0, t[0] * side]) * side   # outward (labial)
            if o[2] < 0 and abs(x) < 0.01: o = -o
            if np.dot(o, [x, 0, z - (zs[0] - 0.05)]) < 0: o = -o
            c = np.array([x, y, z]) - o * inset + np.array([0, -0.0016 if up else 0.0016, 0])
            parts += tooth(c, i, up, t, o)
    return trimesh.util.concatenate(parts)
maxv = np.concatenate([o[1] for o in out if o[0] in ('FJ3269', 'FJ3375')])
mand_i = [k for k, o in enumerate(out) if o[0] == 'FJ3289'][0]
manv = out[mand_i][1]
cz_u, cy_u = arch_fit(maxv, True)
mand_front = manv[manv[:, 2] > manv[:, 2].max() - 0.065]
cz_l, cy_l = arch_fit(mand_front[mand_front[:, 1] > np.percentile(mand_front[:, 1], 40)], False)
up_m = arch_teeth(cz_u, cy_u, True, UPW, 0.0035)
lo_m = arch_teeth(cz_l, cy_l, False, LOW, 0.0032)
# close the jaw: rotate the mandible and lower teeth about the condylar axis until the incisal edges meet
post = manv[manv[:, 2] < manv[:, 2].min() + 0.02]
piv = post[post[:, 1] > post[:, 1].max() - 0.004].mean(0)
def rotX(v, a):
    c, s_ = np.cos(a), np.sin(a); d = v - piv
    return np.stack([d[:, 0], d[:, 1] * c - d[:, 2] * s_, d[:, 1] * s_ + d[:, 2] * c], 1) + piv
fu = up_m.vertices[up_m.vertices[:, 2] > up_m.vertices[:, 2].max() - 0.004]
best, best_gap = 0, 1
for a in np.radians(np.arange(-12, 12.01, 0.1)):
    lv = rotX(lo_m.vertices, a)
    fl = lv[lv[:, 2] > lv[:, 2].max() - 0.004]
    g = fu[:, 1].min() - fl[:, 1].max() + 0.0015      # lower incisal edges 1.5 mm behind/above the upper ones (overbite)
    if abs(g) < abs(best_gap): best, best_gap = a, g
print('jaw rotation', round(np.degrees(best), 2), 'deg (gap residual mm', round(best_gap * 1000, 2), ') pivot', piv.round(3))
out[mand_i] = ('FJ3289', rotX(manv, best).astype(np.float32), out[mand_i][2])
lo_m.vertices = rotX(lo_m.vertices, best)
teeth = trimesh.util.concatenate([up_m, lo_m])
BONES['TEETH'] = ('Зубы', 'dentes')
tv, tf = fast_simplification.simplify(np.asarray(teeth.vertices, np.float32), np.asarray(teeth.faces, np.int32), target_reduction=0.45)
out.append(('TEETH', tv, tf)); tris += len(tf)
print('teeth tris', len(tf))
allv = np.concatenate([o[1] for o in out])
lo, hi = allv.min(0), allv.max(0)
print('tris', tris, 'verts', len(allv), 'bbox', lo.round(3), hi.round(3))
J = np.array([0, 1.595, -0.036])
meshes = []
for fid, v, f in out:
    q = np.round((v - lo) / (hi - lo) * 65535).astype(np.uint16)
    idx = f.astype(np.uint16 if len(v) < 65536 else np.uint32)
    meshes.append({'name': BONES[fid][0], 'latin': BONES[fid][1], 'n': int(len(v)), 'm': int(len(f)), 'q': q, 'i': idx})
def dq(q):
    d = q.astype(np.int32).copy(); d[1:] -= q[:-1].astype(np.int32); return (d & 0xFFFF).astype(np.uint16)
blob = b''.join(dq(m['q'].reshape(-1, 3)).tobytes() + dq(m['i'].reshape(-1).astype(np.uint16)).tobytes() if m['n'] < 65536 else b'' for m in meshes)
gz = gzip.compress(blob, 9)
meta = {'lo': (lo - J).round(6).tolist(), 'hi': (hi - J).round(6).tolist(), 'parts': [{k: m[k] for k in ('name', 'latin', 'n', 'm')} for m in meshes],
        'src': 'BodyParts3D, © The Database Center for Life Science, CC BY 4.0', 'data': base64.b64encode(gz).decode()}
json.dump(meta, open('./skull/skull_bp3d.json', 'w'), ensure_ascii=False)
print('raw KB', len(blob) // 1024, 'gzip KB', len(gz) // 1024, 'base64 KB', len(meta['data']) // 1024)
# reference points after alignment
mand = [o for o in out if o[0] == 'FJ3289'][0][1]
print('chin (lowest front of mandible)', mand[mand[:, 1].argmin()].round(3), 'front-most point', allv[allv[:, 2].argmax()].round(3), 'back-most', allv[allv[:, 2].argmin()].round(3), 'width', (hi[0]-lo[0]).round(3))

save_parts('head', [{'seg': 'head', 'name': BONES[fid][0], 'latin': BONES[fid][1], 'pos': v, 'idx': f} for fid, v, f in out])
