"""Shared helpers for real bones from BodyParts3D (DBCLS, CC BY 4.0).
Rig axes: metres, x = body's left, y = up, z = forward. BodyParts3D: mm, x = left, y = backward(-front), z = up.
Each region script writes skull/parts_<region>.npz + skull/parts_<region>.json via save_parts();
skull/pack_bones.py packs every region into skull/bones_bp3d.json for the page (build.py inlines it)."""
import numpy as np, trimesh, fast_simplification, json, zipfile, re, io, os
ROOT = './skull'
SRC = '/home/claude/skullsrc'
ZIP = f'{SRC}/partof_BP3D_4.0_obj_99.zip'
RIG = json.load(open(f'{ROOT}/rig_rest.json'))   # segs: {name: {J: rest joint position, parent}}, bones: procedural bones (seg, kind, name, c/r/a/b/pts)
_el = {}
for line in open(f'{SRC}/partof_element_parts.txt', encoding='utf-8').read().splitlines()[1:]:
    fma, name, fj = line.split('\t'); _el.setdefault(name, []).append(fj)
def find(pattern):
    """concept names matching a regex -> {name: [FJ ids]} (e.g. find(r'^left femur$'))"""
    rx = re.compile(pattern, re.I)
    return {n: v for n, v in _el.items() if rx.search(n)}
_zf = None
def load(fj):
    """one BodyParts3D element -> (vertices in rig axes, metres, BodyParts3D placement), faces"""
    global _zf
    _zf = _zf or zipfile.ZipFile(ZIP)
    data = _zf.read(f'partof_BP3D_4.0_obj_99/{fj}.obj')
    m = trimesh.load(io.BytesIO(data), file_type='obj', force='mesh', process=True)
    v = np.asarray(m.vertices, float)
    return np.stack([v[:, 0], v[:, 2], -v[:, 1]], 1) * 0.001, np.asarray(m.faces)
def load_many(fjs):
    vs, fs, n = [], [], 0
    for fj in fjs:
        v, f = load(fj); vs.append(v); fs.append(f + n); n += len(v)
    return np.concatenate(vs), np.concatenate(fs)
def umeyama(src, dst, with_scale=True):
    """similarity transform (s, R, t) minimising |s R src + t - dst|; apply with apply(v, s, R, t)"""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(0), dst.mean(0)
    A, B = src - ms, dst - md
    U, D, Vt = np.linalg.svd(B.T @ A / len(src))
    Sg = np.eye(3); Sg[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ Sg @ Vt
    s = (D * np.diag(Sg)).sum() / (A ** 2).sum() * len(src) if with_scale else 1.0
    return s, R, md - s * R @ ms
def apply(v, s, R, t): return (s * (R @ np.asarray(v, float).T)).T + t
def frame_fit(a0, a1, ref0, b0, b1, ref1, scale=True):
    """map segment axis a0->a1 (with a side reference direction ref0) onto b0->b1 (ref1): rotation from the two
    orthonormal frames, uniform scale = |b1-b0| / |a1-a0|, a0 lands on b0"""
    def fr(p, q, r):
        e1 = (q - p) / np.linalg.norm(q - p); e2 = r - e1 * np.dot(r, e1); e2 /= np.linalg.norm(e2); return np.stack([e1, e2, np.cross(e1, e2)], 1)
    a0, a1, b0, b1 = map(np.asarray, (a0, a1, b0, b1))
    R = fr(b0, b1, np.asarray(ref1, float)) @ fr(a0, a1, np.asarray(ref0, float)).T
    s = np.linalg.norm(b1 - b0) / np.linalg.norm(a1 - a0) if scale else 1.0
    return s, R, b0 - s * R @ a0
def decimate(v, f, keep):
    if keep >= 0.999 or len(f) < 200: return v.astype(np.float32), f.astype(np.int32)
    vv, ff = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_reduction=1 - keep)
    return vv, ff
def J(seg): return np.array(RIG['segs'][seg]['J'])
def proc(seg, name=None):
    """procedural bones of a segment (their points show where the rig expects the bone)"""
    return [b for b in RIG['bones'] if b['seg'] == seg and (name is None or b['name'] == name)]
def save_parts(region, parts):
    """parts: list of dict(seg, name, latin, pos (N,3) rig rest world metres, idx (M,3))"""
    arrs, meta = {}, []
    for i, p in enumerate(parts):
        arrs[f'p{i}_pos'] = np.asarray(p['pos'], np.float32); arrs[f'p{i}_idx'] = np.asarray(p['idx'], np.int32)
        meta.append({'key': f'p{i}', 'seg': p['seg'], 'name': p['name'], 'latin': p['latin']})
    np.savez_compressed(f'{ROOT}/parts_{region}.npz', **arrs)
    json.dump(meta, open(f'{ROOT}/parts_{region}.json', 'w'), ensure_ascii=False)
    print(region, len(parts), 'parts,', sum(len(p['idx']) for p in parts), 'triangles')
