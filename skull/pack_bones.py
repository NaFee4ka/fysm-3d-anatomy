# Pack every skull/parts_<region> into skull/bones_bp3d.json: per part 16-bit positions in the part's own box
# relative to its segment's rest joint, 16-bit indices; both as deltas; one gzip; base64.
import numpy as np, json, glob, gzip, base64, os
from bp3d_lib import ROOT, J
parts, blob = [], []
def dq(a):
    d = a.astype(np.int32).copy(); d[1:] -= a[:-1].astype(np.int32); return (d & 0xFFFF).astype(np.uint16)
for meta_f in sorted(glob.glob(f'{ROOT}/parts_*.json')):
    region = os.path.basename(meta_f)[6:-5]
    A = np.load(meta_f[:-5] + '.npz')
    for m in json.load(open(meta_f)):
        pos = A[m['key'] + '_pos'].astype(np.float64) - J(m['seg']); idx = A[m['key'] + '_idx']
        if len(pos) >= 65536: raise SystemExit(f"{region}/{m['name']}: too many vertices ({len(pos)}), decimate more")
        lo, hi = pos.min(0), pos.max(0); span = np.maximum(hi - lo, 1e-6)
        q = np.round((pos - lo) / span * 65535).astype(np.uint16)
        blob.append(dq(q).tobytes()); blob.append(dq(idx.reshape(-1).astype(np.uint16)).tobytes())
        parts.append({'seg': m['seg'], 'name': m['name'], 'latin': m['latin'], 'n': int(len(pos)), 'm': int(len(idx)), 'lo': lo.round(6).tolist(), 'hi': hi.round(6).tolist(), 'region': region})
raw = b''.join(blob); gz = gzip.compress(raw, 9)
json.dump({'src': 'BodyParts3D, © The Database Center for Life Science, CC BY 4.0', 'parts': parts, 'data': base64.b64encode(gz).decode()}, open(f'{ROOT}/bones_bp3d.json', 'w'), ensure_ascii=False)
tri = sum(p['m'] for p in parts)
print('parts', len(parts), 'triangles', tri, 'gzip KB', len(gz) // 1024, 'base64 KB', len(gz) * 4 // 3 // 1024, 'segments', len({p['seg'] for p in parts}))
