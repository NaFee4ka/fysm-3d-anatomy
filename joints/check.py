#!/usr/bin/env python3
"""Validate topology, source labels, legacy coverage and attachment to frozen bones."""
import argparse
import base64
import gzip
import json
from pathlib import Path

import numpy as np

from build import ROOT, build, normals, read_bones

LEGACY = {
    'gh': [('hum', 'Хрящ головки плечевой кости'), ('scap', 'Суставная впадина лопатки'), ('scap', 'Суставная губа')],
    'hip': [('thigh', 'Хрящ головки бедренной кости'), ('pelvis', 'Вертлужная губа')],
    'kn': [('thigh', 'Хрящ мыщелков бедра'), ('shank', 'Хрящ плато большеберцовой кости'),
           ('shank', 'Хрящ надколенника'), ('shank', 'Медиальный мениск'), ('shank', 'Латеральный мениск')],
    'el': [('hum', 'Хрящ блока и головки мыщелка'), ('fore', 'Хрящ головки лучевой кости')],
    'wr': [('fore', 'Хрящ лучезапястного сустава')],
    'an': [('foot', 'Хрящ блока таранной кости')],
}


def distances(points, bone):
    """Exact closest-triangle distance; NumPy only, bounded batches, no rtree."""
    tri = bone['v'][bone['f']]
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    ab, ac = b-a, c-a
    d00, d01, d11 = (ab*ab).sum(1), (ab*ac).sum(1), (ac*ac).sum(1)
    denominator = d00*d11-d01*d01
    good = denominator > 1e-24
    output = []
    for start in range(0, len(points), 48):
        p = points[start:start+48, None, :]
        ap = p-a
        d20, d21 = np.einsum('bfi,fi->bf', ap, ab), np.einsum('bfi,fi->bf', ap, ac)
        u = np.divide(d11*d20-d01*d21, denominator, out=np.zeros_like(d20), where=good)
        w = np.divide(d00*d21-d01*d20, denominator, out=np.zeros_like(d20), where=good)
        d2 = ((ap-u[:, :, None]*ab-w[:, :, None]*ac)**2).sum(2)
        d2[(u < 0) | (w < 0) | (u+w > 1) | ~good] = np.inf
        for x, y in [(a, b), (b, c), (c, a)]:
            edge = y-x
            px = p-x
            denominator_edge = (edge*edge).sum(1)
            t = np.divide(np.einsum('bfi,fi->bf', px, edge), denominator_edge,
                          out=np.zeros((len(p), len(x))), where=denominator_edge > 1e-24)
            t = np.clip(t, 0, 1)
            d2 = np.minimum(d2, ((px-t[:, :, None]*edge)**2).sum(2))
        output.extend(np.sqrt(d2.min(1)))
    return np.asarray(output)


def check(path, rebuild=False):
    data = json.loads(path.read_text())
    bones, rig, sha = read_bones()
    assert data['v'] == 1 and data['basis']['sha256'] == sha, 'finished bone basis changed'
    assert 'CC BY 4.0' in data['license']
    audit = json.loads((Path(__file__).with_name('source-audit.json')).read_text())
    assert audit, 'source availability audit missing'
    raw = gzip.decompress(base64.b64decode(data['data'], validate=True))
    assert len(raw) == sum((p['n']+p['m'])*6 for p in data['parts'])
    offset, ids, covered, joints, metrics, meshes = 0, set(), set(), set(), [], {}
    total_triangles, degenerate, max_gap = 0, 0, 0.
    for p in data['parts']:
        assert p['id'] not in ids; ids.add(p['id'])
        assert p['source'] == 'reconstructed' and p['ids'] == []
        assert p['method'] and p['boneRefs'] and p['limitations']
        assert p['seg'] in rig and p['side'] in ('L', 'R')
        assert p['type'] in ('cartilage', 'labrum', 'meniscus')
        assert 0 < p['n'] < 65536 and 0 < p['m'] < 10000
        q = np.frombuffer(raw, '<u2', p['n']*3, offset).reshape(-1, 3)
        offset += p['n']*6
        q = np.cumsum(q.astype(np.uint64), axis=0) & 65535
        v = np.asarray(p['lo'])+q/65535*(np.asarray(p['hi'])-p['lo'])
        d = np.frombuffer(raw, '<u2', p['m']*3, offset)
        offset += p['m']*6
        f = (np.cumsum(d.astype(np.uint64)) & 65535).reshape(-1, 3).astype(np.int64)
        assert f.min() >= 0 and f.max() < p['n']
        assert np.isfinite(v).all() and np.abs(v).max() < 2
        assert (np.asarray(p['hi']) > p['lo']).all()
        assert np.median(v[:, 0])*(1 if p['side'] == 'L' else -1) > .025, ('opposite side', p['id'])
        area = np.linalg.norm(np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]]), axis=1)/2
        collapsed = int((area < 1e-14).sum())
        assert not collapsed, ('collapsed triangles after quantization', p['id'], collapsed)
        degenerate += collapsed
        directed = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
        edges = np.sort(directed, axis=1)
        _, inverse, counts = np.unique(edges, axis=0, return_inverse=True, return_counts=True)
        assert (counts == 2).all(), ('open or nonmanifold shell', p['id'])
        direction = np.where(directed[:, 0] < directed[:, 1], 1, -1)
        assert (np.bincount(inverse, weights=direction) == 0).all(), ('inconsistent face winding', p['id'])
        assert np.allclose(np.linalg.norm(normals(v, f), axis=1), 1), ('undefined normals', p['id'])
        covered.update((p['seg'], name) for name in p['replaces'])
        joints.add(p['joint'])
        meshes[p['id']]=(v,area)
        if p['id'].startswith('glenoid_'):
            shoulder=np.asarray(rig['hum_'+p['side']]['J'])
            assert v[:,1].max()<shoulder[1]+.022, ('acromial region mistaken for socket',p['id'])
        ref = p['boneRefs'][0]
        matches = [b for b in bones if b['meta']['seg'] == ref['seg'] and b['meta']['latin'] == ref['latin']
                   and (ref['seg'] != 'pelvis' or np.median(b['v'][:, 0])*(1 if p['side'] == 'L' else -1) > 0)]
        assert len(matches) == 1
        if p['type'] == 'cartilage':
            gap = distances(v, matches[0])*1000
            maximum = float(gap.max()); max_gap = max(max_gap, maximum)
            assert maximum < p['parameters']['thicknessMm']+.025, ('floating bone patch', p['id'], maximum)
            metrics.append(dict(id=p['id'], maximumBoneDistanceMm=round(maximum, 5), p95BoneDistanceMm=round(float(np.percentile(gap,95)),5)))
        if p['type'] == 'meniscus':
            assert 'Open C-shaped wedge' in p['method']
            assert p['parameters']['openingDegrees'] >= 50, 'horn gap must remain open'
            gap = distances(v, matches[0])*1000
            assert gap.max() < 2.4, ('floating meniscal wedge', p['id'], gap.max())
        total_triangles += p['m']
    required = {(seg if seg == 'pelvis' else f'{seg}_{side}', name)
                for group in LEGACY.values() for seg,name in group for side in ('L','R')}
    assert required <= covered, ('missing legacy replacements', required-covered)
    assert joints == {f'{joint}_{side}' for joint in LEGACY for side in ('L','R')}
    assert len(data['parts']) == 42 and total_triangles <= 70000
    assert len(path.read_bytes()) < 700000
    paired=[]
    for stem in ['glenoid','glenoid_labrum']:
        left,la=meshes[stem+'_L'];right,ra=meshes[stem+'_R']
        ratio=np.ptp(right,axis=0)/np.ptp(left,axis=0)
        area_ratio=float(ra.sum()/la.sum())
        centre=right.mean(0).copy();centre[0]*=-1
        centre_gap=float(np.linalg.norm(centre-left.mean(0)))
        # Broad sanity only: source sides are not mirrors. A separate acromial
        # contour 30+ mm above the intended socket fails these regional bounds.
        assert ((ratio>.35)&(ratio<2.85)).all(),('asymmetric socket bounds',stem,ratio)
        assert .25<area_ratio<4 and centre_gap<.024,('socket side mismatch',stem,area_ratio,centre_gap)
        paired.append(dict(region=stem,bboxRightLeftRatio=ratio.round(5).tolist(),
                           areaRightLeftRatio=round(area_ratio,5),mirroredCentroidDistanceMm=round(centre_gap*1000,5)))
    if rebuild:
        rebuilt, _ = build()
        assert rebuilt == data, 'builder does not reproduce committed packed data'
    report = dict(parts=len(ids), triangles=total_triangles, jointGroups=len(joints),
                  degenerateTriangles=degenerate, manifoldClosedParts=len(ids),
                  maximumCartilageDistanceMm=round(max_gap,5), boneDistanceMetrics=metrics,
                  pairedSocketSanity=paired,
                  boneBasisSha256=sha, reproducible=bool(rebuild))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', nargs='?', type=Path, default=Path(__file__).with_name('joints.json'))
    parser.add_argument('--rebuild', action='store_true', help='also compare an in-memory full rebuild')
    parser.add_argument('--report', type=Path, help='write reproducible check evidence')
    args = parser.parse_args()
    result = check(args.file, args.rebuild)
    if args.report:
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: v for k,v in result.items() if k != 'boneDistanceMetrics'}, ensure_ascii=False))
