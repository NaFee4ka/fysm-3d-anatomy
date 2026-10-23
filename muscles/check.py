#!/usr/bin/env python3
"""Check packed surface integrity, source uniqueness and both-side coverage."""
import argparse
import base64
import gzip
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('file', nargs='?', type=Path, default=Path(__file__).with_name('muscles_bp3d.json'))
parser.add_argument('--partial', action='store_true', help='allow previews with incomplete coverage')
args = parser.parse_args()
data = json.loads(args.file.read_text())
mapping = json.loads(args.file.with_name('source-map.json').read_text())
raw = gzip.decompress(base64.b64decode(data['data'], validate=True))
expected = sum(6 * (p['n'] + p['m']) for p in data['parts'])
assert len(raw) == expected, (len(raw), expected)
offset, sources, covered, triangles, degenerate = 0, set(), set(), 0, 0
for p in data['parts']:
    assert p['unit'] in mapping['units'] and p['side'] in ('L', 'R')
    assert p['source'] in ('bp3d', 'reconstructed')
    assert 0 < p['n'] < 65536 and p['m'] > 0
    q = np.frombuffer(raw, dtype='<u2', count=p['n'] * 3, offset=offset).reshape(-1, 3)
    offset += p['n'] * 6
    q = np.cumsum(q.astype(np.uint64), axis=0) & 65535
    v = np.asarray(p['lo']) + q / 65535 * (np.asarray(p['hi']) - np.asarray(p['lo']))
    d = np.frombuffer(raw, dtype='<u2', count=p['m'] * 3, offset=offset)
    offset += p['m'] * 6
    f = (np.cumsum(d.astype(np.uint64)) & 65535).reshape(-1, 3)
    assert f.max() < len(v), p
    assert np.isfinite(v).all() and np.abs(v).max() < 3, p
    assert (np.asarray(p['hi']) > np.asarray(p['lo'])).all(), p
    areas = np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1)
    degenerate += int((areas < 1e-12).sum())
    assert (areas > 1e-12).mean() > .97, (p, 'too many collapsed triangles')
    for fid in p['ids']:
        key = fid, p['side']
        assert key not in sources, ('duplicated source volume', key)
        sources.add(key)
    if p['source'] == 'reconstructed':
        assert not p['ids'] and p.get('method') and p.get('limitations'), p
    if p['source'] == 'bp3d':
        assert np.median(v[:, 0]) * (1 if p['side'] == 'L' else -1) >= -.01, ('source on opposite side', p)
        metrics = p.get('registrationMetrics')
        assert metrics and metrics['triangleEdges'] == 3 * p['m'], p
        assert 0 <= metrics['stretchedOver4'] <= metrics['triangleEdges'], p
        assert np.isfinite(metrics['maxRatio']), p
    covered.add((p['unit'], p['side']))
    triangles += p['m']
missing = {(u, side) for u in mapping['units'] for side in ('L', 'R')} - covered
if not args.partial:
    assert not missing, ('uncovered model units', sorted(missing))
print(json.dumps(dict(parts=len(data['parts']), triangles=triangles, degenerateTriangles=degenerate,
                      unitSides=len(covered), missingUnitSides=len(missing), bytes=len(raw))))
