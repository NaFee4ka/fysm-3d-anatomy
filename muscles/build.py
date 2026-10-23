#!/usr/bin/env python3
"""Register BodyParts3D muscle surfaces to the existing skeleton and pack them.

Needs numpy, scipy, trimesh and fast-simplification. Run from the repository:
  python3 muscles/build.py --source /path/to/bp3d --reconstructed /path/to/extra.json
The source directory contains the official 4.0 OBJ archive and relation tables.
No skeleton, pose or physics data is rewritten.
"""
import argparse
import base64
import contextlib
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import runpy
import sys
import types
import zipfile

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import fast_simplification
import trimesh
from detail import TORSO_UNITS, detail_target, weld_positions, refine_edges, sample_source, shape_error

ROOT = Path(__file__).resolve().parents[1]


def bone_registration(source):
    """Recover the exact existing bone fits before their topology was simplified.

    Existing builders are executed with in-memory load/save hooks. Source and
    destination vertices retain their ordering, so a least-squares affine fit
    recovers similarity fits and the legs' axial stretch without ICP ambiguity.
    """
    mod = types.ModuleType('bp3d_lib')
    code = (ROOT / 'skull/bp3d_lib.py').read_text().replace(
        "SRC = '/home/claude/skullsrc'", 'SRC = ' + repr(str(source)))
    exec(compile(code, str(ROOT / 'skull/bp3d_lib.py'), 'exec'), mod.__dict__)
    sys.modules['bp3d_lib'] = mod
    source_faces = {}
    cache = {}
    archives = [zipfile.ZipFile(p) for p in sorted(source.glob('*BP3D_4.0_obj_99.zip'))]
    entries = {}
    for archive in archives:
        for name in archive.namelist():
            if name.endswith('.obj'):
                entries[Path(name).stem] = (archive, name)

    def load(fid):
        if fid not in cache:
            if fid not in entries:
                raise ValueError('Missing source OBJ ' + fid + '; both partof and isa OBJ archives are required')
            archive, name = entries[fid]
            mesh = trimesh.load(io.BytesIO(archive.read(name)), file_type='obj', force='mesh', process=True)
            v = np.asarray(mesh.vertices, float)
            v = np.stack([v[:, 0], v[:, 2], -v[:, 1]], 1) * .001
            cache[fid] = (v, np.asarray(mesh.faces))
        v, f = cache[fid]
        source_faces[id(f)] = (v, [fid])
        return v, f

    def load_many(ids):
        vertices, faces, offset = [], [], 0
        for fid in ids:
            v, f = load(fid)
            vertices.append(v)
            faces.append(f + offset)
            offset += len(v)
        v, f = np.concatenate(vertices), np.concatenate(faces)
        source_faces[id(f)] = (v, list(ids))
        return v, f

    registrations = []

    def save_parts(region, parts):
        for p in parts:
            original, ids = source_faces[id(p['idx'])]
            target = p['pos']
            matrix, *_ = np.linalg.lstsq(
                np.c_[original, np.ones(len(original))], target, rcond=None)
            error = np.linalg.norm(np.c_[original, np.ones(len(original))] @ matrix - target, axis=1)
            if error.max() > 1e-5:
                raise ValueError('Non-affine bone fit: ' + p['name'])
            registrations.append(dict(seg=p['seg'], name=p['name'], ids=ids,
                                      matrix=matrix, original=original,
                                      fitMax=float(error.max())))

    mod.load, mod.load_many = load, load_many
    mod.decimate = lambda v, f, keep: (v, f)
    mod.save_parts = save_parts
    with contextlib.redirect_stdout(io.StringIO()):
        for region in ('trunk', 'arms', 'legs'):
            runpy.run_path(str(ROOT / f'skull/bones_{region}.py'), run_name='__registration__')

    # The former skull builder used one uniform fit for all cranial bones.
    head_ids = ['FJ3200', 'FJ3274', 'FJ3380', 'FJ3309', 'FJ3281', 'FJ3386',
                'FJ3394', 'FJ3199', 'FJ3287', 'FJ3392', 'FJ3269', 'FJ3375',
                'FJ3272', 'FJ3378', 'FJ3265', 'FJ3371', 'FJ3273', 'FJ3379',
                'FJ3395', 'FJ3263', 'FJ3369', 'FJ3289', 'FJ2772']
    head = [load(fid)[0] for fid in head_ids]
    occ = load('FJ3309')[0]
    cond = occ[occ[:, 1] < occ[:, 1].min() + .006].mean(0)
    scale = (1.755 - 1.603) / (max(v[:, 1].max() for v in head) - cond[1])
    matrix = np.vstack([np.eye(3) * scale, np.array([0, 1.603, -.03]) - cond * scale])
    registrations.append(dict(seg='head', name='Cranium', ids=head_ids,
                              matrix=matrix, original=np.concatenate(head), fitMax=0.0))
    load.available = set(entries)
    return registrations, load


def make_warp(registrations):
    # A uniformly spaced subset avoids letting the original triangulation
    # density determine which anatomical bone wins the distance weighting.
    trees = []
    for r in registrations:
        v = r['original']
        cell = np.floor(v / .006).astype(np.int32)
        _, take = np.unique(cell, axis=0, return_index=True)
        trees.append(cKDTree(v[take]))
    matrices = np.asarray([r['matrix'] for r in registrations])

    def warp(v):
        # Query each bone separately: a dense nearby surface must never hide
        # another bone from the candidate set. Compact support reaches zero
        # at the eighth-neighbour radius, so a change of neighbour identity
        # cannot introduce a nonzero jump in the displacement field.
        all_distances = np.stack([t.query(v, workers=1)[0] for t in trees], axis=1)
        chosen = np.argpartition(all_distances, 7, axis=1)[:, :8]
        distances = np.take_along_axis(all_distances, chosen, axis=1)
        radius = distances.max(axis=1, keepdims=True) + 1e-9
        taper = np.maximum(0, 1 - (distances / radius) ** 2) ** 2
        weights = taper / (distances ** 2 + .012 ** 2) ** 1.5
        weights /= weights.sum(1)[:, None]
        hom = np.c_[v, np.ones(len(v))]
        moved = np.einsum('ni,nkij->nkj', hom, matrices[chosen])
        return (moved * weights[:, :, None]).sum(1)

    return warp


def make_scm_warp(registrations, side):
    """Map the anterior neck surface through its actual attachment bones.

    Independent clavicle and posterior cervical fits disagree when extended
    into the anterior neck. SCM attaches to the skull, clavicle and sternum;
    a continuous blend of those supports avoids pulling it towards vertebrae
    that happen to be geometrically close but are not its attachments.
    """
    selected = [r for r in registrations if r['seg'] == 'head'
                or r['seg'] == 'clav_' + side or r['name'] == 'Рукоятка грудины']
    if len(selected) != 3:
        raise ValueError('SCM attachment registration requires skull, clavicle and manubrium')
    trees = [cKDTree(r['original']) for r in selected]
    matrices = np.asarray([r['matrix'] for r in selected])

    def warp(vertices):
        distance = np.stack([tree.query(vertices, workers=1)[0] for tree in trees], axis=1)
        weights = 1 / (distance ** 2 + .025 ** 2) ** 1.5
        weights /= weights.sum(1, keepdims=True)
        moved = np.einsum('ni,kij->nkj', np.c_[vertices, np.ones(len(vertices))], matrices)
        return (moved * weights[:, :, None]).sum(1)

    return warp, dict(method='attachment-bones', structures=['cranium', 'ipsilateral clavicle', 'manubrium'],
                      sourceBoneIds=[fid for r in selected for fid in r['ids']],
                      reason='SCM is an anterior superficial muscle; posterior cervical proximity is not an attachment')


def simplify(v, f, target):
    if len(f) > target:
        v, f = fast_simplification.simplify(
            v.astype(np.float32), f.astype(np.int32),
            target_reduction=1 - target / len(f))
    return np.asarray(v), np.asarray(f)


def unpack_parts(path):
    data = json.loads(path.read_text())
    raw = gzip.decompress(base64.b64decode(data['data']))
    parts, offset = {}, 0
    for meta in data['parts']:
        q = np.frombuffer(raw, '<u2', meta['n'] * 3, offset).reshape(-1, 3)
        offset += meta['n'] * 6
        q = np.cumsum(q.astype(np.uint64), axis=0) & 65535
        vertices = np.asarray(meta['lo']) + q / 65535 * (np.asarray(meta['hi']) - meta['lo'])
        d = np.frombuffer(raw, '<u2', meta['m'] * 3, offset)
        offset += meta['m'] * 6
        faces = (np.cumsum(d.astype(np.uint64)) & 65535).reshape(-1, 3).astype(np.int32)
        if meta['source'] == 'bp3d':
            parts[(meta['unit'], meta['side'], meta['ids'][0])] = (vertices, faces)
    return parts


def edge_metrics(original, placed, faces):
    """Registration distortion, counting each triangle's three directed edges."""
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    before = np.linalg.norm(original[edges[:, 0]] - original[edges[:, 1]], axis=1)
    after = np.linalg.norm(placed[edges[:, 0]] - placed[edges[:, 1]], axis=1)
    ratio = after / np.maximum(before, 1e-8)
    outliers = ratio > 4
    return dict(triangleEdges=len(edges), stretchedOver4=int(outliers.sum()),
                p99Ratio=float(np.percentile(ratio, 99)), maxRatio=float(ratio.max()),
                sourceMaxOutlierMm=float(before[outliers].max() * 1000) if outliers.any() else 0,
                placedMaxOutlierMm=float(after[outliers].max() * 1000) if outliers.any() else 0)


def clean_tiny_islands(vertices, faces):
    """Remove only microscopic, isolated remnants inside an individual part.

    Near-duplicate vertices are joined for connectivity analysis only; the
    retained surface coordinates and topology are never welded or smoothed.
    Distance is measured to *all* other components, preserving real slips.
    """
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    close = cKDTree(vertices).query_pairs(.00005, output_type='ndarray')
    edges = np.concatenate([edges, close])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    face_labels = labels[faces[:, 0]]
    areas = np.linalg.norm(np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                                    vertices[faces[:, 2]] - vertices[faces[:, 0]]), axis=1) * .5
    component_area = np.bincount(face_labels, weights=areas, minlength=count)
    keep = np.ones(len(faces), bool)
    removed = []
    for c in np.unique(face_labels):
        ix = np.flatnonzero(face_labels == c)
        vv = vertices[np.unique(faces[ix])]
        span = float(np.linalg.norm(vv.max(0) - vv.min(0)))
        area = component_area[c]
        if len(ix) > 8 or area > 1e-6 or span > .003 or area / max(areas.sum(), 1e-20) > .0001:
            continue
        other = vertices[np.unique(faces[face_labels != c])]
        if not len(other):
            continue
        distance = float(cKDTree(other).query(vv)[0].min())
        if distance <= .01:
            continue
        keep[ix] = False
        removed.append(dict(triangles=len(ix), areaMm2=float(area * 1e6),
                            spanMm=span * 1000, distanceMm=distance * 1000,
                            decimatedFaceIndices=ix.tolist()))
    if not removed:
        return vertices, faces, np.arange(len(vertices)), []
    used = np.unique(faces[keep])
    remap = np.full(len(vertices), -1, np.int32)
    remap[used] = np.arange(len(used))
    return vertices[used], remap[faces[keep]], used, removed


def pack(parts, out):
    metadata, chunks = [], []

    def delta(a):
        a = np.asarray(a, np.int64)
        d = a.copy()
        d[1:] -= a[:-1]
        return (d & 65535).astype('<u2').tobytes()

    for p in parts:
        v, f = np.asarray(p.pop('positions')).reshape(-1, 3), np.asarray(p.pop('indices')).reshape(-1, 3)
        original = p.pop('_original', None)
        if not np.isfinite(v).all() or len(v) >= 65536 or f.min() < 0 or f.max() >= len(v):
            raise ValueError('Invalid surface: ' + p['unit'])
        lo, hi = v.min(0), v.max(0)
        q = np.round((v - lo) / np.maximum(hi - lo, 1e-7) * 65535)
        if original is not None:
            quantised = lo + q / 65535 * np.maximum(hi - lo, 1e-7)
            p['registrationMetrics'] = edge_metrics(original, quantised, f)
        chunks.extend((delta(q), delta(f.reshape(-1))))
        metadata.append(dict(p, n=len(v), m=len(f), lo=lo.round(7).tolist(), hi=hi.round(7).tolist()))
    data = dict(v=1, src='BodyParts3D 4.0, DBCLS', license='CC BY 4.0', parts=metadata,
                data=base64.b64encode(gzip.compress(b''.join(chunks), 9, mtime=0)).decode())
    temporary = out.with_suffix(out.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, separators=(',', ':')))
    temporary.replace(out)
    return data


def complete_report(report, source, baseline=None, reconstructed=None, torso_detail=False, triangles=800):
    """Record reproducible inputs and the aggregate sampled source audit."""
    def checksum(path):
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                h.update(block)
        return h.hexdigest()

    report['buildParameters'] = dict(baseTriangles=triangles, torsoDetail=torso_detail,
                                    torsoEdgeLimitMm=25 if torso_detail else None,
                                    baselineSha256=checksum(baseline) if baseline else None,
                                    reconstructionSha256=checksum(reconstructed) if reconstructed else None)
    report['sourceArtifacts'] = {p.name: dict(bytes=p.stat().st_size, sha256=checksum(p))
                                  for p in sorted(source.iterdir())
                                  if p.name in {'partof_element_parts.txt', 'isa_element_parts.txt',
                                                'partof_BP3D_4.0_obj_99.zip', 'isa_BP3D_4.0_obj_99.zip'}}
    errors = [p['sourceDeviation'] for p in report['torsoDetail'] if 'sourceDeviation' in p]
    if errors:
        report['sourceDeviationSummary'] = dict(
            reference='768 deterministic area-weighted samples per registered raw-source part; SCM uses its attachment-bone fit',
            parts=len(errors),
            meanOfPartP95BeforeMm=float(np.mean([e['before']['p95Mm'] for e in errors if 'before' in e])),
            meanOfPartP95AfterMm=float(np.mean([e['after']['p95Mm'] for e in errors])),
            allPartP95Improved=all(e['after']['p95Mm'] <= e['before']['p95Mm'] for e in errors if 'before' in e))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--reconstructed', type=Path)
    parser.add_argument('--out', type=Path, default=ROOT / 'muscles/muscles_bp3d.json')
    parser.add_argument('--triangles', type=int, default=800, help='target triangles per source piece (open surfaces may retain more)')
    parser.add_argument('--partial', action='store_true', help='preview only: skip missing source files and report partial coverage')
    parser.add_argument('--torso-detail', action='store_true', help='area/length-adaptive torso LOD and conforming 25 mm edge refinement')
    parser.add_argument('--baseline', type=Path, help='previous packed atlas, for sampled raw-source shape comparison on the torso')
    args = parser.parse_args()
    os.chdir(ROOT)
    source = args.source.resolve()
    mapping = json.loads((ROOT / 'muscles/source-map.json').read_text())
    baseline = unpack_parts(args.baseline) if args.baseline else {}
    registration, load = bone_registration(source)
    print('Recovered', len(registration), 'bone fits', flush=True)
    warp = make_warp(registration)
    scm_warp = {side: make_scm_warp(registration, side) for side in ('L', 'R')}
    parts, seen = [], set()
    coverage = {}
    for unit, u in mapping['units'].items():
        coverage[unit] = {'status': u['status'], 'sides': {}}
        if u['status'] == 'missing':
            continue
        for side in ('L', 'R'):
            ids = list(u['leftIds' if side == 'L' else 'rightIds'])
            mirrored = False
            if not ids and u.get('mirrorMissingSide'):
                ids = u['rightIds' if side == 'L' else 'leftIds']
                mirrored = True
            missing = set(ids) - load.available
            if missing and not args.partial:
                raise ValueError('Missing source OBJ: ' + ', '.join(sorted(missing)))
            if missing:
                ids = [fid for fid in ids if fid in load.available]
                coverage[unit]['status'] = 'partial'
            triangles = 0
            for fid in ids:
                key = (fid, side)
                if key in seen:
                    raise ValueError('Duplicate muscle source volume: ' + repr(key))
                seen.add(key)
                v, f = load(fid)
                raw_count = len(v)
                fit, policy = scm_warp[side] if unit == 'scm' else (warp, None)
                watertight_before = bool(trimesh.Trimesh(vertices=v, faces=f, process=False).is_watertight)
                v, f, merged = weld_positions(v, f)
                topology = dict(rawVertices=raw_count, mergedExactVertices=merged,
                                trianglesBefore=len(f), trianglesAfter=len(f),
                                watertightBefore=watertight_before,
                                watertightAfter=bool(trimesh.Trimesh(vertices=v, faces=f, process=False).is_watertight))
                is_detailed = args.torso_detail and unit in TORSO_UNITS
                reference = fit(sample_source(v, f)) if is_detailed and baseline else None
                target = detail_target(v, f, args.triangles) if is_detailed else args.triangles
                original, f = simplify(v, f, target)
                if is_detailed:
                    original, f, v, detail = refine_edges(original, f, fit)
                    detail['lodTarget'] = target
                else:
                    v = fit(original)
                    detail = None
                v, f, kept_vertices, cleanup = clean_tiny_islands(v, f)
                original = original[kept_vertices]
                if mirrored:
                    v[:, 0] *= -1
                    f = f[:, ::-1]
                # Some relation-table side labels disagree with the actual
                # source geometry (notably FJ1469/FJ1469M). Catch this before
                # a part can bind to the opposite hand in the browser.
                if np.median(v[:, 0]) * (1 if side == 'L' else -1) < -.01:
                    raise ValueError(f'{unit}/{side}/{fid}: source geometry lies on the opposite side')
                part = dict(unit=unit, side=side, ids=[fid], source='bp3d',
                            mirrored=mirrored, positions=v, indices=f, _original=original,
                            topologyCleanup=topology)
                if detail:
                    part['detail'] = detail
                if policy:
                    part['registrationPolicy'] = policy
                if reference is not None:
                    # Mirrored longusColli is not in the detailed torso set;
                    # all audited parts use their original source side.
                    old = baseline.get((unit, side, fid))
                    part['sourceDeviation'] = dict(after=shape_error(reference, v, f))
                    if old:
                        part['sourceDeviation']['before'] = shape_error(reference, *old)
                if cleanup:
                    part['sourceCleanup'] = cleanup
                if fid in u.get('sideAssignmentsFromGeometry', {}):
                    part['sideCorrection'] = u['sideCorrection']
                parts.append(part)
                triangles += len(f)
            coverage[unit]['sides'][side] = {'ids': ids, 'mirrored': mirrored, 'triangles': triangles, 'missingIds': sorted(missing)}
        print(unit, sum(p['indices'].shape[0] for p in parts if p['unit'] == unit), flush=True)
    if args.reconstructed:
        extra = json.loads(args.reconstructed.read_text())
        for p in extra['parts']:
            p.setdefault('source', 'reconstructed')
            p.setdefault('ids', [])
            p['positions'] = np.asarray(p['positions']).reshape(-1, 3)
            p['indices'] = np.asarray(p['indices']).reshape(-1, 3)
            parts.append(p)
            coverage[p['unit']]['status'] = 'reconstructed'
            coverage[p['unit']]['sides'][p['side']] = {
                'ids': [], 'source': 'reconstructed', 'triangles': len(p['indices'])}
    args.out.parent.mkdir(exist_ok=True, parents=True)
    packed = pack(parts, args.out)
    report = dict(source='BodyParts3D 4.0 / DBCLS', boneRegistration=[
        {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in r.items() if k != 'original'}
        for r in registration], units=coverage,
        sourceUnits=len({p['unit'] for p in packed['parts'] if p['source'] == 'bp3d'}),
        reconstructedUnits=sum(u['status'] == 'reconstructed' for u in coverage.values()),
        parts=len(packed['parts']), triangles=sum(p['m'] for p in packed['parts']))
    metrics = [p['registrationMetrics'] for p in packed['parts'] if 'registrationMetrics' in p]
    report['registrationMetrics'] = {
        'triangleEdges': sum(m['triangleEdges'] for m in metrics),
        'stretchedOver4': sum(m['stretchedOver4'] for m in metrics),
        'maxRatio': max(m['maxRatio'] for m in metrics),
        'outlierParts': [{k: p[k] for k in ('unit', 'side', 'ids', 'registrationMetrics')}
                         for p in packed['parts'] if p.get('registrationMetrics', {}).get('stretchedOver4')]}
    report['sourceCleanup'] = [{k: p[k] for k in ('unit', 'side', 'ids', 'sourceCleanup')}
                               for p in packed['parts'] if p.get('sourceCleanup')]
    report['topologyCleanup'] = {
        'parts': sum('topologyCleanup' in p for p in packed['parts']),
        'mergedExactVertices': sum(p.get('topologyCleanup', {}).get('mergedExactVertices', 0) for p in packed['parts']),
        'watertightBefore': sum(p.get('topologyCleanup', {}).get('watertightBefore', False) for p in packed['parts']),
        'watertightAfter': sum(p.get('topologyCleanup', {}).get('watertightAfter', False) for p in packed['parts'])}
    report['torsoDetail'] = [{k: p[k] for k in ('unit', 'side', 'ids', 'detail', 'sourceDeviation') if k in p}
                              for p in packed['parts'] if 'detail' in p]
    complete_report(report, source, args.baseline, args.reconstructed, args.torso_detail, args.triangles)
    (args.out.parent / 'coverage.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('Packed', report['parts'], 'parts,', report['triangles'], 'triangles;', args.out.stat().st_size, 'bytes')


if __name__ == '__main__':
    main()
