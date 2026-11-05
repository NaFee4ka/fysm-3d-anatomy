#!/usr/bin/env python3
"""Register the 23 actual BodyParts3D disks to the frozen ready skeleton.

python3 joints/spine_discs_build.py --archive /path/partof_BP3D_4.0_obj_99.zip
Only Python 3 + NumPy are needed. No skeleton, pose or physics is rewritten.
The source shape is anatomical; registration and two-anchor display binding
remain geometric approximations, without disk pressure/contact mechanics.
"""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
import re
import zipfile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE_SHA = '9fbc713fffeee924a5a657d9813d84d7eb957bded63adb854931dd5e3eb61c97'
ARCHIVE_URL = 'https://dbarchive.biosciencedbc.jp/data/bodyparts3d/LATEST/partof_BP3D_4.0_obj_99.zip'
DEPENDENCIES = ['anatomy3.js', 'skull/bones_bp3d.json', 'skull/rig_rest.json', 'muscles/coverage.json',
                'joints/source-audit.json', 'joints/spine_discs_build.py', 'spine_surface_render.js']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture_sources():
    return {name: sha(ROOT / name) for name in DEPENDENCIES}


def verify_sources(expected):
    current = capture_sources()
    changed = [name for name in DEPENDENCIES if current[name] != expected[name]]
    if changed:
        raise ValueError('Spine disk sources changed during build: ' + ', '.join(changed))


def source_obj(raw):
    vertices, faces = [], []
    for line in raw.decode('utf-8').splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(x) for x in fields[1:4]])
        elif fields[0] == 'f':
            face = [int(x.split('/')[0]) for x in fields[1:]]
            face = [i - 1 if i > 0 else len(vertices) + i for i in face]
            for i in range(1, len(face) - 1):
                faces.append([face[0], face[i], face[i + 1]])
    v, f = np.asarray(vertices, float), np.asarray(faces, np.int64)
    if v.ndim != 2 or v.shape[1] != 3 or not len(f) or not np.isfinite(v).all() or f.min() < 0 or f.max() >= len(v):
        raise ValueError('Invalid source OBJ')
    # Keep the source indices. Several genuine meshes contain coincident
    # opposite-facing thin surfaces; welding those cancels their normals.
    # Equal coordinates receive exactly equal binding weights, so retained
    # source seams remain coincident throughout motion.
    if np.any(np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1) < 1e-12):
        raise ValueError('Source disk has degenerate triangles')
    return np.stack([v[:, 0], v[:, 2], -v[:, 1]], axis=1) * .001, f


def level_of_registration(record):
    name = record['name']
    match = re.search(r'(?<![A-Za-z])(Th\d+|L\d+|C\d+)(?:\b|\s|\()', name)
    if match:
        return match[1].replace('Th', 'T')
    return 'S1' if name == 'Крестец' else None


def normalize_level(name):
    return name.replace('T', 'Th')


def delta(values, channels):
    a = np.asarray(values, dtype=np.int64).reshape(-1, channels)
    out = a.copy()
    out[1:] -= a[:-1]
    return (out & 65535).astype('<u2').tobytes()


def topology(v, f):
    crosses = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    twice_areas = np.linalg.norm(crosses, axis=1)
    if not np.isfinite(v).all() or float(twice_areas.min()) < 1e-13:
        raise ValueError('Registration or quantization collapsed a disk triangle')
    normals = np.zeros_like(v)
    for k in range(3):
        np.add.at(normals, f[:, k], crosses)
    if np.any(np.linalg.norm(normals, axis=1) < 1e-14):
        raise ValueError('Disk contains a zero vertex normal')
    edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return {'vertices': len(v), 'triangles': len(f), 'boundaryEdges': int(np.sum(counts == 1)),
            'nonManifoldEdges': int(np.sum(counts > 2)), 'areaM2': float(twice_areas.sum() / 2),
            'minimumTriangleAreaM2': float(twice_areas.min() / 2),
            'signedVolumeM3': float(np.einsum('ij,ij->i', v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum() / 6)}


def build(archive_path):
    # Bind the geometry to what was actually read. A snapshot taken only after
    # packing could accidentally certify new registration bytes with old disks.
    source_hashes = capture_sources()
    if sha(archive_path) != ARCHIVE_SHA:
        raise ValueError('Official source archive SHA256 differs from the audited release')
    basis = json.loads((ROOT / 'muscles/coverage.json').read_text())
    registration = {level_of_registration(r): r for r in basis['boneRegistration'] if level_of_registration(r)}
    audit = json.loads((ROOT / 'joints/source-audit.json').read_text())
    source_records = audit['intervertebralDisks']['parts']
    if len(source_records) != 23 or len({r['level'] for r in source_records}) != 23:
        raise ValueError('Exactly 23 unique audited intervertebral disks are required')
    parts, blobs, measurements = [], [], []
    with zipfile.ZipFile(archive_path) as archive:
        for record in source_records:
            level = record['level']
            superior, inferior = level.split('-')
            upper, lower = registration[superior], registration[inferior]
            entry = 'partof_BP3D_4.0_obj_99/' + record['elementId'] + '.obj'
            raw = archive.read(entry)
            original, faces = source_obj(raw)
            source_topology = topology(original, faces)
            # The least-variance PCA direction follows the source disk's thin
            # axis. Orient it superiorly; min/max coordinates define geometric
            # attachment blending, not annulus/nucleus or material properties.
            unique_positions = np.unique(original, axis=0)
            center = unique_positions.mean(axis=0)
            _, eigenvectors = np.linalg.eigh(np.cov((unique_positions - center).T))
            axis = eigenvectors[:, 0]
            if axis[1] < 0:
                axis = -axis
            if axis[1] < .5:
                raise ValueError('Ambiguous superior-inferior source disk axis: ' + level)
            heights = (original - center) @ axis
            weights = (heights - heights.min()) / (heights.max() - heights.min())
            lower_matrix, upper_matrix = np.asarray(lower['matrix']), np.asarray(upper['matrix'])
            homogeneous = np.c_[original, np.ones(len(original))]
            low_points, high_points = homogeneous @ lower_matrix, homogeneous @ upper_matrix
            placed = low_points * (1 - weights[:, None]) + high_points * weights[:, None]
            placed_topology = topology(placed, faces)
            # Registration must preserve every source triangle orientation
            # relative to its blended, proper bone rotation. Large pose-fit
            # disagreements may change thickness; we report them explicitly.
            source_cross = np.cross(original[faces[:, 1]] - original[faces[:, 0]], original[faces[:, 2]] - original[faces[:, 0]])
            moved_cross = np.cross(placed[faces[:, 1]] - placed[faces[:, 0]], placed[faces[:, 2]] - placed[faces[:, 0]])
            mid_rotation = (lower_matrix[:3] + upper_matrix[:3]) / 2
            references = source_cross @ mid_rotation
            cosines = np.einsum('ij,ij->i', references, moved_cross) / (np.linalg.norm(references, axis=1) * np.linalg.norm(moved_cross, axis=1))
            if float(cosines.min()) <= 0:
                raise ValueError('Registration inverted a source disk triangle: ' + level)
            lo, hi = placed.min(axis=0), placed.max(axis=0)
            if len(placed) >= 65536:
                raise ValueError('Source disk exceeds 16-bit vertex budget: ' + level)
            q = np.round((placed - lo) / np.maximum(hi - lo, 1e-12) * 65535).astype('<u2')
            quantized = lo + q / 65535 * (hi - lo)
            quantized_topology = topology(quantized, faces)
            wq = np.round(weights * 65535).astype('<u2')
            blobs.extend([delta(q, 3), delta(faces.reshape(-1), 1), wq.tobytes()])
            old_name = 'Межпозвонковый диск ' + normalize_level(inferior) + '–' + normalize_level(superior)
            if inferior == 'S1':
                old_name = 'Межпозвонковый диск L5–S1'
            old_seg = 'lumA' if inferior == 'S1' else lower['seg']
            inferred = 'inferred' in record['levelAssignment']
            part = {'id': 'disc-' + level, 'level': level, 'name': 'Межпозвонковый диск ' + normalize_level(superior) + '–' + normalize_level(inferior),
                    'latin': 'discus intervertebralis ' + level, 'source': 'BodyParts3D 4.0', 'sourceIds': [record['elementId']],
                    'sourceObjSha256': hashlib.sha256(raw).hexdigest(), 'levelAssignment': 'inferred' if inferred else 'metadata',
                    'superior': {'name': superior, 'seg': upper['seg']}, 'inferior': {'name': inferior, 'seg': lower['seg']},
                    'method': 'atlas-shape; adjacent frozen-bone affine registration; two-anchor linear display skinning',
                    'n': len(placed), 'm': len(faces), 'lo': lo.tolist(), 'hi': hi.tolist(),
                    'replaces': [{'seg': old_seg, 'name': old_name}],
                    'registration': {'lowerBone': lower['name'], 'upperBone': upper['name'], 'axisSource': axis.tolist(),
                                     'lowerMatrix': lower['matrix'], 'upperMatrix': upper['matrix'],
                                     'maxInterBoneFitDifferenceM': float(np.linalg.norm(high_points - low_points, axis=1).max()),
                                     'minimumTriangleNormalCosine': float(cosines.min()),
                                     'volumeRatio': placed_topology['signedVolumeM3'] / source_topology['signedVolumeM3'],
                                     'maxQuantizationErrorM': float(np.linalg.norm(quantized - placed, axis=1).max())}}
            parts.append(part)
            measurements.append({'id': part['id'], 'source': source_topology, 'registered': placed_topology, 'quantized': quantized_topology})
    payload = b''.join(blobs)
    output = {'schemaVersion': 1, 'source': 'Actual BodyParts3D intervertebral disk meshes; geometric registration and display deformation are approximations.',
              'license': 'CC BY 4.0; BodyParts3D, © The Database Center for Life Science (DBCLS)',
              'sourceArchive': {'url': ARCHIVE_URL, 'sha256': ARCHIVE_SHA}, 'sourceHashes': source_hashes,
              'encoding': 'u16-position-deltas,u16-index-deltas,u16-superior-weights;gzip;base64',
              'excludedLegacy': [{'seg': 'neckB', 'name': 'Межпозвонковый диск C2–C1', 'reason': 'There is no intervertebral disk between the atlas and axis.'}],
              'limitations': ['Atlas shape is registered to an existing generic rig, not patient anatomy.',
                              'The T12-L1 level assignment is inferred from source geometry and the neighboring sequence.',
                              'Two-anchor linear skinning is a display approximation; no contact, pressure, fluid, volume law or FEM is computed.',
                              'Frozen vertebra fits disagree locally; source disk thickness and shape change during registration and are not measured.'],
              'parts': parts, 'data': base64.b64encode(gzip.compress(payload, compresslevel=9, mtime=0)).decode()}
    report = {'schemaVersion': 1, 'parts': len(parts), 'vertices': sum(p['n'] for p in parts), 'triangles': sum(p['m'] for p in parts),
              'sourceArchiveSha256': ARCHIVE_SHA, 'sourceHashes': source_hashes, 'rawBytes': len(payload), 'measurements': measurements,
              'dataSha256': hashlib.sha256(encode(output).encode('utf-8')).hexdigest(),
              'scope': 'Source identity, topology, registration orientation and quantization. No contact or physiological validation.'}
    verify_sources(source_hashes)
    return output, report


def encode(data):
    return json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--check', action='store_true', help='Compare in memory; write nothing.')
    args = parser.parse_args()
    data, report = build(args.archive)
    for path, value in [(ROOT / 'joints/spine-discs.json', data), (ROOT / 'joints/spine-discs-report.json', report)]:
        text = encode(value)
        verify_sources(data['sourceHashes'])
        if args.check:
            if path.read_text() != text:
                raise SystemExit('Stale spine disk data: ' + str(path))
        else:
            temporary = path.with_suffix(path.suffix + '.tmp')
            temporary.write_text(text); temporary.replace(path)
    print(json.dumps({k: report[k] for k in ['parts', 'vertices', 'triangles', 'rawBytes']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
