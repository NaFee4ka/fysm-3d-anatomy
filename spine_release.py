"""Read-only provenance and manifest guard for the registered atlas disks."""
import json
import math
import re
import base64
import gzip
import struct
from build_verification import current_sources, digest

SPINE_INPUTS = ('anatomy3.js', 'skull/bones_bp3d.json', 'skull/rig_rest.json',
                'muscles/coverage.json', 'joints/source-audit.json',
                'joints/spine_discs_build.py', 'spine_surface_render.js')


def verify_spine_surfaces(root):
    def require(condition, message):
        if not condition:
            raise SystemExit('Spine surface verification: ' + message + '. Rebuild the atlas disk package.')

    try:
        data = json.loads((root / 'joints/spine-discs.json').read_text())
        report = json.loads((root / 'joints/spine-discs-report.json').read_text())
    except (OSError, ValueError) as error:
        raise SystemExit('Spine surface data/report is missing or invalid.') from error
    archive_sha = '9fbc713fffeee924a5a657d9813d84d7eb957bded63adb854931dd5e3eb61c97'
    levels = [f'C{i}-C{i+1}' for i in range(2, 7)] + ['C7-T1']
    levels += [f'T{i}-T{i+1}' for i in range(1, 12)] + ['T12-L1']
    levels += [f'L{i}-L{i+1}' for i in range(1, 5)] + ['L5-S1']
    parts = data.get('parts')
    require(data.get('schemaVersion') == report.get('schemaVersion') == 1
            and report.get('dataSha256') == digest(root / 'joints/spine-discs.json'), 'missing or stale report')
    require(data.get('sourceHashes') == report.get('sourceHashes')
            and current_sources(root, data.get('sourceHashes'), SPINE_INPUTS), 'stale registration sources')
    require(data.get('sourceArchive', {}).get('sha256') == archive_sha
            and report.get('sourceArchiveSha256') == archive_sha
            and 'CC BY 4.0' in data.get('license', ''), 'missing source or attribution')
    require(isinstance(parts, list) and len(parts) == report.get('parts') == 23
            and all(isinstance(p, dict) for p in parts)
            and {p.get('level') for p in parts} == set(levels)
            and len({p.get('id') for p in parts}) == 23, 'expected the 23 anatomical disk levels')
    require(data.get('encoding') == 'u16-position-deltas,u16-index-deltas,u16-superior-weights;gzip;base64'
            and isinstance(data.get('data'), str) and bool(data['data']), 'invalid packed geometry')
    vertices = triangles = 0
    segments = json.loads((root / 'skull/rig_rest.json').read_text())['segs']
    for part in parts:
        require(type(part.get('n')) is int and 3 <= part['n'] <= 65535
                and type(part.get('m')) is int and part['m'] > 0, 'invalid vertex/triangle counts')
        require(part.get('id') == 'disc-' + part['level']
                and isinstance(part.get('name'), str) and bool(part['name'].strip())
                and all(isinstance(part.get(k), list) and len(part[k]) == 3
                        and all(type(v) in (int, float) and math.isfinite(v) for v in part[k])
                        for k in ('lo', 'hi')) and all(a < b for a, b in zip(part['lo'], part['hi']))
                and all(part.get(k, {}).get('seg') in segments for k in ('superior', 'inferior')),
                'invalid disk bounds or anchors')
        require(isinstance(part.get('replaces'), list) and len(part['replaces']) > 0
                and all(isinstance(item, dict) and item.get('seg') in segments
                        and isinstance(item.get('name'), str) and bool(item['name'].strip())
                        for item in part['replaces']), 'invalid legacy replacement manifest')
        require(part.get('source') == 'BodyParts3D 4.0' and isinstance(part.get('sourceIds'), list)
                and len(part['sourceIds']) > 0 and all(isinstance(i, str) and i for i in part['sourceIds'])
                and isinstance(part.get('sourceObjSha256'), str)
                and re.fullmatch('[0-9a-f]{64}', part['sourceObjSha256'])
                and part.get('levelAssignment') == ('inferred' if part['level'] == 'T12-L1' else 'metadata'),
                'invalid source mapping')
        measure = part.get('registration', {})
        require(all(type(measure.get(k)) in (int, float) and math.isfinite(measure[k])
                    for k in ('minimumTriangleNormalCosine', 'volumeRatio', 'maxInterBoneFitDifferenceM',
                              'maxQuantizationErrorM'))
                and measure['minimumTriangleNormalCosine'] > 0 and measure['volumeRatio'] > 0
                and measure['maxInterBoneFitDifferenceM'] >= 0 and measure['maxQuantizationErrorM'] >= 0,
                'invalid registration diagnostics')
        vertices += part['n']
        triangles += part['m']
    require(report.get('vertices') == vertices and report.get('triangles') == triangles
            and report.get('rawBytes') == vertices * 8 + triangles * 6, 'incomplete geometry report')
    try:
        raw = gzip.decompress(base64.b64decode(data['data'], validate=True))
    except (ValueError, OSError, EOFError) as error:
        raise SystemExit('Spine surface verification: invalid compressed geometry.') from error
    require(len(raw) == vertices * 8 + triangles * 6, 'payload length mismatch')
    offset = 0
    for part in parts:
        offset += part['n'] * 6
        index = 0
        for (delta,) in struct.iter_unpack('<H', raw[offset:offset + part['m'] * 6]):
            index = (index + delta) & 65535
            require(index < part['n'], 'triangle index outside its disk')
        offset += part['m'] * 6 + part['n'] * 2
    require(data.get('excludedLegacy') == [{'seg': 'neckB', 'name': 'Межпозвонковый диск C2–C1',
             'reason': 'There is no intervertebral disk between the atlas and axis.'}], 'unexpected legacy replacement')
    return data
