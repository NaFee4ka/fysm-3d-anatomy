"""Check that the actual atlas release guard rejects damaged local exports."""
import base64
import copy
import gzip
import hashlib
import json
from pathlib import Path
import struct
import tempfile
from spine_release import verify_spine_surfaces

ROOT = Path(__file__).resolve().parent


def main():
    original = verify_spine_surfaces(ROOT)
    original_report = json.loads((ROOT / 'joints/spine-discs-report.json').read_text())

    def bad_index(data, report):
        raw = bytearray(gzip.decompress(base64.b64decode(data['data'])))
        struct.pack_into('<H', raw, data['parts'][0]['n'] * 6, 65535)
        data['data'] = base64.b64encode(gzip.compress(raw)).decode()

    def stale_source(data, report):
        data['sourceHashes']['spine_surface_render.js'] = '0' * 64
        report['sourceHashes'] = copy.deepcopy(data['sourceHashes'])

    cases = {
        'anatomically absent C1-C2 disk': lambda d, r: d['parts'][0].update(level='C1-C2'),
        'missing source mapping': lambda d, r: d['parts'][0].update(sourceIds=[]),
        'invalid bounding box': lambda d, r: d['parts'][0].update(hi=d['parts'][0]['lo'][:]),
        'missing skeletal anchor': lambda d, r: d['parts'][0]['inferior'].update(seg='absent'),
        'missing display name': lambda d, r: d['parts'][0].pop('name'),
        'blank display name': lambda d, r: d['parts'][0].update(name=''),
        'missing legacy replacement list': lambda d, r: d['parts'][0].pop('replaces'),
        'empty legacy replacement list': lambda d, r: d['parts'][0].update(replaces=[]),
        'invalid legacy replacement segment': lambda d, r: d['parts'][0]['replaces'][0].update(seg='absent'),
        'missing legacy replacement name': lambda d, r: d['parts'][0]['replaces'][0].pop('name'),
        'stale renderer source': stale_source,
        'invalid compressed mesh': lambda d, r: d.update(data='broken!'),
        'out-of-range triangle index': bad_index,
    }
    with tempfile.TemporaryDirectory(prefix='fysm-spine-guard-') as temporary:
        fixture = Path(temporary)
        for name in original['sourceHashes']:
            target = fixture / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(ROOT / name)
        for label, mutate in cases.items():
            data, report = copy.deepcopy(original), copy.deepcopy(original_report)
            mutate(data, report)
            encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':')).encode()
            report['dataSha256'] = hashlib.sha256(encoded).hexdigest()
            for name, value in [('spine-discs.json', encoded),
                                ('spine-discs-report.json', json.dumps(report).encode())]:
                target = fixture / 'joints' / name
                assert target.parent.resolve().is_relative_to(fixture.resolve()) and not target.is_symlink()
                target.write_bytes(value)
            try:
                verify_spine_surfaces(fixture)
            except SystemExit as error:
                assert str(error).startswith('Spine surface verification:'), (label, error)
            else:
                raise AssertionError('Guard accepted ' + label)
            print('PASS:', label)
    print(f'PASS: current atlas package + {len(cases)} corrupted-export cases')


if __name__ == '__main__':
    main()
