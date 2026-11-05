#!/usr/bin/env python3
"""Reject a source edit during disk generation without touching the repository.

python3 joints/spine_discs_freshness_test.py --archive /path/partof_BP3D_4.0_obj_99.zip
"""
import argparse
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    builder = ROOT / 'joints/spine_discs_build.py'
    spec = importlib.util.spec_from_file_location('disk_builder_test', builder)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    before = {name: module.sha(ROOT / name) for name in module.DEPENDENCIES}
    script = r'''
import importlib.util
from pathlib import Path
import sys
spec = importlib.util.spec_from_file_location('disk_builder_fixture', sys.argv[1])
b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
b.ROOT = Path(sys.argv[2])
original = b.source_obj
called = False
def mutate_after_registration_was_read(raw):
    global called
    if not called:
        called = True
        # Whitespace changes the exact bytes but not the parsed registration.
        path = b.ROOT / 'muscles/coverage.json'
        path.write_bytes(path.read_bytes() + b'\n')
    return original(raw)
b.source_obj = mutate_after_registration_was_read
try:
    b.build(Path(sys.argv[3]))
except ValueError as error:
    if 'Spine disk sources changed during build: muscles/coverage.json' not in str(error):
        raise
    print('PASS: changed registration rejected before returning publishable data')
else:
    raise AssertionError('Builder certified geometry after its registration source changed')
assert not (b.ROOT / 'joints/spine-discs.json').exists()
assert not (b.ROOT / 'joints/spine-discs-report.json').exists()
'''
    with tempfile.TemporaryDirectory(prefix='fysm-spine-source-race-') as directory:
        destination = Path(directory)
        for name in module.DEPENDENCIES:
            path = destination / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, path)
        result = subprocess.run([sys.executable, '-c', script, str(builder), str(destination), str(args.archive.resolve())],
                                capture_output=True, text=True, check=True)
        print(result.stdout.strip())
    assert before == {name: module.sha(ROOT / name) for name in module.DEPENDENCIES}, 'Test changed a real repository input'


if __name__ == '__main__':
    main()
