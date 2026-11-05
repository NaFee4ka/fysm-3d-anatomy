"""Exercise the real Python build with corrupted animation release packets."""
import copy
import hashlib
import json
import math
import pathlib
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent


def main():
    from transition_release import verify_transitions
    packet = verify_transitions(ROOT)
    witness = json.loads((ROOT / 'transitions/check.json').read_text())
    pose_id = packet['poses'][0]['id']
    track_id = next(iter(packet['tracks']))
    segment = packet['segNames'][-1]

    def disconnect(data, check):
        data['tracks'] = {k: v for k, v in data['tracks'].items()
                          if pose_id not in (v['from'], v['to'])}
        check['trackCount'] = len(data['tracks'])

    def stale(data, check):
        data['sourceHashes']['transition_runtime.mjs'] = '0' * 64
        check['sourceHashes'] = copy.deepcopy(data['sourceHashes'])

    def omit_helper(data, check):
        data['sourceHashes'].pop('transition_thumb_release.mjs')
        check['sourceHashes'] = copy.deepcopy(data['sourceHashes'])

    def excessive_translation(data, check):
        for frame in list(data['endpoints'].values()) + [frame for track in data['tracks'].values()
                                                        for frame in track['frames']]:
            frame['rootP'][0] += 20

    def omit_motion_input(data, check):
        data['sourceHashes'].pop('transitions/plan.json.gz')
        check['sourceHashes'] = copy.deepcopy(data['sourceHashes'])

    def provenance_field(key, value):
        def mutate(data, check):
            data['planProvenance'][key] = value
            check['planProvenance'][key] = value
        return mutate

    def angular_jump(data, check):
        q = data['tracks'][track_id]['frames'][1]['local'][segment]
        x, y, z, w = q
        s = math.sqrt(.5)
        q[:] = [s * (x + w), s * (y - z), s * (z + y), s * (w - x)]

    cases = {
        'unverified packet': lambda d, c: d.update(verified=False),
        'missing asana': lambda d, c: d['poses'].pop(),
        'missing pose name': lambda d, c: d['poses'][0].pop('name'),
        'invalid pose kind': lambda d, c: d['poses'][0].update(kind='unknown'),
        'invalid practice step': lambda d, c: next(p for p in d['poses'] if p['kind'] == 'practice').update(stepIndex=99),
        'reordered segment list': lambda d, c: d['segNames'].reverse(),
        'unverified floor': lambda d, c: d['checks'].update(floor=False),
        'unverified regional checks': lambda d, c: d['checks'].update(criticalRegions=False),
        'unverified elbow motion': lambda d, c: d['checks'].update(elbowMotion=False),
        'unverified elbow audit': lambda d, c: c['jointMotion'].update(verified=False),
        'unbounded joint exception': lambda d, c: c['criticalCollision'].update(wholePairExclusions=['thigh-shank']),
        'obsolete regional collision scope': lambda d, c: c['criticalCollision'].update(
            scope='arms-legs-axial-all-legs-distal-fingers-palms'),
        'missing forearm-thumb collision category': lambda d, c: c['criticalCollision']['categories'].remove(
            'own-forearm-first-metacarpals'),
        'missing scapula-Th3 collision category': lambda d, c: c['criticalCollision']['categories'].remove(
            'scapulae-third-thoracic-vertebra'),
        'duplicated regional collision category': lambda d, c: c['criticalCollision']['categories'].append(
            'all-arm-leg-pairs'),
        'unsupported full-body claim': lambda d, c: c.update(fullBodyCollisionFree=True),
        'missing interval samples': lambda d, c: c.update(collisionStates=0),
        'missing final interval audit': lambda d, c: c.pop('finalCollisionStates'),
        'inflated final interval audit': lambda d, c: c.update(finalCollisionStates=c['finalCollisionStates'] + 1),
        'missing floor samples': lambda d, c: c.update(floorStates=0),
        'missing floor audit schedule': lambda d, c: c.pop('floorAuditSubsteps'),
        'narrowed floor audit schedule': lambda d, c: c.update(floorAuditSubsteps=[5, 10, 15]),
        'insufficient seven-step floor audit': lambda d, c: c.update(
            floorStates=6 * sum(len(t['frames']) - 1 for t in d['tracks'].values())),
        'floor penetration report': lambda d, c: c.update(floorMinimumM=-.001),
        'stale runtime source': stale,
        'omitted imported helper': omit_helper,
        'omitted motion candidate input': omit_motion_input,
        'trusted prior candidate verification': provenance_field('priorVerificationTrusted', True),
        'unknown candidate provenance mode': provenance_field('mode', 'trusted-cache'),
        'wrong candidate provenance path': provenance_field('candidateFile', 'poses.json'),
        'wrong candidate provenance digest': provenance_field('candidateSha256', '0' * 64),
        'wrong candidate current producer': provenance_field('currentProducerSha256', '0' * 64),
        'invalid candidate origin producer': provenance_field('originProducerSha256', 'unknown'),
        'coarse angular sampling despite normalized frames': angular_jump,
        'translation beyond runtime bounds': excessive_translation,
        'invalid floor envelope': lambda d, c: d['tracks'][track_id]['frames'][0].update(floorLiftM=float('nan')),
        'excessive floor envelope': lambda d, c: d['tracks'][track_id]['frames'][0].update(floorLiftM=.051),
        'envelope beyond audited maximum': lambda d, c: d['tracks'][track_id]['frames'][0].update(floorLiftM=.03),
        'missing segment': lambda d, c: d['tracks'][track_id]['frames'][1]['local'].pop(segment),
        'reversed frame time': lambda d, c: d['tracks'][track_id]['frames'][1].update(t=-1),
        'invalid phase': lambda d, c: d['tracks'][track_id]['phases'][0].update(t1=-1),
        'uncovered phase start': lambda d, c: d['tracks'][track_id]['phases'][0].update(t0=.001),
        'disconnected route graph': disconnect,
        'endpoint jump': lambda d, c: d['tracks'][track_id]['frames'][0]['rootP'].__setitem__(0,
                         d['tracks'][track_id]['frames'][0]['rootP'][0] + .01),
    }
    with tempfile.TemporaryDirectory(prefix='fysm-transition-build-') as temporary:
        fixture = pathlib.Path(temporary)
        # Only the two intentionally damaged packets are writable copies. Other
        # source/data files stay linked read-only by convention; never write a link.
        for source in ROOT.iterdir():
            if source.name in ('.git', 'transitions', 'dist', 'node_modules', '__pycache__'):
                continue
            if source.name == 'build.py':
                shutil.copy2(source, fixture / source.name)
            else:
                (fixture / source.name).symlink_to(source, target_is_directory=source.is_dir())
        (fixture / 'transitions').mkdir()
        (fixture / 'transitions/plan.json.gz').symlink_to(ROOT / 'transitions/plan.json.gz')
        (fixture / 'transitions/data.json').write_bytes((ROOT / 'transitions/data.json').read_bytes())
        (fixture / 'transitions/check.json').write_bytes((ROOT / 'transitions/check.json').read_bytes())
        verify_transitions(fixture)
        for label, mutate in cases.items():
            data, check = copy.deepcopy(packet), copy.deepcopy(witness)
            mutate(data, check)
            encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':')).encode()
            # Refresh the packet hash so shape/topology checks, not just a hash
            # mismatch, must reject a damaged but apparently fresh export.
            check['dataSha256'] = hashlib.sha256(encoded).hexdigest()
            (fixture / 'transitions/data.json').write_bytes(encoded)
            (fixture / 'transitions/check.json').write_text(json.dumps(check))
            output = fixture / 'rejected-output'
            result = subprocess.run([sys.executable, str(fixture / 'build.py'), str(output)],
                                    capture_output=True, text=True, timeout=30)
            assert result.returncode != 0, 'Build accepted ' + label
            assert 'Transition verification:' in result.stdout + result.stderr, (label, result.stderr)
            assert not output.exists(), 'Build wrote an artifact after ' + label
            print('PASS:', label)
    print(f'PASS: current packet + {len(cases)} actual-build rejection cases')


if __name__ == '__main__':
    main()
