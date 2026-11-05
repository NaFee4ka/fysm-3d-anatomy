"""Validate the exact local animation packet before embedding it in the page.

The witness covers the stated geometric checks. It does not certify dynamic
balance, instructional technique, or collision freedom of all soft tissues.
"""
import json
import math
import re
from collections import defaultdict, deque
from build_verification import current_sources, digest

TRANSITION_INPUTS = (
    'anatomy3.js', 'poses.json', 'practice/poses.json', 'practice/algorithms.json',
    'transition_runtime.mjs', 'transition_build.mjs', 'muscle_surface_render.js',
    'muscles/muscles_bp3d.json', 'skull/bones_bp3d.json', 'body_collision.mjs',
    'practice/arm_foot_collision.mjs', 'whole_body_collision.mjs', 'package-lock.json',
    'transition_floor.mjs', 'practice/ground_fit.mjs', 'practice/contacts.mjs',
    'cycle_lib.mjs', 'collision_geometry.mjs',
    'transition_collision.mjs',
    'practice/kinematics_helpers.mjs', 'seg5_util.mjs',
    'transition_grip.mjs',
    'transition_path.mjs',
    'transition_toe_oracle.mjs',
    'transition_fold_grip.mjs',
    'transition_motion.mjs',
    'transition_bow_release.mjs',
    'transition_thumb_release.mjs',
    'transition_spider_release.mjs',
    'transition_hip_detour.mjs',
    'transitions/plan.json.gz',
)


def verify_transitions(root):
    def require(condition, message):
        if not condition:
            raise SystemExit('Transition verification: ' + message + '. Run node transition_build.mjs.')

    def number(value):
        return type(value) in (int, float) and math.isfinite(value)

    def vector(value, count):
        return isinstance(value, list) and len(value) == count and all(number(n) for n in value)

    try:
        packet = json.loads((root / 'transitions/data.json').read_text())
        witness = json.loads((root / 'transitions/check.json').read_text())
    except (OSError, ValueError) as error:
        raise SystemExit('Transition data is missing or invalid. Run node transition_build.mjs.') from error
    expected_ids = {'dog', 'plank', 'half', 'bow', 'fold', 'handstand'}
    expected_ids.update(json.loads((root / 'practice/poses.json').read_text())['poses'])
    require(packet.get('schemaVersion') == 1 and packet.get('verified') is True
            and packet.get('kind') == 'kinematic-demonstration', 'invalid packet kind/status')
    require(witness.get('schemaVersion') == 1 and witness.get('verified') is True
            and witness.get('kind') == 'kinematic-demonstration'
            and witness.get('dataSha256') == digest(root / 'transitions/data.json'), 'missing or stale witness')
    builder = (root / 'transition_build.mjs').read_text()
    direct_imports = re.findall(r"^import\b[^;]*?\bfrom\s*['\"](\./[^'\"]+)['\"]\s*;", builder, re.M)
    direct_imports += re.findall(r"^import\s*['\"](\./[^'\"]+)['\"]\s*;", builder, re.M)
    required_sources = set(TRANSITION_INPUTS) | {name[2:] for name in direct_imports}
    require(packet.get('sourceHashes') == witness.get('sourceHashes')
            and current_sources(root, packet.get('sourceHashes'), required_sources), 'stale source files')
    provenance = packet.get('planProvenance')
    require(isinstance(provenance, dict) and provenance == witness.get('planProvenance')
            and provenance.get('mode') in ('unverified-coordinate-input', 'regenerated-from-poses')
            and provenance.get('candidateFile') == 'transitions/plan.json.gz'
            and provenance.get('candidateSha256') == packet['sourceHashes']['transitions/plan.json.gz']
            and provenance.get('currentProducerSha256') == packet['sourceHashes']['transition_build.mjs']
            and isinstance(provenance.get('originProducerSha256'), str)
            and re.fullmatch('[0-9a-f]{64}', provenance['originProducerSha256'])
            and provenance.get('priorVerificationTrusted') is False
            and (provenance['mode'] != 'regenerated-from-poses'
                 or provenance['originProducerSha256'] == provenance['currentProducerSha256']),
            'invalid motion candidate provenance or trusted prior verification')
    require(all(packet.get('checks', {}).get(k) is True and witness.get('checks', {}).get(k) is True
                for k in ('oppositeArms', 'armsFeet', 'floor', 'criticalRegions', 'elbowMotion')),
            'incomplete geometric checks')
    motion = witness.get('jointMotion', {})
    require(packet.get('jointMotion') == motion and motion.get('verified') is True
            and motion.get('scope') == 'bilateral-elbow-flexion' and motion.get('substeps') == 20
            and motion.get('modelExtensionLimitDeg') == 5 and motion.get('modelFlexionLimitDeg') == 150
            and number(motion.get('angularToleranceDeg')) and 0 <= motion['angularToleranceDeg'] <= 1e-5
            and all(number(motion.get(key, {}).get(side)) for key in ('minimumFlexDeg', 'maximumFlexDeg')
                    for side in ('L', 'R'))
            and all(-5 - motion['angularToleranceDeg'] <= motion['minimumFlexDeg'][side]
                    <= motion['maximumFlexDeg'][side] <= 150 + motion['angularToleranceDeg'] for side in ('L', 'R')),
            'invalid elbow motion audit')
    critical = witness.get('criticalCollision', {})
    categories = critical.get('categories')
    require(critical.get('verified') is True
            and critical.get('scope') == 'transition-critical-bone-pairs-v2'
            and critical.get('substeps') == 20 and critical.get('boneParts') == 216
            and critical.get('wholePairExclusions') == []
            and critical.get('articulationPolicy') == 'bounded-joint-local-only'
            and isinstance(categories, list) and len(categories) == 6
            and all(isinstance(category, str) for category in categories)
            and set(categories) == {'all-arm-leg-pairs', 'limbs-axial-body-including-head',
                                    'all-leg-pairs', 'distal-fingers-own-palms',
                                    'own-forearm-first-metacarpals', 'scapulae-third-thoracic-vertebra'},
            'missing regional collision checks or changed articulation policy')
    require(witness.get('fullBodyCollisionFree') is False and witness.get('dynamicBalanceVerified') is False,
            'unsupported whole-body or dynamic certificate')
    require(witness.get('floorScope') == 'all-bones-outer-muscles-skin-markers-and-explicit-proxies'
            and all(number(witness.get(k)) for k in ('floorMinimumM', 'floorMaximumM', 'maxLiftM'))
            and -2e-6 <= witness['floorMinimumM'] <= witness['floorMaximumM'] <= .004
            and 0 <= witness['maxLiftM'] <= .025
            and type(witness.get('collisionSubsteps')) is int and witness['collisionSubsteps'] >= 20,
            'invalid geometric sampling report')
    poses, names = packet.get('poses'), packet.get('segNames')
    require(isinstance(poses, list) and len(poses) == 30 and all(isinstance(p, dict) for p in poses)
            and {p.get('id') for p in poses} == expected_ids, 'expected all 30 asanas')
    practice_catalog = {step['id']: (algorithm['id'], index)
                        for algorithm in json.loads((root / 'practice/algorithms.json').read_text())['algorithms']
                        for index, step in enumerate(algorithm['steps'])}
    require(all(isinstance(p.get('name'), str) and p['name'].strip()
                and (p.get('kind') == 'practice' and p.get('algorithmId') == practice_catalog[p['id']][0]
                     and type(p.get('stepIndex')) is int and p['stepIndex'] == practice_catalog[p['id']][1]
                     if p['id'] in practice_catalog else p.get('kind') == 'pose') for p in poses),
            'invalid pose catalog metadata')
    canonical_names = list(json.loads((root / 'poses.json').read_text())['dog']['local'])
    require(isinstance(names, list) and len(names) == 59 and names == canonical_names,
            'invalid segment list/order')

    def frame(value):
        return (isinstance(value, dict) and number(value.get('t'))
                and vector(value.get('rootP'), 3) and vector(value.get('root'), 4)
                and all(abs(n) <= 10 for n in value['rootP'])
                and abs(math.hypot(*value['root']) - 1) < 1e-6
                and isinstance(value.get('local'), dict) and set(value['local']) == set(names)
                and all(vector(q, 4) and abs(math.hypot(*q) - 1) < 1e-6 for q in value['local'].values())
                and number(value.get('floorLiftM', 0)) and 0 <= value.get('floorLiftM', 0) <= .05)

    def same_conf(a, b):
        # q and -q represent the same rigid rotation.
        rotations = [(a['root'], b['root'])] + [(a['local'][n], b['local'][n]) for n in names]
        return (max(abs(x-y) for x, y in zip(a['rootP'], b['rootP'])) < 1e-8
                and all(min(math.dist(x, y), math.dist(x, [-v for v in y])) < 1e-8 for x, y in rotations))

    endpoints, tracks = packet.get('endpoints'), packet.get('tracks')
    require(isinstance(endpoints, dict) and expected_ids.issubset(endpoints)
            and all(frame(f) for f in endpoints.values()), 'invalid endpoints')
    require(isinstance(tracks, dict) and len(tracks) > 0, 'missing tracks')
    adjacency = defaultdict(set)
    interval_count = 0
    maximum_frame_lift = 0
    angular_step_cosine_squared = math.cos(.040001 / 2) ** 2
    for key, track in tracks.items():
        require(isinstance(track, dict) and track.get('id') == key, 'track ID mismatch')
        start, end, duration = track.get('from'), track.get('to'), track.get('duration')
        require(start in endpoints and end in endpoints and start != end
                and number(duration) and duration > 0, 'invalid track ends or duration')
        frames = track.get('frames')
        require(isinstance(frames, list) and len(frames) >= 2 and all(frame(f) for f in frames)
                and frames[0]['t'] == 0 and abs(frames[-1]['t'] - duration) < 1e-8
                and all(a['t'] < b['t'] for a, b in zip(frames, frames[1:])), 'invalid frame sequence')
        for a, b in zip(frames, frames[1:]):
            for qa, qb in [(a['root'], b['root'])] + [(a['local'][n], b['local'][n]) for n in names]:
                dot = sum(x * y for x, y in zip(qa, qb))
                require(dot * dot >= angular_step_cosine_squared * sum(x * x for x in qa) * sum(x * x for x in qb),
                        'saved angular interval exceeds the audited sampling density')
        require(same_conf(frames[0], endpoints[start]) and same_conf(frames[-1], endpoints[end]),
                'track endpoint discontinuity')
        interval_count += len(frames) - 1
        maximum_frame_lift = max(maximum_frame_lift, *(f.get('floorLiftM', 0) for f in frames))
        phases = track.get('phases')
        require(isinstance(phases, list) and len(phases) > 0
                and all(isinstance(p, dict) and number(p.get('t0')) and number(p.get('t1'))
                        and 0 <= p['t0'] < p['t1'] <= duration + 1e-8
                        and isinstance(p.get('label'), str) and p['label'].strip()
                        and ('reverseLabel' not in p or isinstance(p['reverseLabel'], str)
                             and p['reverseLabel'].strip()) for p in phases)
                and phases[0]['t0'] == 0 and abs(phases[-1]['t1'] - duration) < 1e-8
                and all(abs(a['t1'] - b['t0']) <= 1e-8 for a, b in zip(phases, phases[1:])), 'invalid phases')
        adjacency[start].add(end)
        adjacency[end].add(start)
    for start in expected_ids:
        seen, queue = {start}, deque([start])
        while queue:
            for end in adjacency[queue.popleft()] - seen:
                seen.add(end)
                queue.append(end)
        require(expected_ids.issubset(seen), 'disconnected pose graph')
    require(witness.get('poseCount') == 30 and witness.get('trackCount') == len(tracks)
            and witness.get('routeCount') == 870, 'incomplete route inventory')
    require(maximum_frame_lift <= witness['maxLiftM'] + 2e-9,
            'floor envelope exceeds the audited maximum')
    require(witness.get('floorAuditSubsteps') == [2, 5, 7, 10, 13, 15, 18]
            and type(witness.get('floorStates')) is int and witness['floorStates'] >= interval_count * 7
            and type(witness.get('collisionStates')) is int
            and witness['collisionStates'] >= interval_count * 19
            and type(witness.get('finalCollisionStates')) is int
            and witness['finalCollisionStates'] == interval_count * 20 + len(tracks)
            and type(critical.get('states')) is int and critical['states'] == witness['finalCollisionStates'],
            'incomplete interval sampling')
    require(type(motion.get('states')) is int and motion['states'] == interval_count * 20 + len(tracks),
            'incomplete elbow range sampling')
    return packet
