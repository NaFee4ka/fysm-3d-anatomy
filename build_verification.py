"""Read-only release checks. Numerical data must match the checked-in sources."""
import hashlib
import json
import math
from pathlib import PurePath

PRACTICE_GEOMETRY_INPUTS = (
    'anatomy3.js', 'cycle_lib.mjs', 'seg5_util.mjs', 'poses_build.mjs', 'package-lock.json',
    'practice_build.mjs', 'practice/algorithms.json', 'practice/struna.mjs',
    'practice/detali.mjs', 'practice/centering.mjs', 'practice/kinematics_helpers.mjs',
    'practice/ground_fit.mjs', 'practice/contacts.mjs', 'muscle_surface_render.js',
    'muscles/muscles_bp3d.json', 'body_collision.mjs', 'collision_geometry.mjs',
    'skull/bones_bp3d.json', 'practice/arm_foot_collision.mjs',
)
PRACTICE_PHYSICS_INPUTS = (
    'practice/poses.json', 'practice/algorithms.json', 'practice/contacts.mjs',
    'practice_physics_build.mjs', 'anatomy3.js', 'physics_core.mjs', 'physics_params.mjs',
    'muscle_surface_render.js', 'muscles/muscles_bp3d.json', 'skull/bones_bp3d.json', 'package-lock.json',
)
STATIC_PHYSICS_INPUTS = (
    'anatomy3.js', 'physics_core.mjs', 'physics_params.mjs', 'physics_build.mjs',
    'poses.json', 'physics.json', 'package-lock.json', 'physics_sensitivity_build.mjs',
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def current_sources(root, hashes, required):
    if not isinstance(hashes, dict) or not set(required).issubset(hashes):
        return False
    for name, value in hashes.items():
        relative = PurePath(name)
        path = root / name
        if relative.is_absolute() or '..' in relative.parts or not path.is_file() or value != digest(path):
            return False
    return True


def physical_solution(diag):
    quality, physical = diag.get('quality', {}), diag.get('physicalBalance', {})
    numbers = ('forceResidualN', 'momentResidualNm', 'maxAssemblyDifference', 'forceToleranceN', 'momentToleranceNm')
    q_numbers = ('primalResidual', 'dualResidual', 'equalityViolation', 'inequalityViolation',
                 'boundViolation', 'primalTolerance', 'dualTolerance')
    return (quality.get('status') == 'solved' and quality.get('converged') is True
            and quality.get('finite') is True and quality.get('physicalBalanceVerified') is True
            and physical.get('verified') is True and physical.get('assemblyVerified') is True
            and all(isinstance(physical.get(k), (int, float)) and math.isfinite(physical[k])
                    and physical[k] >= 0 for k in numbers)
            and physical['forceResidualN'] <= physical['forceToleranceN']
            and physical['momentResidualNm'] <= physical['momentToleranceNm']
            and all(isinstance(quality.get(k), (int, float)) and math.isfinite(quality[k])
                    and quality[k] >= 0 for k in q_numbers)
            and quality['primalResidual'] <= quality['primalTolerance']
            and quality['dualResidual'] <= quality['dualTolerance']
            and all(quality[k] <= quality['primalTolerance']
                    for k in ('equalityViolation', 'inequalityViolation', 'boundViolation'))
            and physical['maxAssemblyDifference'] <= 1e-7)


def verify_static_physics(root):
    baseline = json.loads((root / 'physics.json').read_text())
    scenarios = json.loads((root / 'research/2025-2026/sensitivity.json').read_text())
    ids = {'stand', 'dog', 'plank', 'half', 'bow', 'fold', 'handstand'}
    if not (set(baseline) == ids and set(scenarios.get('poses', {})) == ids
            and current_sources(root, scenarios.get('provenance', {}).get('sourceSha256'), STATIC_PHYSICS_INPUTS)
            and all(physical_solution(p.get('diag', {})) for p in baseline.values())
            and all(physical_solution({'quality': p.get('quality', {}).get(s, {}),
                                      'physicalBalance': p.get('physicalBalance', {}).get(s, {})})
                    for p in scenarios['poses'].values() for s in ('cue', 'noCue'))):
        raise SystemExit('Static physics verification is missing or stale. Rebuild physics and sensitivity before building.')


def verify_practice_physics(root, ids):
    data = json.loads((root / 'practice/physics.json').read_text())
    witness = json.loads((root / 'practice/physics-check.json').read_text())
    pose_sha = digest(root / 'practice/poses.json')
    if not (data.get('schemaVersion') == 1 and data.get('method') == 'quasistatic-static-optimization'
            and data.get('inputProvenance', {}).get('repositoryBound') is True
            and data.get('verified') is True and set(data.get('poses', {})) == set(ids)
            and data.get('posesSha256') == pose_sha and witness.get('posesSha256') == pose_sha
            and witness.get('schemaVersion') == 1 and witness.get('verified') is True
            and witness.get('poseCount') == 24 and witness.get('physicsSha256') == digest(root / 'practice/physics.json')
            and witness.get('sourceHashes') == data.get('sourceHashes')
            and current_sources(root, data.get('sourceHashes'), PRACTICE_PHYSICS_INPUTS)
            and all(physical_solution(p.get('diag', {})) and p['diag'].get('contactGeometryVerified') is True
                    for p in data['poses'].values())):
        raise SystemExit('Practice physics verification is missing or stale. Run node practice_physics_build.mjs before building.')
    for p in data['poses'].values():
        sensitivity = p.get('modelLimits', {}).get('softTissueContactSensitivity')
        if sensitivity and not (sensitivity.get('verified') is True and len(sensitivity.get('hypotheses', [])) == 2
                                and all(h.get('contactGeometryVerified') is True and physical_solution(h)
                                        for h in sensitivity['hypotheses'])):
            raise SystemExit('Practice soft-tissue contact sensitivity is not verified.')
    sensitivity = data['poses']['struna-08'].get('modelLimits', {}).get('softTissueContactSensitivity', {})
    if not (sensitivity.get('verified') is True
            and sensitivity.get('kind') == 'two-deterministic-geometric-area-hypotheses'
            and sensitivity.get('isConfidenceInterval') is False
            and [h.get('areaScale') for h in sensitivity.get('hypotheses', [])] == [1, .5]):
        raise SystemExit('Practice soft-tissue contact sensitivity is missing or incomplete.')
    return data


def verify_cycle_physics(root, cycle):
    quality = cycle.get('modelQuality') or {}
    required = ('anatomy3.js', 'physics_core.mjs', 'physics_params.mjs', 'cycle_phys.mjs', 'cycle_quality.mjs',
                'cycle_keys/cycle.json', 'poses.json', 'package-lock.json')
    if not (quality.get('verified') is True and quality.get('fullCoverage') is True
            and quality.get('physicalBalanceVerified') is True and quality.get('sourcesVerified') is True
            and quality.get('physicsSha256') == digest(root / 'cycle_phys.json')
            and quality.get('statsSha256') == digest(root / 'cycle_stats.json')
            and current_sources(root, quality.get('statsSourceSha256'), ('cycle_stats.mjs',))
            and current_sources(root, quality.get('sourceSha256'), required)):
        raise SystemExit('Round physics verification is missing or stale. Run ./cycle_run.sh before building.')
