import re, sys, pathlib
# usage: python3 build.py [OUTDIR]   (default dist; the round comes from cycle_data.json when it exists)
root = pathlib.Path(__file__).resolve().parent
outdir = root / (sys.argv[1] if len(sys.argv) > 1 else 'dist')
anat = (root / 'anatomy3.js').read_text().replace('export function createAnatomy', 'function createAnatomy')
import os
cyc_file = pathlib.Path(os.environ.get('CYCLE_DATA', str(root / 'cycle_data.json')))
cycle = cyc_file.read_text() if cyc_file.exists() else 'null'
import json
import hashlib
import base64
import math

# New practice poses have their own data, independent of the six calculated
# poses and the verified round. Never attach another pose's force estimates.
practice = json.loads((root / 'practice/algorithms.json').read_text(encoding='utf-8'))
practice_poses = json.loads((root / 'practice/poses.json').read_text(encoding='utf-8'))
practice_check = json.loads((root / 'practice/check.json').read_text(encoding='utf-8'))
practice_inputs = ('anatomy3.js', 'cycle_lib.mjs', 'seg5_util.mjs', 'poses_build.mjs', 'package-lock.json', 'practice_build.mjs',
                   'practice/algorithms.json', 'practice/struna.mjs', 'practice/detali.mjs', 'practice/centering.mjs',
                   'body_collision.mjs', 'collision_geometry.mjs', 'skull/bones_bp3d.json', 'practice/arm_foot_collision.mjs')
practice_ids = [step['id'] for algorithm in practice['algorithms'] for step in algorithm['steps']]
if not (practice.get('schemaVersion') == 1 and practice_poses.get('schemaVersion') == 1
        and practice_poses.get('kind') == 'static-photo-reconstructions'
        and len(practice['algorithms']) == 3 and all(len(a['steps']) == 8 for a in practice['algorithms'])
        and len(set(practice_ids)) == 24 and set(practice_poses.get('poses', {})) == set(practice_ids)
        and practice_poses.get('sourceSha256') == practice['source']['sha256']
        and practice_check.get('verified') is True and practice_check.get('scope') == 'opposite-arm-bones'
        and practice_check.get('poseCount') == 24 and practice_check.get('boneParts') == 60
        and practice_check.get('armFootCheck') == {'verified': True, 'scope': 'arm-foot-bones', 'poseCount': 24, 'pairGroups': 4}
        and practice_check.get('posesSha256') == hashlib.sha256((root / 'practice/poses.json').read_bytes()).hexdigest()
        and practice_check.get('sourceHashes') == practice_poses.get('sourceHashes')
        and all(practice_poses.get('sourceHashes', {}).get(name) == hashlib.sha256((root / name).read_bytes()).hexdigest()
                for name in practice_inputs)):
    raise SystemExit('Practice geometry is missing or stale. Run node practice_build.mjs before building.')
for pose_id, pose in practice_poses['poses'].items():
    rotations = list(pose.get('local', {}).values()) + [pose.get('root')]
    if not (len(pose.get('local', {})) == 59
            and all(isinstance(q, list) and len(q) == 4
                    and all(isinstance(n, (int, float)) and math.isfinite(n) for n in q)
                    and abs(math.hypot(*q) - 1) < 1e-7 for q in rotations)
            and len(pose.get('rootP', [])) == 3 and all(math.isfinite(n) for n in pose['rootP'])):
        raise SystemExit('Invalid practice geometry: ' + pose_id)
practice['poses'] = practice_poses['poses']
for algorithm in practice['algorithms']:
    reference = (root / ('practice/reference-' + algorithm['id'] + '.webp')).read_bytes()
    algorithm['referenceImage'] = 'data:image/webp;base64,' + base64.b64encode(reference).decode('ascii')
    algorithm['referenceAlt'] = algorithm['name'] + ': восемь исходных фотографий, страница ' + str(algorithm['page']) + ' PDF'
if cycle != 'null':
    cycle_object = json.loads(cycle)
    collision = cycle_object.get('collisionCheck') or {}
    geometry = [cycle_object.get(key) for key in ('n', 'fps', 'segNames', 'q', 'root', 'p')]
    geometry_sha = hashlib.sha256(json.dumps(geometry, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()
    required_sources = ('anatomy3.js', 'collision_geometry.mjs', 'body_collision.mjs', 'cycle_collision_check.mjs', 'skull/bones_bp3d.json')
    source_hashes = collision.get('sourceSha256') or {}
    if not (collision.get('verified') is True and collision.get('scope') == 'opposite-arm-bones'
            and collision.get('substeps', 0) >= 20 and collision.get('frameCount') == cycle_object.get('n')
            and collision.get('states') == (cycle_object.get('n', 0) - 1) * collision.get('substeps', 0) + 1
            and collision.get('geometrySha256') == geometry_sha
            and all(source_hashes.get(filename) == hashlib.sha256((root / filename).read_bytes()).hexdigest()
                    for filename in required_sources)):
        raise SystemExit('Round collision verification is missing or stale. Run node cycle_pack.mjs before building.')
fl = (root / 'fascia3.js').read_text().replace('export function', 'function').replace('export const', 'const')
fr = (root / 'fascia_render.js').read_text().replace('export function', 'function')
fascia_lib = 'const FASCIA_LIB = (() => {\n' + fl + '\nreturn { createFascia, LINE_COLORS };\n})();\n' + fr
def inline_factory(filename, factory):
    source = (root / filename).read_text(encoding='utf-8')
    source = re.sub(r'^export\s+(?=(?:async\s+)?(?:function|const|class)\b)', '', source, flags=re.M)
    return f'const {factory} = (() => {{\n{source}\nreturn {factory};\n}})();\n'

muscle_lib = inline_factory('muscle_surface_render.js', 'createMuscleSurfaces')
connective_lib = inline_factory('connective_surface_render.js', 'createConnectiveSurfaces')
joint_lib = inline_factory('joint_surface_render.js', 'createJointSurfaces')
muscle_data = (root / 'muscles' / 'muscles_bp3d.json').read_text(encoding='utf-8')
joint_data = (root / 'joints' / 'joints.json').read_text(encoding='utf-8')
capsule_rims = (root / 'joints' / 'capsule-rims.json').read_text(encoding='utf-8')
research_sensitivity = (root / 'research' / '2025-2026' / 'sensitivity.json').read_text(encoding='utf-8')
fmodel = (root / 'books' / 'fascia_model.json').read_text()
ml = json.loads((root / 'books' / 'myers_lines.json').read_text())
notes = json.dumps({l['id']: {k: v.get('ru', '') for k, v in (l.get('poses') or {}).items()} for l in ml['lines']}, ensure_ascii=False)
app = (root / 'app3.js').read_text().replace('/*__POSES__*/null', (root / 'poses.json').read_text()).replace('/*__PHYSICS__*/null', (root / 'physics.json').read_text()).replace('/*__CYCLE__*/null', cycle).replace('/*__ANATOMY__*/', anat).replace('/*__SKULL__*/', '').replace('/*__BONEDATA__*/null', (root / 'skull' / 'bones_bp3d.json').read_text()).replace('/*__FASCIA_LIB__*/', fascia_lib).replace('/*__FASCIA__*/null', fmodel).replace('/*__FASCIA_NOTES__*/null', notes)
app = app.replace('/*__MUSCLE_SURFACES_LIB__*/', muscle_lib).replace('/*__CONNECTIVE_SURFACES_LIB__*/', connective_lib).replace('/*__MUSCLE_SURFACES__*/null', muscle_data)
app = app.replace('/*__JOINT_SURFACES_LIB__*/', joint_lib).replace('/*__JOINT_SURFACES__*/null', joint_data)
app = app.replace('/*__CAPSULE_RIMS__*/null', capsule_rims)
app = app.replace('/*__RESEARCH_SENSITIVITY__*/null', research_sensitivity)
app = app.replace('/*__PRACTICE__*/null', json.dumps(practice, ensure_ascii=False, separators=(',', ':')))
tpl = (root / 'template3.html').read_text()
out = tpl.replace('/*__APP__*/', app.replace('/*__DEBUG__*/', ''))
dbg = tpl.replace('/*__APP__*/', app.replace('/*__DEBUG__*/', 'window.__dbg = { camera, controls, setT, setLayer, select, state, resetView, VIEW_DIR, centerTarget, THREE, get W() { return poseW; }, strands, jointById, A, focusOn, switchPose, STATS, setLightMode, PHYS, setOverride: c => { overrideConf = c; poseDirty = true; }, CYC, cyc, enterCycle, exitCycle, setCycT, cycPlay, cycPause, roundStats, FASC, FR, get fasciaM() { return fasciaM; }, updateFascia, get viewDir() { return VIEW_DIR; }, get muscleSurfaces() { return muscleSurfaces; }, get connectiveSurfaces() { return connectiveSurfaces; }, get jointSurfaces() { return jointSurfaces; }, cartMeshes, discMeshes, glowByJoint, boneMeshes, PRACTICE_DATA, enterPractice, exitPractice, practiceCurrent, get displayConf() { return displayConf; }, get morph() { return morph; }, get forcesGroup() { return forcesGroup; } };'))
outdir.mkdir(exist_ok=True)
(outdir / 'index.html').write_text(out, encoding='utf-8')
(outdir / 'test.html').write_text(dbg, encoding='utf-8')
print('built', outdir.name, len(out) // 1024, 'KB', '(round: %s)' % (cyc_file.name if cyc_file.exists() else 'none'))
