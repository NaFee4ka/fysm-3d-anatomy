import re, sys, pathlib
# usage: python3 build.py [OUTDIR]   (default dist; the round comes from cycle_data.json when it exists)
root = pathlib.Path(__file__).resolve().parent
outdir = root / (sys.argv[1] if len(sys.argv) > 1 else 'dist')
anat = (root / 'anatomy3.js').read_text().replace('export function createAnatomy', 'function createAnatomy')
import os
cyc_file = pathlib.Path(os.environ.get('CYCLE_DATA', str(root / 'cycle_data.json')))
cycle = cyc_file.read_text() if cyc_file.exists() else 'null'
import json
fl = (root / 'fascia3.js').read_text().replace('export function', 'function').replace('export const', 'const')
fr = (root / 'fascia_render.js').read_text().replace('export function', 'function')
fascia_lib = 'const FASCIA_LIB = (() => {\n' + fl + '\nreturn { createFascia, LINE_COLORS };\n})();\n' + fr
fmodel = (root / 'books' / 'fascia_model.json').read_text()
ml = json.loads((root / 'books' / 'myers_lines.json').read_text())
notes = json.dumps({l['id']: {k: v.get('ru', '') for k, v in (l.get('poses') or {}).items()} for l in ml['lines']}, ensure_ascii=False)
app = (root / 'app3.js').read_text().replace('/*__POSES__*/null', (root / 'poses.json').read_text()).replace('/*__PHYSICS__*/null', (root / 'physics.json').read_text()).replace('/*__CYCLE__*/null', cycle).replace('/*__ANATOMY__*/', anat).replace('/*__SKULL__*/', '').replace('/*__BONEDATA__*/null', (root / 'skull' / 'bones_bp3d.json').read_text()).replace('/*__FASCIA_LIB__*/', fascia_lib).replace('/*__FASCIA__*/null', fmodel).replace('/*__FASCIA_NOTES__*/null', notes)
tpl = (root / 'template3.html').read_text()
out = tpl.replace('/*__APP__*/', app.replace('/*__DEBUG__*/', ''))
dbg = tpl.replace('/*__APP__*/', app.replace('/*__DEBUG__*/', 'window.__dbg = { camera, controls, setT, setLayer, select, state, resetView, VIEW_DIR, centerTarget, THREE, get W() { return poseW; }, strands, jointById, A, focusOn, switchPose, STATS, setLightMode, PHYS, setOverride: c => { overrideConf = c; poseDirty = true; }, CYC, cyc, enterCycle, exitCycle, setCycT, cycPlay, cycPause, roundStats, FASC, FR, get fasciaM() { return fasciaM; }, updateFascia, get viewDir() { return VIEW_DIR; } };'))
outdir.mkdir(exist_ok=True)
(outdir / 'index.html').write_text(out)
test = '<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><style>:root{padding-top:env(safe-area-inset-top);padding-bottom:env(safe-area-inset-bottom)}body{margin:0;font:14px system-ui}[hidden]{display:none!important}</style></head><body>' + dbg + '</body></html>'
(outdir / 'test.html').write_text(test)
print('built', outdir.name, len(out) // 1024, 'KB', '(round: %s)' % (cyc_file.name if cyc_file.exists() else 'none'))
