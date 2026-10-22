#!/usr/bin/env python3
"""Fascia / Myers lines layer test. usage: python3 ui_fascia_test.py [DISTDIR] [OUTPREFIX] [desk|phone|both]"""
import sys, os, time, json, socket, subprocess, fcntl
from playwright.sync_api import sync_playwright

ROOT = '.'
dist = os.path.join(ROOT, sys.argv[1] if len(sys.argv) > 1 else 'dist')
outp = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'cycle_shots', 'fas')
which = sys.argv[3] if len(sys.argv) > 3 else 'both'
lock = open(os.path.join(ROOT, '.heavy.lock'), 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=dist, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
def ev(pg, js): return pg.evaluate(js)
def shot(pg, sel, path):
    try: pg.locator(sel).screenshot(path=path, timeout=90000, animations='disabled')
    except Exception as e: print('shot failed', path, str(e)[:120])
res = {}
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        vps = [((1300, 820), 'desk'), ((390, 844), 'phone')]
        if which != 'both': vps = [v for v in vps if v[1] == which]
        for vp, tag in vps:
            ctx = b.new_context(viewport={'width': vp[0], 'height': vp[1]}, device_scale_factor=1)
            pg = ctx.new_page(); pg.set_default_timeout(90000)
            errs = []
            pg.on('pageerror', lambda e: errs.append('pageerror: ' + str(e)))
            pg.on('console', lambda m: errs.append('console: ' + m.text) if m.type == 'error' else None)
            pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=180000)
            pg.wait_for_selector('#stage.ready', timeout=180000)
            time.sleep(1.5)
            r = {}
            r['hasF'] = ev(pg, '!!window.__dbg.FASC')
            pg.click('[data-layer="flines"]'); time.sleep(1.5)
            r['overview'] = ev(pg, "document.querySelector('#info .card-title').textContent")
            r['rows'] = ev(pg, "document.querySelectorAll('#info [data-fline]').length")
            ev(pg, "window.__dbg.setLayer('musclesOuter', false)")
            time.sleep(1.0)
            shot(pg, '#stage', f'{outp}_{tag}_lines_dog.png')
            pg.locator('#info [data-fline="SBL"]').first.click(); time.sleep(1.2)
            r['sblTitle'] = ev(pg, "document.querySelector('#info .card-title').textContent")
            r['sblStrain'] = ev(pg, "(document.getElementById('fl-strain')||{}).textContent")
            shot(pg, '#stage', f'{outp}_{tag}_sbl_dog.png')
            pg.set_viewport_size({'width': vp[0], 'height': 2400}); time.sleep(0.6)
            shot(pg, '#info', f'{outp}_{tag}_sbl_card.png')
            pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
            # effort mode
            ev(pg, "window.__dbg.setLightMode('effort')"); time.sleep(0.8)
            r['effortEng'] = ev(pg, "(document.getElementById('fl-eng')||{}).textContent")
            ev(pg, "window.__dbg.setLightMode('length')")
            # fascia layer + element card
            pg.click('[data-layer="fascia"]'); time.sleep(1.2)
            ev(pg, "window.__dbg.select({ kind: 'fascia', id: 'tlfPost' })"); time.sleep(1.0)
            r['felTitle'] = ev(pg, "document.querySelector('#info .card-title').textContent")
            ev(pg, "window.__dbg.select(null)"); time.sleep(0.8)
            ev(pg, "window.__dbg.switchPose('fold')"); time.sleep(3.0)
            shot(pg, '#stage', f'{outp}_{tag}_fascia_fold.png')
            pg.set_viewport_size({'width': vp[0], 'height': 2400}); time.sleep(0.6)
            ev(pg, "window.__dbg.select({ kind: 'fascia', id: 'tlfPost' })"); time.sleep(1.0)
            shot(pg, '#info', f'{outp}_{tag}_fel_card.png')
            pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
            # muscle card lists its lines
            ev(pg, "window.__dbg.select({ kind: 'muscle', unit: 'gastroM', side: 'L' })"); time.sleep(1.0)
            r['muscleLines'] = ev(pg, "document.querySelectorAll('#info [data-lines-of] [data-fline]').length")
            # in the round
            ev(pg, "window.__dbg.select(null)")
            if ev(pg, '!!window.__dbg.CYC'):
                ev(pg, 'window.__dbg.enterCycle()'); time.sleep(2.0)
                ev(pg, 'window.__dbg.cycPause && window.__dbg.cycPause()')
                ev(pg, 'window.__dbg.setCycT(6.5)'); time.sleep(1.5)
                ev(pg, "window.__dbg.select({ kind: 'fline', id: 'SFL' })"); time.sleep(1.2)
                r['cycSFL'] = ev(pg, "(document.getElementById('fl-strain')||{}).textContent")
                r['cycCurve'] = ev(pg, "!!document.querySelector('#info [data-curve]')")
                r['roundLines'] = ev(pg, "document.querySelectorAll('#cues [data-fline]').length")
                pg.set_viewport_size({'width': vp[0], 'height': 2600}); time.sleep(0.6)
                shot(pg, '#info', f'{outp}_{tag}_cycle_line_card.png')
                shot(pg, '#cues', f'{outp}_{tag}_cycle_stats.png')
                pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
                shot(pg, '#stage', f'{outp}_{tag}_cycle_sfl.png')
                ev(pg, 'window.__dbg.cycPlay && window.__dbg.cycPlay()'); time.sleep(3.0)
                r['cycSFL2'] = ev(pg, "(document.getElementById('fl-strain')||{}).textContent")
                ev(pg, 'window.__dbg.cycPause && window.__dbg.cycPause()')
            r['overflowX'] = ev(pg, 'document.documentElement.scrollWidth > window.innerWidth + 1')
            r['errors'] = errs[:8]
            res[tag] = r
            ctx.close()
        b.close()
finally:
    srv.terminate()
print(json.dumps(res, ensure_ascii=False, indent=1))
