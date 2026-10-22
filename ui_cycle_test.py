#!/usr/bin/env python3
"""Interaction test of the round («Круг») mode. usage: python3 ui_cycle_test.py [DISTDIR] [OUTPREFIX]
Runs under the shared heavy-job lock (one browser at a time)."""
import sys, os, time, json, socket, subprocess, fcntl
from playwright.sync_api import sync_playwright

ROOT = '.'
dist = os.path.join(ROOT, sys.argv[1] if len(sys.argv) > 1 else 'dist_ui')
outp = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'cycle_shots', 'ui')
os.makedirs(os.path.dirname(outp), exist_ok=True)
lock = open(os.path.join(ROOT, '.heavy.lock'), 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=dist, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
res = {}
def ev(pg, js): return pg.evaluate(js)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        for vp, tag in [((1300, 820), 'desk'), ((390, 844), 'phone')]:
            ctx = b.new_context(viewport={'width': vp[0], 'height': vp[1]}, device_scale_factor=1 if tag == 'desk' else 2)
            pg = ctx.new_page(); pg.set_default_timeout(120000)
            errs = []
            pg.on('pageerror', lambda e: errs.append('pageerror: ' + str(e)))
            pg.on('console', lambda m: errs.append('console: ' + m.text) if m.type == 'error' else None)
            pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=120000)
            pg.wait_for_selector('#stage.ready', timeout=120000)
            time.sleep(1.5)
            r = {}
            r['tabVisible'] = ev(pg, "!document.getElementById('cycle-tab').hidden")
            pg.click('#cycle-tab')
            time.sleep(3.5)
            r['mode'] = ev(pg, 'window.__dbg.state.mode')
            r['playing'] = ev(pg, 'window.__dbg.cyc.playing')
            r['t1'] = ev(pg, 'window.__dbg.state.cycT')
            pg.locator('#stage').screenshot(path=f'{outp}_{tag}_stage1.png')
            time.sleep(3.0)
            r['t2'] = ev(pg, 'window.__dbg.state.cycT')
            r['header'] = ev(pg, "document.getElementById('pose-sanskrit').textContent + ' | ' + document.getElementById('pose-name').textContent + ' | ' + document.getElementById('cyc-phase').textContent")
            pg.screenshot(path=f'{outp}_{tag}_full1.png')
            # pause and scrub to the middle of the third transition
            pg.click('#cyc-play')
            r['pausedAfterClick'] = not ev(pg, 'window.__dbg.cyc.playing')
            ev(pg, "(() => { const s = document.getElementById('cyc-t'); s.value = 480; s.dispatchEvent(new Event('input', { bubbles: true })); })()")
            time.sleep(1.2)
            r['tScrub'] = ev(pg, 'window.__dbg.state.cycT')
            r['headerScrub'] = ev(pg, "document.getElementById('pose-name').textContent + ' | ' + document.getElementById('cyc-phase').textContent")
            # hold selector
            pg.click('[data-hold-s="30"]')
            time.sleep(0.4)
            r['kpiTime30'] = ev(pg, "document.querySelectorAll('.kpi .kpi-v')[3].textContent")
            pg.click('[data-hold-s="0"]')
            time.sleep(0.4)
            r['kpiTime0'] = ev(pg, "document.querySelectorAll('.kpi .kpi-v')[3].textContent")
            # tap the first muscle row (or the first stretch row when activations are not there yet)
            row = pg.locator('#cues [data-unit]').first
            r['firstRow'] = row.inner_text().split('\n')[0]
            row.click()
            time.sleep(1.0)
            r['sel'] = ev(pg, 'JSON.stringify(window.__dbg.state.sel)')
            r['curve'] = ev(pg, "!!document.querySelector('#info [data-curve]')")
            r['cardTitle'] = ev(pg, "document.querySelector('#info .card-title').textContent")
            pg.set_viewport_size({'width': vp[0], 'height': 2600}); time.sleep(0.6)
            pg.locator('#info').screenshot(path=f'{outp}_{tag}_muscle.png')
            pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
            # play while the card is open: the marker moves
            m0 = ev(pg, "(document.getElementById('cyc-live-mark')||{getAttribute:()=>null}).getAttribute('x1')")
            pg.click('#cyc-play'); time.sleep(2.5)
            m1 = ev(pg, "(document.getElementById('cyc-live-mark')||{getAttribute:()=>null}).getAttribute('x1')")
            r['markerMoved'] = (m0, m1)
            pg.click('#cyc-play')
            # a ligament row
            lr = pg.locator('#cues [data-lig]').first
            if lr.count():
                lr.click(); time.sleep(0.8)
                r['ligSel'] = ev(pg, 'JSON.stringify(window.__dbg.state.sel)')
                r['ligCurve'] = ev(pg, "!!document.querySelector('#info [data-curve]')")
                pg.set_viewport_size({'width': vp[0], 'height': 2600}); time.sleep(0.6)
                pg.locator('#info').screenshot(path=f'{outp}_{tag}_lig.png')
                pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
            # a joint (knee, left) from the model selection API
            ev(pg, "window.__dbg.select({ kind: 'joint', id: 'kn_L' })"); time.sleep(0.8)
            r['jointCard'] = ev(pg, "document.querySelector('#info .card-title').textContent + ' | load ' + !!document.getElementById('cyc-live-load') + ' | curves ' + document.querySelectorAll('#info [data-curve]').length")
            pg.set_viewport_size({'width': vp[0], 'height': 2600}); time.sleep(0.6)
            pg.locator('#info').screenshot(path=f'{outp}_{tag}_joint.png')
            pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
            # effort light and forces
            pg.click('[data-lightmode="effort"]'); time.sleep(0.6)
            pg.click('[data-layer="forces"]'); time.sleep(0.8)
            pg.locator('#stage').screenshot(path=f'{outp}_{tag}_effort.png')
            pg.click('[data-layer="forces"]'); pg.click('[data-lightmode="length"]')
            # close the card, cues card screenshot
            ev(pg, 'window.__dbg.select(null)'); time.sleep(0.5)
            pg.set_viewport_size({'width': vp[0], 'height': 3400}); time.sleep(0.8)
            pg.locator('#cues').screenshot(path=f'{outp}_{tag}_stats.png')
            pg.locator('#info').screenshot(path=f'{outp}_{tag}_now.png')
            pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.4)
            r['overflowX'] = ev(pg, 'document.documentElement.scrollWidth - window.innerWidth')
            # back to a pose
            pg.click('[data-pose="plank"]'); time.sleep(2.0)
            r['afterPose'] = ev(pg, "window.__dbg.state.mode + ' ' + window.__dbg.state.poseId + ' cycHidden=' + document.getElementById('cycle-player').hidden + ' poseHidden=' + document.getElementById('pose-player').hidden")
            pg.locator('#stage').screenshot(path=f'{outp}_{tag}_backpose.png')
            # and into the round again from that pose: starts at the plank
            pg.click('#cycle-tab'); time.sleep(0.8)
            r['reenterT'] = ev(pg, 'window.__dbg.state.cycT')
            pg.click('#cyc-play') if ev(pg, 'window.__dbg.cyc.playing') else None
            # end of the round and the loop: play at 2x from near the end
            ev(pg, "window.__dbg.setCycT(window.__dbg.CYC.T - 0.4)")
            pg.click('[data-speed="2"]'); pg.click('#cyc-play'); time.sleep(4.5)
            r['afterLoopT'] = ev(pg, 'window.__dbg.state.cycT')
            r['opacity'] = ev(pg, "document.getElementById('view').style.opacity")
            r['errors'] = errs
            res[tag] = r
            ctx.close()
        b.close()
finally:
    srv.terminate()
    fcntl.flock(lock, fcntl.LOCK_UN)
print(json.dumps(res, ensure_ascii=False, indent=1))
