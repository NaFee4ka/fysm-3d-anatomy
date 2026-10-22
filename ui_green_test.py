#!/usr/bin/env python3
"""Working muscles in green. usage: python3 ui_green_test.py OUTPREFIX [desk|phone]"""
import sys, os, time, json, socket, subprocess, fcntl
from playwright.sync_api import sync_playwright
ROOT = '.'
outp = sys.argv[1]; which = sys.argv[2] if len(sys.argv) > 2 else 'desk'
lock = open(os.path.join(ROOT, '.heavy.lock'), 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=os.path.join(ROOT, 'dist'), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
ev = lambda pg, js: pg.evaluate(js)
errs = []
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        vp = (1300, 820) if which == 'desk' else (390, 844)
        ctx = b.new_context(viewport={'width': vp[0], 'height': vp[1]}, device_scale_factor=1)
        pg = ctx.new_page(); pg.set_default_timeout(90000)
        pg.on('pageerror', lambda e: errs.append('pageerror: ' + str(e)))
        pg.on('console', lambda m: errs.append('console: ' + m.text) if m.type == 'error' else None)
        pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=180000)
        pg.wait_for_selector('#stage.ready', timeout=180000); time.sleep(1.0)
        for pose in ['dog', 'handstand']:
            pg.click(f'[data-pose="{pose}"]'); time.sleep(6.0)
            ev(pg, "window.__dbg.resetView(false)"); time.sleep(2.0)
            n = ev(pg, "window.__dbg.strands.filter(s => s.inst.kind === 'muscle' && s.on).length")
            tot = ev(pg, "window.__dbg.strands.filter(s => s.inst.kind === 'muscle').length")
            print(pose, 'green strands', n, 'of', tot)
            pg.locator('#stage').screenshot(path=f'{outp}_{which}_{pose}_length.png', timeout=120000, animations='disabled')
            if pose == 'handstand':
                pg.locator('.legend-wrap').screenshot(path=f'{outp}_{which}_legend_length.png', timeout=60000)
            pg.click('[data-lightmode="effort"]'); time.sleep(2.5)
            pg.locator('#stage').screenshot(path=f'{outp}_{which}_{pose}_effort.png', timeout=120000, animations='disabled')
            if pose == 'handstand':
                pg.locator('.legend-wrap').screenshot(path=f'{outp}_{which}_legend_effort.png', timeout=60000)
                pg.set_viewport_size({'width': vp[0], 'height': 3000}); time.sleep(0.6)
                ev(pg, "window.__dbg.select({kind:'muscle',unit:'fingerFlex',side:'L'})"); time.sleep(0.8)
                pg.locator('#info').screenshot(path=f'{outp}_{which}_card.png', timeout=60000)
                ev(pg, "window.__dbg.select(null)"); time.sleep(0.5)
                pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.5)
            pg.click('[data-lightmode="length"]'); time.sleep(1.0)
        ctx.close(); b.close()
finally:
    srv.terminate()
print('errs', errs)
