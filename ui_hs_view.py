#!/usr/bin/env python3
"""Handstand framing test. usage: python3 ui_hs_view.py OUTPREFIX [desk|phone|both] [view]"""
import sys, os, time, json, socket, subprocess, fcntl
from playwright.sync_api import sync_playwright
ROOT = '.'
outp = sys.argv[1]; which = sys.argv[2] if len(sys.argv) > 2 else 'both'
lock = open(os.path.join(ROOT, '.heavy.lock'), 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=os.path.join(ROOT, 'dist'), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
ev = lambda pg, js: pg.evaluate(js)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        vps = [((1300, 820), 'desk'), ((390, 844), 'phone')]
        if which != 'both': vps = [v for v in vps if v[1] == which]
        for vp, tag in vps:
            ctx = b.new_context(viewport={'width': vp[0], 'height': vp[1]}, device_scale_factor=1)
            pg = ctx.new_page(); pg.set_default_timeout(90000)
            pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=180000)
            pg.wait_for_selector('#stage.ready', timeout=180000); time.sleep(1.0)
            pg.click('[data-pose="handstand"]'); time.sleep(7.0)
            ev(pg, "window.__dbg.resetView(false)"); time.sleep(2.5)
            print(tag, 'slider end label:', ev(pg, "document.getElementById('pose-end').textContent"), '| name:', ev(pg, "document.getElementById('pose-name').textContent"))
            pg.locator('#stage').screenshot(path=f'{outp}_{tag}_muscles.png', timeout=120000, animations='disabled')
            ev(pg, "window.__dbg.setLayer('musclesOuter', false); window.__dbg.setLayer('musclesDeep', false)"); time.sleep(2.0)
            pg.locator('#stage').screenshot(path=f'{outp}_{tag}_bones.png', timeout=120000, animations='disabled')
            ctx.close()
        b.close()
finally:
    srv.terminate()
