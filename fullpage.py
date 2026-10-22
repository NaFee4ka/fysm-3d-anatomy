import time, subprocess, sys
from playwright.sync_api import sync_playwright
srv = subprocess.Popen(['python3', '-m', 'http.server', '8765'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.2)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        errs = []
        for name, vp, mob, pose in [('desk', {'width': 1300, 'height': 860}, False, 'half'), ('phone', {'width': 390, 'height': 844}, True, 'bow')]:
            ctx = b.new_context(viewport=vp, device_scale_factor=1 if not mob else 2, is_mobile=mob, has_touch=mob)
            pg = ctx.new_page()
            pg.on('pageerror', lambda e: errs.append(str(e)))
            pg.goto(f'http://localhost:8765/test.html#{pose}', wait_until='networkidle', timeout=90000)
            pg.wait_for_selector('#stage.ready', timeout=90000)
            time.sleep(4)
            pg.screenshot(path=f'./f_{name}.png', full_page=True)
            ctx.close()
        print('errors:', errs)
        b.close()
finally:
    srv.terminate()
