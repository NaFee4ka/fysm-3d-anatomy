import time, subprocess, sys
from playwright.sync_api import sync_playwright
srv = subprocess.Popen(['python3', '-m', 'http.server', '8765'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.2)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        ctx = b.new_context(viewport={'width': 1300, 'height': 900})
        pg = ctx.new_page()
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.goto('http://localhost:8765/test.html', wait_until='networkidle', timeout=90000)
        pg.wait_for_selector('#stage.ready', timeout=90000)
        for spec in sys.argv[1].split(';'):
            pid, sel, name = spec.split('|')
            pg.evaluate(f"window.__dbg.switchPose('{pid}')"); time.sleep(5)
            pg.evaluate(f"window.__dbg.select({sel})"); time.sleep(0.6)
            pg.locator('#info').screenshot(path=f'./c_{name}.png')
        print('errors:', errs)
        b.close()
finally:
    srv.terminate()
