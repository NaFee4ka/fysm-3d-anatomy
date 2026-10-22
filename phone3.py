import time, subprocess, sys
from playwright.sync_api import sync_playwright
srv = subprocess.Popen(['python3', '-m', 'http.server', '8765'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.2)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        ctx = b.new_context(viewport={'width': 390, 'height': 844}, device_scale_factor=2, is_mobile=True, has_touch=True)
        pg = ctx.new_page()
        pg.goto('http://localhost:8765/test.html', wait_until='networkidle', timeout=90000)
        pg.wait_for_selector('#stage.ready', timeout=90000)
        for pid in sys.argv[1].split(','):
            pg.evaluate(f"window.__dbg.switchPose('{pid}')"); time.sleep(2.6)
            pg.locator('#cues').screenshot(path=f'./ph_cues_{pid}.png')
        pg.evaluate("window.scrollTo(0,0)"); time.sleep(0.5)
        pg.screenshot(path='./ph_top.png')
        pg.locator('#info').screenshot(path='./ph_info.png')
        b.close()
finally:
    srv.terminate()
