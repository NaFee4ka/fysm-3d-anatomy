import time
from playwright.sync_api import sync_playwright
url = 'http://localhost:8765/test.html'
with sync_playwright() as p:
    b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
    ctx = b.new_context(viewport={'width': 1200, 'height': 800})
    pg = ctx.new_page()
    logs = []
    pg.on('console', lambda m: logs.append(f'{m.type}: {m.text}'))
    pg.on('pageerror', lambda e: logs.append(f'PAGEERROR: {e}'))
    t0 = time.time()
    pg.goto(url, wait_until='networkidle', timeout=60000)
    pg.wait_for_selector('#stage.ready', timeout=60000)
    print('ready in', round(time.time() - t0, 1), 's')
    title = lambda: pg.locator('#info .card-title').inner_text()
    print('summary:', title())
    pg.locator('#info').screenshot(path='./s_summary.png')
    pg.evaluate("window.__dbg.select({kind:'joint', id:'gh_L'})"); time.sleep(0.4)
    print('joint card:', title()); pg.locator('#info').screenshot(path='./s_card_joint.png')
    pg.evaluate("window.__dbg.select({kind:'muscle', unit:'lat', side:'L'})"); time.sleep(0.4)
    print('muscle card:', title()); pg.locator('#info').screenshot(path='./s_card_muscle.png')
    pg.evaluate("window.__dbg.select({kind:'lig', unit:'ighl', side:'L'})"); time.sleep(0.4)
    print('lig card:', title()); pg.locator('#info').screenshot(path='./s_card_lig.png')
    pg.locator('#info .card-focus').click(); time.sleep(1.2)
    pg.locator('#stage').screenshot(path='./s_focus_lig.png')
    pg.evaluate("window.__dbg.select({kind:'joint', id:'hip_L'})"); time.sleep(0.3)
    pg.locator('#info .card-focus').click(); time.sleep(1.2)
    pg.locator('#stage').screenshot(path='./s_focus_hip.png')
    # slider timing
    t1 = time.time()
    for v in [80, 60, 40, 20, 0]:
        pg.evaluate("(v) => { const s = document.getElementById('pose'); s.value = v; s.dispatchEvent(new Event('input', {bubbles:true})); }", v); time.sleep(0.25)
    print('slider steps took', round(time.time() - t1, 2), 's; card now:', title(), pg.locator('#pose-out').inner_text())
    pg.locator('#stage').screenshot(path='./s_stand.png')
    pg.evaluate("window.__dbg.setT(1)"); time.sleep(0.6)
    fps = pg.evaluate("() => new Promise(res => { let n = 0; const t0 = performance.now(); function f() { n++; if (performance.now() - t0 < 1500) requestAnimationFrame(f); else res(n / 1.5); } requestAnimationFrame(f); })")
    print('fps (software GL):', round(fps, 1))
    print('logs:', logs[:10])
    b.close()
