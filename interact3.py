import time, subprocess
from playwright.sync_api import sync_playwright
srv = subprocess.Popen(['python3', '-m', 'http.server', '8765'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.2)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        ctx = b.new_context(viewport={'width': 1300, 'height': 820})
        pg = ctx.new_page()
        logs = []
        pg.on('console', lambda m: logs.append(f'{m.type}: {m.text}') if m.type in ('error', 'warning') else None)
        pg.on('pageerror', lambda e: logs.append(f'PAGEERROR: {e}'))
        t0 = time.time()
        pg.goto('http://localhost:8765/test.html#bow', wait_until='networkidle', timeout=90000)
        pg.wait_for_selector('#stage.ready', timeout=90000)
        print('ready', round(time.time() - t0, 1), 's; start pose via hash:', pg.locator('#pose-name').inner_text())
        title = lambda: pg.locator('#info .card-title').inner_text()
        for pid in ['dog', 'plank', 'half', 'fold']:
            pg.locator(f'[data-pose="{pid}"]').click(); time.sleep(0.6)
            pressed = pg.locator(f'[data-pose="{pid}"]').get_attribute('aria-pressed')
            print(pid, 'pressed', pressed, '| h1', pg.locator('#pose-name').inner_text(), '| end', pg.locator('#pose-end').inner_text(), '| cues', pg.locator('#cues li').count(), '| summary', title())
        # muscle card in the fold, then switch pose with the card open
        pg.evaluate("window.__dbg.select({kind:'muscle', unit:'gluteMax', side:'L'})"); time.sleep(0.4)
        print('card:', title(), '|', pg.locator('#info .card-sub').all_inner_texts())
        pg.locator('[data-pose="plank"]').click(); time.sleep(2.5)
        print('after switch card:', title(), '|', pg.locator('#info .card-sub').all_inner_texts()[:2], '| big', pg.locator('#info .big').first.inner_text())
        # joint card
        pg.evaluate("window.__dbg.select({kind:'joint', id:'gh_L'})"); time.sleep(0.4)
        print('joint:', title(), pg.locator('#info .comps li').count(), 'comps')
        # layers
        pg.locator('[data-layer="musclesOuter"]').click(); time.sleep(0.2)
        vis = pg.evaluate("() => { const d = window.__dbg; const m = d.strands.filter(s => s.inst.kind === 'muscle'); return [m.filter(s => s.mesh.visible).length, m.length]; }")
        print('outer off -> visible muscles', vis)
        pg.locator('[data-layer="musclesOuter"]').click()
        # slider to 0 then play
        pg.evaluate("() => { const s = document.getElementById('pose'); s.value = 0; s.dispatchEvent(new Event('input', {bubbles:true})); }"); time.sleep(0.6)
        print('slider 0 ->', pg.locator('#pose-out').inner_text())
        pg.locator('#play').click(); time.sleep(6)
        print('after play ->', pg.locator('#pose-out').inner_text())
        print('errors:', logs[:8])
        b.close()
finally:
    srv.terminate()
