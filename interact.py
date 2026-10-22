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
    pg.goto(url, wait_until='networkidle', timeout=60000)
    time.sleep(4)
    def proj(js_point):
        return pg.evaluate("""(expr) => { const d = window.__dbg; const p = (new Function('d', 'return ' + expr))(d).clone(); p.project(d.camera);
          const r = document.getElementById('view').getBoundingClientRect(); return [(p.x+1)/2*r.width + r.left, (1-p.y)/2*r.height + r.top]; }""", js_point)
    title = lambda: pg.locator('#info .card-title').inner_text()
    print('summary title:', title())
    # muscle: middle of a left biceps femoris strand
    x, y = proj("d.strands.find(s => s.inst.unit === 'bicFem' && s.inst.side === 'R').center[15]")
    pg.mouse.click(x, y); time.sleep(0.6)
    print('click bicFem ->', title(), '|', pg.locator('#info .big').inner_text() if pg.locator('#info .big').count() else '')
    # joint: right hip
    x, y = proj("d.jointById['hip_R'].mesh.getWorldPosition(new d.THREE.Vector3())")
    pg.mouse.click(x, y); time.sleep(0.6)
    print('click hip ->', title(), '|', pg.locator('#info .big').inner_text() if pg.locator('#info .big').count() else '')
    # slider to 0
    pg.evaluate("() => { const s = document.getElementById('pose'); s.value = 0; s.dispatchEvent(new Event('input', {bubbles:true})); }")
    time.sleep(1.2)
    print('after slider 0 ->', title(), '|', pg.locator('#info .big').inner_text(), '| out', pg.locator('#pose-out').inner_text())
    # close card
    pg.locator('#info .card-close').click(); time.sleep(0.4)
    print('after close ->', title())
    # summary row click
    pg.locator('#info [data-unit]').first.click(); time.sleep(0.4)
    print('row click ->', title())
    # layer toggles
    pg.locator('[data-layer="muscles"]').click(); time.sleep(0.3)
    print('muscles pressed:', pg.locator('[data-layer="muscles"]').get_attribute('aria-pressed'), 'card:', title())
    pg.locator('[data-layer="muscles"]').click(); time.sleep(0.3)
    # play
    pg.locator('#play').click(); time.sleep(1.5)
    print('during play out:', pg.locator('#pose-out').inner_text())
    time.sleep(3.5)
    print('after play out:', pg.locator('#pose-out').inner_text())
    print('logs:', logs[:10])
    b.close()
