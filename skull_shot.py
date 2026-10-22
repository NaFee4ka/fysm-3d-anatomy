import time, subprocess, sys
from playwright.sync_api import sync_playwright
srv = subprocess.Popen(['python3', '-m', 'http.server', '8771'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1)
out = sys.argv[1] if len(sys.argv) > 1 else 'sk'
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        pg = b.new_page(viewport={'width': 900, 'height': 760})
        errs = []; pg.on('pageerror', lambda e: errs.append(str(e)))
        t0 = time.time()
        pg.goto('http://localhost:8771/test.html', wait_until='networkidle', timeout=120000)
        pg.wait_for_selector('#stage.ready', timeout=120000)
        pg.wait_for_function('window.__skullReady === true', timeout=120000)
        print('load s', round(time.time() - t0, 1))
        pg.evaluate("(() => { const d = window.__dbg; d.setT(0); d.state.userMoved = true; const h=document.getElementById('hint'); if(h) h.hidden=true; })()")
        time.sleep(1.5)
        for name, off, bones in [('side', (0.62, 0.05, 0.0), True), ('q34', (0.42, 0.12, 0.45), True), ('front', (0.0, 0.04, 0.62), True), ('back', (-0.3, 0.2, -0.5), True), ('face', (0.1, 0.0, 0.3), True), ('teeth', (0.08, -0.03, 0.16), True), ('lat', (0.3, 0.0, 0.05), True), ('musc', (0.42, 0.12, 0.45), False)]:
            on = 'false' if bones else 'true'
            pg.evaluate(f"""(() => {{ const d = window.__dbg; d.setLayer('musclesOuter', {on}); d.setLayer('musclesDeep', {on}); d.setLayer('ligs', {on});
              const t = d.W.head.p.clone(); t.y += ('{name}'==='teeth' ? -0.015 : '{name}'==='face'||'{name}'==='lat' ? 0.03 : 0.05); t.z += ('{name}'==='teeth' ? 0.12 : '{name}'==='face' ? 0.09 : 0.03); d.controls.target.copy(t); d.camera.position.set(t.x+{off[0]}, t.y+{off[1]}, t.z+{off[2]}); d.controls.update(); }})()""")
            time.sleep(1.2)
            pg.locator('#stage').screenshot(path=f'./cycle_shots/{out}_{name}.png')
        print('errors', errs)
        b.close()
finally:
    srv.terminate()
