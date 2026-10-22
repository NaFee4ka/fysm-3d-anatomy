import time, subprocess, sys, socket, fcntl
from playwright.sync_api import sync_playwright
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
lock = open('./.heavy.lock', 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1)
out = sys.argv[1]
views = [('side', (1.6, 0.1, 0.0), 1.15), ('front', (0.0, 0.1, 1.6), 1.15), ('back', (0.0, 0.2, -1.6), 1.15), ('q34', (1.0, 0.4, 1.0), 1.15),
         ('neck', (0.55, 0.05, 0.2), 1.5), ('pelv', (0.5, 0.2, 0.55), 0.95)]
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        pg = b.new_page(viewport={'width': 900, 'height': 900})
        errs = []; pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=180000)
        pg.wait_for_selector('#stage.ready', timeout=180000)
        pg.wait_for_function('window.__skullReady === true', timeout=180000)
        pg.evaluate("(() => { const d = window.__dbg; d.setT(0); d.state.userMoved = true; const h=document.getElementById('hint'); if(h) h.hidden=true; d.setLayer('musclesOuter', false); d.setLayer('musclesDeep', false); d.setLayer('ligs', false); })()")
        time.sleep(1.5)
        for name, off, ty in views:
            pg.evaluate(f"""(() => {{ const d = window.__dbg; const t = new d.THREE.Vector3(0, {ty}, 0); d.controls.target.copy(t); d.camera.position.set({off[0]}, {ty}+{off[1]}, {off[2]}); d.controls.update(); }})()""")
            time.sleep(1.0)
            pg.locator('#stage').screenshot(path=f'./cycle_shots/{out}_{name}.png')
        print('errors', errs)
        b.close()
finally:
    srv.terminate()
