import time, subprocess, sys, json
from playwright.sync_api import sync_playwright
out = sys.argv[1]
srv = subprocess.Popen(['python3', '-m', 'http.server', '8781'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1)
anch = json.load(open('./cycle_keys/anchors.json'))
fold = json.load(open('./cycle_keys/fold.json'))
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        pg = b.new_page(viewport={'width': 900, 'height': 760})
        errs = []; pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.goto('http://localhost:8781/test.html', wait_until='networkidle', timeout=120000)
        pg.wait_for_selector('#stage.ready', timeout=120000)
        pg.wait_for_function('window.__skullReady === true', timeout=120000)
        pg.evaluate("(() => { const d = window.__dbg; d.setT(0); d.state.userMoved = true; const h=document.getElementById('hint'); if(h) h.hidden=true; d.setLayer('musclesOuter', false); d.setLayer('musclesDeep', false); d.setLayer('ligs', false); })()")
        time.sleep(1.5)
        def cam(seg, dy, off, name):
            pg.evaluate(f"""(() => {{ const d = window.__dbg; const t = d.W['{seg}'].p.clone(); t.y += {dy};
              d.controls.target.copy(t); d.camera.position.set(t.x+{off[0]}, t.y+{off[1]}, t.z+{off[2]}); d.controls.update(); }})()""")
            time.sleep(1.2); pg.locator('#stage').screenshot(path=f'./cycle_shots/{out}_{name}.png')
        cam('thigh_L', -0.02, (0.35, 0.05, 0.35), 'hip')
        cam('shank_L', 0.0, (0.3, 0.05, 0.3), 'knee')
        cam('shank_L', 0.0, (0.4, 0.0, 0.0), 'knee_side')
        cam('foot_L', 0.0, (0.25, 0.08, 0.25), 'foot')
        cam('foot_L', 0.0, (0.3, 0.0, 0.0), 'foot_side')
        cam('foot_L', 0.0, (0.0, 0.0, 0.35), 'feet_front')
        cam('pelvis', -0.45, (1.6, 0.1, 0.8), 'legs')
        for name, conf in (('dog', anch['dog']), ('plank', anch['plank']), ('fold', fold)):
            pg.evaluate("""(f) => { const d = window.__dbg, T = d.THREE; const local = {}; for (const [s, q] of Object.entries(f.local)) local[s] = new T.Quaternion(q[0], q[1], q[2], q[3]);
                d.setOverride({ local, rootQ: new T.Quaternion(...f.root), rootP: new T.Vector3(...f.rootP) }); }""", conf)
            time.sleep(1.0)
            cam('foot_L', 0.0, (0.45, 0.06, 0.0), name + '_foot')
            cam('shank_L', 0.0, (0.9, 0.1, 0.4), name + '_leg')
        print('errors', errs)
        b.close()
finally:
    srv.terminate()
