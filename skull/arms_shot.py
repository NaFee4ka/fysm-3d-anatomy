# close-ups of the arm bones in plain standing (bones layer only). usage: python3 skull/arms_shot.py PREFIX
import time, subprocess, sys
from playwright.sync_api import sync_playwright
srv = subprocess.Popen(['python3', '-m', 'http.server', '8781'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1)
out = sys.argv[1] if len(sys.argv) > 1 else 'arm'
VIEWS_ALL = [  # name, segment target, camera offset
    ('shoulder_front', 'hum_L', (0.15, 0.05, 0.45)), ('shoulder_back', 'hum_L', (0.2, 0.1, -0.45)), ('shoulder_top', 'scap_L', (0.05, 0.5, 0.05)),
    ('elbow_side', 'fore_L', (0.4, 0.0, 0.05)), ('elbow_front', 'fore_L', (0.1, 0.0, 0.4)),
    ('hand_lat', 'hand_L', (0.3, 0.0, 0.02)), ('hand_med', 'hand_L', (-0.3, 0.0, 0.02)), ('hand_front', 'hand_L', (0.05, 0.0, 0.3)),
    ('right_hand', 'hand_R', (-0.3, 0.0, 0.05)), ('arms_full', 'thC', (0.0, -0.1, 1.3)),
    ('scap_back', 'thC', (0.0, 0.05, -0.75)), ('scap_top', 'thC', (0.0, 0.7, -0.15)), ('shoulder_R_front', 'hum_R', (-0.15, 0.05, 0.45)),
]
VIEWS = [v for v in VIEWS_ALL if len(sys.argv) < 3 or v[0] in sys.argv[2].split(',')]
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
        for name, seg, off in VIEWS:
            pg.evaluate(f"""(() => {{ const d = window.__dbg; const t = d.W['{seg}'].p.clone(); if ('{seg}'.startsWith('hand')) t.y -= 0.08; d.controls.target.copy(t);
              d.camera.position.set(t.x+{off[0]}, t.y+{off[1]}, t.z+{off[2]}); d.controls.update(); }})()""")
            time.sleep(1.0)
            pg.locator('#stage').screenshot(path=f'./cycle_shots/{out}_{name}.png')
        print('errors', errs)
        b.close()
finally:
    srv.terminate()
