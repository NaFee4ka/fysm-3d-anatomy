# hand close-ups in the five poses (bones only). usage: python3 skull/arms_pose_shot.py FRAMES.json PREFIX
import time, subprocess, sys, json
from playwright.sync_api import sync_playwright
frames = json.load(open(sys.argv[1])); out = sys.argv[2]
srv = subprocess.Popen(['python3', '-m', 'http.server', '8782'], cwd='./dist', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1)
CAMS = [('a', (0.25, 0.18, 0.25)), ('b', (-0.05, 0.12, 0.35)), ('c', (0.3, 0.02, -0.1))]
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        pg = b.new_page(viewport={'width': 700, 'height': 600})
        pg.goto('http://localhost:8782/test.html', wait_until='networkidle', timeout=120000)
        pg.wait_for_selector('#stage.ready', timeout=120000)
        pg.wait_for_function('window.__skullReady === true', timeout=120000)
        pg.evaluate("(() => { const d = window.__dbg; d.state.userMoved = true; const h=document.getElementById('hint'); if(h) h.hidden=true; d.setLayer('musclesOuter', false); d.setLayer('musclesDeep', false); d.setLayer('ligs', false); })()")
        for f in frames:
            pg.evaluate("""(f) => { const d = window.__dbg, T = d.THREE; const local = {}; for (const [s, q] of Object.entries(f.local)) local[s] = new T.Quaternion(...q);
                d.setOverride({ local, rootQ: new T.Quaternion(...f.root), rootP: new T.Vector3(...f.rootP) }); }""", f)
            time.sleep(1.2)
            for name, off in CAMS:
                pg.evaluate(f"""(() => {{ const d = window.__dbg; d.state.camAnim = null; const t = d.W.hand_L.p.clone(); d.controls.target.copy(t);
                  d.camera.position.set(t.x+{off[0]}, t.y+{off[1]}, t.z+{off[2]}); d.controls.update(); }})()""")
                time.sleep(0.8)
                pg.locator('#stage').screenshot(path=f"./cycle_shots/{out}_{f['label']}_{name}.png")
        b.close()
finally:
    srv.terminate()
from PIL import Image
for f in frames:
    ims = [Image.open(f"./cycle_shots/{out}_{f['label']}_{n}.png").convert('RGB') for n, _ in CAMS]
    w, h = ims[0].size; s = Image.new('RGB', (w * 3, h))
    for i, im in enumerate(ims): s.paste(im, (i * w, 0))
    s.resize((w * 3 // 2, h // 2)).save(f"./cycle_shots/{out}_{f['label']}.png")
print('done')
