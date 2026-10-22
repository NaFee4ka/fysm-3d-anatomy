#!/usr/bin/env python3
"""Render arbitrary body configurations (cycle frames) with the real page, from fixed cameras.

usage: python3 shot_conf.py FRAMES.json OUT_PREFIX [--views side,front3q] [--every 1] [--only 0,5,9]
                            [--sheet] [--cols 4] [--bones] [--dist 4.3] [--target x,y,z] [--size 1100x700]

FRAMES.json: a list of configurations, or {"frames": [...]} / {"keys": [...]}; each item is
  {"local": {seg: [x,y,z,w]}, "root": [x,y,z,w], "rootP": [x,y,z], "label"?: str, "t"?: seconds}
Views (camera offset from a target fixed for the whole run, so motion relative to the floor is visible):
  side (from the body's left, +x), side_r (from the right, -x), front3q, back3q, top, front, low
Writes OUT_PREFIX_<view>_<frame>.png and, with --sheet, OUT_PREFIX_<view>_sheet.png (labelled grid).
Runs are serialised with a lock file (one browser at a time on this machine).
"""
import sys, os, json, time, socket, subprocess, argparse, fcntl
from playwright.sync_api import sync_playwright

ROOT = '.'
VIEWS = {
    'side': (1.0, 0.18, 0.0), 'side_r': (-1.0, 0.18, 0.0), 'front3q': (0.7, 0.42, 0.7), 'back3q': (0.7, 0.42, -0.7),
    'top': (0.02, 1.0, 0.0), 'front': (0.0, 0.25, 1.0), 'low': (1.0, 0.03, 0.25),
}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames'); ap.add_argument('out')
    ap.add_argument('--views', default='side'); ap.add_argument('--every', type=int, default=1)
    ap.add_argument('--only', default=''); ap.add_argument('--sheet', action='store_true')
    ap.add_argument('--cols', type=int, default=4); ap.add_argument('--bones', action='store_true')
    ap.add_argument('--dist', type=float, default=4.3); ap.add_argument('--target', default='')
    ap.add_argument('--size', default='1100x700')
    a = ap.parse_args()
    data = json.load(open(a.frames))
    if isinstance(data, dict): data = data.get('frames') or data.get('keys') or list(data.values())
    idx = [int(x) for x in a.only.split(',')] if a.only else list(range(0, len(data), a.every))
    if a.every > 1 and not a.only and (len(data) - 1) not in idx: idx.append(len(data) - 1)
    frames = [data[i] for i in idx]
    if a.target: tgt = [float(v) for v in a.target.split(',')]
    else:
        ps = [f['rootP'] for f in frames]
        tgt = [sum(p[0] for p in ps) / len(ps), 0.32, sum(p[2] for p in ps) / len(ps)]
    W, H = [int(v) for v in a.size.split('x')]
    lock = open(os.path.join(ROOT, '.heavy.lock'), 'w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
    srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=os.path.join(ROOT, 'dist'), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    shots = {}
    try:
        with sync_playwright() as p:
            b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
            pg = b.new_context(viewport={'width': W, 'height': H}).new_page()
            errs = []
            pg.on('pageerror', lambda e: errs.append(str(e)))
            pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=120000)
            pg.wait_for_selector('#stage.ready', timeout=120000)
            pg.wait_for_function('window.__skullReady === true', timeout=180000)
            on = 'false' if a.bones else 'true'
            pg.evaluate(f"(() => {{ const d = window.__dbg; d.state.userMoved = true; d.setLayer('musclesOuter', {on}); d.setLayer('musclesDeep', {on}); d.setLayer('ligs', {on}); const h = document.getElementById('hint'); if (h) h.hidden = true; const r = document.getElementById('reset'); if (r) r.style.display = 'none'; }})()")
            for view in a.views.split(','):
                dx, dy, dz = VIEWS[view]
                n = (dx * dx + dy * dy + dz * dz) ** 0.5
                cam = [tgt[0] + dx / n * a.dist, tgt[1] + dy / n * a.dist, tgt[2] + dz / n * a.dist]
                for k, f in zip(idx, frames):
                    pg.evaluate("""([f, tgt, cam]) => { const d = window.__dbg, T = d.THREE;
                        const local = {}; for (const [s, q] of Object.entries(f.local)) local[s] = new T.Quaternion(q[0], q[1], q[2], q[3]);
                        d.setOverride({ local, rootQ: new T.Quaternion(...f.root), rootP: new T.Vector3(...f.rootP) });
                        d.state.userMoved = true; d.state.camAnim = null;
                        d.controls.target.set(...tgt); d.camera.position.set(...cam); d.controls.update(); }""", [f, tgt, cam])
                    pg.evaluate("() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(() => requestAnimationFrame(r))))")
                    time.sleep(0.25)
                    path = f'{a.out}_{view}_{k:03d}.png'
                    pg.locator('#stage').screenshot(path=path)
                    shots.setdefault(view, []).append((k, f, path))
            if errs: print('page errors:', errs)
            b.close()
    finally:
        srv.terminate()
        fcntl.flock(lock, fcntl.LOCK_UN)
    if a.sheet:
        from PIL import Image, ImageDraw, ImageFont
        try: font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
        except Exception: font = ImageFont.load_default()
        for view, lst in shots.items():
            ims = [Image.open(p) for _, _, p in lst]
            w, h = ims[0].size
            tw, th = w // 2, h // 2
            cols = min(a.cols, len(ims)); rows = (len(ims) + cols - 1) // cols
            sheet = Image.new('RGB', (cols * tw, rows * th), (13, 18, 21))
            for i, ((k, f, _), im) in enumerate(zip(lst, ims)):
                t = im.convert('RGB').resize((tw, th))
                d = ImageDraw.Draw(t)
                lab = f'#{k}' + (f"  t={f['t']:.1f}s" if isinstance(f.get('t'), (int, float)) else '') + (f"  {f['label']}" if f.get('label') else '')
                d.text((8, 6), lab, fill=(255, 230, 160), font=font)
                sheet.paste(t, ((i % cols) * tw, (i // cols) * th))
            out = f'{a.out}_{view}_sheet.png'
            sheet.save(out)
            print('sheet', out)
    print('frames', len(frames), 'views', a.views)

if __name__ == '__main__':
    main()
