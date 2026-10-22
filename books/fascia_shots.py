#!/usr/bin/env python3
"""Screenshots of books/fascia_view.html: python3 books/fascia_shots.py "pose:view:mode:lines:layers;..." [out_prefix]
lines = comma list of line ids or 'all' or 'none'; layers = letters s (sheets) b (bands) l (lines) m (muscle lines)."""
import sys, os, time, socket, subprocess, fcntl
from playwright.sync_api import sync_playwright
ROOT = '/home/claude/dog3d'
specs = sys.argv[1].split(';'); out = sys.argv[2] if len(sys.argv) > 2 else ROOT + '/books/shots/fv'
lock = open(ROOT + '/.heavy.lock', 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        pg = b.new_context(viewport={'width': 900, 'height': 700}).new_page()
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None)
        pg.goto(f'http://localhost:{port}/books/fascia_view.html', wait_until='networkidle', timeout=120000)
        pg.wait_for_function('window.__view && window.__view.ready', timeout=120000)
        for k, sp in enumerate(specs):
            pose, view, mode, lines, layers = (sp.split(':') + ['all', 'sblm'])[:5]
            ids = 'null' if lines == 'all' else ('[]' if lines == 'none' else '[' + ','.join(f"'{x}'" for x in lines.split(',')) + ']')
            pg.evaluate(f"""() => {{ const v = window.__view; v.showLines({ids}); v.setLayer('sheets', {str('s' in layers).lower()}); v.setLayer('bands', {str('b' in layers).lower()}); v.setLayer('lines', {str('l' in layers).lower()}); v.muscles({str('m' in layers).lower()}); }}""")
            st = pg.evaluate(f"() => window.__view.set({{ pose: '{pose}', view: '{view}', mode: '{mode}', label: '{lines}' }})")
            time.sleep(0.3)
            path = f'{out}_{k:02d}_{pose}_{view}.png'
            pg.screenshot(path=path)
            print(path)
        if errs: print('errors:', errs[:5])
        b.close()
finally:
    srv.terminate(); fcntl.flock(lock, fcntl.LOCK_UN)
