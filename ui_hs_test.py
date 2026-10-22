#!/usr/bin/env python3
"""Handstand pose UI test. usage: python3 ui_hs_test.py OUTPREFIX [desk|phone|both] [DISTDIR]"""
import sys, os, time, json, socket, subprocess, fcntl
from playwright.sync_api import sync_playwright

ROOT = '.'
outp = sys.argv[1]
which = sys.argv[2] if len(sys.argv) > 2 else 'both'
dist = os.path.join(ROOT, sys.argv[3] if len(sys.argv) > 3 else 'dist')
lock = open(os.path.join(ROOT, '.heavy.lock'), 'w'); fcntl.flock(lock, fcntl.LOCK_EX)
s = socket.socket(); s.bind(('', 0)); port = s.getsockname()[1]; s.close()
srv = subprocess.Popen(['python3', '-m', 'http.server', str(port)], cwd=dist, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(1.0)
ev = lambda pg, js: pg.evaluate(js)
def shot(pg, sel, path):
    try: pg.locator(sel).screenshot(path=path, timeout=90000, animations='disabled')
    except Exception as e: print('shot failed', path, str(e)[:120])
res = {}
try:
    with sync_playwright() as p:
        b = p.chromium.launch(args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        vps = [((1300, 820), 'desk'), ((390, 844), 'phone')]
        if which != 'both': vps = [v for v in vps if v[1] == which]
        for vp, tag in vps:
            ctx = b.new_context(viewport={'width': vp[0], 'height': vp[1]}, device_scale_factor=1)
            pg = ctx.new_page(); pg.set_default_timeout(90000)
            errs = []
            pg.on('pageerror', lambda e: errs.append('pageerror: ' + str(e)))
            pg.on('console', lambda m: errs.append('console: ' + m.text) if m.type == 'error' else None)
            pg.goto(f'http://localhost:{port}/test.html', wait_until='networkidle', timeout=180000)
            pg.wait_for_selector('#stage.ready', timeout=180000)
            time.sleep(1.5)
            r = {}
            r['tabs'] = ev(pg, "[...document.querySelectorAll('.pose-tab')].map(b => { const q = b.getBoundingClientRect(); return b.textContent + ' ' + Math.round(q.left) + ',' + Math.round(q.top) + ' ' + Math.round(q.width) + 'x' + Math.round(q.height); })")
            r['hscroll'] = ev(pg, "document.documentElement.scrollWidth > document.documentElement.clientWidth")
            pg.click('[data-pose="handstand"]'); time.sleep(5.0)
            r['pose'] = ev(pg, "window.__dbg.state.poseId")
            r['pressed'] = ev(pg, "document.querySelector('[data-pose=handstand]').getAttribute('aria-pressed')")
            shot(pg, '#stage', f'{outp}_{tag}_hs_muscles.png')
            ev(pg, "window.__dbg.setLayer('musclesOuter', false); window.__dbg.setLayer('musclesDeep', false)"); time.sleep(1.5)
            shot(pg, '#stage', f'{outp}_{tag}_hs_bones.png')
            ev(pg, "window.__dbg.setLayer('musclesOuter', true); window.__dbg.setLayer('musclesDeep', true)"); time.sleep(1.0)
            ev(pg, "window.__dbg.setT(0.5)"); time.sleep(2.0)
            shot(pg, '#stage', f'{outp}_{tag}_hs_half.png')
            ev(pg, "window.__dbg.setT(1)"); time.sleep(1.5)
            pg.set_viewport_size({'width': vp[0], 'height': 3200}); time.sleep(0.8)
            r['cuesTitle'] = ev(pg, "(document.querySelector('#cues .card-eyebrow')||{}).textContent")
            r['cues'] = ev(pg, "[...document.querySelectorAll('#cues li')].map(l => l.textContent)")
            r['load'] = ev(pg, "[...document.querySelectorAll('#cues .meter-foot, #cues .fl-legend li')].map(l => l.textContent)")
            shot(pg, '#cues', f'{outp}_{tag}_hs_cues.png')
            r['summary'] = ev(pg, "document.querySelector('#info').innerText")
            shot(pg, '#info', f'{outp}_{tag}_hs_summary.png')
            for name, sel in [('fingerFlex', "{kind:'muscle',unit:'fingerFlex',side:'L'}"), ('bicShort', "{kind:'muscle',unit:'bicShort',side:'L'}"), ('lat', "{kind:'muscle',unit:'lat',side:'L'}"), ('wr', "{kind:'joint',id:'wr_L'}"), ('ls', "{kind:'joint',id:'ls'}"), ('gh', "{kind:'joint',id:'gh_L'}")]:
                ev(pg, f"window.__dbg.select({sel})"); time.sleep(0.8)
                r['card_' + name] = ev(pg, "document.querySelector('#info').innerText")
                shot(pg, '#info', f'{outp}_{tag}_hs_card_{name}.png')
            ev(pg, "window.__dbg.select(null)")
            pg.set_viewport_size({'width': vp[0], 'height': vp[1]}); time.sleep(0.6)
            # back to another pose and to handstand again through the hash/tab
            pg.click('[data-pose="fold"]'); time.sleep(3.0)
            r['fold_ok'] = ev(pg, "window.__dbg.state.poseId")
            pg.click('[data-pose="handstand"]'); time.sleep(3.0)
            shot(pg, 'body', f'{outp}_{tag}_hs_page.png')
            r['errs'] = errs
            res[tag] = r
            ctx.close()
        b.close()
finally:
    srv.terminate()
json.dump(res, open(outp + '_ui.json', 'w'), ensure_ascii=False, indent=1)
for tag, r in res.items():
    print('==', tag)
    print(' tabs', r['tabs']); print(' hscroll', r['hscroll'], 'pose', r['pose'], 'pressed', r['pressed'], 'fold', r['fold_ok'])
    print(' errs', r['errs'])
