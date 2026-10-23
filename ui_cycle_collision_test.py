"""Check the real half-backbend to bow transition and save matched arm views.

python3 ui_cycle_collision_test.py --out /tmp/fysm-collision --browser /usr/bin/chromium --fetch-assets
Use --source DIR to serve a saved build, and --baseline to record the known bad
transition without claiming it passes the arm-side regression.
The separate numerical collision suite checks body-volume clearance; this check
verifies the decoded browser motion, side ordering and actual loaded rendering.
"""
import argparse
import fcntl
from functools import partial
from hashlib import sha256
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import tempfile
from threading import Thread
import urllib.request

from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parent
TIMES = [0, .9, 1.75, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3, 3.1, 3.2, 3.3, 3.8, 4.9]
SHOTS = [2.4, 2.6, 2.9, 3.1, 3.3]


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=ROOT / 'dist')
    parser.add_argument('--browser')
    parser.add_argument('--fetch-assets', action='store_true')
    parser.add_argument('--baseline', action='store_true')
    parser.add_argument('--baseline-cameras', type=Path)
    args = parser.parse_args()
    assert not args.out.resolve().is_relative_to(ROOT), 'Screenshots must stay outside git'
    args.out.mkdir(parents=True, exist_ok=True)
    site = args.out / 'site'
    site.mkdir(exist_ok=True)
    for name in ['test.html', 'index.html']:
        shutil.copyfile(args.source / name, site / name)
    html = (site / 'index.html').read_bytes()
    assert b'__dbg' not in html
    report = {'baseline': args.baseline, 'html_sha256': sha256(html).hexdigest(), 'states': {}}
    baseline = json.loads(args.baseline_cameras.read_text()) if args.baseline_cameras else None
    lock = (Path(tempfile.gettempdir()) / ('fysm-ui-' + sha256(str(ROOT).encode()).hexdigest()[:12] + '.lock')).open('w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(site)))
    Thread(target=server.serve_forever, daemon=True).start()
    cache = {}

    def fetch_asset(route):
        url = route.request.url
        if url not in cache:
            with urllib.request.urlopen(url, timeout=30) as response:
                headers = {k: v for k, v in response.headers.items()
                           if k.lower() not in ('content-encoding', 'content-length', 'transfer-encoding')}
                cache[url] = response.status, headers, response.read()
        status, headers, body = cache[url]
        route.fulfill(status=status, headers=headers, body=body)

    def settle(page):
        page.evaluate('() => new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))')

    def point_state(page):
        return page.evaluate('''() => {
          const d=window.__dbg,W=d.W,p=W.pelvis.p;
          return {mode:d.state.mode,t:d.state.cycT,
            leftElbowX:W.fore_L.p.x-p.x,rightElbowX:W.fore_R.p.x-p.x,
            leftWristX:W.hand_L.p.x-p.x,rightWristX:W.hand_R.p.x-p.x,
            elbowSeparation:W.fore_L.p.distanceTo(W.fore_R.p),
            wristSeparation:W.hand_L.p.distanceTo(W.hand_R.p),
            symmetryError:Math.max(...['hum','fore','hand'].map(s=>
              Math.abs(W[s+'_L'].p.x+W[s+'_R'].p.x-2*p.x)+
              Math.abs(W[s+'_L'].p.y-W[s+'_R'].p.y)+Math.abs(W[s+'_L'].p.z-W[s+'_R'].p.z)))};
        }''')

    def camera(page, view, previous=None):
        if previous:
            return page.evaluate('''(v)=>{
              const d=window.__dbg;d.state.userMoved=true;d.state.camAnim=null;
              d.controls.target.fromArray(v.target);d.camera.up.fromArray(v.up);
              d.camera.position.fromArray(v.camera);d.camera.lookAt(d.controls.target);
              d.controls.update();d.camera.updateMatrixWorld(true);return v;
            }''', previous)
        return page.evaluate('''(view)=>{
          const d=window.__dbg,T=d.THREE,W=d.W;
          const target=W.hand_L.p.clone().add(W.hand_R.p).multiplyScalar(.5);
          target.lerp(W.fore_L.p.clone().add(W.fore_R.p).multiplyScalar(.5),.45);
          const offset=view==='top'?new T.Vector3(.04,1.13,-.18):new T.Vector3(.55,.52,-.85);
          d.state.userMoved=true;d.state.camAnim=null;
          d.controls.target.copy(target);d.camera.up.set(0,1,0);
          d.camera.position.copy(target).add(offset);d.camera.lookAt(target);
          d.controls.update();d.camera.updateMatrixWorld(true);
          return {camera:d.camera.position.toArray(),target:target.toArray(),up:d.camera.up.toArray()};
        }''', view)

    try:
        with sync_playwright() as playwright:
            options = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
            if args.browser:
                options['executable_path'] = args.browser
            browser = playwright.chromium.launch(**options)
            for layout, width, height in [('desktop', 1200, 850), ('phone', 390, 844)]:
                context = browser.new_context(viewport={'width': width, 'height': height}, reduced_motion='reduce')
                if args.fetch_assets:
                    context.route('https://**/*', fetch_asset)
                page = context.new_page()
                page.set_default_timeout(90000)
                errors, warnings = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('console', lambda message: warnings.append(message.text) if message.type in ('warning', 'error') else None)
                page.goto(f'http://127.0.0.1:{server.server_port}/test.html', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]', timeout=120000)
                page.evaluate('window.__dbg.enterCycle();window.__dbg.cycPause();window.__dbg.select(null)')
                seg = page.evaluate("window.__dbg.CYC.segs.find(s=>s.from==='half'&&s.to==='bow')")
                assert seg and abs(seg['t1']-seg['t0']-4.9) < 1e-6
                result = {'states': [], 'cameras': {}, 'errors': errors, 'warnings': warnings}
                # Include every middle-of-frame point as well as the published
                # frames. A safe keyframe must not hide a crossing during slerp.
                samples = sorted(set(TIMES + [i / 20 for i in range(99)]))
                for local_t in samples:
                    page.evaluate('(t)=>window.__dbg.setCycT(t)', seg['t0'] + local_t)
                    settle(page)
                    state = point_state(page)
                    state['localT'] = local_t
                    assert state['mode'] == 'cycle' and abs(state['t']-seg['t0']-local_t) < 1e-6
                    assert state['symmetryError'] < 1e-5, state
                    if not args.baseline:
                        assert state['leftElbowX'] > 0 and state['rightElbowX'] < 0, state
                        assert state['leftWristX'] > 0 and state['rightWristX'] < 0, state
                    result['states'].append(state)
                    if local_t in SHOTS:
                        for view in (['top', 'feet'] if layout == 'desktop' else ['top']):
                            key = f'{local_t:.2f}-{view}'
                            previous = baseline['states'][layout]['cameras'][key] if baseline else None
                            result['cameras'][key] = camera(page, view, previous)
                            settle(page)
                            page.locator('#stage').screenshot(path=str(args.out / f'{layout}-{local_t:.2f}-{view}.png'), animations='disabled')
                assert page.evaluate('document.documentElement.scrollWidth<=document.documentElement.clientWidth')
                assert page.evaluate("document.getElementById('view').getContext('webgl2').getError()") == 0
                assert not errors and not warnings, (errors, warnings)
                report['states'][layout] = result
                context.close()
                print(layout, 'recorded' if args.baseline else 'PASS', len(samples), 'motion states', flush=True)
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('Recorded baseline' if args.baseline else 'PASS: browser arm-side ordering, interpolation and rendering', flush=True)


if __name__ == '__main__':
    main()
