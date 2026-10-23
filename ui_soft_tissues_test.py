"""Check loaded soft-tissue surfaces on desktop and phone layouts.

python3 ui_soft_tissues_test.py --out /tmp/fysm-ui --browser /usr/bin/chromium
Add --fetch-assets when Chromium cannot use the environment's HTTPS CA/proxy;
Python then fetches external assets with normal certificate verification.
Screenshots and results are written outside the repository.
"""
import argparse
import fcntl
from functools import partial
from hashlib import sha256
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
from threading import Thread
import urllib.request

from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parent


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--browser')
    parser.add_argument('--fetch-assets', action='store_true')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    assert not args.out.resolve().is_relative_to(ROOT), 'Keep screenshots outside the repository'
    lock_path = Path(tempfile.gettempdir()) / ('fysm-ui-' + sha256(str(ROOT).encode()).hexdigest()[:12] + '.lock')
    lock = lock_path.open('w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    manifest = json.loads((ROOT / 'muscles/muscles_bp3d.json').read_text())
    expected = len({(p['unit'], p['side']) for p in manifest['parts']})
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(ROOT / 'dist')))
    Thread(target=server.serve_forever, daemon=True).start()
    cache, report = {}, {}

    def fetch_asset(route):
        url = route.request.url
        if url not in cache:
            with urllib.request.urlopen(url, timeout=30) as response:
                headers = {k: v for k, v in response.headers.items()
                           if k.lower() not in ('content-encoding', 'content-length', 'transfer-encoding')}
                cache[url] = (response.status, headers, response.read())
        status, headers, body = cache[url]
        route.fulfill(status=status, headers=headers, body=body)

    # Waiting for two animation frames lets applyPose update paths and skinning
    # before checking the chosen pose or taking its screenshot.
    def settled(page):
        page.evaluate('() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))')
        page.evaluate('window.__dbg.resetView(false)')
        settled_script = '() => new Promise(r => requestAnimationFrame(r))'
        page.evaluate(settled_script)

    try:
        with sync_playwright() as playwright:
            options = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
            if args.browser:
                options['executable_path'] = args.browser
            browser = playwright.chromium.launch(**options)
            for name, width, height in [('desktop', 1300, 820), ('phone', 390, 844)]:
                context = browser.new_context(viewport={'width': width, 'height': height}, reduced_motion='reduce')
                if args.fetch_assets:
                    context.route('https://**/*', fetch_asset)
                page = context.new_page()
                page.set_default_timeout(60000)
                errors, warnings = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('console', lambda message: warnings.append(message.text) if message.type in ('error', 'warning') else None)
                page.goto(f'http://127.0.0.1:{server.server_port}/test.html', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"]', timeout=120000)
                result = {'poses': [], 'errors': errors, 'warnings': warnings}
                count = page.evaluate('window.__dbg.muscleSurfaces.covered.size')
                assert count == expected, (count, expected)
                assert page.evaluate('window.__dbg.connectiveSurfaces.parts.length') == 63
                assert page.evaluate("document.characterSet.toLowerCase() === 'utf-8'")
                assert page.evaluate('document.documentElement.scrollWidth <= document.documentElement.clientWidth')
                if name == 'desktop':
                    assert page.evaluate('document.documentElement.scrollHeight <= document.documentElement.clientHeight')
                for pose in ['dog', 'plank', 'half', 'bow', 'fold', 'handstand']:
                    page.locator(f'[data-pose="{pose}"]').click()
                    for t in [0, 0.5, 1]:
                        page.evaluate('(t) => window.__dbg.setT(t)', t)
                        settled(page)
                        status = page.evaluate('''() => {
                          const d = window.__dbg;
                          return {pose:d.state.poseId,t:d.state.t,
                            visible:d.muscleSurfaces.pickables.filter(m=>m.visible).length,
                            duplicate:d.strands.filter(s=>s.mesh.visible && s.inst.kind==='muscle' && d.muscleSurfaces.covered.has(s.inst.unit+'_'+s.inst.side)).length};
                        }''')
                        assert status['pose'] == pose and status['t'] == t and status['duplicate'] == 0
                        assert status['visible'] == expected
                        result['poses'].append(status)
                    page.locator('#stage').screenshot(path=str(args.out / f'{name}-{pose}.png'), animations='disabled')
                    print(name, pose, 'loaded surfaces:', count, flush=True)
                # Exercise the actual pointer path, including CPU-skinned
                # raycasting. Programmatic select() alone would miss a broken
                # pick list or stale moving bounds.
                page.evaluate("for(const l of ['bones','joints','ligs'])window.__dbg.setLayer(l,false)")
                settled(page)
                probe = page.evaluate('''() => {
                  const d=window.__dbg,T=d.THREE,rect=document.getElementById('view').getBoundingClientRect();
                  const surfaces=d.muscleSurfaces.pickables.filter(m=>m.visible),ray=new T.Raycaster();
                  for(const mesh of surfaces) {
                    const ids=mesh.geometry.index;
                    for(let k=0;k<ids.count;k+=Math.max(3,Math.floor(ids.count/24/3)*3)) {
                      const p=new T.Vector3();
                      for(let j=0;j<3;j++)p.add(mesh.getVertexPosition(ids.getX(k+j),new T.Vector3()));
                      mesh.localToWorld(p.divideScalar(3));p.project(d.camera);
                      if(Math.abs(p.x)>.9||Math.abs(p.y)>.9||p.z<0||p.z>1)continue;
                      ray.setFromCamera(new T.Vector2(p.x,p.y),d.camera);
                      const hit=ray.intersectObjects(surfaces,false)[0];if(!hit)continue;
                      const x=rect.left+(p.x+1)*rect.width/2,y=rect.top+(1-p.y)*rect.height/2;
                      if(y>innerHeight-20)continue;
                      return {x,y,unit:hit.object.userData.unit,side:hit.object.userData.side};
                    }
                  }
                  return null;
                }''')
                assert probe, 'No visible muscle triangle available for pointer selection'
                page.mouse.click(probe['x'], probe['y'])
                selected = page.evaluate('window.__dbg.state.sel')
                assert selected and selected['kind'] == 'muscle' and selected['unit'] == probe['unit'], (selected, probe)
                result['pointer_pick'] = selected
                page.evaluate("window.__dbg.select(null);for(const l of ['bones','joints','ligs'])window.__dbg.setLayer(l,true)")
                page.evaluate("window.__dbg.setLayer('musclesOuter',false);window.__dbg.setLayer('musclesDeep',false)")
                assert page.evaluate('window.__dbg.muscleSurfaces.pickables.every(m=>!m.visible)')
                page.locator('#stage').screenshot(path=str(args.out / f'{name}-connective.png'), animations='disabled')
                page.evaluate("window.__dbg.setLayer('musclesOuter',true);window.__dbg.setLayer('musclesDeep',true);window.__dbg.setLightMode('effort');window.__dbg.select({kind:'muscle',unit:'bicLong',side:'L'})")
                settled(page)
                assert 'BodyParts3D' in page.locator('#info').inner_text()
                page.evaluate("window.__dbg.select({kind:'muscle',unit:'lat',side:'L'})")
                assert 'реконструкц' in page.locator('#info').inner_text()
                page.evaluate('window.__dbg.select(null);window.__dbg.enterCycle();window.__dbg.cycPause()')
                for t in [0, 4.2, 10.6, 15.5, 20.5, 23, 26.1]:
                    page.evaluate('(t)=>window.__dbg.setCycT(t)', t)
                    settled(page)
                    assert page.evaluate('window.__dbg.state.mode') == 'cycle'
                page.locator('#stage').screenshot(path=str(args.out / f'{name}-cycle.png'), animations='disabled')
                page.evaluate("window.__dbg.switchPose('dog')")
                settled(page)
                assert page.evaluate('window.__dbg.state.mode') == 'pose'
                result['coverage_note'] = page.locator('#muscle-source-note').inner_text()
                assert not errors, errors
                report[name] = result
                context.close()
            context = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='reduce')
            if args.fetch_assets:
                context.route('https://**/*', fetch_asset)
            page = context.new_page()
            page.goto(f'http://127.0.0.1:{server.server_port}/index.html', wait_until='networkidle')
            page.wait_for_selector('#stage[data-muscle-surfaces="ready"]', timeout=120000)
            report['production'] = page.evaluate('''() => ({charset:document.characterSet,
              debug:typeof window.__dbg,viewport:document.querySelector('meta[name=viewport]').content,
              overflow:document.documentElement.scrollWidth>document.documentElement.clientWidth,
              title:document.title})''')
            assert report['production']['debug'] == 'undefined'
            assert report['production']['charset'] == 'UTF-8'
            assert report['production']['title'] == 'Анатомия шести асан'
            assert not report['production']['overflow']
            context.close()
            context = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='no-preference')
            if args.fetch_assets:
                context.route('https://**/*', fetch_asset)
            page = context.new_page()
            animation_errors = []
            page.on('pageerror', lambda error: animation_errors.append(str(error)))
            page.goto(f'http://127.0.0.1:{server.server_port}/test.html#cycle', wait_until='networkidle')
            page.wait_for_selector('#stage[data-muscle-surfaces="ready"]', timeout=120000)
            page.evaluate('window.__dbg.cycPause();window.__dbg.setCycT(8);window.__dbg.cycPlay()')
            start = page.evaluate('window.__dbg.state.cycT')
            page.wait_for_timeout(1500)
            page.evaluate('window.__dbg.cycPause()')
            end = page.evaluate('window.__dbg.state.cycT')
            assert end > start, 'The round did not advance with motion enabled'
            page.evaluate("window.__dbg.switchPose('bow')")
            page.wait_for_timeout(1500)
            settled(page)
            assert page.evaluate('window.__dbg.state.poseId') == 'bow'
            assert not animation_errors, animation_errors
            report['animation'] = {'cycle_advanced': end - start, 'morph_to': 'bow', 'errors': animation_errors}
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('PASS: desktop, phone, production HTML, six poses, round, layers, coverage and provenance', flush=True)


if __name__ == '__main__':
    main()
