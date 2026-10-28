"""Check the 24 static FYSM reconstructions on the real built page.

python3 ui_practice_test.py --out /tmp/fysm-practice --browser /usr/bin/chromium --fetch-assets
The test snapshots the built HTML before opening it. It verifies presentation,
geometry selection and mode isolation; it does not validate anatomy or forces.
"""
import argparse
import fcntl
from functools import partial
from hashlib import sha256
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import shutil
import tempfile
from threading import Thread
import urllib.request

from PIL import Image, ImageDraw
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent
SHORT = 'в|во|и|не|ни|до|на|с|со|к|ко|о|об|от|у|по|за|из|а|но|же|для|при|без|над|под|про|или|как|что|это|то|их|его|её|ее|чем|чём|уже'
BREAKABLE_SHORT = re.compile(r'(^|[\s(«„"])(' + SHORT + r')[\t\r\n ]+', re.IGNORECASE)


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def settle(page):
    page.evaluate('() => new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))')
    page.evaluate('window.__dbg.resetView(false)')
    page.evaluate('() => new Promise(r=>requestAnimationFrame(r))')


def frame_report(page):
    return page.evaluate('''() => {
      const d=window.__dbg, p=d.practiceCurrent(), c=d.displayConf, g=p.geometry;
      const Q = values => new d.THREE.Quaternion(...values).normalize();
      const local=Object.fromEntries(Object.entries(g.local).map(([k,v])=>[k,Q(v)]));
      const W=d.A.computeWorld(local,Q(g.root),new d.THREE.Vector3(...g.rootP));
      const error=Math.max(...d.A.SEG.map(s=>d.W[s.name].p.distanceTo(W[s.name].p)));
      const qerror=Math.max(...d.A.SEG.map(s=>1-Math.abs(c.local[s.name].dot(local[s.name]))));
      let greenSurfaceVertices=0;
      for(const mesh of d.muscleSurfaces?.pickables || []){
        const colors=mesh.geometry.attributes.color.array;
        for(let i=0;i<colors.length;i+=3) if(colors[i+1]>colors[i]+1e-6) greenSurfaceVertices++;
      }
      return {id:p.step.id,mode:d.state.mode,algorithm:d.state.algorithmId,step:d.state.practiceStep,
        positionErrorM:error,quaternionDotError:qerror,rootErrorM:c.rootP.distanceTo(new d.THREE.Vector3(...g.rootP)),
        minMarkerM:Math.min(...d.A.LANDMARKS.map(l=>d.A.pt(d.W,l.seg,l.pc).y)),
        activeMuscles:d.strands.filter(s=>s.inst.kind==='muscle'&&s.on).length,greenSurfaceVertices,
        playing:!!d.state.playing,cyclePlaying:!!d.cyc.playing,morph:!!d.morph,
        forces:d.state.forces,visibleForceSets:d.forcesGroup.children.filter(g=>g.visible).length,
        lightMode:d.state.lightMode,t:d.state.t,
        overflow:document.documentElement.scrollWidth>document.documentElement.clientWidth,
        webgl:document.getElementById('view').getContext('webgl2').getError()};
    }''')


def verify_frame(page, step, algorithm):
    result = frame_report(page)
    assert result['id'] == step['id'], result
    assert result['mode'] == 'practice' and result['algorithm'] == algorithm['id'], result
    assert result['positionErrorM'] < 1e-10 and result['rootErrorM'] < 1e-10, result
    assert abs(result['quaternionDotError']) < 1e-8, result
    assert abs(result['minMarkerM']) < 1e-5, result
    assert result['activeMuscles'] == result['greenSurfaceVertices'] == result['visibleForceSets'] == 0, result
    assert not any(result[k] for k in ['playing', 'cyclePlaying', 'morph', 'forces', 'overflow', 'webgl']), result
    assert result['t'] == 1 and result['lightMode'] == 'length', result
    assert page.locator('#pose-player').is_hidden() and page.locator('#cycle-player').is_hidden()
    assert page.locator('[data-lightmode="effort"]').is_hidden()
    assert page.locator('[data-layer="forces"]').is_hidden()
    assert page.locator('.research-comparison').count() == 0
    assert ' '.join(page.locator('#pose-name').inner_text().split()) == step['name']
    assert ' '.join(page.locator('#pose-sanskrit').text_content().split()) == step['sanskrit']
    assert 'не рассчитана' in ' '.join(page.locator('#practice-caption').inner_text().split())
    assert page.locator('#practice-prev').is_disabled() == (step['number'] == 1)
    assert page.locator('#practice-next').is_disabled() == (step['number'] == 8)
    assert page.locator('[data-practice-step][aria-current="step"]').count() == 1
    for selector in ['#pose-name', '#pose-sanskrit', '#practice-navigation', '#cues', '#info']:
        text = page.locator(selector).text_content()
        assert not BREAKABLE_SHORT.search(text), ('Hanging short word', selector, text)
    return result


def contact_sheet(files, target):
    cell_w, cell_h = 320, 310
    canvas = Image.new('RGB', (cell_w * 4, cell_h * 6), '#0d1215')
    draw = ImageDraw.Draw(canvas)
    for i, (label, filename) in enumerate(files):
        image = Image.open(filename).convert('RGB')
        image.thumbnail((cell_w, cell_h - 30))
        x, y = i % 4 * cell_w, i // 4 * cell_h
        canvas.paste(image, (x + (cell_w - image.width) // 2, y + 30))
        draw.text((x + 12, y + 9), label, fill='#ece4d3')
    canvas.save(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--browser')
    parser.add_argument('--fetch-assets', action='store_true')
    parser.add_argument('--preview', action='store_true', help='Only render/check the 24 poses at desktop size')
    parser.add_argument('--interactions-only', action='store_true', help='Skip pose screenshots for a quick UI regression pass')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    assert not args.out.resolve().is_relative_to(ROOT)
    site = args.out / 'site'
    site.mkdir(exist_ok=True)
    for filename in ['index.html', 'test.html']:
        shutil.copyfile(ROOT / 'dist' / filename, site / filename)
    html = (site / 'index.html').read_bytes()
    assert b'__dbg' not in html
    manifest = json.loads((ROOT / 'practice/algorithms.json').read_text())
    report = {'htmlSha256': sha256(html).hexdigest(), 'preview': args.preview, 'interactionsOnly': args.interactions_only,
              'sourceSha256': manifest['source']['sha256']}
    lock = (Path(tempfile.gettempdir()) / ('fysm-ui-' + sha256(str(ROOT).encode()).hexdigest()[:12] + '.lock')).open('w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(site)))
    Thread(target=server.serve_forever, daemon=True).start()
    origin = f'http://127.0.0.1:{server.server_port}'
    cache = {}

    def fetch_asset(route):
        url = route.request.url
        if url not in cache:
            with urllib.request.urlopen(url, timeout=45) as response:
                headers = {k: v for k, v in response.headers.items()
                           if k.lower() not in ('content-encoding', 'content-length', 'transfer-encoding')}
                cache[url] = response.status, headers, response.read()
        status, headers, body = cache[url]
        route.fulfill(status=status, headers=headers, body=body)

    try:
        with sync_playwright() as playwright:
            options = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
            if args.browser:
                options['executable_path'] = args.browser
            browser = playwright.chromium.launch(**options)
            sizes = [('desktop', 1300, 900)] if args.preview else [('desktop', 1300, 900), ('phone', 390, 844)]
            for name, width, height in sizes:
                # Exercise the regular-motion path: a static practice step must
                # remain static even when reduced motion is not requested.
                context = browser.new_context(viewport={'width': width, 'height': height}, reduced_motion='no-preference')
                if args.fetch_assets:
                    context.route('https://**/*', fetch_asset)
                page = context.new_page()
                page.set_default_timeout(120000)
                errors, warnings = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('console', lambda message: warnings.append(message.text) if message.type in ('warning', 'error') else None)
                page.goto(origin + '/test.html', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]')
                page.wait_for_function('typeof window.__dbg?.enterPractice === "function"')
                result = {'frames': [], 'errors': errors, 'warnings': warnings}
                report[name] = result
                page.locator('#practice-mode').click()
                files = []
                for algorithm in manifest['algorithms']:
                    if args.interactions_only:
                        break
                    page.locator(f'[data-algorithm="{algorithm["id"]}"]').click()
                    for index, step in enumerate(algorithm['steps']):
                        page.locator(f'[data-practice-step="{index}"]').click()
                        settle(page)
                        frame = verify_frame(page, step, algorithm)
                        result['frames'].append(frame)
                        page.locator('#view').scroll_into_view_if_needed()
                        target = args.out / f'{name}-{step["id"]}.png'
                        page.locator('#stage').screenshot(path=str(target), animations='disabled')
                        files.append((step['id'], target))
                        print(name, step['id'], 'PASS', flush=True)
                if files:
                    contact_sheet(files, args.out / f'{name}-all24.png')
                page.screenshot(path=str(args.out / f'{name}-interface.png'), full_page=True, animations='disabled')
                if args.preview:
                    assert not errors and not warnings, (errors, warnings)
                    context.close()
                    continue

                # The controls cannot force practice geometry into a made-up
                # standing interpolation, effort mode or a stale force set.
                before = frame_report(page)
                page.evaluate('window.__dbg.setT(.2);window.__dbg.setLightMode("effort");window.__dbg.setLayer("forces",true)')
                settle(page)
                after = frame_report(page)
                assert before == after, (before, after)

                # Compare both actual side strains, rather than a mirrored
                # left-only value from the original static physics exporter.
                page.evaluate('window.__dbg.enterPractice("centering",2)')
                settle(page)
                case = page.evaluate('''() => {
                  const d=window.__dbg, groups={};
                  for(const s of d.strands.filter(s=>s.inst.kind==='muscle')){
                    const g=(groups[s.inst.unit]||={L:[],R:[]}); g[s.inst.side]?.push(s.strain);
                  }
                  return Object.entries(groups).map(([unit,g])=>({unit,
                    L:g.L.reduce((a,b)=>a+b,0)/g.L.length,R:g.R.reduce((a,b)=>a+b,0)/g.R.length}))
                    .filter(g=>Number.isFinite(g.L)&&Number.isFinite(g.R)).sort((a,b)=>Math.abs(b.L-b.R)-Math.abs(a.L-a.R))[0];
                }''')
                assert abs(case['L'] - case['R']) > .01, case
                page.evaluate('(unit)=>window.__dbg.select({kind:"muscle",unit,side:"L"})', case['unit'])
                values = page.locator('#info .practice-lengths dd').all_text_contents()
                # UI uses an explicit plus only for positive values.
                for actual, value in zip(values, [case['L'], case['R']]):
                    numeric = float(re.sub(r'[^\d,.+−-]', '', actual).replace(',', '.').replace('−', '-'))
                    assert abs(numeric-value * 100) <= .51, (actual, value)
                page.locator('[data-practice-side="R"]').click()
                assert page.evaluate('window.__dbg.state.sel.side') == 'R'
                assert 'справа' in page.locator('#info').inner_text().lower()
                assert page.locator('#info .tag-active,.research-comparison').count() == 0
                page.locator('#info .card-focus').click()
                page.wait_for_function('!window.__dbg.state.camAnim', timeout=15000)
                focus = page.evaluate('''() => {const d=window.__dbg,s=d.state.sel;
                  const left=d.muscleSurfaces.getBounds(s.unit,'L').getCenter(new d.THREE.Vector3());
                  const right=d.muscleSurfaces.getBounds(s.unit,'R').getCenter(new d.THREE.Vector3());
                  return {rightError:d.controls.target.distanceTo(right),leftDistance:d.controls.target.distanceTo(left)}; }''')
                assert focus['rightError'] < 1e-7 and focus['leftDistance'] > .02, focus
                settle(page)
                result['sideStrains'] = {**case, 'shown': values}
                result['rightSideFocus'] = focus
                page.evaluate('window.__dbg.select({kind:"joint",id:"hip_R"})')
                angles = page.evaluate('window.__dbg.jointById.hip_R.res.comps.map(c=>c.value)')
                shown = page.locator('#info .parts.comps .num').all_text_contents()
                assert len(shown) == len(angles)
                for actual, expected in zip(shown, angles):
                    numeric = float(re.sub(r'[^\d,.+−-]', '', actual).replace(',', '.').replace('−', '-'))
                    assert abs(numeric-expected) <= .51, (actual, expected)
                result['rightJointAngles'] = angles
                print(name, 'PASS: independent side lengths and right joint angles', flush=True)

                for layer in ['musclesOuter', 'musclesDeep']:
                    page.evaluate('(layer)=>window.__dbg.setLayer(layer,false)', layer)
                assert page.evaluate('window.__dbg.muscleSurfaces.pickables.every(m=>!m.visible)')
                for layer in ['musclesOuter', 'musclesDeep', 'fascia', 'flines']:
                    page.evaluate('(layer)=>window.__dbg.setLayer(layer,true)', layer)
                settle(page)
                assert page.evaluate('window.__dbg.muscleSurfaces.pickables.every(m=>m.visible)')
                assert page.evaluate('window.__dbg.fasciaM !== null')
                page.evaluate('window.__dbg.select({kind:"flines"})')
                assert 'включённость' not in page.locator('#info').inner_text().lower()
                for layer in ['fascia', 'flines']:
                    page.evaluate('(layer)=>window.__dbg.setLayer(layer,false)', layer)
                page.evaluate('window.__dbg.select(null)')
                page.locator('.practice-reference summary').click()
                page.locator('.practice-reference img').scroll_into_view_if_needed()
                assert page.locator('.practice-reference img').evaluate('(img)=>img.complete&&img.naturalWidth>1000')
                page.locator('.practice-reference summary').click()

                # Real click picking still works when the stage shows a
                # practice configuration instead of a calculated pose.
                page.locator('#view').scroll_into_view_if_needed()
                candidates = page.evaluate('''() => {const d=window.__dbg,rect=document.getElementById('view').getBoundingClientRect();
                  return d.strands.filter(s=>s.inst.kind==='muscle').slice(0,100).map(s=>{
                    const p=s.center[Math.floor(s.center.length/2)].clone().project(d.camera);
                    return {x:rect.left+(p.x+1)/2*rect.width,y:rect.top+(1-p.y)/2*rect.height,z:p.z};
                  }).filter(p=>p.z<1&&p.x>rect.left&&p.x<rect.right&&p.y>rect.top&&p.y<rect.bottom); }''')
                for point in candidates[:24]:
                    page.mouse.click(point['x'], point['y'])
                    if page.evaluate('window.__dbg.state.sel !== null'):
                        break
                assert page.evaluate('window.__dbg.state.sel !== null'), 'Canvas picking did not select a visible tissue'
                result['picked'] = page.evaluate('window.__dbg.state.sel')
                print(name, 'PASS: layers, fascia, source illustration and click picking', flush=True)

                # Buttons support normal keyboard interaction, and boundaries
                # neither wrap nor leave a missing ninth frame selected.
                page.locator('[data-algorithm="detali"]').click()
                page.locator('#practice-next').focus()
                page.keyboard.press('Enter')
                assert page.evaluate('window.__dbg.state.practiceStep') == 1
                page.locator('#practice-prev').focus()
                page.keyboard.press('Space')
                assert page.evaluate('window.__dbg.state.practiceStep') == 0
                assert page.locator('#practice-prev').is_disabled()
                page.locator('[data-practice-step="7"]').click()
                assert page.locator('#practice-next').is_disabled()

                page.evaluate('window.__dbg.exitPractice();window.__dbg.switchPose("bow");window.__dbg.setT(.37);window.__dbg.setLightMode("effort");window.__dbg.setLayer("forces",true)')
                settle(page)
                page.locator('#practice-mode').click()
                assert page.evaluate('window.__dbg.state.mode') == 'practice'
                page.locator('#main-mode').click()
                settle(page)
                assert page.evaluate('({mode:window.__dbg.state.mode,id:window.__dbg.state.poseId,t:window.__dbg.state.t,light:window.__dbg.state.lightMode,forces:window.__dbg.state.forces})') == {'mode': 'pose', 'id': 'bow', 't': .37, 'light': 'effort', 'forces': True}
                assert page.locator('#pose-player').is_visible()
                page.evaluate('window.__dbg.enterCycle(8,false);window.__dbg.cycPause()')
                settle(page)
                page.locator('#practice-mode').click()
                page.locator('#main-mode').click()
                settle(page)
                assert page.evaluate('window.__dbg.state.mode') == 'cycle'
                assert abs(page.evaluate('window.__dbg.state.cycT')-8) < 1e-9
                assert not page.evaluate('window.__dbg.cyc.playing')
                result['restorePoseAndCycle'] = True
                print(name, 'PASS: keyboard navigation and restoring pose/cycle', flush=True)

                # A deep link, reload and browser history all resolve the same
                # pose, and invalid algorithm/step combinations get a real pose.
                page.evaluate('window.__dbg.enterPractice("detali",6)')
                page.reload(wait_until='networkidle')
                page.wait_for_function('window.__dbg?.state.mode === "practice"', timeout=15000)
                settle(page)
                assert page.evaluate('window.__dbg.practiceCurrent().step.id') == 'detali-07'
                page.locator('#practice-next').click()
                page.go_back(wait_until='domcontentloaded')
                page.wait_for_function('window.__dbg.practiceCurrent().step.id === "detali-07"', timeout=15000)
                page.go_forward(wait_until='domcontentloaded')
                page.wait_for_function('window.__dbg.practiceCurrent().step.id === "detali-08"', timeout=15000)
                for invalid in ['practice/missing/99', 'practice/constructor/1', 'practice/__proto__/1', 'practice/toString/1']:
                    page.evaluate('(hash)=>{location.hash=hash}', invalid)
                    page.wait_for_function('window.__dbg.practiceCurrent().step.id === "struna-01"', timeout=15000)
                    page.wait_for_function('location.hash === "#practice/struna/1"', timeout=15000)
                settle(page)
                assert not page.evaluate('document.documentElement.scrollWidth>document.documentElement.clientWidth')
                assert not errors and not warnings, (errors, warnings)
                result['navigation'] = True
                context.close()
                print(name, 'PASS:', str(len(result['frames'])) + ' pose frames, independent sides, mode isolation, navigation and regressions', flush=True)

            if not args.preview:
                context = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='reduce')
                if args.fetch_assets:
                    context.route('https://**/*', fetch_asset)
                page = context.new_page()
                page.goto(origin + '/index.html#practice/centering/6', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]')
                assert 'Полубакасана' in page.locator('#pose-name').inner_text()
                assert page.locator('#practice-navigation').is_visible()
                assert page.locator('#pose-player').is_hidden()
                assert page.evaluate('typeof window.__dbg') == 'undefined'
                assert not page.evaluate('document.documentElement.scrollWidth>document.documentElement.clientWidth')
                assert page.evaluate('document.characterSet') == 'UTF-8'
                page.screenshot(path=str(args.out / 'production-phone.png'), full_page=True)
                report['production'] = {'debug': False, 'deepLink': 'centering-06', 'overflow': False, 'charset': 'UTF-8'}
                production_errors = []
                page.on('pageerror', lambda error: production_errors.append(str(error)))
                for invalid in ['constructor', '__proto__', 'toString']:
                    page.goto(origin + '/index.html#practice/' + invalid + '/1', wait_until='networkidle')
                    page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]')
                    assert page.evaluate('location.hash') == '#practice/struna/1'
                    assert ' '.join(page.locator('#pose-name').inner_text().split()) == manifest['algorithms'][0]['steps'][0]['name']
                assert not production_errors, production_errors
                report['production']['prototypeHashCases'] = True
                context.close()
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('PASS: practice UI' + (' preview' if args.preview else ' on desktop/phone and production'), flush=True)


if __name__ == '__main__':
    main()
