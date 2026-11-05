"""Check the real practice force estimates, sides and rendering on the built page.

python3 ui_practice_physics_test.py --out /tmp/fysm-practice-physics --browser /usr/bin/chromium --fetch-assets --all-screenshots
This validates implementation and source identity, not personal biomechanics.
"""
import argparse
import fcntl
from functools import partial
from hashlib import sha256
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import shutil
import tempfile
from threading import Thread
import urllib.request

from playwright.sync_api import sync_playwright
from ui_practice_test import QuietHandler, settle, contact_sheet, BREAKABLE_SHORT


ROOT = Path(__file__).resolve().parent


def number(text):
    return float(re.sub(r'[^\d,.+−-]', '', text).replace(',', '.').replace('−', '-'))


def known(record):
    return record.get('status') == 'estimated' and isinstance(record.get('a'), (int, float)) and math.isfinite(record['a'])


def replace_physics(html, value):
    replacement = 'const PRACTICE_PHYSICS = ' + json.dumps(value, ensure_ascii=False, separators=(',', ':')) + ';'
    result, count = re.subn(r'^const PRACTICE_PHYSICS = .*;$', lambda match: replacement, html, count=1, flags=re.M)
    assert count == 1, 'The built HTML must contain exactly one practice physics constant'
    return result


def frame(page, expected):
    flags = {unit + '_' + side: known(record) and record['a'] >= .08
             for unit, records in expected['muscles'].items() for side, record in records.items()}
    result = page.evaluate('''flags => {
      const d=window.__dbg,p=d.practiceCurrent(),g=p.geometry;
      const Q=v=>new d.THREE.Quaternion(...v).normalize();
      const W=d.A.computeWorld(Object.fromEntries(Object.entries(g.local).map(([id,q])=>[id,Q(q)])),Q(g.root),new d.THREE.Vector3(...g.rootP));
      const mismatched=[];let active=0,inactiveGreen=0,greenVertices=0;
      for(const strand of d.strands.filter(s=>s.inst.kind==='muscle')){
        const key=strand.inst.unit+'_'+strand.inst.side;
        if(strand.on!==flags[key])mismatched.push(key);
        active+=strand.on?1:0;
      }
      for(const mesh of d.muscleSurfaces?.pickables || []){
        const key=mesh.userData.unit+'_'+mesh.userData.side,colors=mesh.geometry.attributes.color.array;
        for(let i=0;i<colors.length;i+=3)if(colors[i+1]>colors[i]+1e-6){greenVertices++;if(!flags[key])inactiveGreen++;}
      }
      return {id:p.step.id,mode:d.state.mode,t:d.state.t,morph:!!d.morph,playing:!!d.state.playing,
        cyclePlaying:d.cyc.playing,positionErrorM:Math.max(...d.A.SEG.map(s=>d.W[s.name].p.distanceTo(W[s.name].p))),
        mismatched:[...new Set(mismatched)],activeStrands:active,greenVertices,inactiveGreen,
        skinningMethod:d.muscleSurfaces?.skinningMethod,
        overflow:document.documentElement.scrollWidth>document.documentElement.clientWidth,
        webgl:document.getElementById('view').getContext('webgl2').getError()};
    }''', flags)
    assert result['mode'] == 'practice' and result['t'] == 1
    assert result['positionErrorM'] < 1e-10 and not result['mismatched'], result
    assert not any(result[key] for key in ['morph', 'playing', 'cyclePlaying', 'inactiveGreen', 'overflow', 'webgl']), result
    assert result['skinningMethod'] == 'bend-aware-dual-quaternion', result
    assert page.locator('#pose-player').is_hidden() and page.locator('#cycle-player').is_hidden()
    assert page.locator('[data-lightmode="effort"]').is_visible()
    assert page.locator('[data-layer="forces"]').is_visible()
    assert 'оценки' in page.locator('#practice-caption').inner_text().lower()
    assert page.locator('#cues [data-practice-quality="verified"]').count() == 1
    assert 'Площадь вокруг кожных меток задана условно; это не измеренная площадь контакта.' in ' '.join(page.locator('#cues').inner_text().split())
    for selector in ['#pose-name', '#pose-sanskrit', '#practice-navigation', '#cues', '#info']:
        text = page.locator(selector).text_content()
        assert not BREAKABLE_SHORT.search(text), ('Hanging short word', selector, text)
        assert not re.search(r'\b(?:NaN|undefined|Infinity)\b', text), (selector, text)
    pads = [pad for pad in expected.get('diag', {}).get('contacts', {}).get('inferredPads', [])
            if pad.get('estimated') is True]
    if pads:
        notes = ' '.join(page.locator('#cues').inner_text().split())
        assert 'Опора включает расчётные площадки мягких тканей.' in notes
        assert 'без измерений кожи и давления.' in notes
        if any(pad.get('kind') == 'distal-phalanx-skin-proxy' for pad in pads):
            assert 'Подушечки пальцев оценены по костям с условной толщиной мягкой ткани 2 мм.' in notes
        if expected.get('modelLimits', {}).get('softTissueContactSensitivity', {}).get('verified') is True:
            assert 'Для ягодичной опоры отдельно рассчитаны исходная и вдвое меньшая площади.' in notes
            assert 'не доверительный интервал и не измеренная точность.' in notes
    result['inferredPadsDisclosed'] = len(pads)
    return result


def force_arrows(page, expected):
    arrows = []
    for point in expected['floor']:
        if point['pct'] >= .8:
            arrows.append({'P': [point['cop'][0], .002, point['cop'][2]], 'F': point['F']})
    for contact in expected.get('internalContacts', []):
        if math.hypot(*contact['F']) >= 5:
            arrows.append({'P': contact['P'], 'F': contact['F']})
            arrows.append({'P': contact['P'], 'F': [-value for value in contact['F']]})
    result = page.evaluate('''expected=>{
      const d=window.__dbg,visible=d.forcesGroup.children.filter(g=>g.visible);
      if(visible.length!==1)return{groups:visible.length};
      const arrows=visible[0].children.filter(g=>g.isGroup&&g.children.length===2);
      if(arrows.length!==expected.length)return{groups:1,count:arrows.length,expected:expected.length};
      const scale=.0045/(d.PHYS.dog.bw/100);let positionError=0,directionError=0,lengthError=0;
      arrows.forEach((arrow,i)=>{
        const F=new d.THREE.Vector3(...expected[i].F),P=new d.THREE.Vector3(...expected[i].P);
        positionError=Math.max(positionError,arrow.position.distanceTo(P));
        directionError=Math.max(directionError,new d.THREE.Vector3(0,1,0).applyQuaternion(arrow.quaternion).distanceTo(F.clone().normalize()));
        lengthError=Math.max(lengthError,Math.abs(arrow.children[0].scale.y+arrow.children[1].scale.y-Math.max(.012,F.length()*scale)));
      });return{groups:1,count:arrows.length,positionError,directionError,lengthError};
    }''', arrows)
    assert result['groups'] == 1 and result.get('count') == len(arrows), result
    assert max(result[key] for key in ['positionError', 'directionError', 'lengthError']) < 1e-9, result
    return result


def geometric_support(page, expected):
    polygon = expected['diag']['contacts']['unclusteredSupport']['poly']
    result = page.evaluate('''polygon=>{
      const group=window.__dbg.forcesGroup.children.find(g=>g.visible);
      const outline=group.children.find(g=>g.isLineLoop);
      if(polygon.length<3)return{count:outline?outline.geometry.attributes.position.count:0};
      if(!outline)return{count:0};
      const positions=outline.geometry.attributes.position;let error=0;
      if(positions.count===polygon.length)polygon.forEach((p,i)=>{
        error=Math.max(error,Math.abs(positions.getX(i)-p[0]),Math.abs(positions.getY(i)-.003),Math.abs(positions.getZ(i)-p[1]));
      });return{count:positions.count,error};
    }''', polygon)
    assert result['count'] == (len(polygon) if len(polygon) >= 3 else 0), result
    assert result.get('error', 0) < 1e-6, result
    return result


def methodology_check(args):
    """Check the final copy and an explicit, actually loaded internal contact."""
    assert not args.out.resolve().is_relative_to(ROOT)
    args.out.mkdir(parents=True, exist_ok=True)
    site = args.out / 'site'
    site.mkdir(exist_ok=True)
    for name in ['index.html', 'test.html']:
        shutil.copyfile(ROOT / 'dist' / name, site / name)
    html = (site / 'index.html').read_text()
    assert '__dbg' not in html
    report = {'snapshots': {name: sha256((site / name).read_bytes()).hexdigest()
                            for name in ['index.html', 'test.html']}, 'mode': 'methodology-only'}
    if args.baseline_html:
        baseline = args.baseline_html.read_text()
        old_copy = 'Для части внутренних контактов индивидуальная сила хвата неизвестна. Общая сила контакта ограничена допущениями модели.'
        new_copy = 'Предельная сила части внутренних контактов не определена для конкретного человека. Общая сила контакта ограничена допущениями модели.'
        assert baseline.count(old_copy) == 1 and baseline.replace(old_copy, new_copy) == html, 'Expected exactly the agreed copy-only HTML difference'
        report['copyOnlyDifference'] = True
        constants = ['POSE_DATA', 'PHYS', 'RESEARCH_SENSITIVITY', 'CYC_DATA', 'PRACTICE_DATA',
                     'PRACTICE_PHYSICS', 'MUSCLE_SURFACE_DATA', 'JOINT_SURFACE_DATA', 'CAPSULE_RIM_DATA']
        report['unchangedData'] = {}
        for name in constants:
            def literal(document):
                start = re.search(rf'^const {name} = ', document, re.M).end()
                _, end = json.JSONDecoder().raw_decode(document[start:])
                return document[start:start + end]
            old, current = literal(baseline), literal(html)
            assert old == current, ('Numerical/geometry pack changed during copy-only check', name)
            report['unchangedData'][name] = sha256(current.encode()).hexdigest()
        report['baselineHtmlSha256'] = sha256(baseline.encode()).hexdigest()
    expected = json.loads((ROOT / 'practice/physics.json').read_text())['poses']['centering-06']
    assert any(math.hypot(*contact['F']) >= 5 for contact in expected['internalContacts'])
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
                headers = {key: value for key, value in response.headers.items()
                           if key.lower() not in ('content-encoding', 'content-length', 'transfer-encoding')}
                cache[url] = response.status, headers, response.read()
        status, headers, body = cache[url]
        route.fulfill(status=status, headers=headers, body=body)

    try:
        with sync_playwright() as playwright:
            options = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
            if args.browser:
                options['executable_path'] = args.browser
            browser = playwright.chromium.launch(**options)
            for name, width, height in [('desktop', 1300, 900), ('phone', 390, 844)]:
                context = browser.new_context(viewport={'width': width, 'height': height}, reduced_motion='reduce')
                if args.fetch_assets:
                    context.route('https://**/*', fetch_asset)
                page = context.new_page()
                page.set_default_timeout(120000)
                errors, warnings = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('console', lambda message: warnings.append(message.text) if message.type in ('warning', 'error') else None)
                page.goto(origin + '/test.html#practice/centering/6', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]')
                page.evaluate('window.__dbg.setLayer("forces",true);window.__dbg.select({kind:"joint",id:"wr_L"})')
                page.locator('#model-method').evaluate('(node)=>{node.open=true}')
                settle(page)
                method = ' '.join(page.locator('#model-method').inner_text().split())
                assert 'Предельная сила части внутренних контактов не определена для конкретного человека. Общая сила контакта ограничена допущениями модели.' in method
                assert 'Для части внутренних контактов индивидуальная сила хвата неизвестна.' not in method
                assert 'Площадь вокруг кожных меток задана условно; это не измеренная площадь контакта.' in method
                assert not BREAKABLE_SHORT.search(page.locator('#model-method').text_content())
                assert page.locator('#practice-navigation').is_visible()
                assert page.locator('[data-lightmode="effort"]').is_visible()
                assert page.locator('[data-layer="forces"]').is_visible()
                assert not page.evaluate('document.documentElement.scrollWidth>document.documentElement.clientWidth')
                wrist = page.evaluate('window.__dbg.jointById.wr_L.res.comps.map(c=>({label:c.label,value:c.value,rom:c.rom}))')
                assert any(abs(component['value']) > component['rom'] for component in wrist), wrist
                assert 'Диапазон модели' in ' '.join(page.locator('#info').inner_text().split())
                result = report[name] = {'id': 'centering-06', 'wrist': wrist,
                    'internalForcesN': [math.hypot(*contact['F']) for contact in expected['internalContacts']],
                    'forceArrows': force_arrows(page, expected), 'geometricSupport': geometric_support(page, expected),
                    'errors': errors, 'warnings': warnings}
                page.screenshot(path=str(args.out / f'{name}-centering-06-controls.png'), full_page=True)
                assert not errors and not warnings, result
                context.close()
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('PASS: final methodology, explicit C6 ±F contact and full-page desktop/phone controls', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--browser')
    parser.add_argument('--fetch-assets', action='store_true')
    parser.add_argument('--all-screenshots', action='store_true')
    parser.add_argument('--methodology-only', action='store_true')
    parser.add_argument('--baseline-html', type=Path)
    args = parser.parse_args()
    if args.methodology_only:
        return methodology_check(args)
    assert not args.out.resolve().is_relative_to(ROOT)
    args.out.mkdir(parents=True, exist_ok=True)
    site = args.out / 'site'
    site.mkdir(exist_ok=True)
    for name in ['index.html', 'test.html']:
        shutil.copyfile(ROOT / 'dist' / name, site / name)
    html = (site / 'index.html').read_text()
    assert '__dbg' not in html and '/*__PRACTICE_PHYSICS__*/' not in html
    data = json.loads((ROOT / 'practice/physics.json').read_text())
    manifest = json.loads((ROOT / 'practice/algorithms.json').read_text())
    assert data['posesSha256'] == sha256((ROOT / 'practice/poses.json').read_bytes()).hexdigest()
    ids = [step['id'] for algorithm in manifest['algorithms'] for step in algorithm['steps']]
    assert set(data['poses']) == set(ids) and len(ids) == 24
    assert all(pose['diag']['contactGeometryVerified'] is True for pose in data['poses'].values())
    (site / 'missing.html').write_text(replace_physics(html, None))
    stale = dict(data, posesSha256='0' * 64)
    (site / 'stale.html').write_text(replace_physics(html, stale))
    invalid = json.loads(json.dumps(data))
    invalid['poses']['struna-01']['diag']['quality']['converged'] = False
    (site / 'unverified.html').write_text(replace_physics(html, invalid))
    invalid_geometry = json.loads(json.dumps(data))
    invalid_geometry['poses']['struna-01']['diag']['contactGeometryVerified'] = False
    (site / 'geometry-invalid.html').write_text(replace_physics(html, invalid_geometry))
    mismatch_flags = json.loads(json.dumps(data))
    mismatch_flags['poses']['struna-01']['diag']['quality']['physicalBalanceVerified'] = False
    (site / 'flags-mismatch.html').write_text(replace_physics(html, mismatch_flags))
    report = {'htmlSha256': sha256(html.encode()).hexdigest(), 'posesSha256': data['posesSha256']}
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
                headers = {key: value for key, value in response.headers.items()
                           if key.lower() not in ('content-encoding', 'content-length', 'transfer-encoding')}
                cache[url] = response.status, headers, response.read()
        status, headers, body = cache[url]
        route.fulfill(status=status, headers=headers, body=body)

    try:
        with sync_playwright() as playwright:
            options = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
            if args.browser:
                options['executable_path'] = args.browser
            browser = playwright.chromium.launch(**options)
            for name, width, height in [('desktop', 1300, 900), ('phone', 390, 844)]:
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
                result = report[name] = {'frames': [], 'errors': errors, 'warnings': warnings}
                files = []
                for algorithm in manifest['algorithms']:
                    for index, step in enumerate(algorithm['steps']):
                        page.evaluate('([id,index])=>window.__dbg.enterPractice(id,index)', [algorithm['id'], index])
                        page.evaluate('window.__dbg.setLightMode("effort");window.__dbg.setLayer("forces",false)')
                        settle(page)
                        result['frames'].append(frame(page, data['poses'][step['id']]))
                        assert ' '.join(page.locator('#pose-name').inner_text().split()) == step['name']
                        assert page.locator('#practice-prev').is_disabled() == (index == 0)
                        assert page.locator('#practice-next').is_disabled() == (index == 7)
                        if args.all_screenshots or index in [0, 5, 6]:
                            target = args.out / f'{name}-{step["id"]}.png'
                            page.locator('#view').scroll_into_view_if_needed()
                            page.locator('#stage').screenshot(path=str(target), animations='disabled')
                            files.append((step['id'], target))
                        print(name, step['id'], 'PASS', flush=True)
                if args.all_screenshots:
                    contact_sheet(files, args.out / f'{name}-all24.png')

                # Pick the strongest asymmetry in the independent saved output.
                candidates = [(abs(records['L']['a'] - records['R']['a']), identity, unit)
                              for identity, pose in data['poses'].items() for unit, records in pose['muscles'].items()
                              if known(records['L']) and known(records['R'])]
                difference, identity, unit = max(candidates)
                assert difference > .01
                algorithm_id, ordinal = identity.rsplit('-', 1)
                page.evaluate('([id,index])=>window.__dbg.enterPractice(id,index)', [algorithm_id, int(ordinal) - 1])
                settle(page)
                for side in ['L', 'R']:
                    page.evaluate('([unit,side])=>window.__dbg.select({kind:"muscle",unit,side})', [unit, side])
                    record = data['poses'][identity]['muscles'][unit][side]
                    block = page.locator(f'[data-practice-muscle="{unit}"][data-physics-side="{side}"]')
                    assert abs(float(block.get_attribute('data-force-fraction')) - record['a']) < 1e-12
                    for shown, key in zip(block.locator('.force-metrics dd').all_text_contents(), ['F', 'Fp', 'Fe']):
                        assert abs(number(shown) - record[key]) <= .051, (side, key, shown, record)
                result['independentSideCase'] = {'id': identity, 'unit': unit, 'difference': difference}
                page.evaluate('window.__dbg.select({kind:"muscle",unit:"thenar",side:"R"})')
                assert page.locator('[data-muscle-estimate="unavailable"]').count() == 1
                assert page.locator('#info [data-force-fraction]').count() == 0
                assert 'не определена' in ' '.join(page.locator('#info').inner_text().split())

                right_id = next(joint_id for joint_id, joint in data['poses'][identity]['joints'].items()
                                if joint_id.endswith('_R') and 'jr' in joint)
                page.evaluate('(id)=>window.__dbg.select({kind:"joint",id})', right_id)
                reaction = data['poses'][identity]['joints'][right_id]['jr']['bw']
                assert abs(number(page.locator(f'[data-practice-joint="{right_id}"] .big').inner_text()) - reaction) <= .051
                page.evaluate('window.__dbg.setLayer("forces",true)')
                settle(page)
                result['forceArrows'] = force_arrows(page, data['poses'][identity])
                result['geometricSupport'] = geometric_support(page, data['poses'][identity])
                internal_force, internal_id = max((math.hypot(*contact['F']), pose_id)
                    for pose_id, pose in data['poses'].items() for contact in pose.get('internalContacts', []))
                assert internal_force >= 5, 'The rendered contact-pair branch needs a real loaded internal contact'
                internal_algorithm, internal_ordinal = internal_id.rsplit('-', 1)
                page.evaluate('([id,index])=>window.__dbg.enterPractice(id,index)',
                              [internal_algorithm, int(internal_ordinal) - 1])
                settle(page)
                result['internalContactArrows'] = {'id': internal_id, 'maximumInternalForceN': internal_force,
                    **force_arrows(page, data['poses'][internal_id])}
                page.evaluate('([id,index])=>window.__dbg.enterPractice(id,index)',
                              [algorithm_id, int(ordinal) - 1])
                settle(page)
                page.locator('#model-method').evaluate('(node)=>{node.open=true}')
                assert 'Численный расчёт' in page.locator('#model-quality').inner_text()
                assert not BREAKABLE_SHORT.search(page.locator('#model-method').text_content())

                # The static reconstruction never becomes a standing morph.
                page.evaluate('window.__dbg.setT(.2)')
                settle(page)
                assert page.evaluate('window.__dbg.state.t') == 1
                page.locator('#practice-next').click() if int(ordinal) < 8 else page.locator('#practice-prev').click()
                settle(page)
                assert page.evaluate('window.__dbg.state.lightMode') == 'effort'
                assert page.evaluate('window.__dbg.state.forces')
                page.evaluate('window.__dbg.switchPose("bow");window.__dbg.setT(.37);window.__dbg.setLightMode("length")')
                settle(page)
                assert page.locator('#interpolated-physics-note').is_visible()
                assert 'интерполяция двух оценок' in page.locator('#interpolated-physics-note').inner_text()
                legacy_unit = page.evaluate('''() => Object.entries(window.__dbg.PHYS.bow.muscles)
                  .find(([,m])=>Number.isFinite(m.a)&&(!m.status||m.status==='estimated'))[0]''')
                page.evaluate('(unit)=>window.__dbg.select({kind:"muscle",unit,side:"L"})', legacy_unit)
                assert 'Интерполяция оценок' in ' '.join(page.locator('#info').inner_text().split())
                assert page.evaluate('window.__dbg.forcesGroup.children.every(g=>!g.visible)')
                page.evaluate('window.__dbg.setT(1)')
                settle(page)
                assert page.locator('#interpolated-physics-note').is_hidden()
                assert 'По расчёту' in ' '.join(page.locator('#info').inner_text().split())
                page.evaluate('window.__dbg.setT(.37)')
                settle(page)
                page.evaluate('window.__dbg.enterPractice("detali",6)')
                page.locator('#main-mode').click()
                settle(page)
                assert page.evaluate('window.__dbg.state.poseId') == 'bow'
                assert abs(page.evaluate('window.__dbg.state.t') - .37) < 1e-9
                page.evaluate('window.__dbg.enterCycle(8);window.__dbg.cycPause();window.__dbg.enterPractice("struna",0)')
                page.locator('#main-mode').click()
                settle(page)
                assert page.evaluate('window.__dbg.state.mode') == 'cycle'
                assert abs(page.evaluate('window.__dbg.state.cycT') - 8) < 1e-9
                assert not page.evaluate('window.__dbg.cyc.playing')
                result['restorePoseAndCycle'] = True
                assert not errors and not warnings, (errors, warnings)
                context.close()

            context = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='reduce')
            if args.fetch_assets:
                context.route('https://**/*', fetch_asset)
            page = context.new_page()
            page.set_default_timeout(120000)
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.goto(origin + '/index.html#practice/centering/6', wait_until='networkidle')
            page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]')
            assert page.evaluate('typeof window.__dbg') == 'undefined'
            assert page.locator('[data-lightmode="effort"]').is_visible()
            assert page.locator('#cues [data-practice-quality="verified"]').count() == 1
            assert not page.evaluate('document.documentElement.scrollWidth>document.documentElement.clientWidth')
            page.screenshot(path=str(args.out / 'production-phone.png'), full_page=True)
            report['production'] = {'computed': True, 'debug': False}
            for variant in ['missing', 'stale', 'unverified', 'geometry-invalid', 'flags-mismatch']:
                page.goto(origin + '/' + variant + '.html#practice/struna/1', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]')
                assert page.locator('[data-lightmode="effort"]').is_hidden()
                assert page.locator('[data-layer="forces"]').is_hidden()
                assert 'недоступна' in page.locator('#practice-caption').inner_text()
                assert page.locator('[data-practice-quality="verified"]').count() == 0
            report['production']['unverifiedSnapshotsRejected'] = 5
            assert not errors, errors
            context.close()
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('PASS: computed practice physics on desktop, phone and public HTML', flush=True)


if __name__ == '__main__':
    main()
