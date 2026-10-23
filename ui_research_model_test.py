"""Check research comparisons against real sensitivity data on the built page.

python3 ui_research_model_test.py --out /tmp/fysm-research --browser /usr/bin/chromium --fetch-assets
Verified HTTPS bridging and the shared browser lock match the other UI checks.
This tests presentation/scope and regressions, not independent model accuracy.
"""
import argparse
import base64
import fcntl
from functools import partial
from hashlib import sha256
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import re
from pathlib import Path
import tempfile
from threading import Thread
import urllib.request

from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parent
DOIS = ['https://doi.org/10.1186/s12984-025-01629-5', 'https://doi.org/10.1016/j.jbiomech.2025.113114',
        'https://doi.org/10.1007/s11044-025-10096-8']
SHORT = 'в|во|и|не|ни|до|на|с|со|к|ко|о|об|от|у|по|за|из|а|но|же|для|при|без|над|под|про|или|как|что|это|то|их|его|её|ее|чем|чём|уже'
BREAKABLE_SHORT = re.compile(r'(^|[\s(«„"])(' + SHORT + r')[\t\r\n ]+', re.IGNORECASE)


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
    assert not args.out.resolve().is_relative_to(ROOT)
    data_text = (ROOT / 'research/2025-2026/sensitivity.json').read_text()
    data = json.loads(data_text)
    cycle = json.loads((ROOT / 'cycle_data.json').read_text())
    loads = base64.b64decode(cycle['jload']['v'])
    peak_index = max(range(len(loads)), key=loads.__getitem__)
    peak_frame, peak_joint = divmod(peak_index, len(cycle['jload']['ids']))
    peak_load = loads[peak_index] / cycle['jload'].get('scale', 50)
    load_probe = {'id': cycle['jload']['ids'][peak_joint], 't': peak_frame / cycle['fps'], 'bw': peak_load}
    candidates = [(abs(m['a']['cue']-m['a']['noCue']), pose, unit)
                  for pose, entry in data['poses'].items() if pose != 'stand'
                  for unit, m in entry['muscles'].items()
                  if abs(m['a']['cue']-m['activation']['cue']) > 0.001
                  and m['a']['cue'] >= .08 and data['poses']['stand']['muscles'][unit]['a']['cue'] < .08]
    difference, pose, unit = max(candidates)
    assert difference > 0, 'Need a real case distinguishing scenarios and force fraction from solver activation'
    html = (ROOT / 'dist/index.html').read_bytes()
    assert b'__dbg' not in html
    report = {'case': {'pose': pose, 'unit': unit, 'scenario_difference': difference},
              'data_sha256': sha256(data_text.encode()).hexdigest(), 'html_sha256': sha256(html).hexdigest()}
    lock = (Path(tempfile.gettempdir()) / ('fysm-ui-' + sha256(str(ROOT).encode()).hexdigest()[:12] + '.lock')).open('w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(ROOT / 'dist')))
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
        page.evaluate('window.__dbg.resetView(false)')
        page.evaluate('() => new Promise(r=>requestAnimationFrame(r))')

    def compare(page, key, kind, identity, metric_keys):
        comparison = page.locator('.research-comparison')
        assert comparison.count() == 1
        assert comparison.get_attribute('data-research-pose') == key
        assert comparison.get_attribute('data-research-kind') == kind
        assert comparison.get_attribute('data-research-id') == identity
        expected = data['poses'][key]['muscles' if kind == 'muscle' else 'joints'][identity]
        for metric_key in metric_keys:
            row = comparison.locator(f'[data-research-metric="{metric_key}"]')
            assert row.count() == 1
            metric = expected[metric_key]
            for attr, value in [('cue', metric['cue']), ('no-cue', metric['noCue']), ('low', metric['range'][0]), ('high', metric['range'][1])]:
                assert abs(float(row.get_attribute('data-'+attr))-value) < 1e-12, (identity, metric_key, attr)
        text = comparison.inner_text().lower()
        raw_text = comparison.text_content()
        assert not BREAKABLE_SHORT.search(raw_text), ('Hanging short word in comparison', raw_text)
        assert 'доверительный' in text and 'персональной' in text
        if kind == 'muscle':
            assert comparison.locator('[data-research-metric="activation"]').count() == 0
            assert 'эмг' in text and 'максимальной' in text
        return comparison.inner_text()

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
                page.set_default_timeout(90000)
                errors, warnings = [], []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.on('console', lambda message: warnings.append(message.text) if message.type in ('warning', 'error') else None)
                page.goto(f'http://127.0.0.1:{server.server_port}/test.html', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"]', timeout=120000)
                result = {'errors': errors, 'warnings': warnings}
                page.locator(f'[data-pose="{pose}"]').click()
                page.evaluate('(unit)=>window.__dbg.select({kind:"muscle",unit,side:"L"})', unit)
                settle(page)
                result['muscle_comparison'] = compare(page, pose, 'muscle', unit, ['a', 'forceN'])
                assert page.locator('#info .tags .tag-active').count() == 1
                # Even a tiny departure from the endpoint must remove stale ranges.
                for t in [.9999, .5, .0001]:
                    page.evaluate('(t)=>window.__dbg.setT(t)', t)
                    settle(page)
                    assert page.locator('.research-comparison').count() == 0, t
                page.evaluate('window.__dbg.setT(0)')
                settle(page)
                result['stand_comparison'] = compare(page, 'stand', 'muscle', unit, ['a', 'forceN'])
                assert page.locator('#info .tags .tag-active').count() == 0, 'Full-pose work tag leaked into standing'
                page.evaluate('window.__dbg.setT(1)')
                settle(page)
                compare(page, pose, 'muscle', unit, ['a', 'forceN'])
                green = page.evaluate('''() => {const d=window.__dbg,ms=d.PHYS[d.state.poseId].muscles;
                  return d.strands.filter(s=>s.inst.kind==='muscle').every(s=>s.on===(ms[s.inst.unit].a>=.08));}''')
                assert green, 'Working green must use force fraction a, not the different solver activation'
                result['green_force_fraction'] = green
                for layer in ['musclesOuter', 'musclesDeep']:
                    page.evaluate('(layer)=>window.__dbg.setLayer(layer,false)', layer)
                assert page.evaluate('window.__dbg.muscleSurfaces.pickables.every(m=>!m.visible)')
                for layer in ['musclesOuter', 'musclesDeep']:
                    page.evaluate('(layer)=>window.__dbg.setLayer(layer,true)', layer)
                assert page.evaluate('window.__dbg.muscleSurfaces.pickables.every(m=>m.visible)')
                # Turning off the selected muscle's layer correctly clears its
                # selection. Restore the card before checking/capturing it.
                page.evaluate('(unit)=>window.__dbg.select({kind:"muscle",unit,side:"L"})', unit)
                settle(page)
                compare(page, pose, 'muscle', unit, ['a', 'forceN'])
                page.locator('.research-comparison').screenshot(path=str(args.out / f'{name}-muscle.png'), animations='disabled')
                page.evaluate('window.__dbg.select({kind:"joint",id:"ls"})')
                settle(page)
                result['joint_comparison'] = compare(page, pose, 'joint', 'ls', ['compressionN', 'shearN'])
                page.locator('.research-comparison').screenshot(path=str(args.out / f'{name}-spine.png'), animations='disabled')
                page.evaluate('window.__dbg.select(null)')
                settle(page)
                assert page.locator('.research-comparison[data-research-id="ls"]').count() == 1
                page.evaluate('window.__dbg.setT(.5)')
                settle(page)
                assert page.locator('.research-comparison').count() == 0, 'Summary kept endpoint range'
                page.evaluate('window.__dbg.enterCycle();window.__dbg.cycPause();window.__dbg.setCycT(8);window.__dbg.select({kind:"muscle",unit:"rectAbd",side:"L"})')
                settle(page)
                assert page.locator('.research-comparison').count() == 0
                assert page.evaluate('window.__dbg.state.mode') == 'cycle'
                # The details are still closed. text_content checks their updated
                # content; inner_text may correctly omit the collapsed subtree.
                assert 'не рассчитывался' in ' '.join(page.locator('#model-quality').text_content().split())
                page.evaluate('window.__dbg.select({kind:"joint",id:"ls"})')
                settle(page)
                assert page.locator('.research-comparison').count() == 0
                page.evaluate('({id,t})=>{window.__dbg.setCycT(t);window.__dbg.select({kind:"joint",id});}', load_probe)
                settle(page)
                shown_load = page.locator('#cyc-live-load').inner_text()
                assert abs(float(shown_load.replace('×', '').replace(',', '.'))-peak_load) <= .051, (load_probe, shown_load)
                result['cycle_peak_load'] = {'expected': load_probe, 'shown': shown_load}
                page.evaluate('(pose)=>window.__dbg.switchPose(pose)', pose)
                settle(page)
                page.locator('#model-method summary').click()
                for doi in DOIS:
                    assert page.locator(f'#model-method a[href="{doi}"]').count() == 1
                assert page.locator('#model-quality').inner_text()
                result['quality'] = page.locator('#model-quality').inner_text()
                page.locator('#model-method').screenshot(path=str(args.out / f'{name}-sources.png'), animations='disabled')
                assert page.evaluate('document.documentElement.scrollWidth<=document.documentElement.clientWidth')
                assert page.evaluate("document.getElementById('view').getContext('webgl2').getError()") == 0
                assert not errors and not warnings, (errors, warnings)
                report[name] = result
                context.close()
                print(name, 'PASS: data, endpoint scope, circle, green, layers, sources', flush=True)

            context = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='reduce')
            if args.fetch_assets:
                context.route('https://**/*', fetch_asset)
            page = context.new_page()
            page.goto(f'http://127.0.0.1:{server.server_port}/index.html', wait_until='networkidle')
            page.wait_for_selector('#stage[data-muscle-surfaces="ready"]', timeout=120000)
            report['production'] = page.evaluate('''()=>({debug:typeof window.__dbg,charset:document.characterSet,
              overflow:document.documentElement.scrollWidth>document.documentElement.clientWidth})''')
            assert report['production'] == {'debug': 'undefined', 'charset': 'UTF-8', 'overflow': False}
            assert page.locator('.research-comparison[data-research-pose="dog"]').count() == 1
            for doi in DOIS:
                assert page.locator(f'#model-method a[href="{doi}"]').count() == 1
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('PASS: research UI on desktop/phone and production', flush=True)


if __name__ == '__main__':
    main()
