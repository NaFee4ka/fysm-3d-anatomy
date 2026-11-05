"""Exercise prepared transitions on the real page at desktop and phone widths.

Snapshots the final HTML; checks playback, geometry/force separation, endpoint
restoration, routes, reduced motion, keyboard, history and mobile layout.
This is UI verification, not physiological or continuous collision validation.
"""
import argparse
import fcntl
from functools import partial
from hashlib import sha256
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import re
import shutil
import tempfile
from threading import Thread
import urllib.request

from PIL import Image, ImageDraw
from playwright.sync_api import sync_playwright
from ui_practice_test import QuietHandler, BREAKABLE_SHORT, settle

ROOT = Path(__file__).resolve().parent
EXTRA_BREAKABLE = re.compile(r'(^|[\s(«„"])(через|перед|после|между)[\t\r\n ]+', re.IGNORECASE)


def snapshot(page):
    return page.evaluate('''() => {
      const d=window.__dbg,t=d.transition,c=d.TRANSITIONS.sample(t.route,t.time),v=d.displayConf;
      let minSpineY=Infinity;
      for(const part of d.spineSurfaces?.parts||[]){const a=part.mesh.geometry.attributes.position.array,e=part.mesh.matrixWorld.elements;
        for(let i=0;i<a.length;i+=3)minSpineY=Math.min(minSpineY,e[1]*a[i]+e[5]*a[i+1]+e[9]*a[i+2]+e[13]);}
      return {mode:d.state.mode,from:t.from,to:t.to,time:t.time,duration:t.route.duration,
        playing:t.playing,rootError:v.rootP.distanceTo(c.rootP),
        rotationError:Math.max(...d.A.SEG.map(s=>1-Math.abs(v.local[s.name].dot(c.local[s.name])))),
        green:d.strands.filter(s=>s.inst.kind==='muscle'&&s.on).length,
        visibleForceSets:d.forcesGroup.children.filter(g=>g.visible).length,
        overflow:document.documentElement.scrollWidth>document.documentElement.clientWidth,
        lightMode:d.state.lightMode,staticPhysics:!!d.practicePhysics(),spineParts:d.spineSurfaces?.parts.length,minSpineY,
        error:document.getElementById('view').getContext('webgl2').getError()};
    }''')


def check_frame(page):
    settle(page)
    r = snapshot(page)
    assert r['mode'] == 'transition', r
    assert r['rootError'] < 1e-10 and abs(r['rotationError']) < 1e-8, r
    assert r['green'] == r['visibleForceSets'] == r['error'] == 0, r
    assert not r['overflow'] and not r['staticPhysics'], r
    assert r['lightMode'] == 'length' and r['spineParts'] == 23, r
    assert r['minSpineY'] >= -1e-5, r
    assert page.locator('#pose-player').is_hidden() and page.locator('#cycle-player').is_hidden()
    assert page.locator('#transition-player').is_visible()
    assert page.locator('[data-lightmode="effort"]').is_hidden()
    assert page.locator('[data-layer="forces"]').is_hidden()
    assert page.locator('.research-comparison,.tag-active,[data-practice-muscle],[data-practice-joint]').count() == 0
    for selector in ['#pose-name', '#transition-navigation', '#transition-player', '#cues', '#info']:
        text = page.locator(selector).text_content()
        assert not BREAKABLE_SHORT.search(text) and not EXTRA_BREAKABLE.search(text), selector
    return r


def static_card_snapshot(page):
    """Compare the visible retained card with fresh geometry, not a mode label."""
    return page.evaluate('''() => {
      const d=window.__dbg,c=d.displayConf,s=d.state.sel;
      const W=c.rootP?d.A.computeWorld(c.local,c.rootQ,c.rootP):d.A.placeW(c.local,c.rootQ);
      const worldError=Math.max(...d.A.SEG.map(part=>W[part.name].p.distanceTo(d.W[part.name].p)));
      if(s.kind==='joint'){
        const joint=d.jointById[s.id],fresh=d.A.analyzeJoint(joint.def,c.local,W,d.STATS[d.state.poseId].joint[s.id].mode);
        return {mode:d.state.mode,pose:d.state.poseId,selection:s,worldError,
          cacheError:Math.max(...fresh.comps.map((comp,i)=>Math.abs(comp.value-joint.res.comps[i].value))),
          expected:fresh.comps.map(comp=>Math.round(Math.abs(comp.value))+'°'),
          shown:[...document.querySelector('#info .parts.comps').children].map(row=>row.querySelector('.num').textContent.trim())};
      }
      const strands=d.strands.filter(part=>part.inst.kind==='muscle'&&part.inst.unit===s.unit);
      const strains=strands.map(part=>d.A.pathLength(d.A.strandPath(W,part.inst))/part.L0-1);
      const strain=strains.reduce((a,b)=>a+b,0)/strains.length,v=Math.round(strain*100);
      return {mode:d.state.mode,pose:d.state.poseId,selection:s,worldError,
        cacheError:Math.max(...strains.map((value,i)=>Math.abs(value-strands[i].strain))),
        expected:[(v>0?'+':v<0?'−':'±')+Math.abs(v)+'%'],
        shown:[document.querySelector('#info .meter .num').textContent.trim()]};
    }''')


def check_static_card(page, pose):
    settle(page)
    result = static_card_snapshot(page)
    assert result['mode'] == 'pose' and result['pose'] == pose, result
    assert result['worldError'] < 1e-10 and result['cacheError'] < 1e-9, result
    assert result['shown'] == result['expected'], result
    return result



def endpoint_sheet(files, path):
    cell_w, cell_h = 300, 310
    canvas = Image.new('RGB', (cell_w * 5, cell_h * 6), '#0d1215')
    draw = ImageDraw.Draw(canvas)
    for i, (label, filename) in enumerate(files):
        image = Image.open(filename).convert('RGB')
        image.thumbnail((cell_w, cell_h - 28))
        x, y = (i % 5) * cell_w, (i // 5) * cell_h
        canvas.paste(image, (x + (cell_w - image.width) // 2, y + 28))
        draw.text((x + 10, y + 7), label, fill='#ece4d3')
    canvas.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--browser', default='/usr/bin/chromium')
    parser.add_argument('--fetch-assets', action='store_true')
    args = parser.parse_args()
    assert not args.out.resolve().is_relative_to(ROOT)
    args.out.mkdir(parents=True, exist_ok=True)
    site = args.out / 'site'
    site.mkdir(exist_ok=True)
    for name in ['index.html', 'test.html']:
        shutil.copyfile(ROOT / 'dist' / name, site / name)
    public = (site / 'index.html').read_bytes()
    assert b'__dbg' not in public
    inputs = ['app3.js', 'template3.html', 'build.py', 'ui_transitions_test.py', 'transition_runtime.mjs', 'transitions/data.json',
              'spine_surface_render.js', 'joints/spine-discs.json', 'fascia_render.js']
    input_hashes = {name: sha256((ROOT / name).read_bytes()).hexdigest() for name in inputs}
    report = {'htmlSha256': sha256(public).hexdigest(), 'testHtmlSha256': sha256((site / 'test.html').read_bytes()).hexdigest(),
              'inputSha256': input_hashes}
    # Corrupt only the atlas-disk literal in an owned, isolated negative fixture.
    # The production snapshot and all other anatomical packets stay unchanged.
    fixture = (site / 'test.html').read_text()
    marker = 'const SPINE_SURFACE_DATA = '
    offset = fixture.index(marker) + len(marker)
    disk_packet, length = json.JSONDecoder().raw_decode(fixture[offset:])
    disk_packet['data'] = 'AAAA'
    (site / 'spine-invalid.html').write_text(fixture[:offset] + json.dumps(disk_packet, ensure_ascii=False) + fixture[offset + length:])
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
        status, headers, data = cache[url]
        route.fulfill(status=status, headers=headers, body=data)

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(executable_path=args.browser,
                args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
            for name, width, height in [('desktop', 1300, 900), ('phone', 390, 844)]:
                ctx = browser.new_context(viewport={'width': width, 'height': height}, reduced_motion='no-preference')
                if args.fetch_assets:
                    ctx.route('https://**/*', fetch_asset)
                page = ctx.new_page()
                page.set_default_timeout(120000)
                errors, warnings = [], []
                page.on('pageerror', lambda e: errors.append(str(e)))
                page.on('console', lambda m: warnings.append(m.text) if m.type in ['error', 'warning'] else None)
                page.goto(origin + '/test.html', wait_until='networkidle')
                page.wait_for_selector('#stage[data-muscle-surfaces="ready"][data-joint-surfaces="ready"][data-spine-surfaces="ready"]')
                page.wait_for_function('window.__dbg?.TRANSITIONS?.valid === true')
                catalog = page.evaluate('window.__dbg.TRANSITIONS.poses')
                assert len(catalog) == 30 and len({c['id'] for c in catalog}) == 30
                result = {'frames': [], 'errors': errors, 'warnings': warnings, 'catalog': [c['id'] for c in catalog]}
                report[name] = result
                page.locator('#transitions-mode').click()
                assert page.locator('#transition-from option').count() == 30
                assert page.locator('#transition-to option').count() == 30
                assert not snapshot(page)['playing'], 'Entering transitions must remain paused'
                # Ordinary tabs select endpoints instantly. They must not start
                # the former unchecked pose-to-pose/circle-entry morphs.
                page.locator('#main-mode').click()
                result['instantNavigation'] = []
                for pose in ['bow', 'handstand', 'dog']:
                    page.locator(f'[data-pose="{pose}"]').click()
                    case = page.evaluate('({mode:window.__dbg.state.mode,id:window.__dbg.state.poseId,morph:!!window.__dbg.morph,playing:!!window.__dbg.state.playing})')
                    assert case == {'mode': 'pose', 'id': pose, 'morph': False, 'playing': False}, case
                    result['instantNavigation'].append(case)
                # A second consecutive instant switch used to leave retained
                # cards measured in the previous pose, even after the next RAF.
                result['selectedStaticCards'] = []
                for selection in [{'kind': 'joint', 'id': 'hip_L'}, {'kind': 'muscle', 'unit': 'psoas', 'side': 'L'}]:
                    page.locator('[data-pose="dog"]').click()
                    settle(page)
                    page.evaluate('(selection)=>window.__dbg.select(selection)', selection)
                    cases = []
                    for pose in ['bow', 'fold', 'handstand']:
                        page.locator(f'[data-pose="{pose}"]').click()
                        cases.append(check_static_card(page, pose))
                    assert len({tuple(case['expected']) for case in cases}) > 1, cases
                    result['selectedStaticCards'].extend(cases)
                # Opening an original static endpoint must measure the retained
                # selection after leaving its adjusted animation configuration.
                result['transitionEndpointCards'] = []
                for pose, selection in [('dog', {'kind': 'joint', 'id': 'hip_L'}),
                                        ('fold', {'kind': 'muscle', 'unit': 'psoas', 'side': 'L'})]:
                    page.evaluate('(pose)=>{const d=window.__dbg;d.enterTransition("bow",pose);d.setTransitionTime(d.transition.route.duration)}', pose)
                    page.evaluate('(selection)=>window.__dbg.select(selection)', selection)
                    page.locator('#transition-open-pose').click()
                    result['transitionEndpointCards'].append(check_static_card(page, pose))
                page.locator('[data-pose="dog"]').click()
                page.evaluate('window.__dbg.select(null)')
                page.evaluate('window.__dbg.setT(.4)')
                settle(page)
                assert page.locator('#interpolated-physics-note').is_visible()
                page.evaluate('window.__dbg.setT(1);window.__dbg.setLayer("forces",true)')
                page.locator('#cycle-tab').click()
                page.evaluate('window.__dbg.cycPause()')
                settle(page)
                assert page.evaluate('window.__dbg.state.mode==="cycle"&&window.__dbg.forcesGroup.children.filter(g=>g.visible).length===1')
                page.locator('#transitions-mode').click()
                check_frame(page)
                # Every endpoint can be selected in either direction without a missing route.
                availability = page.evaluate('''() => {const d=window.__dbg,ids=d.TRANSITIONS.poses.map(p=>p.id);let count=0;
                  for(const a of ids)for(const b of ids)if(a!==b){const r=d.TRANSITIONS.route(a,b);if(!(r.duration>0))throw Error(a+'/'+b);count++;}return count;}''')
                assert availability == 870, availability
                result['orderedRoutes'] = availability
                endpoint_files = []
                result['endpoints'] = []
                for i, endpoint in enumerate(catalog):
                    other = catalog[(i + 1) % len(catalog)]['id']
                    page.evaluate('([a,b])=>window.__dbg.enterTransition(a,b)', [endpoint['id'], other])
                    frame = check_frame(page)
                    result['endpoints'].append(frame)
                    notes = page.evaluate('''(id)=>{const out=[],walk=x=>{if(!x||typeof x!=="object")return;if(typeof x.note==="string"&&x.note.trim())out.push(x.note.trim());if(Array.isArray(x.adjustments))x.adjustments.forEach(walk);};walk(window.__dbg.TRANSITION_DATA.endpoints[id].kinematicAdjustment);return [...new Set(out)];}''', endpoint['id'])
                    assert page.locator('#transition-adjustments').is_visible() == bool(notes)
                    if notes:
                        shown = ' '.join(page.locator('#transition-adjustments').inner_text().split())
                        assert all(' '.join(note.split()) in shown for note in notes), (notes, shown)
                    page.locator('#view').scroll_into_view_if_needed()
                    target = args.out / f"{name}-endpoint-{endpoint['id']}.png"
                    page.locator('#stage').screenshot(path=str(target), animations='disabled')
                    endpoint_files.append((endpoint['id'], target))
                endpoint_sheet(endpoint_files, args.out / f'{name}-all30-endpoints.jpg')
                pairs = [('dog', 'plank'), ('half', 'bow'), ('fold', 'handstand'),
                         ('struna-01', 'struna-02'), ('detali-02', 'detali-03'),
                         ('centering-05', 'centering-06'), ('struna-08', 'centering-08')]
                for a, b in pairs:
                    page.locator('#transition-from').select_option(a)
                    page.locator('#transition-to').select_option(b)
                    page.locator('#transition-build').click()
                    assert page.evaluate('window.__dbg.state.mode') == 'transition'
                    for u in [0, .25, .5, .75, 1]:
                        page.evaluate('(u)=>window.__dbg.setTransitionTime(u*window.__dbg.transition.route.duration)', u)
                        r = check_frame(page)
                        result['frames'].append(r)
                        if u in [.5, 1]:
                            page.locator('#view').scroll_into_view_if_needed()
                            page.locator('#stage').screenshot(path=str(args.out / f'{name}-{a}-{b}-{u}.png'), animations='disabled')
                # Force and effort controls/debug calls cannot attach a foreign static solution.
                page.evaluate('window.__dbg.setLayer("forces",true);window.__dbg.setLightMode("effort")')
                check_frame(page)
                page.evaluate('window.__dbg.select({kind:"muscle",unit:"rectAbd",side:"R"})')
                check_frame(page)
                assert page.locator('#info [data-muscle-estimate="unavailable"]').count() == 1
                assert 'В переходе оценки сил не рассчитаны.' in ' '.join(page.locator('#info').inner_text().split())
                result['uncomputedMuscleCard'] = page.locator('#info').inner_text()
                page.locator('[data-practice-side="L"]').click()
                assert page.evaluate('window.__dbg.state.sel.side') == 'L'
                page.evaluate('window.__dbg.select({kind:"joint",id:"hip_R"})')
                check_frame(page)
                assert page.locator('#info .parts.comps').count() == 1
                page.evaluate('window.__dbg.select(null)')
                disc = page.evaluate('window.__dbg.spineSurfaces.parts[0].id')
                page.evaluate('(id)=>window.__dbg.select({kind:"spinalDisc",id})', disc)
                assert 'BodyParts3D' in page.locator('#info').inner_text()
                assert 'Давление' in page.locator('#info').inner_text()
                assert page.locator('#info [data-practice-joint],#info [data-static-physics]').count() == 0
                page.evaluate('window.__dbg.setLayer("joints",false)')
                assert page.evaluate('window.__dbg.spineSurfaces.parts.every(p=>!p.mesh.visible)&&window.__dbg.state.sel===null')
                page.evaluate('window.__dbg.setLayer("joints",true)')
                assert page.evaluate('window.__dbg.spineSurfaces.parts.every(p=>p.mesh.visible)')
                # A real pointer tap must reach an atlas disk, not an unrelated
                # coarse force-joint hit target. Isolate the visible joint layer.
                for layer in ['musclesOuter', 'musclesDeep', 'bones', 'ligs']:
                    page.evaluate('(k)=>window.__dbg.setLayer(k,false)', layer)
                saved_route = page.evaluate('({from:window.__dbg.transition.from,to:window.__dbg.transition.to,time:window.__dbg.transition.time})')
                page.evaluate('window.__dbg.enterTransition("handstand","dog")')
                settle(page)
                result['spinalDiskPointerPicks'] = []
                for target_disc in ['disc-C2-C3', 'disc-C4-C5', 'disc-T12-L1', 'disc-L5-S1']:
                    page.evaluate('(id)=>window.__dbg.select({kind:"spinalDisc",id})', target_disc)
                    page.locator('#info .card-focus').click()
                    page.wait_for_function('!window.__dbg.state.camAnim')
                    picked = None
                    for direction in [None, [1, .2, .4], [.1, .2, -1]]:
                        if direction:
                            page.evaluate('''v=>{const d=window.__dbg,r=d.camera.position.distanceTo(d.controls.target);
                              d.camera.position.copy(d.controls.target).addScaledVector(new d.THREE.Vector3(...v).normalize(),r);d.controls.update();d.state.userMoved=true;}''', direction)
                        candidates = page.evaluate('''(id)=>{const d=window.__dbg,part=d.spineSurfaces.parts.find(p=>p.id===id),g=part.mesh.geometry.attributes.position,
                          r=document.getElementById('view').getBoundingClientRect(),out=[];
                          for(let i=0;i<g.count;i+=Math.max(1,Math.floor(g.count/40))){const q=part.mesh.localToWorld(new d.THREE.Vector3().fromBufferAttribute(g,i)).project(d.camera);
                            const x=r.left+(q.x+1)/2*r.width,y=r.top+(1-q.y)/2*r.height;
                            if(q.z>-1&&q.z<1&&x>r.left&&x<r.right&&y>r.top&&y<r.bottom)out.push({x,y});}return out;}''', target_disc)
                        for point in candidates:
                            page.mouse.click(point['x'], point['y'])
                            picked = page.evaluate('window.__dbg.state.sel')
                            if picked and picked.get('kind') == 'spinalDisc' and picked.get('id') == target_disc:
                                break
                        if picked and picked.get('kind') == 'spinalDisc' and picked.get('id') == target_disc:
                            break
                    assert picked and picked.get('kind') == 'spinalDisc' and picked.get('id') == target_disc, ('Atlas disk tap failed', target_disc, picked)
                    result['spinalDiskPointerPicks'].append(picked)
                page.evaluate('r=>{window.__dbg.enterTransition(r.from,r.to);window.__dbg.setTransitionTime(r.time)}', saved_route)
                for layer in ['musclesOuter', 'musclesDeep', 'bones', 'ligs']:
                    page.evaluate('(k)=>window.__dbg.setLayer(k,true)', layer)
                page.evaluate('window.__dbg.select(null)')
                settle(page)
                # Seek and reversal remain exact and paused; replay and speed use the same prepared route.
                page.locator('#transition-swap').click()
                assert snapshot(page)['from'] == 'centering-08' and snapshot(page)['to'] == 'struna-08'
                page.locator('[data-transition-speed="2"]').click()
                page.locator('#transition-play').click()
                page.wait_for_function('window.__dbg.transition.playing && window.__dbg.transition.time>0')
                page.locator('#transition-play').click()
                t = page.evaluate('window.__dbg.transition.time')
                page.wait_for_timeout(200)
                assert page.evaluate('window.__dbg.transition.time') == t
                page.locator('#transition-restart').click()
                assert snapshot(page)['time'] == 0
                # Keyboard only acts outside native controls.
                page.evaluate('document.activeElement.blur();document.body.tabIndex=-1;document.body.focus()')
                page.keyboard.press('End')
                assert abs(snapshot(page)['time'] - snapshot(page)['duration']) < 1e-8
                page.keyboard.press('Home')
                assert snapshot(page)['time'] == 0
                page.keyboard.press('ArrowRight')
                assert abs(snapshot(page)['time'] - .5) < 1e-8
                page.locator('#transition-to').focus()
                before = page.evaluate('window.__dbg.transition.time')
                page.keyboard.press('ArrowRight')
                assert page.evaluate('window.__dbg.transition.time') == before
                # Same endpoint is a visible selection error, not a NaN route.
                page.locator('#transition-from').select_option('dog')
                page.locator('#transition-to').select_option('dog')
                page.locator('#transition-build').click()
                assert page.locator('#transition-error').is_visible()
                # The source static pose owns its unchanged packet and force controls.
                page.evaluate('window.__dbg.enterTransition("dog","struna-01");window.__dbg.setTransitionTime(window.__dbg.transition.route.duration)')
                page.locator('#transition-open-pose').click()
                settle(page)
                assert page.evaluate('window.__dbg.state.mode==="practice" && !!window.__dbg.practicePhysics()')
                page.evaluate('window.__dbg.setLayer("forces",true)')
                assert page.evaluate('window.__dbg.forcesGroup.children.filter(g=>g.visible).length') == 1
                page.locator('#practice-transition-next').click()
                assert snapshot(page)['from'] == 'struna-01' and snapshot(page)['to'] == 'struna-02'
                assert not snapshot(page)['playing']
                assert snapshot(page)['time'] == 0
                # At time zero the button opens the displayed source endpoint.
                # Seek to the destination before expecting its original pose.
                page.locator('#transition-open-pose').click()
                source_endpoint = page.evaluate('({mode:window.__dbg.state.mode,algorithm:window.__dbg.state.algorithmId,step:window.__dbg.state.practiceStep,physics:!!window.__dbg.practicePhysics()})')
                assert source_endpoint == {'mode': 'practice', 'algorithm': 'struna', 'step': 0, 'physics': True}, source_endpoint
                page.locator('#practice-transition-next').click()
                assert snapshot(page)['from'] == 'struna-01' and snapshot(page)['to'] == 'struna-02'
                page.evaluate('window.__dbg.setTransitionTime(window.__dbg.transition.route.duration)')
                check_frame(page)
                page.locator('#transition-open-pose').click()
                assert page.evaluate('window.__dbg.state.practiceStep') == 1
                destination_endpoint = page.evaluate('({mode:window.__dbg.state.mode,algorithm:window.__dbg.state.algorithmId,step:window.__dbg.state.practiceStep,physics:!!window.__dbg.practicePhysics()})')
                assert destination_endpoint == {'mode': 'practice', 'algorithm': 'struna', 'step': 1, 'physics': True}, destination_endpoint
                result['practiceTransitionEndpointRestoration'] = [source_endpoint, destination_endpoint]
                page.locator('#practice-flow').click()
                assert page.evaluate('window.__dbg.transition.sequence==="struna" && window.__dbg.transition.playing')
                page.evaluate('window.__dbg.transitionPause();window.__dbg.setTransitionTime(window.__dbg.transition.route.duration)')
                settle(page)
                # Joining the next route in an algorithm must preserve manual
                # camera rotation, focus and selection at the shared endpoint.
                continuity = page.evaluate('''() => {
                  const d=window.__dbg,t=d.transition,to=d.TRANSITIONS.poses.find(p=>p.id===t.to),next=d.TRANSITIONS.poses.find(p=>p.algorithmId===to.algorithmId&&p.stepIndex===to.stepIndex+1);
                  d.state.userMoved=true;d.camera.position.add(new d.THREE.Vector3(.13,.07,-.08));d.controls.update();d.select({kind:'muscle',unit:'rectAbd',side:'R'});
                  const p=d.camera.position.clone(),c=d.controls.target.clone(),v=d.viewDir.clone(),h=history.length;
                  const ok=d.enterTransition(t.to,next.id,{play:true,sequence:t.sequence,writeHash:false});
                  const result={ok,positionError:p.distanceTo(d.camera.position),targetError:c.distanceTo(d.controls.target),viewError:v.distanceTo(d.viewDir),userMoved:d.state.userMoved,
                    historyDelta:history.length-h,selection:d.state.sel};d.transitionPause();return result;
                }''')
                assert continuity['ok'] and continuity['userMoved'], continuity
                assert continuity['positionError'] == continuity['targetError'] == continuity['viewError'] == continuity['historyDelta'] == 0, continuity
                assert continuity['selection']['kind'] == 'muscle' and continuity['selection']['side'] == 'R', continuity
                result['automaticBoundaryContinuity'] = continuity
                # Existing circle frames keep their own solved arrows; entering
                # a geometric transition removes them and all force comparisons.
                phase_times = page.evaluate('window.__dbg.CYC.segs.map(s=>(s.t0+s.t1)/2)')
                result['legacyCircleFrames'] = []
                for t in phase_times:
                    page.evaluate('(t)=>{const d=window.__dbg;d.enterCycle(t,false);d.cycPause();d.setCycT(t);d.setLayer("forces",true)}', t)
                    settle(page)
                    case = page.evaluate('({mode:window.__dbg.state.mode,t:window.__dbg.state.cycT,arrows:window.__dbg.forcesGroup.children.filter(g=>g.visible).length})')
                    assert case['mode'] == 'cycle' and abs(case['t'] - t) < 1e-8 and case['arrows'] == 1, case
                    result['legacyCircleFrames'].append(case)
                    page.evaluate('window.__dbg.enterTransition("dog","plank")')
                    check_frame(page)
                page.evaluate('window.__dbg.openTransitionEndpoint()')
                settle(page)
                assert page.evaluate('window.__dbg.state.mode==="pose"&&window.__dbg.state.poseId==="dog"')
                known = page.evaluate('Object.entries(window.__dbg.PHYS.dog.muscles).find(([u,m])=>Number.isFinite(m.a)&&(!m.status||m.status==="estimated"))[0]')
                page.evaluate('(u)=>{const d=window.__dbg;d.select({kind:"muscle",unit:u});d.setLayer("forces",true)}', known)
                settle(page)
                assert page.locator('.research-comparison[data-research-pose="dog"]').count() == 1
                assert page.evaluate('window.__dbg.forcesGroup.children.filter(g=>g.visible).length') == 1
                page.evaluate('window.__dbg.enterTransition("dog","struna-01")')
                check_frame(page)
                # Deep links and history never autoplay or push history on every animation tick.
                page.evaluate('window.__dbg.enterTransition("fold","handstand")')
                history_before = page.evaluate('history.length')
                page.evaluate('window.__dbg.setTransitionTime(window.__dbg.transition.route.duration*.5)')
                assert page.evaluate('history.length') == history_before
                page.evaluate('location.hash="practice/detali/3"')
                page.wait_for_function('window.__dbg.state.mode==="practice"&&window.__dbg.state.practiceStep===2')
                page.go_back(wait_until='networkidle')
                page.wait_for_function('window.__dbg.state.mode==="transition"')
                assert not snapshot(page)['playing']
                check_frame(page)
                page.screenshot(path=str(args.out / f'{name}-controls.png'), full_page=True, animations='disabled')
                assert not errors and not warnings, (errors, warnings)
                ctx.close()
                print(name, 'PASS', flush=True)
            # Reduced motion: entry/algorithm flow stays paused, manual playback is explicit.
            ctx = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='reduce')
            if args.fetch_assets:
                ctx.route('https://**/*', fetch_asset)
            page = ctx.new_page(); page.set_default_timeout(120000)
            page.goto(origin + '/test.html#transition/fold/handstand', wait_until='networkidle')
            page.wait_for_function('window.__dbg?.state.mode==="transition"')
            assert not snapshot(page)['playing']
            assert page.locator('[data-transition-speed="1"]').is_visible()
            assert page.locator('#transition-play').is_visible()
            page.locator('#transition-open-pose').click()
            assert page.evaluate('window.__dbg.state.mode==="pose"&&window.__dbg.state.poseId==="fold"')
            page.evaluate('window.__dbg.enterCycle(0,false);window.__dbg.cycPause()')
            assert page.locator('#cyc-play').is_hidden() and page.locator('#cycle-player .seg-inline').is_hidden()
            page.evaluate('window.__dbg.enterPractice("centering",0)')
            page.locator('#practice-flow').click()
            assert not snapshot(page)['playing']
            result = check_frame(page)
            report['reducedMotion'] = result
            ctx.close()
            # Fallback preserves the 23 valid legacy diagrams atomically;
            # the anatomically nonexistent atlas-axis disk never returns.
            report['spineFallbacks'] = []
            for failure in ['invalid-payload', 'no-decompressor']:
                ctx = browser.new_context(viewport={'width': 390, 'height': 844}, reduced_motion='reduce')
                if args.fetch_assets:
                    ctx.route('https://**/*', fetch_asset)
                if failure == 'no-decompressor':
                    ctx.add_init_script('Object.defineProperty(globalThis,"DecompressionStream",{value:undefined,configurable:true})')
                page = ctx.new_page(); page.set_default_timeout(120000)
                fallback_errors, fallback_warnings = [], []
                page.on('pageerror', lambda e: fallback_errors.append(str(e)))
                page.on('console', lambda m: fallback_warnings.append(m.text) if m.type in ['error', 'warning'] else None)
                page.goto(origin + ('/spine-invalid.html' if failure == 'invalid-payload' else '/test.html'), wait_until='networkidle')
                page.wait_for_selector('#stage[data-spine-surfaces="fallback"]')
                result = page.evaluate('''() => {const d=window.__dbg;
                  const key=m=>m.userData.seg+'|'+m.userData.name;
                  const shown=d.discMeshes.map(key).sort();
                  const expected=[...new Set(d.A.BONES.filter(b=>b.mat==='disc'&&!(b.seg==='neckB'&&b.name==='Межпозвонковый диск C2–C1')).map(b=>b.seg+'|'+b.name))].sort();
                  return {loaded:!!d.spineSurfaces,shown,expected,ready:document.getElementById('stage').classList.contains('ready'),
                    wrongDisk:d.discMeshes.some(m=>m.userData.name==='Межпозвонковый диск C2–C1')};}''')
                assert result['ready'] and not result['loaded'] and not result['wrongDisk'], result
                assert result['shown'] == result['expected'] and len(result['shown']) == 23, result
                assert 'недоступны' in page.locator('#spine-source-note').inner_text()
                assert not fallback_errors, fallback_errors
                assert any('Spinal atlas disks could not load' in w for w in fallback_warnings), fallback_warnings
                report['spineFallbacks'].append({'case': failure, **result, 'errors': fallback_errors, 'expectedWarnings': fallback_warnings})
                ctx.close()
            # Production page has no debug hook and exposes the same mode.
            ctx = browser.new_context(viewport={'width': 390, 'height': 844})
            if args.fetch_assets:
                ctx.route('https://**/*', fetch_asset)
            page = ctx.new_page(); page.set_default_timeout(120000)
            page.goto(origin + '/index.html#transition/fold/handstand', wait_until='networkidle')
            page.wait_for_selector('#transition-player:not([hidden])')
            assert page.evaluate('typeof window.__dbg') == 'undefined'
            assert page.locator('#transition-play').get_attribute('aria-pressed') == 'false'
            ctx.close(); browser.close()
            assert input_hashes == {name: sha256((ROOT / name).read_bytes()).hexdigest() for name in inputs}, 'UI inputs changed during the frozen-snapshot suite'
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        fcntl.flock(lock, fcntl.LOCK_UN); lock.close()
    print('PASS: transitions desktop/phone, 870 routes, controls, endpoints, reduced motion and production', flush=True)


if __name__ == '__main__':
    main()
