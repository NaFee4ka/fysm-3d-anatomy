"""Exercise joint reconstructions on the actual built page, without regeneration.

python3 ui_joint_surfaces_test.py --out /tmp/fysm-joints --browser /usr/bin/chromium
Use --fetch-assets when Chromium cannot use the HTTPS proxy CA: urllib verifies
the original HTTPS resources and supplies their bytes at the original URLs.
Optional --baseline-cameras points to a joints-before-report.json outside git.
Screenshots/reports stay outside the repository; one browser holds the shared lock.
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
POSES = ['dog', 'plank', 'half', 'bow', 'fold', 'handstand']


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


CHECK = r"""() => {
  const d=window.__dbg,T=d.THREE,R=d.jointSurfaces;
  const out={parts:R.parts.length,visible:0,errors:[],max_owner_error:0};
  const own=new Set(R.pickables),seen=new Set();
  for(const part of R.parts) {
    const m=part.mesh,g=m.geometry,p=g.attributes.position,n=g.attributes.normal;
    if(seen.has(part.id))out.errors.push('duplicate id '+part.id);seen.add(part.id);
    if(m.visible)out.visible++;
    if(d.cartMeshes.filter(x=>x===m).length!==1)out.errors.push('cart registry '+part.id);
    if((d.glowByJoint[part.joint]||[]).filter(x=>x===m).length!==1)out.errors.push('glow registry '+part.id);
    if(m.userData.kind!=='cart'||m.userData.joint!==part.joint||!m.parent)out.errors.push('pick metadata '+part.id);
    if(!part.source||!part.method||!part.boneRefs.length)out.errors.push('provenance '+part.id);
    for(let i=0;i<p.count;i++) {
      const q=new T.Vector3().fromBufferAttribute(p,i),nn=new T.Vector3().fromBufferAttribute(n,i);
      if(![q.x,q.y,q.z,nn.x,nn.y,nn.z].every(Number.isFinite)||nn.lengthSq()<.5||nn.lengthSq()>1.5) {
        out.errors.push('invalid position/normal '+part.id+':'+i);break;
      }
    }
    const w=d.W[part.seg];m.updateWorldMatrix(true,false);
    for(const i of [0,Math.floor(p.count/2),p.count-1]) {
      const local=new T.Vector3().fromBufferAttribute(p,i);
      const expected=local.clone().applyQuaternion(w.q).add(w.p);
      const actual=local.applyMatrix4(m.matrixWorld);
      out.max_owner_error=Math.max(out.max_owner_error,actual.distanceTo(expected));
    }
  }
  const legacy=m=>!own.has(m)&&R.covered.has(m.userData.seg+'|'+m.userData.name);
  for(const [name,list] of [['cart',d.cartMeshes],['disc',d.discMeshes],...Object.entries(d.glowByJoint)])
    if(list.some(legacy))out.errors.push('retired surface in registry '+name);
  let scene=R.group;while(scene.parent)scene=scene.parent;
  scene.traverse(o=>{if(o.isMesh&&legacy(o))out.errors.push('retired surface in scene '+o.userData.name);});
  const caps=d.connectiveSurfaces.parts.filter(p=>p.kind==='cap');
  out.fitted_caps=caps.filter(p=>p.displayFit==='bone-rim').length;
  if(caps.length!==12||out.fitted_caps!==12||document.getElementById('stage').dataset.capsuleSurfaces!=='fitted')
    out.errors.push('capsule rims are not fitted for all twelve sides');
  return out;
}"""


CAPSULE_CHECK = r"""() => {
  const d=window.__dbg,R=d.connectiveSurfaces,caps=R.parts.filter(p=>p.kind==='cap');
  const original=new Map(caps.flatMap(p=>p.insts.map(inst=>[inst.id,d.A.strandPath(d.W,inst).map(q=>q.toArray())])));
  R.update(d.W);
  const out={count:caps.length,fitted:0,display_vertices:0,display_columns:0,max_rim_error:0,max_column_rim_error:0,max_length_error:0,max_strain_error:0,errors:[]};
  for(const part of caps) {
    if(part.displayFit==='bone-rim')out.fitted++;
    if(!Array.isArray(part.displayPaths)||part.displayPaths.length!==part.insts.length){out.errors.push('missing display paths '+part.unit+'_'+part.side);continue;}
    for(const path of part.displayPaths)for(const point of path){
      out.display_vertices++;
      if(!point||![point.x,point.y,point.z].every(Number.isFinite))out.errors.push('nonfinite display path '+part.unit+'_'+part.side);
    }
    if(!Array.isArray(part.displayColumnPaths)||part.displayColumnPaths.length!==part.columns){
      out.errors.push('missing dense display columns '+part.unit+'_'+part.side);
    } else {
      out.display_columns+=part.displayColumnPaths.length;
      for(const path of part.displayColumnPaths)for(const point of path)
        if(!point||![point.x,point.y,point.z].every(Number.isFinite))out.errors.push('nonfinite dense column '+part.unit+'_'+part.side);
      const geometry=part.mesh.geometry,position=geometry.attributes.position,rows=position.count/part.columns;
      if(!Number.isInteger(rows)||rows<2)out.errors.push('invalid dense row layout '+part.unit+'_'+part.side);
      part.mesh.updateWorldMatrix(true,false);
      for(const [end,row] of [['prox',0],['dist',rows-1]]) {
        const rim=part.displayRims?.[end],dense=rim?.densePositions;
        if(!rim||!Array.isArray(dense)||dense.length!==part.columns){out.errors.push('missing dense rim '+part.unit+'_'+part.side+'/'+end);continue;}
        for(let column=0;column<part.columns;column++) {
          const expected=d.A.pt(d.W,rim.seg,dense[column]);
          const rendered=new d.THREE.Vector3().fromBufferAttribute(position,row*part.columns+column).applyMatrix4(part.mesh.matrixWorld);
          const path=part.displayColumnPaths[column],point=path[end==='prox'?0:path.length-1];
          out.max_rim_error=Math.max(out.max_rim_error,rendered.distanceTo(expected));
          out.max_column_rim_error=Math.max(out.max_column_rim_error,point.distanceTo(expected));
        }
      }
    }
    let expectedStrain=-Infinity;
    part.insts.forEach((inst,i)=>{
      const points=d.A.strandPath(d.W,inst),before=original.get(inst.id);
      if(JSON.stringify(points.map(p=>p.toArray()))!==JSON.stringify(before))out.errors.push('changed mechanical path '+inst.id);
      const length=d.A.pathLength(points);
      out.max_length_error=Math.max(out.max_length_error,Math.abs(length-part.lengths[i]));
      expectedStrain=Math.max(expectedStrain,length/part.restLengths[i]-1);
    });
    out.max_strain_error=Math.max(out.max_strain_error,Math.abs(expectedStrain-part.strain));
  }
  if(out.max_rim_error>1e-6||out.max_column_rim_error>1e-6)out.errors.push('rendered capsule boundaries left their dense bone rims');
  return out;
}"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--browser')
    parser.add_argument('--fetch-assets', action='store_true')
    parser.add_argument('--baseline-cameras', type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    assert not args.out.resolve().is_relative_to(ROOT), 'Screenshots must stay outside git'
    manifest_text = (ROOT / 'joints/joints.json').read_text(encoding='utf-8')
    manifest = json.loads(manifest_text)
    expected = len(manifest['parts'])
    html = (ROOT / 'dist/test.html').read_text(encoding='utf-8')
    assert html.count(manifest_text) == 1, 'Build is stale relative to joint data'
    broken = dict(manifest, data='AA==')
    corrupted_html = html.replace(manifest_text, json.dumps(broken, ensure_ascii=False))
    lock_path = Path(tempfile.gettempdir()) / ('fysm-ui-' + sha256(str(ROOT).encode()).hexdigest()[:12] + '.lock')
    lock = lock_path.open('w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(ROOT / 'dist')))
    Thread(target=server.serve_forever, daemon=True).start()
    cache, report = {}, {
        'expected_parts': expected,
        'data_sha256': sha256(manifest_text.encode()).hexdigest(),
        'html_sha256': sha256(html.encode()).hexdigest(),
    }

    def fetch_asset(route):
        url = route.request.url
        if url not in cache:
            with urllib.request.urlopen(url, timeout=30) as response:
                headers = {k: v for k, v in response.headers.items()
                           if k.lower() not in ('content-encoding', 'content-length', 'transfer-encoding')}
                cache[url] = response.status, headers, response.read()
        status, headers, body = cache[url]
        route.fulfill(status=status, headers=headers, body=body)

    def settle(page, reset=True):
        page.evaluate('() => new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))')
        if reset:
            page.evaluate('window.__dbg.resetView(false)')
            page.evaluate('() => new Promise(r=>requestAnimationFrame(r))')

    def check(page, label):
        result = page.evaluate(CHECK)
        assert result['parts'] == expected, (label, result)
        assert not result['errors'], (label, result['errors'])
        assert result['max_owner_error'] < 1e-6, (label, result)
        capsules = page.evaluate(CAPSULE_CHECK)
        assert capsules['count'] == 12 and capsules['fitted'] == 12 and capsules['display_vertices'] > 0, (label, capsules)
        assert not capsules['errors'], (label, capsules)
        assert capsules['max_rim_error'] < 1e-6 and capsules['max_column_rim_error'] < 1e-6, (label, capsules)
        assert capsules['max_length_error'] < 1e-9 and capsules['max_strain_error'] < 1e-9, (label, capsules)
        result['capsules'] = capsules
        return result

    def context_page(browser, width, height, reduced='reduce'):
        context = browser.new_context(viewport={'width': width, 'height': height}, reduced_motion=reduced)
        if args.fetch_assets:
            context.route('https://**/*', fetch_asset)
        page = context.new_page()
        page.set_default_timeout(90000)
        errors, warnings = [], []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('console', lambda message: warnings.append(message.text) if message.type in ('error', 'warning') else None)
        return context, page, errors, warnings

    def open_ready(page, filename='test.html'):
        page.goto(f'http://127.0.0.1:{server.server_port}/{filename}', wait_until='networkidle')
        page.wait_for_selector('#stage[data-joint-surfaces="ready"]', timeout=120000)
        page.wait_for_function('window.__skullReady === true')
        if filename == 'test.html':
            page.evaluate("()=>{const d=window.__dbg;for(const l of ['musclesOuter','musclesDeep','fascia','flines','forces'])d.setLayer(l,false);for(const l of ['bones','joints','ligs'])d.setLayer(l,true);d.setLayer('light',false);}")
            settle(page)

    try:
        with sync_playwright() as playwright:
            options = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
            if args.browser:
                options['executable_path'] = args.browser
            browser = playwright.chromium.launch(**options)
            for name, width, height in [('desktop', 1300, 820), ('phone', 390, 844)]:
                context, page, errors, warnings = context_page(browser, width, height)
                open_ready(page)
                result = {'poses': [], 'errors': errors, 'warnings': warnings}
                assert page.evaluate('document.documentElement.scrollWidth<=document.documentElement.clientWidth')
                for pose in POSES:
                    page.locator(f'[data-pose="{pose}"]').click()
                    for t in [0, .5, 1]:
                        page.evaluate('(t)=>window.__dbg.setT(t)', t)
                        settle(page)
                        state = check(page, f'{name}/{pose}/{t}')
                        assert state['visible'] == expected
                        result['poses'].append({'pose': pose, 't': t, **state})
                    page.locator('#stage').screenshot(path=str(args.out / f'{name}-{pose}.png'), animations='disabled')
                    print(name, pose, 'joint surfaces:', expected, flush=True)
                # Toggle repeatedly: removed geometry must stay out of both scene
                # and registries when the layer returns.
                for _ in range(2):
                    page.evaluate("window.__dbg.setLayer('joints',false)")
                    settle(page)
                    assert check(page, name+'/joints off')['visible'] == 0
                    assert page.evaluate("window.__dbg.connectiveSurfaces.parts.filter(p=>p.kind==='cap').every(p=>!p.mesh.visible)")
                    page.evaluate("window.__dbg.setLayer('joints',true)")
                    settle(page)
                    assert check(page, name+'/joints on')['visible'] == expected
                # A real canvas tap on a triangle (not a programmatic select).
                # The diagnostic camera gives phone pixels enough area to hit.
                page.evaluate("""()=>{const d=window.__dbg,T=d.THREE,j=d.jointById.kn_L.def;
                  d.switchPose('dog');d.setT(0);d.setLayer('bones',false);d.setLayer('ligs',false);d.select(null);
                }""")
                settle(page)
                page.evaluate("""()=>{const d=window.__dbg,T=d.THREE,j=d.jointById.kn_L.def,target=d.A.pt(d.W,j.seg,new T.Vector3(...j.at));
                  d.state.userMoved=true;d.state.camAnim=null;d.controls.target.copy(target);d.camera.up.set(0,1,0);
                  d.camera.position.copy(target).add(new T.Vector3(.16,.06,.29));d.camera.lookAt(target);d.controls.update();d.camera.updateMatrixWorld(true);}""")
                settle(page, reset=False)
                probe = page.evaluate("""()=>{const d=window.__dbg,T=d.THREE,rect=document.getElementById('view').getBoundingClientRect(),ray=new T.Raycaster();
                  const surfaces=d.jointSurfaces.pickables.filter(m=>m.visible&&m.userData.joint==='kn_L');
                  const objects=[...Object.values(d.jointById).map(j=>j.pick),...d.cartMeshes,...d.discMeshes,...d.connectiveSurfaces.pickables.filter(m=>m.visible)];
                  for(const mesh of surfaces){const ids=mesh.geometry.index;
                    for(let k=0;k<ids.count;k+=Math.max(3,Math.floor(ids.count/40/3)*3)){
                      const p=new T.Vector3();for(let j=0;j<3;j++)p.add(mesh.getVertexPosition(ids.getX(k+j),new T.Vector3()));mesh.localToWorld(p.divideScalar(3));p.project(d.camera);
                      if(Math.abs(p.x)>.85||Math.abs(p.y)>.85||p.z<0||p.z>1)continue;
                      ray.setFromCamera(new T.Vector2(p.x,p.y),d.camera);const hit=ray.intersectObjects(objects,false)[0];if(!hit)continue;
                      const q=hit.object.userData,id=q.kind==='joint'?q.id:q.kind==='cart'?q.joint:q.kind==='cap'?q.unit+'_'+q.side:null;
                      const x=rect.left+(p.x+1)*rect.width/2,y=rect.top+(1-p.y)*rect.height/2;
                      if(id==='kn_L'&&y>rect.top+15&&y<Math.min(rect.bottom,innerHeight)-20)return {x,y,id,first_kind:q.kind,surface_id:mesh.userData.id};
                    }}return null;}""")
                assert probe, 'No joint triangle available for real pointer selection'
                page.mouse.click(probe['x'], probe['y'])
                settle(page, reset=False)
                selected = page.evaluate('window.__dbg.state.sel')
                assert selected and selected['kind'] == 'joint' and selected['id'] == probe['id'], (selected, probe)
                assert 'реконструкц' in page.locator('#info').inner_text().lower()
                result['pointer_pick'] = {'probe': probe, 'selected': selected}
                page.locator('#stage').screenshot(path=str(args.out / f'{name}-knee-selected.png'), animations='disabled')
                page.evaluate("window.__dbg.select(null);window.__dbg.setT(1);window.__dbg.setLayer('bones',true);window.__dbg.setLayer('ligs',true);window.__dbg.setLayer('light',true)")
                settle(page)
                assert page.evaluate('window.__dbg.jointSurfaces.parts.some(p=>p.mat.emissiveIntensity>0)')
                page.evaluate("window.__dbg.setLayer('light',false)")
                settle(page)
                assert page.evaluate('window.__dbg.jointSurfaces.parts.every(p=>p.mat.emissiveIntensity===0)')
                page.evaluate('window.__dbg.enterCycle();window.__dbg.cycPause()')
                result['cycle'] = []
                for t in [0, 4.2, 10.6, 15.5, 20.5, 23, 26.1]:
                    page.evaluate('(t)=>window.__dbg.setCycT(t)', t)
                    settle(page)
                    result['cycle'].append({'t': t, **check(page, name+'/cycle/'+str(t))})
                page.locator('#stage').screenshot(path=str(args.out / f'{name}-cycle.png'), animations='disabled')
                result['source_note'] = page.locator('#joint-source-note').inner_text()
                assert 'реконструкц' in result['source_note'].lower()
                assert not errors, errors
                assert not warnings, warnings
                report[name] = result
                context.close()

            # Fallback must retain the legacy surfaces and identify the limitation.
            report['fallback'] = {}
            for mode in ['no-decompressor', 'corrupted-payload']:
                context, page, errors, warnings = context_page(browser, 390, 844)
                if mode == 'no-decompressor':
                    context.add_init_script("Object.defineProperty(window,'DecompressionStream',{value:undefined,configurable:true})")
                    filename = 'test.html'
                else:
                    context.route('**/corrupt.html', lambda route: route.fulfill(status=200, content_type='text/html; charset=utf-8', body=corrupted_html))
                    filename = 'corrupt.html'
                page.goto(f'http://127.0.0.1:{server.server_port}/{filename}', wait_until='networkidle')
                page.wait_for_selector('#stage[data-joint-surfaces="fallback"]', timeout=120000)
                fallback = page.evaluate("""(parts)=>{const d=window.__dbg,keys=new Set(parts.flatMap(p=>p.replaces.map(name=>p.seg+'|'+name)));
                  const legacy=d.cartMeshes.filter(m=>keys.has(m.userData.seg+'|'+m.userData.name));
                  return {renderer:d.jointSurfaces===null,legacy_count:legacy.length,visible:legacy.filter(m=>m.visible&&m.parent).length,note:document.getElementById('joint-source-note').textContent};}""", manifest['parts'])
                assert fallback['renderer'] and fallback['legacy_count'] > 0 and fallback['visible'] > 0, fallback
                assert 'схема' in fallback['note'].lower() and 'недоступны' in fallback['note'].lower()
                assert not errors, errors
                assert any('Joint surface reconstructions could not load' in text for text in warnings), warnings
                report['fallback'][mode] = {**fallback, 'errors': errors, 'expected_warnings': warnings}
                context.close()

            context, page, errors, warnings = context_page(browser, 390, 844)
            open_ready(page, 'index.html')
            report['production'] = page.evaluate("""()=>({debug:typeof window.__dbg,charset:document.characterSet,
              overflow:document.documentElement.scrollWidth>document.documentElement.clientWidth,
              ready:document.getElementById('stage').dataset.jointSurfaces,note:document.getElementById('joint-source-note').textContent})""")
            assert report['production']['debug'] == 'undefined' and report['production']['charset'] == 'UTF-8'
            assert not report['production']['overflow']
            assert not errors and not warnings, (errors, warnings)
            context.close()

            if args.baseline_cameras:
                baseline = json.loads(args.baseline_cameras.read_text())
                context, page, errors, warnings = context_page(browser, 1300, 900)
                open_ready(page)
                report['same_camera_views'] = []
                for view in baseline['views']:
                    page.evaluate("({pose,t})=>{window.__dbg.select(null);window.__dbg.switchPose(pose);window.__dbg.setT(t);}", view)
                    settle(page)
                    actual = page.evaluate("""(v)=>{const d=window.__dbg;d.state.userMoved=true;d.state.camAnim=null;d.controls.target.fromArray(v.target);d.camera.position.fromArray(v.camera);d.camera.up.fromArray(v.up||[0,1,0]);d.camera.quaternion.fromArray(v.quaternion);d.controls.update();d.camera.updateMatrixWorld(true);return {target:d.controls.target.toArray(),camera:d.camera.position.toArray(),quaternion:d.camera.quaternion.toArray(),fov:d.camera.fov,aspect:d.camera.aspect};}""", view)
                    for key in ['target', 'camera', 'quaternion']:
                        assert max(abs(a-b) for a, b in zip(actual[key], view[key])) < 1e-7, (view['name'], key)
                    assert abs(actual['fov']-view['fov']) < 1e-7 and abs(actual['aspect']-view['aspect']) < 1e-7
                    settle(page, reset=False)
                    stem = view['name'].replace('joints-before-', 'joints-after-')
                    page.locator('#stage').screenshot(path=str(args.out / (stem+'.png')), animations='disabled')
                    report['same_camera_views'].append({'name': stem, **actual})
                    print(stem, flush=True)
                assert not errors and not warnings, (errors, warnings)
                context.close()
            browser.close()
    finally:
        server.shutdown()
        (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        lock.close()
    print('PASS: joint reconstructions, poses, round, registries, pointer, layers, fallback and production', flush=True)


if __name__ == '__main__':
    main()
