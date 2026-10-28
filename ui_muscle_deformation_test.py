"""Run the actual muscle DQS GLSL in WebGL2 transform feedback.

This needs Chromium/Playwright but no HTTP access, built HTML or external assets.
It cross-checks CPU vertex queries and GPU deformation, not tissue physiology.
python3 ui_muscle_deformation_test.py --browser /usr/bin/chromium --out /tmp/fysm-dqs-gpu
"""
import argparse
import fcntl
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import tempfile

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser', default='/usr/bin/chromium')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    assert not args.out.resolve().is_relative_to(ROOT), 'test artifacts belong outside the repository'
    fixture = json.loads(subprocess.check_output(
        ['node', 'muscle_deformation_test.mjs', '--gpu-fixture'], cwd=ROOT, text=True))
    lock = (Path(tempfile.gettempdir()) / ('fysm-ui-' + sha256(str(ROOT).encode()).hexdigest()[:12] + '.lock')).open('w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    errors = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=args.browser, args=[
            '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        page = browser.new_page()
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.set_content('<canvas id="gpu" width="2" height="2"></canvas>')
        report = page.evaluate('''fixture => {
          const gl=document.getElementById('gpu').getContext('webgl2');
          if(!gl) throw Error('WebGL2 transform feedback is required for this test');
          const compile=(type,source)=>{
            const shader=gl.createShader(type);gl.shaderSource(shader,source);gl.compileShader(shader);
            if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(shader));
            return shader;
          };
          const source='#version 300 es\\nprecision highp float;\\nprecision highp int;\\n'
            +fixture.glsl.replace(/\\battribute\\b/g,'in')
            +'\\nin vec3 position;in vec4 skinIndex;in vec4 skinWeight;out vec3 result;'
            +'void main(){vec4 qr,qd;muscleBlendDQ(skinIndex,skinWeight,qr,qd);'
            +'float s=(muscleBellyScale-1.0)*muscleRadial.w;'
            +'result=muscleHybridPoint(position+s*muscleRadial.xyz,muscleAxis,skinIndex,skinWeight,qr,qd);gl_Position=vec4(result,1.0);}';
          const program=gl.createProgram();
          gl.attachShader(program,compile(gl.VERTEX_SHADER,source));
          gl.attachShader(program,compile(gl.FRAGMENT_SHADER,
            '#version 300 es\\nprecision highp float;out vec4 color;void main(){color=vec4(1.0);}'));
          gl.transformFeedbackVaryings(program,['result'],gl.SEPARATE_ATTRIBS);gl.linkProgram(program);
          if(!gl.getProgramParameter(program,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
          gl.useProgram(program);gl.bindVertexArray(gl.createVertexArray());
          const input=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,input);
          gl.bufferData(gl.ARRAY_BUFFER,18*4,gl.DYNAMIC_DRAW);
          for(const [name,size,offset] of [['position',3,0],['skinIndex',4,3],['skinWeight',4,7],['muscleRadial',4,11],['muscleAxis',3,15]]){
            const location=gl.getAttribLocation(program,name);if(location<0)throw Error('Missing attribute '+name);
            gl.enableVertexAttribArray(location);gl.vertexAttribPointer(location,size,gl.FLOAT,false,18*4,offset*4);
          }
          const output=gl.createBuffer();gl.bindBuffer(gl.TRANSFORM_FEEDBACK_BUFFER,output);
          gl.bufferData(gl.TRANSFORM_FEEDBACK_BUFFER,3*4,gl.DYNAMIC_READ);gl.bindBufferBase(gl.TRANSFORM_FEEDBACK_BUFFER,0,output);
          gl.enable(gl.RASTERIZER_DISCARD);
          const value=new Float32Array(3);let samples=0,maxErrorM=0;
          for(const group of fixture.groups){
            gl.uniform4fv(gl.getUniformLocation(program,'muscleDQReal[0]'),group.real);
            gl.uniform4fv(gl.getUniformLocation(program,'muscleDQDual[0]'),group.dual);
            for(const test of group.cases){
              gl.bindBuffer(gl.ARRAY_BUFFER,input);
              gl.bufferSubData(gl.ARRAY_BUFFER,0,new Float32Array([...test.point,...test.ids,...test.weights,...test.radial,...test.axis]));
              gl.uniform1f(gl.getUniformLocation(program,'muscleBellyScale'),test.scale);
              gl.beginTransformFeedback(gl.POINTS);gl.drawArrays(gl.POINTS,0,1);gl.endTransformFeedback();
              gl.bindBuffer(gl.TRANSFORM_FEEDBACK_BUFFER,output);gl.getBufferSubData(gl.TRANSFORM_FEEDBACK_BUFFER,0,value);
              if(!Array.from(value).every(Number.isFinite))throw Error('Nonfinite GPU vertex');
              const error=Math.hypot(...Array.from(value,(n,i)=>n-test.expected[i]));
              maxErrorM=Math.max(maxErrorM,error);samples++;
            }
          }
          const webglError=gl.getError();
          return {samples,maxErrorM,webglError,version:gl.getParameter(gl.VERSION),
            renderer:gl.getParameter(gl.RENDERER)};
        }''', fixture)
        browser.close()
    assert not errors, errors
    assert report['samples'] == 192, report
    assert report['webglError'] == 0, report
    assert report['maxErrorM'] < 3e-6, report
    report.update(passed=True, sourceSha256=sha256((ROOT / 'muscle_surface_render.js').read_bytes()).hexdigest())
    (args.out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
