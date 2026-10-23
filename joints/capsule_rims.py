#!/usr/bin/env python3
"""Build display-only capsule rims on bounded regions of the frozen skeleton."""
import argparse
import json
import subprocess
from pathlib import Path

import numpy as np

from build import ROOT, clip, keep_regions, normals, outer_loop, read_bones, resample_curve, unit
from check import distances


def project(points, bone):
    v, f = bone['v'], bone['f']
    tri = v[f]; a,b,c = tri[:,0],tri[:,1],tri[:,2]
    ab,ac = b-a,c-a
    d00,d01,d11 = (ab*ab).sum(1),(ab*ac).sum(1),(ac*ac).sum(1)
    denominator = d00*d11-d01*d01
    nn = normals(v,f)
    result, displacement = [], []
    for point in points:
        ap = point-a; d20,d21 = (ap*ab).sum(1),(ap*ac).sum(1)
        u = np.divide(d11*d20-d01*d21,denominator,out=np.zeros(len(f)),where=denominator>1e-24)
        w = np.divide(d00*d21-d01*d20,denominator,out=np.zeros(len(f)),where=denominator>1e-24)
        q = a+u[:,None]*ab+w[:,None]*ac
        d2 = ((q-point)**2).sum(1)
        d2[(u<0)|(w<0)|(u+w>1)|(denominator<=1e-24)] = np.inf
        bary = np.c_[1-u-w,u,w]
        candidates = [(d2,q,bary)]
        for i,j in [(0,1),(1,2),(2,0)]:
            x,y = tri[:,i],tri[:,j];e=y-x
            t = np.clip(((point-x)*e).sum(1)/np.maximum((e*e).sum(1),1e-24),0,1)
            q = x+t[:,None]*e
            bary = np.zeros((len(f),3));bary[:,i]=1-t;bary[:,j]=t
            candidates.append((((q-point)**2).sum(1),q,bary))
        best = min(((float(d.min()),int(d.argmin()),q,bary) for d,q,bary in candidates),key=lambda x:x[0])
        _,k,q,bary = best
        n = unit((nn[f[k]]*bary[k,:,None]).sum(0))
        result.append(q[k]+n*.0006)
        displacement.append(np.linalg.norm(q[k]-point))
    return np.asarray(result), displacement


def curve_project(points, curve, bone):
    # Independent nearest points may collapse adjacent source angles on one
    # socket corner. Keep the whole contour cyclically ordered instead.
    samples=resample_curve(curve,len(points)*32)
    candidates=[]
    for direction in [1,-1]:
        for shift in range(len(samples)):
            ids=(shift+direction*np.arange(len(points))*32)%len(samples)
            out=samples[ids]
            candidates.append((float(((out-points)**2).sum()),out))
    out=min(candidates,key=lambda x:x[0])[1]
    fitted,_=project(out,bone)
    return fitted,[float(np.linalg.norm(a-b)) for a,b in zip(fitted,points)]


def check_ring(points, key, min_edge=.0003):
    edge=np.linalg.norm(points-np.roll(points,-1,axis=0),axis=1)
    assert edge.min()>min_edge,(key,'collapsed adjacent rim samples')
    frame=np.linalg.svd(points-points.mean(0),full_matrices=False)[2]
    q=(points-points.mean(0))@frame[:2].T
    def orient(a,b,c):
        x,y=b-a,c-a
        return x[0]*y[1]-x[1]*y[0]
    for i in range(len(q)):
        for j in range(i+2,len(q)):
            if i==0 and j==len(q)-1:continue
            a,b=q[i],q[(i+1)%len(q)];c,d=q[j],q[(j+1)%len(q)]
            assert not (orient(a,b,c)*orient(a,b,d)<-1e-18 and
                        orient(c,d,a)*orient(c,d,b)<-1e-18),(key,'projected rim crossing',i,j)


def dense_rim(points,bone,closed,curve=None):
    samples=[];intervals=len(points) if closed else len(points)-1
    for i in range(intervals):
        for k in range(4):
            samples.append(points[i]*(1-k/4)+points[(i+1)%len(points)]*(k/4))
    samples.append(points[0] if closed else points[-1])
    samples=np.asarray(samples)
    if curve is not None:
        a,b=curve,np.roll(curve,-1,axis=0);e=b-a
        for i,p in enumerate(samples):
            if i%4==0:continue
            t=np.clip(((p-a)*e).sum(1)/np.maximum((e*e).sum(1),1e-24),0,1)
            q=a+t[:,None]*e;samples[i]=q[np.linalg.norm(q-p,axis=1).argmin()]
    result,_=project(samples,bone)
    for i,p in enumerate(points):result[i*4]=p
    if closed:result[-1]=result[0]
    return result


def section_rim(source,bone,y,closed=True):
    """Sample an actual horizontal epiphyseal section in source angular order."""
    v,f=bone['v'],bone['f'];segments=[]
    for face in f:
        polygon=v[face];crossings=[]
        for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
            if (a[1]<y)!=(b[1]<y):
                crossings.append(a+(b-a)*(y-a[1])/(b[1]-a[1]))
        if len(crossings)==2:segments.append(crossings)
    segments=np.asarray(segments)[:,:,[0,2]]
    assert len(segments)>3,'bone section missing'
    centre=(segments.reshape(-1,2).min(0)+segments.reshape(-1,2).max(0))/2
    original=source[:,[0,2]]-source[:,[0,2]].mean(0)
    angles=np.unwrap(np.arctan2(original[:,1],original[:,0]))
    direction=1 if np.median(np.diff(angles))>0 else -1
    last=angles[0]+direction*np.pi*2 if closed else angles[-1]
    extended=np.r_[angles,last] if closed else angles
    dense_angles=[]
    for a,b in zip(extended[:-1],extended[1:]):dense_angles.extend(np.linspace(a,b,4,endpoint=False))
    dense_angles.append(last)
    a=segments[:,0]-centre;e=segments[:,1]-segments[:,0]
    def cross(x,z):return x[...,0]*z[...,1]-x[...,1]*z[...,0]
    output=[]
    for theta in dense_angles:
        ray=np.array([np.cos(theta),np.sin(theta)]);den=cross(ray,e)
        t=np.divide(cross(a,e),den,out=np.full(len(a),np.nan),where=np.abs(den)>1e-12)
        u=np.divide(cross(a,ray),den,out=np.full(len(a),np.nan),where=np.abs(den)>1e-12)
        ok=(t>0)&(u>=-1e-7)&(u<=1+1e-7)
        assert ok.any(),'open source section has no radial intersection'
        q=centre+ray*t[ok].max();output.append([q[0],y,q[1]])
    dense,_=project(np.asarray(output),bone)
    if closed:dense[-1]=dense[0]
    knots=dense[np.arange(len(source))*4].copy()
    return knots,dense


def generate():
    source = subprocess.run(['node','--input-type=module','-e',
        "import * as THREE from 'three'; import {createAnatomy} from './anatomy3.js'; "
        "console.log(JSON.stringify(createAnatomy(THREE).strandInstances().filter(s=>s.kind==='cap')));"],
        cwd=ROOT,text=True,capture_output=True,check=True)
    strands=json.loads(source.stdout)
    bones,rig,sha=read_bones()
    groups={}
    for s in strands:
        groups.setdefault(s['unit']+'_'+s['side'],[]).append(s)
    result={}
    for key,insts in groups.items():
        joint,side=key.rsplit('_',1);sx=1 if side=='L' else -1
        def get(seg,latin):
            found=[b for b in bones if b['meta']['seg']==seg and b['meta']['latin']==latin and
                   (seg!='pelvis' or b['v'][:,0].mean()*sx>0)]
            assert len(found)==1,(key,seg,latin)
            return found[0]
        def B(seg,latin):return get(seg+'_'+side,latin)
        def region(b,field,near=None):
            v,f=clip(b['v'],b['f'],field(b['v'],b['normals']))
            v,f=keep_regions(v,f,near)
            return dict(meta=b['meta'],v=v,f=f,normals=normals(v,f))
        def band(b,lo,hi):
            return region(b,lambda v,n:np.minimum(v[:,1]-lo,hi-v[:,1]))
        def combined(items):
            v,f=[],[];n=0
            for b in items:v.append(b['v']);f.append(b['f']+n);n+=len(b['v'])
            vv,ff=np.concatenate(v),np.concatenate(f)
            return dict(meta=items[0]['meta'],v=vv,f=ff,normals=normals(vv,ff))
        ends=[]
        for index in [0,1]:
            anchor=[s['pts'][index] for s in insts]
            points=np.asarray([[p['p'][a] for a in ['x','y','z']] for p in anchor])
            ends.append((anchor[0]['seg'],points))
        H=np.array(rig['hum_'+side]['J']);C=np.array(rig['thigh_'+side]['J'])
        E=np.array(rig['fore_'+side]['J']);K=np.array(rig['shank_'+side]['J'])
        curve=None
        if joint=='gh':
            scap=B('scap','scapula');hum=B('hum','humerus')
            prox=region(scap,lambda v,n:np.minimum.reduce([.037-np.linalg.norm(v-H,axis=1),(H-v)[:,0]*sx-.010,np.sum(n*unit(H-v),axis=1)*.03-.004,H[1]+.018-v[:,1]]),near=H+np.array([-.026*sx,0,0]))
            curve=outer_loop(prox['v'],prox['f'])
            dist=band(hum,H[1]-.052,H[1]-.017)
            envelope=region(hum,lambda v,n:.029-np.linalg.norm(v-H,axis=1))
        elif joint=='hip':
            coxa=get('pelvis','os coxae');femur=B('thigh','femur')
            prox=region(coxa,lambda v,n:np.minimum(.035-np.linalg.norm(v-C,axis=1),np.sum(n*unit(C-v),axis=1)*.03-.005))
            curve=outer_loop(prox['v'],prox['f'])
            dist=region(femur,lambda v,n:np.minimum.reduce([v[:,1]-C[1]+.050,C[1]-.004-v[:,1],v[:,0]*sx-C[0]*sx-.015,.075-np.linalg.norm(v-C,axis=1)]))
            envelope=region(femur,lambda v,n:.029-np.linalg.norm(v-C,axis=1))
        elif joint=='kn':
            femur=B('thigh','femur');tibia=B('shank','tibia')
            prox=band(femur,K[1]+.017,K[1]+.045)
            dist=band(tibia,tibia['v'][:,1].max()-.022,tibia['v'][:,1].max()-.004)
            envelope=band(femur,K[1]-.030,K[1]+.018)
        elif joint=='el':
            hum=B('hum','humerus');ulna=B('ulna','ulna')
            prox=band(hum,E[1]+.012,E[1]+.036)
            dist=band(ulna,E[1]-.020,E[1]+.020)
            envelope=band(hum,E[1]-.015,E[1]+.018)
        elif joint=='wr':
            radius=B('fore','radius');lowest=radius['v'][:,1].min()
            prox=band(radius,lowest+.001,lowest+.010)
            carpals=[B('hand',la) for la in ['os scaphoideum','os lunatum','os triquetrum']]
            allcarp=combined(carpals);top=allcarp['v'][:,1].max()
            dist=band(allcarp,top-.011,top)
            envelope=allcarp
        elif joint=='an':
            tibia=B('shank','tibia');talus=B('foot','talus');low=tibia['v'][:,1].min()
            prox=band(tibia,low+.013,low+.030)
            dist=band(talus,talus['v'][:,1].min()+.005,talus['v'][:,1].max()-.009)
            envelope=region(talus,lambda v,n:np.minimum(v[:,1]-talus['v'][:,1].max()+.020,.010-v[:,2]))
        else:raise AssertionError(key)
        prox_points,prox_move=(curve_project(ends[0][1],curve,prox) if curve is not None else project(ends[0][1],prox))
        dist_points,dist_move=project(ends[1][1],dist)
        closed=joint!='kn'
        dense_prox=dense_rim(prox_points,prox,closed,curve)
        dense_dist=dense_rim(dist_points,dist,closed)
        if joint=='el':
            dist_points,dense_dist=section_rim(ends[1][1],dist,E[1]-.014)
        if joint=='wr':
            prox_points,dense_prox=section_rim(ends[0][1],prox,lowest+.008)
        prox_move=np.linalg.norm(prox_points-ends[0][1],axis=1)
        dist_move=np.linalg.norm(dist_points-ends[1][1],axis=1)
        lo,hi=envelope['v'].min(0),envelope['v'].max(0)
        center=(lo+hi)/2;radii=(hi-lo)/2+.0006
        result[key]=dict(source='reconstructed',sourceIds=[s['id'] for s in insts],
            closed=closed,subdivisions=4,
            prox=dict(seg=ends[0][0],positions=prox_points.tolist(),densePositions=dense_prox.tolist(),fixedSampleCount=len(dense_prox)),
            dist=dict(seg=ends[1][0],positions=dist_points.tolist(),densePositions=dense_dist.tolist(),fixedSampleCount=len(dense_dist)),
            wrap=dict(seg=envelope['meta']['seg'],c=center.tolist(),r=radii.tolist()),
            method='Display-only bounded ready-bone regions, dense between-knot projection and 0.6 mm outward offset; socket contour for shoulder/hip; ordered bone-plane sections for elbow distal and wrist proximal; source angular order retained.',
            boneRefs=[dict(seg=b['meta']['seg'],latin=b['meta']['latin']) for b in ([prox,*carpals] if joint=='wr' else [prox,dist])],
            metrics=dict(proxProjectionMaxMm=round(max(prox_move)*1000,5),distProjectionMaxMm=round(max(dist_move)*1000,5)),
            limitations=['Display reconstruction; numerical attachments, routed lengths, strain, forces and wrapping remain unchanged.',
                          'Ellipsoid dimensions come from the selected bone region; it is a display routing estimate, not a contact or collision surface.'])
        # Projection is checked against the complete relevant bone basis.
        full_prox=next((b for b in bones if b['meta']==prox['meta']),prox)
        full_dist=allcarp if joint=='wr' else next((b for b in bones if b['meta']==dist['meta']),dist)
        for label,points,b in [('prox',dense_prox,full_prox),('dist',dense_dist,full_dist)]:
            max_gap=float(distances(points,b).max())
            assert max_gap<.000625,(key,'floating capsule rim')
            result[key]['metrics'][label+'DenseMaxBoneDistanceMm']=round(max_gap*1000,6)
        assert len(prox_points)==len(dist_points)==len(insts)
        check_ring(prox_points,key+'/prox');check_ring(dist_points,key+'/dist')
        for label,points,controls in [('prox',dense_prox,prox_points),('dist',dense_dist,dist_points)]:
            check_ring(points[:-1] if closed else points,key+'/'+label+'/dense',.000005)
            assert np.array_equal(points[np.arange(len(controls))*4],controls),(key,'dense samples moved source display knots')
    return dict(v=1,source='reconstructed',basis=dict(file='skull/bones_bp3d.json',sha256=sha,coordinates='rest-world metres'),
                capsuleRims=result)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='validate and compare an in-memory rebuild without writing files')
    args=parser.parse_args();result=generate();out=Path(__file__).with_name('capsule-rims.json')
    if args.check:
        assert json.loads(out.read_text())==result,'capsule rims are not reproducible from current bones/anatomy'
    else:
        temporary=out.with_suffix('.tmp');temporary.write_text(json.dumps(result,ensure_ascii=False,separators=(',',':'))+'\n');temporary.replace(out)
    print(json.dumps(dict(capsules=len(result['capsuleRims']),points=sum(len(v['prox']['positions'])+len(v['dist']['positions']) for v in result['capsuleRims'].values()),densePoints=sum(len(v['prox']['densePositions'])+len(v['dist']['densePositions']) for v in result['capsuleRims'].values()),bytes=out.stat().st_size,reproducible=args.check)))
