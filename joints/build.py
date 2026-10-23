#!/usr/bin/env python3
"""Reconstruct joint surfaces on the finished packed skeleton; never edits bones."""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-15)


def compact(v, f):
    u, inverse = np.unique(np.round(v, 10), axis=0, return_inverse=True)
    f = inverse[f]
    area = np.linalg.norm(np.cross(u[f[:, 1]]-u[f[:, 0]], u[f[:, 2]]-u[f[:, 0]]), axis=1)
    f = f[area > 1e-18]
    used, inv = np.unique(f, return_inverse=True)
    return u[used], inv.reshape(-1, 3)


def normals(v, f):
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return unit(n)


def read_bones():
    file = ROOT/'skull/bones_bp3d.json'
    data = json.loads(file.read_text())
    rig = json.loads((ROOT/'skull/rig_rest.json').read_text())['segs']
    raw, offset, bones = gzip.decompress(base64.b64decode(data['data'])), 0, []
    for p in data['parts']:
        q = np.frombuffer(raw, '<u2', p['n']*3, offset).reshape(-1, 3)
        offset += p['n']*6
        q = np.cumsum(q.astype(np.uint64), axis=0) & 65535
        v = np.asarray(p['lo']) + q/65535*(np.asarray(p['hi'])-p['lo']) + rig[p['seg']]['J']
        d = np.frombuffer(raw, '<u2', p['m']*3, offset)
        offset += p['m']*6
        f = (np.cumsum(d.astype(np.uint64)) & 65535).reshape(-1, 3).astype(np.int64)
        v, f = compact(v, f)
        # Source winding is retained except a globally reversed solid.
        signed = np.einsum('ij,ij->i', v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum()
        if signed < 0:
            f = f[:, ::-1]
        bones.append(dict(meta=p, v=v, f=f, normals=normals(v, f)))
    assert offset == len(raw)
    return bones, rig, hashlib.sha256(file.read_bytes()).hexdigest()


def clip(v, f, values):
    """Continuous scalar clipping, followed by exact-position weld at new edges."""
    out, faces = [], []
    for face in f:
        polygon = [(v[i], values[i]) for i in face]
        clipped = []
        for a, b in zip(polygon, polygon[1:]+polygon[:1]):
            if a[1] >= 0:
                clipped.append(a[0])
            if (a[1] >= 0) != (b[1] >= 0):
                t = a[1]/(a[1]-b[1])
                clipped.append(a[0]+t*(b[0]-a[0]))
        for k in range(1, len(clipped)-1):
            j = len(out)
            out.extend([clipped[0], clipped[k], clipped[k+1]])
            faces.append([j, j+1, j+2])
    if not faces:
        raise ValueError('empty anatomical region')
    return compact(np.asarray(out), np.asarray(faces))


def subdivide(v, f, limit=.008, max_rounds=8):
    for _ in range(max_rounds):
        edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        edges, inv = np.unique(edges, axis=0, return_inverse=True)
        split = np.linalg.norm(v[edges[:, 0]]-v[edges[:, 1]], axis=1) > limit
        if not split.any():
            break
        mids = np.full(len(edges), -1, dtype=np.int64)
        mids[split] = len(v)+np.arange(split.sum())
        mid = mids[inv.reshape(3, -1).T]
        output = []
        for tri, m in zip(f, mid):
            count = int((m >= 0).sum())
            if count == 0:
                output.append(tri.tolist()); continue
            if count == 3:
                a,b,c = tri; ab,bc,ca = m
                output.extend([[a,ab,ca],[ab,b,bc],[ca,bc,c],[ab,bc,ca]])
            else:
                # Rotate only, retaining the input triangle winding.
                k = int(np.flatnonzero(m >= 0)[0]) if count == 1 else (int(np.flatnonzero(m < 0)[0])+1) % 3
                a,b,c = np.roll(tri, -k); ab,bc,ca = np.roll(m, -k)
                if count == 1:
                    output.extend([[a,ab,c],[ab,b,c]])
                else:
                    output.extend([[b,bc,ab],[a,ab,c],[ab,bc,c]])
        f = np.asarray(output)
        v = np.concatenate([v, v[edges[split]].mean(axis=1)])
    return v, f


def boundary_loops(f):
    edges = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    _, ids, count = np.unique(np.sort(edges, axis=1), axis=0, return_index=True, return_counts=True)
    edges = edges[ids[count == 1]]
    return edges


def keep_regions(v, f, near=None):
    """Drop only tiny mask slivers, retain substantial disconnected condyles."""
    parent = np.arange(len(v))
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, c in f:
        for q in (b, c):
            aa, bb = find(a), find(q)
            parent[bb] = aa
    group = np.array([find(face[0]) for face in f])
    area = np.linalg.norm(np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]]), axis=1)/2
    keys = np.unique(group)
    total = {int(k): float(area[group == k].sum()) for k in keys}
    threshold = min(max(total.values())*.025, 4e-6)
    if near is not None:
        key = min(keys, key=lambda k:np.linalg.norm(v[np.unique(f[group == k])].mean(0)-near))
        use = group == key
    else:
        use = np.array([total[int(k)] >= threshold for k in group])
    return compact(v, f[use])


def split_vertex_fans(v, f):
    """A point-touch between separate face fans must not share an extrusion edge."""
    incident = [[] for _ in v]
    for face_index, face in enumerate(f):
        for vertex in face:
            incident[vertex].append(face_index)
    points, faces = v.tolist(), f.copy()
    for vertex, face_ids in enumerate(incident):
        if len(face_ids) < 2:
            continue
        neighbors = {}
        for face_id in face_ids:
            for other in f[face_id]:
                if other != vertex:
                    neighbors.setdefault(int(other), []).append(face_id)
        adjacency = {k: set() for k in face_ids}
        for group in neighbors.values():
            for a in group:
                adjacency[a].update(group)
        remaining, groups = set(face_ids), []
        while remaining:
            todo, component = [min(remaining)], set()
            while todo:
                a = todo.pop()
                if a in component:
                    continue
                component.add(a)
                todo.extend(adjacency[a]-component)
            remaining -= component
            groups.append(component)
        for component in groups[1:]:
            new_vertex = len(points); points.append(v[vertex].tolist())
            for face_id in component:
                faces[face_id, faces[face_id] == vertex] = new_vertex
    return np.asarray(points), faces


def shell(v, f, thickness):
    n = normals(v, f)
    # Outer face stands off 0.08 mm from bone; thickness is an illustrative value.
    bottom = v+n*.00008
    top = v+n*thickness
    edges = boundary_loops(f)
    N = len(v)
    sides = np.concatenate([np.c_[edges[:, 0], edges[:, 1], edges[:, 1]+N],
                            np.c_[edges[:, 0], edges[:, 1]+N, edges[:, 0]+N]])
    faces = np.concatenate([f[:, ::-1], f+N, sides])
    return np.concatenate([bottom, top]), faces


def resample_curve(v, N, closed=True):
    p = np.concatenate([v, v[:1]]) if closed else v
    length = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    t = np.linspace(0, length[-1], N, endpoint=not closed)
    return np.stack([np.interp(t, length, p[:, k]) for k in range(3)], axis=1)


def outer_loop(v, f):
    edges = boundary_loops(f)
    neighbors = {}
    for a, b in edges:
        neighbors.setdefault(int(a), []).append(int(b))
        neighbors.setdefault(int(b), []).append(int(a))
    loops, remaining = [], {tuple(sorted(e)) for e in edges}
    while remaining:
        a, b = next(iter(remaining)); chain = [a, b]; remaining.remove(tuple(sorted((a, b))))
        while chain[-1] != chain[0]:
            options = [k for k in neighbors[chain[-1]] if tuple(sorted((chain[-1], k))) in remaining]
            if not options:
                break
            c = options[0]; remaining.remove(tuple(sorted((chain[-1], c)))); chain.append(c)
        if len(chain) > 5 and chain[0] == chain[-1]:
            loops.append(v[chain[:-1]])
    if not loops:
        raise ValueError('socket rim has no closed contour')
    return max(loops, key=lambda p: np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1).sum())


def rim(v, f, centre, height, width):
    p = resample_curve(outer_loop(v, f), 96)
    # Two mild fairing steps remove triangle-level stair steps, <= 0.6 mm typically.
    for _ in range(2):
        p = p*.7+(np.roll(p, 1, axis=0)+np.roll(p, -1, axis=0))*.15
    t = unit(np.roll(p, -1, axis=0)-np.roll(p, 1, axis=0))
    inward = unit(centre-p)
    across = unit(np.cross(t, inward))
    # Labrum is a raised triangular lip rather than a circular torus.
    profile = np.array([[-.5, 0], [-.38, .65], [0, 1], [.38, .55], [.5, 0], [0, -.08]])
    pos = (p[:, None, :]+across[:, None, :]*profile[None, :, :1]*width+
           inward[:, None, :]*profile[None, :, 1:]*height).reshape(-1, 3)
    idx = []
    K = len(profile)
    for i in range(len(p)):
        for k in range(K):
            a, b, c, d = i*K+k, ((i+1) % len(p))*K+k, ((i+1) % len(p))*K+(k+1) % K, i*K+(k+1) % K
            idx += [[a, b, c], [a, c, d]]
    return pos, np.asarray(idx)


def upper_y(bone, x, z):
    v, f = bone['v'], bone['f']; tri = v[f]
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    d00, d01, d10, d11 = b[:, 0]-a[:, 0], c[:, 0]-a[:, 0], b[:, 2]-a[:, 2], c[:, 2]-a[:, 2]
    det = d00*d11-d01*d10
    safe = np.abs(det) > 1e-12
    u = np.divide((x-a[:, 0])*d11-(z-a[:, 2])*d01, det, out=np.zeros(len(f)), where=safe)
    w = np.divide(d00*(z-a[:, 2])-d10*(x-a[:, 0]), det, out=np.zeros(len(f)), where=safe)
    y = a[:, 1]+u*(b[:, 1]-a[:, 1])+w*(c[:, 1]-a[:, 1])
    ok = safe & (u >= -1e-8) & (w >= -1e-8) & (u+w <= 1+1e-8)
    return y[ok].max() if ok.any() else None


def meniscus(bone, sx, medial):
    v = bone['v']; top = v[v[:, 1] > v[:, 1].max()-.012]
    mid = np.median(top[:, 0]*sx)
    cloud = top[(top[:, 0]*sx < mid) if medial else (top[:, 0]*sx >= mid)]
    lo, hi = np.percentile(cloud[:, [0, 2]], [5, 95], axis=0)
    cx, cz = (lo+hi)/2
    rx, rz = (hi-lo)/2*.87
    # Opening faces the intercondylar area; medial C is wider open than lateral C.
    opening = 1 if (medial == (sx > 0)) else -1
    middle = 0 if opening > 0 else np.pi
    gap = np.deg2rad(86 if medial else 54)
    theta = np.linspace(middle+gap/2, middle+2*np.pi-gap/2, 65)
    profile = np.array([[.56, .00015], [.68, .00055], [.89, .0023], [1, .0018], [1, .0001], [.56, .0001]])
    pos = []
    for i, angle in enumerate(theta):
        taper = .32+.68*np.sin(np.pi*i/(len(theta)-1))**.35
        for radius, dy in profile:
            x, z = cx+rx*radius*np.cos(angle), cz+rz*radius*np.sin(angle)
            y = upper_y(bone, x, z)
            for _ in range(12):
                if y is not None and y > v[:, 1].max()-.018:
                    break
                x, z = cx+(x-cx)*.93, cz+(z-cz)*.93
                y = upper_y(bone, x, z)
            assert y is not None, 'meniscal footprint outside plateau'
            pos.append([x, y+dy*taper, z])
    f, K = [], len(profile)
    for i in range(len(theta)-1):
        for k in range(K):
            a, b, c, d = i*K+k, (i+1)*K+k, (i+1)*K+(k+1) % K, i*K+(k+1) % K
            f += [[a, b, c], [a, c, d]]
    for end, reverse in ((0, True), (len(theta)-1, False)):
        for k in range(1, K-1):
            face = [end*K, end*K+k, end*K+k+1]
            f.append(face if reverse else face[::-1])
    return np.asarray(pos), np.asarray(f), dict(centreXZ=[float(cx), float(cz)], radiusXZ=[float(rx), float(rz)], openingDegrees=float(np.rad2deg(gap)))


def build():
    bones, rig, bone_hash = read_bones()
    parts, arrays, reports = [], [], []
    def get(seg, latin, sx=None):
        found = [b for b in bones if b['meta']['seg'] == seg and b['meta']['latin'] == latin]
        if sx is not None:
            found = [b for b in found if b['v'][:, 0].mean()*sx > 0]
        assert len(found) == 1, (seg, latin, len(found))
        return found[0]
    def add(id_, joint, side, bone, name, latin, type_, v, f, replaces, method, parameters):
        # Preserve coincident but topologically distinct fans after extrusion.
        # Rewelding them would recreate a vertical edge with 4+ incident faces.
        v, f = np.asarray(v), np.asarray(f)
        signed = np.einsum('ij,ij->i', v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum()
        if signed < 0:
            f = f[:, ::-1]
        p = dict(id=f'{id_}_{side}', joint=f'{joint}_{side}', seg=bone['meta']['seg'], side=side,
                 name=name, latin=latin, type=type_, source='reconstructed', ids=[],
                 method=method, boneRefs=[dict(seg=bone['meta']['seg'], name=bone['meta']['name'], latin=bone['meta']['latin'])],
                 replaces=replaces, parameters=parameters,
                 limitations=['Регион и толщина реконструированы по поверхности кости; это не сегментация хряща из атласа.',
                              'Риг не рассчитывает суставной контакт, компрессию и скольжение тканей.'])
        parts.append(p); arrays.append((v, f))
        edges = boundary_loops(f)
        assert not len(edges), (id_, 'unsealed shell', len(edges))
        directed = np.concatenate([f[:, [0,1]], f[:, [1,2]], f[:, [2,0]]])
        _, inv, counts = np.unique(np.sort(directed,axis=1),axis=0,return_inverse=True,return_counts=True)
        assert (counts == 2).all(), (id_, 'nonmanifold shell')
        direction = np.where(directed[:,0] < directed[:,1], 1, -1)
        assert not np.bincount(inv,weights=direction).any(), (id_, 'inconsistent winding')
        reports.append(dict(id=p['id'], vertices=len(v), triangles=len(f), boundaryEdges=len(edges),
                            nonmanifoldEdges=int((counts != 2).sum()),
                            inconsistentWindingEdges=int((np.bincount(inv,weights=direction) != 0).sum()),
                            areaMm2=round(float(np.linalg.norm(np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]]), axis=1).sum()*5e5), 2)))
    for side, sx in [('L', 1), ('R', -1)]:
        def B(seg, latin):
            return get(f'{seg}_{side}', latin)
        def patch(id_, joint, bone, name, latin, field, replaces, region, thickness=.0008, details=None, near=None):
            v, f = clip(bone['v'], bone['f'], field(bone['v'], bone['normals']))
            v, f = keep_regions(v, f, near)
            v, f = subdivide(v, f)
            before_split = len(v)
            v, f = split_vertex_fans(v, f)
            vv, ff = shell(v, f, thickness)
            add(id_, joint, side, bone, name, latin, 'cartilage', vv, ff, replaces,
                'Region-clipped ready-bone faces; interpolated normal offset; sealed rim and base.',
                dict(region=region, thicknessMm=thickness*1000, standOffMm=.08,
                     splitPointTouchFans=len(v)-before_split, **(details or {})))
            return v, f
        hum, scap = B('hum', 'humerus'), B('scap', 'scapula')
        H = np.array(rig[f'hum_{side}']['J'])
        direction = unit(np.array([-sx, .28, 0]))
        patch('humeral_head', 'gh', hum, 'Хрящ головки плечевой кости', 'cartilago capitis humeri',
              lambda v,n: np.minimum(.029-np.linalg.norm(v-H, axis=1), (v-H)@direction+.002),
              ['Хрящ головки плечевой кости'], 'Head around shoulder centre, medial-superior cap; neck and tubercles excluded.', .0010)
        glen_field = lambda v,n: np.minimum.reduce([.037-np.linalg.norm(v-H, axis=1),
                         (H-v)[:, 0]*sx-.010, np.sum(n*unit(H-v), axis=1)*.03-.004,
                         H[1]+.018-v[:,1]])
        gv, gf = patch('glenoid', 'gh', scap, 'Хрящ суставной впадины лопатки', 'cartilago cavitatis glenoidalis',
                       glen_field, ['Суставная впадина лопатки'], 'Lateral scapular socket facing humeral centre; <=37 mm from centre, below shoulder+18 mm; component nearest glenoid landmark, acromion excluded.', .0007,
                       near=H+np.array([-.026*sx,0,0]))
        vv, ff = rim(gv, gf, H, .0021, .0036)
        add('glenoid_labrum', 'gh', side, scap, 'Суставная губа', 'labrum glenoidale', 'labrum', vv, ff,
            ['Суставная губа'], 'Resampled outer boundary of bone-derived glenoid patch; closed triangular rim.', dict(heightMm=2.1, widthMm=3.6))
        femur, coxa = B('thigh', 'femur'), get('pelvis', 'os coxae', sx)
        C = np.array(rig[f'thigh_{side}']['J']); direction = unit(np.array([-sx, .28, .12]))
        patch('femoral_head', 'hip', femur, 'Хрящ головки бедренной кости', 'cartilago capitis femoris',
              lambda v,n: np.minimum(.029-np.linalg.norm(v-C, axis=1), (v-C)@direction+.006),
              ['Хрящ головки бедренной кости'], 'Femoral head within 29 mm of hip centre; medial cap excluding neck.', .0010)
        socket = lambda v,n: np.minimum(.035-np.linalg.norm(v-C, axis=1), np.sum(n*unit(C-v), axis=1)*.03-.005)
        sv, sf = clip(coxa['v'], coxa['f'], socket(coxa['v'], coxa['normals']))
        sv, sf = keep_regions(sv, sf)
        sv, sf = subdivide(sv, sf)
        axis = unit(np.array([-sx, .24, .1]))
        def lune(v, n):
            d = v-C; perpendicular = np.linalg.norm(d-np.outer(d@axis, axis), axis=1)
            # Central fossa and inferior acetabular notch are uncoated.
            return np.minimum.reduce([socket(v,n), perpendicular-.010, v[:,1]-C[1]+.017])
        patch('acetabulum', 'hip', coxa, 'Лунная поверхность вертлужной впадины', 'facies lunata acetabuli',
              lune, [], 'Acetabular faces facing hip centre; central 10 mm fossa and inferior notch excluded.', .0008)
        vv, ff = rim(sv, sf, C, .0025, .0042)
        add('acetabular_labrum', 'hip', side, coxa, 'Вертлужная губа', 'labrum acetabulare', 'labrum', vv, ff,
            ['Вертлужная губа'], 'Resampled outer bone-derived acetabular socket contour; closed triangular lip.', dict(heightMm=2.5, widthMm=4.2))
        tibia, patella = B('shank', 'tibia'), B('shank', 'patella')
        K = np.array(rig[f'shank_{side}']['J'])
        patch('femoral_condyles', 'kn', femur, 'Хрящ мыщелков бедра', 'cartilago condylorum femoris',
              lambda v,n: np.minimum(K[1]+.012-v[:,1], np.maximum(.006+v[:,2]-K[2], K[1]-.002-v[:,1])),
              ['Хрящ мыщелков бедра'], 'Distal femoral condyles and anterior trochlear face; shaft/epicondyles excluded.', .0009)
        ty = tibia['v'][:,1].max()
        patch('tibial_plateau', 'kn', tibia, 'Хрящ плато большеберцовой кости', 'cartilago tibiae',
              lambda v,n: np.minimum(v[:,1]-ty+.011, n[:,1]*.02-.004),
              ['Хрящ плато большеберцовой кости'], 'Superior tibial plateau within 11 mm of its highest point; upward-facing faces.', .00065)
        patch('patella', 'kn', patella, 'Хрящ надколенника', 'cartilago patellae',
              lambda v,n: np.minimum(-n[:,2]*.02-.002, np.median(patella['v'][:,2])+.002-v[:,2]),
              ['Хрящ надколенника'], 'Posterior patellar facets; anterior subcutaneous face excluded.', .0009)
        for medial in [True, False]:
            vv, ff, metrics = meniscus(tibia, sx, medial)
            name = 'Медиальный мениск' if medial else 'Латеральный мениск'
            add('medial_meniscus' if medial else 'lateral_meniscus', 'kn', side, tibia, name,
                'meniscus medialis' if medial else 'meniscus lateralis', 'meniscus', vv, ff, [name],
                'Open C-shaped wedge measured from each tibial plateau footprint; base projected vertically to actual ready-bone triangles; tapered horns and closed caps.', metrics)
        radius, ulna = B('fore', 'radius'), B('ulna', 'ulna')
        E = np.array(rig[f'fore_{side}']['J'])
        patch('distal_humerus', 'el', hum, 'Хрящ блока и головки мыщелка', 'cartilago trochleae et capituli humeri',
              lambda v,n: np.minimum(E[1]+.018-v[:,1], .031-np.linalg.norm((v-E)[:,[0,2]], axis=1)),
              ['Хрящ блока и головки мыщелка'], 'Distal humeral articular region within 18 mm above elbow; epicondylar sides excluded.', .0007)
        ry = radius['v'][:,1].max()
        patch('radial_head', 'el', radius, 'Хрящ головки лучевой кости', 'cartilago capitis radii',
              lambda v,n: np.minimum(v[:,1]-ry+.011, n[:,1]*.02+.004),
              ['Хрящ головки лучевой кости'], 'Proximal radial head top and proximal rim, limited to 11 mm.', .00055)
        uy = ulna['v'][:,1].max()
        patch('ulnar_notch', 'el', ulna, 'Хрящ блоковидной вырезки локтевой кости', 'cartilago incisurae trochlearis ulnae',
              lambda v,n: np.minimum.reduce([v[:,1]-uy+.043, .038-np.linalg.norm(v-E,axis=1), n[:,2]*.025+.004]),
              [], 'Anterior-facing proximal ulnar trochlear notch; olecranon exterior excluded.', .00055)
        ry = radius['v'][:,1].min()
        patch('distal_radius', 'wr', radius, 'Хрящ лучезапястного сустава', 'cartilago radiocarpalis radii',
              lambda v,n: np.minimum(ry+.014-v[:,1], -n[:,1]*.02-.003),
              ['Хрящ лучезапястного сустава'], 'Distal radius concave carpal-facing face within 14 mm of lowest point.', .0005)
        for label, la, ru, lat in [('scaphoid','os scaphoideum','ладьевидной кости','ossis scaphoidei'),
                                  ('lunate','os lunatum','полулунной кости','ossis lunati'),
                                  ('triquetrum','os triquetrum','трёхгранной кости','ossis triquetri')]:
            b = B('hand', la); top = b['v'][:,1].max()
            patch('carpal_'+label, 'wr', b, 'Хрящ проксимальной поверхности '+ru, 'cartilago '+lat,
                  lambda v,n,top=top: np.minimum(v[:,1]-top+.007, n[:,1]*.02-.002),
                  [], 'Proximal carpal face within 7 mm of highest point, facing forearm.', .0004)
        talus, fibula = B('foot', 'talus'), B('shank', 'fibula')
        top = talus['v'][:,1].max()
        patch('talar_dome', 'an', talus, 'Хрящ блока таранной кости', 'cartilago trochleae tali',
              lambda v,n: np.minimum.reduce([v[:,1]-top+.017, n[:,1]*.025+.003, .008-v[:,2]]),
              ['Хрящ блока таранной кости'], 'Superior talar trochlea and upper shoulders; anterior talar head excluded.', .00065)
        bottom = tibia['v'][:,1].min()
        patch('tibial_plafond', 'an', tibia, 'Хрящ суставной поверхности большеберцовой кости', 'cartilago faciei articularis inferioris tibiae',
              lambda v,n: np.minimum(bottom+.035-v[:,1], -n[:,1]*.02-.003),
              [], 'Distal tibial plafond facing talar dome; excludes shaft and medial malleolus exterior.', .0006)
        bottom = fibula['v'][:,1].min()
        patch('fibular_facet', 'an', fibula, 'Хрящ суставной поверхности латеральной лодыжки', 'cartilago malleoli lateralis',
              lambda v,n: np.minimum(bottom+.034-v[:,1], -n[:,0]*sx*.02-.003),
              [], 'Medial-facing distal fibular articular facet within 34 mm of lowest point.', .00045)
    blob = bytearray()
    for p, (v, f) in zip(parts, arrays):
        lo, hi = v.min(axis=0), v.max(axis=0)
        q = np.rint((v-lo)/(hi-lo)*65535).astype(np.int64)
        dq = np.diff(q, axis=0, prepend=np.zeros((1,3), dtype=np.int64)) & 65535
        df = np.diff(f.reshape(-1), prepend=0) & 65535
        blob.extend(dq.astype('<u2').tobytes()); blob.extend(df.astype('<u2').tobytes())
        p.update(n=len(v), m=len(f), lo=lo.tolist(), hi=hi.tolist())
    data = dict(v=1, src='Reconstructed joint tissues on finished BodyParts3D-derived skeleton. No atlas cartilage/meniscus/labrum segmentation is claimed.',
                license='CC BY 4.0; bone basis: BodyParts3D, © DBCLS (https://lifesciencedb.jp/bp3d/). Reconstructed surfaces: Fysm 3D Anatomy.',
                basis=dict(file='skull/bones_bp3d.json', sha256=bone_hash, coordinates='rest-world metres; +X left, +Y up, +Z anterior'),
                parts=parts, data=base64.b64encode(gzip.compress(bytes(blob), compresslevel=9, mtime=0)).decode())
    coverage = dict(v=1, basis=data['basis'], parts=len(parts), triangles=sum(p['m'] for p in parts),
                    vertices=sum(p['n'] for p in parts), source='reconstructed', regions=reports,
                    splitPointTouchFans=sum(p['parameters'].get('splitPointTouchFans',0) for p in parts),
                    limitations=['Frozen skeleton has no physiological cartilage clearance or contact solver; small rest overlaps remain.',
                                 'Masks, thicknesses, labral cross sections and meniscal horn positions are educational reconstructions.',
                                 'Spine, fingers, sternoclavicular/acromioclavicular and other joints retain their existing schematic surfaces.'])
    return data, coverage


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path(__file__).with_name('joints.json'))
    args = parser.parse_args()
    data, coverage = build()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    packed = json.dumps(data, ensure_ascii=False, separators=(',', ':'))+'\n'
    temporary = args.out.with_suffix('.tmp')
    temporary.write_text(packed)
    temporary.replace(args.out)
    coverage['packedBytes'] = len(packed.encode())
    report = args.out.with_name('coverage.json')
    report.write_text(json.dumps(coverage, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: coverage[k] for k in ['parts','vertices','triangles','packedBytes']}))
