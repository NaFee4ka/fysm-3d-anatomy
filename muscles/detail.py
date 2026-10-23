"""Geometry-only detail policy for long/wide atlas muscles of the torso.

Subdivision samples the existing surface and the registration field; it never
changes the rig, force paths, muscle parameters or source attribution.
"""
import numpy as np
import trimesh

TORSO_UNITS = frozenset({
    'trapUp', 'trapMid', 'trapLow', 'pecMaj', 'pecMin', 'serratus', 'rhomb',
    'scm', 'scalene', 'splenius', 'semispinalis', 'erector', 'extObl',
})


def detail_target(vertices, faces, base=800):
    """Allocate by physical surface area and length, capped per source part."""
    tri = vertices[faces]
    area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1).sum() * .5
    extent = np.ptp(vertices, axis=0).max()
    by_area = area / (.008 ** 2 * np.sqrt(3) / 4)
    by_length = base * max(1, extent / .16)
    return int(np.clip(np.ceil(max(by_area, by_length)), base, 3200))


def weld_positions(vertices, faces):
    """Join only identical source positions, ignoring OBJ normal/UV seams."""
    unique, index, inverse = np.unique(vertices, axis=0, return_index=True, return_inverse=True)
    return vertices[index], inverse[faces], len(vertices) - len(unique)


def refine_edges(source, faces, warp, max_edge=.025, max_iterations=8):
    """Conforming edge splits: adjacent faces always share the same midpoint.

    New positions are on the source triangles. The registration is evaluated
    again there, so nonlinear bone blending is sampled between original LOD
    vertices instead of stretching one flat face over a whole body segment.
    """
    placed = warp(source)
    original_count = len(faces)
    for iteration in range(max_iterations):
        edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        edges = np.sort(edges, axis=1)
        unique, inverse = np.unique(edges, axis=0, return_inverse=True)
        length = np.linalg.norm(placed[unique[:, 0]] - placed[unique[:, 1]], axis=1)
        selected = length > max_edge
        if not selected.any():
            break
        midpoint_ids = np.full(len(unique), -1, np.int32)
        midpoint_ids[selected] = len(source) + np.arange(selected.sum())
        midpoint = source[unique[selected]].mean(axis=1)
        source = np.concatenate([source, midpoint])
        placed = np.concatenate([placed, warp(midpoint)])
        split = midpoint_ids[inverse].reshape(3, -1).T
        output = []
        for face, middle in zip(faces, split):
            a, b, c = map(int, face)
            ab, bc, ca = map(int, middle)
            mask = (ab >= 0) + 2 * (bc >= 0) + 4 * (ca >= 0)
            if mask == 0:
                output.append([a, b, c])
            elif mask == 1:
                output.extend([[a, ab, c], [ab, b, c]])
            elif mask == 2:
                output.extend([[b, bc, a], [bc, c, a]])
            elif mask == 4:
                output.extend([[c, ca, b], [ca, a, b]])
            elif mask == 3:
                output.extend([[ab, b, bc], [a, ab, c], [ab, bc, c]])
            elif mask == 6:
                output.extend([[bc, c, ca], [b, bc, a], [bc, ca, a]])
            elif mask == 5:
                output.extend([[ca, a, ab], [c, ca, b], [ca, ab, b]])
            else:
                output.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
        faces = np.asarray(output, np.int32)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    maximum = np.linalg.norm(placed[edges[:, 0]] - placed[edges[:, 1]], axis=1).max()
    if maximum > max_edge * 1.001:
        raise ValueError(f'Atlas detail refinement did not converge: longest edge {maximum}')
    return source, faces, placed, dict(maxEdgeMm=float(maximum * 1000),
                                      edgeLimitMm=max_edge * 1000,
                                      trianglesBeforeEdgeRefinement=original_count,
                                      addedTriangles=len(faces) - original_count)


def sample_source(vertices, faces, count=768):
    """Deterministic samples distributed by triangle area, for shape audits."""
    tri = vertices[faces]
    area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    cdf = np.cumsum(area)
    selected = np.searchsorted(cdf, (np.arange(count) + .5) / count * cdf[-1])
    # Fixed low-discrepancy barycentric positions, independent of mesh density.
    u = np.sqrt((np.arange(count) * .6180339887498949 + .37) % 1)
    v = (np.arange(count) * .4142135623730951 + .19) % 1
    t = tri[selected]
    return (1 - u[:, None]) * t[:, 0] + u[:, None] * (1 - v[:, None]) * t[:, 1] + (u * v)[:, None] * t[:, 2]


def shape_error(reference_points, vertices, faces):
    """Sampled raw-source-to-LOD surface distance, in millimetres (not clinical)."""
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    _, distance, _ = trimesh.proximity.closest_point(mesh, reference_points)
    return dict(samples=len(distance), meanMm=float(distance.mean() * 1000),
                p95Mm=float(np.percentile(distance, 95) * 1000),
                maxMm=float(distance.max() * 1000))
