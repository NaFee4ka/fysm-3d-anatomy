"""Geometric invariants for atlas seam repair and conforming refinement."""
import unittest
import numpy as np
import trimesh
from detail import detail_target, weld_positions, refine_edges


class AtlasDetailTests(unittest.TestCase):
    def test_source_seams_join_without_moving_the_surface(self):
        cube = trimesh.creation.box()
        split_vertices = np.asarray(cube.vertices)[np.asarray(cube.faces)].reshape(-1, 3)
        split_faces = np.arange(len(split_vertices)).reshape(-1, 3)
        vertices, faces, merged = weld_positions(split_vertices, split_faces)
        np.testing.assert_array_equal(vertices[faces], split_vertices[split_faces])
        self.assertGreater(merged, 0)
        self.assertEqual(len(faces), len(split_faces))
        self.assertTrue(trimesh.Trimesh(vertices=vertices, faces=faces, process=False).is_watertight)

    def test_edge_refinement_is_conforming_and_preserves_orientation(self):
        vertices = np.array([[0., 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
        faces = np.array([[0, 1, 2], [0, 2, 3]])
        source, refined, placed, report = refine_edges(vertices, faces, lambda p: p.copy(), max_edge=.2)
        np.testing.assert_array_equal(source, placed)
        tri = placed[refined]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        self.assertTrue((normals[:, 2] > 0).all())
        self.assertAlmostEqual(normals[:, 2].sum() / 2, 1)
        edges = np.sort(np.concatenate([refined[:, [0, 1]], refined[:, [1, 2]], refined[:, [2, 0]]]), axis=1)
        unique, counts = np.unique(edges, axis=0, return_counts=True)
        self.assertTrue((counts <= 2).all())
        for edge in unique[counts == 1]:
            p = placed[edge]
            self.assertTrue(any(np.all(p[:, axis] == bound) for axis in (0, 1) for bound in (0, 1)))
        self.assertLessEqual(report['maxEdgeMm'], 200.001)

    def test_registration_is_resampled_between_old_lod_vertices(self):
        vertices = np.array([[0., 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
        faces = np.array([[0, 1, 2], [0, 2, 3]])

        def curved_registration(p):
            out = p.copy()
            out[:, 2] = .1 * np.sin(np.pi * p[:, 0]) * np.sin(np.pi * p[:, 1])
            return out

        source, refined, placed, report = refine_edges(vertices, faces, curved_registration, max_edge=.2)
        np.testing.assert_allclose(placed, curved_registration(source), atol=1e-15)
        self.assertGreater(placed[:, 2].max(), .09)
        self.assertTrue((source[:, 2] == 0).all())
        self.assertLessEqual(report['maxEdgeMm'], 200.001)

    def test_long_thin_parts_receive_more_detail_than_small_parts(self):
        square = np.array([[0., 0, 0], [.05, 0, 0], [.05, .05, 0], [0, .05, 0]])
        faces = np.array([[0, 1, 2], [0, 2, 3]])
        long = square * np.array([10, .2, 1])
        self.assertEqual(detail_target(square, faces), 800)
        self.assertGreater(detail_target(long, faces), 2000)
        self.assertLessEqual(detail_target(long, faces), 3200)


if __name__ == '__main__':
    unittest.main()
