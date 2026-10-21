"""Convex runtime validation and reproducible full-triangle finger coverage."""

import copy
import hashlib
import importlib.util
import itertools
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import coal
import numpy as np
import pinocchio as pin
from libpyelite import _native
from libpyelite.collision import StationCollision

ROOT = Path(__file__).resolve().parents[1]


def generator():
    spec = importlib.util.spec_from_file_location("collision_generator", ROOT / "scripts/generate_collision_profile.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cube(offset=(0, 0, 0)):
    vertices = np.array(list(itertools.product((-.05, .05), repeat=3))) + offset
    # Fixed outward triangles; no runtime/scipy hull construction is required.
    faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
             [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
             [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return {"type": "convex", "vertices": vertices.tolist(), "faces": faces,
            "name": "left", "frame": "left", "group": "left", "xyz": [0, 0, 0], "rpy": [0, 0, 0]}


def native_shape(shape):
    result = _native.CollisionShape()
    result.frame, result.type = shape["frame"], 3
    result.vertices, result.faces = shape["vertices"], shape["faces"]
    result.xyz, result.rpy = shape["xyz"], shape["rpy"]
    return result


class ConvexRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.urdf = Path(self.directory.name) / "shapes.urdf"
        joints = "".join(
            f'<link name="{name}"/><joint name="{name}_joint" type="prismatic">'
            f'<parent link="world"/><child link="{name}"/><axis xyz="1 0 0"/>'
            '<limit lower="-5" upper="5" effort="1" velocity="1"/></joint>'
            for name in ("left", "right")
        )
        self.urdf.write_text(f'<robot name="shapes"><link name="world"/>{joints}</robot>')
        self.model = pin.buildModelFromUrdf(str(self.urdf))

    def collision(self, shape):
        other = {"type": "box", "size": [.1] * 3, "name": "right", "frame": "right",
                 "group": "right", "xyz": [.5, 0, 0], "rpy": [0, 0, 0]}
        vertices = np.asarray(shape["vertices"])
        center = (pin.rpy.rpyToMatrix(np.asarray(shape["rpy"])) @ vertices.mean(0)
                  if np.isfinite(vertices).all() else np.zeros(3))
        other["xyz"] = (center + [.5, 0, 0]).tolist()
        profile = {"schema": 1, "margin_m": .01, "self_margin_m": .0005, "detection_distance_m": .05,
                   "urdf_sha256": hashlib.sha256(self.urdf.read_bytes()).hexdigest(),
                   "allowed_link_pairs": [], "shapes": [shape, other]}
        path = Path(self.directory.name) / "collision.json"
        path.write_text(json.dumps(profile))
        return StationCollision(self.model, path, self.urdf)

    def test_analytic_distance_penetration_rotated_offcentre_and_cutoff(self):
        for offset, yaw in (((0, 0, 0), 0), ((.4, -.2, .1), np.pi / 2), ((.4, -.2, .1), np.pi / 4)):
            with self.subTest(offset=offset, yaw=yaw):
                shape = cube(offset)
                shape["rpy"][2] = yaw
                collision = self.collision(shape)
                extent = .05 * (abs(np.sin(yaw)) + abs(np.cos(yaw)))
                expected = .5 - .05 - extent
                self.assertAlmostEqual(collision.check([0, 0]).minimum_distance, expected, places=8)
                self.assertAlmostEqual(collision.check([0, -expected + .015], cutoff=.02).minimum_distance, .015, places=8)
                self.assertAlmostEqual(collision.check([0, -expected - .02]).minimum_distance, -.02, places=8)
                np.testing.assert_allclose(collision.placements([0, 0])[0].translation, 0)

    def test_binding_roundtrip_and_invalid_native_exteriors(self):
        original = cube()
        shape = native_shape(original)
        self.assertEqual(shape.vertices, original["vertices"])
        self.assertEqual(shape.faces, original["faces"])
        bad = []
        for key, value in (("vertices", original["vertices"] + [[8, 0, 0]]),
                           ("vertices", [[0, 0, 0]] * 8),
                           ("faces", original["faces"][:-1]),
                           ("faces", original["faces"] + [original["faces"][0]])):
            candidate = copy.deepcopy(original)
            candidate[key] = value
            bad.append(candidate)
        for change in ("winding", "index", "nonconvex", "nan", "degenerate", "transform"):
            candidate = copy.deepcopy(original)
            if change == "winding":
                candidate["faces"][0].reverse()
            elif change == "index":
                candidate["faces"][0][0] = 1234
            elif change == "nonconvex":
                candidate["vertices"][0] = [0, 0, 0]
            elif change == "nan":
                candidate["vertices"][0][0] = float("nan")
            elif change == "degenerate":
                for v in candidate["vertices"]:
                    v[2] = 0
            else:
                candidate["xyz"][0] = float("inf")
            bad.append(candidate)
        for i, candidate in enumerate(bad):
            with self.subTest(case=i), self.assertRaises(ValueError):
                _native.StationCollisionModel(str(self.urdf), [native_shape(candidate)], [], [], .05, .2, .001)

    def test_profile_rejects_noninteger_faces_and_nonfinite_vertices(self):
        for value in (1.5, float("nan"), True):
            candidate = cube()
            candidate["faces"] = [[value] * 3] * 12
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.collision(candidate)
        candidate = cube()
        candidate["vertices"][0][0] = float("inf")
        with self.assertRaises(ValueError):
            self.collision(candidate)

    def test_far_offcentre_geometry_remains_in_broadphase(self):
        shape = cube((2, -.3, .4))
        shape["rpy"] = [.2, -.4, .8]
        collision = self.collision(shape)
        points = coal.StdVec_Vec3s()
        faces = coal.StdVec_Triangle()
        for v in shape["vertices"]:
            points.append(np.asarray(v))
        for f in shape["faces"]:
            faces.append(coal.Triangle(*f))
        convex = coal.Convex(points, faces)
        request = coal.DistanceRequest()
        request.enable_signed_distance = True
        for displacement in np.linspace(-.7, .3, 31):
            placements = collision.placements([0, displacement])
            expected = coal.distance(
                convex, coal.Transform3s(placements[0].rotation, placements[0].translation),
                coal.Box(.1, .1, .1), coal.Transform3s(placements[1].rotation, placements[1].translation),
                request, coal.DistanceResult())
            self.assertAlmostEqual(collision.check([0, displacement]).minimum_distance, expected, places=8)
            if expected < .05:
                self.assertAlmostEqual(collision.check([0, displacement], cutoff=.05).minimum_distance, expected, places=8)

    def test_viewer_uses_exact_local_vertices_faces_and_native_placement(self):
        from libpyelite.sim import MeshcatRobotView

        shape = cube((.4, .2, -.1))
        shape["rpy"] = [.2, -.4, .8]
        collision = self.collision(shape)
        nodes = {}

        class Node:
            def __init__(self, path=""):
                self.path = path

            def __getitem__(self, path):
                key = self.path + "/" + path
                return nodes.setdefault(key, Node(key))

            def set_object(self, geometry, material=None):
                self.geometry = geometry

            def set_transform(self, transform):
                self.transform = transform

            def set_property(self, *args):
                pass

            def url(self):
                return "offline-viewer"

        visualizer = MagicMock()
        visualizer.viewer = Node()
        simulation = SimpleNamespace(model=self.model, urdf_path=self.urdf, collision=collision,
                                     viewer_q=lambda: np.zeros(2), targets=lambda: {}, tcp_poses=lambda: {})
        with patch("pinocchio.visualize.MeshcatVisualizer", return_value=visualizer):
            view = MeshcatRobotView(simulation, open_browser=False)
            view.set_collision_visible(True)
            view.update(simulation)
        geometry = nodes["/collision/0"].geometry
        np.testing.assert_allclose(geometry.vertices, shape["vertices"], atol=1e-8)
        np.testing.assert_array_equal(geometry.faces, shape["faces"])
        geometry.lower({})  # serialize exactly the geometry sent to Meshcat
        np.testing.assert_allclose(nodes["/collision/0"].transform, collision.placements([0, 0])[0].homogeneous)


class FingerConvexCoverageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.generator = generator()
        cls.station = ROOT / "libpyelite/assets/stations/diagonal-08"
        cls.urdf = cls.station / "assembly_taccap_identified.urdf"
        cls.profile = json.loads((cls.station / "collision.json").read_text())

    def test_full_surface_padding_partition_determinism_and_table_planes(self):
        from scipy.spatial import ConvexHull

        for link in ET.parse(self.urdf).findall("link"):
            name = link.get("name")
            if not name.startswith("left_") or "finger_link" not in name:
                continue
            with self.subTest(finger=name):
                mesh = link.find("collision/geometry/mesh")
                origin = link.find("collision/origin")
                tris = self.generator.triangles((self.urdf.parent / mesh.get("filename")).resolve())
                tris *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                rotation = pin.rpy.rpyToMatrix(np.fromstring(origin.get("rpy"), sep=" "))
                tris = tris @ rotation.T + np.fromstring(origin.get("xyz"), sep=" ")
                shapes = [s for s in self.profile["shapes"] if s["frame"] == name]
                generated = self.generator.finger_convexes(tris)
                for stored, repeated in zip(shapes, generated, strict=True):
                    self.assertEqual({k: stored[k] for k in repeated}, repeated)
                self.assertLessEqual(len(shapes), 6)
                self.assertLessEqual(max(len(s["vertices"]) for s in shapes), 32)
                self.assertTrue(all(s["type"] == "convex" for s in shapes))
                # Axis cell union covers the entire original AABB, not only the
                # sampled vertices. Each cell's clipped polygons then cover it.
                cells = np.array([s["source_cell"] for s in shapes])
                edges = [np.unique(cells[:, :, axis]) for axis in range(3)]
                for interval in itertools.product(*(list(zip(e[:-1], e[1:])) for e in edges)):
                    midpoint = np.array(interval).mean(1)
                    self.assertTrue(np.any(np.all((midpoint >= cells[:, 0]) & (midpoint <= cells[:, 1]), axis=1)))
                for shape in shapes:
                    points = self.generator.clipped_points(tris, *np.array(shape["source_cell"]))
                    self.assertGreaterEqual(self.generator.verify_convex_coverage(points, shape), .002 - 1e-10)
                old_boxes = self.generator.slab_boxes(tris)
                old_vertices = np.concatenate([
                    np.array(list(itertools.product(*zip(lo - .002, hi + .002)))) for lo, hi in old_boxes
                ])
                old_hull = ConvexHull(old_vertices)
                vertices = np.concatenate([s["vertices"] for s in shapes])
                # This inclusion proves the plane/table assertion for ALL
                # orientations; explicit orientations below check the distances.
                self.assertLessEqual((vertices @ old_hull.equations[:, :3].T + old_hull.equations[:, 3]).max(), 1e-10)
                directions = np.vstack((np.eye(3), -np.eye(3), np.random.default_rng(4).normal(size=(256, 3))))
                directions /= np.linalg.norm(directions, axis=1)[:, None]
                support = (directions @ vertices.T).max(1)
                old_support = (directions @ old_vertices.T).max(1)
                self.assertTrue(np.all(support <= old_support + 1e-10))
                self.assertGreater(np.max(old_support - support), .005)
                # Actual Coal polytope/table distance at multiple orientations,
                # for both fingers, agrees with the exact planar support bound.
                polytopes = []
                for shape in shapes:
                    points, faces = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
                    for vertex in shape["vertices"]:
                        points.append(np.asarray(vertex))
                    for face in shape["faces"]:
                        faces.append(coal.Triangle(*face))
                    polytopes.append(coal.Convex(points, faces))
                table, table_tf = coal.Box(10, 10, .1), coal.Transform3s(np.eye(3), np.array([0, 0, -.05]))
                request = coal.DistanceRequest()
                request.enable_signed_distance = True
                for angles in ((0, 0, 0), (0, .47, 0), (.47, 0, .3), (0, 1.05, 0), (0, np.pi, 0)):
                    rotation = pin.rpy.rpyToMatrix(np.asarray(angles))
                    transform = coal.Transform3s(rotation, np.array([0, 0, .3]))
                    distance = min(coal.distance(p, transform, table, table_tf, request, coal.DistanceResult())
                                   for p in polytopes)
                    direction = -rotation[2]
                    expected = .3 - np.max(vertices @ direction)
                    self.assertAlmostEqual(distance, expected, places=7)
                    self.assertGreaterEqual(distance, .3 - np.max(old_vertices @ direction) - 1e-8)
                    self.assertLessEqual(distance, .3 - np.max(tris.reshape(-1, 3) @ direction) - .002 + 1e-8)
                # A single global hull would fill the concavity. Keep separated
                # pieces with strictly smaller total volume than that hull.
                volume = sum(ConvexHull(s["vertices"]).volume for s in shapes)
                self.assertLess(volume, ConvexHull(vertices).volume * .9)

    def test_both_profiles_keep_margins_exemptions_and_reduce_pairs(self):
        for station in ("diagonal-07", "diagonal-08"):
            root = self.station.parent / station
            profile = json.loads((root / "collision.json").read_text())
            urdf = next(root.glob("assembly*.urdf"))
            collision = StationCollision(pin.buildModelFromUrdf(str(urdf)), root / "collision.json", urdf)
            self.assertEqual(profile["margin_m"], .01)
            self.assertEqual(profile["self_margin_m"], .0005)
            self.assertEqual(profile["padding_m"], .002)
            self.assertLess(len(collision.shapes), 370)
            self.assertLess(len(collision.pairs), 16000)
            self.assertEqual(sum(s["type"] == "convex" for s in collision.shapes), 20)


if __name__ == "__main__":
    unittest.main()
