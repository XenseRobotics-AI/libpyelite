import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin
from libpyelite.collision import StationCollision
from libpyelite.sim import StationSimulation


class CollisionTests(unittest.TestCase):
    def synthetic(self, directory):
        # Two boxes on prismatic x joints; the native layer reads the URDF.
        joints = "".join(
            f'<link name="{name}"/><joint name="{name}_joint" type="prismatic"><parent link="world"/>'
            f'<child link="{name}"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
            for name in ("left", "right")
        )
        urdf = Path(directory) / "synthetic.urdf"
        urdf.write_text(f'<robot name="synthetic"><link name="world"/>{joints}</robot>')
        model = pin.buildModelFromUrdf(str(urdf))
        profile = {
            "schema": 1,
            "margin_m": 0.02,
            "detection_distance_m": 0.5,
            "gain": 0.2,
            "allowed_link_pairs": [],
            "shapes": [
                {
                    "name": name,
                    "frame": name,
                    "group": name,
                    "type": "box",
                    "size": [0.1] * 3,
                    "xyz": [0] * 3,
                    "rpy": [0] * 3,
                }
                for name in ("left", "right")
            ],
        }
        path = Path(directory) / "collision.json"
        profile["urdf_sha256"] = hashlib.sha256(urdf.read_bytes()).hexdigest()
        path.write_text(json.dumps(profile))
        return StationCollision(model, path, urdf)

    def test_normal_jacobian_sign_and_coupled_projection(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.synthetic(d)
            q = np.array([-0.1, 0.1])
            g_matrix, h = c.compute_qp_inequalities(q, [0, 1])
            np.testing.assert_allclose(g_matrix, [[1, -1]], atol=1e-9)
            np.testing.assert_allclose(h, [0.016], atol=1e-9)
            proposed = np.array([0.03, -0.03])
            delta, count = c.project_displacement(q, proposed, [0, 1], [-0.1] * 2, [0.1] * 2)
            np.testing.assert_allclose(delta, [0.008, -0.008], atol=1e-7)
            self.assertEqual(count, 1)
            self.assertTrue(c.check(q + delta).safe)
            eps = 1e-6
            numerical = (c.check(q + [eps, 0]).minimum_distance - c.check(q - [eps, 0]).minimum_distance) / (2 * eps)
            self.assertAlmostEqual(numerical, -g_matrix[0, 0], places=6)

    def test_path_crossing_and_infeasible_motion_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.synthetic(d)
            q = np.array([-0.2, 0.2])
            self.assertTrue(c.check(q).safe)
            self.assertTrue(c.check(-q).safe)
            self.assertFalse(c.check_segment(q, -q).safe)
            with self.assertRaises(RuntimeError):
                c.project_displacement([-0.1, 0.1], [0.03, -0.03], [0, 1], [0.03, -0.03], [0.03, -0.03])

    def test_station_baseline_and_environment_volumes(self):
        s = StationSimulation(collision_profile="auto")
        self.assertIsNone(s.collision_error)
        groups = {x["group"] for x in s.collision.shapes}
        self.assertEqual(groups, {"left", "right", "environment"})
        self.assertIn("worktable_120cm_base", {x["frame"] for x in s.collision.shapes})
        self.assertEqual({x["type"] for x in s.collision.shapes}, {"box", "capsule", "cylinder", "convex"})
        # Opposite base / shoulder must not disappear via the shared fixed world.
        frames = [{s.collision.shapes[i]["frame"], s.collision.shapes[j]["frame"]} for i, j in s.collision.pairs]
        self.assertIn({"left_base_link_inertia", "right_shoulder_link"}, frames)
        target = s.tcp_poses()["left"].copy()
        target[0, 3] += 0.005
        s.set_target("left", target)
        for _ in range(120):
            s.step()
        self.assertIsNone(s.collision_error)
        self.assertTrue(s.collision.check(s.viewer_q()).safe)
        self.assertLess(np.linalg.norm(s.tcp_poses()["left"][:3, 3] - target[:3, 3]), 0.001)

    def test_coupled_motion_limits_and_fault_latch(self):
        from libpyelite.control_profile import load_recipe

        root = Path(__file__).resolve().parents[3]
        settings = load_recipe(root / "recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml")
        settings.update(json.loads((Path(__file__).resolve().parents[1] / "examples/sim_safety.json").read_text()))
        sim = StationSimulation(settings=settings, collision_profile="auto")
        q = [sim.q.copy()] * 3
        home = sim.targets()["left"].copy()
        for i in range(120):
            target = home.copy()
            target[0, 3] += 0.02 if i < 60 else -0.01
            sim.set_target("left", target)
            sim.step()
            q.append(sim.q.copy())
        q = np.array(q)
        dt = sim.arms["left"].config.period
        arm = sim.arms["left"].config
        for n, limit in ((1, arm.max_joint_velocity), (2, arm.max_joint_acceleration), (3, arm.max_joint_jerk)):
            derivative = np.abs(np.diff(q, n=n, axis=0) / dt**n).reshape(-1, 2, 6)
            self.assertTrue(np.all(derivative <= np.asarray(limit) * (1 + 1e-6) + 1e-4))
        before = sim.q.copy()

        def infeasible(*args, **kwargs):
            raise RuntimeError("injected infeasible constraints")

        guard = sim.guard
        sim.guard = type("FailingGuard", (), {"tick": staticmethod(infeasible), "reset": guard.reset})()
        with self.assertRaises(RuntimeError):
            sim.step()
        np.testing.assert_array_equal(sim.q, before)
        with self.assertRaises(RuntimeError):
            sim.step()
        sim.reset(before)
        self.assertIsNone(sim.collision_error)

    def test_soft_motion_bounds_never_make_the_barrier_infeasible(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.synthetic(d)
            q = np.array([-0.1, 0.1])
            # The acceleration bounds demand closing faster than the barrier allows.
            delta, _ = c.project_displacement(q, [0.03, -0.03], [0, 1], [-0.1] * 2, [0.1] * 2, [0.03, -0.1], [0.1, -0.03])
            g_matrix, h = c.compute_qp_inequalities(q, [0, 1])
            self.assertTrue(np.all(g_matrix @ delta <= h + 1e-8))

    def test_blocked_target_holds_the_arm_without_drift(self):
        from libpyelite.control_profile import load_recipe

        root = Path(__file__).resolve().parents[3]
        settings = load_recipe(root / "recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml")
        settings.update(json.loads((Path(__file__).resolve().parents[1] / "examples/sim_safety.json").read_text()))
        sim = StationSimulation(settings=settings, collision_profile="auto")
        home = sim.targets()["left"].copy()
        errors = []
        for k in range(150):  # 12 cm down: well into the table
            target = home.copy()
            target[:3, 3] += np.array([0.0, -0.15, -0.12]) * min(1.0, k / 31)
            sim.set_target("left", target)
            for _ in range(4):
                sim.step()
            errors.append(np.linalg.norm(sim.tcp_poses()["left"][:3, 3] - target[:3, 3]))
        self.assertIsNone(sim.collision_error)
        self.assertTrue(sim.collision.check(sim.viewer_q()).safe)
        # Held where the table stopped it: no slide along it (it once drifted > 0.3 m).
        self.assertLess(max(errors[-60:]) - min(errors[-60:]), 0.002)

    def _station(self):
        from libpyelite.control_profile import load_recipe

        root = Path(__file__).resolve().parents[3]
        settings = load_recipe(root / "recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml")
        settings.update(json.loads((Path(__file__).resolve().parents[1] / "examples/sim_safety.json").read_text()))
        return StationSimulation(settings=settings, collision_profile="auto")

    def test_joint_target_into_the_table_stops_at_the_margin(self):
        # A joint slider used to reset each value and check only the end
        # configuration, so dragging on jumped through the table (J2: 31 deg).
        sim = self._station()
        home = sim.arms["left"].q.copy()
        worst = np.inf
        for k in range(1, 190):
            q = home.copy()
            q[1] += np.radians(0.5) * k
            sim.set_joint_target("left", q)
            for _ in range(4):
                sim.step()
            worst = min(worst, sim.last_collision.clearance)
        moved = sim.arms["left"].q - home
        self.assertIsNone(sim.collision_error)
        self.assertGreaterEqual(worst, 0.0)
        self.assertLess(np.degrees(moved[1]), 15.0)  # stopped at the table, far short of 94 deg
        self.assertLess(np.abs(np.delete(moved, 1)).max(), 1e-6)  # along the commanded joint only

    def test_ik_drag_past_contact_holds_the_arm(self):
        # The leftover IK step used to swing elbow and wrist by up to 84 deg.
        sim = self._station()
        home = sim.targets()["left"].copy()
        arm = sim.arms["left"]
        q, contact = [], None
        for k in range(250):
            target = home.copy()
            target[2, 3] -= 0.002 * k
            sim.set_target("left", target)
            for _ in range(4):
                held = sim.step()["left"]["collision_hold"]
            if contact is None and held:
                contact = k
            q.append(arm.q.copy())
        self.assertIsNotNone(contact)
        after = np.array(q[contact + 10 :])
        self.assertIsNone(sim.collision_error)
        self.assertTrue(sim.collision.check(sim.viewer_q()).safe)
        self.assertLess(np.degrees(np.abs(after[-1] - after[0]).max()), 0.5)
        # Dragging back above the table releases the hold and tracking resumes.
        deep = sim.targets()["left"].copy()
        goal = home.copy()
        goal[1, 3] -= 0.05
        for k in range(160):
            target = deep.copy()
            target[:3, 3] += (goal[:3, 3] - deep[:3, 3]) * min(1.0, k / 60)
            sim.set_target("left", target)
            for _ in range(4):
                sample = sim.step()["left"]
        self.assertFalse(sample["collision_hold"])
        self.assertLess(sample["position_error_m"], 1e-3)

    def test_native_layer_matches_brute_force_distances(self):
        # Broad phase, cached functors and warm starts must not change any answer:
        # compare against a fresh, unpruned Coal query on every pair.
        import coal

        def convex(shape):
            vertices, faces = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
            for vertex in shape["vertices"]:
                vertices.append(np.asarray(vertex))
            for face in shape["faces"]:
                faces.append(coal.Triangle(*face))
            return coal.Convex(vertices, faces)

        s = StationSimulation(collision_profile="auto")
        c = s.collision
        rng = np.random.default_rng(3)
        indices = s.indices["left"] + s.indices["right"]
        for _ in range(20):
            q = s.viewer_q()
            q[indices] += rng.normal(0, 0.3, 12)
            placements = c.placements(q)
            objects = []
            for shape, placement in zip(c.shapes, placements, strict=True):
                geometry = {
                    "box": lambda: coal.Box(*shape["size"]),
                    "capsule": lambda: coal.Capsule(shape["radius"], shape["length"]),
                    "cylinder": lambda: coal.Cylinder(shape["radius"], shape["length"]),
                    "convex": lambda: convex(shape),
                }[shape["type"]]()
                objects.append(coal.CollisionObject(geometry, coal.Transform3s(placement.rotation, placement.translation)))
            request = coal.DistanceRequest()
            request.enable_signed_distance = True
            clearance = min(
                coal.distance(objects[i], objects[j], request, coal.DistanceResult()) - margin
                for (i, j), margin in zip(c.pairs, c.margins, strict=True)
            )
            self.assertAlmostEqual(c.check(q).clearance, clearance, places=9)

    def test_motion_bound_rejects_a_step_through_an_obstacle(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.synthetic(d)
            q = np.array([-0.1, 0.1])
            c.linearize(q, [0, 1])
            self.assertTrue(c.verify_step(q, q + [0.001, -0.001]).safe)  # proven by the bound alone
            self.assertFalse(c.verify_step(q, q + [0.2, -0.2]).safe)  # passes through: sampled and rejected

    def test_guard_treats_an_arm_without_planner_as_a_fixed_obstacle(self):
        # The hardware coordinator passes no planner for an arm whose servo is
        # stopped (MoveJ, start-up): it must stay put and still constrain the other.
        sim = self._station()
        left, right = sim.arms["left"], sim.arms["right"]
        fixed = right.q.copy()
        self.assertEqual(sim.guard.tick(None, None, left.q, fixed, 0.0).q[1], list(fixed))
        target = sim.targets()["left"].copy()
        target[2, 3] -= 0.3  # into the table
        sim.set_target("left", target)
        for _ in range(400):
            left.submit()
            result = sim.guard.tick(left.planner, None, left.q, fixed, left.time)
            left.q = np.asarray(result.q[0])
            left.time += left.config.period
            np.testing.assert_array_equal(result.q[1], fixed)
        self.assertTrue(sim.collision.check(sim.viewer_q()).safe)
        self.assertTrue(result.held[0])

    def test_stale_model_and_missing_frame_fail_closed(self):
        s = StationSimulation()
        with tempfile.TemporaryDirectory() as d:
            profile = json.loads((s.assembly.root / "collision.json").read_text())
            path = Path(d) / "collision.json"
            profile["urdf_sha256"] = "invalid"
            path.write_text(json.dumps(profile))
            with self.assertRaises(ValueError):
                StationCollision(s.model, path, s.urdf_path)
            profile["shapes"][0]["frame"] = "missing"
            path.write_text(json.dumps(profile))
            with self.assertRaises(ValueError):
                StationCollision(s.model, path)


if __name__ == "__main__":
    unittest.main()
