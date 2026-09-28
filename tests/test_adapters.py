"""Execute the adapter methods without importing optional camera/gripper SDKs."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[3]
ROBOTS = ROOT / "src/lerobot/robots"


class Rotation:
    def __init__(self, matrix):
        self.matrix = matrix

    @classmethod
    def from_rotvec(cls, v):
        return cls(pin.exp3(np.asarray(v)))

    @classmethod
    def from_matrix(cls, m):
        return cls(m)

    def as_matrix(self):
        return self.matrix

    def as_rotvec(self):
        return pin.log3(self.matrix)


def extract(path, class_name, methods, namespace):
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    selected = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    assert len(selected) == len(methods)
    tree = ast.Module(body=selected, type_ignores=[])
    exec(compile(tree, str(path), "exec"), namespace)
    return {name: namespace[name] for name in methods}


class AdapterTests(unittest.TestCase):
    def test_preserved_start_poses_and_mount_roundtrip(self):
        config = ROBOTS / "bi_elite_cs66_rt/config_bi_elite_cs66_rt.py"
        names = {"_LEFT_START_POSE", "_RIGHT_START_POSE", "_PRESETS"}
        assignments = [n for n in ast.parse(config.read_text()).body
                       if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                       and n.targets[0].id in names]
        assignments += [n for n in ast.parse(config.read_text()).body
                        if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
                        and n.target.id == "_PRESETS"]
        ns = {}
        exec(compile(ast.Module(body=assignments, type_ignores=[]), str(config), "exec"), ns)
        self.assertEqual(ns["_LEFT_START_POSE"], [-1.3963, -0.5236, 1.5708, -0.5236, 1.5708, -0.7854])
        self.assertEqual(ns["_RIGHT_START_POSE"], [-1.7453, -2.6180, -1.5708, -2.6180, -1.5708, -0.7854])
        methods = extract(
            ROBOTS / "bi_elite_cs66_rt/bi_elite_cs66_rt.py", "BiEliteCS66RT",
            ["_base_pose6_to_world", "_world_pose6_to_base"],
            {"np": np, "Rotation": Rotation},
        )
        for preset in ns["_PRESETS"].values():
            obj = SimpleNamespace(_R_wb={s: np.array(preset[s+"_world_R"]) for s in ("left", "right")})
            for side in ("left", "right"):
                pose = np.array([0.2, -0.1, 0.4, 0.5, -0.3, 0.2])
                world = methods["_base_pose6_to_world"](obj, side, pose)
                restored = methods["_world_pose6_to_base"](obj, side, world)
                np.testing.assert_allclose(restored, pose, atol=1e-7)

    def test_bi_target_goes_once_to_native_and_reset_gates_arm(self):
        from libpyelite import _native
        mode = SimpleNamespace(CARTESIAN_SERVO="cartesian")
        methods = extract(
            ROBOTS / "bi_elite_cs66_rt/bi_elite_cs66_rt.py", "BiEliteCS66RT",
            ["_send_arm_action"],
            {"np": np, "Any": object, "BiEliteCS66RTControlMode": mode,
             "_pose6_to_matrix": lambda p: np.array(_native.pose_from_rotvec(p)).reshape(4, 4)},
        )
        status = SimpleNamespace(reset_active=False)
        driver, gripper = Mock(), Mock()
        driver.status.return_value = status
        pose = np.array([0.1, 0.2, 0.3, 0., 0., 0.])
        obj = SimpleNamespace(
            _driver={"left": driver}, _gripper={"left": gripper},
            _gripper_key={"left": "left_gripper.pos"},
            config=SimpleNamespace(control_mode="cartesian"),
            _raise_servo_error_if_any=Mock(), _cartesian_action_to_tcp_pose=Mock(return_value=pose),
            _trace_send_action=Mock(), _last_tcp_command={}, _external_command_received={},
            _base_pose6_to_world=lambda side, p: p,
            _tcp_rotvec_to_feature_values=lambda side, p: {"left_tcp.x": float(p[0])},
        )
        action = {"left_gripper.pos": 0.5}
        sent = {}
        methods["_send_arm_action"](obj, "left", action, sent)
        driver.submit_target.assert_called_once()
        driver.writeServoj.assert_not_called()
        np.testing.assert_allclose(driver.submit_target.call_args.args[0][:3, 3], pose[:3])
        self.assertEqual(sent["left_gripper.pos"], 0.5)
        status.reset_active = True
        methods["_send_arm_action"](obj, "left", action, {})
        driver.submit_target.assert_called_once()
        self.assertEqual(gripper.set_gripper_position.call_count, 2)

    def test_no_python_high_rate_loop_remains(self):
        for name, cls in (("elite_cs66_rt", "EliteCS66RT"), ("bi_elite_cs66_rt", "BiEliteCS66RT")):
            source = (ROBOTS / name / (name+".py")).read_text()
            tree = ast.parse(source)
            node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
            self.assertNotIn("_servo_loop", [n.name for n in node.body if isinstance(n, ast.FunctionDef)])
            self.assertNotIn(".writeServoj(", source)
            self.assertNotIn("setCurrentThreadFiFoScheduling", source)


if __name__ == "__main__":
    unittest.main()
