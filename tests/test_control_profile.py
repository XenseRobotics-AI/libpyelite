"""Native option parity between the real adapter and the offline station."""

import ast
import json
import unittest
from contextlib import suppress
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from libpyelite import EliteDriverConfig
from libpyelite.assembly import Assembly
from libpyelite.control_profile import describe_config, load_recipe, native_config

ROOT = Path(__file__).resolve().parents[3]


class ControlProfileTests(unittest.TestCase):
    def test_servo_failure_tolerance_maps_to_native_sender(self):
        assembly = Assembly("diagonal-07")
        configured = native_config({"servo_failure_tolerance_ticks": 40}, "left", assembly)
        default = native_config({}, "left", assembly)
        self.assertEqual(configured.max_consecutive_write_failures, 40)
        self.assertEqual(default.max_consecutive_write_failures, 250)
        self.assertEqual(EliteDriverConfig().max_consecutive_write_failures, 1)

    def test_real_adapter_and_simulator_use_same_native_parameters(self):
        config_path = ROOT / "src/lerobot/robots/bi_elite_cs66_rt/config_bi_elite_cs66_rt.py"
        tree = ast.parse(config_path.read_text())
        config = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "BiEliteCS66RTConfig")
        defaults = {}
        for n in config.body:
            if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name):
                with suppress(ValueError, TypeError):
                    defaults[n.target.id] = ast.literal_eval(n.value)
        path = ROOT / "src/lerobot/robots/elite_cs66_rt/elite_cs66_rt.py"
        fn = next(
            n
            for n in ast.parse(path.read_text()).body
            if isinstance(n, ast.FunctionDef) and n.name == "_configure_native_driver"
        )
        namespace = {}
        exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec"), namespace)
        for name in ("diagonal-07", "diagonal-08"):
            settings = load_recipe(ROOT / f"recipes/teleop/bi_elite_cs66_rt/{name}-taccap.yaml")
            # Exercise real overrides, not just identical defaults.
            for overrides in (
                {},
                {
                    "servoj_time": 0.008,
                    "native_ik_max_iterations": 4,
                    "native_joint_velocity_scale": 0.3,
                    "max_lin_speed": 0.12,
                    "native_joint_motion_limits": True,
                    "native_max_joint_velocity": [0.4] * 6,
                    "native_max_joint_acceleration": [2.0] * 6,
                    "native_max_joint_jerk": [10.0] * 6,
                },
            ):
                requested = settings | overrides
                effective = defaults | requested
                assembly = Assembly(name)
                for side in ("left", "right"):
                    real = EliteDriverConfig()
                    namespace["_configure_native_driver"](
                        real, SimpleNamespace(**effective), effective.get(side + "_tool_transform")
                    )
                    real.model_path = assembly.control_model_path(side)
                    simulated = native_config(requested, side, assembly)
                    self.assertEqual(describe_config(real), describe_config(simulated))
                    self.assertEqual(real.ik.streaming_iteration_budget, min(real.ik.max_iterations, 8))

    def test_packaged_profile_matches_recipe(self):
        for name in ("diagonal-07", "diagonal-08"):
            assembly = Assembly(name)
            profile = json.loads((assembly.root / "simulation_profile.json").read_text())
            recipe = load_recipe(ROOT / f"recipes/teleop/bi_elite_cs66_rt/{name}-taccap.yaml")
            for side in ("left", "right"):
                self.assertEqual(
                    describe_config(native_config(profile, side, assembly)),
                    describe_config(native_config(recipe, side, assembly)),
                )
            if name == "diagonal-08":
                cfg = native_config(profile, "left", assembly)
                np.testing.assert_allclose(np.asarray(cfg.tool).reshape(4, 4)[:3, 3], [0.0019, -0.04363, 0.21187])
