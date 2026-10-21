"""Hardware-free CS66 Qt + Meshcat sandbox using the production native planner.

Slider widgets and XYZ/RPY helpers adapted from tianji_arm/tianji_sim.py.
No LeRobot imports, drivers, robot sockets, or separate Python IK solver.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
from collections.abc import Callable, Iterable, Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from . import PinkIKModel, PinkIKOptions, PlannerConfig, _native, model_path
from .assembly import Assembly
from .control_profile import describe_config, load_recipe, native_config

ARM_DOF = 6

LEFT = np.array([-1.3963, -0.5236, 1.5708, -0.5236, 1.5708, -0.7854])
RIGHT = np.array([-1.7453, -2.6180, -1.5708, -2.6180, -1.5708, -0.7854])


def _pose_from_xyz_rpy(values: Iterable[float]) -> np.ndarray:
    """Build a row-major homogeneous pose from x/y/z and fixed-axis RPY."""

    x, y, z, roll, pitch, yaw = (float(value) for value in values)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = rotation
    pose[:3, 3] = [x, y, z]
    return pose


def _rpy_from_rotation(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    pitch = float(np.arcsin(np.clip(-matrix[2, 0], -1.0, 1.0)))
    cosine = float(np.cos(pitch))
    if abs(cosine) > 1.0e-8:
        roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
        yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    else:
        roll = 0.0
        yaw = float(np.arctan2(-matrix[0, 1], matrix[1, 1]))
    return roll, pitch, yaw


def pose_to_xyz_rpy(pose: Sequence[Sequence[float]] | Sequence[float]) -> np.ndarray:
    """Return ``[x, y, z, roll, pitch, yaw]`` for a homogeneous pose."""

    matrix = np.asarray(pose, dtype=float).reshape(4, 4)
    roll, pitch, yaw = _rpy_from_rotation(matrix[:3, :3])
    return np.array([matrix[0, 3], matrix[1, 3], matrix[2, 3], roll, pitch, yaw])


def _prepare_qt_runtime() -> None:
    """Restart the CLI with wheel Qt libraries before conflicting Conda libraries.

    Linux reads LD_LIBRARY_PATH at process startup, so changing os.environ after
    importing Qt cannot fix a mixed private Qt ABI. Preserve other library paths
    for Pinocchio and the user's environment. Never restart library callers.
    """
    if not sys.platform.startswith("linux"):
        return
    spec = importlib.util.find_spec("PySide6")
    if spec is None or spec.origin is None:
        return  # _load_qt supplies the installation instructions.
    qt_lib = Path(spec.origin).parent / "Qt" / "lib"
    if not (qt_lib / "libQt6Core.so.6").is_file():
        return  # Conda-only layout: there is no bundled Qt to prioritize.
    plugins = qt_lib.parent / "plugins"
    # Conda activation or OpenCV may leave plugin paths pointing at another Qt.
    # Resolve them from the same PySide6 distribution as the shared libraries.
    if (plugins / "platforms").is_dir():
        os.environ["QT_PLUGIN_PATH"] = str(plugins)
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(plugins / "platforms")
    paths = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p]
    if paths and paths[0] == str(qt_lib):
        return
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join([str(qt_lib), *[p for p in paths if p != str(qt_lib)]])
    os.execve(sys.executable, [sys.executable, "-m", "libpyelite.sim", *sys.argv[1:]], env)


def _load_qt() -> dict[str, Any]:
    try:
        from PySide6.QtCore import QSignalBlocker, Qt, QTimer, QUrl
        from PySide6.QtGui import QDesktopServices
        from PySide6.QtWidgets import (
            QApplication,
            QDoubleSpinBox,
            QFormLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QPushButton,
            QSlider,
            QTabWidget,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as exc:  # pragma: no cover - optional dependency
        if isinstance(exc, ModuleNotFoundError) and exc.name == "PySide6":
            message = (
                "The Elite slider UI requires PySide6. From the repository root run: "
                "python -m pip install './third_party/libpyelite[sim]'"
            )
        else:
            message = (
                f"PySide6 could not load its Qt runtime: {exc}. "
                "This may be a mixed Conda/pip Qt ABI. Launch with "
                "python -m libpyelite.sim so bundled Qt libraries are prioritized."
            )
        raise RuntimeError(message) from exc
    return {
        "QSignalBlocker": QSignalBlocker,
        "Qt": Qt,
        "QTimer": QTimer,
        "QUrl": QUrl,
        "QDesktopServices": QDesktopServices,
        "QApplication": QApplication,
        "QDoubleSpinBox": QDoubleSpinBox,
        "QFormLayout": QFormLayout,
        "QGroupBox": QGroupBox,
        "QHBoxLayout": QHBoxLayout,
        "QLabel": QLabel,
        "QMainWindow": QMainWindow,
        "QPushButton": QPushButton,
        "QSlider": QSlider,
        "QTabWidget": QTabWidget,
        "QVBoxLayout": QVBoxLayout,
        "QWidget": QWidget,
    }


class _Slider:
    """Floating-point QSlider synchronized with a numeric spin box."""

    def __init__(
        self,
        qt: Mapping[str, Any],
        label: str,
        minimum: float,
        maximum: float,
        value: float,
        *,
        decimals: int,
        step: float,
        suffix: str,
    ) -> None:
        if not minimum < maximum:
            raise ValueError(f"invalid range for slider {label}")
        self.qt = qt
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.steps = 20_000
        self.callbacks: list[Callable[[float], None]] = []
        self.label = qt["QLabel"](label)
        self.label.setMinimumWidth(92)
        self.slider = qt["QSlider"](qt["Qt"].Orientation.Horizontal)
        self.slider.setRange(0, self.steps)
        self.slider.setTracking(True)
        self.spin = qt["QDoubleSpinBox"]()
        self.spin.setRange(self.minimum, self.maximum)
        self.spin.setDecimals(decimals)
        self.spin.setSingleStep(step)
        self.spin.setSuffix(suffix)
        self.spin.setKeyboardTracking(True)
        self.spin.setMinimumWidth(112)
        row = qt["QHBoxLayout"]()
        row.setContentsMargins(0, 1, 0, 1)
        row.addWidget(self.label)
        row.addWidget(self.slider, 1)
        row.addWidget(self.spin)
        self.widget = qt["QWidget"]()
        self.widget.setLayout(row)
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)
        self.set_value(value)

    def connect(self, callback: Callable[[float], None]) -> None:
        self.callbacks.append(callback)

    def _to_position(self, value: float) -> int:
        ratio = (float(value) - self.minimum) / (self.maximum - self.minimum)
        return int(round(float(np.clip(ratio, 0.0, 1.0)) * self.steps))

    def _from_position(self, position: int) -> float:
        return self.minimum + float(position) / self.steps * (self.maximum - self.minimum)

    def _emit(self, value: float) -> None:
        for callback in tuple(self.callbacks):
            callback(float(value))

    def _from_slider(self, position: int) -> None:
        blocker = self.qt["QSignalBlocker"](self.spin)
        value = self._from_position(position)
        self.spin.setValue(value)
        del blocker
        self._emit(value)

    def _from_spin(self, value: float) -> None:
        blocker = self.qt["QSignalBlocker"](self.slider)
        self.slider.setValue(self._to_position(value))
        del blocker
        self._emit(float(value))

    def value(self) -> float:
        return float(self.spin.value())

    def set_value(self, value: float, *, emit: bool = False) -> None:
        value = float(np.clip(value, self.minimum, self.maximum))
        slider_blocker = self.qt["QSignalBlocker"](self.slider)
        spin_blocker = self.qt["QSignalBlocker"](self.spin)
        self.slider.setValue(self._to_position(value))
        self.spin.setValue(value)
        del slider_blocker, spin_blocker
        if emit:
            self._emit(value)


class _JointSliders:
    def __init__(self, qt: Mapping[str, Any], title: str, names: Sequence[str], limits: np.ndarray) -> None:
        self.controls: list[_Slider] = []
        group = qt["QGroupBox"](title)
        layout = qt["QVBoxLayout"](group)
        for index, (name, limit) in enumerate(zip(names, limits, strict=True)):
            control = _Slider(
                qt,
                f"J{index + 1} {name}",
                math.degrees(float(limit[0])),
                math.degrees(float(limit[1])),
                0.0,
                decimals=2,
                step=0.1,
                suffix=" deg",
            )
            self.controls.append(control)
            layout.addWidget(control.widget)
        self.widget = group

    def connect(self, callback: Callable[[float], None]) -> None:
        for control in self.controls:
            control.connect(callback)

    def q(self) -> np.ndarray:
        return np.radians([control.value() for control in self.controls])

    def set_q(self, q: Sequence[float]) -> None:
        for control, value in zip(self.controls, np.degrees(q), strict=True):
            control.set_value(float(value))


class _PoseSliders:
    _SPECS = (
        ("x", -1.5, 1.5, 4, 0.001, " m"),
        ("y", -1.5, 1.5, 4, 0.001, " m"),
        ("z", -1.5, 1.5, 4, 0.001, " m"),
        ("roll", -math.pi, math.pi, 4, 0.01, " rad"),
        ("pitch", -math.pi, math.pi, 4, 0.01, " rad"),
        ("yaw", -math.pi, math.pi, 4, 0.01, " rad"),
    )

    def __init__(self, qt: Mapping[str, Any], title: str, center, xyz_range=0.3) -> None:
        self.controls: dict[str, _Slider] = {}
        group = qt["QGroupBox"](title)
        layout = qt["QVBoxLayout"](group)
        for name, minimum, maximum, decimals, step, suffix in self._SPECS:
            if name in ("x", "y", "z"):
                value = float(center[("x", "y", "z").index(name)])
                minimum, maximum = value - xyz_range, value + xyz_range
            control = _Slider(qt, name, minimum, maximum, 0.0, decimals=decimals, step=step, suffix=suffix)
            self.controls[name] = control
            layout.addWidget(control.widget)
        self.widget = group

    def include_pose(self, pose):
        """Recenter a local slider only when an explicit FK edit leaves its range."""
        for index, name in enumerate(("x", "y", "z")):
            control = self.controls[name]
            value = float(pose[index, 3])
            if not control.minimum <= value <= control.maximum:
                half = (control.maximum - control.minimum) / 2
                control.minimum, control.maximum = value - half, value + half
                blocker = control.qt["QSignalBlocker"](control.spin)
                control.spin.setRange(control.minimum, control.maximum)
                del blocker

    def connect(self, callback: Callable[[float], None]) -> None:
        for control in self.controls.values():
            control.connect(callback)

    def values(self) -> np.ndarray:
        return np.array([self.controls[name].value() for name in ("x", "y", "z", "roll", "pitch", "yaw")])

    def pose(self) -> np.ndarray:
        return _pose_from_xyz_rpy(self.values())

    def set_pose(self, pose: Sequence[Sequence[float]] | Sequence[float]) -> None:
        for name, value in zip(self.controls, pose_to_xyz_rpy(pose), strict=True):
            self.controls[name].set_value(float(value))


class Simulation:
    """Ideal joint-following plant; all motion samples come from native Planner."""

    def __init__(self, urdf=None, q=LEFT, tool=None, options=None, planner_config=None):
        self.urdf = str(urdf or model_path())
        self.tool = np.eye(4) if tool is None else np.asarray(tool).reshape(4, 4)
        self.model = PinkIKModel(self.urdf, self.tool, options)
        self.config = planner_config or PlannerConfig()
        self.planner = _native.Planner(self.urdf, self.tool.ravel(), options or PinkIKOptions(), self.config)
        self.sender = None
        if self.config.joint_motion_limits:
            self.sender = _native.MotionLimiter(
                self.config.period,
                self.config.max_joint_velocity,
                self.config.max_joint_acceleration,
                self.config.max_joint_jerk,
                self.model.lower_limits,
                self.model.upper_limits,
            )
        self.time = 0.0
        self.sequence = 0
        self.last_submit = -1.0
        self.reset(q)

    def reset(self, q):
        self.planner.seed(q)
        if self.sender:
            self.sender.seed(q)
        self.q = np.asarray(q, dtype=float).copy()
        self.target = self.model.fk(q)
        self.joint_target = None  # set while joint (FK) targets drive the arm
        self.last_submit = -1.0
        self._target_dirty = True

    def set_target(self, target):
        target = np.asarray(target, dtype=float).reshape(4, 4)
        if self.joint_target is not None or not np.array_equal(target, self.target):
            self.target = target.copy()
            self.joint_target = None
            self._target_dirty = True

    def set_joint_target(self, q):
        """Joint-space goal, planned under the same limits and collision gate (not a reset)."""
        q = np.asarray(q, dtype=float).reshape(6)
        if not np.isfinite(q).all() or np.any(q < self.model.lower_limits) or np.any(q > self.model.upper_limits):
            raise ValueError("Joint target outside CS66 limits")
        reference = self.q if self.joint_target is None else self.joint_target
        jump = self.config.max_target_joint_jump
        if jump > 0 and np.max(np.abs(q - reference)) > jump:
            raise ValueError(f"Joint target jump > {jump:g} rad rejected; move the slider in smaller steps")
        if self.joint_target is None or not np.array_equal(q, self.joint_target):
            self.joint_target = q.copy()
            self.target = self.model.fk(q)
            self._target_dirty = True

    def step(self, target=None, *, heartbeat=True):
        self.begin(target, heartbeat=heartbeat)
        return self.finish()

    def begin(self, target=None, *, heartbeat=True):
        """Submit the target and plan the joint reference; ``finish`` publishes q.

        Between the two, ``planner.reference()`` may be replaced with
        ``planner.set_reference()`` when ``self.pending`` is true.
        """
        self.submit(target, heartbeat=heartbeat)
        self._previous = self.q.copy()
        self.pending = self.planner.plan_reference(self.time)

    def submit(self, target=None, *, heartbeat=True):
        """Hand the current target to the planner (on change, or as a 32 ms heartbeat)."""
        if target is not None:
            self.set_target(target)
        if heartbeat and (self._target_dirty or self.time - self.last_submit >= 0.032 - 1e-12):
            self.sequence += 1
            if self.joint_target is not None:
                self.planner.submit_joints(self.joint_target, self.time, self.sequence)
            else:
                self.planner.submit(self.target.ravel(), self.time, self.sequence)
            self.last_submit = self.time
            self._target_dirty = False

    def finish(self):
        previous = self._previous
        self.q = np.asarray(self.planner.advance() if self.pending else self.planner.q())
        self.pending = False
        self.time += self.config.period
        status = self.planner.status()
        if self.sender:
            # Same rule as NativeDriver::sendLoop: pass feasible planner samples through.
            self.q = np.asarray(
                self.sender.step(self.q, True)
                if status.idle
                else self.sender.gate(self.q, status.joint_velocity, status.joint_acceleration)
            )
        return self.sample(previous)

    def sample(self, previous):
        status = self.planner.status()
        pose = self.model.fk(self.q)
        angle = np.linalg.norm(_native.pose_to_rotvec((np.linalg.inv(pose) @ self.target).ravel())[3:])
        return {
            "time": self.time,
            "position_error_m": float(np.linalg.norm(pose[:3, 3] - self.target[:3, 3])),
            "orientation_error_rad": float(angle),
            "max_joint_step_rad": float(np.max(np.abs(self.q - previous))),
            "solve_time_s": status.solve_time_s,
            "success": status.ik.success,
            "progress": status.ik.progress,
            "qp_failed": status.ik.qp_failed,
            "ik_failures": status.ik_failures,
            "moving": status.moving,
            "stale": status.stale,
        }


class StationSimulation:
    """Two native planners in base coordinates; UI/viewer poses in station world."""

    joint_names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    left_joint_names = right_joint_names = joint_names

    def __init__(self, station="diagonal-08", initial_q=None, tool=None, settings=None, collision_profile=None):
        import pinocchio as pin

        self.assembly = Assembly(station)
        self.urdf_path = self.assembly.urdf
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.indices = {
            side: [self.model.joints[self.model.getJointId(side + "_" + name)].idx_q for name in self.joint_names]
            for side in ("left", "right")
        }
        self.settings = (
            json.loads((self.assembly.root / "simulation_profile.json").read_text()) if settings is None else settings
        )
        self.configs = {side: native_config(self.settings, side, self.assembly, tool) for side in ("left", "right")}
        self.arms = {
            side: Simulation(
                cfg.model_path,
                self.settings.get(f"{side}_start_position_rad", self.assembly.initial_pose(side)),
                np.asarray(cfg.tool).reshape(4, 4),
                cfg.ik,
                cfg.planner,
            )
            for side, cfg in self.configs.items()
        }
        self.joint_limits = np.concatenate(
            [np.column_stack((arm.model.lower_limits, arm.model.upper_limits)) for arm in self.arms.values()]
        )
        self.collision = None
        self.guard = None
        self.collision_error = None
        self.last_collision = None
        if collision_profile is not None:
            from .collision import StationLayer

            # Built exactly as the hardware adapter builds it: the same native
            # StationGuard the StationCoordinator runs on the robot.
            layer = StationLayer(self.assembly, collision_profile, [arm.tool for arm in self.arms.values()])
            self.collision, self.guard = layer.collision, layer.guard
            # The guard's joint gate is the final command gate, as on hardware,
            # where the sender then only checks velocity and position.
            for arm in self.arms.values():
                arm.sender = None
        if initial_q is not None:
            self.reset(initial_q)
        if self.collision:
            self.last_collision = self.collision.check(self.viewer_q())
            if not self.last_collision.safe:
                self.collision_error = f"Initial collision clearance invalid: {self.last_collision}"

        self.home_q = self.q.copy()
        self.home_q.setflags(write=False)

    def reset_to_home(self):
        """Offline simulation reset, not a planned robot motion."""
        self.reset(self.home_q)

    @property
    def q(self):
        return np.concatenate([arm.q for arm in self.arms.values()])

    def reset(self, q, side="both"):
        q = np.asarray(q, dtype=float).reshape(12)
        candidate = self.q.copy()
        for index, selected in enumerate(self.arms):
            if side in ("both", selected):
                candidate[index * 6 : (index + 1) * 6] = q[index * 6 : (index + 1) * 6]
        if (
            not np.isfinite(candidate).all()
            or np.any(candidate < self.joint_limits[:, 0])
            or np.any(candidate > self.joint_limits[:, 1])
        ):
            raise ValueError("Invalid reset joint configuration")
        if self.collision:
            result = self.collision.check(self.viewer_q(candidate))
            if not result.safe:
                raise ValueError(f"Reset rejected by collision check: {result}")
        for index, selected in enumerate(self.arms):
            if side in ("both", selected):
                self.arms[selected].reset(candidate[index * 6 : (index + 1) * 6])
        if self.guard:
            self.guard.reset()
        self.collision_error = None

    def viewer_q(self, arm_q=None):
        q = np.zeros(self.model.nq)
        for name, value in self.assembly.viewer_joint_defaults().items():
            joint = self.model.joints[self.model.getJointId(name)]
            q[joint.idx_q] = value
        for index, (side, arm) in enumerate(self.arms.items()):
            q[self.indices[side]] = arm.q if arm_q is None else arm_q[index * 6 : (index + 1) * 6]
        return q

    def tcp_poses(self):
        return {side: self.assembly.world_from_base(side) @ arm.model.fk(arm.q) for side, arm in self.arms.items()}

    def targets(self):
        return {side: self.assembly.world_from_base(side) @ arm.target for side, arm in self.arms.items()}

    def set_target(self, side, world_pose):
        self.arms[side].set_target(np.linalg.inv(self.assembly.world_from_base(side)) @ world_pose)

    def set_joint_target(self, side, q):
        self.arms[side].set_joint_target(q)

    def step(self):
        if self.collision_error:
            raise RuntimeError(self.collision_error + "; reset explicitly to a checked safe configuration")
        if not self.collision:
            return {side: arm.step() for side, arm in self.arms.items()}
        previous = self.q.copy()
        left, right = self.arms["left"], self.arms["right"]
        try:
            for arm in self.arms.values():
                arm.submit()
            result = self.guard.tick(left.planner, right.planner, left.q, right.q, left.time)
            samples = {}
            for index, (side, arm) in enumerate(self.arms.items()):
                arm.q = np.asarray(result.q[index])
                arm.pending = False
                arm.time += arm.config.period
                sample = arm.sample(previous[index * 6 : (index + 1) * 6])
                sample["collision_constraints"] = result.constraints
                sample["collision_distance_m"] = result.clearance.minimum_distance
                sample["collision_limited"] = bool(result.limited[index])
                sample["collision_hold"] = bool(result.held[index])
                samples[side] = sample
            self.last_collision = self.collision.result(result.clearance)
            return samples
        except Exception as exc:
            # Only inconsistent hard bounds or an already-violated clearance get here:
            # halt and latch, never resume on the next timer tick (on hardware both
            # arms stop with the error latched).
            for index, arm in enumerate(self.arms.values()):
                arm.reset(previous[index * 6 : (index + 1) * 6])
            self.collision_error = str(exc)
            raise


class MeshcatRobotView:
    def __init__(self, simulation, *, open_browser=True):
        import meshcat.geometry as geometry
        import pinocchio as pin
        from pinocchio.visualize import MeshcatVisualizer

        visual = pin.buildGeomFromUrdf(
            simulation.model,
            str(simulation.urdf_path),
            pin.GeometryType.VISUAL,
            package_dirs=[str(simulation.urdf_path.parent)],
        )
        self.visualizer = MeshcatVisualizer(simulation.model, visual_model=visual)
        self.visualizer.initViewer(open=open_browser)
        self.visualizer.loadViewerModel(rootNodeName="elite_station")
        self.viewer = self.visualizer.viewer
        for side, color in (("left", 0xE53935), ("right", 0x1E88E5)):
            self.viewer[f"targets/{side}"].set_object(
                geometry.Sphere(0.025), geometry.MeshLambertMaterial(color=color, opacity=0.85)
            )
            self.viewer[f"targets/{side}/axes"].set_object(geometry.triad(scale=0.12))
            self.viewer[f"tcp/{side}"].set_object(geometry.triad(scale=0.08))
        self.collision_nodes = []
        self.collision_visible = False
        if simulation.collision:
            for index, shape in enumerate(simulation.collision.shapes):
                node = self.viewer[f"collision/{index}"]
                material = geometry.MeshLambertMaterial(
                    color=0xFF9800 if shape["group"] == "environment" else 0x26A69A, opacity=0.25
                )
                if shape["type"] == "box":
                    node.set_object(geometry.Box(shape["size"]), material)
                elif shape["type"] == "convex":
                    node.set_object(
                        geometry.TriangularMeshGeometry(
                            np.asarray(shape["vertices"]), np.asarray(shape["faces"], dtype=np.int32)
                        ),
                        material,
                    )
                else:
                    node["body"].set_object(geometry.Cylinder(shape["length"], shape["radius"]), material)
                    rotation = np.eye(4)
                    rotation[:3, :3] = _pose_from_xyz_rpy([0, 0, 0, np.pi / 2, 0, 0])[:3, :3]
                    node["body"].set_transform(rotation)
                    if shape["type"] == "capsule":
                        for end in (-1, 1):
                            node[str(end)].set_object(geometry.Sphere(shape["radius"]), material)
                            tf = np.eye(4)
                            tf[2, 3] = end * shape["length"] / 2
                            node[str(end)].set_transform(tf)
                self.collision_nodes.append(node)
            self.viewer["collision"].set_property("visible", False)
        self.url = str(self.viewer.url())
        self.update(simulation)

    def set_collision_visible(self, visible):
        """Toggle collision geometry and avoid sending hidden transforms to MeshCat."""
        self.collision_visible = bool(visible)
        self.viewer["collision"].set_property("visible", self.collision_visible)

    def update(self, simulation):
        self.visualizer.display(simulation.viewer_q())
        if self.collision_nodes and self.collision_visible:
            for node, tf in zip(
                self.collision_nodes, simulation.collision.placements(simulation.viewer_q()), strict=True
            ):
                node.set_transform(tf.homogeneous)
        for side, pose in simulation.targets().items():
            self.viewer[f"targets/{side}"].set_transform(pose)
        for side, pose in simulation.tcp_poses().items():
            self.viewer[f"tcp/{side}"].set_transform(pose)


def run_headless_validation(urdf=None):
    """Deterministic reachable, unreachable, recovery and wrist-near-zero traces."""
    report = {}
    for name, seed in (
        ("left", LEFT),
        ("right", RIGHT),
        ("near_singular", np.array([-1.4, -0.5, 1.5, -0.5, 0.001, -0.8])),
    ):
        sim = Simulation(urdf, seed)
        home = sim.target.copy()
        samples = []
        for i in range(1250):
            target = home.copy()
            if i < 250:
                target[0, 3] += 0.015 * math.sin(i * math.pi / 250)
            elif i < 500:
                target[0, 3] += 2.0  # intentionally unreachable
            samples.append(sim.step(target))
            if not np.isfinite(sim.q).all():
                raise AssertionError("nonfinite command")
            if np.any(sim.q < sim.model.lower_limits) or np.any(sim.q > sim.model.upper_limits):
                raise AssertionError("joint position limit violated")
        max_step = max(s["max_joint_step_rad"] for s in samples)
        if max_step > max(sim.model.velocity_limits) * sim.config.period + 1e-10:
            raise AssertionError("joint velocity limit violated")
        if any(s["qp_failed"] for s in samples):
            raise AssertionError("QP failure in validation trace")
        report[name] = {
            "samples": len(samples),
            "max_joint_step_rad": max_step,
            "solve_p99_ms": float(np.percentile([s["solve_time_s"] for s in samples], 99) * 1000),
            "recovery_position_error_m": samples[-1]["position_error_m"],
            "recovery_orientation_error_rad": samples[-1]["orientation_error_rad"],
        }
        if samples[-1]["position_error_m"] > 0.001 or samples[-1]["orientation_error_rad"] > 0.01:
            raise AssertionError(f"{name}: recovery failed: {report[name]}")
    return report


def target_status(sample):
    """Classify one arm sample as ``(level, message)``.

    ``level`` is ``"ok"`` (user target reached), ``"tracking"``, ``"limited"`` or
    ``"error"``. Only the user target counts as reached; a converged waypoint
    does not. Distance to a moving slider target is planned tracking lag, not
    an IK failure: the solver is stuck only when its QP fails or a solve
    neither met the waypoint tolerance nor reduced the error. A zero-iteration
    solve (waypoint already within tolerance) reports ``success`` without
    ``progress`` and is healthy.
    """
    if sample["qp_failed"]:
        return "error", "IK 失败（QP 求解失败）"
    if sample["position_error_m"] <= 1e-4 and sample["orientation_error_rad"] <= 1e-3:
        return "ok", "已到达目标"
    if sample.get("collision_hold"):
        return "limited", "接触障碍物：保持当前姿态（目标在障碍物另一侧）"
    if not (sample["success"] or sample["progress"]):
        return "error", "IK 无进展（目标可能不可达或受关节限位约束）"
    if sample.get("collision_limited"):
        return "limited", "跟踪中（碰撞 / 运动约束限速）"
    return "tracking", "跟踪中（IK 正常，受速度 / 加速度限制滞后）"


# A frame never runs more than this many control periods; beyond it the
# simulation falls behind the wall clock and the window reports that.
_MAX_STEPS_PER_FRAME = 8

# Tracking that stops closing on the user target is reported as unreachable.
_STALL_WINDOW_S = 0.5
_STALL_PROGRESS_M = 5e-4
_STALL_PROGRESS_RAD = 5e-3


def make_window(qt, model, visualizer=None, xyz_range=0.3):
    """Tianji window layout, with six joints per arm and native streaming control."""

    class EliteSimWindow(qt["QMainWindow"]):
        def __init__(self):
            super().__init__()
            self.model = model
            self.visualizer = visualizer
            self._syncing = False
            # side -> (target, best position error, best orientation error, sim time of last improvement)
            self._progress = {}
            self.setWindowTitle("Elite CS66 FK / IK — MeshCat sandbox")
            self.resize(1180, 820)
            self._build_ui(qt)
            self._sync_from_q()
            self.render_timer = qt["QTimer"](self)
            self.render_timer.setInterval(16)
            self.render_timer.timeout.connect(self._on_timer)
            self._clock = None  # wall-clock time of the last timer tick
            self._backlog = 0.0  # simulated seconds owed to the wall clock
            self._realtime = 1.0  # smoothed simulated / wall-clock time
            self.render_timer.start()

        def _on_timer(self):
            """Advance by the elapsed wall time, so a slow step shows as a lower
            real-time factor instead of silently stretching motion in time."""
            import time

            now = time.perf_counter()
            if self._clock is None:
                self._clock = now
                return
            elapsed, self._clock = now - self._clock, now
            if elapsed > 0.5:  # resumed after a pause: do not replay the gap
                return
            period = model.arms["left"].config.period
            self._backlog = min(self._backlog + elapsed, _MAX_STEPS_PER_FRAME * period)
            steps = int(self._backlog / period)
            self._backlog -= steps * period
            # Simulated over wall time for this frame; the cost of the previous
            # frame's steps is part of this frame's elapsed time.
            self._realtime += 0.1 * (steps * period / max(elapsed, 1e-6) - self._realtime)
            if steps:
                self.advance(steps)

        def _build_ui(self, qt_symbols: Mapping[str, Any]) -> None:
            central = qt_symbols["QWidget"]()
            root = qt_symbols["QVBoxLayout"](central)
            title = qt_symbols["QLabel"]("Elite CS66 双臂 FK / IK 轻量仿真")
            title.setStyleSheet("font-size: 20px; font-weight: 600;")
            root.addWidget(title)
            root.addWidget(qt_symbols["QLabel"](f"URDF: {model.urdf_path}"))
            profile = describe_config(model.configs["left"])
            root.addWidget(
                qt_symbols["QLabel"](
                    f"Native Planner（与真机共用）：周期 {profile['planner']['period'] * 1000:g} ms，"
                    f"连续 IK ≤{profile['streaming_iteration_budget']} 次，"
                    f"离线 IK ≤{profile['ik']['max_iterations']} 次，阻尼 {profile['ik']['damping']:g}；"
                    "TCP 使用 recipe 配置"
                )
            )
            mesh_row = qt_symbols["QHBoxLayout"]()
            self.mesh_status = qt_symbols["QLabel"](
                f"MeshCat: {visualizer.url}" if visualizer is not None else "MeshCat: disabled"
            )
            mesh_row.addWidget(self.mesh_status, 1)
            open_button = qt_symbols["QPushButton"]("打开 MeshCat")
            open_button.setEnabled(visualizer is not None)
            open_button.clicked.connect(self._open_meshcat)
            mesh_row.addWidget(open_button)
            if model.collision:
                toggle = qt_symbols["QPushButton"]("显示碰撞包络")
                toggle.setCheckable(True)
                toggle.setEnabled(visualizer is not None)
                if visualizer:
                    toggle.toggled.connect(visualizer.set_collision_visible)
                mesh_row.addWidget(toggle)
                root.addWidget(qt_symbols["QLabel"]("仿真碰撞约束已启用：自碰撞 / 双臂 / 环境；异常锁存后需显式重置。"))
            root.addLayout(mesh_row)

            tabs = qt_symbols["QTabWidget"]()
            fk_page = qt_symbols["QWidget"]()
            fk_layout = qt_symbols["QVBoxLayout"](fk_page)
            fk_layout.addWidget(qt_symbols["QLabel"]("FK：拖动关节滑条或点击应用，发送关节目标（受速度 / 加速度 / 碰撞约束，不是瞬移）。"))
            joint_row = qt_symbols["QHBoxLayout"]()
            limits = np.asarray(model.joint_limits, dtype=float)
            self.left_joints = _JointSliders(qt_symbols, "左臂关节", model.left_joint_names, limits[:ARM_DOF])
            self.right_joints = _JointSliders(qt_symbols, "右臂关节", model.right_joint_names, limits[ARM_DOF:])
            joint_row.addWidget(self.left_joints.widget)
            joint_row.addWidget(self.right_joints.widget)
            fk_layout.addLayout(joint_row)
            fk_buttons = qt_symbols["QHBoxLayout"]()
            for label, side in (("应用左臂 FK", "left"), ("应用右臂 FK", "right"), ("应用双臂 FK", "both")):
                button = qt_symbols["QPushButton"](label)
                button.clicked.connect(lambda _checked=False, selected=side: self._apply_fk(selected))
                fk_buttons.addWidget(button)
            fk_layout.addLayout(fk_buttons)
            self.fk_status = qt_symbols["QLabel"]()
            self.fk_status.setWordWrap(True)
            self.fk_status.setStyleSheet("font-family: monospace;")
            fk_layout.addWidget(self.fk_status)
            fk_layout.addStretch(1)
            tabs.addTab(fk_page, "FK · Joint sliders")
            self.left_joints.connect(lambda _value: self._apply_fk("left"))
            self.right_joints.connect(lambda _value: self._apply_fk("right"))

            ik_page = qt_symbols["QWidget"]()
            ik_layout = qt_symbols["QVBoxLayout"](ik_page)
            ik_layout.addWidget(
                qt_symbols["QLabel"](
                    f"IK：世界坐标 XYZ / RPY；XYZ 局部范围 ±{xyz_range:g} m（不保证所有位姿可达）。绿色=已到达，橙色=跟踪中，红色=IK 失败或目标不可达。"
                )
            )
            pose_row = qt_symbols["QHBoxLayout"]()
            self.left_pose = _PoseSliders(qt_symbols, "左臂 TCP 目标", model.tcp_poses()["left"][:3, 3], xyz_range)
            self.right_pose = _PoseSliders(qt_symbols, "右臂 TCP 目标", model.tcp_poses()["right"][:3, 3], xyz_range)
            pose_row.addWidget(self.left_pose.widget)
            pose_row.addWidget(self.right_pose.widget)
            ik_layout.addLayout(pose_row)
            self.left_pose.connect(lambda _value: self._schedule_ik("left"))
            self.right_pose.connect(lambda _value: self._schedule_ik("right"))
            ik_buttons = qt_symbols["QHBoxLayout"]()
            for label, side in (("求解左臂", "left"), ("求解右臂", "right"), ("求解双臂", "both")):
                button = qt_symbols["QPushButton"](label)
                button.clicked.connect(lambda _checked=False, selected=side: self._solve(selected))
                ik_buttons.addWidget(button)
            sync_button = qt_symbols["QPushButton"]("从当前 FK 同步目标")
            sync_button.clicked.connect(self._sync_from_q)
            ik_buttons.addWidget(sync_button)
            ik_layout.addLayout(ik_buttons)
            self.ik_status = qt_symbols["QLabel"]()
            self.ik_status.setWordWrap(True)
            self.ik_status.setStyleSheet("font-family: monospace;")
            ik_layout.addWidget(self.ik_status)
            ik_layout.addStretch(1)
            tabs.addTab(ik_page, "IK · XYZ / RPY sliders")
            root.addWidget(tabs, 1)
            home = qt_symbols["QPushButton"]("Reset to home · 恢复启动姿态（仿真重置）")
            home.setObjectName("reset_to_home")
            home.clicked.connect(self._reset_home)
            root.addWidget(home)
            reset = qt_symbols["QPushButton"]("从当前状态恢复 / 同步目标")
            reset.clicked.connect(self._sync_from_q)
            root.addWidget(reset)
            self.setCentralWidget(central)

        def _reset_home(self):
            try:
                model.reset_to_home()
            except ValueError as exc:
                self.ik_status.setText(f"Home 重置被拒绝: {exc}")
                return
            self._sync_from_q()
            if visualizer:
                visualizer.update(model)
            self.ik_status.setText("已恢复启动关节姿态和 TCP 目标；这是仿真配置重置。")

        def _open_meshcat(self) -> None:
            if self.visualizer is not None:
                qt["QDesktopServices"].openUrl(qt["QUrl"](self.visualizer.url))

        def _sync_from_q(self):
            try:
                model.reset(model.q)
            except ValueError as exc:
                self.ik_status.setText(str(exc))
                return
            if hasattr(self, "render_timer"):
                self.render_timer.start()
            self._syncing = True
            self.left_joints.set_q(model.q[:6])
            self.right_joints.set_q(model.q[6:])
            poses = model.tcp_poses()
            self.left_pose.include_pose(poses["left"])
            self.right_pose.include_pose(poses["right"])
            self.left_pose.set_pose(poses["left"])
            self.right_pose.set_pose(poses["right"])
            self._syncing = False
            self.fk_status.setText("已从当前关节配置同步 TCP 目标。")
            self.ik_status.setText("拖动 TCP 目标或点击求解。")

        def _apply_fk(self, side):
            """Joint sliders command a planned joint motion; it never teleports.

            A reset placed each slider value directly and checked only that end
            configuration, so dragging past the table jumped through it once the
            far side was free. A joint target runs through the same limits and
            collision gate as IK targets, and stops at the obstacle.
            """
            if self._syncing:
                return
            q = np.r_[self.left_joints.q(), self.right_joints.q()]
            try:
                for index, selected in enumerate(model.arms):
                    if side in ("both", selected):
                        model.set_joint_target(selected, q[index * 6 : (index + 1) * 6])
            except ValueError as exc:
                self.fk_status.setText(str(exc))
                return
            self._syncing = True
            for selected, pose in model.targets().items():
                if side in ("both", selected):
                    getattr(self, selected + "_pose").include_pose(pose)
                    getattr(self, selected + "_pose").set_pose(pose)
            self._syncing = False
            self.fk_status.setText(
                "关节目标已发送（经速度 / 加速度 / 碰撞约束执行）；目标 q [deg] = "
                + np.array2string(np.degrees(q), precision=2)
            )

        def _schedule_ik(self, side):
            if self._syncing:
                return
            # Submit the newest target without waiting for dragging to stop.
            # The next render tick consumes it; intermediate edits are coalesced.
            self._solve(side)

        def _solve(self, side):
            for selected in model.arms:
                if side in ("both", selected):
                    model.set_target(selected, getattr(self, selected + "_pose").pose())

        def advance(self, steps=4):
            try:
                for _ in range(steps):
                    samples = model.step()
            except Exception as exc:
                self.ik_status.setText(f"仿真已暂停: {exc}。修改目标后点击恢复，或 Reset to home。")
                self.ik_status.setStyleSheet("font-family: monospace; color: #b00020;")
                self.render_timer.stop()
                return
            # Joint sliders show the measured arm, except while they command it:
            # overwriting them then would fight the drag.
            self._syncing = True
            for selected, sliders in (("left", self.left_joints), ("right", self.right_joints)):
                if model.arms[selected].joint_target is None:
                    sliders.set_q(model.arms[selected].q)
            self._syncing = False
            if any(arm.joint_target is not None for arm in model.arms.values()):
                self.fk_status.setText("实际 q [deg] = " + np.array2string(np.degrees(model.q), precision=2))
            if visualizer:
                visualizer.update(model)
            results = [self._classify(side, sample) for side, sample in samples.items()]
            self.ik_status.setText(
                f"仿真实时倍率 {min(self._realtime, 1.0):.2f}×（低于 1 表示计算跟不上，动作按真实时间变慢）\n"
                + "\n".join(
                    f"{side}: {result[1]}, "
                    f"pos={sample['position_error_m']:.2e} m, "
                    f"ori={sample['orientation_error_rad']:.2e} rad, "
                    f"QP failure={sample['qp_failed']}, "
                    f"solve={sample['solve_time_s'] * 1000:.3f} ms"
                    + (
                        ", collision="
                        + (
                            f"{sample['collision_distance_m'] * 1000:.2f} mm"
                            if np.isfinite(sample["collision_distance_m"])
                            else f"> {model.collision.detection_distance * 1000:g} mm"
                        )
                        + f", constraints={sample['collision_constraints']}"
                        if "collision_distance_m" in sample
                        else ""
                    )
                    for (side, sample), result in zip(samples.items(), results, strict=True)
                )
            )
            levels = {level for level, _ in results}
            color = "#b00020" if "error" in levels else "#1b7f2a" if levels == {"ok"} else "#a15c00"
            self.ik_status.setStyleSheet(f"font-family: monospace; color: {color};")

        def _classify(self, side, sample):
            """target_status plus a stall check: tracking that stopped closing is an error."""
            level, message = target_status(sample)
            target = model.arms[side].target
            position, angle, now = sample["position_error_m"], sample["orientation_error_rad"], sample["time"]
            previous = self._progress.get(side)
            if (
                previous is None
                or not np.array_equal(previous[0], target)
                or position < previous[1] - _STALL_PROGRESS_M
                or angle < previous[2] - _STALL_PROGRESS_RAD
            ):
                self._progress[side] = (target.copy(), position, angle, now)
            elif level in ("tracking", "limited") and not sample.get("collision_hold") and now - previous[3] > _STALL_WINDOW_S:
                return "error", f"目标不可达或受限（{_STALL_WINDOW_S:g} s 内误差未再减小）"
            return level, message

    return EliteSimWindow()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--initial-q", type=float, nargs=12, metavar="DEG")
    parser.add_argument("--xyz-range", type=float, default=0.3, help="XYZ slider half-range around initial TCP, metres")
    parser.add_argument("--station", choices=("diagonal-07", "diagonal-08"))
    parser.add_argument("--recipe", type=Path, help="use the real robot recipe native settings and TCP")
    parser.add_argument(
        "--collision-profile",
        help="auto, a collision JSON, or off; overrides the recipe's native_collision_profile (simulation only)",
    )
    parser.add_argument("--safety-config", type=Path, help="JSON overrides for simulation tuning only")
    parser.add_argument(
        "--show-config", action="store_true", help="print effective native parameters without GUI/hardware"
    )
    parser.add_argument("--tool", type=float, nargs=6, metavar="XYZ_RPY", help="tool0 to TCP: metres / radians")
    parser.add_argument("--headless-check", action="store_true")
    parser.add_argument("--no-meshcat", action="store_true")
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--no-show", action="store_true", help="construct and exercise UI, then exit")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.headless_check:
        report = run_headless_validation(args.urdf)
        result = json.dumps(report, indent=2)
        print(result)
        if args.report:
            args.report.write_text(result + "\n")
        return 0
    if not np.isfinite(args.xyz_range) or args.xyz_range <= 0:
        parser.error("--xyz-range must be positive and finite")
    if not args.show_config:
        _prepare_qt_runtime()
    if args.urdf is not None:
        parser.error("--urdf applies only to --headless-check; select the GUI model with --station")
    settings = None if args.recipe is None else load_recipe(args.recipe)
    if args.safety_config:
        overrides = json.loads(args.safety_config.read_text())
        if settings is None:
            selected = args.station or "diagonal-08"
            settings = json.loads((Assembly(selected).root / "simulation_profile.json").read_text())
        settings = settings | overrides
    station = args.station or (settings or {}).get("native_assembly", "diagonal-08")
    if settings and settings.get("native_assembly", station) != station:
        parser.error("--station and recipe native_assembly must agree")
    collision_profile = args.collision_profile
    if collision_profile is None:
        collision_profile = (settings or {}).get("native_collision_profile")
    if collision_profile in ("off", "none", "null"):
        collision_profile = None
    simulation = StationSimulation(
        station,
        None if args.initial_q is None else np.radians(args.initial_q),
        None if args.tool is None else _pose_from_xyz_rpy(args.tool),
        settings,
        collision_profile,
    )
    if args.show_config:
        print(json.dumps({side: describe_config(cfg) for side, cfg in simulation.configs.items()}, indent=2))
        return 0
    qt = _load_qt()
    app = qt["QApplication"].instance() or qt["QApplication"]([])
    visualizer = None if args.no_meshcat else MeshcatRobotView(simulation, open_browser=not args.no_browser)
    window = make_window(qt, simulation, visualizer, args.xyz_range)
    if args.no_show:
        window.advance()
        app.processEvents()
        window.close()
        return 0
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
