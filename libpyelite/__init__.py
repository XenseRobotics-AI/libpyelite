"""Elite native control. Cartesian poses are 4x4 SE(3), metres/radians.

No connections, threads or robot motion are started by importing this module.
"""
from pathlib import Path

import numpy as np

from . import _native
from ._native import (
    DashboardClientInterface,
    EliteDriverConfig,
    IK_BACKEND,
    PinkIKOptions,
    PlannerConfig,
    RtsiIOInterface,
)

__version__ = "0.1.0"
_ROOT = Path(__file__).resolve().parent


def model_path():
    return str(_ROOT / "assets" / "cs66" / "urdf" / "cs66.urdf")


def _pose(value):
    value = np.asarray(value, dtype=np.float64)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("Expected finite 4x4 SE(3), translation in metres")
    return value.ravel().tolist()


class PinkIKModel:
    """Hardware-free model using the worker's solver and URDF."""

    def __init__(self, urdf=None, tool=None, options=None):
        self._model = _native.PinkIKModel(
            urdf or model_path(), _pose(np.eye(4) if tool is None else tool),
            options or PinkIKOptions(),
        )

    def fk(self, q):
        return np.asarray(self._model.fk(q)).reshape(4, 4)

    def solve(self, target, reference, max_step_rad=None):
        step = [0.15] * 6 if max_step_rad is None else max_step_rad
        q, diagnostics = self._model.solve(_pose(target), reference, step)
        return np.asarray(q), diagnostics

    @property
    def lower_limits(self):
        return np.asarray(self._model.lower_limits)

    @property
    def upper_limits(self):
        return np.asarray(self._model.upper_limits)

    @property
    def velocity_limits(self):
        return np.asarray(self._model.velocity_limits)


def controller_tcp_to_pose6(tcp):
    """Convert Elite RTSI [x,y,z,roll,pitch,yaw] feedback to rotvec pose."""
    return np.asarray(_native.pose_to_rotvec(_native.pose_from_controller_tcp(tcp)))


class EliteDriver:
    """Native bridge retaining the existing Elite adapter's lifecycle names."""

    def __init__(self, config):
        if not config.model_path:
            config.model_path = model_path()
        if not config.script_file_path:
            config.script_file_path = str(_ROOT / "external_control.script")
        self._native = _native.NativeDriver(config)

    def __getattr__(self, name):
        return getattr(self._native, name)

    def submit_target(self, pose, duration=0.0):
        return self._native.submit(_pose(pose), float(duration))

    def check_error(self):
        error = self._native.status().error
        if error:
            raise RuntimeError(f"Elite native servo failed: {error}")

    def commanded_pose(self):
        return np.asarray(self._native.status().commanded_pose).reshape(4, 4)
