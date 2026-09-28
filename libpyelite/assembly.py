"""Packaged station geometry; controller joint angles remain absolute radians."""
import json
from pathlib import Path

import numpy as np


class Assembly:
    """Full viewer URDF plus derived, base-relative six-joint control chains.

    Assembly URDF joints use q_relative = q_controller - initial_pose.
    The generated control chains undo that reparameterization and expose the
    same absolute joint coordinates as the controller. Tool offsets are not
    baked in: the pendant TCP remains the source of truth for each arm.
    """

    def __init__(self, name):
        if name not in ("diagonal-07", "diagonal-08"):
            raise ValueError(f"Unsupported Elite assembly: {name!r}")
        self.root = Path(__file__).resolve().parent / "assets" / "stations" / name
        self.metadata = json.loads((self.root / "control.json").read_text())
        self.urdf = self.root / self.metadata["assembly_urdf"]

    def control_model_path(self, side):
        return str(self.root / self.metadata["arms"][side]["control_urdf"])

    def world_from_base(self, side):
        return np.asarray(self.metadata["arms"][side]["world_from_base"], dtype=float).copy()

    def initial_pose(self, side):
        return np.asarray(self.metadata["arms"][side]["initial_pose"], dtype=float).copy()
