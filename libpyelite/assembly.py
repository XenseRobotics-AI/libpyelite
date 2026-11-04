"""Packaged station geometry; controller joint angles remain absolute radians."""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class AssemblyModel:
    id: str
    station: str
    end_effector: str
    label: str
    directory: str
    aliases: tuple[str, ...]


def available_models():
    """Packaged model choices for CLI and viewer selectors; no hardware I/O."""
    path = Path(__file__).resolve().parent / "assets/stations/catalog.json"
    return tuple(AssemblyModel(**(entry | {"aliases": tuple(entry["aliases"])}))
                 for entry in json.loads(path.read_text())["models"])


class Assembly:
    """Full viewer URDF plus derived, base-relative six-joint control chains.

    Assembly and generated control chains both use absolute controller angles.
    Tool fingers keep the source mimic relationship at a fixed viewer opening. Tool offsets are not
    baked in: the pendant TCP remains the source of truth for each arm.
    """

    def __init__(self, name, *, end_effector=None):
        choices = available_models()
        matches = [entry for entry in choices
                   if (name == entry.id or name in entry.aliases or
                       (end_effector is not None and name == entry.station))
                   and (end_effector is None or end_effector == entry.end_effector)]
        if len(matches) != 1:
            raise ValueError(f"Unsupported Elite model: {name!r}, end_effector={end_effector!r}; "
                             f"choose from {[entry.id for entry in choices]}")
        self.model = matches[0]
        self.name = self.model.id
        self.station = self.model.station
        self.end_effector = self.model.end_effector
        self.root = Path(__file__).resolve().parent / "assets" / "stations" / self.model.directory
        self.metadata = json.loads((self.root / "control.json").read_text())
        self.urdf = self.root / self.metadata["assembly_urdf"]

    def control_model_path(self, side):
        return str(self.root / self.metadata["arms"][side]["control_urdf"])

    def world_from_base(self, side):
        return np.asarray(self.metadata["arms"][side]["world_from_base"], dtype=float).copy()

    def initial_pose(self, side):
        return np.asarray(self.metadata["arms"][side]["initial_pose"], dtype=float).copy()

    def viewer_joint_defaults(self):
        return dict(self.metadata.get("viewer_joint_defaults", {}))
