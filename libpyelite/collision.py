"""Coal primitives in named URDF link frames; independent JSON geometry policy.

The profile is validated here; distances, barrier rows and step verification
run in the native StationCollisionModel (src/station_collision.cpp).
"""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from . import _native

_TYPES = {"box": 0, "capsule": 1, "cylinder": 2, "convex": 3}


@dataclass(frozen=True)
class CollisionResult:
    safe: bool
    minimum_distance: float
    pair: tuple[str, str] | None = None
    clearance: float = float("inf")


class StationCollision:
    def __init__(self, model, profile, urdf=None):
        self.model = model
        self.profile = json.loads(Path(profile).read_text())
        if self.profile["schema"] != 1:
            raise ValueError("Unsupported collision profile schema")
        if urdf and hashlib.sha256(Path(urdf).read_bytes()).hexdigest() != self.profile["urdf_sha256"]:
            raise ValueError("Collision profile is stale: regenerate/review after URDF update")
        if urdf:
            for source, digest in self.profile.get("sources", {}).items():
                path = Path(urdf).parent.parent / source
                if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                    raise ValueError("Collision geometry source changed: " + source)
        self.margin = float(self.profile["margin_m"])
        if not np.isfinite(self.margin) or self.margin < 0:
            raise ValueError("Invalid collision margin")
        self.self_margin = float(self.profile.get("self_margin_m", self.margin))
        self.detection_distance = float(self.profile.get("detection_distance_m", 0.10))
        self.gain = float(self.profile.get("gain", 0.2))
        # Extra clearance the linearized barrier aims for beyond the margin, so its
        # first-order error does not land a step inside the margin the check enforces.
        self.qp_buffer = float(self.profile.get("qp_buffer_m", 0.0))
        if not (
            np.isfinite([self.self_margin, self.detection_distance, self.gain, self.qp_buffer]).all()
            and 0 <= self.self_margin < self.detection_distance
            and self.margin < self.detection_distance
            and 0 < self.gain <= 1
            and 0 <= self.qp_buffer < self.detection_distance - max(self.margin, self.self_margin)
        ):
            raise ValueError("Invalid collision barrier parameters")
        self.shapes = self.profile["shapes"]
        native_shapes = []
        for s in self.shapes:
            if not model.existFrame(s["frame"]):
                raise ValueError(f"Missing collision frame: {s['frame']}")
            if s["type"] not in _TYPES:
                raise ValueError(f"Unknown primitive type: {s['type']}")
            if s["type"] == "convex":
                vertices = np.asarray(s.get("vertices", []), dtype=float)
                faces = np.asarray(s.get("faces", []))
                if (
                    vertices.ndim != 2 or vertices.shape[1:] != (3,) or not 4 <= len(vertices) <= 256
                    or not np.isfinite(vertices).all() or faces.ndim != 2 or faces.shape[1:] != (3,)
                    or not 4 <= len(faces) <= 2 * len(vertices) - 4 or faces.dtype.kind not in "iu"
                    or np.any(faces < 0) or np.any(faces >= len(vertices))
                ):
                    raise ValueError("Invalid convex vertices/faces")
                # Native construction additionally verifies the closed, outward,
                # nondegenerate convex exterior, including every supplied vertex.
            else:
                values = np.asarray(s.get("size", []) if s["type"] == "box" else
                                    [s.get("radius", 0), s.get("length", 0)], dtype=float)
                expected = 3 if s["type"] == "box" else 2
                if values.shape != (expected,) or not np.isfinite(values).all() or np.any(values <= 0):
                    raise ValueError("Invalid primitive dimensions")
            xyz, rpy = np.asarray(s["xyz"]), np.asarray(s["rpy"])
            if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
                raise ValueError("Invalid primitive transform")
            shape = _native.CollisionShape()
            shape.frame, shape.type = s["frame"], _TYPES[s["type"]]
            shape.size = list(s.get("size", [0.0, 0.0, 0.0]))
            shape.radius, shape.length = float(s.get("radius", 0.0)), float(s.get("length", 0.0))
            shape.xyz, shape.rpy = xyz.tolist(), rpy.tolist()
            if s["type"] == "convex":
                shape.vertices, shape.faces = vertices.tolist(), faces.tolist()
            native_shapes.append(shape)
        names = {s["name"] for s in self.shapes}
        frames = {s["frame"] for s in self.shapes}
        if len(names) != len(self.shapes):
            raise ValueError("Duplicate collision shape name")
        if any(len(p) != 2 or not set(p) <= frames for p in self.profile["allowed_link_pairs"]):
            raise ValueError("Unknown allowed collision pair")
        allowed = {frozenset(p) for p in self.profile["allowed_link_pairs"]}
        self.pairs = [
            (i, j)
            for i, a in enumerate(self.shapes)
            for j, b in enumerate(self.shapes[:i])
            if a["frame"] != b["frame"]
            and frozenset((a["frame"], b["frame"])) not in allowed
            and not (a["group"] == b["group"] == "environment")
        ]
        self.margins = np.array(
            [
                self.self_margin if self.shapes[i]["group"] == self.shapes[j]["group"] else self.margin
                for i, j in self.pairs
            ]
        )
        if urdf is None:
            raise ValueError("StationCollision needs the URDF the model was built from")
        self.native = _native.StationCollisionModel(
            str(urdf), native_shapes, [list(p) for p in self.pairs], self.margins.tolist(),
            self.detection_distance, self.gain, self.qp_buffer,
        )
        if self.native.nq != model.nq:
            raise ValueError("Collision URDF does not match the model")
        self._solvers = {}
        self._linearized = None  # (q, velocity_indices) of the last linearize()

    def result(self, native):
        """CollisionResult (with shape names) for a native ClearanceResult."""
        pair = None
        if native.pair >= 0:
            i, j = self.pairs[native.pair]
            pair = (self.shapes[i]["name"], self.shapes[j]["name"])
        return CollisionResult(bool(native.safe), float(native.minimum_distance), pair, float(native.clearance))

    @staticmethod
    def _configuration(q, nq):
        q = np.asarray(q, dtype=float)
        if q.shape != (nq,) or not np.isfinite(q).all():
            raise ValueError("Invalid collision configuration")
        return q

    def placements(self, q):
        return [pin.SE3(np.asarray(m).reshape(4, 4)) for m in self.native.placements(self._configuration(q, self.model.nq))]

    def check(self, q, cutoff=None):
        """Smallest clearance; with ``cutoff`` (m) pairs farther apart report safe/inf."""
        q = self._configuration(q, self.model.nq)
        return self.result(self.native.check(q, -1.0 if cutoff is None else float(cutoff)))

    def linearize(self, q, velocity_indices):
        """Barrier rows, bounds and closest in-band clearance at q, computed once.

        Every QP of a control step reuses this linearisation; verify_step()
        needs it too.
        """
        q = self._configuration(q, self.model.nq)
        rows, bounds, closest, _ = self.native.linearize(q, list(velocity_indices))
        self._linearized = (q.copy(), tuple(velocity_indices))
        return np.asarray(rows), np.asarray(bounds), self.result(closest)

    def compute_qp_inequalities(self, q, velocity_indices):
        """Mink-style displacement barrier: -J_distance dq <= gain*(d-d_min).

        https://kevinzakka.github.io/mink/api/limits.html#collision-avoidance-limits
        Pinocchio point Jacobians replace MuJoCo Jacobians. Environment Jacobians
        are naturally zero; cross-arm rows contain columns for both arms.
        """
        rows, bounds, _ = self.linearize(q, velocity_indices)
        return rows, bounds

    def verify_step(self, start, end):
        """Margin check of the joint segment from the last linearized q to ``end``.

        Whole-interval motion/clearance bounds certify the path, including
        pairs initially outside the detection band. Unresolved intervals are
        subdivided; budget exhaustion returns unsafe so the caller can hold.
        Distances are conservative bounds, not necessarily measured contact.
        Moving collision chains must use scalar revolute/prismatic joints.
        """
        start = self._configuration(start, self.model.nq)
        end = self._configuration(end, self.model.nq)
        return self.result(self.native.verify_step(start, end))

    def project_displacement(
        self, q, proposed, velocity_indices, lower, upper, soft_lower=None, soft_upper=None, metric=None, rows=None
    ):
        """Closest jointly feasible displacement, by the native ProjectionQP.

        ``metric`` (symmetric positive definite, n x n) measures "closest"; a
        task-space metric J^T W J + lambda I keeps the TCP motion the planner
        asked for and drops only what the barrier forbids. ``lower``/``upper``
        (velocity and position) and the distance barrier are hard;
        ``soft_lower``/``soft_upper`` (acceleration/jerk) may be violated at a
        steep cost. Hard rows hold exactly: the solution is scaled toward zero
        displacement if needed, so the bounds must contain 0 (else RuntimeError).
        """
        proposed = np.asarray(proposed, dtype=float)
        n = len(proposed)
        lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
        soft_lower = np.full(n, -np.inf) if soft_lower is None else np.asarray(soft_lower, dtype=float)
        soft_upper = np.full(n, np.inf) if soft_upper is None else np.asarray(soft_upper, dtype=float)
        g_matrix, h = self.compute_qp_inequalities(q, velocity_indices) if rows is None else rows
        g_matrix = np.asarray(g_matrix, dtype=float).reshape(-1, n)
        h = np.asarray(h, dtype=float)
        slack = 1e-9 * max(1.0, float(np.max(np.abs(upper - lower)))) if n else 0.0
        within = np.all(proposed >= np.maximum(lower, soft_lower - slack)) and np.all(
            proposed <= np.minimum(upper, soft_upper + slack)
        )
        if within and np.all(g_matrix @ proposed <= h + 1e-10):
            return proposed, len(h)
        metric = np.eye(n) if metric is None else np.asarray(metric, dtype=float)
        if metric.shape != (n, n) or not np.isfinite(metric).all():
            raise ValueError("Invalid projection metric")
        solver = self._solvers.setdefault((n, len(h)), _native.ProjectionQP())
        return np.asarray(solver.solve(metric, proposed, lower, upper, soft_lower, soft_upper, g_matrix, h)), len(h)

    def check_segment(self, start, end, resolution=0.002, include_start=True, cutoff=None):
        """Certify the entire straight joint path, including thin obstacles.

        ``resolution``, ``include_start`` and ``cutoff`` remain accepted for
        compatibility; no spacing, endpoint or distance cutoff can skip proof.
        Unresolved intervals return unsafe. An existing linearization is
        restored, but this method still mutates native caches: concurrent
        preflight and servo checks must use separate StationCollision objects.
        """
        start = self._configuration(start, self.model.nq)
        end = self._configuration(end, self.model.nq)
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError("Invalid collision segment")
        if cutoff is not None and not np.isfinite(cutoff):
            raise ValueError("Invalid collision cutoff")
        initial = self.check(start)
        if not initial.safe or np.array_equal(start, end):
            return initial
        previous = self._linearized
        try:
            self.linearize(start, [])
            return self.result(self.native.verify_step(start, end, float(resolution)))
        finally:
            if previous is not None:
                self.linearize(*previous)


def station_guard(collision, station_urdf, default_q, tools):
    """Native StationGuard over ``collision`` (a StationCollision) for two arms.

    ``default_q`` is the station configuration for the joints the arms do not
    drive (gripper fingers); ``tools`` are the two arms' tool0 <- TCP transforms.
    """
    return _native.StationGuard(
        collision.native,
        str(station_urdf),
        np.asarray(default_q, dtype=float),
        [np.asarray(tool, dtype=float).reshape(16).tolist() for tool in tools],
    )


class StationLayer:
    """The station collision layer for one assembly, as the simulator and the robot use it.

    ``profile`` is ``"auto"`` (the assembly's collision.json) or a path; ``tools``
    are the two arms' tool0 <- TCP transforms. ``guard`` is the native
    StationGuard both the simulator and the hardware StationCoordinator run.
    """

    SIDES = ("left", "right")
    JOINTS = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )

    def __init__(self, assembly, profile, tools):
        self.assembly = assembly
        self.path = assembly.root / "collision.json" if profile == "auto" else Path(profile)
        self.urdf = assembly.urdf
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.indices = {
            side: [self.model.joints[self.model.getJointId(f"{side}_{name}")].idx_q for name in self.JOINTS]
            for side in self.SIDES
        }
        self.defaults = np.zeros(self.model.nq)
        for name, value in assembly.viewer_joint_defaults().items():
            self.defaults[self.model.joints[self.model.getJointId(name)].idx_q] = value
        self.collision = StationCollision(self.model, self.path, self.urdf)
        self.guard = station_guard(self.collision, self.urdf, self.defaults, tools)
        self._gripper_maps = []
        # Native planning owns collision.native's mutable distance caches.
        # MoveJ preflight owns a separate checker, serialized only with other
        # Python preflights; no whole-path lock can block the 4 ms planner.
        import threading
        self._path_lock = threading.Lock()
        self._path_collision = None

    def configure_grippers(self, calibration, max_age_s=0.1):
        """Require actual gripper samples, with explicitly measured calibration.

        ``calibration[side][driver_joint]`` contains ordered pairs of actual
        calibrated sensor opening and URDF joint coordinate (rad or m). The
        motor's normalized travel is NOT assumed equal to the URDF's range.
        Mimic joints are expanded from the URDF, including multiplier/offset.
        Call once before a StationCoordinator starts; simulators may retain
        their fixed defaults by leaving this method unused.
        """
        import xml.etree.ElementTree as ET

        if self._gripper_maps:
            raise ValueError("Gripper calibration is already configured")
        if not isinstance(calibration, dict) or not calibration:
            raise ValueError(
                "Live collision geometry requires native_collision_gripper_calibration: "
                "{left: {driver_joint: [[sensor_opening, joint_rad], ...]}, right: {...}}. "
                "Measure this mapping on the installed grippers; URDF limits are not sensor calibration."
            )
        if set(calibration) - set(self.SIDES):
            raise ValueError("Unknown gripper calibration side")
        arm_indices = {i for values in self.indices.values() for i in values}
        movable = {
            str(name): self.model.joints[self.model.getJointId(name)]
            for name in self.model.names[1:]
            if self.model.joints[self.model.getJointId(name)].idx_q not in arm_indices
        }
        elements = {j.attrib["name"]: j for j in ET.parse(self.urdf).getroot().findall("joint")}
        mapped = {}
        for side, drivers in calibration.items():
            if not isinstance(drivers, dict) or not drivers:
                raise ValueError(f"Missing driver-joint calibration for {side}")
            for name, values in drivers.items():
                if name not in movable or not name.startswith(side + "_") or elements[name].find("mimic") is not None:
                    raise ValueError(f"Not an independent {side} gripper joint: {name}")
                knots = np.asarray(values, dtype=float)
                if (
                    knots.ndim != 2 or knots.shape[1] != 2 or len(knots) < 2
                    or not np.isfinite(knots).all() or np.any(np.diff(knots[:, 0]) <= 0)
                    or knots[0, 0] > 0 or knots[-1, 0] < 1
                ):
                    raise ValueError(f"Invalid sensor calibration knots for {name}; must span 0..1")
                mapped[name] = (self.SIDES.index(side), knots.copy())
        pending = set(movable) - set(mapped)
        while pending:
            resolved = []
            for name in sorted(pending):
                mimic = elements[name].find("mimic")
                if mimic is None or mimic.attrib["joint"] not in mapped:
                    continue
                side, parent = mapped[mimic.attrib["joint"]]
                knots = parent.copy()
                knots[:, 1] = knots[:, 1] * float(mimic.get("multiplier", "1")) + float(mimic.get("offset", "0"))
                mapped[name] = (side, knots)
                resolved.append(name)
            if not resolved:
                raise ValueError(f"Unmapped movable gripper joints: {sorted(pending)}")
            pending.difference_update(resolved)
        maps = []
        for name in sorted(mapped):
            side, knots = mapped[name]
            joint = movable[name]
            if (
                joint.nq != 1 or joint.nv != 1 or not np.isfinite(knots).all()
                or np.any(knots[:, 1] < self.model.lowerPositionLimit[joint.idx_q])
                or np.any(knots[:, 1] > self.model.upperPositionLimit[joint.idx_q])
            ):
                raise ValueError(f"Gripper calibration exceeds scalar URDF joint limits: {name}")
            maps.append((name, side, knots))
        if not maps:
            raise ValueError("Station has no movable gripper joints to calibrate")
        self.guard.configure_grippers([m[0] for m in maps], [m[1] for m in maps], float(max_age_s))
        self._gripper_maps = maps

    def publish_gripper_snapshots(self, snapshots):
        """Publish both sides atomically; readers never refresh a sensor's age.

        Snapshots expose position, sampled_at (monotonic seconds), sequence and
        valid. Missing/invalid/out-of-calibration values explicitly invalidate
        the native guard; they do not leave an old valid snapshot in its place.
        """
        if not self._gripper_maps:
            raise RuntimeError("Configure gripper calibration before publishing measurements")
        positions, times, sequences, valid = [], [0.0, 0.0], [0, 0], [False, False]
        openings = [float("nan"), float("nan")]
        for i, side in enumerate(self.SIDES):
            sample = snapshots.get(side)
            if sample is not None:
                try:
                    openings[i] = float(sample.position)
                    times[i], sequences[i] = float(sample.sampled_at), int(sample.sequence)
                    valid[i] = bool(sample.valid) and np.isfinite(openings[i]) and sequences[i] > 0
                except (AttributeError, TypeError, ValueError, OverflowError):
                    valid[i] = False
            if sequences[i] < 0 or sequences[i] > 2**64 - 1:
                sequences[i], valid[i] = 0, False
        for _, side, knots in self._gripper_maps:
            opening = openings[side]
            if not valid[side] or not knots[0, 0] <= opening <= knots[-1, 0]:
                valid[side] = False
                positions.append(0.0)
            else:
                positions.append(float(np.interp(opening, knots[:, 0], knots[:, 1])))
        self.guard.update_grippers(positions, times, sequences, valid)

    def station_q(self, left_q, right_q):
        if self._gripper_maps:
            return np.asarray(self.guard.station_q([left_q, right_q]))
        q = self.defaults.copy()
        q[self.indices["left"]] = left_q
        q[self.indices["right"]] = right_q
        return q

    def check_joint_path(self, start, end, resolution=0.002):
        """Margin check along the straight joint path ``start -> end`` ({side: q}).

        A joint-interpolated move (MoveJ) of both arms with one duration follows
        this path when both are commanded together. The native verifier must
        certify the whole interval; ``resolution`` remains a compatibility argument.
        """
        # Freeze one coherent gripper snapshot across both endpoints.
        first = self.station_q(start["left"], start["right"])
        last = first.copy()
        for side in self.SIDES:
            last[self.indices[side]] = end[side]
        with self._path_lock:
            if self._path_collision is None:
                self._path_collision = StationCollision(self.model, self.path, self.urdf)
            return self._path_collision.check_segment(first, last, resolution)
