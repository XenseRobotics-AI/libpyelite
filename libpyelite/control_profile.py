"""One native configuration mapping shared by the LeRobot adapter and simulator.

No drivers are constructed. TCP, timing, tolerances and numerical IK settings
come from the native option objects; recipe overrides pass through the same code.
"""

from pathlib import Path

import numpy as np

from . import EliteDriverConfig, load_compliance_config

STANDARD_GRAVITY = 9.80665

IK_FIELDS = (
    "max_iterations",
    "dt",
    "damping",
    "step_size",
    "max_step_norm",
    "position_tolerance",
    "orientation_tolerance",
    "streaming_position_tolerance",
    "streaming_orientation_tolerance",
    "position_cost",
    "orientation_cost",
    "frame_gain",
    "posture_weight",
    "posture_gain",
)
PLANNER_FIELDS = (
    "period",
    "stale_timeout",
    "max_linear_velocity",
    "max_angular_velocity",
    "velocity_scale",
    "joint_motion_limits",
    "joint_target_horizon",
    "max_joint_velocity",
    "max_joint_acceleration",
    "max_joint_jerk",
    "max_target_translation_jump",
    "max_target_rotation_jump",
    "max_target_joint_jump",
    "joint_tracking_gain",
    "cartesian_motion_limits",
    "max_linear_acceleration",
    "max_angular_acceleration",
    "max_linear_jerk",
    "max_angular_jerk",
    "cartesian_tracking_gain",
)


def contact_monitor_config(values, side=None, world_from_base=None):
    """Joint-current contact monitor from ``native_contact_monitor``, then ``<side>_native_contact_monitor``.

    Returns None when it is not configured or ``enabled`` is false. The model
    gravity defaults to standard gravity in the arm base frame, from the mount
    rotation; a probe_force_noise.py fit (``gravity``) overrides it. The payload
    defaults to the recipe's ``payload_mass``/``payload_cog`` declared to the
    controller, because the measured motor torque carries the tool's weight.
    """
    merged = dict(values.get("native_contact_monitor") or {})
    if side is not None:
        merged.update(values.get(f"{side}_native_contact_monitor") or {})
    if not merged or not merged.pop("enabled", True):
        return None
    if "gravity" not in merged:
        rotation = None
        if world_from_base is not None:
            rotation = np.asarray(world_from_base, dtype=float)[:3, :3]
        elif side is not None and values.get(f"{side}_world_rotation") is not None:
            rotation = np.asarray(values[f"{side}_world_rotation"], dtype=float)
        if rotation is None:
            raise ValueError("native_contact_monitor needs 'gravity' (base frame) or the arm mount rotation")
        merged["gravity"] = (rotation.T @ [0.0, 0.0, -STANDARD_GRAVITY]).tolist()
    if "payload_mass" not in merged and values.get("payload_mass") is not None:
        merged["payload_mass"] = values["payload_mass"]
        merged.setdefault("payload_cog", values.get("payload_cog", [0.0, 0.0, 0.0]))
    return load_compliance_config(None, **merged)


# Adapter default (BiEliteCS66RTConfig / EliteCS66RTConfig). The C++ DriverConfig
# keeps 1 for direct callers; test_real_adapter_and_simulator_use_same_native_parameters
# catches drift from the adapter dataclass default.
SERVO_FAILURE_TOLERANCE_TICKS = 250


def configure_native_driver(cfg, settings, tool=None, side=None, world_from_base=None):
    values = settings if isinstance(settings, dict) else vars(settings)
    if side is not None:
        cfg.planner.arm_name = side
    cfg.ik.max_iterations = values.get("native_ik_max_iterations", cfg.ik.max_iterations)
    cfg.servoj_time = values.get("servoj_time", cfg.servoj_time)
    cfg.planner.period = cfg.servoj_time
    cfg.planner.stale_timeout = values.get("command_stale_timeout_s", cfg.planner.stale_timeout)
    cfg.planner.max_linear_velocity = values.get("native_max_tcp_linear_velocity", cfg.planner.max_linear_velocity)
    cfg.planner.max_angular_velocity = values.get("native_max_tcp_angular_velocity", cfg.planner.max_angular_velocity)
    for key, field in (("max_lin_speed", "max_linear_velocity"), ("max_ang_speed", "max_angular_velocity")):
        if values.get(key) is not None:
            setattr(cfg.planner, field, min(getattr(cfg.planner, field), values[key]))
    cfg.planner.velocity_scale = values.get("native_joint_velocity_scale", cfg.planner.velocity_scale)
    for key in (
        "joint_motion_limits",
        "joint_target_horizon",
        "max_joint_velocity",
        "max_joint_acceleration",
        "max_joint_jerk",
        "max_target_translation_jump",
        "max_target_rotation_jump",
        "max_target_joint_jump",
        "joint_tracking_gain",
        "cartesian_motion_limits",
        "max_linear_acceleration",
        "max_angular_acceleration",
        "max_linear_jerk",
        "max_angular_jerk",
        "cartesian_tracking_gain",
    ):
        if values.get("native_" + key) is not None:
            setattr(cfg.planner, key, values["native_" + key])
    for key in ("check_robot_safety", "max_contact_wrench", "max_start_joint_speed"):
        if values.get("native_" + key) is not None:
            setattr(cfg, key, values["native_" + key])
    contact = contact_monitor_config(values, side, world_from_base)
    if contact is not None:
        cfg.contact_monitor = contact
    cfg.command_timeout_ms = values.get("command_timeout_ms", cfg.command_timeout_ms)
    cfg.max_consecutive_write_failures = values.get(
        "servo_failure_tolerance_ticks", SERVO_FAILURE_TOLERANCE_TICKS
    )
    if values.get("native_model_path") is not None:
        cfg.model_path = str(Path(values["native_model_path"]).expanduser())
    if tool is not None:
        cfg.tool = np.asarray(tool, dtype=float).reshape(16).tolist()


def native_config(settings, side, assembly, tool=None):
    cfg = EliteDriverConfig()
    configured_tool = settings.get(f"{side}_tool_transform") if tool is None else tool
    configure_native_driver(cfg, settings, configured_tool, side, assembly.world_from_base(side))
    cfg.model_path = assembly.control_model_path(side)
    return cfg


def describe_config(cfg):
    return {
        "ik": {key: getattr(cfg.ik, key) for key in IK_FIELDS},
        "streaming_iteration_budget": cfg.ik.streaming_iteration_budget,
        "planner": {key: getattr(cfg.planner, key) for key in PLANNER_FIELDS},
        "safety": {
            key: getattr(cfg, key)
            for key in (
                "check_robot_safety",
                "max_contact_wrench",
                "max_start_joint_speed",
                "feedback_timeout",
                "planner_timeout",
                "command_timeout_ms",
                "max_consecutive_write_failures",
            )
        },
        "contact_monitor": {
            key: (lambda v: list(v) if not isinstance(v, (bool, int, float)) else v)(getattr(cfg.contact_monitor, key))
            for key in (
                "enabled", "gravity", "payload_mass", "payload_cog", "friction_coulomb", "friction_viscous",
                "torque_band", "collision_threshold", "transient_factor", "transient_window", "bias_time",
            )
        },
        "tool": np.asarray(cfg.tool).reshape(4, 4).tolist(),
        "model_path": cfg.model_path,
    }


def load_recipe(path):
    import yaml

    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or not isinstance(data.get("robot"), dict):
        raise ValueError("Expected a recipe with a robot mapping")
    if data["robot"].get("type") != "bi_elite_cs66_rt":
        raise ValueError("Station simulator expects robot.type=bi_elite_cs66_rt")
    return data["robot"]
