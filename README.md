# libpyelite

Native CS66 bridge adapted from libpytianji's Pink-style IK and latest-target
arrival-time interpolation. The six-variable box QP uses Pinocchio; Python Pink
and qpsolvers are not invoked by the worker. Offline IK defaults to 500 iterations;
streaming tracking uses at most 8 bounded iterations per planner tick.

Python owns LeRobot/VR/VLA logic, installation rotations, cameras and grippers.
Each arm has native planner and sender threads. The sender calls the Elite SDK
in **joint** servoJ mode, never controller Cartesian IK. Default period: 4 ms.
The SDK also retains its native RTSI receiver. No claim of hard real-time:
the vendor socket write is synchronous and OS/network latency remains relevant.

## Build

Initialize libpyelite and its pinned SDK submodule from the LeRobot checkout:

```sh
git submodule update --init --recursive third_party/libpyelite
python -m pip wheel ./third_party/libpyelite --no-deps -w dist
python -m pip install dist/libpyelite-0.1.0-*.whl
```

Linux, C++17 compiler and CMake are required. Build/runtime Pinocchio is pinned to
4.1.0 to avoid silently mixing native ABIs. The SDK is linked statically; its
external-control script and the model/meshes/licenses ship in the wheel.
No ROS, Gazebo or elite_cs_sdk Python package is needed.

The bridge also exposes SDK `setPayload`, `getPrimaryPackage`, and
`KinematicsInfo` for LeRobot's payload configuration and optional DH-based
motion guards. These calls do not replace the native Pink IK planner.

## Model and limits

assets/cs66 is copied from xense-description, preserving its upstream provenance.
joint_limits.yaml is the user's designated official CS66 configuration from
Elite_Robots_CS_ROS2_Driver/eli_cs_robot_description/config/cs66. Its limits
already match the supplied URDF: J3 +/-pi, other joints +/-2pi, all speeds pi
rad/s. No numeric change was necessary. Acceleration limits are explicitly
unavailable in that source; this package does not invent a rated acceleration.
Inertia is not used by this kinematic IK. Installation rotations are NOT baked
into the URDF: the existing Elite LeRobot adapter applies them once.

Public planner poses: 4x4 row-major SE(3), m/rad. Elite RTSI feedback is converted from its controller [x,y,z,roll,pitch,yaw] boundary into the internal rotation-vector representation before model seeding and before LeRobot publishes poses. Tool transform: tool0 <- active TCP.
The configured tool must match the controller. Startup checks model FK against
measured TCP (default 5 mm / 0.05 rad); mismatch refuses servo startup instead
of estimating calibration from a single pose. Verify multi-pose FK on hardware.
Current station start/home q and mount rotations remain in the LeRobot config.

## Streaming contract

submit_target replaces pending/uncompleted ordinary targets. Rebase starts from
the current planned pose, duration follows producer arrivals and TCP speed caps.
Timed reset owns the planner until completion or cancel, preserving Elite's
min-jerk reset profile and strict convergence requirement. Streaming IK accepts
error-reducing partial progress even before reaching pose tolerance. Its total
joint displacement is bounded inside the QP by one servo cycle's velocity
allowance; nonlinear backtracking rejects steps that increase task error.
No-progress/QP failures hold the previous command for that tick. Stale input
goes idle; fresh targets re-anchor to feedback. Waypoints are capped relative to
achieved FK so a blocked target cannot accumulate an elapsed-time catch-up jump.
Position bounds constrain the IK QP; velocity gates remain in planner and sender.
Diagnostics distinguish `success`, `progress`, and `qp_failed`; `ik_failures`
counts held failed ticks (not all partial-convergence ticks). No acceleration/jerk guarantee is claimed.

Native workers acquire no Python GIL and call no Python callbacks. moveJ uses a
native callback/keepalive loop with the GIL released. stopServo joins workers;
it is not a hardware emergency stop. Controller safety mechanisms remain active.
Disconnect/home behavior remains the existing LeRobot behavior.

### Target jump diagnostics

Jump rejections retain the `Cartesian target jump rejected` / `Joint target jump
rejected` prefixes and include the arm, rejected sequence and last accepted
sequence. Cartesian errors report the effective `native_max_target_translation_jump`
(m) and `native_max_target_rotation_jump` (rad) recipe limits, both measured deltas,
and an `exceeded=[...]` list for each comparison. A zero limit is marked disabled.
`previous_reference` says whether `previous` uses the previous accepted target or,
when no active previous target exists, the current planned pose. `planned_pose`
is the FK of the planner output, not a fresh measured TCP sample. The streaming
guard still requires **both** comparisons to exceed at least one enabled limit;
timed resets still bypass this jump guard. Joint errors name every offending
axis (`J1`..`J6`), its absolute delta, target and reference, and the effective limit.

`configure_native_driver(..., side="left"/"right")` sets `cfg.planner.arm_name`
for diagnostics, using the side already supplied by the LeRobot adapter. Direct
planner/driver callers can set `PlannerConfig.arm_name` themselves; an unset name
is reported as `arm=unspecified`. The source arm stays in the exception when the
station coordinator forwards a failure to both drivers. These changes require
rebuilding and reinstalling the native extension, with no LeRobot source changes.

## Offline validation

```sh
python -m unittest discover -s third_party/libpyelite/tests -v
```

Tests must not connect to hardware. Real-robot commissioning (TCP calibration,
mount-axis checks, initial-pose transfer, stale-command behavior, dual-arm timing)
is a separate supervised step.

## Qt + Meshcat IK sandbox

The sandbox lives entirely in `libpyelite.sim`, with no LeRobot dependency.
It reuses the Tianji simulation's dual-arm FK/IK tabs, joint/XYZ-RPY sliders,
left/right/both buttons and Meshcat layout, executing the **production C++ Planner and IK**.
There is no independent Python IK and no cosmetic interpolation of IK results.
Each render advances four 4-ms simulated ticks. Slider edits immediately replace
the latest target and are submitted on the next simulated tick; unchanged targets
receive a 32-ms heartbeat. There is no wait-for-stop debounce on slider dragging. Slow rendering slows simulation time instead of catching up.

```sh
python -m pip install './third_party/libpyelite[sim]'
python -m libpyelite.sim
# Equivalent installed entry point: elite-ik-sim
python -m libpyelite.sim --headless-check --report /tmp/elite-ik-report.json
QT_QPA_PLATFORM=offscreen python -m libpyelite.sim --no-show --no-browser
```

The default viewer loads the full **diagonal-08 station, both arms and table**
with TacCap grippers, derived from the byte-preserved source
`assets/stations/source_assets/assemblies/diagonal-08-elite-cs66/urdf/diagonal-08-elite-cs66-taccap.urdf`.
The default viewer file is `assets/stations/diagonal-08/assembly_taccap_identified.urdf`.
The FK tab has left/right joint panels and apply buttons; the IK tab has
left/right TCP panels and left/right/both solve buttons, matching Tianji's UI.
Both arms are displayed simultaneously and each runs its own native planner.
Joint sliders and the new assembly both use absolute controller angles (degrees in UI).
There is no subtraction of initial_pose. Finger mimic coordinates are initialized
together from the source start pose; the UI controls only the twelve arm joints. XYZ/RPY targets use
**station world coordinates**, transformed to each base before native IK.
Target markers are red (left) and blue (right), with separate actual TCP frames.
No visual smoothing is added to the planned joint states.

Optional `--initial-q` takes twelve degrees (left six, then right six).
`--tool x y z roll pitch yaw` takes metres/radians and defines tool0 to active
TCP for both arms. `--station diagonal-07` selects the other packaged station.
`--no-meshcat` runs only Qt. `--headless-check` exercises the single-arm solver
with default TCP/seeds and needs neither Qt nor Meshcat; `--urdf` applies only
to that single-arm check. Dual-arm mapping/independence tests are in
`tests/test_station_sim.py`.

The automatic trace checks small streaming motion, an unreachable target, return
to the original target, and a near-singular wrist seed. These are ideal kinematic
simulations: they do not model servo lag, collision, dynamics, VR coordinate
mapping, or acceleration/jerk limits. No robot connection is made.

### Why the previous tracker could freeze

The previous planner discarded *every* IK result outside 0.1-mm/1-mrad tolerance
and stopped the target. A one-iteration test with a 2-cm target reproduces zero
motion even though the numerical step reduces error. New VR targets could then
resume movement, causing a stop-and-catch-up sensation. This is a reproduced
software mechanism, not proof of the cause of a particular hardware incident.

Pink is differential IK: its velocity step need not solve an entire pose in one
call. Six joints (including UR5) do not imply every SE(3) target is reachable or
that a local solver can cross every singularity/IK branch. In this box-QP, zero
velocity is feasible inside position bounds; a remaining Cartesian residual is
not evidence that the QP itself is infeasible. See the
[Pink tracking description](https://github.com/pink-kinematics/pink) and
[discussion of differential versus global IK](https://github.com/stephane-caron/pink/discussions/66).
The existing CS66 position/velocity limits and convergence tolerances are retained.

### Conda / pip Qt runtime conflicts

If PySide6 is installed but startup reports `libQt6DBus.so.6: undefined symbol`
with `Qt_6_PRIVATE_API`, a Conda Qt library is being mixed with the pip wheel's
Qt libraries. Reinstalling the same dependencies alone does not fix this.
The simulator CLI now restarts itself on Linux with the bundled PySide6 `Qt/lib`
first in `LD_LIBRARY_PATH`, preserving other entries. This affects only the
simulator process; it does not change shell settings or the Conda installation.
Both `python -m libpyelite.sim` and `elite-ik-sim` use this setup. Headless checks
skip Qt entirely. Reinstall the updated package from the repository root:

```sh
python -m pip install './third_party/libpyelite[sim]'
python -m libpyelite.sim
```

For an older installed simulator, this command applies the same workaround:

```sh
ELITE_QT_LIB="$(python -c 'import pathlib, PySide6; print(pathlib.Path(PySide6.__file__).parent / "Qt" / "lib")')"
LD_LIBRARY_PATH="$ELITE_QT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" python -m libpyelite.sim
```

Do not use `pip install -e 'libpyelite[sim]'`: editable installs need a local
project path, such as `'./third_party/libpyelite[sim]'`.

Run from the **LeRobot repository root**, not `third_party/libpyelite`:

```sh
cd /home/olorin/Olorin/01_Elite/lerobot-xense
python -m libpyelite.sim
```

Inside `third_party/libpyelite`, the source `libpyelite/` directory shadows the
installed wheel, so `_native` cannot be imported without an in-place build.
Alternatively, run the installed `elite-ik-sim` entry point from any directory.
The Linux CLI also sets `QT_PLUGIN_PATH` and `QT_QPA_PLATFORM_PLUGIN_PATH` to
PySide6's bundled plugins so stale Conda/OpenCV paths cannot hide `xcb`.

XYZ sliders default to a local ±0.3 m interval around each initial world TCP.
Use `--xyz-range 0.15` for a finer ±0.15 m range. Explicit FK edits outside the
interval recenter that axis. This is a UI range, not a certified reachable
workspace: XYZ and orientation cannot be constrained independently to guarantee IK.
As in the Tianji/tron2-ik UI, the status text is green for success and red for
nonconvergence/exceptions (not a modal dialog). Success is measured against the
**user target**, not an intermediate waypoint or the QP result. Red text distinguishes
tracking progress, no progress/possible unreachable target, and QP failure.
Local numerical nonconvergence alone is not proof of global infeasibility.

## Native configuration parity and TacCap payload

The simulator and real Elite adapter now call the same `control_profile.configure_native_driver`
function. Native IK/QP settings and the streaming iteration-budget getter come from C++.
The shared planner still performs at most 8 iterations/tick in streaming mode and up to
500 in offline/timed-reset mode. A window label and `--show-config` expose the distinction.

```sh
python -m libpyelite.sim --recipe recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml
python -m libpyelite.sim --recipe recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml --show-config
```

Without `--recipe`, a packaged snapshot of the relevant station recipe fields is used,
with its source SHA-256. The bimanual adapter defaults are 1.0 m/s, 3.14 rad/s,
1.0-s command stale timeout; these are distinct from the standalone native defaults.
Bench 08 uses its recipe-calibrated TCP [0.00190, -0.04363, 0.21187] m, not identity.
`--tool` and `--initial-q` are explicit testing overrides and can break recipe parity.
The GUI simulates the native planner, not controller servo lag or the LeRobot adapter's
optional feedback-DH target guards. Equal native parameters do not claim full plant parity.

The identified TacCap aggregate tool mass is 0.7728457285779433 kg per arm, copied from
`lerobot-xensehand/README.md`. Source text/hash and all ten original values are preserved
in `assets/stations/taccap_identification{.json,_source.md}`. With the user's approval,
the derived URDF removes per-tool/per-finger inertias and records one equivalent rigid
payload at each adapter mount; original assembly/component files remain unchanged.
COM millimetres are converted to metres. Source inertia is recorded as an effective
COM tensor in mount axes, **not a validated inertia/frame transfer**. The input tensor
is positive definite but slightly violates the rigid-body triangle inequality; it is
not silently projected or repaired. `dynamics_validated=false` documents this limitation.
This is a kinematic/geometry validation model, not validated finger/grasp dynamics.
The change does not write any robot controller payload or TCP setting.

Reproduce this snapshot into an empty destination:

```sh
python third_party/libpyelite/scripts/import_assemblies.py /path/to/xense-description \
  --identification-readme /path/to/lerobot-xensehand/README.md \
  --destination /tmp/elite-stations-new
```

Visual and collision models must be distinguished. The current assembly has 22 collision
objects (~334800 triangles) on arms/tools, while the table/stand currently have no collision
geometry. Future distance constraints should use conservative simplified link/finger
shapes and explicit table/stand shapes in the same assembly frames, with the source mesh
as an offline reference. Collision avoidance and acceleration/jerk limiting are not added
by this asset/configuration update. IK damping remains 0.001.

### 可配置运动限制与碰撞仿真

新增关节速度/加速度/jerk、突变阈值、反馈故障和接触力检查接口，以及独立 JSON
几何配置的双臂碰撞仿真。启动命令、参数、模型更新流程和适用范围见
[安全仿真说明](docs/safety_simulation.md)。示例参数在 `examples/sim_safety.json`，
只用于仿真调参；碰撞 QP 尚未接入真机双臂实时线程。
