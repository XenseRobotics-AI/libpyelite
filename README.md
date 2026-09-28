# libpyelite

Native CS66 bridge adapted from libpytianji's Pink-style IK and latest-target
arrival-time interpolation. The six-variable box QP uses Pinocchio; Python Pink
and qpsolvers are not invoked by the worker. Default maximum IK iterations: 500.

Python owns LeRobot/VR/VLA logic, installation rotations, cameras and grippers.
Each arm has native planner and sender threads. The sender calls the Elite SDK
in **joint** servoJ mode, never controller Cartesian IK. Default period: 4 ms.
The SDK also retains its native RTSI receiver. No claim of hard real-time:
the vendor socket write is synchronous and OS/network latency remains relevant.

## Build

Initialize the SDK submodule (the parent checkout currently registers its nested
path; .gitmodules also describes it for extraction as a standalone repository):

```sh
git submodule update --init third_party/libpyelite/vendor/elite_cs_sdk
python -m pip wheel ./third_party/libpyelite --no-deps -w dist
python -m pip install dist/libpyelite-0.1.0-*.whl
```

Linux, C++17 compiler and CMake are required. Build/runtime Pinocchio is pinned to
4.1.0 to avoid silently mixing native ABIs. The SDK is linked statically; its
external-control script and the model/meshes/licenses ship in the wheel.
No ROS, Gazebo or elite_cs_sdk Python package is needed.

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
min-jerk reset profile. Failed IK holds the previous valid command and reports
diagnostics. Stale input goes idle; fresh targets re-anchor to feedback.
Position bounds constrain the IK QP; velocity bounds apply both after IK and
again in the sender. No acceleration/jerk guarantee is claimed.

Native workers acquire no Python GIL and call no Python callbacks. moveJ uses a
native callback/keepalive loop with the GIL released. stopServo joins workers;
it is not a hardware emergency stop. Controller safety mechanisms remain active.
Disconnect/home behavior remains the existing LeRobot behavior.

## Offline validation

```sh
python -m unittest discover -s third_party/libpyelite/tests -v
```

Tests must not connect to hardware. Real-robot commissioning (TCP calibration,
mount-axis checks, initial-pose transfer, stale-command behavior, dual-arm timing)
is a separate supervised step.
