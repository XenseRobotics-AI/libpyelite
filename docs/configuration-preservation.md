# CS66 workspace boundaries and saved configuration

`PlannerConfig.preserve_configuration` is opt-in in the native SDK. The
`elite_cs66_rt` and `bi_elite_cs66_rt` adapters and their supplied recipes enable
it. No shared LeRobot teleoperation, recording, or other robot code is involved.

## Session configuration

The first successful measured seed captures each arm's own controller joint
coordinates. Later idle reseeds, cancellation and grip anchors validate that
configuration and do not replace it. A new connection creates a new session.
The diagonal-07/08 assemblies specify `q_assembly = q_controller`: mirrored
mounts are proper fixed rotations, so no right-arm sign inversion is applied.

The native model validates the six-joint CS66 topology, axes, local placements,
and terminal TCP attachment before using the analytic policy. Unknown models,
nonfinite inputs, and initial configurations inside an excluded singular margin
are rejected explicitly.

For the supplied chain, the singular factors are:

```
E = sin(q3)
W = sin(q5)
S = -L2*cos(q2) - L3*cos(q2+q3) + h*sin(q2+q3+q4)
L2 = 0.427 m; L3 = 0.3905 m
h = d5 - d6*cos(beta) ≈ 0.0965 m
```

The saved signs of E, W and S cannot change along an accepted command. Elbow
and wrist remain in their initial half-turn, with a default 3-degree margin.
S retains a 0.1 mm signed margin. Every joint also remains within the initial
unwrapped angle ±(π − 0.001), intersected with physical URDF bounds.
These margins are tunable engineering defaults, not manufacturer-certified
clearances or a bound on the Jacobian's smallest singular value.

The policy certifies complete joint rectangles using analytic trigonometric
intervals and Taylor bounds. Checking only the endpoint signs is insufficient:
a joint segment can leave and reenter the same shoulder branch. Certificates
are conservative and may reject a motion whose broad rectangle cannot be
proved valid.

## Boundary behavior

The IK starts from its last accepted reference, solves within one bounded local
joint region, and accepts only candidates that improve the weighted nonlinear
position/orientation objective. A useful partial improvement remains usable
even when full convergence is impossible. The `progress` diagnostic refers to
that pose objective, not position alone.

An unreachable target is retained as the user's absolute request. Unachieved
internal Cartesian interpolation is fed back to the accepted reference, so
holding the controller outside the workspace does not accumulate extra internal
catch-up motion. Reference lead is bounded relative to the executed joint
trajectory. Returning to a reachable target can resume motion without resetting
the session. Grip anchoring remains the adapter's existing measured-TCP path.

`boundary_orientation_tolerance` (default 0.01 rad) limits the IK reference's
translation/orientation trade-off. A new orientation request may initially have
a larger error; accepted IK steps must not increase that error. This parameter
is not a hard bound on transient TCP orientation after per-joint acceleration
and jerk limiting, or on physical servo tracking. A temporary joint-space
braking orientation must never redefine the IK reference or its allowance.

Before accepting a limited trajectory, the limiter checks its complete
polynomial position extrema and a feasible stop from the next sample. It
brakes before leaving the saved configuration instead of clipping joint
position or zeroing its velocity/acceleration. The bundled Ruckig velocity
profiles do not initialize a final position field; the guard calculates extrema
directly from their segments, including constant-acceleration extrema and the
continuation to a servo sample.

Station collision projection remains active. The final projected command is
also checked against the saved configuration. An external station-approved
sample cannot be replaced by an independent sender trajectory: an infeasible
continuation faults before the socket write. Collision and configuration
constraints may make a requested motion unavailable.

Station output uses one outstanding sample per arm. The coordinator waits for
both senders' successful socket writes before advancing either planner; an
independent sender clock must not skip an approved step. Repeated reads and
failed writes retain that sample. Live retries keep the handoff watchdog alive
while the configured consecutive-write-failure budget remains owned by the
sender. A stalled sender or planner still faults. Cancellation and the final
braking sample pass through the same station gate before idle.

The sender's hard velocity comparison allows only floating-point subtraction
roundoff at the exact speed limit. It does not relax the physical speed cap by
a percentage. Rejections report position, velocity, configuration-segment and
stopping-envelope results with joint states, so these failure modes are distinct.

## Returning to the start

Keeping singular-surface signs and winding does not establish global IK
uniqueness for every closed Cartesian path. The Elite adapter's explicit reset
therefore targets the saved measured six-joint vector, independently for each
arm, through the native motion limits and configured station collision guard.
Success requires fresh measured joint-angle and velocity tolerances for a
continuous dwell; TCP equality, nominal duration, and angles modulo 2π are not
success criteria. A blocked or timed-out reset reports failure.

## Offline validation

`tests/test_configuration_policy.cpp` checks policy bounds, singular references,
nonfinite inputs, a double shoulder crossing, and random rectangle/segment
certificates. `tests/test_configuration_geometry.py` checks the analytic
Jacobian determinant and both mirrored assemblies. The tracking regressions
exercise outward/return motion on both elbows, prolonged boundary holds,
regrip recovery, low-jerk braking, stationary velocity profiles, rejected
configuration changes, repeated station extension/return with the sender gate
enabled, exact speed-limit samples, and station sender refusal. The station
simulator now runs that final sender gate as well as collision projection.

`tests/test_command_handoff.cpp` injects asynchronous dual senders, delayed and
repeated reads, write failures, stale acknowledgements, timeouts and shutdown.
It can be built and run without the SDK or robot:

```sh
c++ -std=c++17 -O2 -pthread tests/test_command_handoff.cpp -o /tmp/test_command_handoff
/tmp/test_command_handoff
```

The Python adapter tests separately exercise measured reset completion,
freshness, cancellation, command ownership, mirrored startup snapshots and the
existing recording/teleoperation handshake. These are offline checks; no
physical controller or robot is contacted.
