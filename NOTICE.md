# Sources and adaptation

- Native Pink-style IK and fixed-size active-set QP are adapted from the user's
  libpytianji src/pink_ik.{hpp,cpp}, checkout
  d4788fa83cf13f7b50578834f25971360904d6fd. Defaults retain 500 iterations,
  dt=0.1, damping=1e-3 and frame_gain=0.5. Elite changes: six joints,
  URDF-loaded geometry/limits, matrix tool transform, and strict FK input bounds.
- The Cartesian interpolation/latest-target semantics follow libpytianji;
  the Elite adapter's timed-reset min-jerk profile and station transforms remain.
  This is not the Python Pink package and does not claim full API equivalence.
- Elite SDK vendor/elite_cs_sdk is pinned as a Git submodule at
  c7c425ccfe025c6d18aa64fa2d44974994554c11, under its original MIT license.
  CMake adds the missing cmath include and Boost include/link dependencies
  without modifying the SDK checkout.
- CS66 assets and original licenses/provenance are preserved under
  libpyelite/assets/cs66. The separate joint_limits.yaml is copied from the
  user-designated official ROS2 driver's CS66 configuration. URDF values already
  match that file; no limit was widened and no acceleration value was invented.

No robot calibration or physical commissioning is claimed by the offline tests.

- `libpyelite/sim.py` reuses/adapts XYZ/RPY helpers and Qt slider widgets from
  the user's `lerobot-xensehand/src/lerobot/robots/tianji_arm/tianji_sim.py`.
  The model, native planner integration, diagnostics and validation traces are
  Elite-specific; no Tianji runtime dependency is introduced.

- Stations now snapshot the current xense-description TacCap variants with absolute
  arm angles. Source assemblies/components and their hashes remain unmodified.
  `assembly_taccap_identified.urdf` is an explicitly derived, aggregate-tool model
  using the user's lerobot-xensehand README identification (0.7728457285779433 kg).
  Effective inertia conventions and non-physical triangle-inequality residual are
  documented in `taccap_identification.json`; no dynamics validation is claimed.

Ruckig 0.15.3 (https://github.com/pantor/ruckig/tree/v0.15.3), MIT licensed,
is vendored under vendor/ruckig. Only offline C++ trajectory generation is built;
no cloud client or Python binding is compiled. The license is packaged in the wheel.
Collision displacement inequalities follow Mink's published normal-distance limit
(https://kevinzakka.github.io/mink/api/limits.html); implementation uses Pinocchio
point Jacobians, Coal/FCL primitives and OSQP, without importing MuJoCo or Mink.
