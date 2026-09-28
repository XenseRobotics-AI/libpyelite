# Elite CS66 source and modification notice

Source: EliteRobots/Elite_Robots_CS_ROS2_Description
https://github.com/EliteRobots/Elite_Robots_CS_ROS2_Description/tree/edbbfdca9220ce264c99ec51d37440ff4fa13a9a

The upstream repository declares Apache License 2.0. Its complete LICENSE is
preserved in LICENSES/elite_robots_description.txt. Upstream package metadata
(including maintainers/license declaration) is preserved as
provenance/upstream/package.xml.txt, outside ROS package discovery.
These third-party assets are not relicensed under the host repository's MIT license.

Original CS66 meshes, YAML, and the Xacro macros used for generation are copied
without modification. Per-file source paths, SHA-256 and Git blob SHA-1 hashes
are in asset.json. Only this model's required mesh resources are included.

Local additions: standalone Xacro invocation, deterministic offline expansion,
relative mesh paths in the viewer URDF, manifest and documentation. The expanded
provenance URDF is a locally generated reference, not an upstream-shipped file.
Hardware/control and simulator integration are disabled by macro arguments;
kinematic, inertial and geometry parameters retain upstream values.

Upstream explicitly cautions that base mass, inertia and center of gravity
might be incorrect. No physical calibration or contact validation is claimed.
