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

## 本地派生 USD（2026-10-14）

`usd/` 由锁定官方 URDF/mesh 经 URDF USD Converter 0.3.3 生成，不是厂家发布物。
`SourceSemantics.usda` 记录碰撞近似及基座语义覆盖；原网格、关节和物性输入未修改。
不增加来源未提供的密度、驱动、接触或夹爪耦合。转换工具的许可证不改变源资产许可。
方法、输入/环境/输出哈希和验证范围见 `provenance/usd/conversion.json` 与 `validation.json`。
转换脚本不随本仓库或资产分发，动力学、接触和硬件准确性未验收。
