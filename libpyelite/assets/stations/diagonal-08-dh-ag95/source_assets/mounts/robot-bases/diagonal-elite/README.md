2026-10-15：普通 `urdf/elite_cs66_a1_stand.urdf` 已同时包含原视觉模型和已审查碰撞体。`collision` ID 为同一文件的兼容别名，不再维护 `_collision.urdf`。源视觉输入保存在 `provenance/visual_source.xml`；未新增拟合或物性。

# diagonal-elite

Default entry: visual and reviewed avoidance-collision geometry for the support, four casters and two controller cabinets from the hash-locked A1 STEP. Two fixed CS66 mount frames preserve source assembly poses. Arms and the 120 cm table are reused as separate catalog assets.

See `provenance/step_derivation.json` for occurrence mapping, source units, coordinate transform and table position. The original visual-only input is preserved in provenance/visual_source.xml; no dynamics were introduced. Calibration remains unvalidated.

## 普通入口的碰撞几何

普通入口：[urdf/elite_cs66_a1_stand.urdf](urdf/elite_cs66_a1_stand.urdf)，同时包含 visual/collision；`collision` 资产 ID 仅为同一文件的兼容别名。

碰撞体采用 STEP 圆柱面测量得到的 5 个圆柱、4 个独立加强筋凸包，以及其余 7 个零件各自的包围盒。圆管内部、螺孔、脚轮和控制柜支脚周围的局部空隙会被保守覆盖。

依据在 `provenance/collision_cad_measurements.json` 和 `collision_recipe.json`；结果、源哈希及采样覆盖检查在 `collision_derivation.json`。复现：在仓库根运行 `python scripts/generators/generate_diagonal07_collisions.py`，只检查用 `--check`。

这些几何用于避障候选审查，不提供质量、惯量、摩擦或接触标定。采样覆盖通过不等于连续 CAD 表面或实机安全验证。详见 [07 碰撞版说明](../../../assemblies/diagonal-07-elite-cs66/README.collision.md)。
