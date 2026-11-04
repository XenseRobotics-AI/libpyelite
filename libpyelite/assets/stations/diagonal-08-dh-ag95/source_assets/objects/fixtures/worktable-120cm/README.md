# Worktable 120 cm × 70 cm

Viewer: [urdf/worktable_120cm.urdf](urdf/worktable_120cm.urdf).

Generated from the hash-locked STEP with `scripts/importers/import_diagonal_02.py`; details and occurrence transforms are in `provenance/step_derivation.json`. Coordinates are metres, X toward the worktable, Y left, Z up.

Table top length 1.20 m, depth 0.70 m, top height 0.75 m; origin is on the floor beneath the table centre. The source table includes its legs.

The default entry includes the reviewed nine CAD-derived collision primitives. No mass or inertia has been estimated; geometry is described below. Visual and mount-interface acceptance remain pending human review.

## 普通入口的碰撞几何

普通入口：[urdf/worktable_120cm.urdf](urdf/worktable_120cm.urdf)。同时包含 visual 和 collision；`collision` 资产 ID 仅为兼容别名，不再维护第二份活动 URDF。原视觉输入冻结在 `provenance/visual_source.xml`。

碰撞体由 4 条 CAD 圆柱桌腿、1 个桌面长方体和 4 个桌面下框架长方体组成，保留桌下空间。桌面 R50 圆角被长方体覆盖，角点最大横向多覆盖约 20.71 mm；不适合直接作为圆角精确接触模型。

依据在 `provenance/collision_cad_measurements.json` 和 `collision_recipe.json`；结果、源哈希及采样覆盖检查在 `collision_derivation.json`。复现：在仓库根运行 `python scripts/generators/generate_diagonal07_collisions.py`，只检查用 `--check`。

这些几何用于避障候选审查，不提供质量、惯量、摩擦或接触标定。采样覆盖通过不等于连续 CAD 表面或实机安全验证。详见 [07 碰撞版说明](../../../assemblies/diagonal-07-elite-cs66/README.collision.md)。
