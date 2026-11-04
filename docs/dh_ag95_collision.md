# 在 08 工作台测试 DH AG95 碰撞

此模型用于 `xense-motion-viewer` 的离线集成测试。夹爪开度使用 URDF 行程映射，TCP 默认位于法兰 `tool0`。这些值不代表实物开度或 DH TCP 标定。

## 选择末端模型

安装新的 Python 3.12 wheel，并更新 viewer 中的模型选择和开度适配代码：

```sh
python -m pip install --force-reinstall --no-deps /path/to/libpyelite/dist/libpyelite-0.1.0-cp312-cp312-linux_x86_64.whl
xmv-elite-qt --list-models
xmv-elite-qt --station diagonal-08 --end-effector dh-ag95 --collision-profile auto
```

窗口标题显示 `08 · Elite CS66 + DH AG95`。夹爪滑块仍是 0% 闭合、100% 张开。DH 的主关节角随开度增加而减小，viewer 会按映射计算，不需要反转 URDF 的轴向。

切回 TacCap：

```sh
xmv-elite-qt --station diagonal-08 --end-effector taccap --collision-profile auto
```

`--station diagonal-08-dh-ag95` 等价于上面的 DH 选择。已有 `--station diagonal-08` 默认选择 TacCap。Qt、RRD 录制和 policy 入口共用 `libpyelite.assembly.available_models()`，新增模型不需要再维护多份 CLI 白名单。

直接使用 libpyelite：

```python
from libpyelite.assembly import Assembly, available_models
from libpyelite.sim import StationSimulation

print(available_models())
simulation = StationSimulation("diagonal-08", end_effector="dh-ag95", collision_profile="auto")
assembly = Assembly("diagonal-08-dh-ag95")
recipe_path = assembly.root / "simulation_recipe.json"
```

如需配置实际 DH TCP，复制 `simulation_recipe.json`，修改 `robot.left_tool_transform` 和 `robot.right_tool_transform`，然后通过 `--recipe` 加载。模型 ID 和 recipe 的 `native_assembly` 必须一致。不要使用 TacCap 的 TCP 或开度标定代替 DH 标定。

## 检查碰撞效果

1. 打开碰撞包络显示，确认左右夹爪各有九个凸体，包络随手指运动。
2. 在安全初始姿态下，分别操作左右夹爪的全开、半开和闭合，再复位。
3. 向桌面、支架或另一只夹爪靠近，观察碰撞保持状态。检查同一侧手指豁免没有取消这些外部碰撞。
4. 切回 TacCap，重复原来的测试动作。

同一只 DH 夹爪的两个指尖存在来源模型闭合重叠。本次按用户要求仅临时豁免这一对，每侧独立设置。显示模型仍保留原始重叠，外部障碍检测继续使用完整凸体。

DH 连杆正常机械接口使用显式零件对规则。另有三对零件的凸包在闭合端相交，但原始网格在完整联动行程内保持大于 0.5 mm 的间隙。生成脚本用距离及区间运动上界验证这些间隙后，才加入豁免。提高 `self_margin_m` 超过证明范围会拒绝加载，直接传入破坏 mimic 关系的配置也会拒绝检查。

分离证明适用于 URDF 规定的联动关系和主关节 `[0, 0.6524]` rad 范围。此测试不描述受接触后的被动变形。上游指尖碰撞形状每侧各自绑定 link，左右夹爪之间仍使用外部碰撞间距。

## 更换指尖后重新生成

当前来源是 `xense-description` 的 `feat/mech-prototype-drawings@1f480099e1743383df687c250505f1a19004640a`，装配入口为 `assembly.diagonal_07_elite_cs66_dh_ag95`。导入前会比较它与本库 08 工作台的机械臂、环境网格和关节变换。只有一致才复用原有 327 个碰撞体，再绑定 18 个 DH 凸体。

原始装配、组件 URDF、网格、来源清单和独立夹爪 profile 保存在 `assets/stations/diagonal-08-dh-ag95/source_assets`。运行时 URDF 将 12 个 continuous mimic 转为有界 revolute，保留轴向、关节角、原点和 mimic 系数。

换指尖时：

1. 在 xense-description 更新指尖视觉、碰撞凸体及装配输入锁。
2. 审查 `scripts/dh_ag95_collision_policy.json`。旧指尖的临时豁免不能直接沿用；删除已不需要的规则，再更新源 URDF 和 profile 哈希。
3. 在空目录重新生成。脚本会重新证明三个候选零件对的全行程分离；证明失败时停止，不会自动放行。

```sh
python scripts/import_dh_ag95.py /path/to/xense-description --destination /tmp/elite-dh-new
```

4. 审阅输出，替换本库生成的 `diagonal-08-dh-ag95` 目录，执行验证。

```sh
python scripts/import_dh_ag95.py /path/to/xense-description --check
python -m unittest discover -s tests -p test_dh_assembly.py -v
```

`--check` 在临时目录重新生成并逐文件比较。几何哈希变化但豁免规则没有重新审查时，导入会报错。未来如果换成新的夹爪型号，新增 catalog 条目及其独立快照，避免覆盖已有 TacCap 或 AG95 模型。

## 验证范围

测试覆盖资源搬移、源哈希、21 组姿态的全部 frame 等价、原样凸体绑定、101 档开度及连续路径、指定指尖豁免、外部障碍拒绝和 native 小步运动。viewer 回归覆盖正反方向开度映射、mimic、复位和渲染坐标与 native 的一致性。

实机夹爪驱动、实物 TCP 与开度标定、接触动力学和完整控制周期性能不在本轮离线验收范围内。
