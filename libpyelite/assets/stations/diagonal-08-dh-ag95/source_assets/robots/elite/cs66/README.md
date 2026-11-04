# Elite CS66

Elite CS66 六轴机械臂，资产 ID：`elite.cs66`。型号来自官方 `config/cs66`；不是 CS66A 或 EC66。

- robot viewer 入口：[urdf/cs66.urdf](urdf/cs66.urdf)。打开此文件即可，相邻 meshes 必须保留。
- [来源、哈希和验收状态](asset.json)；[许可证与改动说明](NOTICE.md)。
- 根坐标 `world` 固定连接 `base_link`，保留官方 `base`、`ft_frame`、`flange` 和 `tool0` 坐标系。
- 六个活动关节：`shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、`wrist_1_joint`、`wrist_2_joint`、`wrist_3_joint`。
- 7 个 visual DAE 和 7 个 collision STL 按原始字节保留，无外部纹理依赖；资产约 15.2 MiB。
- 单位：m、rad、kg。网格缩放、轴向、零位、限位、阻尼、惯量和碰撞几何沿用官方宏与 CS66 YAML。

## 来源与维护

官方仓库：<https://github.com/EliteRobots/Elite_Robots_CS_ROS2_Description>

固定提交：`edbbfdca9220ce264c99ec51d37440ff4fa13a9a`。
原始宏及 CS66 参数保存在 `provenance/upstream/`，均保留来源 SHA-256 与 Git blob SHA-1。
`provenance/cs66.urdf.xacro` 是本项目的最小调用入口，显式选用 CS66 参数、关闭硬件控制和仿真插件。
`provenance/urdf/cs66.upstream.urdf` 是本项目展开宏得到的比对快照，含 `package://`，不是 viewer 入口。
正式 URDF 仅将该快照的 mesh 路径改成相对路径；不增加第二份可独立维护的机械参数。

从仓库根目录运行（使用已有的 `xacro==2.1.1` 环境，无须 ROS）：

```bash
python scripts/generators/generate_elite_cs66.py --check
python scripts/generators/generate_elite_cs66.py --output-dir /tmp/cs66-candidates
```

第二条命令要求输出目录尚不存在。修改权威输入后，先比较候选文件，接受改动后更新 `asset.json`
中输入及生成文件的哈希、来源和说明，并重新验证。不直接修改生成的 URDF。
更换官方版本时应整体核对宏、CS66 YAML 和网格，不能拿其他型号补参数。

## 验证边界

检查结构、来源一致性、mesh 解析、零位静态图和安装后的资源路径；这些检查不代表硬件准确性。
robot viewer 人工交互验收、实机关节零位/轴向、质量/惯量准确性仍待核对，接触未验证。
**官方 `physical_parameters.yaml` 明确注明底座质量、惯量和重心可能不正确**，本次保留原值。
默认运动学不是针对某台实机的标定数据。未生成 MJCF，未执行 MuJoCo/Isaac 或 ROS share 安装验证。

## 来源与参数核对（2026-09-26）

固定提交中的四份 CS66 YAML、三份 Xacro 与官方源逐字节相同；
七份可视 DAE、七份碰撞 STL 也按源字节保留。来源文件及哈希见 [asset.json](asset.json)。
六个活动关节的原点、轴向、限位与展开后的官方模型一致，
详见[关节审查](../../../../docs/ROBOT_JOINT_AUDIT.md)。

七个实体 link 的质量、质心和惯量与官方 CS66 参数文件一致，URDF 质量和
19.140270 kg；其余六个 link 只是无几何坐标框架。六个关节声明的阻尼、摩擦均为 0，
仍是源模型值，不是实测值。官方对底座参数的警告及数学筛查见
[物理参数审查](../../../../docs/ROBOT_PHYSICS_AUDIT.md)。

## 本轮交叉核查（2026-10-12）

24 个官方原件再次核实；另与官方 Driver 的模型/网格比对一致，visual YAML 仅 ROS 包名不同。
产品页的 20 kg、J1/J2 150°/s、J3 ±360°、J4–J6 230°/s 与当前模型的
19.14027 kg、180°/s、J3 ±180° 存在差异。官方 DH d6=92 mm，默认运动学末腕偏移=95 mm，
对应关系及版本待厂家确认。加速度限位没有公开。当前保留锁定模型原值，不自行修改为宣传规格。
详见 [官方来源核查](../../../../docs/ROBOT_OFFICIAL_SOURCE_AUDIT.md) 与
[冲突记录](provenance/official_source_conflicts.json)。这些差异不影响来源一致性结论，但限制实机参数可信度。

## 07 可选碰撞派生入口

新增 `collision_lite` 入口，只替换碰撞 mesh；默认入口、源 STL、关节、当前物性及其他格式不改。原生 mesh 坐标和 collision origin 保持原值。派生网格放在 `meshes/collision-lite/`，锁定配方和误差记录在 `provenance/collision_lite_recipe.json` / `collision_lite_derivation.json`。

采用按源连通部件的边界/法向/拓扑保护 QEM 与自适应双向曲面距离界检查。腕部 1 mm，其他 link 2 mm。这是非凸三角网格，未通过实机、单侧实体包络、接触标定或引擎自动凸化验收。仅 07 装配引用本入口。

## 本地派生 USD（2026-10-14）

入口：[usd/elite_cs66.usda](usd/elite_cs66.usda)，保留完整 usd 目录。
从 default 官方输入以 URDF USD Converter 0.3.3 派生；不是厂家发布的 USD，没有使用 collision_lite。
保留 13 个源帧、7 刚体、6 转动关节及源 world 锚定；取消转换器默认单凸包近似，保留源碰撞表面。
来源和版本锁见 [配方](provenance/usd/conversion.json)，数值结果见 [验证](provenance/usd/validation.json)。
已检查多姿态约束正运动学、完整惯量、全部三角形、OpenUSD 和独立 ZIP 搬移；动力学/接触未验收。
转换脚本位于仓库外；方法及 usd-brep 比较见 [实施记录](../../../../docs/ELITE_ACONE_USD_CONVERSION.md)。
