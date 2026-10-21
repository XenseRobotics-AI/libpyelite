# Elite 安全接口与仿真调参

本实现没有进行真机验收。IK 的阻尼/迭代次数不变。新增的加速度/jerk 限制需要显式启用，
示例数值仅供仿真调参，不是 Elite 额定限值。碰撞层（C++ `StationGuard`）仿真和真机共用，
真机由 `StationCoordinator` 驱动；协调器的线程/锁胶水代码离线无法构造（SDK 构造时即连接
机器人），**尚未在真机上验证**，首次上机按下文"真机碰撞层"的步骤低速进行。

## 从主仓库根目录启动

运动规划和安全项都在 recipe 的 `robot:` 段里（`native_*`，见各 bi_elite_cs66_rt
teleop / record recipe 的 "Native motion planning & safety" 块），真机和仿真读同一份：

```bash
# Use the recipe collision setting shared by hardware and simulation.
python -m libpyelite.sim --recipe recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml
# Explicitly enable the station collision layer.
python -m libpyelite.sim --recipe recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml --collision-profile auto
```

- `--collision-profile auto|<json>|off` 覆盖 recipe 的 `native_collision_profile`。
  recipe 里设了非 null 值时，真机也启用同一碰撞层（见"真机碰撞层"）；需要 `native_assembly`。
- `--safety-config <json>` 仍可临时覆盖任意项（不会改写 recipe），`--show-config` 输出实际参数。
- 改了 TacCap teleop recipe 后，用
  `python third_party/libpyelite/scripts/import_assemblies.py --profiles-only` 刷新仿真包内
  的 `simulation_profile.json`（不加 `--recipe` 启动时的默认配置；测试会校验二者一致）。

启用碰撞时，Qt 的“显示碰撞包络”可叠加显示 Meshcat 基本几何体；被障碍物挡住时臂停在
最近的可行点继续跟踪，只有锁存的异常（已穿透、硬界矛盾）需要通过界面的显式重置恢复。
FK 滑条发送关节目标，和 IK 目标一样经过速度/加速度/jerk 限制和碰撞门执行，不是瞬移。

## 真机碰撞层

仿真和真机跑同一个 C++ 对象 `StationGuard`（`src/station_guard.{hpp,cpp}`）：每个 4 ms 周期
规划两臂参考 → 线性化一次 → 约束参考（接触即保持）→ 关节限幅 → 关节 QP 门 → 运动界复检。
QP 是可行起点的原始有效集法（`src/projection_qp.cpp`）：零位移总可行，每次迭代都满足硬约束，
迭代上限只损失最优性；实测 3–11 次迭代。

真机上 `native_collision_profile` 非 null 时：

- 两个 `NativeDriver` 切换为外部规划（不再各自起规划线程），一个 `StationCoordinator`
  线程每周期取两臂最新目标、调用 `StationGuard::tick`、把结果交给两臂各自的发送线程。
- 某臂伺服未运行或正在启停（MoveJ、startServo），本周期视为测得关节处的固定障碍物。
- 协调器、guard 或任一臂出错 → **两臂发送线程一起停止并锁存错误**（C++ 内完成，
  不依赖 Python 轮询）。
- 发送端在这种模式下只检查速度和位置：guard 的加速度约束是软的（靠近障碍物时可以
  更急地刹车），发送端不能把这个刹车再"平滑"掉。
- 启动时的回到起始位、断开时的回 home 都是 MoveJ（控制器关节插值）。执行前用独立检查器
  证明两臂一起的完整关节直线路径：起始位被挡 → 拒绝连接；回 home 被挡 → 跳过回 home、
  保持原位并报错。前提是 Elite MoveJ 为关节空间插值、两臂时长相同。夹爪快照只在预检查时
  冻结，此检查不能保证整个实际 MoveJ 期间自由变化的手指轨迹。
- `get_native_status()["station"]`：tick 数、超时次数、最大单步耗时、最近一步的
  保持/限速/最近距离。

实机夹爪采用“仅监测夹爪、限制双臂”的策略：夹爪开合命令和正常抓握力控制保持独立，
双臂暂停时仍以默认 100 Hz 更新实测夹爪几何，不依赖 `get_observation()`。原生 4 ms
循环只读不可变快照；缺失、过期、无效或倒退的样本按故障停止双臂。新的实测手指位置若使
当前几何违反余量或重叠，则 `measured_geometry_hold` 重复上一条接受的双臂指令，
不推进参考、不规划逃离；新鲜几何恢复间隙后自动继续候选指令检查。夹爪本身不会因此停止。

`native_collision_gripper_calibration` 必须填写每侧实际传感器开度到独立 URDF 关节的标定点；
从动手指按 URDF mimic 关系展开。没有实测传动关系时保留 null，拒绝实机碰撞模式；
不把 URDF 的名义上限当标定。TacCap 模型也不能用于串口 XGripper。

迁移时可复用 URDF／JSON 几何和标量关节整段检查器；当前 guard 固定为双六轴机械臂，
协调器绑定 Elite RTSI／驱动。其他机械臂需要适配关节与 TCP 变换、周期／限位、实测关节与
新鲜夹爪样本、候选指令发布、保持／停止及故障状态接口，不能直接把整个实时栈换接硬件。

两臂、每周期 guard 实测（仿真，同一份代码，i7-14650HX，每指 26 个盒体）：自由运动平均
0.11 ms（p99 0.16、最大 0.24 ms）；单臂压桌 0.21 ms（p99 0.28、最大 0.35 ms）；两臂同时压桌
0.31 ms（p99 0.42、最大 0.56 ms）。粗筛分两层（同一运动 frame 的形状合成一组，先测组对再展开），
43k 个形状对每周期只做约 5.5k 次组 AABB 测试（与每指 2 个盒体时相同）；接触时的主要开销是每个贴近的指尖盒体各一行屏障约束。

首次上机建议：低速 recipe；先只开碰撞层不动手柄，确认 `station` 状态 tick 正常增长、
无 overrun；再单臂缓慢靠近桌面，确认停在约 11 mm 处；最后两臂互相靠近。

### grip 重新按下时从机器人位置起算

Pico 的目标是"按下 grip 时的目标 + 手柄位移"，接触保持后虚拟目标会停在障碍物另一侧。
`grip_anchor_to_robot`（Pico4 / BiPico4，默认 true）让每次按下 grip 时以机器人实测 TCP
为起点，松开再按即可立即反向离开。Elite 的 teleoperate / record 入口会自动提供位姿来源；
读取失败时退回原行为（沿用上一个目标）。

## 关节轨迹与参数

Python `PlannerConfig` / 配置文件 `robot` 字段对应如下（后者加 `native_` 前缀）：

| PlannerConfig | 类型/单位 | 含义 |
|---|---|---|
| joint_motion_limits | bool | 启用 Ruckig 轨迹；默认 false |
| max_joint_velocity | 6 个 rad/s | 不得超过 URDF 速度乘 velocity_scale |
| max_joint_acceleration | 6 个 rad/s² | 必须全部为正、有限 |
| max_joint_jerk | 6 个 rad/s³ | 必须全部为正、有限 |
| max_target_translation_jump | m | 相邻已接受目标的平移突变阈值；0 关闭 |
| max_target_rotation_jump | rad | 最短旋转角突变阈值；0 关闭 |
| max_target_joint_jump | rad | 相邻关节目标突变阈值；0 关闭 |
| joint_tracking_gain | 1/s | 跟随运动参考时的位置修正增益；默认 20 |
| joint_target_horizon | s | 已弃用、忽略（旧的前瞻补丁，见下文） |

示例 `examples/sim_safety.json` 的取值和依据（仿真起点，不是 Elite 额定值）：

| 量 | 示例 | 依据 |
|---|---|---|
| 速度 | 2.6, 2.6, 3.1, 3.1, 3.1, 3.1 rad/s | CS66 规格书 J1–J2 150°/s、J3 180°/s；腕部 230°/s 但打包 URDF 上限 π |
| 加速度 | 10 rad/s² | Elite 未公布；SDK `stopj_acc` 默认 8，Franka FR3 10 |
| jerk | 500 rad/s³ | Elite 未公布；MoveIt 缺省 1000，Franka FR3 5000；旧示例 10 低 2–3 个数量级 |

上真机建议先取速度的约 80%。a=20、j=1000 跟手更紧，突然停止时过冲更小（见下）。

首次命令与反馈初始化姿态比较；拒绝目标不会更新已接受目标的序号。
显式 timed reset 绕过笛卡尔突变阈值，但仍受关节轨迹限制。
数值 IK 仍由现有 Pinocchio box-QP 完成。

### 为什么旧实现滞后严重

旧实现把每个周期的 IK 解作为 Ruckig 位置接口的目标、目标速度为 0，并且串联两次
（planner 一次、发送线程一次）。速度 0 的位置目标意味着"在那里停下"，跟随端必须
始终和参考保持一个完整的制动距离，所以参考一直在动时输出一直落后；Ruckig README
原文："this approach will always lag behind the signal"，消除它的 tracking 接口只在
Pro 版。示例的 jerk=10 又让每次加减速都至少需要 0.2 s 的 jerk 爬坡，两层串联再叠加
约 50%。0.158 m 阶跃实测：无限幅 0.16 s，旧方案 1.70 s（horizon=0 时 6.24 s）。

Flexiv 不是这样做的：RDK 的实时流式接口（`StreamJointPosition` /
`StreamCartesianMotionForce`）本身没有速度/加速度/jerk 规划，要求上层发送平滑命令；
本机 libpyflexiv 只做逐周期位姿步长裁剪（0.5 m/s、1 rad/s）和速度前馈变化率裁剪
（2 m/s²、5 rad/s²），平滑交给控制器的阻抗环。RDK 的 API 里没有 jerk 参数。

### 笛卡尔平滑（2026-10-14 修订，默认关闭，仿真示例开启）

逐关节独立限幅会打乱关节配合：各关节被限住的时机不同，TCP 不再走直线、也不再绕
TCP 点转（实测 15 cm 平移偏离直线 28 mm，0.8 rad yaw 时 TCP 漂 65 mm、过冲 15°）。
调研的成熟方案都不这样做：

| 来源 | 平滑所在空间 | 超限处理 |
|---|---|---|
| Flexiv RDK / libpyflexiv | 笛卡尔（0.5 m/s、1 rad/s、2 m/s²、5 rad/s²） | 平移按步长模长、旋转按转角整体缩放，方向不变 |
| MoveIt Servo、Drake diff-IK | 关节 | 所有关节同一个缩放系数（方向不变） |
| UMI、KDL | 笛卡尔直线 + SLERP | 平移和旋转共用时长，同时到达 |
| libfranka | 笛卡尔 | v/a/jerk 按向量模长；逐关节只作最后兜底 |

现在的链路：

1. `CartesianSmoother`（`src/cartesian_smoother.hpp`）在 IK 之前平滑目标流：
   命令 twist = 目标 twist 前馈 + 沿 6 维位姿误差的修正（受制动距离约束），
   v/a/jerk 都按线、角两部分的**模长**限制，并用同一个系数缩放整个 6 维向量，
   所以直线保持直线、纯转动绕 TCP，平移和旋转同时到达。
2. IK 在更宽的信任域里解，然后**整个关节步按同一系数缩放**到关节速度上限
   （不逐关节截断），并把这个系数反馈给平滑器做时间缩放（参考变慢，而不是偏离路径）。
3. 流式 IK 收敛容差单独设为 1e-7 m / 1e-6 rad（`streaming_*_tolerance`）。原来在
   0.1 mm 内任意时刻停止迭代，残差每周期跳动，差分出来是关节加速度 38 rad/s²、
   jerk 1.5e4 rad/s³ 的锯齿，之前被逐关节 Ruckig 掩盖。
4. 关节 Ruckig 跟随保留，参考可行时几乎原样复现；只在参考自身突变（碰撞层冻结、
   奇异点附近）时平滑地偏离。示例关节限值 40 rad/s²、5000 rad/s³ 只作兜底。

示例（`sim_safety.json`）：TCP 0.5 m/s、1.0 rad/s（Flexiv 默认），5 m/s²、10 rad/s²，
1000 m/s³、2500 rad/s³（MoveIt 缺省 1000；libfranka 上限 4500）。同一套拖动实测：

| 动作 | 偏离路径 | 滑条瞬停过冲 | 平滑停止过冲 |
|---|---|---|---|
| x 15 cm | 0.8 mm | 11 mm | 2 mm |
| yaw 0.8 rad | TCP 漂 0.1 mm | ≈0 | ≈0 |
| roll / pitch 0.3 rad | TCP 漂 ≤ 0.2 mm | 1.3° | 0.2° |

jerk 不能太低：平滑器修正增益 20 1/s 的时间常数是 50 ms，jerk 100 m/s³ 下加速度爬到
5 m/s² 也要 50 ms，两者相当时环路相位滞后，到位后会在目标附近 ±16/11/8 mm 来回摆
约 0.7 s。jerk 1000 时爬坡 5 ms，过冲后一次回位、0.3 s 内进入 0.1 mm。接近目标的
允许速度按"减速 + jerk 释放"两段的制动距离计算（`brakingSpeed`），只用一半加速度。

瞬停过冲是加速度有限的物理结果（v²/2a），只能用滞后来换；真实手柄会自己减速。
加速度取 Flexiv 的 2 m/s² 时过冲约 30 mm。
**不可达目标**：例如在默认姿态绕 TCP roll 0.8 rad，需要肘关节伸直（J3→0，肘部奇异），
臂停在边界上离目标约 22 mm；这不是平滑问题，界面会显示目标不可达。

### 现在的做法

- IK 在一条不受加速度限制的**关节参考**上运行（从参考自身热启动，单周期速度信任域）。
- 参考在动时，Ruckig 用**速度接口**跟随：命令速度 = 参考速度前馈 +
  `sign(e)·min(gain·|e|, √(a·|e|))`，再受"在关节限位前能刹住"的速度上限约束；
  加速度和 jerk 仍由 Ruckig 保证。这是 MoveIt Servo 的 Ruckig 插件
  （target_velocity = v + a·dt）一类的做法。
- 参考停在段末时，切回位置接口时间最优地停到参考上，不过冲。
- 发送线程只做**门**：规划样本相对已发送值的有限差分满足 v/a/jerk 限制（5% 余量）
  时原样发送；漏掉或重复的规划周期才由同参数的限幅器跟踪补齐。双层叠加因此消失。

同一拖动（0.32 m/s 斜坡，示例参数）实测：最大滞后 11 mm（起步加速段），
匀速稳态约 1.4 mm，发送端 500/500 样本透传，v/a/jerk 均不超限。
代价是参考**瞬间**停止时会过冲 v²/2a：滑条示例约 16 mm（a=20 时 10 mm），
0.24 s 内回到 1 mm 以内；真实手柄不会瞬间停止，过冲更小。不过冲就必须保持制动
距离的滞后，这是限幅本身决定的取舍。

位置接口路径的限位检查覆盖整个生成轨迹的极值，包括制动过冲；速度接口路径的轨迹
不是制动包络，改为限制命令速度不超过到限位的制动速度，并检查每个发布样本。

`Planner.status()` 提供 `joint_velocity`, `joint_acceleration`, `braking`。
`NativeDriver.status()` 中 `commanded_q` 是规划输出；`sent_q`, `sent_velocity`,
`sent_acceleration` 是实际交给 SDK 的最后命令状态，调参时应优先查看后者。
这些是命令轨迹，不是测得的实际机器人导数；硬件采样周期抖动仍需实测。

`cancel()` 和过期命令在限幅启用时转入有限 jerk 制动，制动结束保持。
500 ms 是开始制动的时刻，不是承诺在 500 ms 内完成物理停机。
硬件 fault、SDK 失败、限位不可行和碰撞层不可行属于异常停止，优先停止运动指令，
不承诺异常瞬间仍满足正常运行的 jerk 上限。

## 反馈、故障与接触力

- 启动仍要求 RTSI 时间戳推进、有效且静止的反馈、FK 与控制器 TCP 一致。
  `native_max_start_joint_speed` 默认 0.02 rad/s，可配置。
- `native_check_robot_safety` 默认 true。只接受 RUNNING 加 NORMAL/REDUCED；
  protective stop、fault、急停和未知状态使发送线程退出并记录错误，不自动清故障。
- 双臂 `command_stale_timeout_s` 默认由原先 1.0 s 改为 0.5 s；单臂仍为 0.5 s。
  既有 IK 参数、TCP 速度默认及真实 recipe 中显式覆盖值保持不变。
- `native_max_contact_wrench`：6 个非负阈值 [Fx,Fy,Fz,Mx,My,Mz]，单位 N、Nm，默认全关闭。
  **CS66 台架实测 RTSI `actual_TCP_force` 对外力几乎没有响应**（约 100 N 推力变化
  < 0.05 N），这个检查实际上不起作用，示例已改为全 0。请改用下面的关节电流残差。
- `native_contact_monitor`（双臂另有 `left_/right_native_contact_monitor` 逐臂覆盖）：
  关节电流残差碰撞停止，覆盖任何连杆而不只是工具。估计器来自 compliance WIP
  （`src/compliance.{hpp,cpp}`）：`actual_joint_torques` 减去 RNEA 刚体模型
  （拟合重力向量 + 点质量负载）和 Coulomb+粘滞摩擦，滤波后减去启动时静止采集的零偏；
  任一关节超过 `collision_threshold`（启停/换向后 `transient_window` 内放宽
  `transient_factor` 倍）即停止发送并锁存。这里只用它的碰撞判断，导纳偏移被忽略。
  - `gravity` 默认按装配的底座姿态算标准重力在底座系下的分量（斜装底座约
    [-6.93, 0, -6.93]）；`probe_force_noise.py analyze --export` 的拟合值优先。
  - 负载默认取 recipe 的 `payload_mass`/`payload_cog`（电机力矩包含工具重量）。
  - 示例 `examples/contact_monitor.json` 的摩擦和负载取自 2026-10-10/11 台架辨识；
    阈值 35/35/30/8/8/8 Nm 是起点，未经验收。
  - 启动时臂必须静止且无接触 `bias_time`（0.3 s）完成零偏，状态 `contact.biased`；
    之前不检测。`rebiasContact()` 可在静止时重新零偏。
  - 已知局限：J1 在 ±0.3 rad 以上的大范围扫动中重力模型漂移，可能误报；
    这一路无法在仿真中验证（仿真没有电机电流），必须上台架调阈值。
- 随附输出 recipe 新增 `robot_mode`, `safety_status`, `actual_TCP_force`；
  自定义 recipe 缺字段时 LeRobot 适配器启动检查会明确拒绝。
- SDK 写失败/异常后 best-effort writeIdle；连接已断时不能保证停机命令送达，
  控制器通信 watchdog 仍是独立的保护路径。

## 碰撞几何与约束

运行时读取 `assets/stations/<station>/collision.json`，不修改或解析 STL 拟合几何。
JSON 指定每个几何的 `frame`, `xyz`, `rpy`, `type`, 尺寸、分组和允许接触对。
`type: "convex"` 使用局部坐标 `vertices`（N×3，单位米）和朝外的三角形索引
`faces`（M×3，索引从 0 开始）。原生 Coal 3.0.3 直接使用这些凸多面体；不需要升级依赖。
加载时同时检查有限数值、索引、面朝向、闭合边、凸性、非零体积和所有顶点都被引用；
畸形输入会被拒绝。局部顶点不必以原点为中心：AABB 中心随姿态变换，运动界的包围球则
始终以 `xyz/rpy` 定义的局部原点为球心，覆盖包括膨胀在内的全部顶点。
URDF 只提供关节与坐标变换；双臂已经共享装配中的 world，环境固定 frame 的雅可比为零。
双臂距离行同时包含双方关节列，不是把另一臂当成永远静止的障碍。

离线生成脚本将原始三角面保守包在基本几何中：长杆胶囊体、基座圆柱、腕部和
桌面支架的分块盒体。桌子的每个三角面完整落入一个盒体；不使用一个填满桌下空间的大盒子。
TacCap 手指改为少量凸多面体，沿长轴分区，并在体积明显减小时保留横向分区，
跟随渐缩外形和 L 形缺口，不把整根凹手指直接取一个凸包。每个跨分区的 STL 三角面
先精确裁剪为多边形，再取支撑平面并向外移动 2 mm；每个完整片段及其欧氏 2 mm
邻域都被包住。`source_cell` 保存分区边界，测试重新裁剪原始 STL，验证分区无空隙且
所有片段位于每个外表面内侧至少 2 mm。这不是只抽查原网格顶点。
新多面体还与旧版 26 个膨胀盒体的整体凸包求交；旧包络已覆盖同样的膨胀，因此交集
不会丢失覆盖，并保证任何朝向下向平面桌面的伸出量都不超过旧版。此结论针对平面接近，
不表示新旧包络对任意形状障碍物的距离完全相同。
离线生成需要现有 `scipy.spatial` 的凸包和半空间求交功能；运行时和 Meshcat 渲染不需要
SciPy。生成按固定裁剪次序、支撑平面选择和顶点/面排序，避免随机简化。
diagonal-07/08 每指由 26 个盒体改为 5 个凸多面体；每片 20–32 个顶点，每指共
133 个顶点、78 个不同支撑平面、246 个三角面（原盒体分别为 208、156、312）。
整站由 435 个基元、43,289 个潜在碰撞对减少到 351 个基元、13,217 对，
其中涉及手指的对由 36,712 减至 6,640。对象对、支撑平面和渲染三角面是不同的计数；
这些减少不等同于相同比例的运行时间提升。
现有模型上 primitive 仍是近似，特别是曲面附近会比真实网格更早限制运动。
默认额外几何膨胀约 2 mm；自碰撞额外间距 0.5 mm，双臂/环境间距 10 mm。
自碰撞间距较小是因为关节附近的保守包络本身已很接近，不代表真实设备只需要 0.5 mm 安全余量。
请在 Meshcat 检查包络，并通过 JSON 调整；这些数值未经硬件安全验收。

只排除机械相邻刚体、安装基座/肩部与支架的固定连接、TacCap 同侧两指根部内部接触。
两指的原始 STL 在默认开度接触，豁免仅针对这对内部几何；仍分别检查手指与其余物体。
不会根据随机采样结果自动关闭其他碰撞对。

另一类豁免是 `kinematically_separated_pairs`：同一臂上、中间最多隔两个转动关节的连杆对，
生成脚本对原始碰撞网格（不是包络）在完整关节范围内做分支定界证明：每个关节区间单元，
中心处网格距离减去单元内任意顶点可能移动的 Lipschitz 上界仍 > 5 mm，才算通过；
任何单元无法证明就不豁免。这不是采样，也不依赖当前姿态。
例如 upperarm 胶囊体与 base_link_inertia 圆柱体在 home 位姿只差 0.9 mm，
而真实网格在 J1/J2 全范围内最小距离约 16.6 mm；保留这行约束只会让屏障一直压住 J2，
限制几乎所有笛卡尔运动。diagonal-07/08 各豁免 14 对（每臂 7 对），几何基元本身不变。

运行时把环境（支架、桌子）基元的位姿和 AABB 在加载时缓存一次；距离查询先用 AABB
下界剔除检测带以外的对，再做精确距离，不影响结果，只减少计算。

约束参考 Mink：`-J_distance * delta_q <= gain * max(0, distance-min_distance-qp_buffer)`，
默认 gain=0.2、检测距离 0.05 m（Mink 默认）、`qp_buffer_m`=1 mm。Pinocchio 提供最近点
雅可比，Coal/FCL 提供几何距离。原生 IK 的内部 box-QP 没有被替换。

每个仿真周期做两次投影（C++ `StationGuard` 和原生主动集 `ProjectionQP`）：

1. **参考投影**：planner 拆成 `plan_reference()` / `advance()` 两半，中间把两臂的
   关节参考位移联合投影，再 `set_reference()` 写回，限幅器只跟随无碰撞的参考，
   前馈速度也取投影后的实际参考运动。只约束输出时，参考会留在障碍物另一侧，
   输出一直朝它推、沿障碍物在关节空间里滑走（实测目标压入桌面 7 cm 时漂到 0.34 m 外）。
2. **输出门**：最终发布的 q 再投影一次；速度和位置界、距离屏障是硬约束，
   加速度/jerk 的离散导数界是**软约束**（大权重松弛变量）。屏障要求的减速可能超过
   加速度限制，硬约束时这就是之前的 `primal infeasible`；软化后 dq=0 总满足所有
   硬约束，QP 不会再无解。

### 接触后的行为：保持，不绕着障碍物重构手臂

- **IK 目标**：参考投影后的步只有在让 TCP **位置误差和姿态误差都不增加、且至少一项减小**
  时才放行；否则该臂参考保持不动，输出平稳停住，状态显示"接触障碍物：保持当前姿态"。
  减速接近的阶段仍在缩小误差，会放行；目标压到障碍物另一侧后就停。旧行为把被挡住
  之外的 IK 步余量照常执行，压入桌面后 6 个关节又转了 10°–84°（峰值 2.86 rad/s），
  也就是"手停住了胳膊还在动"。目标拖回可达一侧后自动恢复跟踪。
- **关节目标（FK 滑条）**：沿指令的关节方向按屏障允许的比例缩短步长，在余量处停下，
  其它关节不动。旧 FK 滑条是逐值离线重置、只检查终点，拖过桌子厚度后会一次穿过去
  （J2 一步 31°）。
- 可选的另一种语义是沿障碍物表面滑向目标（Mink 的默认效果）：姿态权重取 1 m、
  姿态容差 1e-4 rad 时斜向拖动会沿桌面滑出约 120 mm，但正压时有约 1°/2 mm 的蠕动，
  因此默认不用。正压时臂停在屏障前约 2 mm（离余量约 3 mm），比顶到余量更保守。

两次投影的度量都是 TCP 任务空间 `JᵀJ + λI`（姿态行按 0.1 m 半径加权），即在可行集里
找对 TCP 运动改变最小的关节位移；欧氏关节度量会把被挡住的运动变成笛卡尔意义上的
乱滑。输出被门修改后只重锚输出状态，保留 Ruckig 自己的连续 v/a（加上修正量/dt）；
用有限差分导数重锚会滞后一个周期，每周期反馈会抹掉正在建立的减速度，臂刹不住。

投影后验证「上次接受的指令 → 本次预测指令」的完整关节线段；活动臂不改成以实测状态
重锚。距离减去整段运动上界可以证明安全的小步直接放行，其余区间自适应细分，覆盖起点
检测带以外可能进入安全间距的碰撞对。证明失败或计算预算耗尽时把步长减半、再减半，
最后原地保持；目标回到可达一侧后自动恢复，不再锁存暂停。
当前构型违反间距或硬界互相矛盾通常锁存并要求显式重置；启用实测夹爪时，手指变化导致的
当前余量不足／重叠采用上述 `measured_geometry_hold`，间隙恢复后继续。遥测失效仍锁存故障。
这是所加载几何与支持的关节模型下的整段保守证明，不是设备实际运动或真实网格的安全认证。

### 线段验证与速度上限

`check(q)` 只回答单个构型的间距，不能替代 `verify_step(q0, q1)`。
后者要求 `q0` 与最近一次 `linearize()` 的构型完全一致；所有输入和原生 `resolution`
参数都必须有限，`resolution` 必须为正。该参数仅保留接口兼容，不再以固定采样间隔判定安全。
返回的距离/间距可以是保守下界；`safe=false` 也可能表示未能完成证明，并不必然表示已测到穿透。
Python `check_segment()`（包括 MoveJ 直线关节路径预检）使用同一个整段验证器；
`include_start` 和 `cutoff` 只保留参数兼容，不能用来跳过起点或潜在碰撞对。
它会恢复已有线性化，但仍会修改内部缓存；并发预检和实时碰撞门必须各用独立实例。

原生验证对每个基元建立覆盖所有点的包围球（凸包也按其局部原点计算半径）。从基元向根
逐个累加固定连杆长度、包围球半径和整段内移动关节的最大伸长，得到转动关节的杠杆臂
上界 `R_j`。对参数化路径 `q(t)=q0+t·Δq`，每个点在任意子区间内的位移不超过
`|Δt|·B`，其中 `B=Σ转动关节 |Δq_j|·R_j + Σ移动关节 |Δq_j|`。
这个界覆盖多关节同时转动与下游伸缩，不使用起点雅可比乘固定百分比来近似有限转动。
使用各关节的配置索引 `idx_q`，不假定它等于速度索引 `idx_v`。

在区间中点取得 AABB 距离下界或 Coal 距离后，减去双方到该区间任意位置的运动界、
碰撞对安全间距及 1 µm 数值储备；结果非负才证明整个区间安全。因此即使两个采样点之间
有很薄的障碍，也不会仅凭采样点安全放行。检测带外的对同样必须通过此证明或进入细分。
支持的运动链是标量转动/移动关节（含非坐标轴方向），不支持的连续转动双配置坐标、
复合或其他关节会明确拒绝线段验证。`check()` 的单点查询语义保持不变。

细分的空间预算取 `0.5 × (检测距离 − 最大碰撞对间距 − qp_buffer − 1 µm)`；
该值控制何时细化昂贵的精确查询，不能单独证明无碰撞。没有可用检测带时仍使用全对距离证明。
初始证书扫描一次配对表；后续细分阶段最多处理 512 个区间、4096 次精确距离查询、
131072 次候选对访问及 32 层细分；
任何限制到达而区间仍未证明时返回不安全，绝不跳过区间或只查终点。这是计算量上界，
不是 4 ms 内必定完成的实时保证；贴近间距的安全运动也可能被保守保持。
上述证明依赖刚体几何和 Coal 距离查询的数值精度，1 µm 储备不等同于浮点区间算术认证。

当前 recipe 周期为 4 ms，2.6/3.1 rad/s 对应每周期 0.0104/0.0124 rad。
50 mm 检测距离与 10 mm 双臂/环境间距之间共有 40 mm 的双方相对运动空间；
再扣除默认 1 mm QP 缓冲与数值储备、取一半后，细分预算约为 **19.5 mm/碰撞对**。
关节角步长必须通过连杆长度、包围球和双方同时运动转换成空间运动界，不能直接与毫米比较。
建议维持 recipe 当前速度上限；提高前应重新验证几何、保持/恢复行为和最坏计算耗时。
更高速度通常意味着更多细分或更保守的保持，没有脱离姿态和几何的通用 rad/s 安全阈值，
也不保证默认速度永远不会触发细分或保持。

### 计算实现与开销

距离、约束行和步长校验在 C++（`src/station_collision.{hpp,cpp}`，Coal + Pinocchio）中计算，
Python 只校验 JSON 并编排。做法参考 Pinocchio `computeDistances`、OCS2、Tesseract、Mink：

- **平铺的固定配对表**，每对一个缓存的 Coal `ComputeDistance`（省去形状分派），
  GJK 用上一周期的结果热启动（`CachedGuess` + `updateGuess`）。
- **粗筛**：285 个静态环境基元（支架、桌子）的 AABB 只在加载时算一次，每周期只更新 66 个运动基元；
  AABB 间隙是真实距离的下界，超出检测带（5 cm）的对不做精确计算。
- **每周期只线性化一次**（距离 + 约束行 + TCP 度量），参考投影和输出门共用；参考投影
  把这组约束平移到参考构型上（上界不低于 0，保证"原地保持"始终可行）。
- **整段有效的运动界**：使用上节的串联运动链长度界；小步复用线性化缓存直接证明安全。
  证明不了的区间才计算中点距离并细分；大步覆盖检测带外候选对，预算耗尽保守保持。

此前版本两臂、每个 4 ms 步实测（仿真，Python 编排；不是本次整段验证的最坏耗时保证）：

| 场景 | 改前（Python） | 改后 | 其中 C++ 碰撞计算 | 每步精确距离次数 |
|---|---|---|---|---|
| 空中移动 | 3.35 ms | 0.43 ms | 0.06 ms | 48 |
| 压着桌面 | 4.35 ms | 0.90 ms | 0.09 ms | 60 |

上述历史版本的单次调用：`check` 0.90 → 0.06 ms，约束行 0.76 → 0.05 ms，
步长校验 2.27 → 0.02 ms；146 个构型的最小距离差 3e-12 m，与逐对 Coal 查询一致。
当前参考/输出投影、任务度量和线段验证均已在 C++ 中执行，由 `StationGuard` 调用
原生主动集 `ProjectionQP`，不再依赖 Python OSQP 编排高频碰撞门。

API：

```python
from libpyelite.collision import StationCollision
checker = StationCollision(pin_model, "collision.json", urdf_path)
result = checker.check(q_full)  # safe, minimum_distance, pair, clearance
G, h = checker.compute_qp_inequalities(q_full, controlled_velocity_indices)
delta, active = checker.project_displacement(q_full, suggested_delta,
                                             controlled_velocity_indices, lower, upper,
                                             soft_lower, soft_upper, metric)
```

配置和源网格带 SHA-256。URDF/link/网格更新后旧配置会拒绝加载，需要重新生成并审阅：

```bash
python third_party/libpyelite/scripts/generate_collision_profile.py \
  third_party/libpyelite/libpyelite/assets/stations/diagonal-08/assembly_taccap_identified.urdf \
  third_party/libpyelite/libpyelite/assets/stations/diagonal-08/collision.json
```

底层几何引擎按 URDF 和独立配置工作；当前 `StationGuard` 和 Elite 协调器仍针对两台
六轴机械臂。生成脚本提供当前 CS66/TacCap 桌面装配的拟合策略。
新增车型或更名 frame 时需更新独立几何配置/生成策略，不能沿用过期尺寸。

参考：[Mink 距离约束](https://kevinzakka.github.io/mink/api/limits.html)、
[Ruckig](https://github.com/pantor/ruckig/tree/v0.15.3)。

补充参考：[Pink SelfCollisionBarrier](https://stephane-caron.github.io/pink/barriers.html)。
Pink 文档提醒距离屏障对非光滑几何的行为存在限制；此处采用 Mink 的逐步法向位移约束，
对盒体边角处最近点切换保留最终非线性几何检查，不宣称具有全局连续屏障保证。
检测带宽应大于每步最大可能闭合距离；高速配置需要重新检查该条件。
双臂适配器在 Python 层检查到任一侧原生错误后停止两侧发送线程；这不是独立于
Python 调度的硬件双臂安全互锁。

### 界面状态与 Home

`native_joint_target_horizon` 是上一版的前瞻补丁（把停止点推到 0.1 s 之后），已被
参考跟随取代，字段保留但不再生效。碰撞仿真不模拟发送门（输出门本身就是最终输出）。

IK 页状态颜色：绿色=已到达，橙色=跟踪中（IK 正常，受限幅或碰撞约束滞后），
红色=IK 失败或 0.5 s 内误差不再减小（目标不可达或被挡住）。
界面 `Reset to home` 恢复启动关节角和目标，清除暂停并重启刷新；是离线重置，不是运动规划。
