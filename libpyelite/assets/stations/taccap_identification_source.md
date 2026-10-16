# LeRobot XenseHand

本分支以 Hugging Face LeRobot **v0.6.0** 为框架基线，保留并迁移 Xense 的 Franka Research 3、Pico4、Revo2、TacCap 与 Manus 遥操作集成仓库。

## 支持的设备

Robot：

- `franka_research3`
- `franka_research3_dexhand`
- `bi_flexiv_rizon4_rt`
- `bi_flexiv_rizon4_rt_wuji`
- `revo2_hand`
- `wuji_hand`
- `taccap_gripper`
- `tianji_arm`
- `tianji_arm_wuji`

Teleoperator：

- `pico4`
- `pico4_hand`
- `bi_pico4`
- `manus_hand`
- `manus_wuji`
- `manus_pico4`
- `manus_bi_pico4`

机器人末端夹爪：

- `taccap_follower`（单臂或双臂 follower/slave；不是手持 leader）

## 环境安装

已验证的主机环境为 Ubuntu 22.04/24.04 x86_64。先安装 Miniforge，然后执行：
环境使用 Python 3.12 和 NumPy 2.x（LeRobot 0.6.0 的最低要求）。

```bash
git clone git@github.com:XenseRobotics-AI/lerobot-xensehand.git
cd lerobot-xensehand
git switch main

bash ./setup_env.sh --mamba lerobot-xensehand
mamba activate lerobot-xensehand
bash ./setup_env.sh --install
bash ./setup_env.sh --verify
```

`--install` 在成功结束前会自动对同一组模块做一次不连接硬件的软件验证；
`--verify` 可用于安装后的独立复查。

如果同名环境已经存在，脚本会检查它是否满足当前基础环境定义，并询问是更新、
覆盖重建还是保留。自动化场景可分别使用 `--update` 或 `--force`。

不带模块名的 `--install` 只安装 LeRobot 基础环境，并提示可选模块。按需追加一个或
多个模块名即可安装对应集成（逗号和空格均可分隔）：

```bash
bash ./setup_env.sh --install franka pico4 xense tianji
bash ./setup_env.sh --verify franka pico4 xense tianji

# 安装所有可选模块（原来的默认行为）
bash ./setup_env.sh --install all
```

可用模块为 `tools`、`flexiv`、`franka`、`pico4`、`xense`、`taccap`、`revo2`、
`dex-retargeting`、`manus`、`wuji` 和 `tianji`。其中 `manus` 会拉取私有 SDK
的 Git LFS runtime 并使用当前 Python 编译 `manus_glove`；`wuji` 安装 Wuji SDK 与
`wuji-retargeting`。首次安装 `flexiv` 时，脚本会在 `$FLEXIV_RDK_PREFIX`（默认
`~/rdk_install`）缺失或不完整时自动构建固定版本的 Flexiv RDK 及其 C++ 依赖，
无需预先手动安装 RDK。选择 `taccap` 会自动包含 `xense`，准备并验证相同的
XenseSDK 原生运行时，因此 TacCap 可单独安装。首次安装 `pico4` 时，脚本会
下载并校验官方 XenseVR PC Service 0.2.1 `.deb`，通过 `sudo` 安装服务后使用包内
SDK 构建 Python binding，不会 clone 完整的 XenseVR 源码仓库；离线时可通过
`XENSEVR_DEB=/path/to/XenseVR-PC-Service_0.2.1_amd64.deb` 指定安装包。安装
Manus → Revo2 与 Manus → Wuji 分别使用：

```bash
mamba activate lerobot-xensehand

bash ./setup_env.sh --install manus revo2 dex-retargeting
bash ./setup_env.sh --verify manus revo2 dex-retargeting

bash ./setup_env.sh --install manus wuji
bash ./setup_env.sh --verify manus wuji
```

旧的 `./setup_manus.sh` 和 `./setup_manus_wuji.sh` 仍可使用，但仅作为上述命令的
兼容包装。`--install all` 现在也包含 Manus 与 Wuji，因此需要能访问对应私有子模块。
Franka 仍要求与机器人固件兼容的系统 `libfranka`、PREEMPT_RT 内核及实时权限。
完整说明见 [环境安装指南](docs/environment_setup.md)、[Manus 使用指南](docs/manus_control.md)
与 [Manus → Wuji 真手遥操作](docs/manus_wuji_control.md)。

## 0.6.0 迁移结构

官方 0.6.0 的设备工厂、配置解析、processor pipeline、数据集 v3 和可视化入口保持不变。
仓库内的 Xense 设备通过轻量注册表按 CLI 中的 `type` 延迟注册；仅当机器人与
遥操作器属于 Xense 支持组合时，`lerobot-teleoperate` 才分发到包含回零、重锚定、
脚踏使能和实时控制诊断的 Xense 后端。其他官方 0.6.0 设备继续走原生流程。

详细边界和验证结果见 [LeRobot 0.6.0 迁移说明](docs/lerobot_0_6_migration.md)。

## 快速开始

遥操和录制的硬件参数集中在 [`recipes/`](recipes/README.md)。命令行参数仍可覆盖
recipe，例如在首次上真机时追加 `--dryrun=true`。

Flexiv 双 Rizon4 + 双 TacCap follower + BiPico4：

```bash
bash ./setup_env.sh --install flexiv pico4 taccap

lerobot-teleoperate \
  --config_path=recipes/teleop/bi_flexiv_rizon4_rt/forward-01-taccap.yaml \
  --dryrun=true
```

确认左右机械臂、左右 follower 和图像映射后去掉 `--dryrun=true`。两个 TacCap
都是安装在机器人末端、接收 `left_gripper.pos` / `right_gripper.pos` 的
follower/slave，不是手持 leader。腕部与触觉相机按各自 USB 拓扑自动发现。
`dryrun` 只禁止遥操动作，连接阶段的 startup MoveJ 仍会执行；连接前必须先
核对 recipe 中的机器人 SN 与 start/home 位姿。完整步骤见
[TacCap follower 指南](docs/taccap_follower.md)。

Flexiv 双 Rizon4 + Wuji 双手（Pico 控臂 + Manus 控手，双脚踏使能）：

复合设备 `bi_flexiv_rizon4_rt_wuji`（双臂 + 双手 + 三路 RealSense 相机）由 `manus_bi_pico4` 遥操：一个脚踏（`P`）同时使能左右臂，另一个脚踏（`U`）复位；两只手由 Manus 手套持续跟随。手柄固定在手上，故不用夹持作使能。`--display_data=true` 时头部/左腕/右腕三路 RGB 送 rerun 实时可视化（相机故障只告警、不影响控制）。

Flexiv 在 `192.168.0.x`、Wuji 双手固定在 `192.168.50.x`。按以下顺序操作：

```bash
# 1) 网络：给接机器人交换机的网卡加一个 Wuji 子网副地址（enp12s0 按实际网卡名替换）
#    Flexiv 的 192.168.0.x 保持不变；重启后失效，持久化见下方 netplan 说明。
sudo ip addr add 192.168.50.100/24 dev enp12s0
ping -c2 192.168.50.110 && ping -c2 192.168.50.111   # 左手 110 / 右手 111 应可达

# 2) 启动 Pico（XenseVR）服务（另开一个终端，保持常驻）
/opt/apps/roboticsservice/runService.sh

# 3) 启动控制（本终端）。安装时已 setcap，无需 sudo。
lerobot-teleoperate \
  --config_path=recipes/teleop/bi_flexiv_rizon4_rt_wuji/forward-01.yaml
```

说明：

- Wuji 左右手按 IP 指派（左=`192.168.50.110`、右=`192.168.50.111`）。若物理左右手接反，用 `--robot.wuji_left_ip` / `--robot.wuji_right_ip` 对调，无需改代码。
- 三路相机按序列号识别、挂在复合设备上（`head`/`left_wrist`/`right_wrist`）；换工位用 `--robot.left_wrist_camera_sn/...` 覆盖。首跑用 `--display_data=true` 低速核对左右腕不接反。
- 第 1 步的副地址重启会丢失。持久化：新建 `/etc/netplan/60-wuji.yaml`，内容为 `enp12s0` 保留 `dhcp4: true` 并 `addresses: [192.168.50.100/24]`，然后 `sudo netplan apply`（默认路由仍走 `192.168.0.1`，不影响机械臂）。
- 首跑建议加 `--dryrun=true` 先核对左右映射与手指动作，再上真机、低速确认。

Tianji 双臂 + Wuji 双手（Pico 控臂 + Manus 控手，双脚踏使能）：

新工位的 Manus 接收器、U 单脚踏板和 L/R 双脚踏板 USB 权限可自动配置：

```bash
bash ./setup_tianji_wuji_usb.sh
```

将 Manus 接收器、U 单脚踏板和 L/R 双脚踏板接在同一个 Hub 上，停止遥操作，
在本地图形桌面终端以普通用户运行。按提示**拔掉 Hub 上行 USB 线 → 回车 →
插回 Hub → 回车**，一次识别三个新增设备，自动排除 Hub 本身并配置权限。
无需逐个拔插或手工查询 VID/PID；相同型号的两个脚踏控制器也会分别计数。
目标 Hub 应只接这三个设备；识别不明确时不写规则。规则保存到
`/etc/udev/rules.d/70-xense-tianji-wuji.rules`，重跑会备份旧文件。
脚本不修改脚踏的 U/L/R 键位，配置后需重启遥操作程序；`uaccess` 面向本地
桌面会话，不保证 SSH 会话的设备权限。

`tianji_arm_wuji` 保留 `tianji_arm + bi_pico4` 的机械臂控制、坐标变换和安全检查，
只在外层组合 Wuji 双手；遥操作端复用同一个 `manus_bi_pico4`。先启动 Pico 服务，
并确保主机能访问天机控制器和 Wuji 的 `192.168.50.x` 网段：

```bash
bash ./setup_env.sh --install tianji pico4 manus wuji
bash ./setup_env.sh --verify tianji pico4 manus wuji

# Wayland 首次安装一次脚踏 evdev 权限规则，然后重新插拔两个脚踏
sudo install -m 0644 config/udev/70-xense-footpedal.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=input

# 另开终端常驻
/opt/apps/roboticsservice/runService.sh

# 首次先用 --dryrun=true 核对左右臂和左右手映射
lerobot-teleoperate \
  --robot.type=tianji_arm_wuji \
  --robot.robot_ip=192.168.50.190 \
  --robot.control_mode=joint_impedance_pd \
  --robot.control_hz=60 \
  --robot.enable_inter_arm_distance_check=false \
  --teleop.type=manus_bi_pico4 \
  --fps=60 \
  --display_data=false \
  --dryrun=true
```

### 天机 Marvin CCS 双臂 + 舞肌工具动力学参数

使用官方 CCS 右臂工具动力学辨识轨迹，对双臂安装的舞肌末端组件完成带载/空载辨识。
该结果只适用于辨识时的末端装配状态；**更换灵巧手、转接件或其他末端载荷后需要重新辨识**。工具运动学参数（TCP）需单独标定，不能用上述质心坐标代替。
辨识成功结果如下（左右臂使用相同参数）：

- 质量 `m`：`1.0508684981 kg`
- 相对末端法兰的质心 `mcp_xyz`：`[15.4559994740, -17.4725515080, 156.2427762186] mm`
- 惯量 `I=[ixx, ixy, ixz, iyy, iyz, izz]`：`[0.0568371140, 0.0, 0.0, 0.0587507931, 0.0, 0.0014650213]`

按天机 SDK 的 `m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz` 顺序，可直接记录为：

```python
wuji tool
tool_dynamic = [
    1.0508684980857583,
    15.455999474038974,
    -17.47255150800114,
    156.2427762185974,
    0.05683711399900849,
    0.0,
    0.0,
    0.05875079306738924,
    0.0,
    0.0014650212761514636,
]
```

```python
TacCap tool
tool_dynamic = [
    0.7728457285779433,
    0.49133915860229604,
    24.13611877488365,
    78.32092580083444,
    0.02759972544759407,
    0.0,
    0.0,
    0.030053894711934717,
    0.0,
    0.0024016242589825296
]
[0.7728457285779433, 0.49133915860229604, 24.13611877488365, 78.32092580083444, 0.02759972544759407, 0.0, 0.0, 0.030053894711934717, 0.0, 0.0024016242589825296]
```

左右臂安装相同末端组件，动力学参数以实际配置为准。当前工位的 TCP 经手动调整，
采用法兰坐标系 Z 偏移 -80 mm、无相对旋转，即
`TCP XYZABC=[0,0,-80,0,0,0]`（mm/deg），不再使用此前的 +140 mm 描述。
该值是当前工位的配置，不代表 Wuji 通用掌心尺寸。`tianji_arm_wuji` 配置和配套 recipe
把 `tool_kine` 作为唯一 TCP 配置源传给 `libpytianji`：native controller 在进入 PD
模式前分别对左右臂调用 `SetTool`，native Pink/Pinocchio FK/IK 同时使用同一参数。
若需在 LeRobot 之外写入或核对控制器参数，也可运行
[`set_controller_tool_params.py`](third_party/libpytianji/vendor/tianji_arm/DEMO_PYTHON/set_controller_tool_params.py)；
该脚本可将两臂参数写入控制器文件、应用到当前控制会话并完成回读校验；官方
`showcase_set_save_tool.py` 保持原样，不再承载项目参数。

确认映射和动作方向后去掉 `--dryrun=true`。Wuji 默认左手
`192.168.50.110:7447`、右手 `192.168.50.111:7447`；接反时用
`--robot.wuji_left_ip` / `--robot.wuji_right_ip` 对调。脚踏 `P` 同时使能双臂，
脚踏 `U` 复位；复位后必须松开并重新踩下 `P` 才会再次接管。Linux 下脚踏优先
通过 `/dev/input/event*` 的 evdev 读取，因此 GNOME Wayland 也能获得可靠的按下/
释放事件；启动日志应出现 `Foot-press listener started with evdev`。若提示没有可读
设备，需要给当前用户配置 input 设备 ACL/组权限后重新登录。

Manus 双手按侧初始化：缺少一只手套时连接阶段会告警，已连接侧仍可控制对应 Wuji 手，
缺失侧保持当前姿态；两只手套均缺失时仍可用 Pico4 调试双臂。启动后再连接的手套不会
自动接管，需要重启当前 teleoperate/record 会话。Manus SDK 或服务本身连接失败仍会停止启动。

`tianji_arm_wuji` 也支持只安装一只物理 Wuji 手。在 teleoperate/record recipe 中将
`robot.wuji_hand_type` 与 `teleop.manus_hand_type` 分别设为 `left`、`right` 或 `both`。
例如仅安装左手并只开启左手套时，两项都设为 `left`；若双手均安装、但为了调试省电只
开启左手套，则前者保持 `both`、后者设为 `left`。单物理手模式只连接所选 Wuji，并只在
数据集中产生该侧 20 个手关节字段；不会用 fake hand 补出另一侧数据。该选择不会
自动改写左右臂的 `tool_kine/tool_dyn`，须按法兰上实际保留的手、转接件或其他负载填写。
Wuji 组合不保留基础 Tianji 的 `left/right_gripper.pos` 普通夹爪占位维度；
当前单手配置的 action/state 分别为 38/66 维，不影响 Tianji + TacCap。

Franka Research 3 + TacCap follower + Pico4：

```bash
lerobot-teleoperate \
  --config_path=recipes/teleop/franka_research3/taccap.yaml \
  --dryrun=true
```

更换工具或负载后必须重新确认 Franka Desk 中的质量、质心、惯量和末端坐标，并
重新验证控制参数。右侧 follower 使用 `--robot.taccap_side=right` 覆盖 recipe。
录制 smoke test 使用对应的
`recipes/record/franka_research3/taccap-test.yaml`。

### 数据录制（lerobot-record）

用 `lerobot-record` 通过遥操作采集数据集，命令结构与 `lerobot-teleoperate` 一致，
再加 `--dataset.*` 参数。以 Tianji 双臂 + Wuji 双手为例：

```bash
lerobot-record \
  --robot.type=tianji_arm_wuji \
  --config_path=recipes/record/tianji_arm_wuji/grasp_demo.yaml \
  --robot.robot_ip=192.168.50.190 \
  --teleop.type=manus_bi_pico4 \
  --dataset.repo_id=<用户名>/<数据集名> \
  --dataset.single_task="抓取物体" \
  --dataset.num_episodes=20 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --display_data=true
```

复位踏板（`U`）会把双臂送回 home、双手张开归零，**这段复位轨迹会被录进 episode**
（用机器人实测状态作为 action，不产生断帧）；复位结束后需重新踩下使能踏板 `P`
才恢复手臂与手指跟随。`bi_flexiv_rizon4_rt_wuji` 用法相同，只需替换 `--robot.type`。

#### 用 recipe 文件（`--config_path`）

与其在命令行堆参数，可把整套配置存成 YAML recipe，用 `--config_path` 传入：

```bash
lerobot-record --config_path=recipes/record/tianji_arm_wuji/grasp_demo.yaml
```

- 设备类型写在 recipe 里（`robot: {type: ...}` / `teleop: {type: ...}`）——CLI 会
  **从 `--config_path` 指向的 YAML 读取设备类型**并按需注册对应配置类，因此无需再
  在命令行传 `--robot.type` / `--teleop.type`。
- 命令行参数仍可覆盖 recipe 内任意字段，例如
  `--config_path=... --dataset.repo_id=me/other --dataset.num_episodes=5`。
- 现成 recipe 见 [recipes/](recipes/)（含 `tianji_arm_wuji`、`bi_flexiv_rizon4_rt_wuji`），
  编写约定见 [recipes/README.md](recipes/README.md)（Wuji 参数用扁平 `wuji_*` 字段，
  勿写嵌套 `wuji:` 块）。

#### 录制深度

在 recipe 的某台相机块里设 `use_depth: true` 即录制该相机深度（`(H,W,1)` uint16 毫米，与
彩色时间同步、不做像素对齐）。深度用 12-bit HEVC 无损编码；无效/无回波像素编码为码 0、解码
为 0（**不再**被糊成假的近距/远距），深度统计与视频落在同一域。

可按相机设置量化范围（米），键为完整深度特征名：

```yaml
dataset:
  depth_encoder_overrides:
    observation.images.head_depth:
      { depth_min: 0.3, depth_max: 2.0, use_log: false, shift: 3.5 }
    observation.images.left_wrist_depth:
      { depth_min: 0.1, depth_max: 1.5, use_log: true, shift: 3.5 }
    observation.images.right_wrist_depth:
      { depth_min: 0.1, depth_max: 1.5, use_log: true, shift: 3.5 }
```

未列出的深度相机回退到默认 `depth_encoder`（0.01–10m）。

> **播放 RGB 全黑？** RGB 用 AV1 编码，部分播放器解不了 → 显示黑屏（数据本身正常）。用较新的
> mpv / VLC，或转码：`ffmpeg -i <in>.mp4 -c:v libx264 -pix_fmt yuv420p <out>.mp4`。

#### 遥操作测试（不录制）

只想测控制并在 rerun 里看 RGB/深度，用 teleop recipe（不含 `dataset` 块）：

```bash
lerobot-teleoperate --config_path=recipes/teleop/tianji_arm_wuji/grasp_demo.yaml --dryrun=true
```

首跑务必先 `--dryrun=true` 核对左右手/手指映射，确认无误再去掉 `--dryrun` 实控。

更多组合的完整命令见 [命令参考](docs/commands.md)：

- Franka + Pico4
- Franka + TacCap + Pico4
- 双 Flexiv + 双 TacCap + BiPico4
- Franka + Revo2 + Pico4 hand tracking
- Franka + Revo2 + Manus + Pico4 脚踏使能
- 独立 Revo2 手遥操作
- 数据集录制

## 控制与硬件文档

- [Franka 笛卡尔阻抗控制](docs/franka_control.md)：控制模式、默认参数、回零与负载注意事项
- [Manus 使用指南](docs/manus_control.md)：安装、标定、离线校验和脚踏使能
- [Manus → Wuji 指南](docs/manus_wuji_control.md)：不使用 LeRobot kinematics 的 Wuji 真手遥操作
- [TacCap follower 指南](docs/taccap_follower.md)：机器人末端单/双 TacCap 的发现、标定、遥操与录制
- [TacCap 独立手持采集指南](src/lerobot/robots/taccap_gripper/README.md)：`robot.type=taccap_gripper` 自驱录制；与 follower 遥操链路不同
- [遥操与录制 recipes](recipes/README.md)：自包含工位配置、覆盖规则与可执行入口
- [命令参考](docs/commands.md)：当前支持组合的可执行命令
- [Franka/Pico4 调试周报](docs/reports/franka_teleop_2026_w31.md)：参数实验与历史结论

Manus teleoperator 已接入 `lerobot-teleoperate` 和 `lerobot-record`。支持
`franka_research3_dexhand + manus_pico4` 与 `revo2_hand + manus_hand` 录制组合。
首次连接新硬件时，先使用 `--dryrun=true` 检查输入与映射。
