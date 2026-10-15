# diagonal-08-elite-cs66-taccap

双 CS66、原 A1 支架／控制柜及居中工作台，加两个与 01 平台同款的连接件和裸 TacCap 从夹爪。
同目录基础版继续使用 `assembly.json`；本版使用 [assembly.taccap.json](assembly.taccap.json)，
部件见 [COMPONENTS.taccap.md](COMPONENTS.taccap.md)。

## 查看工位姿态

打开 [diagonal-08-elite-cs66-taccap.urdf](urdf/diagonal-08-elite-cs66-taccap.urdf)，
在 Robot Viewer 动画面板 **☰ → Import JSON** 中导入
[config/start_pose.taccap.robotanim.json](config/start_pose.taccap.robotanim.json)。
两臂按本工位真实角度架起，夹爪开合方向相对水平面倾斜约 **5.9°**，工具轴朝前下方。
两侧主动夹指默认 +28.92° 张开，从动指为 −1 mimic；只创建两个主动指动画轨道。
14 条轨道包含 12 个机械臂关节和 2 个夹爪驱动，全部使用弧度。

URDF 直接加载时仍为官方机械臂零位与夹爪闭合位；工位动画是可选状态，
不改写关节原点、轴向或限位，也不随普通 URDF bundle 强制交付。

## 连接与来源

每侧：`<side>_flange` → `<side>_adapter_taccap_mount_link` → `<side>_tool_taccap_base_link`。
CS66 的锁定官方模型中，`wrist_3_link`、`flange`、`tool0` 原点和轴向一致，+Z 沿末端朝外。
法兰至连接件无平移，左右绕 +Z 的安装相位分别为 **0°／−90°**；连接件至夹爪保留源变换
`xyz="0 0 0.012" rpy="0 0 0"`。根据本工位的腕部角度选择四孔的 90° 档位，
使夹爪略倾斜、夹指偏置向下；不借用其他工位的关节角。

[安装依据](provenance/taccap/mounting_reference.json)记录用户同款连接件要求、
01／02 参考配方、CS66／连接件网格哈希和截面孔位。四个孔中心位于 PCD 50 mm，
模型中心差小于 0.001 mm；腕部端面 Z=0 对应连接件安装面 Z=0。
CS66 另有一个定位孔，此连接件没有对应定位孔；定位销是否使用、螺纹、螺钉规格与公差需实物核验。
这些是网格几何观察，不代表加工或实物配合精度。

07 的 TacCap 配方与 `config/start_pose.taccap.json` 是权威输入；
08 的同款配方和角度缓存通过 `configuration_source` 跟随 07。
两套工位分别导入自己目录内的动画文件，元数据记录各自模型与哈希。
基础版工位角、装配配方与参考来源快照保存在 `provenance/taccap/`。

## 验证范围

只增加资产引用和固定连接，机械臂、连接件和裸夹爪的源参数均保持原值。
连接件质量仅有部件 JSON 中的几何估计，质心／惯量仍缺来源，不作补猜。
候选安装、全行程干涉、紧固件、相机标定、承载和动力学／接触尚待实物验收。
本次仅新增 URDF 与 Viewer 配置，无新增 MJCF/USD 或仿真调用。

检查记录：[EL-02 报告](../../../reports/elite_taccap_assemblies_validation.json)。
静态图：[官方零位](../../../reports/previews/assembly.diagonal_08_elite_cs66_taccap_default.png) /
[架臂工位姿态](../../../reports/previews/assembly.diagonal_08_elite_cs66_taccap_start.png)。

维护命令（从仓库根目录执行）：

```bash
python scripts/assemblies/build_assemblies.py
python scripts/assemblies/build_viewer_poses.py
python scripts/assemblies/build_assemblies.py --check
python scripts/assemblies/build_viewer_poses.py --check
```
