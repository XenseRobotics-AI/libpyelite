# Diagonal-07 · CS66 + DH AG95

查看入口：[diagonal-07-elite-cs66-dh-ag95.urdf](urdf/diagonal-07-elite-cs66-dh-ag95.urdf)。
装配 ID：`assembly.diagonal_07_elite_cs66_dh_ag95`；权威输入为 [assembly.dh-ag95.json](assembly.dh-ag95.json)，[部件表](COMPONENTS.dh-ag95.md)。

保留基础版双 CS66、A1 支架、控制柜和桌子，两侧分别直接固定连接`dh_robotics.ag95:default`（官方视觉与机械定义，Xense自制碰撞）。不使用 TacCap 或额外连接件，不复制网格或独立维护机械参数。生成后的 URDF 保留各部件原始关节、限位、质量、惯量及 mimic，仅增加实例前缀与外部固定连接。

## 安装与参考布局

CS66 `flange` 与 AG95 `gripper_base_link` 的安装面均为局部 Z=0，平移 `0 0 0`。实际网格四螺孔约在 `(±17.678, ±17.678)` mm，即 PCD 50 mm。截线拟合在当前网格精度下最大中心偏差约 0.0063 mm；该数字不是实物装配公差。

AG95 沿本地 X 开合，TacCap 沿本地 Y 开合。因此，相对参考 TacCap 左侧 0°／右侧 −90°，AG95 采用左侧 +90°／右侧 0°（绕法兰 Z），保持相同的世界开合轴线。直接连接后 AG95 根平面比 TacCap 根平面少 12 mm 转接件厚度，不用平移补回该厚度。

[网格、源哈希和安装证据](provenance/dh-ag95/mounting_reference.json)保存了测量方法及来源；[可复现测量脚本](../../../reports/diagonal07-dh-ag95-2026-10-29/check_mounting.py)。四个螺孔与额外定位孔分开处理。AG95 中央凸台伸至 Z=−3.5 mm，属于定位配合，不能据此把安装面悬空平移；中央凸台、定位销、紧固件与硬件装配仍待验收。

## 初始姿态与操作

URDF 本身保留官方 q=0。打开该模型后，在 Robot Viewer 动画面板选择 **☰ → Import JSON**，导入 [start_pose.dh-ag95.robotanim.json](config/start_pose.dh-ag95.robotanim.json)，即可显示与 TacCap 参考版相同的 12 个机械臂初始角。

初始角保存在 [start_pose.dh-ag95.json](config/start_pose.dh-ag95.json)，参考文件已冻结在 provenance 中。CS66 每侧只有六个机械臂关节；夹爪主动关节为 `left_tool_gripper_finger1_joint`／`right_tool_gripper_finger1_joint`，范围 `[0, 0.6524]` rad。每侧其余七个关节保持官方 mimic。此动画仅写机械臂轨道；从新加载模型导入时，AG95 保留源 q=0。若此前拖动过夹爪，请先将两个主动关节设回 0。不要套用 TacCap 的夹爪角度或动画。

重建仅本配方，可在仓库根运行：

```python
import sys
from pathlib import Path
sys.path.insert(0, "scripts")
from lib.catalog import discover
from lib.assembly import build
from lib.viewer_pose import build as build_pose
recipe = Path("assets/assemblies/diagonal-07-elite-cs66/assembly.dh-ag95.json")
assets = discover()
build(recipe, assets)  # 输入变更须先审阅并显式更新锁
build_pose(recipe, assets)
```

## 验证边界

本次验证 URDF 结构、原参数保留、四孔网格中心、开合方向、静态网格渲染及 ZIP 解压搬移。初始姿态需要手动导入；静态渲染不代表交互 Viewer 人工验收。AG95 与 AG-160-95 实物版本对应、源惯性/关节/接触、全行程干涉、动力学、硬件装配与标定仍未验证；来源只读审计由另一会话负责。本次不新增 MJCF/USD。

## 自制碰撞模型

现配方消费[AG95资产](../../grippers/dh-robotics/ag95/README.md)：每侧9凸体/612三角面，visual、机械参数、左右安装相位和根坐标不变；全装配仍49link/48joint。profile仅描述独立工具，不含完整工位允许对。

[本轮报告](../../../reports/ag95-collision/README.md)另列完整曲面覆盖、101档开合、全部原始碰撞对与间隙、离线计时及安装搬移。原厂闭合端约2.04mm重叠/COM与惯量疑点保留；不能将新凸体称为物理接触模型或端到端250Hz认证。
