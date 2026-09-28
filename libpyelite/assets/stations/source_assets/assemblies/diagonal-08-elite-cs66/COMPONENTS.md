# diagonal-08-elite-cs66：部件与连接件

装配 ID：`assembly.diagonal_08_elite_cs66`。当前可用 URDF 的部件以 [assembly.json](assembly.json) 为准；
以下“目标/可选”仍是规划，未写入配方、未生成网格或安装变换。

## 当前 URDF 的必需部件

| 配方实例 | 已登记资产及入口 | 状态 |
|---|---|---|
| `left_arm` | `elite.cs66:default` | 已引用；重现当前 URDF 必需 |
| `right_arm` | `elite.cs66:default` | 已引用；重现当前 URDF 必需 |
| `stand` | `elite_cs66.a1_stand:default` | 已引用；重现当前 URDF 必需 |
| `table` | `worktable.120cm:default` | 已引用；重现当前 URDF 必需 |

## 目标配置与可选方案

| 要求 | 部件/连接件 | 关系与当前状态 |
|---|---|---|
| 目标必需，待接入 | [`forward01_04_05_diagonal07_08_to_d435i_head`](../../mounts/camera_adapters/forward01_04_05_diagonal07_08_to_d435i_head/README.md) + D435i | 07/08 与 01/04/05 共用头部方案；D435i 本体尚未登记。 |
| 末端型号二选一，待接入 | [`common_arm_flange_to_xenseflare`](../../mounts/tool_adapters/common_arm_flange_to_xenseflare/README.md) + `xense.xenseflare:default` | 每侧选此末端时同时需要该转接件。 |
| 末端型号二选一，待接入 | [`common_arm_flange_to_taccap_slave`](../../mounts/tool_adapters/common_arm_flange_to_taccap_slave/README.md) + `xense.taccap_slave:default` | 每侧选此末端时同时需要该转接件；不用 `with_mount` 叠装。 |

双臂目标平台的每一侧需在 XenseFlare 与 TacCap Slave 中确定一种末端；
两种是可选型号，选定后相应工具和转接件一起成为该侧必需部件。
共用法兰接口是用户规划，具体安装尺寸/位姿尚未验证。

连接件命名与平台映射见[统一规划](../../mounts/ADAPTER_PLAN.md)。
规划件尚无 `asset.json`；补入前需验证机械接口、左右安装、质量与碰撞，
更新配方输入哈希并重新生成/检查 URDF。
