# diagonal-07-elite-cs66-taccap：部件与连接件

装配 ID：`assembly.diagonal_07_elite_cs66_taccap`，以 [assembly.taccap.json](assembly.taccap.json) 为准。

| 配方实例 | 已登记资产及入口 | 状态 |
| --- | --- | --- |
| `left_arm` | `elite.cs66:default` | 已引用；重现当前 URDF 必需 |
| `right_arm` | `elite.cs66:default` | 已引用；重现当前 URDF 必需 |
| `stand` | `elite_cs66.a1_stand:default` | 已引用；重现当前 URDF 必需 |
| `table` | `worktable.120cm:default` | 已引用；重现当前 URDF 必需 |
| `left_adapter` | `xense.flexiv_elite_franka_to_taccap_slave:default` | 已引用；重现当前 URDF 必需 |
| `left_gripper` | `xense.taccap_slave:default` | 已引用；重现当前 URDF 必需 |
| `right_adapter` | `xense.flexiv_elite_franka_to_taccap_slave:default` | 已引用；重现当前 URDF 必需 |
| `right_gripper` | `xense.taccap_slave:default` | 已引用；重现当前 URDF 必需 |

每侧按支架 → CS66 → 独立 `flexiv_elite_franka_to_taccap_slave` → 裸 TacCap Slave 维护。
连接件与 01 平台同款，分层组织参考 02 平台；不用 `with_mount` 重复叠装。
两个主动指独立控制，另一指保留源 −1 mimic；质量、惯量、碰撞和网格由各部件来源维护。
安装候选已核对坐标和网格四孔中心，实物紧固、定位销、全行程干涉、接触和动力学待验收。
