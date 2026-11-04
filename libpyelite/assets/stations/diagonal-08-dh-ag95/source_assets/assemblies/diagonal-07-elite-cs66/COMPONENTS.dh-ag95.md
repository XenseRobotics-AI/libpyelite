# diagonal-07-elite-cs66-dh-ag95：部件与连接

装配 ID：`assembly.diagonal_07_elite_cs66_dh_ag95`，以 [assembly.dh-ag95.json](assembly.dh-ag95.json) 为准。

| 配方实例 | 已登记资产及入口 | 状态 |
| --- | --- | --- |
| `left_arm` | `elite.cs66:default` | 已引用；重现当前 URDF 必需 |
| `right_arm` | `elite.cs66:default` | 已引用；重现当前 URDF 必需 |
| `stand` | `elite_cs66.a1_stand:default` | 已引用；重现当前 URDF 必需 |
| `table` | `worktable.120cm:default` | 已引用；重现当前 URDF 必需 |
| `left_gripper` | `dh_robotics.ag95:default` | 已引用；重现当前 URDF 必需 |
| `right_gripper` | `dh_robotics.ag95:default` | 已引用；重现当前 URDF 必需 |

每侧：支架 → 官方 CS66 → 官方 AG95；无额外转接件。左右夹爪独立驱动，保留各自七个源 mimic，前缀分别 `left_tool_`、`right_tool_`。源参数及网格由部件维护，输入 URDF 由配方 SHA-256 锁定。安装位置、四孔测量与验证边界见 [README.dh-ag95.md](README.dh-ag95.md)。
