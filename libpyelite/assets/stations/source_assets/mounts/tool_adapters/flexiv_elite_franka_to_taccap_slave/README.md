# flexiv_elite_franka_to_taccap_slave

Flexiv / Elite / Franka 机械臂与 TacCap 从夹爪之间的独立连接件。
按用户指定，从 `xense.taccap_slave:with_mount` 中拆出 `taccap_mount_link`。

- 资产 ID：`xense.flexiv_elite_franka_to_taccap_slave`。
- [独立 URDF](urdf/flexiv_elite_franka_to_taccap_slave.urdf)：只有一个连接件 link。
- [STL](meshes/Asm_link.STL)：原始字节保留，单位 m，无缩放、重网格化。
- [质量记录](config/mass.json)：6061-T6 CNC 铝合金，均匀密度 2700 kg/m³；
  闭合网格体积约 12.753727 cm³，计算质量 **0.034435064 kg（34.435 g）**，不含紧固件。
  密度依据为 [6061-T6 密度表](https://wpw.bnl.gov/rgupta/wp-content/uploads/sites/9/2023/02/section14.pdf)
  的 300 K 数据 2.70 g/cm³。材料由用户指定，质量为几何估算，未称重。
- 质心与惯量暂不处理，JSON 明确为 null；URDF 不填写不完整的 `<inertial>`，
  以免默认原点被误认为已知质心、零惯量被误认为真实值。viewer 可能不显示该估算质量。

坐标原点保留源模型的机械臂侧安装面中心，+Z 指向夹爪；网格的 visual/collision
旋转仍为 `0 -pi/2 0`。到裸夹爪 `taccap_base_link` 的固定变换保留为
`xyz="0 0 0.012" rpy="0 0 0"`。装配时使用 `xense.taccap_slave:default`，
避免与 `with_mount` 重复叠装。原夹爪资产和耦合、质量/惯量均未修改；
源模型中的 0.415 kg 属于夹爪基座，不是连接件质量。

现有规划平台与候选 frame 见 [plan.json](plan.json) 和 [interfaces.json](../interfaces.json)。
名称中的三个品牌及原规划保留的 Tianji 配对不构成全部型号兼容性验证；
[Forward-04 TacCap 候选装配](../../../assemblies/forward-04-flexiv-rizon4r/README.taccap.md)
已采用历史 Forward44R 法兰桥接，左右分别绕法兰 Z 轴 +90°/−90°；
其他型号臂侧变换及全部螺孔/定位销、实物安装时钟方向仍待核对。
源碰撞网格原样保留，接触、承载与动力学未验证。

来源 URDF 快照见 `provenance/urdf/`，来源哈希、密度、提取方式和连接变换见
[derivation.json](provenance/derivation.json)，版权边界见 [NOTICE.md](NOTICE.md)。
重新生成：`python scripts/generators/generate_taccap_flange_adapter.py`；
检查输出：同一命令添加 `--check`。重新生成后须刷新 asset.json 中的派生文件哈希。
当前交付 URDF/STL 与质量 JSON；无新增 MJCF/USD。

2026-10-10：同一连接件还用于 01/02/05 的独立 TacCap 候选装配。
01/05 沿用 04 左右 +90°/−90° 档位；02 使用 0°/0°。
候选装配 ID 见 plan.json；安装与来源记录分别由各 assembly.json 维护。

2026-10-12：同一连接件用于 [Elite 07](../../../assemblies/diagonal-07-elite-cs66/README.taccap.md)／
[08](../../../assemblies/diagonal-08-elite-cs66/README.taccap.md) 的 TacCap 候选装配。
锁定 CS66 的 flange 与 tool0 坐标一致，法兰至连接件无平移，左右绕 +Z 分别 0°／−90°；
至夹爪仍为 +Z 12 mm。截面四孔中心在这两个相位均对应；CS66 另有定位孔，
连接件没有对应孔。实物定位销使用、紧固件、安装相位及公差待确认。
这不更改连接件 URDF、网格和质量记录；候选配方 ID 已加入 plan.json。
