# DH Robotics AG95

唯一资产ID：`dh_robotics.ag95`；默认入口：[urdf/ag95.urdf](urdf/ag95.urdf)。

视觉与机械定义来自官方DH-Robotics/dh_gripper_ros@f59f9c2f4bc8eb116448b1d798791424bf64e337。
原始五份visual STL字节、0.001缩放、九link/八joint/七mimic、continuous类型、
限位、轴向、质量/惯量与安装根保持厂家来源值。

## 自制碰撞模型

默认URDF的碰撞模型由Xense自行生成，替代厂家原有碰撞配置；不是厂家提供或验收的碰撞模型。
复用Elite支持平面算法，每个已有刚性link一个凸体：9凸体/612三角面，源碰撞为42,216面。
完整visual三角面支持外包，最多20支持平面，最小2mm padding；2mm不是最大几何误差。
碰撞网格在meshes/collision，profile在collision/default.json；各凸体属于各自link。
消费端须绑定完整工位frame和碰撞对策略，普通URDF加载器不一定按凸体处理STL。

原始上游URDF、替换前独立URDF、原资产清单与文件SHA保留在provenance，仅供追溯，不作为第二公共入口。
算法配方：collision/recipe.json；生成记录：provenance/collision-derivation.json。
复现：python scripts/generators/generate_ag95_collision.py [--check]。
完整覆盖/开合/间隙/离线计时/安装搬移验证见仓库reports/ag95-collision/README.md。

## 来源及验证边界

AG95与用户AG-160-95具体硬件修订对应、完整许可文本仍待确认。
厂家原COM=0、惯量疑点和闭合端约2.04mm指尖重叠保留，不改变关节限位掩盖问题。
URDF/profile的几何验收不代表实物接触、动力学、CCD或PICO+IK+碰撞完整250Hz验收。
本资产不新增MJCF/USD。许可与来源见NOTICE.md及asset.json。
