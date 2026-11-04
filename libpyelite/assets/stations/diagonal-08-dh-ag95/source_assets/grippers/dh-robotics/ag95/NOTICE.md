# DH AG95 来源与自制碰撞说明

视觉网格与机械定义采用厂家原始来源；默认碰撞网格由Xense基于官方visual自行生成，替换原有碰撞配置。
自制碰撞未获厂家物理/硬件验收。原URDF与原资产记录保存在provenance，仅供溯源。

以下为导入时的历史来源记录（其中碰撞未重估说明仅适用于替换前快照）：

# dh_robotics.ag95 来源记录

本资产为本地源文件快照，来源仓库及提交见 asset.json。
files 逐文件记录源路径、源 SHA-256 与本地 SHA-256；提交号不代替工作区文件哈希。
仅调整资源路径或按记录截取 link 子树；关节、单位、惯性和碰撞参数没有重新估算。
provenance/urdf 保留原始输入，仅用于比对，不是独立可视化入口。
质量、关节限位、零位、轴向及实物一致性待人工核对。

本地源未提供可确认的资产授权；不推断许可证或对外再分发权限。
- Official AG95 source; correspondence to requested AG-160-95 hardware revision requires confirmation.
- Removed world, 100 kg demonstration root box, ee_link and bridge; all gripper links/joints preserved.
- Package declares BSD; no scoped full license text found.
