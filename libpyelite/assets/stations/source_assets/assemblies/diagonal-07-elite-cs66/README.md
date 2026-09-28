# diagonal-07-elite-cs66

Two catalog `elite.cs66` arms, `elite_cs66.a1_stand` and `worktable.120cm`. The stand, controller cabinets, two arm base frames and table pose come from the hash-locked `A1-CS66总装.STEP`; Diagonal-08 uses the same configuration. End effectors remain pending. CAD arm display joint positions are not imported; assembly q=0 displays the LeRobot bench 07 start pose while the catalog arm remains unchanged.

After the stand's 180° world-Z turn, the controller's left arm attaches to the CAD `right_arm_mount` and the right arm to `left_arm_mount`. Local Z corrections of +90° and -90° respectively align both base rotations with the LeRobot recipe's `left_world_rotation` and `right_world_rotation` matrices. The table remains fixed.

Viewer: [URDF](urdf/diagonal-07-elite-cs66.urdf). Source mapping is in [the A1 import notes](../../../docs/A1_CS66_AND_GRIPPERS_IMPORT.md). Clearance, contact, dynamics and hardware calibration remain unvalidated.
[Initial joint angles and shifted assembly coordinates](../../../docs/PLATFORM_INITIAL_POSES.md).

[部件、必需项与可选连接件](COMPONENTS.md)。
