# diagonal-08-elite-cs66

Two catalog `elite.cs66` arms, `elite_cs66.a1_stand` and `worktable.120cm`. The stand, controller cabinets, two arm base frames and table pose come from the hash-locked `A1-CS66总装.STEP`; this configuration copies Diagonal-07. End effectors remain pending. CAD arm display joint positions are not imported; assembly q=0 displays the LeRobot bench 08 start pose while the catalog arm remains unchanged.

As in bench 07, the controller left/right arms attach to the post-rotation CAD right/left mounts. Local Z corrections of +90°/-90° align the base rotations with the LeRobot recipe matrices; the table stays fixed.

Viewer: [URDF](urdf/diagonal-08-elite-cs66.urdf). Source mapping is in [the A1 import notes](../../../docs/A1_CS66_AND_GRIPPERS_IMPORT.md). Clearance, contact, dynamics and hardware calibration remain unvalidated.
[Initial joint angles and shifted assembly coordinates](../../../docs/PLATFORM_INITIAL_POSES.md).

[部件、必需项与可选连接件](COMPONENTS.md)。
