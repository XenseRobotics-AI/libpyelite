# TacCap flange adapter provenance

Extracted from the local `xense.taccap_slave:with_mount` URDF and its
`Asm_link.STL`, originally from XenseRobotics-AI/robosuite-xense revision
`865e81be89706f8d1d25ba83119c1d0f3de8d4e9`. Source URDF snapshots, hashes
and the unchanged mesh are retained. See `provenance/source_NOTICE.md` and
`LICENSES/robosuite.txt` for the original notice and unresolved asset-specific
redistribution provenance. No new mesh license is inferred.

2026-10-10: user explicitly identified the existing mount as this adapter and
authorized a uniform CNC 6061-T6 aluminium mass estimate. Density 2700 kg/m³
is applied to the closed STL volume; fasteners are excluded. This is not a
measured mass or a vendor-confirmed material. COM and inertia are deferred;
no inertial block or new contact parameters are authored. Source collision
geometry is retained without claiming contact validation. The gripper's
0.415 kg base mass and all existing gripper resources remain unchanged.
