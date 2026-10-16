# TacCap asset provenance

## G1 slave and mounting adapter

This private-development package extracts the TacCap G1 slave model and five
STL files from XenseRobotics-AI/robosuite-xense, revision
`865e81be89706f8d1d25ba83119c1d0f3de8d4e9`. Original paths and checksums are in
`config/model.json`; `LICENSES/robosuite.txt` preserves the source repository's
license notice.

The source describes the CAD input as a SolidWorks-exported TacCap URDF and
the adapter as originating from `forward-44R-silex/A-/A-.urdf`. No immutable
original CAD revision or asset-specific redistribution grant was recorded.
The source assembly notice labels project-supplied assemblies Apache-2.0,
while its Forward44R README says redistribution provenance is incomplete.
This extraction preserves that uncertainty and does not infer a license for
meshes from the surrounding Python code. Confirm those terms before public
distribution; no GitHub repository or package-index release is created here.

The Xacro is a mechanical transcription of the existing simulation model.
Generated URDF and MJCF files use local relative mesh paths. They retain the
existing simulated masses and mounting transform, including
approximations documented in `config/model.json`. They are not hardware
calibration data. The source filename `viusal_link.STL` is intentionally
preserved for provenance.

Only the TacCap component and its Asm_link adapter are included. No Flexiv arm,
Forward44R stand or Wuji hand assets are redistributed by this package.

2026-09-17 correction: joint axes/ranges/coupling now follow the supplied master
reference convention (common -Z axes, opposite coordinates, 28.92-degree opening).
This intentionally supersedes the imported one-radian range; it is not a new
hardware calibration. Original CAD mesh hashes and source fixture are retained.

2026-09-23 adapter geometry correction: the imported `base_link.STL` already
contains the mounting adapter. All five original STL contents and their recorded
hashes remain unchanged. The imported `base_link.STL` is stored locally as
`base_link_with_mount.STL`; `source_mesh_renames` records this filename change.
`scripts/generate_slave_mesh.py` removes the 1,550
triangles matching `Asm_link.STL` after applying the inverse `adapter_to_base`
transform, producing `base_link.STL` with 69,846 triangles. Every
retained triangle record is byte-identical and remains in source order. Units,
source/output hashes, matching tolerance and transformation are recorded in
`config/model.json` under `derived_meshes`.

Both variants use the derived base for visual and collision geometry. Only the
mounted variant adds the separate adapter. The original base STL and USD mesh
layer remain available as `base_link_with_mount.STL` and
`base_link_with_mount.usdc` source artifacts; current entry points do not reference
them. Existing masses/inertias are retained without estimating the removed
part's physical parameters. Contact geometry changes, and no contact or hardware
calibration is claimed. The separately rendered cover also exists in the source
base mesh; that pre-existing duplication is outside this adapter correction.

2026-10-08 URDF revision: user-confirmed flange reference and semantic link/joint
names. Master virtual flange follows the rear/holding-plane 23.5 mm datums;
slave retains the original Asm_link flange, virtual when adapter geometry is
omitted. Original body-relative EEF positions/orientations and all mechanical
parameters are retained. Pre-change URDF/Xacro snapshots are byte-preserved
under provenance/urdf. Existing MJCF/USD retain their legacy frames and names;
this revision applies to URDF/Xacro only.

2026-10-09 URDF revision: slave flange axes changed to +Z along the tool axis,
+Y toward the left finger, +X toward the finger offset, matching the existing
EEF axes. The with_mount origin stays on the Asm_link robot-side face; the
standalone origin moves to the body/Asm_link mating face (12 mm along +Z).
Meshes, joints and remaining mass properties are unchanged; MJCF/USD are
unchanged. slave_cover_link is removed from the URDF (its geometry is part of
base_link.STL; its 0.068231 kg imported mass is dropped). slave_eef_link is a
massless frame at the former cover frame origin with +Z toward the tips.

2026-10-09 format revision: MJCF/USD are regenerated from the current URDFs and
share their flange root, axes, names, EEF and cover removal.

2026-10-09 structure revision: slave_body_link is merged into taccap_base_link
(body meshes and the same 0.415 kg inertial expressed through origins), which
always sits on the gripper mounting face; the optional adapter is the separate
root taccap_mount_link. Links and joints use the taccap_ prefix instead of
slave_. Derived base_link.STL bytes are unchanged.

2026-10-09: massless wrist camera and REP-103 optical frames added from the
mesh dome-window geometry; position depth, roll and calibration are unverified.

2026-10-09: taccap_camera_link moved to the CAD-marked camera point
[-0.02275, 0, 0.06125] m from taccap_base_link (user-provided SolidWorks
measurement); orientation and calibration remain unverified.

2026-10-09: EEF unified as the centre of the closed fingertips' most distal
face (mesh-derived, shared by master and slave).
