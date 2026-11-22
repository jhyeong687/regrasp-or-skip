# Printable camera and tactile accessories

## 26 cm RGB mount

Each nominal lens center is 260 mm from the robot's centerline, 50 mm forward of its base origin and 240 mm above the table. The local forward vector is pitched down 27.08°. The left and right yaw angles are −73.91° and +73.91° in the scene's base frame.

Print a left and a right rail, lower mast and upper mast. The longest piece is 143.0 mm. Use black for the rails and masts and violet for the two PCB support parts. The STL files are millimetre-scale geometry. The [color STEP assembly](rgb-26cm/mount-assembly.step) shows the parts together.

[STL directory](rgb-26cm/STL) · [Korean assembly guide with fasteners](rgb-26cm/assembly.ko.md) · [Clearance and optical checks](../data/cad-clearance.json)

The new rails bolt onto the existing base wings and rest on the work surface at both ends. The mast fits in two pieces with a bolted overlap. Assembly needs about 538 mm of flat table width. Allow space for a brim on the 150 mm print bed. The SA26MCM reference brackets fit a 24–30 mm nominal square M2-hole pattern. Check the two physical boards, connectors and cables before printing a complete set.

## Wrist and fit checks

| Part | Interface |
| --- | --- |
| [D435i wrist bridge](wrist-and-fit/D435i_Wrist_Bridge_R5.stl) | Keeps RGB and depth on the wrist. Verify bolt engagement and lens alignment on the physical camera. |
| [PaXini forearm saddle](wrist-and-fit/PaXini_Forearm_Saddle_R5.stl) | Supports the communications box with a short cable route. The case envelope is provisional. |
| [M3025 screw fit coupon](wrist-and-fit/M3025_Screw_Fit_Test.stl) | Test hole fit with the same printer, material and slicer settings as the intended mount. |

Corresponding STEP files are in the same directory. The R4 rack design used the [ggao50 gripper as a starting point](https://github.com/ggao50/SO101-Parallel-Gripper/tree/b0614b72c62e3891b1f7faa8b06c6eb3d490a9c2). The [original Onshape assembly](https://cad.onshape.com/documents/d1149295f86aada4af179739/w/6cb56f8acd609f12b3c3bfe1/e/1fb88de2f9833f29914e7cb3) shows the baseline rack-and-pinion assembly. The R4 film illustrates our modified sensor mounting and rectangular cable window. This public print pack contains our camera mounts and accessory parts.

## Verification scope

STL manifold checks, fastener-axis clearance, and a CAD intersection check at 51 sampled poses are recorded for the 26 cm mount. Both views contain eight modeled work-volume corners at the nominal 70° horizontal field of view. At 60°, each contains seven. Actual field of view, fit, stiffness and cable routing require a bench check.
