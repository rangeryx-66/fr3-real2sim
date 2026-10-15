# 45671: separate collision-representation issue

No new physical episode or executable grasp search is run on 45671 in this
round. This review uses the archived evaluator from `3103651` and
`8425f13`; its GT/cooked geometry does not enter the new visual input adapter.

Preserving visible ends repaired most longitudinal truncation: single-view
span 227.04 → 325.63 mm versus 334.69 mm GT, and original-visible coverage
53.8% → 94.6%. Multi-view visible coverage rises to 96.4%, but fused point
P95 error remains 2.458 mm, including residual end/trim contamination.

There is also an independent simulator contact discrepancy. The native convex
handle/support proxy fills rear space absent from the visual mesh; archived
native→visual surface-distance P95 is 31.54 mm. At the same archived grasp:

| Surface/measurement | Left first-contact aperture |
|---|---:|
| GT visual mesh, coarse offline sweep | 13.99 mm |
| Fused measured visual patches, coarse sweep | 11.94 mm |
| Native collider plus cooked finger, coarse sweep | 24.29 mm |
| Archived native physics | 24.66 mm |

This is not evidence to thicken RGB-D reconstruction until it reproduces
invisible convex fill-in. The visual gap remains visual evidence; unknown
surfaces remain uncertain. Neither the collider nor object physics is changed.

Recommended separately versioned physics-fidelity comparison: inspect native
cooking against the original handle/support shape and compare the existing
single convex representation with an explicitly documented decomposition that
preserves its gap. Keep the controller, mass/inertia, friction and original
benchmark fixed; publish separate representation hashes and outcomes. That
comparison is not executed or folded into this two-case experiment.

Sources: [reference geometry study](../native_g1_contact_geometry_study_20261015/REPORT.md),
[GT/observed diagnostic](../native_g1_gt_observed_diagnostic_20261015/REPORT.md).
