# Sensor-driven framing correction — 2026-10-06

The live `run_v2_low_views` retained real clean images: two for 7320 and one
for 45746 at the time of this diagnosis. They are not complete clean states.
Rejected cropped images and all failed mobile candidates remain preserved.

The initial SAM3 RGB-D cloud already contains a useful object extent, but its
visible-surface median is a sampling statistic, not the framing center:

| Asset | Old median (m) | Observed bounds midpoint (m) |
| --- | --- | --- |
| 7320 | (0.601, -0.176, 0.250) | (0.511, -0.061, 0.173) |
| 45746 | (0.857, -0.121, 0.421) | (0.674, 0.119, 0.303) |

The previous proposal search accepted 97% projected points. Dense surfaces
therefore hid missing sparse edges, while image QA correctly rejected crops.
New proposals use the sensor bounds midpoint and frame all eight bounds
corners. Remaining finite proposals are refreshed after actual current-state
SAM3 RGB-D observations. The initial/current sensor envelope is for camera
planning only; backend point clouds remain actual, separate per-state inputs.
No simulator mesh, articulation axis, state, or QA mask enters this envelope.

Five regression tests pass, including nonuniform sampling, camera/TCP chain,
rectangular ArtGS rays, explicit entry imports, and frozen physical hashes.
Offline 20-base/view diagnostics with the same stored inputs found 3 valid
goals each at 6°/12° for both assets. This is goal feasibility, not proof of
complete paths or actual scanning. Actual execution remains required.

`wrist_reconstruction_v2_sensor_bounds.json` reserves seven cumulative capture
hours and up to nine total hours, capped by the existing 11:00 Shanghai
deadline. It retains the original 02:38:34 start and subtracts all earlier
capture time. Each state still has at most 48 proposals; its finite wall
allocation is 60 minutes. These are task budgets, not physical safety limits.
Previously captured data are retained rather than relabelled as new success.

Grasp/contact/proxy/controller/force/friction/joint safety files are unchanged.
The correction does not establish multistate capture or reconstruction success.
