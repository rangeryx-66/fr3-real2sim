# Independent scan-station skill

This skill starts after the frozen grasp executor has placed and released an
object. It does not import or modify the grasp selection/execution code.

## State machine

`placed on clean station → release → park FR3/hand outside view → two-ring
RGB-D capture → frame QA/crops → RGB-D object-frame estimate → ReconViaGen
v0.5 → metric constraint → real-RGB texture bake → CoACD → Isaac USD reload`

The default station captures 24 views: 12 horizontal/medium-tilt views and 12
higher-tilt views at 30° azimuth spacing. The target is placed at the station
center and the camera radius/intrinsics are chosen so the object bbox is
between 40% and 70% of image height. A 15% object-bbox crop is saved for the
ReconViaGen input.

Every frame records RGB, uint16 depth in millimetres, object and gripper masks,
camera intrinsics, `T_base_camera`/`T_B_camera`, `T_B_TCP`, and the scan pose.
QA rejects empty masks, invalid depth, bad framing/exposure, clipping, or any
image-wide/bbox gripper occlusion above the configured limits. The object
frame is estimated only from robust RGB-D mask points transformed by the
recorded robot-camera pose. Simulator target pose and GT mesh are stored under
`eval_gt/` for final evaluation only.

## Run

On the Arena host:

```bash
cd /data1/home/rangeryx/fr3_moveit_grasp
/data1/home/rangeryx/isaaclab-arena/.venv/bin/python -u \
  calibration/run_scan_station.py \
  --target soup --seed 1030 \
  --output results/scan_station_soup \
  --gpu 3 --port 18930 --views-per-ring 12 --radius-m .30 \
  --crop-padding .15 \
  --inertial results/real2sim_soup_v1_final/inertial_object_simulation.json
```

Set `--skip-reconstruction` to inspect only capture/QA. The complete run writes
`scan_station_run.json` and `SCAN_STATION_REPORT.md`; products are under
`reconviagen_v05/{generated,metric,textured,collision,asset}`.

The staging transform (`--robot-stage-x/--robot-stage-y`) is a station-scene
parking placement performed after release. It is separate from and does not
change the frozen grasp executor.

The default downstream asset uses the measured RGB texture bake. To audit the
official ReconViaGen v0.5 generated texture (`tex_slat`) without changing the
normal product, run the same pipeline with
`RECONVIAGEN_TEXTURE_MODE=native`. This writes a separate
`native_texture_compare/` product, transfers its UVs onto the RGB-D-constrained
mesh, and records the native texture source and coordinate conversion. The
native texture remains a comparison because generated labels/details can be
hallucinated; the RGB-D metric mesh and CoACD collision stay authoritative.
