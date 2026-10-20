# Explicit headless sensor illumination repair

The user paused V7.2 after observing near-black wrist RGB. Two render-only,
same-pose A/B tests confirm inadequate front/ambient illumination as the main
cause. This is a sensor-input configuration change, not an unchanged-input
benchmark continuation or a manipulation result.

## Diagnosis and change

The frozen scene builder authors no explicit ambient or key light. Actual USD
inspection found one inherited `/World/defaultGroundPlane/SphereLight`
(intensity 100000), no dome fill and no deliberate front key. RTX used
`RealTimePathTracing`, tonemap operator 6, histogram/auto exposure disabled.
It is therefore inaccurate to describe the scene as having literally no light.
The existing illumination left the cabinet front almost black.

`configs/sensor_lighting_v1.json` adds a white DomeLight at intensity 1000 and
a white DistantLight at intensity 1500, rotated XYZ [45,0,0] degrees. Exposure
remains zero. These are renderer intensity values, not claimed physical lux.
No camera exposure, material, texture, mesh, collider or physics changes are
made. No image brightness postprocessing is used.

The reusable installer is `interaction_runtime/sensor_lighting_v1.py`.
`LitRegisteredPrismaticRuntimeV72` and
`scripts/run_registered_prismatic_lighting_v72.py` explicitly install it before
Agent observations while inheriting V7.1 manipulation unchanged. The original
V7.1 runtime/entry and all frozen results remain intact. The new entry has not
been used for a physical benchmark attempt during this repair.

## Actual rendering checks

|Asset|Mean gray before → after|Pixels gray <16|Pixels gray ≥250 after|Depth max difference|Safe park|
|---|---:|---:|---:|---:|---|
|19179|27.24 → 117.64|65.89% → 5.36%|0%|0 m|yes|
|45290|54.49 → 214.15|33.32% → 0.58%|0%|0 m|yes|

Both tests used the same actual wrist pose, material, intrinsics and extrinsics
for each before/after pair. Joint arrays were bit-identical and object native
coordinates unchanged; finite-depth masks and all finite depth values matched.
Authored physics/mesh attribute digests matched before and after. On 45290,
reinstalling lights did not duplicate them. The bright cabinet remains bright;
image-wide statistics are not calibrated exposure or perception-success gates.

The diagnostic uses the normal robot model and validated actuator-executed
initial survey route. It bypasses randomized object authoring in this isolated
render-only scene, reads a local empty diagnostic configuration instead of the
benchmark secret, and performs no grasp/opening or dynamics estimation.
Wall times including initialization and parking: 110.91 s and 115.45 s.

One offline run of the unchanged SAM3 pipeline on the saved relit 19179 frame
produced two masks passing the original border filter (scores 0.671875 and
0.640625, threshold still 0.25); another clipped mask remained rejected. No
proposal recovery was needed. This is detection evidence only, not verification
of registered moving-part association, grasp feasibility or physical traction.

## Benchmark state and integrity

- Eight revolute physical results remain unchanged. Local 71 frozen file
  hashes, remote registered 81 source hashes, and 22 external dependencies
  passed checks with zero drift.
- The two completed 19179 slots retain their original visual-failure records,
  each with zero physical acquisitions. Mark them as **lighting-affected visual
  failures**; they do not establish a drawer-traction failure. No retrospective
  success, deletion or automatic rerun is authorized by this repair.
- 45290_0 was paused on user request before any grasp. Its original actor
  captured one fresh initial observation, parked safely and exited (138.15 s).
  Preserve this as an orchestration interruption, not a consumed physical
  attempt or a force-transmission failure.
- Benchmark execution remains paused during this repair. Hidden dynamics and
  seeds remain unread. Relit future inputs must be explicitly versioned and
  distinguished from the earlier dark observations.
- Both render-only actors exited, and no owned Isaac actor remains.

`METRICS.json` contains compact derived statistics and private report hashes.
Raw RGB-D remain on `test-X640-G50` under
`/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/sensor_lighting_v1_20261020/`.
No raw frames, depth, video or telemetry are published.

NVIDIA's [headless rendering documentation](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/headless_example.html)
explains the need to author lighting explicitly for headless scenes. The
root-cause conclusion here additionally relies on the actual installed scene
audit and same-pose rendered A/B results.
