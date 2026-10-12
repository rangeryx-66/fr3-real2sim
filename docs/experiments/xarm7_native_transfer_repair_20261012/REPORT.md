# Native xArm7 G1 transfer repair — bounded round

Reference `fe639b6`; execution source frozen at `efaa495` before physical outcomes. Started 2026-10-12, completed 2026-10-13. **ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT / XARM7_NATIVE_GRIPPER_APPROXIMATE_ACTUATOR_SIMULATION**.

**Both registered physical attempts reached 30° through native contact.** Exactly one normal closed-start episode each, sequential actors, fixed cabinet/base, zero intentional releases or regrasps, no manual continuation, restart, stitching, object actuation or attachment. No tuning between cases. 45134 was evaluated offline only; its previous physical result is preserved.

| Case / registered handle | Original result | This round | Actual after valid grasp | Result |
|---|---|---|---:|---|
| 45134 / original-32 | Native G1 30.000061° | Offline regression; old plan remains feasible, automatic new plan covers 30° | No new physical run | Offline compatible |
| 45443 / original-38 | RTX initialization failure, no physical result | Correct-handle bilateral grasp | 30.000179° | PASS |
| 45249 / original-32 | 0.053795°, sustained-force stop | Correct-handle bilateral grasp | 30.000196° | PASS |

This is evidence of transfer across two handle geometries, not a robust success-rate estimate. There is no new physical 45134 regression claim.

## Diagnosis and the one shared manipulation change

The archived 45443 crash preceded robot/object creation: RTX allocator, SceneDb and LightCache failures, exit 139, plus an inotify warning. Its GPU-2 launch overlapped another actor. Resource contention is a credible contributor; the exact driver cause was not isolated. Exclusive actor ownership, sequential execution on inventory-checked GPU 4, executable hashes and camera/encoder preflight repaired orchestration without changing dynamics or behavior. Both new actors exited normally with code 0; no retries or forced cleanup were needed.

Archived 45249 acquired bilateral contact, then left-pad unloading and right-pad reaction grew while the opening reference advanced but the door barely moved. Final overload-window averages were 2.0205, 2.0148 and 2.0217 N. There was no evidence of an effort increase, an active object joint drive, or a loaded non-handle collision stop. Sampled original cooked-solid checks did not identify an environment collision. Actual hinge resistance was not directly measured.

The shared candidate utility now weights existing common-aperture bilateral support/opposition by the **native pad-normal component along the opening tangent**. It replaces an unverified tangential/friction-like utility; all candidate families, original collision/approach/IK checks and control logic remain unchanged. Existing ±15° candidates supply the selected pitch automatically. No asset-ID pose/controller branches, new runtime veto, friction change or force increase. See [diagnosis](DIAGNOSIS.md) and [shared selection adapter](../../../scripts/xarm7_native_transfer_contact.py).

45249's measured first-probe normal projection improved from approximately 0.0043 to **0.2747**, matching the new plan's 0.2736. At 2.9 s after acquisition, actual/reference motion changed from 0.0490°/0.2007° to **0.2015°/0.2510°**; L/R reaction changed from 0/2.010 N to **0.116/0.885 N**. Contacts remain on the native inner pad plane, near a lateral edge; weighted local y shifted from approximately 31–32 mm to 40–43 mm. This supports improved transmission, but selection also changes station/pose, so it is not an isolated causal ablation.

PhysX point impulses are **normal-only**; unrecorded friction anchors cannot be assumed zero. Reported pad reactions are not complete interaction forces or hardware sensor measurements. [PhysX source](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/include/PxSimulationEventCallback.h).

## Complete physical metrics

| Metric | 45443 | 45249 |
|---|---:|---:|
| Acquisition simulation time | 125.767 s | 132.129 s |
| Acquisition L/R pad load | 0.507 / 0.509 N | 0.475 / 0.532 N |
| Peak raw pad load | 1.133 N | 0.979 N |
| Maximum translation / rotation drift | 3.036 mm / 0.510° | 1.084 mm / 0.402° |
| Maximum loaded measured TCP speed | 3.214 mm/s | 1.450 mm/s |
| Maximum all-phase measured TCP speed | 21.226 mm/s | 23.747 mm/s |
| Minimum joint margin | 0.441953 rad | 0.942846 rad |
| Maximum arm tracking error | 0.004917 rad | 0.004584 rad |
| Both filtered pads unloaded intervals | None | None |
| Force-warning/overload transitions | None | None |
| Local correction / reopening commands | 0 / 0 | 0 / 0 |
| Simulation duration | 515.683 s | 435.383 s |
| Wall duration (runner, including startup/export) | 3127.270 s | 2477.666 s |
| 240 Hz observations / physics steps | 123764 / 123764 | 104492 / 104492 |
| Continuous video | 515.667 s, 15470 frames | 435.367 s, 13061 frames |

The 5 mm/s protection applies to loaded motion; the all-phase maxima occur before grasp. No measured spikes are filtered away. Original 0.05 rad joint margin, collision/contact/temporal-force protections, actuator limits, gains, preload, object physics and source assets are retained. Single-pad unloading was tolerated by the unchanged controller; absence of both-pad unloading is a diagnostic summary, not proof of zero microscopic slip. First physical divergence: none recorded in either new episode.

## Evidence, provenance and limitations

Private lab evidence root: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_native_transfer_repair_20261012`. Each `45443/physical_01` and `45249/physical_01` contains continuous `contact_baseline.mp4`, lossless `observations_chunks`, native `physics_steps_chunks`, raw collider/contact points/normals/impulses, force-window history, joint/TCP/reference state, sensor metadata, generated executable and actual source provenance. Raw streams contain every step; export markers and record counts agree. Local copy: `engineering_artifacts_20261012/xarm7_native_transfer_repair/`. Evidence and videos are **not uploaded to public GitHub or Releases**.

Loaded runner: `scripts/run_xarm7_native_transfer_repair.py`, SHA256 `c08fb22edcf6cffdddef32c3ddaf37a8123f6f7f59025d5e785e8305b97065e9`. Native configuration SHA256 `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`. Full compact source/plan hashes are in [PROVENANCE.json](PROVENANCE.json). Original source files and frozen baselines remain unchanged. The shared runner delegates to the frozen native execution code.

Official G1 geometry with approximate finite-effort simulation actuators and virtual mounting remains the reference combination. GT handle geometry, known hinge/collision model and dynamic object-state feedback remain execution dependencies. This does not establish visual autonomy, full force identification or hardware performance. The bounded round is finished; no further physical attempts were launched.
