# Native G1 transfer repair: archived evidence and one shared hypothesis

Reference: `fe639b6`. No original source, controller, gripper effort, friction, object dynamics, collision model or protection is changed. Raw data and video remain local/on the lab server; only code and compact reports are public.

45443's old launch overlapped 45134 on GPU 2. The log records RTX allocator/SceneDb/LightCache allocation failures before robot/object scene creation, followed by exit 139; an inotify-watch warning is also present. This is an initialization failure, not a grasp result. Concurrency is a credible resource contributor, not a fully isolated driver diagnosis. The repair uses a single exclusive owner lock, sequential child processes, GPU inventory capture, exact source provenance, pre-launch encoder/decoder check and actual camera/encoder verification before approach. Robot dynamics and manipulation timing are unchanged.

## 45249 versus successful 45134

Both old runs established bilateral contact on the registered handle and used the same native position-preload adapter. Joint drives on the object were zero; the authored joint URDFs contain no additional friction/damping, and the preserved plant report says unchanged source. No loaded non-handle collision guard fired. Offline checks using the original cooked solids show no moving-door/table/fixture collision at 0°, 0.0538°, 1° or 30°; minimum environmental separation for 45249 is about 134 mm. These sampled checks are diagnostic, not trajectory certification or direct measurement of hinge resistance.

| Archived first opening segment | 45134 | 45249 |
|---|---:|---:|
| Grasp-acquisition loads, L/R | 0.499 / 0.532 N | 0.553 / 0.472 N |
| Recorded normal projection on opening tangent | ~0.090 | ~0.0043 |
| Actual angle 2.9 s after acquisition | 0.0932° | 0.0490° |
| Reference angle at that sample | 0.2116° | 0.2007° |
| Reaction loads at that sample, L/R | 0.527 / 0.468 N | 0 / 2.010 N |
| Final actual task result | 30.000061° | 0.053795°, sustained-force stop |

Both sets of recorded contacts lie on the native inner pad plane (local z approximately −26.003 mm), near a lateral pad edge. Pad dimensions and physical geometry are unchanged. The failure's left pad unloads as the reference progresses; right reaction grows while actual motion remains small. Recorded preload requests and their finite position-target mapping do not show an asset-specific effort increase. The last three force windows average 2.0205, 2.0148 and 2.0217 N under the original policy. Neither a single zero-load sample, centering proxy nor tracking residual terminates the run.

**Measurement limitation:** the point-impulse channel is normal-only. PhysX `extractContacts()` forms impulse = normal × scalar impulse; friction anchors are separate and were not recorded by the old callback. The vectors cannot prove absence of friction force or identify full hinge resistance. This also prevents claiming that the ~2.02 N is a complete total interaction force. [Primary source](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/include/PxSimulationEventCallback.h).

## The single manipulation change

The existing candidate family, common-aperture support evaluation, approach/collision checks, base search and Pink branches are reused. Only the selection utility changes: retain bilateral support/opposition, and weight it by the actual native pad-normal component along the opening tangent. This prefers a direct normal reaction channel over the old unverified tangential/friction-like utility. Purely friction-dependent candidates are not made fatal or deleted; there is no new runtime force-quality gate.

This is an evidence-backed hypothesis: better task-normal leverage may transmit opening motion below the existing normal-load limit. It does not prove the original asset is mechanically blocked or the gripper is incompatible. No controller rewrite, manual pose offsets or object-ID branch is introduced.

| Automatic plan | Frozen pad-normal channel | New channel | New pitch | Planned range |
|---|---:|---:|---:|---:|
| 45134, offline regression only | 0.0894 | 0.3440 | −15° | 30° |
| 45443 | 0.00660 | 0.2748 | +15° | 30° |
| 45249 | 0.00459 | 0.2736 | −15° | 30° |

The existing ±15° family supplies these orientations automatically. The old successful 45134 plan remains feasible under the original constraints. The new 45249 candidate retains original collision/IK feasibility and substantially improves the predicted channel, so one closed-start physical attempt is authorized by the round protocol. This conditional offline decision is not a runtime manipulation gate. The shared implementation is frozen before either new physical outcome.
