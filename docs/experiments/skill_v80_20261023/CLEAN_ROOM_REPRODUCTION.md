# Fresh-context V8 contact reproduction smokes

A clean-context subagent discovered the Skill through `AGENTS.md`, read its runbook/API/schema, audited the package, and ran exactly two sequential fresh contact scenes. Both completed one physical acquisition, actual pull/reverse motion, two separately predeclared validation actions, guarded release and measured parking. These are registered-station interface smokes, separate from earlier benchmark statistics; they are not a separate Codex installation, cross-machine test, blind benchmark or hardware result.

## Source and resource commitments

| Item | Cabinet | Drawer |
|---|---|---|
| Actual actor checkout commit | `22510aa31d44984d57b59bb62a1c22f441e1257c` | `e453c7f655ecd6fa0e16a2af27f728984a1dcac7` |
| Checkout | `/data1/home/rangeryx/agentic_v8_checkout` | `/data1/home/rangeryx/agentic_v8_drawer_checkout` |
| Runtime | `RuntimeV7` | `PrismaticRuntimeV751` |
| Offline package tests | 6 passed | 6 passed |
| Prelaunch audit | 606 current physical files; frozen 71 + 22 external; no failures | 955 release files, including 606 physical; frozen 71 + 22 external; no failures |
| Job SHA256 | `3a4713ba1cdf0777b984f4431adb92c6fe053b155568b898cfd7890bb6146111` | `396af397d7928bc892f1738c12b0f106681f4b6ce0cbe08882582a0248feeae6` |
| Plant-file commitment only | `ef92278ff59b4df57dbffd8e784e5eab56c51637f97cbf44830d920a023d523e` | `24be80a6afa509669defaa97da17b1d0640fe3a15e732546998f449605fe6e43` |

Shared bundle manifest SHA256: `70127bd0e2f5868aa72ad8aa16e81d95ed46f5b621741f43a3f28b18496ae042`. Environment-profile SHA256: `4fa5affd8060669ed95aa1e4265f6853c82431341bf417faba04598a60275d1e`. Actual interpreter reports Python 3.12.14. Cabinet spec SHA256: `666f5ed9db6a294ec4b989d9145427b1d52bfa9c86668aeec26d9309697e7b7b`; drawer spec: `05719987d51b9627eb6153d6e20fdaf19a19f35fa8376e36370388c4fff78894`.

Both used the packaged launch/atomic command interface, 2400 s scene wall budget, 1400 s simulation budget, 360 s unloaded decision window, 300 s reserved wall exit budget and acquisition ceiling 2. No implementation, physical/controller coefficients, drive authority, protection thresholds, manual poses or object joint commands were changed by the Agent. Drawer retained the initial 1.0 N requested preload and `transverse_unloading=false` throughout. Requested preload is not measured grip force.

Plant contents, hidden/native truth files, scene-generator contents, passive camera data, old images, old grasps and historical source snapshots were not online decision inputs. Source-audit hash metadata is distinct from behavioral guidance.

## Fresh observations and recovery

The Agent actually inspected copied current wrist RGB/SAM/depth/local-cloud boards, derived geometry boards, selected-plan boards and contact feedback with `view_image`.

- Cabinet: initial wrist hardware occluded the upper vertical bar, with no complete SAM cue. A raised leftward view exposed the full bar; a second oblique view added a flank. The automatic middle-bar plan was inspected and current return/approach validated. Two Agent-selected adjustments, one generation, one acquisition; no contact recovery required.
- Drawer: initial SAM selected an upper neighboring bar. After one raised complementary view, all 16 automatic native branches collided with a nonselected body at q=0, before acquisition. This failure was preserved. The Agent invalidated that geometry and reobserved. The new lower-bar SAM included its cast shadow; frozen read-only depth-protrusion analysis on the current capture isolated the actual lower bar. Explicit depth proposal 0 produced a feasible automatic middle-bar plan, then current return/approach passed. One Agent-selected adjustment, two observation generations, one acquisition. No collision veto was waived and no episode was restarted.

Read-only plan renderer source SHA256: `15fe3d0966060413bc371078a5e1e0a16cedc95a1aa262d6e9ba65d3dd93c395`. Cabinet was first rendered using the explicit episode robot-IO locator; the newly packaged helper subsequently produced the identical board hash. Drawer used `python -m agentic_interaction_v8.inspect_plan --episode ...`.

Selected-plan image SHA256: cabinet `451bd0441247c64b36681c09c661f5a8d18af7b0453cb7f983643277c2d8f605`; drawer `539717b183c391a2de6c29ad331b1b2669d6aad0f2a2968d7e494656ebeedc40`. Private image traces retain every inspected board's full SHA256, interpretation, uncertainty, selected action and result. Unseen rear geometry stayed uncertain.

## Measured contact results

Both acquisitions verified a one-second hold. All eight listed segments returned `TARGET_REACHED`, valid physical transmission and safely stopped state, without target truncation. Values below are actual endpoints, not commanded travel.

| Scene / role | Requested endpoint | Measured endpoint | Measured change | Full action duration |
|---|---:|---:|---:|---:|
| Cabinet FIT pull | 0.100 rad | 0.100166 rad | +0.099931 rad | 12.5167 s |
| Cabinet FIT reverse | 0.025 rad | 0.024580 rad | −0.075586 rad | 6.8875 s |
| Cabinet validation pull | 0.065 rad | 0.065297 rad | +0.040717 rad | 4.6167 s |
| Cabinet validation reverse | 0.030 rad | 0.029651 rad | −0.035646 rad | 4.3292 s |
| Drawer FIT pull | 25 mm | 25.0673 mm | +25.0674 mm | 10.1667 s |
| Drawer FIT reverse | 6 mm | 5.8813 mm | −19.1860 mm | 7.4375 s |
| Drawer validation pull | 16 mm | 16.0883 mm | +10.2070 mm | 5.3417 s |
| Drawer validation reverse | 8 mm | 7.9211 mm | −8.1672 mm | 4.9375 s |

Cabinet reported no panel contacts. Contact boards showed asymmetric pad-normal reports during motion; absence of positive normal on one pad was not interpreted as proven separation. Drawer current-attempt native point diagnostics were inspected and preserved; point samples are not pressure areas or full contact wrenches. Only frozen deterministic owners handled loaded execution and real-time protection.

## Identification and validation limits

Validation trajectories were declared before execution, had no fitting intervals, completed physically and were not used for fitting or model selection. Predictions use held-out compensated wrist effort and one initial measured state without subsequent state correction. They are derivative-supported, processed interior windows, not predictions over the full action durations.

| Scene | FIT support | Selected model / limitation | Validation prediction horizons | Coordinate RMSE |
|---|---|---|---|---|
| Cabinet | 1055 pull + 335 reverse eligible samples; q support [0.002532,0.041460] and [0.077370,0.094001] rad | Constant model. Damping and Coulomb term conditionally supported; effective inertia unsupported and remains unknown. | 3.6042 / 3.3167 s | 0.002097 / 0.002186 rad |
| Drawer | 838 pull eligible samples. Reverse action complete, but its declared interval produced only 240 records: `insufficient local filter support`. q support [1.136,11.511] mm. | Constant model has conditional mass/damping/Coulomb support from short one-direction FIT only. No complete bidirectional estimator FIT, trajectory-CV or position-dependent resistance claim. | 2.8250 / 1.9208 s | 0.0917 / 0.0256 mm |

Both prediction outputs flag some evaluation outside FIT support. Conditional uncertainty excludes compensation, contact, model-selection and other systematic errors. These short strokes do not establish full-range dynamics, precise friction or hardware accuracy. Drawer reverse-FIT insufficiency is retained, not repaired by relabeling validation data or repeating a valid episode.

## Parking, export and cleanup

Both final results are `SESSION_COMPLETE`, process return code 0 and `actor_exited=true`, with guarded safe exit, measured native-rest parking, unloaded pad reports [0,0] N and valid measured collision checks. Maximum joint-cell errors were 0.003715 rad (cabinet) and 0.003717 rad (drawer), within the frozen 0.025 rad parking cell.

| Scene | Actor wall time | Final recorded storage summary | Post-summary on-disk bytes | Owned launcher / actor |
|---|---:|---:|---:|---|
| Cabinet | 1096.065 s | 44,296,283 B | 44,620,009 B | 1613615 / 1613619 |
| Drawer | 1458.667 s | 45,132,608 B | 45,394,540 B | 1620277 / 1620301 |

Post-summary disk size includes sealed derived metadata. Exact actor/launcher PIDs were absent after graceful shutdown; no process referenced the drawer episode, and neither owned actor remained in GPU compute-process listings. No unrelated jobs were terminated. Raw compressed essential telemetry and sparse fresh RGB-D remain private on the host; no continuous video was requested or generated by these package defaults.

Packaging-audit gap: both original actors lack `V8_LOADED_SOURCES.json` because Isaac shutdown skipped the atexit writer. Their source/job/prelaunch/core provenance and physical results remain preserved, but these actors do **not** establish a successful final imported-module ledger. No ledger was reconstructed. A later package finalization fix/component test is separate from these two original smokes.

Private local evidence: `/Users/rangeryx/Documents/ChatGPT/robot in loop real2sim/private_experiments/skill_v80_cleanroom/{cabinet,drawer}`. Remote episode roots: `/data1/home/rangeryx/agentic_v8_private/episodes/v80_{cabinet_bar,drawer_bar}_00`. Drawer first-plan failure audits remain in `CLEAN_ROOM_FAILED_PLAN_00`. No raw frames or telemetry were added to Git.
