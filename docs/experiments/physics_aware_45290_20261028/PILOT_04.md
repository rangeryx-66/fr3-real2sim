# Qualification 04: ordinary interruption, same-World recovery and validation

Execution source: `ac9b5eb284cd1de080f1c9b7f0df3c3508ed55d9`.
Read-only analysis: `eb6d6ae58c5173bde5d9ca9a86f375a8d74312fc`.
Initial plant commitment:
`02ab1eb782d8f312f366c26285a69b3703aba952a1166c075a20e040275bcebf`.
Raw evidence seal:
`995184ef966e6a3dea71ab1ff4f504ca92d28ec9a4a85b44f9a06f4390a77001`.

This bounded qualification finished with actual release, local clearance and
measured parking, `SAFE_PARK_VERIFIED`. Owned actor/GPU release was verified.
The failed sine trajectory remains a failed trajectory. This is not an A/B/C
comparison, an inherited 180 mm goal completion or a Physics-Aware benefit.

| Segment | Actual start/end mm | Outcome |
|---|---:|---|
| FIT forward | approximately 0 / 20.81940 | TARGET_REACHED |
| FIT reverse | 20.81942 / 2.99431 | TARGET_REACHED |
| FIT sine-modulated | 2.99429 / 14.94014 | recoverable numerical TCP projection interruption |
| Original pending constant VALIDATION, after recovery | 14.95052 / 7.24808 | TARGET_REACHED |

Maximum recorded coordinate was 20.81962 mm; first reverse travel was
17.82510 mm. After interruption, the original controller actually stopped,
released, retreated and parked. It retained the World at 15.19383 mm, with no
hard latch and zero raw hard-stop samples. The Agent inspected a new actual
wrist view, selected the new observed bar, generated a different automatic
grasp branch and completed the original pending constant validation. Closing
that second grasp moved the drawer slightly; validation travel is calculated
from its actual 14.95052 mm start, not the earlier unloaded coordinate.
Two physical acquisitions and one recovery are retained in the record.

Base projected shares of actual TCP net travel were 1.01827 / 0.98989 /
1.01322 / 1.01038. Shares can exceed one because arm compensation opposes a
small part of base displacement. Maximum arm-joint changes within the four
segments were 0.00148 / 0.00207 / 0.00175 / 0.00440 rad. Setup/regrasp arm
motion is excluded from those manipulation-segment figures and retained in
the full telemetry.

At requested preload 1 N, first-segment original pad-report medians were
0.95248 / 1.05370 N. Across segment records, gravity-compensated wrist-force
peaks were 0.68703 / 1.33268 / 1.20739 / 0.88063 N; corresponding sensor-origin
torque peaks were 0.09278 / 0.20380 / 0.18789 / 0.13346 Nm. Requested preload,
pad normals, tangential contact force and wrist wrench remain distinct.

The pre-validation frozen model hash was
`e20b27595240aa592d64706ed3b44e3ff23594d4946dbe38f87518a0ac28336f`;
it matches the native validation seal. Original constant-model parameters
were `[2.19240564 kg, 1.51966640 N s/m, 0.35241451 N]`. These are estimates,
not GT. The original independent open-loop predictor gave effort RMSE
0.00120886 N, coordinate RMSE 0.0155154 mm and velocity RMSE 0.0812895 mm/s
over its 1.95417 s derivative-supported interior (470 samples), without
subsequent state correction or leaving FIT support. This short prediction
does not establish full-stroke accuracy or calibrated physical confidence.

Launcher-to-process-completion wall time was 3110.24 s, native simulation
278.8125 s, recorded Agent waiting 549.22 s. Current-command and stopping-path
checks accounted for 716.55 s; these costs are not hidden as physics execution
time. Continuous H.264 video covers 277.0 s, 1385 frames, 640×480. Raw native
motion/contact/F/T, all failed actions, recovery images, model seals, curves
and separate derived-analysis hashes remain in the private study directory.

The scalar braking repair has now demonstrated a real safe recovery for this
ordinary failure. It has not eliminated numerical rejection of nonzero
modulated commands. Subsequent registered probing uses distinct constant
trajectories for every method, without raising any physical limit.
