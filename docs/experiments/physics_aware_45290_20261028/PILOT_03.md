# Moderate-profile qualification: motion passed, numerical stop blocked exit

Source 9301355. Actual forward maximum 20.8054 mm; reverse reached 2.9985 mm
(17.8067 mm reverse). The third, sinusoidal FIT stopped at 11.5685 mm with
`TCP_PROJECTION_INFEASIBLE`; complete independent VALIDATION was not executed.
There were ZERO recorded native force/speed/collision hard-stop samples.

The subsequent controlled stop/release could not be confirmed. The lifecycle
owner retained a software control-state latch with an explicit statement that
this was not a measured force/speed hard event. It exposed no validated reset.
The World was retained for diagnosis, then gracefully finalized: NOT PASS,
release/retreat/parking unconfirmed, latch retained, all video/raw points/F/T
sealed, owned PID/GPU release verified. The terminal label HARD_STOP_UNPARKED
does not turn this software failure into evidence of a physical limit.

Offline replay uses the recorded commands, native robot kinematics and the
original speed-program timestamp. At t=111.195833 s the declared speed was
0.002742044272 m/s and acceleration 0.008 m/s². The old governor rejects the
FIRST braking step; the repaired governor reaches zero in 83 steps (0.346 s).
Maximum reconstructed command speed is 0.002708708232 m/s and acceleration
0.007999992002 m/s², both inside the SAME declared bounds. Angular and joint
slew checks also remain active. Arm telemetry is float32: this is a very close
reconstruction, not a bit-exact live-controller state or a physical exit proof.

The repair searches a single common braking fraction before the existing
multivariable optimizer when its maximal joint-slew brake is outside the TCP
slew envelope. It does not change limits, position targets by fiat, actuator
authority, contact physics or hard reset rules. A numerical failure in a
hypothetical stopping preview is separately labelled as a preview rejection.
Actual unsafe/inconsistent states are still rejected.

Validation: 21 coordinated/stopping tests and all 101 V8.7 tests pass. The
closed-episode replay is reproducible with
`python -m agentic_interaction_v88.replay_stopping EPISODE --output REPORT`.
The original failure/latch remains unchanged; a NEW bounded physical
qualification must still prove actual safe stop, validation and exit before
any formal A/B/C comparison. Physics-Aware operational benefit is unproven.
