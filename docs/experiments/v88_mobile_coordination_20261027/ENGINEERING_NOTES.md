# Qualification failure and shared corrections

The first initialized 45290 qualification **did not pass**. It established
bilateral contact and reached 2.840 mm, without completing its short forward,
reverse or validation chain. Base travel was largely opposed by arm motion.
The legacy forecast ended the action; a failed stationary dwell then produced
an unconfirmed-control software latch. No raw measured hard-safety event was
recorded. Parking was not confirmed. Preserve this NOT PASS and its latch;
the instance was exported and closed, never resumed. This is not a formal
four-hour case or evidence of a physical travel limit.

Source audit identified two command-semantics errors in the new adapter:

* Native measured-state IK returns a next seed position, which becomes a
  position command after retained bias. Its corrective seed twist cannot be
  clipped as though it were the already integrated command velocity. Convert
  the measured/nominal seed, retained bias and both XY/arm proposals into one
  command coordinate vector before final rate bounds.
* Acceleration-normalized projection can introduce opposing XY/arm motion,
  because the coordinates have different units and slew scales. Preserve the
  jointly planned direction along an independently verified feasible segment.
  Retain the original nonlinear optimizer when no feasible anchor exists, and
  independently verify every result against the same declared limits.

| Restriction | Previous authority | Corrected authority |
|---|---|---|
| Native collision, actual runaway/invalid state, hard joint/speed/load protection | Hard stop | Unchanged hard stop; no latch clearing |
| Final TCP and joint command speed/acceleration | Nonlinear command bounds | Same bounds applied to the simultaneous nine-variable command |
| Raw linear load forecast into a provisional sustained band | Ends action and requires unload | Continuous reference regulation plus recording; measured emergency owner unchanged |
| Finite stationary dwell not yet satisfied, with finite trusted state and stopped commands | Unconfirmed-control terminal result | Guarded real-physics C, protective hold only; B denied until verified |
| Repeated C expiry while protective hold remains necessary | Could fall into a paused loaded wait | Further bounded real-physics C windows within the original episode resources |
| Work budget ending before absolute deadline | Decision renewal could consume exit reserve | Existing protected exit owner starts inside the reserved time |

The load forecast retains its original causal samples and bands and now
includes the base braking horizon. It does not certify future safety. No
force/torque threshold, native gain, actuator effort, friction, collision
waiver or estimator coefficient is increased. The original physical source
audit remains clean: 606 physical, 71 revolute and 956 release source files.

Offline verification passed **131 tests**, including the recorded 0.600200 N m
band crossing, the prior excessive TCP-speed proposal, real emergency
force/torque/speed/collision stops, coupled braking, held-contact dwell failure,
continued World ownership, denied B continuation while stability is pending,
measured-seed/command conversion, FIT separation and protected exit resources.

An additional **kinematic-only** comparison used the same native robot model
and 100 deliberately excessive pure-XY proposals. The previous projection took
18.953 s and introduced up to 0.01248 rad/s of unrequested arm motion. The
direction-preserving path took 0.492 s and introduced zero arm motion. Both
retained independent verification of the original declared bounds. This is
neither a collision certificate nor a physical manipulation result.

A fresh bounded physical qualification is required before the formal 45290
case. The subsequent 45600 case remains fixed and requires its own independent
admission. Full raw telemetry, video and internal registration stay private;
only summaries and source are published.
