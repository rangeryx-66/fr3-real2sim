# Additional lifecycle infrastructure failure (preserved)

`v81_integrated_revolute_01`, execution source86d74ce, acquired twice in one
scene. First pull stopped at10.829 degrees with transmission degradation. Fresh
reobservation and a different automatic family then physically executed reverse
4.994 -> open9.063 -> reverse4.981 -> original independently declared validation
10.046 degrees, followed by verified unloaded native-rest parking. This is a
recovery/validation result, not completion of the90-degree task.

After another actual wrist observation, station evaluation correctly refused
`STATION_SETTLE_REQUIRES_MEASURED_PARK_CELL`. The subsequent release/park operation
hit `PHYSICS_CLOCK_MISMATCH` on its first step. Native contact-window dt was zero;
all prior registered interaction samples remain intact. The actor terminated;
no hard stop was reset and no post-stop motion was attempted.

Source diagnosis: frozen `native_g1_hybrid_contact.initial_observation` pauses the
world after unloaded sensing. Frozen `interaction_runtime.probing_v2.serve`
explicitly calls `world.play()` before release/finish. V8.1's separate session
omitted that precondition. The exit owner directly steps jaw release before its
later hold, so hold's play was too late. The V8.1-only release adapter now resumes
physics after checking the existing hard latch, before delegating to the same
exit owner. No synthetic contact dt, skipped clock guard, latch reset or changed
physics/controller is introduced. A unit regression reproduces dt=0 with a paused
world and checks resume ordering plus authoritative latch refusal. Actual native
validation remains to be established in the next separately numbered engineering
scene. Do not call this software test a physical exit PASS.

Also preserve SI `coordinate_unit=m` when accepting an unexecuted prismatic
candidate from the current pool, after the original native whole-chain check.
This retains its reverse-crossing-start certificate; it changes neither scores
nor collision checks. All150 V8.1 contract tests pass; frozen606 physical/71
revolute/956 release sources remain unchanged.
