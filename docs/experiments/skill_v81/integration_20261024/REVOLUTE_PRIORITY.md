# Priority change: revolute 90-degree lifecycle first

2026-10-24: User stopped drawer range/preload engineering. Preserve every result; do not resume drawer range work or independent repetitions until a same-scene revolute episode completes actual 90 degrees, meaningful reversal, qualified bidirectional FIT, complete independent VALIDATION and measured safe parking.

## Drawer shutdown

`v81_integrated_prismatic_02`, loaded source `aeb9636153c51a3644d35a002d037c0f3244199b`: one physical acquisition, actual maximum drawer displacement 2.3553 mm. First pull returned TRANSMISSION_FAILURE, no qualified FIT or complete VALIDATION. Actual unloaded base relocation and fresh same-scene recovery observations/planning were preserved; no second acquisition was authorized. On user request, finish command 15 executed original guarded release/retreat/parking: measured native-rest joint-cell error 0.003722 rad, native measured-state collision check valid, pad loads 0/0 N. Actor exited rc=0 after 2597.275 wall seconds. This is safe termination, not manipulation success.

## Clock failure and unchanged safety

Frozen wrist observation pauses the world at an unloaded boundary. The old session played the world before calling guarded release. The V8.1 session omitted that precondition; the release owner's first direct step preceded hold(), giving native contact-window dt=0 and PHYSICS_CLOCK_MISMATCH. Isolated `physics_lifecycle.resume_for_exit` restores play before delegation and rejects an existing hard latch. Physics rate, native dt measurement, clock guard, collisions and joint protections are unchanged. Unit regression reproduces paused dt=0 and verifies resumed 1/240 s and authoritative hard latch.

## Short native revolute parking protocol (before range integration)

New engineering-only unloaded scene, same original robot, cabinet, native collision representation, physical configuration and safety owners. No grasp or object-driving actuation:

1. Inspect fresh actual wrist RGB-D; request guarded release/parking from the paused observation boundary.
2. Request a physically collision-invalid station route using native whole-robot/support collision checks. Do not execute rejected route. Verify a subsequent guarded park remains available and physically completes.
3. Select and validate a nearby clear station from the fresh visual context, actuate the native base stage, reobserve with the wrist, then physically park from the new station.
4. Finish/export, inspect native clock samples, measured state, zero contact and process cleanup.

Planning rejection is recoverable; a hard latch is never reset. A purely injected rejection or endpoint IK is not physical collision/path evidence. This smoke is distinct from the one subsequent full revolute integration episode.

## Range integration admission

Before grasp: fresh observed geometry and complete current-state approach/closing, timed loaded route, native collisions/joint margins and exit checks. Report nominal arc coverage separately from execution certificate; partial coverage cannot certify 90 degrees. Try cheaper same-station branches/candidates first. Keep the global 90-degree goal through intermediate actions. If needed, guarded release, Agent-selected collision-checked base movement, fresh observation and new grasp at the actual door state. Preserve pending VALIDATION and seal the FIT-only online model before validation. Keep finite resources with ten-minute exit reserve; no loaded Agent reasoning.

All original V8.0/frozen revolute source bytes and private evidence remain unchanged. Hidden physics is not an online planning or estimator input. No new drawer tests or four independent repetitions are authorized before this gate passes.

## Unloaded native result

`v81_revolute_parking_smoke_01`, behavioral source `aeb9636`, completed in 590.333 wall seconds, no acquisition and no object-driving action. Normal park: PASS, 41.664 s. Real station-route rejection: `DANGEROUS_GEOMETRY_COLLISION:link_base:/World/table` on all three alternatives, no rejected motion executed; following park PASS. Actual driven base movement from (0.19605,-0.53282) to (0.12,-0.62), fresh wrist observation, subsequent park: PASS, 44.193 s. Final native-rest joint-cell error 0.003722 rad, loads 0/0 N, measured native collision check valid, door 0 degrees. SESSION_COMPLETE, rc=0, owned actor lock released. Loaded clock repair SHA256 `da58485bc359ab79c618a2d9cfd6cced8bbee7fa0e3b5072a22a380ca1692d09`.

## Isolated range certificate

The integrated V8.1 revolute candidate seam now additionally checks the original closing/CENTER envelope, all declared bidirectional loaded segments with the existing half-degree reference allowance, perpendicular-radius stopping distance, retained command bias, native finger apertures and joint margin, and released-jaw/parking feasibility at each nominal stopped endpoint. Native rejection advances to other ranked candidates; utility is unchanged. A changed action declaration reruns this certificate. These are conditional geometric predictions; measured-state refinement, route checks and physical contact protections remain authoritative. A certificate covering short targets does not certify the global 90-degree goal. No frozen V8.0/revolute file or physics setting is edited.
