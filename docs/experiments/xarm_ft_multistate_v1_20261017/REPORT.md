# Multi-State Active Physical Identification V1 — public brief

**Outcome: not achieved. The two registered development episodes are complete and stopped.** Initial physical opening and closing worked, but neither scene completed the first bidirectional block. This is not a validated multi-state dynamics model.

Reference `d6e690f`; frozen manipulation/recovery `136b6e0`. Shared runtime, priors, action family and validation protocol were frozen at **`c360e38` before physical outcomes**. All58 frozen source hashes still match and all44 inherited files existing at `136b6e0` remain unchanged. [Preservation verification](FINAL_BASELINE_PRESERVATION.json).

## Physical results

Object46427; independently initialized normally closed scenes; fresh Agent-inspected wrist RGB-D and real xArm7/native-G1 contact. Known hinge/state, registered station, simulated wrist mounting, ideal virtual wrist load cell and approximate actuators remain assumptions.

| Registered family | Maximum actual angle | Open / close endpoints | Reopening toward6° | Actual grasps / recovery grasps |
|---|---:|---:|---|---:|
|Constant-resistance reference|3.062285°|3.047875° /0.921954°|Stopped near0.991°|1 /0|
|State-dependent-resistance configuration|3.058041°|3.046993° /0.930422°|Stopped near1.020°|1 /0|

Both stopped through the original `CONTACT_TRANSMISSION_FAILURE_AFTER_ONE_LOCAL_CORRECTION`. Relative robot–handle motion increased without effective reopening after the second direction reversal. There was no newly introduced symmetry, inference-quality or information-coverage gate. Single-pad unloading was not automatically treated as failure. No safety limit, gain, gripping effort, friction or collision setting was changed in response to the outcomes.

Both physical attempts then completed original guarded release, clearance and parking. Only after unloading did the Agent inspect failure feedback and obtain fresh same-scene recovery observations. Each automatic alternative passed return validation but its approach was rejected by the original native collision checker; no recovery physical grasp was executed. Final scene state was an unloaded observation pose after that rejection, not a newly verified final park. This does not establish that every possible alternative is infeasible.

Simulation / runner wall times were659.21/5084.17s and667.52/5653.68s. Native240Hz recordings and continuous videos were exported. Owned actors/encoders are closed. No other user's processes were managed.

## Identification outcome

Implemented versioned multi-segment references, passive state-dependent joint resistance, segment-local derivative processing, strict whole-segment/interior holdouts and independent evaluation. Wrist-interface six-axis measurements remain the primary interaction-wrench source. Raw/compensated measurements and tool compensation remain separate; no GT object torque or contact-sum substitute was used for fitting.

Neither scene reached6°, so10/15/20° expansion was not executed. **Completed online information blocks:0; in-episode parameter updates:0.** Post-episode analysis cannot be presented as online identification. Actual acceleration excitation and separated angular coverage were insufficient; inertia/damping remain unsupported, and independent forward-motion validation is unavailable. A small conditional torque residual does not establish a predictive physical twin. The state-dependent model is not validated beyond the visited interval.

Hidden physical truth was accessed only after both scenes ended/exported. All detailed truth comparisons, parameter estimates/errors, uncertainty, sensor-resolution sensitivity, models, figures, Agent decision traces, RGB-D associations and evidence indexes are delivered **privately**. The sensor remains an idealized simulation load cell, not a hardware-accuracy reproduction; normal-only contact data are not complete wrenches.

## Next bottleneck and stopping

The next engineering question is bidirectional contact retention and reference handover around reversal, together with a collision-feasible alternative approach. Independent training speed/acceleration excitation is still needed once contact supports it; larger angular travel alone does not identify inertia. No higher squeeze force or relaxed protection is justified by this result.

Stopped after the two registered configurations. No extra physics trial, object, controller/ranking tuning or hinge-discovery module was started.

## Source and delivery boundary

- Frozen runtime/protocol: `c360e38` and [protocol](PROTOCOL.md).
- Offline evaluator: `scripts/evaluate_xarm_ft_v1.py`.
- Private plots/analysis: `scripts/plot_xarm_ft_v1_results.py`, `scripts/summarize_xarm_ft_v1.py`.
- Private evidence audit: `scripts/audit_xarm_ft_v1_evidence.py`.
- [Post-episode source hashes](POSTHOC_SOURCE_HASHES.json).

Automatic publication review rejected the internal evidence-location index, internal estimated-model artifact, and detailed hidden-truth evaluation summary. Those artifacts are retained privately, not published through another route. Public delivery is limited to implementation, this brief and source-preservation provenance. Full private reports and original evidence remain available to the project owner.
