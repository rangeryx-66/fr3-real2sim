# Agent-guided pull–push V1.2 registration

Reference: `587a269`. Case: 45936. Scope: KNOWN_MODEL / simulated wrist RGB-D / native G1 / approximate actuator / ideal virtual six-axis wrist load cell.

One fresh continuous closed-start episode, conditional on the existing full panel-route preflight. At most two planned handle acquisitions and one panel-push attempt. No physical retry, between-case tuning, or additional object. Four original Agent-selected wrist adjustments. All original perception, ranking, closure, hold, motion, collision, joint, force and loaded-TCP protections remain unchanged.

The Agent inspects fresh RGB-D, masks, geometry, selected grasp and physical feedback. Initial pull is toward 10–15° when supported. Subsequent contact mode is selected only from a verified unloaded decision state. Panel targets come from the fresh wrist RGB-D, never from the archived preflight coordinates. Optional second acquisition requires a completed planned panel transition. A failed manipulation is preserved and is not a pass merely because safe parking succeeds.

Before the episode: concurrent atomic publisher/consumer and malformed-message tests; actual transformed owner test; reversed-start archived F/T packet replay; same runtime panel planner applied to a measured closed-state panel patch propagated to 10° with the permitted hinge prior. This offline propagation is a feasibility hypothesis, not a fresh opened-state observation or executable target.

Transport uses protocol version 1, exact sequence IDs, ACCEPTED and COMPLETED ACKs. Invalid messages get a PROTOCOL_ERROR ACK, zero robot action and the next sequence; a corrected message can continue in the preserved unloaded scene. Duplicate publishers cannot overwrite the same sequence. Every Agent operation shares this transport. Safety exceptions retain their original termination behavior and get an outcome ACK.

Identification writes RAW_READY only after all packet inputs close, checks hashes, waits for predecessor MODEL_READY in a passive worker, and never silently fits an incomplete prefix. Estimation remains diagnostic and cannot block loaded execution. Full raw data stays private.

Constant-resistance dynamics are sampled once using seed 181845936 from the existing V1.1 prior family. Hidden values are accessible to scene generation and post-hoc evaluation only. The estimator retains the same segment-local algorithm and uncertainty limitations. No valid opening/closing data is discarded because a later action fails.

Private evidence root on the experiment server: `results/agent_pull_push_v12_20261018`. Source/configuration hashes are frozen in `FROZEN.json`, and the actual generated executable is hashed separately. No physical result is claimed by this registration.
