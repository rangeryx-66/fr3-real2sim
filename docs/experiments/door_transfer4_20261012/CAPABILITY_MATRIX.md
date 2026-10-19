# Capability and failure mechanisms

| Layer | Evidence from existing cases | Shared response in this round | Limit that remains |
|---|---|---|---|
| Hardware / configuration | Original 45415 hit 0.05 rad joint margin; 45203 oracle ROI exceeded a documented height bound | Preserve limits and mounting; optimize initial base/branch; use exact solved 45415 plan as control | A finite fixed-base interval is not proof of a global robot limit |
| Collision representation | Bow visual rear gaps disappear in several native whole-mesh hulls | Compare unchanged visual/native geometry at a common pose/aperture; report warning | Original-v1 geometry stays; [v2 proposal](COLLISION_V2_PROPOSAL.md) is not executed |
| Candidate / planning | Benchmark A picked trim on 45176/45203/45244; four cases lacked executable plans | Explicit oracle handle ROI; contact-interior/opposition/opening-direction score; bounded range reward and common-aperture collision checks | Predicted support and IK range are not physical success or execution gates |
| Contact transmission | Correct 45176 grasp later became unilateral and slipped while arm tracked within 46 μm | Actual progress versus drift guides reference hold and at most one existing centering correction | No release/regrasp/base recovery is permitted; single-pad unloading alone is not failure |
| Execution / software | Earlier wrong runner and SIGINT buffer loss obscured repair/evidence | Pin actual source/config hashes, compile explicit shared policy, reuse verified chunked native-rate export and cooperative stop | Unexpected software exits remain failures; missing optional telemetry is not a task veto |

The four-case outcomes are recorded separately after physical execution. They
must not overwrite original Benchmark A or imply a robust statistical estimate.
