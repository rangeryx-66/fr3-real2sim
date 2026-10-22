# R2 registration after preserved R1 failure

R1 `v75_19179_fresh_00` is a valid engineering result, not an infrastructure retry.
Initial pull stalled at 1.373 mm; guarded release and park succeeded. Same-scene
fresh observation, a different automatic candidate and legal 1.5 N preload gave
a completed 10.934 mm pull (absolute 12.124 mm). Reverse travel was only 1.859 mm
before original command/native-collider hard stop. No VALIDATION or final park.
The actor exited; no latch was reset and no further action was executed there.

Offline terminal geometry is clear by 7.31 micrometres at the last accepted
command, then the next proposed command was rejected. The reverse command moved
3.208 mm while measured TCP moved 2.149 mm and drawer 1.859 mm. The old preflight
translated robot and drawer together, omitting the already-permitted relative
reference lead. Actual coupled-finger state also differs from nominal geometry.

R2 changes only prismatic conditional planning and feedback presentation:

- Check both signs of the existing total rail-reference allowance, derived from
  **two original** stopping-distance/one-step allowances (3.7 mm total), with the
  object held at the corresponding measured/predicted physical coordinate.
  No reference-clock parameter or collision margin changes.
- Recheck that relative envelope over each next segment with actual coupled
  finger state and retained arm command bias. A rejected successor remains
  SAFE_PARTIAL_PROGRESS, zero new motion; original release/exit is requested.
- Focus contact plots on acquisition/interaction, retaining every native sample
  and explicit display interval; distinguish the two motion errors numerically.
- Allow current-attempt terminal-hard-stop diagnostics without claiming an
  unloaded boundary or permitting any recovery/motion after that stop.

Offline R2 rejects the R1 collision pose before acquisition. It preserves the
archived 45290 positive chain (2,376 native configurations). Those are geometric
checks, not physical success. A repeat development scene on 19179 is separately
identified as R2, with all original R1 evidence intact. No benchmark is started.
Remaining 27044/45746/45290 scenes retain the original bounded engineering scope.
If observations cannot produce an envelope-safe grasp, preserve that limitation
instead of weakening the envelope, copying an archived grasp or changing physics.

Load bands, G1 drive authority, physics, estimator mathematics, original hold,
hard stops and all 71+22 revolute dependencies remain unchanged. Readiness still
requires actual bidirectional contact, complete VALIDATION and verified park.
