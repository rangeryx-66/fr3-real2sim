# Same-scene planning-recovery gate

Reference: 84be8e0. Engineering validation, not a full-range benchmark.

The archived 90° declaration had a best nominal endpoint of 81.1°. After a
successful measured 20° segment, the original live successor check rejected
link3/original31. No collision path was executed. The Agent then finished,
without testing a cheaper same-station alternative. Both decisions were wrong:
partial nominal reachability is not full-target feasibility, and one rejected
route does not establish global infeasibility.

## Contract

* Every declared action, including its optional alternatives, must fall inside
  the selected candidate's checked arc. The existing candidate enumeration
  searches its ranked geometries and IK branches; incomplete candidates now
  fail admission and the search continues. Ranking weights remain unchanged.
* `target_coverage.full_requested_horizon_feasible` is separate from
  `full_requested_actions_feasible`. An advisory longer horizon is never an
  executable action or a physical progress measurement. An executable 90°
  target cannot be approved when only 81.1° was checked.
* A FIT successor may contain at most two complete `path_alternatives`, with
  distinct FIT roles and explicitly declared targets/profiles. After native
  route rejection, the deterministic owner checks shorter, same-direction
  alternatives from the measured stopped state. It executes only one whose
  original native checker passes. The original rejected request remains in
  the trace. VALIDATION targets cannot have substitutes.
* Unstable transmission, resource exhaustion or a hard latch cannot enter this
  continuation branch. No Agent wait occurs while loaded. If all checks fail,
  original guarded release/parking returns `PLANNING_REJECTED` and an unloaded
  recovery packet, not an automatic episode failure. Where measured costs and
  resources permit, fresh post-contact wrist observation is acquired without
  an extra communication round trip. The Agent must inspect it before choosing
  a new grasp or station. Existing automatic candidate search and explicit
  native-validated relocation remain available.
* Before another acquisition, measured previous observation, planning and
  attempt costs reserve one recovery opportunity, in addition to the existing
  deterministic safe-exit reserve. Unknown initial costs are labeled unknown.
* Multiple safe plans and acquisitions within the registered same-scene budget
  are allowed. This does not authorize fresh-scene outcome retries, changing
  physics, cached grasps or hard-stop resets.

## Short physical test (one fresh scene)

One revolute development scene, fresh wrist images and automatic native-G1
grasp, existing physics/authority/guards. Targets are chosen after fresh image
inspection and are restricted to a short checked arc. The episode explicitly
opts into `path_rejection_fixture:true`: at the first native-valid successor,
inject one **planning rejection**, then require a different predeclared action
to pass the unchanged native checker and physically continue. This is not
reported as an actual collision or proof of full-range reachability. An actual
native rejection is never overridden by the fixture.

Acceptance: the rejected request stays unexecuted; a distinct checked action
produces measurable continued object motion; meaningful reverse motion and
verified safe parking; unchanged loaded-source/frozen-baseline audits. Preserve
any physical failure and do not retry a valid completed scene. No full-range
benchmark or independent repetitions in this gate.
