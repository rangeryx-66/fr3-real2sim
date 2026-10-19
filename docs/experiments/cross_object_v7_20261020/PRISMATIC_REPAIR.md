# V7 prismatic breakaway repair — engineering record

Status: native-contact validation in progress. No registered research benchmark
has started; software tests and a known-force fixture are not robot capability.
Reference: `25e2189`. Earlier contact failures and their evidence are immutable.

## What the existing recordings establish

| Quantity | 27044 first probe | 45290 first probe |
|---|---:|---:|
| Probe duration including stop hold | 1.804 s | 1.804 s |
| Commanded TCP rail travel | 1.921 mm | 1.852 mm |
| Measured TCP rail travel | 0.705 mm | 1.114 mm |
| Drawer travel | 0.114 mm | 0.0021 mm |
| Incremental command-to-TCP travel deficit | 1.216 mm | 0.738 mm |
| Measured TCP-to-drawer relative travel | 0.591 mm | 1.112 mm |
| SG121 reliable-interior compensated wrist rail force, median | 0.162 N | 0.177 N |
| Same force, maximum | 0.208 N | 0.210 N |

Both probes acquired bilateral contact and completed the original one-second
hold. Neither had a force warning/overload. The repeated stop consisted of
three 0.5 s increasing-drift windows plus the original stop hold. These facts
support delayed/weak transmission and premature application of sliding-stall
logic; they do not establish that extending time alone fixes traction.

The old compact archives lack per-point normal/friction impulses and contact
locations. Absence of these channels is reported rather than reconstructed as
measurements. A missing positive normal report does not prove separation.

## Independent native breakaway fixture

This is a separate, explicitly labeled engineering test with no robot contact.
It applies a known external rail force at the measured moving-body COM, under
unchanged native mass/constraints, Coulomb 0.3 N and damping 3 N s/m. It is
never imported by the benchmark, controller or identification worker.

| Asset | Resolved opening / reverse onset | Maximum actual displacement | Closing travel |
|---|---|---:|---:|
| 27044 | +0.325 / -0.325 N | 17.5724 mm | 11.6862 mm |
| 45290 | +0.325 / -0.325 N | 17.5724 mm | 11.6862 mm |

The stepped-ramp onset bracket is (0.300, 0.325] N, using a 0.1 mm/s motion
resolution; this is not a calibrated precision claim. The drawer is movable in
both directions without reducing its resistance. The first 27044 fixture was
interface-invalid before force application (installed COM tensor shape); its
failure is retained, and the verified shape binding was repaired.

## Shared change

A finite CONTACT_PRELOAD stage now precedes ordinary sliding-stall detection.
It uses actual rail motion, measured TCP, commanded-TCP FK, and wrist reaction
trends. The reference remains bounded by the same 1.85 mm lead and the existing
slow closure rate (1 mm/s); no force command or actuator authority is increased.
Its finite budget follows permitted reference travel/rate, measured initial
tracking mismatch and two original monitoring windows. It distinguishes loading,
tracking deficit, measured tangential relative motion, incipient breakaway and
persistent sliding stall. A physical breakaway, rather than reference motion,
starts the ordinary profile. The grasp/slip origin is not reset.

Native normal reports and separate PhysX friction-anchor measurements are now
available as passive diagnostics. Empty anchor reports remain unavailable, not
invented zero friction. Contact-point material velocity is a derived kinematic
diagnostic, not a measured pressure distribution or complete contact wrench.

Current-plan validity is explicit: failed or refreshed visual planning cannot
execute a stale saved plan. Same-episode recovery retains failed candidate
families, obtains fresh wrist observations, checks ranked alternatives against
measured released fingers and current native collisions, and requires explicit
Agent approval. The Agent inspects actual images and force/motion plots.

## Checks and remaining evidence

18 deterministic V7 tests pass, including reverse motion, tracking versus slip,
finite stalled loading, native friction-buffer bounds and no new single-pad
veto. The existing fast-runtime safety/protocol regression passes; policy SHA256
is `9ef5bd91047d46b56d3ba61b5bb704272bbd302eccbd14fdb3673e05183a8ee9`.
Native robot/gripper transmission, reversal and traction-sign validation are
pending. Do not freeze the benchmark or label the repair PASS before these.

Private server root: `labserver_inschool` / `test-X640-G50`,
`/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/cross_object_v7_20261019/engineering/`.
No raw images, telemetry or video are published with this report.

## First repair regression: negative result retained

`native_repair_27044_00` executed two actual acquisitions in one continuous
scene. Each acquired bilateral contact and completed the original hold. The
first gained 0.1224 mm, briefly resolved breakaway at 2.504 s / approximately
0.30 N native traction, then stopped with persistent sliding stall. It did not
complete the requested pull, reversal or validation. Same-scene fresh wrist
observation and automatic feasible-candidate search selected the opposite-roll
family; that recovery gained only 0.000169 mm and ended with tangential relative
motion evidence. Both attempts released and reached verified native-rest parking.
Neither is a manipulation PASS. No benchmark object was discarded or replaced.

New native measurements confirm traction is overwhelmingly tangential. During
0.5–1.5 s of attempt 1, mean native rail force was 0.14163 N: normal projection
0.000166 N and friction projection 0.141464 N. At 2.6–3.8 s, native total was
0.143923 N and fully compensated wrist median 0.143797 N. Instantaneous sensor
versus native RMS disagreement was 0.00852 N, with 0.00839 N downstream-tool
inertia RMS. Thus an F/T compensation/sign error is not supported as the main
cause of weak sustained traction. Native points are near lateral pad edges;
this is a diagnostic concern, not an added fatal pad-centering requirement.

The 1.85 mm reference/object lead also consumed incremental command-to-TCP
tracking deflection (~1 mm) and actual TCP-to-drawer motion (~0.8 mm) together.
The clock capped the command, followed by traction settling below breakaway.
The second revision separately allows at most 1.85 mm of **measured incremental
servo tracking deficit**, in addition to the original 1.85 mm contact lead.
Total reference/object lead is therefore finite at 3.70 mm (plus integration
roundoff), never an unbounded reference or hidden friction feedforward. Feedback
and virtual endpoint use the same offset; target success still uses measured
physical coordinate. All original force, 5 mm cumulative slip, velocity,
acceleration, collision, actuator and joint protections remain authoritative.
No grasp/slip origin is reset. Preload budget accounts for both finite travel
components rather than multiplying the timeout arbitrarily.

The Agent can now request an existing candidate ID from a fresh, hashed
measured-geometry catalog. This changes preference, not weights, poses or native
execution checks. Unknown/manual poses and stale observations are rejected
recoverably. This permits skipping mechanically similar edge-supported proposals
based on observed failure feedback without object-specific offsets or a blacklist.
24 deterministic tests and the unchanged safety/protocol regression pass.
Second-revision results follow; this does not establish a complete drawer Skill.

## Second revision: real transmission, incomplete reversal

`native_repair_27044_01` acquired bilateral contact and completed the unchanged
hold. The first pull gained **7.3046 mm**, but failed its 20 mm target and stopped
with persistent sliding stall. During 2.5–5.5 s it transmitted 6.2380 mm of actual
drawer travel with only 0.0203 mm variation in TCP/drawer relative position.
Mean native friction traction was 0.29866 N, normal projection 0.00832 N, and
fully compensated wrist median 0.30809 N. Thus the revised loading clock permits
genuine breakaway and sustained transmission without raising clamp authority.
Afterward contact-relative motion increased and traction degraded; the failed
target remains failed. The qualified prefix was retained for engineering FIT.

After guarded parking, the Agent inspected fresh wrist observations and selected
the automatically generated opposite-roll family. This second acquisition also
completed the hold. Closing gained only **0.7056 mm**, ending near 6.596 mm.
During 0.5–2.0 s closing traction was approximately -0.302 N, while actual drawer
travel was 0.602 mm and TCP/drawer relative-position variation was 1.297 mm.
Incremental command-to-TCP travel deficit remained below 0.115 mm throughout:
the remaining reverse limitation is genuine contact-relative motion, rather
than the original reference clock merely consuming arm tracking error.
Measured aperture increased under a fixed command and pad loading was asymmetric;
these observations do not independently identify actuator compliance or friction.

The complete requested reversal and independent VALIDATION did not execute.
Both attempts released and parked safely. The Agent made a further same-scene
observation: the visible handle's fresh SAM3 maximum score was 0.163, below the
unchanged 0.25 threshold. No stale geometry, GT fallback or repeated inference
was used. With the four-view budget consumed, the actor exported and shut down.
This is bounded recovery failure, not proof that all grasps are impossible.

An engineering-only online estimate from the qualified pull prefix gave
m=1.9901 kg, b=3.0700 N s/m, C=0.29987 N. It has no complete held-out motion and
is not blind identification success. Two physical attempts, no extra trials,
the second-revision scene took 2247.35 s wall time and occupied 60.14 MB.
The 48-byte optional video is an invalid recording and is not claimed as video
evidence; checkpoint RGB-D and essential numerical/contact streams are retained.

The subsequent candidate-interface deployment occurred after this actor's final
FINISH response but before export PID termination because a shell assertion did
not gate the next command. Initial executed snapshots and
`POST_FINISH_SOURCE_BOUNDARY.json` preserve the exact boundary. No subsequent
physics or Agent operation used changed files; the prismatic-controller hash
remained unchanged. The actor's death was verified before starting 45290.
Future deployment is gated on actual PID termination.

## 45290 cross-object regression

`native_repair_45290_00` uses fresh wrist views and the same second-revision
implementation. Its first automatic family `[-0.3,0,0,0]` acquired bilateral
contact and completed the original hold. A predeclared 6 mm pull at 0.8 mm/s
gained only **0.0169 mm** and stopped after 6.308 s with
`PRISMATIC_STARTUP_KINEMATIC_TANGENTIAL_SLIP_EVIDENCE`. No reverse/validation
was executed. Safe unloaded native-rest parking was verified.

Commanded TCP rail travel was 2.652 mm, measured TCP 1.915 mm, drawer 0.0169 mm:
servo travel deficit was 0.737 mm, while actual TCP/drawer relative travel was
1.898 mm. Fully compensated reliable-interior rail force had median 0.114 N
and maximum 0.225 N. Native mean friction traction was 0.1016 N and normal
projection 0.00022 N; native versus compensated wrist RMS difference was
0.0164 N, with dynamic-compensation uncertainty retained. The unfiltered peak
was 0.341 N. No force warning or hard stop occurred. Raw normal pad peaks were
0.971/1.085 N, and actual points lay within approximately 0.573 mm of the lateral
pad edge. Positive pad loads therefore did not establish adequate rail traction.

The Agent inspected load/geometry plots, safely reobserved in the same scene,
and requested an existing pitched candidate using the fresh hashed catalog.
Its complete native approach IK was infeasible. The shared interface then
evaluated other eligible candidates and returned `[0.3,0,180,0]`, rather than
terminating on the first rejected proposal. Fresh frontal geometry was accepted;
a lower oblique observation showed the handle but produced no valid SAM3 mask,
so that failed view supplied no grasp surface. The fallback plan and complete
measured-state transition passed and were inspected before the final short
recovery test.

That recovery acquired bilateral contact and held for one second, but its
predeclared 4 mm pull at 0.4 mm/s gained only **0.0352 mm**. It stopped after
11.308 s with the same startup tangential-relative-motion classification.
Commanded TCP travel was 2.459 mm, measured TCP 1.919 mm and TCP/drawer relative
travel 1.883 mm. Native mean friction traction increased to 0.2060 N, fully
compensated wrist median 0.2192 N and reliable-interior maximum 0.2407 N,
still below sustained 0.3 N breakaway. The unfiltered peak was 0.3205 N.
No force warning/hard stop occurred; the physical failure preceded resource
exhaustion. There was no reverse or complete independent validation.

Both attempts reached verified unloaded native-rest parking. The runner then
returned BUDGET_EXHAUSTED at its global resource boundary, preserving the two
physical transmission failures separately. Wall time including initialization
and cleanup was 3585.99 s; simulation 580.12 s; footprint approximately 60.9 MB.
The late feedback command 015 was published after actor exit and was not
consumed; final feedback figures were generated and inspected offline from
archived measurements, without another action. Optional video was invalid
(48 bytes); no continuous-video claim is made. Essential numeric/contact and
RGB-D checkpoint evidence is intact and private.

The two different feasible grasps still produced native contact points near
the same lateral pad edge. Their observed-geometry predictions had substantially
larger nominal lateral margins. This is evidence of contact-model/placement
mismatch, not proof of a unique friction or compliance cause. The known-force
fixture rules out an immovable rail at 0.3 N; neither longer loading alone nor
the present alternative grasp fixes sustained transmission. The shared startup
and candidate-fallback mechanisms work, but **prismatic capability is unresolved**.

Since the 27044 reversal prerequisite remains unresolved, the user's subsequent
instruction authorizes the frozen four-door/eight-episode revolute benchmark
first, with all eight prismatic configurations explicitly unexecuted. No further
drawer tuning or replacement object is authorized automatically after this test.
26 deterministic V7 tests, the unchanged safety/protocol regression and adaptive
segment/validation-isolation regression pass. These are software checks, not
prismatic capability or hardware safety evidence.
