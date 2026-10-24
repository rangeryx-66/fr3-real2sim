# Full integration activation, after matched controls

These are fresh engineering scenes, not independent repetitions or replacements
for historical failures. Run revolute then prismatic, sequentially. Use the fixed
previously revealed engineering configurations only as opaque physics commitments:
revolute SHA256 f234c4ce272cdde1224580e838bf9c7c89c7a5e1c17a92bf01e64d114ef0ca50;
prismatic SHA256 72259bb52cf82eb5f24a1be3fb58dc49ae4f3af44c4bb9787e157628ba272215.
Do not read authored parameters for online actions or fitting.

Use fresh wrist RGB-D/SAM3 and automatic measured-geometry candidates. Matched
control poses and historical successful views/grasps are forbidden inputs.
Original goals remain 90 degrees and 150 mm, regardless of intermediate actions.
Current declared actions and final-goal coverage must be reported separately.
Native collision/IK rejects remain authoritative and recoverable at safe
boundaries; search current automatic alternatives, then fresh observation and
Agent-selected station relocation/regrasp when needed. Never silently discard the
original goal. Seek real native rejection -> different checked plan -> physical
continuation evidence; an injected rejection alone does not establish that branch.

Each scene has the existing full-range DEBUG envelope: 5400 wall seconds, 2700
simulation seconds, 600 wall/120 simulation seconds reserved exit resources,
600-second maximum verified-unloaded Agent window and six finite acquisitions.
No slow Agent decision occurs under unstable load. Use measured phase costs to
admit recovery, prioritize minimum qualified bidirectional FIT and complete
independent VALIDATION before optional expansion, and stop safely when resources
or genuine native protections prevent continuation.

## Numerical handover scheduling amendment

The matched study exposed a worker scheduling race without changing physics:
condition1 completed validation; conditions2/3 completed all three FIT motions,
but the latest worker became ready after the one-second endpoint handover.
Their original validation requests remained pending and became READY after safe
parking; they must not be called completed validation.

Before full integration, publish measured per-packet worker wall latency. At a
verified stable stopped endpoint only, allow deterministic numeric handover for
up to 1.5 times the preceding measured worker latency, capped at 10 wall seconds
and 3 simulation seconds (also bounded by remaining work resources). Without a
measured latency, retain the one-second fallback. Real 240 Hz stepping and all
original protections continue. The original acquisition hold remains one second.
Aperture change >=0.5 mm or relative-motion change >=the original closure slip
bound ends waiting and routes to guarded release. No pad-symmetry or single-pad
zero rule is added. No stale model is used. Pending validation survives unloading
and must receive fresh current-state planning and Agent grasp approval before
resuming. Data insufficiency requests specific missing excitation; worker failure
is a separate infrastructure result.

This amendment concerns finite scheduling and passive provenance only, not G1
force authority, controller gains, force/collision/joint limits or estimator math.
The four matched preload conditions keep their original identical execution
version; do not reprocess their physical outcome into a new validation PASS.

Only both complete engineering tasks may unlock a frozen behavioral release and
two genuinely fresh independent repetitions per joint type. Unknown articulation
is out of scope. Preserve every attempt and report unsuccessful stages separately.
