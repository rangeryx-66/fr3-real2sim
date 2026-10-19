# Identical Agent instructions for both registered development cases

Inspect actual fresh wrist RGB, SAM3 masks (including partial cues), metric depth,
local measured clouds and incremental measured/uncertain surfaces. Choose useful
reachable views from observed missing information, with at most four adjustments
in the episode. No saved views, GT handle surfaces, scene-camera depth, clicks,
asset-specific framing or changed prompts/scores. Known station/hinge/state and
existing collision checks remain explicit assumptions.

Inspect automatic native-G1 geometry and grasp plan, then explicitly request one
attempt_grasp_and_probe. It calls frozen approach/refinement/closure, original
one-second hold and +3-degree controller. Existing soft-force warning ends the
current probe for automatic guarded unloading; any original hard exception ends
its current operation immediately. No cleared force history under load, no
disabled guards, force increase, manual grasp change or single-pad-zero veto.

Never wait for Agent analysis under unstable contact. Automatic release must
first remove load with guarded finite position commands, check actual contact
release and collision-checked retreat. If it cannot, end the episode safely and
preserve the stop. Only verified unloaded observation states may pause/wait.

Actually inspect returned contact images, raw/filtered load/aperture/relative
motion plots, known-state progress and original stops. Distinguish measured,
inferred and unavailable fields. No need to prove a unique cause before deciding
to reobserve and use a different existing automatically generated candidate.
Explain which observed issue the new action addresses. No hardcoded offsets or
historical successful pose. Episode-local failed families must not be repeated.

At most one first grasp plus one recovery grasp per case. First success forbids
a recovery grasp. If both cases succeed first time, report recovery untriggered;
do not manufacture failure. After recovery budget, finish and export. These are
two development cases, not a population success rate. Keep raw evidence private.
