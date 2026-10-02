# Ownership repair: blocked by cooked exterior mismatch

Date: 2026-10-02, Shanghai. Status: **NOT ACCEPTED FOR EXECUTION**.

The same frozen nominal grasp was retained. No base, IK bounds/settings,
margin, friction, effort limit, object joint/mass/inertia, visual geometry,
grasp family or mobile base was changed. No new pose search was run.

## Implemented

- V2 generation uses the original unsplit native finger envelope, whose source
  collider raw vertices are checked against the official STL. Ownership is an
  interior partition; official pad surface IDs remain unchanged. The authored
  union conserves original native volume to floating-point precision.
- Each v2 cell uses a centred local coordinate representation with a
  compensating USD transform, preserving world dimensions. The legacy manifest
  installer remains unchanged. This is not an official geometry rescale/inset.
- Before closure the runner checks actual cooked vertices against authored
  cell halfspaces and the original envelope, original support planes, and
  sampled exterior coverage. Support planes alone can miss gaps between cells.
  The float32 bound is a numeric representation bound, not a physical
  penetration tolerance. Failure pauses/closes the simulation before approach.
- `NonpadClassifier` separates raw-positive-gap proximity/representation
  warnings from raw contact and sustained load. Contact on an unknown target or
  an unexplained impulse fails safely. The provisional conservative sustained
  load gate is 50 ms of consecutive non-pad impulse; no new force allowance is
  introduced. This temporal gate has not been physically validated in a new
  closure, because the exterior gate has not passed.
- The previously recorded first substep (6017) was processed with this
  classifier: gap **0.247310 mm**, raw intersection false, classification
  `PROXIMITY_OR_REPRESENTATION_WARNING`, confirmed forbidden false. This was
  offline replay of a real recorded contact, not a fresh closure experiment.

## Actual geometry results

Several ownership-only exports were tested; no robot manipulation was run.
They are kept on the server under `results/semantic_interaction`.

| Representation | Cooked exterior result |
|---|---|
| Large halfspace cells from dense raw hull | outward and inward deviations; rejected |
| Normalized original raw tetrahedra | residual outward deviation; rejected |
| Convex-union merge of adjacent raw tetrahedra | residual outward deviation; rejected |
| Original native envelope, 7 halfspace cells | outward error <1 nm, but uncovered exterior samples about 46.6 µm; rejected |
| Original native envelope, rear-first 8 cells | outward error <1 nm, but uncovered exterior samples **35.826 µm**; rejected |
| Native envelope facet cells | outward error **23.667 µm**, uncovered samples 11.566 µm; rejected |

The 8-cell result is the retained candidate, **not a validated repair**. Its
cooked volume sum is 5.202971939e-5 m³ versus original native 5.204736795e-5 m³.
The gap proves that simply eliminating the old protrusion is insufficient to
claim union/exterior equivalence. The old support-only pass was superseded by
the stricter coverage check; it did not authorize physical closure.

The original SDK collision hull already differs from the dense visual STL.
Using that original hull restores the original execution reference rather than
introducing a new simplified finger model. The official STL remains an
independent non-pad contact audit. Neither representation is silently waived.

## Nominal grasp result

- Original first non-pad impulse: representation-induced near-contact; not
  confirmed official non-pad touch.
- Bilateral pad load after accepted repair: **NOT TESTED**.
- Low-preload hold after accepted repair: **NOT TESTED**.
- 2 mm pull after accepted repair: **NOT TESTED**.
- First confirmed failure this round: **ownership geometry acceptance**, not
  an actual grasp/contact failure. The nominal grasp's true feasibility remains
  undetermined. No illegal-grasp or successful-grasp claim is justified.

Next work must prevent independent convex recooking from altering partition
edges, or use a supported native ownership representation that retains one
physical envelope. Do not resume closure by relaxing the envelope gate or
introducing a global penetration threshold.

## Artifacts

- [Retained 8-cell cooked audit](experiments/ownership_repair_20261002/ownership_envelope_audit.json)
- [Unaccepted authored candidate](experiments/ownership_repair_20261002/ownership_candidate_not_accepted.json)
- [Prepared but unexecuted slow-close policy](experiments/ownership_repair_20261002/policy_not_executed.json)
- [Rejected native-cell audit](experiments/ownership_repair_20261002/native_cells_rejected_audit.json)

The retained candidate/policy are diagnostic snapshots, not promoted defaults.
The policy only slows closure to 0.25 mm/s and changes its ownership hash;
friction, clamp limit and preload remain unchanged. The runner permits a longer
closure time budget for this slow rate, while preserving the 05:00 cutoff.

Dependencies: the Isaac Python environment additionally needs `python-fcl` for
online raw gap classification. Large cooked exports and physics traces stay on
the server. No simulator process is left running.
