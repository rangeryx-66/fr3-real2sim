# PiPER contact ownership: implemented and physically tested

## Result

The old whole-finger contact-point projection classifier is retired. Current
PiPER execution and canonical entry points use native collider identity.

| Test | Actual result | Native non-pad contact | Cooked/raw audit | Decision |
|---|---|---:|---|---|
| Canonical interior box, 16×24×6 mm | bilateral pad closure, aperture 24.0016 mm; 2 mm command produced **1.519 mm measured tangential displacement** | 0 / 2700 physics substeps | no forbidden intersections; no ownership mismatch | PASS |
| Frozen formal handle, original fixed base and grasp | first forbidden contact during closure at aperture **24.4450 mm**; pads not yet loaded | right finger body, total approximately **0.779 N** | corresponding body collider in native contact envelope; original body mesh intersects original handle in 6 triangle pairs | REJECT; no pull or opening |

Canonical minimum joint margin: **0.3460 rad**. Its pad forces at closure were
approximately 9.9 / 9.9 N. Every loaded physics substep stayed within
approximately 9.77–9.99 N across both pads. The canonical box is a diagnostic section wholly inside the
pad footprint, not a replacement for the formal handle. A wider 20 mm section
was also tested and rejected when its pull reached a non-pad surface.

The formal grasp's closure margin was **0.3615 rad**. The door angle at the stop
was approximately 0.000226°, not an opening result. No base reposition or grasp
pose search was performed in this change.

## Ownership and geometry

- The official AgileX finger DAE contains a separate distal surface geometry
  with **four triangles**. Its vertices map to the unchanged official STL
  within 46 nm. Those exact STL face IDs define the permitted surface.
- All other exposed finger surfaces are forbidden `metal` ownership. This
  label means **non-pad finger body**, including surfaces whose visual material
  happens to be named plastic. Material color does not grant contact permission.
- An interior pyramid gives the distal surface its own volumetric collider.
  Its other faces are interior to the source collision hull. The complement
  belongs to forbidden finger body. This is a mathematical interior partition,
  **not a claim about a manufacturer-certified pad thickness/material**.
- The authored compound preserves the original full-hull exterior, bounds and
  volume (5.2784412e-5 m³ per finger; numerical volume difference <5e-20 m³).
  Metal partitions are further tessellated into tetrahedra; the pad remains one convex shape: **505 colliders/finger** (504 metal tetrahedra plus one convex pad).
  Official STL vertices, dimensions, visual mesh and mass/inertia are unchanged.
- The old hull remains the source collision approximation: it fills concavities
  of the official non-watertight STL. This experiment does not certify that
  approximation as a measured hardware contact model.
- Re-cooking seven large partitions removed about 2.05% of hull volume and was
  superseded. Tetrahedral cooking changes union volume by **+0.251%/+0.266%**;
  some cooked bounds expand by up to 0.68 mm. These native differences are
  reported rather than hidden. Original raw-body surface audits remain a veto.
- Offline checking reads **native PhysX convex vertices/polygons and full
  transforms** from `request_convex_collision_representation`. It never replaces
  a failed export with a synthetic trimesh hull.
- Original native finger `contactOffset=0.52999996 mm`, `restOffset=0` are copied
  from the measured source export. No distance tolerance permits body contact.
  The canonical analytic box has an explicit 0.12 mm contactOffset (2% of its
  smallest dimension); both sides use the same box geometry and transform.

`pad ↔ semantic handle collider` is allowed; `metal/unknown ↔ target` and
`pad ↔ non-handle part` are forbidden. Native impulses identify loaded contact. Canonical complete substep replay passed
with zero cooked/raw forbidden intersections and zero native-owner mismatches.
ContactOffset proximity is reported separately as **potential contact**, not
misrepresented as contact force or ignored penetration.

## Agreement and time alignment

PhysX contact headers provide collider IDs. This runtime returned face index 0
for every old whole-finger contact, so face index could not resolve ownership.
Independent collider paths now resolve it directly. There is no point-to-pad
projection, ±1 mm rule, or nearest-point contact classification.

The segmented frozen part cloud is associated with a scene collider once,
before execution. The formal handle won **99.59%** of nearest-surface votes.
This association is generic and contains no object-ID-specific rule.

Contact generation precedes the solver/integration pose update. At the first
formal body contact, SDK separation was **14.564 µm**, while FCL using the
same native pre-integration pose returned **14.607 µm** (difference 0.043 µm). Post-step geometry is audited separately; comparing it to pre-integration
contacts can create a temporal discrepancy. Recording now captures native rigid-body poses and collider contacts at every
physics substep, plus pre/post control-loop poses. Raw audit also checks the actual post-step
geometry, where original non-pad/handle surfaces intersect.

Canonical replay covers all **2700 native physics substeps**, rather than only
1440 control-loop records or an estimated point-cloud aperture. Formal motion stopped on the first native body impulse.
Original official-mesh audit independently rejected the resulting state.

## Force measurement and discarded prototype

Rendering a frame advances eight physics substeps; the intervening control
records advance one. Dividing all eight accumulated impulses by one 1/240 s
step inflated the apparent force to about 80 N. The reporter now measures
actual substep time and divides the net impulse vector by that interval.
Instantaneous substep contact forces are retained separately. No gains, effort
cap, friction or simulation dt changed.

The earlier 514-shape all-tetra prototype also exhibited a transient roughly
2 kN aggregate impulse report. Its earlier acceptance is superseded. Keeping
the pad as one convex shape removes artificial internal pad contact faces;
the adopted 505-shape version showed no such spike (maximum single reported
contact below 10 N). Native cooking expansion and raw-mesh veto remain explicit.

A repeated canonical run with corrected video timing produced byte-identical
physics-step and cooked-geometry files. Diagnostic video is encoded at 16 fps
for the existing 15 physics steps per recorded frame, instead of presenting
11.25 s of physical motion in a 6 s video.

The old `Model.check(..., allow_pad=True)` fails closed with
`OWNED_CONTACT_VALIDATION_REQUIRED`; it cannot certify contact using raw hull
point projections. Its FK, IK, limits, margin and free-space collision path
are unchanged. The historical cooked-contact CLI now forwards to the owned
validator. Legal loaded grasp claims require actual native closure and replay.

## Files and reproduction

- `piper_mobile_demo/contact_ownership.py`: source surface IDs, preserved-volume
  compound generation, native ownership reporting.
- `config/piper_contact_ownership.json`: reproducible generated geometry and
  source hashes, native offset provenance.
- `piper_mobile_demo/cooked_geometry.py`: native export including ownership.
- `scripts/validate_piper_owned_contact.py`: cooked-solid checking, original
  forbidden-body surface audit, actual trajectory replay.
- `scripts/piper_owned_contact_test.py`: actual canonical closure and pull.
- `scripts/piper_owned_formal_contact_test.py`: canonical-gated original formal
  closure and small pull only if the complete closure passes.
- `scripts/annotate_piper_probe_evidence.py`: measured displacement, bilateral
  contact and complete replay acceptance.
- `scripts/audit_piper_owned_shapes.py`: native bounds/volume/transform audit.

The compatibility entry `piper_mobile_execute.py` now routes to the owned formal
contact diagnostic and requires the canonical acceptance file; opening arcs are
not executed. `piper_canonical_handle_test.py` routes to the owned canonical test.
The original R1/Dex1 and FR3 implementations are unchanged.

Server checkout: `/data1/home/rangeryx/fr3_real2sim_piper_mobile`.

```bash
GEOM_PY=/data1/home/rangeryx/.conda/envs/anygrasp/bin/python
SIM_PY=/data1/home/rangeryx/isaaclab-arena/.venv/bin/python

env -u PYTHONPATH "$GEOM_PY" scripts/prepare_piper_contact_ownership.py \
  --source-cooked-export results/piper_cooked_audit/cooked_shapes.json \
  --output config/piper_contact_ownership.json

env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES "$SIM_PY" \
  scripts/piper_owned_contact_test.py \
  --measured-report results/piper_final_strict_fixed/report.json \
  --pull-plan results/piper_canonical_pull.json \
  --output results/canonical_owned --gpu 7

env -u PYTHONPATH "$GEOM_PY" scripts/validate_piper_owned_contact.py \
  results/canonical_owned/cooked_closed.json results/canonical_owned/full_validation.json \
  --target-body canonical_handle --observations results/canonical_owned/physics_steps.json

env -u PYTHONPATH "$GEOM_PY" scripts/annotate_piper_probe_evidence.py \
  results/canonical_owned/full_validation.json results/canonical_owned/physics_steps.json \
  --ownership config/piper_contact_ownership.json \
  --pull-plan results/piper_canonical_pull.json
```

Formal invocation uses `--canonical-validation .../full_validation.json`, the
same manifest hash, the original `--source`, `--asset-root`, frozen `--plan` and
`--candidate-index 7`, plus `--pull-plan results/piper_canonical_pull.json`.
Physical runs enforce the Shanghai 05:00 cutoff. No test remains running.

## Remaining blocker

Unified contact validation is demonstrated on a legal canonical grasp and
an illegal formal grasp. The current formal pose closes into a non-pad finger
surface before distal pads carry load. Opening remains blocked by this real
geometry conflict, not IK or a permissive pad-label mismatch. Any future grasp
search must use this ownership model; no pose/base/friction/effort adjustments
were made to improve the formal outcome in this change.
