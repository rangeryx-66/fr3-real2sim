# PiPER parallel-gripper contact representation audit

## Scope

2026-10-01/02, `codex/piper-mobile-door`, based on `0d4a3b0`.
This is a contact-model diagnostic, not a successful door-opening experiment.
The existing R1/Dex1 baseline is untouched. PiPER base `(0.50, -0.55, -0.10 m; yaw=150°)`, IK, >0.05 rad joint margin, gains, 10 N finger effort caps, source meshes, friction and formal articulated-object parameters remain unchanged. No mobile-base motion or larger opening-angle experiment was performed.

## Findings

### 1. The previous “cooked geometry audit” was incomplete

The importer puts collider meshes below USD instance proxies. Ordinary `stage.Traverse()` omitted them; the previous robot collision audit was empty. The new exporter uses `Usd.PrimRange.Stage(..., Usd.TraverseInstanceProxies())`, exports the PhysX cooking interface's native vertices/polygons for the running stage, and reads runtime offsets/materials using a stage-specific tensor view. It does not reconstruct a substitute hull with trimesh.

The offline validator consumes these exported shapes, including each individual door collider and the mesh-to-world affine transform. Scale is baked into vertices before FCL construction. It independently retains an original-mesh audit: matching the simplified simulator geometry must not turn an official-mesh metal intersection into a legal grasp. Missing fingers, missing targets or failed cooking exports are errors, never an empty “collision-free” result.

The finger collision-local transforms are identity with scale 1; FK-to-USD position errors are 0.116/0.127 µm. Imported finger vertices match the official STL to floating-point precision. Door collision meshes match the URDF/FCL geometry within 0.022 µm. A transform/scale mismatch is therefore not the observed submillimeter cause.

### 2. Actual cooking changes the contact envelope

| Shape | Raw imported mesh / complete raw hull | PhysX cooked representation | Measured difference |
|---|---|---|---|
| Each official PiPER finger | 1,017 raw vertices; complete raw hull 122 vertices | 29 vertices, 54 triangles | Hull volume −1.676%; maximum raw-hull vertex distance to cooked surface 0.648 mm |
| Contacting handle collider (`original19`) | 239 vertices | 34 vertices, 64 triangles | Volume −6.991%; one bound recedes 0.699 mm; maximum raw-hull vertex distance to cooked surface 1.277 mm |

These distances are vertex-to-surface diagnostics, not a certified continuous Hausdorff bound. Cooking fills the raw finger's concavities while also dropping convex-envelope detail. Bounds alone do not establish equivalence.

The measured finger runtime `contactOffset` is 0.530 mm, `restOffset` is 0. The door's 15 runtime shapes have contact offsets about 0.341–0.463 mm and zero rest offsets. The tensor interface returns offsets per body shape; this audit does **not** assert an unverified index-to-door-mesh mapping. Runtime materials are static/dynamic friction 0.5/0.5 and restitution 0 for fingers and door, unchanged. FCL has no added shell or mesh shrink. `contactOffset` is the contact-generation shell, not permission for metal intersections.

A diagnostic stage-only `hullVertexLimit=255` ablation left the exported finger/handle geometry unchanged. It was not adopted in production. The remaining simplification is not explained by the vertex budget alone. PhysX documents QuickHull plane-tolerance pruning, but its live plane-tolerance value was not exposed/read in this run; attributing the entire error to a particular SDK parameter would be an inference.

### 3. End-state agreement does not make the closure legal

At the repeated physical closure endpoint, actual aperture is **23.868842 mm**. Pairwise cooked/cooked surfaces do not intersect, but original finger/original target meshes do have non-pad intersection witnesses. Cross ablations reproduce violations when just one side is restored to its original geometry. This is not merely an FCL contact-position artifact: the checker uses actual triangle-intersection witnesses rather than assigning the opposing object's vertex to the pad.

The complete **720-frame closure replay** gives a stronger result:

| Representation | Frames with strict non-pad witnesses | Maximum non-pad normal depth |
|---|---:|---:|
| Actual PhysX cooked shapes | 74 / 720 | 0.259 mm |
| Original finger + original target meshes | 356 / 720 | 0.582 mm |

Thus the earlier 0.02–0.30 mm endpoint problem includes real original-geometry conflicts hidden by cooking, and the closure transient also conflicts in cooked geometry. It cannot be dismissed as a validator-only bug or excused by contact offsets.

### 4. “PhysX pad-only” was an application label

PhysX reports contact for the whole finger body. The earlier Python code projected contact points onto an inferred flat distal patch and accepted points within **±1 mm** of its plane. That label is not a native PhysX pad/material identity and can admit metal-edge contacts.

The new reports separate the legacy point label, cooked-mesh witnesses and raw-mesh witnesses. The inferred contact patch still comes from the official STL's distal flat faces; the official model does not supply a separately certified pad-material mesh. No metal contact was added to an allowed-collision list.

## Canonical handle sanity test

An independent diagnostic scene uses an **analytic box 200 × 24 × 18 mm**, similar in length/width to the current handle. Its pose is computed from the official finger-pad centroids, not an object-ID offset. USD and FCL use the same analytic box dimensions and world transform. The same official coupled fingers, fixed robot base, measured arm posture, gains, friction and effort caps are reused. The box is locked: a tangential pull tests contact/load geometry, not free-object lifting or door interaction.

- Actual closure aperture: **24.000872 mm**; bilateral finger forces approximately **9.84 / 9.92 N**.
- Cooked endpoint audit: no strict non-pad witnesses.
- Raw endpoint audit: finger-2 pad-edge witnesses fail the unchanged 1 µm classification threshold; approximately 1.17 µm normal penetration and up to 13.5 µm distance beyond the inferred flat patch.
- Full diagnostic closure + 2 mm pull replay: cooked violations up to 1.85 µm edge distance / 1.22 µm normal depth; raw violations up to 19.0 µm edge distance / 1.65 µm normal depth. Joint margin stayed above 0.346 rad.
- **Certified legal pad-only canonical grasps: 0 / 1 tested posture.** The final strict rerun stopped after closure and did not execute the pull.

The earlier pull recording is diagnostic evidence only. A missing-static-target selection bug in the initial audit guard was fixed; the full retrospective audit rejects that recording and the corrected rerun blocks continuation. No success rate is inferred from the recording.

Because the analytic target also shows disagreement, the fault is **not exclusively the microwave handle collision mesh**. There is a generic cooking/contact-validation issue, amplified by the original curved handle's larger cooking error. This evidence does not prove that the physical PiPER gripper cannot grasp the handle.

## Code and reproduction

- `piper_mobile_demo/cooked_geometry.py`: read-only native cooking export and runtime offsets/materials.
- `scripts/validate_piper_cooked_contact.py`: exact exported-shape FCL check and raw/cooked cross ablations.
- `scripts/replay_piper_contact_geometry.py`: every recorded closure/pull state, not an estimated aperture endpoint.
- `scripts/piper_contact_geometry_audit.py`: recorded-posture cooking/transform audit; optional vertex-budget ablation in an in-memory diagnostic stage.
- `scripts/piper_canonical_handle_test.py`: independent analytic-box physical closure/pull, with mandatory geometry gate before pull.
- `scripts/prepare_piper_canonical_pull.py`: unchanged PiPER IK for a 0.25/0.5/1/2 mm TCP-tangential diagnostic.
- `scripts/piper_mobile_execute.py --contact-audit-only`: actual existing grasp approach/closure, exports and audits the complete closure, always stops before door-opening motion.

Server root: `/data1/home/rangeryx/fr3_real2sim_piper_mobile`.
Geometry Python: `/data1/home/rangeryx/.conda/envs/anygrasp/bin/python`.
Isaac Python: `/data1/home/rangeryx/isaaclab-arena/.venv/bin/python`.
Use `env -u PYTHONPATH`, and also `-u CUDA_VISIBLE_DEVICES` for Isaac.

```sh
# Actual formal scene, existing frozen rank-9 variant; never opens the door.
env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/piper_mobile_execute.py \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --plan results/piper_fixed_center_search/report.json --candidate-index 7 \
  --contact-audit-only --output results/piper_measured_cooked_closure_final \
  --gpu 6 --deadline-shanghai 2026-10-02T05:00:00+08:00

# Independent analytic target; a raw/cooked failure blocks the pull.
env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/piper_canonical_handle_test.py \
  --measured-report results/piper_final_strict_fixed/report.json \
  --pull-plan results/piper_canonical_pull.json \
  --output results/piper_canonical_box_strict --gpu 7 \
  --deadline-shanghai 2026-10-02T05:00:00+08:00
```

Server retains full native exports, joint/contact histories and videos. Delivery subsets preserve contact-relevant exported vertices/transforms and record the original export SHA-256; they are not recooked approximations. See `docs/piper_contact_model_summary.json` and the local `results/piper_contact_audit` artifacts.

## Remaining blocker and next step

No legitimate grasp was certified; no opening or mobile-base motion was attempted. First preserve the official contact envelope during cooking, or build a validated conservative representation shared by PhysX and the offline checker. Then replace the broad contact-point label with a surface-ownership test that distinguishes pad contact, speculative zero-force contacts and adjacent metal contact throughout closure. Keep the original-mesh guard until cooking fidelity and canonical closure/pull agree. Do not resume grasp-pose optimization to work around this modeling discrepancy.

## Primary references

- [Omni Physics cooking export API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/extensions/runtime/source/omni.physx/docs/api/python.html)
- [PhysX convex hull schema and vertex limit](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/schemas/physxschema.html)
- [PhysX cooking plane-tolerance semantics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/_api_build/structPxCookingParams.html)

## Follow-up: unified ownership

The legacy point-projection contact classifier documented above is superseded
by [PIPER_CONTACT_OWNERSHIP.md](PIPER_CONTACT_OWNERSHIP.md). Native collider
ownership now accepts a canonical interior-box closure and measured 1.524 mm
pull, and rejects the unchanged formal grasp on its first non-pad body contact.
The historical audit data and rejected wider-handle diagnostics are retained.
