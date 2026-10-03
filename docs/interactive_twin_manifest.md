# Frozen multi-object benchmark inventory

This is a new experiment layer. It does not modify the successful physical-contact
or Act2See-style runners. Asset selection is completed before TEST outcomes exist.

## Inputs and preparation

`interactive_twin.prepare` imports the existing PhysX-Mobility dataset using the
existing single-revolute preparation implementation:

```bash
python -m interactive_twin.prepare \
  --source /data1/home/rangeryx/datasets/physx_mobility/extracted/PhysX_mobility \
  --output results/interactive_twin_asset_preparation --rank
```

The height is `finaljson.dimension[2] / 100` metres. The source stores dimensions
as centimetres, for example `60*40*90`. This is a source scale prior, not a
per-object grasp-fit adjustment. Missing or invalid dimensions cause a recorded
exclusion. Existing assets are never overwritten. Source masses, joint types,
axes, limits, dynamics and link associations are checked against the prepared
URDF; joint-origin translations undergo the documented metric scaling. The
existing preparation's box-inertia approximation is recorded explicitly and is
not called measured inertia. A new object contact proxy is generated separately
using the existing generic semantic bar method.

`preparation_inventory.json` is written after every asset, including timeouts and
failures. The old geometric ranker is reused through an isolated temporary
symlink per asset so a malformed mesh does not discard the whole batch.

## Frozen selection

```bash
python -m interactive_twin.manifest \
  --prepared results/interactive_twin_asset_preparation/assets \
  --prepared results/semantic_interaction/asset \
  --ranking results/interactive_twin_asset_preparation/ranking.json \
  --source /data1/home/rangeryx/datasets/physx_mobility/extracted/PhysX_mobility \
  --output results/interactive_twin/frozen_benchmark_manifest.json
```

Prepared manifests are found recursively. Existing semantic-proxy manifests can
provide handle observations directly; ordinary prepared assets need a ranking
JSON or `--scan-geometry`. Selection uses only prepared visual geometry and
source joint metadata, not execution reports:

- DEV asset `7320` is excluded from every unseen TEST denominator.
- At least four distinct revolute TEST IDs are required. Multiple preparations
  of the same ID are never counted as different objects.
- Supported bar geometry: length at least 25.43 mm, width below 100 mm,
  aspect ratio at least 1.6, visual rear clearance at least 5 mm.
- Score: `100 * min(clearance, 0.05) + min(length, 0.15)`.
- Category/object-name groups are visited in lexical round-robin order. Each
  group is ordered by decreasing geometry score then asset ID. This promotes
  category diversity without using execution success.
- Prepared semantic-proxy variants take precedence over ordinary preparations
  of the same ID, then geometry score and path determine the canonical variant.
- Every rejection, duplicate and eligible-but-unselected record stays in the
  inventory. If fewer than four suitable IDs exist, the CLI saves the manifest,
  returns exit code 2 and reports `selection_complete=false`.

Geometry screening does **not** certify a reachable or stable physical grasp.
The old ranker's raw/cooked ownership acceptance text is not propagated. Raw
triangle intersections remain diagnostic, and existing physical-contact safety
checks determine actual execution success or failure.

## Three initial configurations per asset

Seed `732061660` is expanded into a deterministic per-asset/configuration seed.
Configuration zero is the nominal deployment. Configurations one and two use:

- articulation initialization offsets 1° and 2° from the source closed state;
- along-handle offsets +10 mm and −10 mm, clipped only to remain inside the
  visually supported grasp interval;
- bounded initial handle-frame X/Y placement perturbation ±10 mm;
- bounded object yaw perturbation ±3°; robot base remains fixed.

The common placement rule aligns the observed handle anchor/frame to the DEV
nominal workspace anchor, followed by these perturbations. Resolving that scene
anchor is the environment initializer's job. An unresolved anchor is explicitly
marked, not silently replaced with a per-object world offset. Source articulation
limits are inspected only during episode initialization; an invalid requested
initial state is retained as a preflight failure.

Each episode freezes 12 grasp candidates and 4 probe directions. The standalone
manifest defaults include 24 system-identification simulations and 1800 seconds;
the integrated benchmark configuration explicitly overrides these with **48
system-identification simulations and 25200 active seconds shared across all
three configurations of an asset**, plus a 16-rollout cap per configuration. The
main entry enforces the shared ledger and global 05:00 cutoff. Read the frozen
configuration and ledger for the actual run budget. The initial candidate
family is bar-side-pinch: three small along-handle offsets, two depth offsets and
two symmetric finger orientations, for 12 candidates total. Safety margin stays
strictly above 0.05 rad. The actual executor can stop sooner on safety grounds.

## API and data boundary

```python
from interactive_twin.manifest import (
    build_manifest, save_frozen_manifest, verify_frozen_manifest,
    controller_episode, geometry_rankings,
)

manifest = build_manifest(
    prepared_roots,
    ranking_paths=[ranking_path],
    source_roots=[source_dataset],
    policy=policy_overrides,
    frozen_algorithm_files=[config_path, controller_path, fitter_path],
)
save_frozen_manifest(manifest, output_path)
verify_frozen_manifest(output_path, verify_inputs=True)
visual_setup = controller_episode(manifest, "test_<asset_id>_00")
```

`controller_episode` exposes only initial visual handle geometry, deployment
perturbations, candidate policy and budgets. It omits source joint name/type,
limits and initial articulation state. `initialization_only` belongs to the
separate simulation setup process and must not be forwarded to online control.
The controller also must not open the prepared URDF to recover hidden joints.

A canonical JSON SHA-256 protects the full manifest. Source prepared manifests,
URDFs, referenced mesh files, ranking inputs and supplied algorithm/config files
are separately hashed. `verify_inputs=True` detects changes. Saving a different
manifest over an existing frozen path is refused; a changed method requires a
new version and full TEST rerun. Episode results are separate files, never edits
to the selection manifest. Failed selected episodes retain their denominator.

## Integrity checks

```bash
python scripts/test_interactive_twin_manifest.py
```

The metadata-only test fixtures verify selection independence from outcomes,
category ordering, complete failure inventory, three distinct configurations,
GT-field exclusion, immutable hashing, input tamper detection, refusal to replace
a frozen selection, budget limits and source dimension parsing. They are not
synthetic experiment objects and are never counted as benchmark results.

## Generic placement and grasp planning

`interactive_twin.planning.make_deployment_template(dev_asset_root,
source_report)` freezes the DEV initial visual handle anchor and outward normal.
`prepare_deployment(episode, asset_root, template, output)` aligns an unseen
handle's outward yaw and anchor X/Y to that template, then applies the frozen
handle-frame placement perturbations. The support height is floored at zero.
A physically tall object keeps its dimensions and high handle, even if this
subsequently makes it unreachable. Robot base pose is unchanged. Initial
articulation transformation occurs once in scene setup; its joint metadata is
not included in `initial_visual_handle_world.json`.

`plan_grasps(robot_only_model, physical_scene, moving_initial, initial_visual,
base, seed=episode_seed, budget=12, output=plan_path)` uses exactly the manifest
candidate grid. It checks exact IK, strict joint margin, cooked geometry with
open fingers, the complete approach (including interpolated joint edges), and a
home-to-pregrasp joint plan with the existing 1500-iteration budget. It never uses
the old ownership scene. Plans are explicitly `PENDING_REAL_CLOSURE`: only actual
simulation closure can establish legal and stable physical grasp.

The optional `stop_after_first` changes only whether already ordered candidates
are all planned; it cannot exceed the frozen candidate budget. `wall_clock_s`
bounds planning and each rejection remains in the plan JSON.

```bash
python scripts/test_interactive_twin_planning.py
```

This test requires NumPy/SciPy and verifies fixture-floor behavior, fixed base,
visual alignment, one-time initial-state setup, candidate bounds and collision
checks using a lightweight robot fixture. It does not require Isaac or FCL and
does not claim physical execution.

## Nonzero initial configuration and twin coordinates

The reference retains its original asset and configured 0°/1°/2° initial state.
Before compiling a twin, use the independent setup helper:

```python
from interactive_twin.initial_state import bake_initial_articulation
prepared = bake_initial_articulation(reference_asset_root, zero_frame_directory,
                                     original_initial_articulation_rad)
# Feed prepared['asset_root'] to write_twins(..., q_initial=0.).
# T0/T1/T2 replay starts at 0. Reference and Oracle keep the original initial q.
```

For a revolute joint, the new origin is `old_origin @ R(axis, q_initial)` and
limits are shifted by `-q_initial`. The relation is
`q_original = q_initial + q_zero_frame`. Shapes, scales, original inertial
properties, friction and force limits are not changed. An audit compares all
physical element frames at three corresponding coordinates. The source files
are checked unchanged. Identical inputs reuse a hash-verified copy; mismatched
inputs cannot overwrite it.

The existing loader separately overrides moving-body mass properties with a
uniform-box approximation. Recomputing that approximation from the old closed
bounds after rotating a joint frame would incorrectly move the COM. The new
helper therefore records the **effective original loader mass state**, applies
the configured initial rigid transformation once, and stores invariant root-frame
COM and inertia tensors. Every T0/T1/T2 loader must run, after the legacy mass
override and before the first `world.reset()`:

```python
from interactive_twin.initial_state import restore_baked_mass_properties
baked_mass_audit = restore_baked_mass_properties(
    stage, asset_path, asset_urdf, manifest)
```

This converts those invariant properties into each compiled link's coordinates.
It is a no-op for unbaked reference assets. It does not change a body's physical
mass or inertia, command a joint, or apply an object force. The bake and mass-state
metadata are private scene initialization records, not controller or estimator
inputs. The supported mass model is explicitly the existing `geometry` mode.

```bash
python scripts/test_interactive_twin_initial_state.py
```

The test checks initial/motion-frame equivalence, limits, unchanged source bytes,
COM and full inertia tensor preservation across all three twins, and detects the
incorrect legacy recomputation that the setup hook corrects. Native import should
also retain the returned mass audit before running physics.

## Descriptive metadata refresh before freeze

If an older preparation copied the category into `object_name`, fix only these
two descriptive fields from the unchanged source metadata:

```bash
python -m interactive_twin.prepare --source SOURCE_DATASET \
  --output NEW_AUDIT_DIRECTORY --refresh-metadata-only PREPARED_ROOT
```

URDF/mesh/physics files and all other manifest fields are left unchanged. Run this
before selecting/freezing TEST, because category participates in selection. A
frozen benchmark requires a new version if its source metadata changes.
