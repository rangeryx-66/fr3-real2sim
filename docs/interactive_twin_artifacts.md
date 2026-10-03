# T0 / T1 / T2 articulated twin compiler

`interactive_twin/twin.py` creates independent assets outside the reference asset directory. It reads the initial prepared URDF and association metadata, a saved EE-only articulation estimate, and optional accepted physics-fit results. It never reads simulator joint state and never modifies the reference.

```python
from interactive_twin.twin import write_twins

summary = write_twins(
    asset_root, fresh_output_directory, saved_ee_only_fit, T_world_asset_initial,
    initial_physics_prior={"tau_c": 0.001, "b": 0.002, "J_eff": fixed_prior},
    physics_estimate=fit_resistance_result,
    ee_poses=measured_ee_poses,
    attempt_history=attempt_history,
    q_initial=0.0,
)
```

The supplied numerical physics prior is a caller-selected frozen prior, not a recommended physical parameter. `fit_resistance_result` follows `interactive_twin.sysid`: only `status=IDENTIFIABLE_ON_FROZEN_GRID` with non-null `accepted_parameters` updates T2. Diagnostic best candidates are never promoted when identifiability fails. Profile intervals remain discrete grid support, not confidence intervals or measured hardware friction.

## Coordinate compensation

Replacing an original joint origin with an estimated axis-line point alone would move the entire door. The compiler instead keeps the original joint-frame rotation and replaces its position. It converts the estimated world axis into that joint frame, then computes:

```text
T_parent_new_joint.translation = T_parent_world × estimated_world_axis_point
axis_new_joint = R_parent_new_jointᵀ × R_world_parentᵀ × estimated_world_axis
C = inverse(T_parent_new_joint) × T_parent_old_joint
```

`C` is pure translation. The compiler left-multiplies `C` into every visual, collision and inertial origin on the existing moving child, and every joint originating at that child. All fixed descendants retain their initial physical poses. Link names remain unchanged, `manifest.moving_link` remains the revolute child, and there is no new massless virtual body.

The moving-link coordinate origin changes; its surfaces, COM, mass and inertia do not move. A numerical audit checks every physical element's world transform and unchanged shape/scale/mass/inertia values. It also verifies that reference URDF/manifest bytes remain unchanged. Synthetic tests include nonidentity world/root/parent/joint rotations and inertial orientation, nested fixed descendants, and a deliberately different estimated axis.

## Initial state and limits

The compiler requires `q_initial=0` in the prepared prior's coordinate. Nonzero initial scenes must first be baked into a separate start-at-zero prior during scene preparation, using the authorized initialization configuration. A nonzero argument is rejected rather than querying GT online or silently moving the asset.

The frozen twin operation window is ±10° (`policy_not_physical_limit`). It applies consistently to T0/T1/T2 and avoids using the GT axis to resolve the estimated axis's sign. It is not an estimated full joint limit. `observed_range` is separately calculated from measured EE poses and contains only actual observed motion.

## Physics and native replay

T0 keeps the initial articulation and selected default physics prior. T1 applies the EE-only kinematic estimate and keeps the same physics prior. T2 applies accepted `tau_c/b` or retains T1 physics when unidentifiable. Mass and inertia remain fixed.

URDF `dynamics friction/damping` are serialized, and `twin.json` additionally records native `dynamicFrictionEffort` and `viscousFrictionCoefficient`. The native USD angular schema uses degrees: `viscousFrictionCoefficient = b * pi / 180`, while the model input `b` remains in effective simulator N·m·s/rad. The runner must apply and verify the intended PhysX joint-axis attributes after import; URDF values alone do not prove importer semantics.

The legacy loader can otherwise recompute fixture geometry from the changed axis and sweep. **The replay runner must freeze the initial fixture's pose and dimensions for reference, T0, T1 and T2.** The compiler records this requirement but cannot enforce geometry outside its URDF.

## Artifacts and scope

Each `T0/`, `T1/`, `T2/` contains a native `manifest.json`, `urdf/<asset_id>.urdf`, and `twin.json`. Top-level files contain the normalized estimate, complete updated URDF, structured memory with measured EE observations/attempts, and `twin_versions.json`.

Mesh references are absolute and read-only; the original prepared assets must remain available. Geometry association comes from the initial prepared prior. No new segmentation/shape reconstruction improvement is claimed; `geometry_improvement_metric=null`. GT/evaluation containers are rejected from estimate and physics payloads.

```bash
python scripts/test_interactive_twin_artifacts.py
```
