"""Compile separate T0/T1/T2 twins without modifying the reference asset.

Only an already saved EE-only estimate enters the updated joint definition.
The initial prepared link association is a geometry prior, not an identified
segmentation result. Changing a link's coordinate frame must not move its solid.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from articulated_demo.kinematics import URDFChain, transform

OPERATIONAL_LIMITS_RAD = (-0.1745329252, 0.1745329252)
PARAMETER_SEMANTICS = "effective simulator resistance parameters; not calibrated real hinge torque"


def _numbers(vector):
    return " ".join(format(float(value), ".17g") for value in vector)


def _origin(element):
    if element is None:
        return np.eye(4)
    return transform(np.fromstring(element.get("xyz", "0 0 0"), sep=" "),
                     np.fromstring(element.get("rpy", "0 0 0"), sep=" "))


def _set_origin(element, matrix):
    node = element.find("origin")
    if node is None:
        node = ET.SubElement(element, "origin")
    node.set("xyz", _numbers(matrix[:3, 3]))
    node.set("rpy", _numbers(Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")))


def _matrix(value):
    T = np.asarray(value, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-8)):
        raise ValueError("EXPECTED_SE3_TRANSFORM")
    return T


def _vector(value):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError("EXPECTED_FINITE_3_VECTOR")
    return a


def _reject_truth(value):
    if isinstance(value, dict):
        for key, child in value.items():
            name = str(key).lower()
            if (name.startswith("gt_") or name in {"gt", "evaluation", "ground_truth", "door_angle", "object_joint_q", "source_joint_axis"}
                    or "ground_truth" in name):
                raise ValueError("GROUND_TRUTH_IN_ESTIMATED_PAYLOAD:" + str(key))
            _reject_truth(child)
    elif isinstance(value, (tuple, list)):
        for child in value:
            _reject_truth(child)


def sanitized_estimate(estimate):
    """Whitelist the saved fitter output; never copy its evaluation container."""
    if estimate is None:
        return None
    _reject_truth(estimate)
    if estimate.get("joint_type") not in ("revolute", "prismatic"):
        return None
    kind = estimate["joint_type"]
    model = estimate.get(kind, estimate)
    axis = _vector(model["axis"])
    if np.linalg.norm(axis) < 1e-10:
        raise ValueError("ZERO_ESTIMATED_AXIS")
    axis /= np.linalg.norm(axis)
    point = _vector(model["point_on_axis"])
    result = {"joint_type": kind, "axis_world": axis.tolist(),
              "point_on_axis_world_m": point.tolist(),
              "axis_point_gauge": model.get("axis_point_gauge", "arbitrary point on axis line; axial origin coordinate unidentifiable"),
              "input_source": "saved measured-EE-only articulation estimate"}
    for key in ("confidence", "confidence_definition", "sample_count", "travel_m"):
        if key in estimate:
            result[key] = estimate[key]
    for key in ("radius_m", "position_rmse_m", "rotation_rmse_rad", "angle_span_rad"):
        if key in model:
            result[key] = float(model[key])
    return result


def measured_observed_range(estimate, ee_poses):
    """Measure signed local motion from EE rotation/translation, not object q."""
    if estimate is None or ee_poses is None or len(ee_poses) < 2:
        return {"status": "NO_MEASURED_EE_RANGE", "minimum": None, "maximum": None,
                "full_joint_limits_identified": False}
    poses = np.asarray([_matrix(T) for T in ee_poses])
    axis = np.asarray(estimate["axis_world"])
    if estimate["joint_type"] == "revolute":
        relative = poses[:, :3, :3] @ poses[0, :3, :3].T
        values = Rotation.from_matrix(relative).as_rotvec() @ axis
        units = "rad"
    else:
        values = (poses[:, :3, 3] - poses[0, :3, 3]) @ axis
        units = "m"
    return {"status": "OBSERVED_EE_MOTION", "minimum": float(values.min()),
            "maximum": float(values.max()), "units": units,
            "coordinate_zero": "first supplied measured EE pose",
            "supporting_observations": len(poses), "full_joint_limits_identified": False}


def _asset_urdf(asset_root, manifest):
    proposed = asset_root / "urdf" / (str(manifest["asset_id"]) + ".urdf")
    if proposed.is_file():
        return proposed
    supplied = Path(manifest.get("prepared_urdf", ""))
    if supplied.is_file():
        return supplied.resolve()
    raise ValueError("PREPARED_ASSET_URDF_MISSING")


def _absolute_resources(root, source_urdf):
    for node in list(root.findall(".//geometry/mesh")) + list(root.findall(".//texture")):
        filename = node.get("filename")
        if filename and not "://" in filename:
            node.set("filename", str((source_urdf.parent / filename).resolve()))


def _joint(root, name):
    found = [j for j in root.findall("joint") if j.get("name") == name]
    if len(found) != 1:
        raise ValueError("EXPECTED_ONE_NAMED_JOINT")
    return found[0]


def _set_dynamics(joint, parameters):
    values = {key: float(parameters[key]) for key in ("tau_c", "b")}
    if any(not math.isfinite(v) or v < 0 for v in values.values()):
        raise ValueError("RESISTANCE_PARAMETERS_MUST_BE_FINITE_NONNEGATIVE")
    dynamics = joint.find("dynamics")
    if dynamics is None:
        dynamics = ET.SubElement(joint, "dynamics")
    dynamics.set("friction", str(values["tau_c"]))
    dynamics.set("damping", str(values["b"]))
    return values


def _operational_window(limits):
    """A controller motion budget; never author it as a physical joint stop."""
    lo, hi = map(float, limits)
    if not (math.isfinite(lo) and math.isfinite(hi) and lo <= 0 <= hi and lo < hi):
        raise ValueError("OPERATIONAL_WINDOW_MUST_CONTAIN_INITIAL_ZERO")
    return {"lower": lo, "upper": hi}


def _prior_physical_limits(joint):
    node = joint.find("limit")
    if node is None:
        raise ValueError("INITIAL_PRIOR_REQUIRES_EXISTING_PHYSICAL_LIMITS")
    lo, hi = float(node.get("lower")), float(node.get("upper"))
    if not (math.isfinite(lo) and math.isfinite(hi) and lo <= 0 <= hi and lo < hi):
        raise ValueError("PREPARED_PHYSICAL_LIMITS_MUST_CONTAIN_INITIAL_ZERO")
    return {"lower": lo, "upper": hi}


def _physical_limits_in_estimated_coordinate(root, chain, joint_name, estimate, world, prior):
    """Transfer prepared stop coordinates without replacing the estimated axis.

    A revolute axis is an unoriented line. The prior direction is consulted ONLY
    to choose whether its existing scalar coordinate has the same sign as the
    saved estimated direction. Axis orientation/location still come entirely
    from the EE estimate. Misalignment is retained, not corrected using prior.
    """
    joint = _joint(root, joint_name)
    parent = joint.find("parent").get("link")
    # chain.joints is keyed by child; it still represents the read-only prior.
    prior_joint = chain.joints[joint.find("child").get("link")]
    prior_axis_world = (world @ chain.root_to_link(parent, {}) @ prior_joint.origin)[:3, :3] @ prior_joint.axis
    prior_axis_world = prior_axis_world / np.linalg.norm(prior_axis_world)
    agreement = float(prior_axis_world @ np.asarray(estimate["axis_world"]))
    if abs(agreement) <= 1e-12:
        raise ValueError("ESTIMATED_AXIS_COORDINATE_SIGN_AMBIGUOUS_WITH_PRIOR")
    sign = 1 if agreement > 0 else -1
    limits = dict(prior) if sign == 1 else {"lower": -prior["upper"], "upper": -prior["lower"]}
    if sign == -1:
        node = joint.find("limit")
        node.set("lower", format(limits["lower"], ".17g"))
        node.set("upper", format(limits["upper"], ".17g"))
    return {"limits": limits, "coordinate_sign_vs_prepared_prior": sign,
            "coordinate_relation": "q_twin = sign * q_prepared_prior",
            "axis_direction_dot_prior_diagnostic": agreement,
            "prior_axis_use": "coordinate sign convention only; estimated axis direction and line unchanged",
            "limit_source": "prepared URDF prior, including any initialization re-zero; not interaction-identified"}


def reframe_moving_child(root, chain, joint_name, estimate, T_world_asset):
    """Put the joint on the estimated line without moving any physical solid.

    Keep the original joint-frame rotation and express the estimated axis in it.
    Thus C = inv(new_origin) @ old_origin is a pure translation. Applying C to
    child visual/collision/inertial and child-joint origins retains every initial
    world geometry and inertia pose. No massless virtual body/importer change.
    """
    if estimate["joint_type"] != "revolute":
        raise ValueError("TWIN_COMPILER_CURRENTLY_REQUIRES_REVOLUTE_ESTIMATE")
    joint = _joint(root, joint_name)
    if joint.get("type") != "revolute":
        raise ValueError("TWIN_COMPILER_REQUIRES_SINGLE_REVOLUTE_PREPARED_ASSET")
    parent = joint.find("parent").get("link")
    child = joint.find("child").get("link")
    old = _origin(joint.find("origin"))
    W_parent = T_world_asset @ chain.root_to_link(parent, {})
    point_parent = (np.linalg.inv(W_parent) @ np.r_[estimate["point_on_axis_world_m"], 1.])[:3]
    new = old.copy()
    new[:3, 3] = point_parent
    axis_joint = new[:3, :3].T @ W_parent[:3, :3].T @ np.asarray(estimate["axis_world"])
    compensation = np.linalg.inv(new) @ old
    if not np.allclose(compensation[:3, :3], np.eye(3), atol=1e-10):
        raise AssertionError("REFRAME_MUST_NOT_ROTATE_BODY_FRAME")
    child_node = next(link for link in root.findall("link") if link.get("name") == child)
    updated = []
    for kind in ("visual", "collision", "inertial"):
        for index, element in enumerate(child_node.findall(kind)):
            _set_origin(element, compensation @ _origin(element.find("origin")))
            updated.append(f"{child}/{kind}/{index}")
    for descendant_joint in root.findall("joint"):
        if descendant_joint.find("parent").get("link") == child:
            _set_origin(descendant_joint, compensation @ _origin(descendant_joint.find("origin")))
            updated.append("joint/" + descendant_joint.get("name"))
    _set_origin(joint, new)
    axis_node = joint.find("axis")
    if axis_node is None:
        axis_node = ET.SubElement(joint, "axis")
    axis_node.set("xyz", _numbers(axis_joint))
    return {"method": "child-frame compensation; unchanged original link names",
            "moving_link": child, "parent_link": parent,
            "new_origin_parent": new.tolist(), "axis_joint_local": axis_joint.tolist(),
            "child_frame_compensation": compensation.tolist(), "compensated_elements": updated,
            "physical_geometry_moved": False,
            "note": "Moving-link coordinate origin changes; its initial physical geometry, COM and inertia do not."}


def _physical_frames(urdf, T_world_asset):
    chain = URDFChain(urdf)
    root = ET.parse(urdf).getroot()
    frames, properties = {}, {}
    for link in root.findall("link"):
        name = link.get("name")
        W_link = T_world_asset @ chain.root_to_link(name, {})
        for kind in ("visual", "collision", "inertial"):
            for index, element in enumerate(link.findall(kind)):
                key = f"{name}/{kind}/{index}"
                frames[key] = W_link @ _origin(element.find("origin"))
                if kind == "inertial":
                    properties[key] = {"mass": copy.deepcopy(element.find("mass").attrib),
                                       "inertia": copy.deepcopy(element.find("inertia").attrib)}
                else:
                    geometry = element.find("geometry")
                    properties[key] = ET.tostring(geometry, encoding="unicode").strip()
    return frames, properties


def audit_initial_geometry(initial_urdf, updated_urdf, T_world_asset):
    """Check all physical frames and unchanged shape/scale/mass/inertia values."""
    before, bp = _physical_frames(initial_urdf, T_world_asset)
    after, ap = _physical_frames(updated_urdf, T_world_asset)
    if set(before) != set(after) or bp != ap:
        raise AssertionError("TWIN_MODIFIED_GEOMETRY_OR_MASS_PROPERTIES")
    translation = max((float(np.linalg.norm(before[k][:3, 3] - after[k][:3, 3])) for k in before), default=0.)
    rotation = max((float(Rotation.from_matrix(before[k][:3, :3].T @ after[k][:3, :3]).magnitude()) for k in before), default=0.)
    if translation > 1e-9 or rotation > 1e-9:
        raise AssertionError(f"TWIN_MOVED_INITIAL_GEOMETRY:{translation}:{rotation}")
    return {"physical_element_count": len(before), "max_translation_error_m": translation,
            "max_rotation_error_rad": rotation, "geometry_scale_shape_unchanged": True,
            "mass_and_inertia_values_unchanged": True,
            "scope": "URDF physical element transforms and properties; not a new mesh-fidelity claim"}


def _physics_result(physics_estimate, prior):
    if physics_estimate is None:
        return {"status": "NOT_ESTIMATED", "updated": False, "parameters": prior,
                "parameter_intervals": None, "parameter_semantics": PARAMETER_SEMANTICS}
    _reject_truth(physics_estimate)
    status = str(physics_estimate.get("status", "NOT_ESTIMATED"))
    accepted = physics_estimate.get("accepted_parameters")
    updated = status == "IDENTIFIABLE_ON_FROZEN_GRID" and accepted is not None
    parameters = {key: float(accepted[key]) for key in ("tau_c", "b")} if updated else prior
    result = {"status": status, "updated": updated, "parameters": parameters,
              "parameter_intervals": physics_estimate.get("parameter_intervals"),
              "interval_semantics": physics_estimate.get("interval_semantics"),
              "parameter_semantics": PARAMETER_SEMANTICS,
              "J_eff_prior_fixed": physics_estimate.get("J_eff_prior_fixed"),
              "current_or_effort_used": bool(physics_estimate.get("current_or_effort_used", False))}
    if not updated:
        result["reason"] = "No accepted identifiable parameters; T2 retains T1 physics rather than promoting a diagnostic best candidate."
    return result


def write_twins(asset_root, output_dir, estimated_articulation, T_world_asset, *,
                initial_physics_prior=None, physics_estimate=None, ee_poses=None,
                q_initial=0.0, operational_joint_limits_rad=OPERATIONAL_LIMITS_RAD,
                attempt_history=None):
    """Save three native-loadable assets, their JSON models, and measured range.

    ``q_initial`` must be zero in the *prepared prior's coordinate*. A nonzero
    scene configuration must be baked into a separate start-at-zero prior during
    preparation, never inferred by querying GT while fitting. No source writes.
    All outputs must be fresh directories so prior results cannot be overwritten.
    """
    if abs(float(q_initial)) > 1e-12:
        raise ValueError("NONZERO_INITIAL_STATE_REQUIRES_PREPARATION_TIME_GEOMETRY_BAKE")
    source = Path(asset_root).resolve()
    output = Path(output_dir).resolve()
    if output == source or source in output.parents:
        raise ValueError("TWIN_OUTPUT_MUST_BE_OUTSIDE_REFERENCE_ASSET")
    if output.exists() and any(output.iterdir()):
        raise ValueError("TWIN_OUTPUT_EXISTS_PRESERVE_PREVIOUS_RESULTS")
    W_asset = _matrix(T_world_asset)
    source_manifest = source / "manifest.json"
    manifest_bytes = source_manifest.read_bytes()
    manifest = json.loads(manifest_bytes)
    source_urdf = _asset_urdf(source, manifest)
    source_bytes = source_urdf.read_bytes()
    root = ET.fromstring(source_bytes)
    movable = [j for j in root.findall("joint") if j.get("type") != "fixed"]
    if len(movable) != 1 or movable[0].get("type") != "revolute":
        raise ValueError("TWIN_COMPILER_REQUIRES_SINGLE_REVOLUTE_PREPARED_ASSET")
    name = manifest["joint_name"]
    joint = _joint(root, name)
    child = joint.find("child").get("link")
    if child != manifest["moving_link"]:
        raise ValueError("PREPARED_MOVING_LINK_MUST_BE_REVOLUTE_CHILD")
    estimate = sanitized_estimate(estimated_articulation)
    if estimate and estimate["joint_type"] != "revolute":
        raise ValueError("NONREVOLUTE_ESTIMATE_NOT_APPLICABLE_TO_REVOLUTE_TWIN")
    dynamics = joint.find("dynamics")
    prior = {"tau_c": float(dynamics.get("friction", 0.)) if dynamics is not None else 0.,
             "b": float(dynamics.get("damping", 0.)) if dynamics is not None else 0.}
    if initial_physics_prior is not None:
        _reject_truth(initial_physics_prior)
        prior.update({key: float(initial_physics_prior[key]) for key in ("tau_c", "b")})
    prior = _set_dynamics(joint, prior)
    fixed_inertia_prior = None if initial_physics_prior is None else initial_physics_prior.get("J_eff")
    limits = _operational_window(operational_joint_limits_rad)
    physical_prior = _prior_physical_limits(joint)
    _absolute_resources(root, source_urdf)
    initial_root = copy.deepcopy(root)
    updated_root = copy.deepcopy(root)
    reframe = None
    updated_limits = {"limits": physical_prior, "coordinate_sign_vs_prepared_prior": 1,
                      "coordinate_relation": "q_twin = q_prepared_prior",
                      "prior_axis_use": "unchanged prepared coordinate; no accepted estimate"}
    if estimate:
        chain = URDFChain(source_urdf)
        reframe = reframe_moving_child(updated_root, chain, name, estimate, W_asset)
        updated_limits = _physical_limits_in_estimated_coordinate(updated_root, chain, name, estimate, W_asset, physical_prior)
    physics = _physics_result(physics_estimate, prior)
    if physics["updated"] and estimate is None:
        raise ValueError("PHYSICS_UPDATE_REQUIRES_ACCEPTED_KINEMATIC_ESTIMATE")
    observed = measured_observed_range(estimate, ee_poses)
    if attempt_history is not None:
        _reject_truth(attempt_history)
    relation = {"source": "initial prepared association; not interaction-identified segmentation",
                "moving_link": child, "moving_links": manifest.get("moving_links", [child]),
                "static_links": sorted({link.get("name") for link in root.findall("link")}
                                       - set(manifest.get("moving_links", [child]))),
                "static_parent_link": joint.find("parent").get("link"),
                "handle_link": manifest.get("door_link", child),
                "handle_relation": "fixed descendant association retained from prepared geometry",
                "support_relation": "same frozen initial fixture/layout in every reference and twin replay",
                "geometry_improvement_measured": False, "geometry_improvement_metric": None}
    output.mkdir(parents=True, exist_ok=True)
    versions = {}
    for version in ("T0", "T1", "T2"):
        directory = output / version
        (directory / "urdf").mkdir(parents=True)
        robot = copy.deepcopy(initial_root if version == "T0" else updated_root)
        selected_physics = prior if version != "T2" else physics["parameters"]
        physical_limits = physical_prior if version == "T0" else updated_limits["limits"]
        coordinate_sign = 1 if version == "T0" else updated_limits["coordinate_sign_vs_prepared_prior"]
        _set_dynamics(_joint(robot, name), selected_physics)
        urdf = directory / "urdf" / source_urdf.name
        ET.indent(robot)
        ET.ElementTree(robot).write(urdf, encoding="utf-8", xml_declaration=True)
        twin = {"schema": "interactive-articulated-twin-v1", "version": version,
                "asset_id": manifest["asset_id"], "joint_name": name,
                "T_world_asset_initial": W_asset.tolist(), "initial_joint_coordinate": 0.,
                "kinematics_updated": version != "T0" and estimate is not None,
                "physics_updated": version == "T2" and physics["updated"],
                "estimated_articulation": estimate if version != "T0" else None,
                "observed_range": observed if version != "T0" else None,
                "operational_joint_window": {**limits, "units": "rad", "source": "frozen policy_not_physical_limit", "full_joint_limits_identified": False, "authored_to_physical_joint": False},
                "physical_joint_limits": {**physical_limits, "units": "rad", "source": "prepared URDF prior; not estimated from observed motion", "prepared_prior_limits": physical_prior, "coordinate_sign_vs_prepared_prior": coordinate_sign, "full_joint_limits_identified": False},
                "joint_coordinate_convention": {"coordinate_sign_vs_prepared_prior": 1, "prior_axis_use": "unchanged prepared coordinate"} if version == "T0" else updated_limits,
                "physics": physics if version == "T2" else {"status": "INITIAL_PRIOR", "updated": False, "parameters": prior, "J_eff_prior_fixed": fixed_inertia_prior, "parameter_semantics": PARAMETER_SEMANTICS},
                "native_physics_application": {"dynamicFrictionEffort": selected_physics["tau_c"],
                    "viscousFrictionCoefficient": selected_physics["b"] * math.pi / 180.,
                    "viscousFrictionCoefficient_units": "N m s/degree in native USD angular axis",
                    "b_input_units": "effective simulator N m s/rad",
                    "note": "Apply/verify native PhysxJointAxisAPI attributes after import; URDF dynamics alone is not proof of equivalent PhysX resistance."},
                "geometry_relations": relation, "reframe": reframe if version != "T0" else None,
                "geometry_source_urdf_sha256": hashlib.sha256(source_bytes).hexdigest(),
                "reference_modified": False,
                "fixture_must_be_frozen_from_initial_scene": True,
                "resource_policy": "read-only absolute references; source assets must remain available"}
        meta = copy.deepcopy(manifest)
        # Legacy loader fields remain actual physical prior stops in this
        # twin's coordinate. The controller operation window is metadata only.
        axis_element = _joint(robot, name).find("axis")
        meta.update(prepared_urdf=str(urdf), prepared_geometry_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                    source_joint_limits_rad=physical_limits, source_joint_axis="1 0 0" if axis_element is None else axis_element.get("xyz", "1 0 0"))
        meta["interactive_twin"] = {"version": version, "twin_metadata": str(directory / "twin.json"),
            "axis_source": "EE-only estimate" if twin["kinematics_updated"] else "initial prepared prior",
            "legacy_source_joint_limits_field_semantics": "prepared physical prior limits in twin coordinate; not interaction-identified",
            "coordinate_sign_vs_prepared_prior": coordinate_sign,
            "geometry_changed": False, "coordinate_frame_reexpressed": bool(twin["kinematics_updated"]),
            "reference_modified": False, "freeze_fixture_layout_from_initial_scene": True}
        (directory / "manifest.json").write_text(json.dumps(meta, indent=2) + "\n")
        versions[version] = {"asset_root": str(directory), "urdf": str(urdf), "metadata": twin}
    initial = versions["T0"]["urdf"]
    for version, item in versions.items():
        item["metadata"]["initial_geometry_audit"] = audit_initial_geometry(initial, item["urdf"], W_asset)
        (Path(item["asset_root"]) / "twin.json").write_text(json.dumps(item["metadata"], indent=2) + "\n")
    if source_manifest.read_bytes() != manifest_bytes or source_urdf.read_bytes() != source_bytes:
        raise AssertionError("REFERENCE_MODIFIED_DURING_TWIN_COMPILATION")
    if estimate:
        (output / "estimated_articulation.json").write_text(json.dumps(estimate, indent=2) + "\n")
        # A complete articulated model, not an unconnected axis-only sketch.
        (output / "estimated_articulation.urdf").write_bytes(Path(versions["T1"]["urdf"]).read_bytes())
    (output / "structured_memory.json").write_text(json.dumps({"estimated_articulation": estimate,
        "observed_range": observed, "geometry_relations": relation,
        "physics": physics, "supporting_ee_observation_count": 0 if ee_poses is None else len(ee_poses),
        "supporting_ee_observations": [] if ee_poses is None else np.asarray(ee_poses).tolist(),
        "attempt_history": [] if attempt_history is None else attempt_history,
        "full_joint_limits_identified": False}, indent=2) + "\n")
    summary = {"schema": "interactive-twin-versions-v1", "asset_id": manifest["asset_id"],
        "versions": {key: {"asset_root": value["asset_root"], "urdf": value["urdf"],
                            "kinematics_updated": value["metadata"]["kinematics_updated"],
                            "physics_updated": value["metadata"]["physics_updated"]} for key, value in versions.items()},
        "observed_range": observed, "geometry_improvement_claimed": False,
        "reference_urdf_unchanged": True, "reference_manifest_unchanged": True}
    (output / "twin_versions.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary
