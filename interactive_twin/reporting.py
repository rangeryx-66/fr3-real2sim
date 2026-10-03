"""Outcome reporting with a frozen denominator and explicit missing evidence."""
from __future__ import annotations

from collections import Counter, defaultdict
import csv
import json
from pathlib import Path

import numpy as np

from interactive_twin.manifest import verify_frozen_manifest
from interactive_twin.sysid import validate_log


STAGES = {
    "A": "perception_interaction_geometry", "B": "grasp", "C": "unknown_articulation_probe",
    "D": "kinematic_identification", "E": "physics_probe", "F": "physics_identification",
    "G": "twin_update", "H": "heldout_prediction", "I": "final_manipulation",
}
PASS = {"PASS", "PASSED", "SUCCESS", "COMPLETE", "COMPLETED", "EVALUATED", "IDENTIFIED",
        "ACCEPTED", "IDENTIFIABLE_ON_FROZEN_GRID", "PHYSICS_PROTOCOL_COMPLETE"}
UNASSESSSED = {"NOT_RUN", "PENDING", "RUNNING", "UNAVAILABLE", "NOT_APPLICABLE"}


def _load(path):
    path = Path(path)
    return json.loads(path.read_text()) if path.is_file() else None


def _write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _stage(value="NOT_RUN", **fields):
    if isinstance(value, dict):
        return {"status": "NOT_RUN", **value, **fields}
    return {"status": str(value), **fields}


def _status(stage):
    return str(stage.get("status", "NOT_RUN")).upper()


def _is_pass(stage):
    return _status(stage) in PASS


def _interaction_pass(stage):
    return _is_pass(stage) or _status(stage) == "KINEMATIC_ONLY_SUCCESS"


def _is_evaluated(stage):
    status = _status(stage)
    return status not in UNASSESSSED and not status.startswith("BLOCKED_")


def _normalize_stages(stages):
    result = {key: _stage() for key in STAGES}
    aliases = {value: key for key, value in STAGES.items()}
    aliases.update({"perception": "A", "interaction_geometry": "A", "kinematics": "D",
                    "physics_fit": "F", "heldout": "H", "manipulation": "I"})
    for key, value in (stages or {}).items():
        label = str(key)
        stage_key = label if label in STAGES else aliases.get(label.lower())
        if stage_key is None and label[:1].upper() in STAGES and label[1:2] in ("_", ":", " "):
            stage_key = label[:1].upper()
        if stage_key:
            result[stage_key] = _stage(value)
    return result


def _find_file(directory, name, summary):
    explicit = summary.get(name.replace(".json", "") + "_path")
    if explicit:
        path = Path(explicit)
        return path if path.is_absolute() else directory / path
    options = [directory / name, directory / "reference" / name]
    return next((p for p in options if p.is_file()), options[0])


def _raw_report(directory, summary):
    if summary.get("selected_report"):
        path = Path(summary["selected_report"])
        if not path.is_absolute():
            path = directory / path
        return _load(path), path, None
    if summary.get("selected_job", {}).get("output"):
        path = Path(summary["selected_job"]["output"]) / "report.json"
        if not path.is_absolute():
            path = directory / path
        return _load(path), path, None
    direct = directory / "report.json"
    reference = directory / "reference" / "report.json"
    if direct.is_file():
        return _load(direct), direct, None
    if reference.is_file():
        return _load(reference), reference, None
    # Never select the best among repeats/candidates based on outcome.
    paths = sorted(directory.glob("**/report.json")) if directory.exists() else []
    if len(paths) == 1:
        return _load(paths[0]), paths[0], None
    return None, None, "AMBIGUOUS_REPORT_SELECTION" if paths else None


def _fallback_stages(raw, manifest_episode, fit, heldout, twin_files):
    result = {key: _stage() for key in STAGES}
    exclusion = manifest_episode.get("preflight_exclusion_reason")
    if exclusion:
        result["A"] = _stage("FAILED", reason=exclusion)
    if not raw:
        return result
    status = str(raw.get("status", "UNKNOWN"))
    result["A"] = _stage("COMPLETE", evidence="Isaac episode report exists")
    legal = raw.get("bilateral_hold_established", raw.get("legal_bilateral", False)) is True
    result["B"] = _stage("SUCCESS" if legal else "FAILED", reason=None if legal else status)
    estimate = raw.get("estimate") or {}
    probe = raw.get("probe") or {}
    travel = probe.get("distance_m", probe.get("measured_ee_travel_m"))
    identified = (estimate.get("joint_type") in {"revolute", "prismatic"} and
                  (raw.get("accepted_estimate_count", 0) > 0 or estimate.get("accepted") is True))
    if legal:
        result["C"] = _stage("COMPLETE" if travel is not None and travel >= .005 else "UNOBSERVABLE",
                             measured_ee_travel_m=travel)
        result["D"] = _stage("IDENTIFIED" if identified else "UNOBSERVABLE",
                             joint_type=estimate.get("joint_type", "UNOBSERVABLE"))
    if raw.get("physics_protocol_complete") is True or status == "PHYSICS_PROTOCOL_COMPLETE":
        result["E"] = _stage("PHYSICS_PROTOCOL_COMPLETE")
    if fit:
        result["F"] = _stage(fit.get("status", "UNKNOWN"),
                             accepted_parameters=fit.get("accepted_parameters"))
    if twin_files.get("T2") and fit and fit.get("accepted_parameters") is not None:
        result["G"] = _stage("COMPLETE")
    elif twin_files.get("T1"):
        result["G"] = _stage("KINEMATICS_ONLY", reason="Physics update not accepted or not executed")
    if heldout:
        methods = heldout.get("methods", {})
        all_core = all(methods.get(name, {}).get("status") == "EVALUATED" for name in ("B0", "B1", "B2"))
        result["H"] = _stage("EVALUATED" if all_core else "PARTIAL", reason=None if all_core else "Missing B0/B1/B2 independent held-out rollouts")
    if raw.get("success") is True and status == "SUCCESS":
        result["I"] = _stage("SUCCESS")
    elif identified and raw.get("followed_estimated_articulation") is True and status != "PHYSICS_PROTOCOL_COMPLETE":
        result["I"] = _stage("FAILED", reason=status)
    return result


def _first_failure(stages):
    for key, value in stages.items():
        if _is_evaluated(value) and not _is_pass(value) and _status(value) not in {"PARTIAL", "KINEMATICS_ONLY", "KINEMATIC_ONLY_SUCCESS"}:
            return key, value.get("reason") or value["status"]
    return None, None


def _episode(manifest_episode, root):
    directory = root / manifest_episode["episode_id"]
    summary = _load(directory / "episode_summary.json") or {}
    raw, raw_path, ambiguity = _raw_report(directory, summary)
    fit_path = _find_file(directory, "physics_fit.json", summary)
    heldout_path = _find_file(directory, "heldout_comparison.json", summary)
    fit, heldout = _load(fit_path), _load(heldout_path)
    twin_files = {}
    declared_versions = (summary.get("stages", {}).get("G", {}).get("versions") or
                         summary.get("kinematic_prior_twins") or {})
    for name in ("T0", "T1", "T2"):
        declaration = declared_versions.get(name, {})
        declared_root = declaration.get("asset_root") if isinstance(declaration, dict) else None
        if declared_root:
            path = Path(declared_root) / "twin.json"
            paths = [path if path.is_absolute() else directory / path]
        else:
            paths = [directory / "twins_final" / name / "twin.json",
                     directory / "kinematic_update" / "twins" / name / "twin.json",
                     directory / "twins" / name / "twin.json", directory / "twins" / (name + ".json"),
                     directory / (name + ".json"), directory / name / "twin.json"]
        path = next((p for p in paths if p.is_file()), None)
        twin_files[name] = str(path) if path else None
    stages = _fallback_stages(raw, manifest_episode, fit, heldout, twin_files)
    if summary.get("stages"):
        explicit = _normalize_stages(summary["stages"])
        # The wrapper is the authority for its actual stage attempts. Do not
        # invent a completed stage from an artifact leftover in the directory.
        stages = explicit
    if ambiguity:
        stages["A"] = _stage("BLOCKED_REPORT_AMBIGUITY", reason=ambiguity)
    first_stage, reason = _first_failure(stages)
    if first_stage:
        for key in STAGES:
            if key > first_stage and _status(stages[key]) == "NOT_RUN":
                stages[key] = _stage("BLOCKED_" + first_stage, reason=reason)
    raw = raw or {}
    # Identification evidence belongs to the selected initial trial. Final action
    # metrics/video belong to stage I's explicitly identified execution report.
    # A missing final report must never borrow an earlier successful trial.
    final_declared = stages["I"].get("report") or summary.get("final_report")
    interaction_raw, interaction_path = raw, raw_path
    final_missing = False
    if final_declared:
        interaction_path = Path(final_declared)
        if not interaction_path.is_absolute():
            interaction_path = directory / interaction_path
        interaction_raw = _load(interaction_path) or {}
        final_missing = not interaction_path.is_file()
    elif _interaction_pass(stages["I"]) and (directory / "final_updated_interaction/report.json").is_file():
        interaction_path = directory / "final_updated_interaction/report.json"
        interaction_raw = _load(interaction_path) or {}
    evaluation = stages["D"].get("evaluation") or raw.get("evaluation") or summary.get("evaluation") or {}
    interaction_evaluation = interaction_raw.get("evaluation") or (summary.get("evaluation") if not final_declared else {}) or {}
    displacement = interaction_evaluation.get("actual_door_displacement_deg", interaction_evaluation.get("actual_articulation_displacement_deg"))
    initial = manifest_episode.get("initialization_only", {}).get("joint_position_rad")
    if displacement is None and initial is not None and interaction_evaluation.get("actual_final_door_angle_deg") is not None:
        displacement = float(interaction_evaluation["actual_final_door_angle_deg"] - np.rad2deg(initial))
    estimate = stages["D"].get("estimate") or raw.get("estimate") or summary.get("estimate") or {}
    joint_type = estimate.get("joint_type", "UNOBSERVABLE")
    model = estimate.get(joint_type, {})
    video = interaction_raw.get("video")
    if video and interaction_path:
        video_path = Path(video)
        if not video_path.is_absolute():
            video_path = interaction_path.parent / video_path
        video = str(video_path) if video_path.is_file() else None
    final_success_evidence = (not final_missing and interaction_raw.get("success") is True and
                              interaction_raw.get("bilateral_hold_established") is True and
                              displacement is not None and displacement >= 5.)
    heldout_improvement = None
    if heldout and heldout.get("B2_improvement", {}).get("B1"):
        heldout_improvement = heldout["B2_improvement"]["B1"].get("relative_loss_reduction")
    physical_ran = any(bool(r.get("duration_s") is not None or r.get("first_failure_state") or
                            r.get("bilateral_hold_established") is not None) for r in (raw, interaction_raw))
    reasons = [("selected_report", raw.get("status", "")), ("interaction_report", interaction_raw.get("status", ""))]
    reasons += [("stage_" + key, stage.get("reason", stage.get("status", ""))) for key, stage in stages.items()]
    reasons += [("candidate_attempt", attempt.get("status", "")) for attempt in summary.get("candidate_attempts", [])]
    safety_tokens = ("COLLISION", "JOINT_MARGIN", "CONTACT_LOSS", "SLIP", "FORCE_LIMIT", "LOAD_LIMIT",
                     "OVERSPEED", "SPEED_LIMIT", "EFFORT_LIMIT")
    safety_events = [{"source": source, "reason": str(value)} for source, value in reasons
                     if any(token in str(value).upper() for token in safety_tokens)]
    safety_stop = bool(safety_events)
    physics_accepted = bool(fit and fit.get("status") == "IDENTIFIABLE_ON_FROZEN_GRID" and fit.get("accepted_parameters") is not None)
    heldout_core_complete = bool(heldout and all(heldout.get("methods", {}).get(name, {}).get("status") == "EVALUATED"
                                                for name in ("B0", "B1", "B2")))
    full_chain_success = (all(_is_pass(value) for value in stages.values()) and final_success_evidence and
                          physics_accepted and heldout_core_complete and all(twin_files.values()))
    if full_chain_success:
        overall_status = "FULL_CHAIN_SUCCESS"
    elif reason:
        overall_status = str(reason)
    elif _status(stages["I"]) == "KINEMATIC_ONLY_SUCCESS" and final_success_evidence:
        overall_status = "KINEMATIC_ONLY_SUCCESS"
    elif all(_is_pass(value) for value in stages.values()):
        overall_status = "MISSING_VERIFIED_FULL_CHAIN_EVIDENCE"
    else:
        overall_status = str(summary.get("status") or interaction_raw.get("status") or raw.get("status") or "NOT_RUN")
    return {
        "episode_id": manifest_episode["episode_id"], "asset_id": str(manifest_episode["asset_id"]),
        "split": str(manifest_episode["split"]).upper(), "configuration_index": manifest_episode.get("configuration_index"),
        "seed": manifest_episode.get("seed"), "included_in_denominator": manifest_episode.get("included_in_denominator", False),
        "manifest_exclusion_reason": manifest_episode.get("preflight_exclusion_reason"),
        "stages": stages, "first_failure_stage": first_stage, "first_failure_reason": reason,
        "overall_status": overall_status, "declared_wrapper_status": summary.get("status"),
        "physical_episode_executed": physical_ran,
        "full_chain_success": full_chain_success,
        "interaction_5deg_success": bool(_interaction_pass(stages["I"]) and final_success_evidence),
        "kinematics": {"joint_type": joint_type, "type_correct": evaluation.get("type_correct"),
                       "axis_angular_error_deg": evaluation.get("axis_angular_error_deg"),
                       "axis_line_distance_m": evaluation.get("axis_line_distance_m"),
                       "axis_line_metric_note": "Axis-line/observed-plane metric, not raw origin-point distance",
                       "trajectory_prediction_rmse_m": evaluation.get("gt_constrained_reconstruction_rmse_m"),
                       "fit_position_rmse_m": model.get("position_rmse_m"),
                       "fit_rotation_rmse_rad": model.get("rotation_rmse_rad"), "confidence": estimate.get("confidence")},
        "physics": {"status": fit.get("status") if fit else "NOT_RUN",
                    "accepted_parameters": fit.get("accepted_parameters") if fit else None,
                    "parameter_intervals": fit.get("parameter_intervals") if fit else None,
                    "heldout_B2_vs_B1_relative_loss_reduction": heldout_improvement,
                    "B3_status": (heldout or {}).get("methods", {}).get("B3", {}).get("status", "NOT_RUN"),
                    "Oracle_status": (heldout or {}).get("methods", {}).get("Oracle", {}).get("status", "NOT_RUN")},
        "interaction": {"actual_final_door_angle_deg": interaction_evaluation.get("actual_final_door_angle_deg"),
                        "actual_door_displacement_deg": displacement,
                        "actual_articulation_displacement_deg": displacement,
                        "initial_articulation_state_rad": manifest_episode.get("initialization_only", {}).get("joint_position_rad"),
                        "final_true_relative_slip_m": interaction_evaluation.get("final_true_relative_translation_slip_m"),
                        "maximum_true_relative_slip_m": interaction_raw.get("maximum_true_relative_slip_m"),
                        "maximum_detected_contact_surface_drift_m": interaction_raw.get("maximum_detected_contact_surface_drift_m"),
                        "minimum_joint_margin_rad": interaction_raw.get("minimum_joint_margin_rad"),
                        "peak_finger_handle_force_n": interaction_raw.get("peak_finger_handle_force_n"),
                        "bilateral_hold_established": interaction_raw.get("bilateral_hold_established"),
                        "simulator_only_safety_supervisor": interaction_raw.get("simulator_only_safety_supervisor"),
                        "full_relative_slip_observable_online": interaction_raw.get("full_relative_slip_observable_online"),
                        "safety_stop": safety_stop, "safety_events": safety_events,
                        "report_status": interaction_raw.get("status"), "final_report_missing": final_missing},
        "artifacts": {"episode_directory": str(directory), "selected_report": str(raw_path) if raw_path else None,
                      "episode_summary": str(directory / "episode_summary.json") if summary else None,
                      "physics_fit": str(fit_path) if fit else None, "heldout_comparison": str(heldout_path) if heldout else None,
                      "twins": twin_files, "continuous_video": video,
                      "interaction_report": str(interaction_path) if interaction_path else None,
                      "interaction_report_is_final_updated_run": bool(interaction_path and raw_path and interaction_path != raw_path)},
    }


def _rates(rows):
    planned = len(rows)
    stages = {}
    for key in STAGES:
        successful = sum((row["interaction_5deg_success"] if key == "I" else _is_pass(row["stages"][key])) for row in rows)
        evaluated = sum(_is_evaluated(row["stages"][key]) for row in rows)
        stages[key] = {"name": STAGES[key], "success": successful, "evaluated": evaluated,
                       "planned": planned, "not_evaluated": planned - evaluated,
                       "success_fraction_of_frozen_denominator": successful / planned if planned else None,
                       "success_rate_among_evaluated": successful / evaluated if evaluated else None}
    measured_improvements = [r["physics"]["heldout_B2_vs_B1_relative_loss_reduction"] for r in rows
                             if r["physics"]["heldout_B2_vs_B1_relative_loss_reduction"] is not None]
    accepted_kinematics = [r["kinematics"] for r in rows if _is_pass(r["stages"]["D"])]
    types = [r["type_correct"] for r in accepted_kinematics if r["type_correct"] is not None]
    axes = [r["axis_angular_error_deg"] for r in accepted_kinematics if r["axis_angular_error_deg"] is not None]
    lines = [r["axis_line_distance_m"] for r in accepted_kinematics if r["axis_line_distance_m"] is not None]
    progressed = [r["interaction"]["actual_articulation_displacement_deg"] for r in rows
                  if r["interaction"]["actual_articulation_displacement_deg"] is not None]
    return {"planned_episodes": planned, "physical_episodes_executed": sum(r["physical_episode_executed"] for r in rows),
            "full_chain_success": sum(r["full_chain_success"] for r in rows), "stages": stages,
            "accepted_kinematic_type_evaluations": len(types),
            "accepted_kinematic_joint_type_accuracy": sum(bool(x) for x in types) / len(types) if types else None,
            "accepted_kinematic_axis_error_mean_deg": float(np.mean(axes)) if axes else None,
            "accepted_kinematic_axis_line_error_mean_m": float(np.mean(lines)) if lines else None,
            "observed_max_articulation_displacement_deg": max(progressed) if progressed else None,
            "interaction_5deg_success_count": sum(r["interaction_5deg_success"] for r in rows),
            "interaction_5deg_success_fraction_of_frozen_denominator": sum(r["interaction_5deg_success"] for r in rows) / planned if planned else None,
            "physics_identified_count": sum(r["physics"]["status"] == "IDENTIFIABLE_ON_FROZEN_GRID" and r["physics"]["accepted_parameters"] is not None for r in rows),
            "heldout_comparisons_evaluated": len(measured_improvements),
            "heldout_improvement_positive_count": sum(v > 0 for v in measured_improvements),
            "heldout_mean_relative_loss_reduction": float(np.mean(measured_improvements)) if measured_improvements else None,
            "safety_stop_count": sum(r["interaction"]["safety_stop"] for r in rows),
            "interpretation": "Missing/blocked episodes remain in planned denominator; no observations are not a measured zero success rate"}


def _csv(path, rows, fields):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in fields} for row in rows)


def build_benchmark_report(manifest, episodes_root, output_dir, *, make_plots=True):
    """Summarize all frozen TEST slots, including assets that never ran.

    The manifest is hash verified. DEV never contributes to unseen TEST rates.
    When selection is incomplete, unselected slots remain explicit blockers.
    """
    manifest = verify_frozen_manifest(manifest)
    root, output = Path(episodes_root), Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    declared = [dict(row) for row in manifest["episodes"]]
    if len({row["episode_id"] for row in declared}) != len(declared):
        raise ValueError("Duplicate episode IDs would corrupt the frozen denominator")
    test_assets = list(dict.fromkeys(str(x) for x in manifest.get("test_asset_ids", [])))
    expected_assets = max(4, int(manifest["policy"].get("test_asset_count", 4)))
    per_asset = max(3, int(manifest["policy"].get("episodes_per_asset", 3)))
    for index in range(max(0, expected_assets - len(test_assets))):
        test_assets.append(f"UNSELECTED_TEST_SLOT_{index + 1:02d}")
    for asset_id in test_assets:
        have = [row for row in declared if str(row["asset_id"]) == asset_id and str(row["split"]).upper() == "TEST"]
        for index in range(len(have), per_asset):
            declared.append({"episode_id": f"test_{asset_id}_missing_{index:02d}", "asset_id": asset_id,
                             "split": "TEST", "configuration_index": index, "included_in_denominator": True,
                             "preflight_exclusion_reason": "BENCHMARK_ASSET_SELECTION_INCOMPLETE" if asset_id.startswith("UNSELECTED_") else "FROZEN_EPISODE_MISSING"})
    rows = [_episode(row, root) for row in declared]
    test = [r for r in rows if r["split"] == "TEST"]
    dev = [r for r in rows if r["split"] == "DEV"]
    per_asset_rows = []
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["split"], row["asset_id"])].append(row)
    for (split, asset_id), own in grouped.items():
        per_asset_rows.append({"split": split, "asset_id": asset_id, **_rates(own)})
    taxonomy = Counter()
    for row in rows:
        if row["full_chain_success"]:
            continue
        first = row["first_failure_stage"] or next((key for key in STAGES if not _is_pass(row["stages"][key])), "I")
        why = row["first_failure_reason"] or (row["stages"][first]["status"] if not _is_pass(row["stages"][first]) else "MISSING_VERIFIED_FULL_CHAIN_EVIDENCE")
        taxonomy[(row["split"], first, why)] += 1
    test_stats = _rates(test)
    identified_assets = sorted({r["asset_id"] for r in test if _is_pass(r["stages"]["D"]) and
                                r["kinematics"]["joint_type"] in ("revolute", "prismatic")})
    physical_assets = sorted({r["asset_id"] for r in test if r["interaction_5deg_success"] and
                              r["physical_episode_executed"]})
    proven = {
        "mode": "SIM_TO_SIM_BLIND_SYSID",
        "test_assets_with_real_simulated_contact_manipulation": physical_assets,
        "test_assets_with_accepted_ee_only_articulation": identified_assets,
        "cross_object_physical_interaction_observed": len(physical_assets) >= 2,
        "four_unseen_assets_demonstrated": len(physical_assets) >= 4,
        "physics_identification_accepted_episode_count": test_stats["physics_identified_count"],
        "heldout_prediction_improved_episode_count": test_stats["heldout_improvement_positive_count"],
        "full_loop_completed_episode_count": test_stats["full_chain_success"],
    }
    report = {
        "schema_version": 1, "manifest_sha256": manifest["manifest_sha256"],
        "frozen_selection_complete": manifest.get("selection_complete", False),
        "required_test_asset_count": expected_assets, "required_test_episodes_per_asset": per_asset,
        "TEST": test_stats, "DEV": _rates(dev), "per_asset": per_asset_rows,
        "episodes": rows,
        "failure_taxonomy": [{"split": split, "stage": stage, "reason": reason, "count": count}
                             for (split, stage, reason), count in sorted(taxonomy.items())],
        "supported_evidence": proven,
        "not_proven": ["Calibrated real robot hinge friction in N m", "Generalization to all household articulated objects",
                       "Full slip observability without external sensing", "Exact equivalence of interaction proxy and real geometry",
                       "Hardware deployment of simulator-only contact safety supervisor", "Complete joint limits from a short observed range"],
        "baseline_scope": "B0/B1/B2/B3 are internal ablations; Oracle is diagnostic only",
        "method_change_rule": "Changing method thresholds requires a new frozen version and rerunning all TEST slots",
    }
    _write(output / "benchmark_summary.json", report)
    episode_dir = output / "per_episode"
    episode_dir.mkdir(exist_ok=True)
    for row in rows:
        _write(episode_dir / (row["episode_id"] + ".json"), row)
    flat = [{"episode_id": r["episode_id"], "asset_id": r["asset_id"], "split": r["split"],
             "physical_episode_executed": r["physical_episode_executed"], "full_chain_success": r["full_chain_success"],
             **{key: r["stages"][key]["status"] for key in STAGES},
             "first_failure_stage": r["first_failure_stage"], "first_failure_reason": r["first_failure_reason"],
             **r["kinematics"], **r["interaction"],
             "physics_status": r["physics"]["status"],
             "heldout_B2_vs_B1_relative_loss_reduction": r["physics"]["heldout_B2_vs_B1_relative_loss_reduction"]} for r in rows]
    _csv(output / "episodes.csv", flat, list(flat[0]) if flat else ["episode_id", "asset_id", "split"])
    kinematic = [{"episode_id": r["episode_id"], "asset_id": r["asset_id"], "split": r["split"],
                  "identification_status": r["stages"]["D"]["status"], **r["kinematics"]} for r in rows]
    _csv(output / "articulation_errors.csv", kinematic, list(kinematic[0]) if kinematic else ["episode_id"])
    _csv(output / "failure_taxonomy.csv", report["failure_taxonomy"], ["split", "stage", "reason", "count"])
    _csv(output / "per_asset.csv", [{"asset_id": r["asset_id"], "split": r["split"],
                                     "planned": r["planned_episodes"], "physical_executed": r["physical_episodes_executed"],
                                     "full_chain_success": r["full_chain_success"],
                                     **{f"{key}_success": r["stages"][key]["success"] for key in STAGES},
                                     "physics_identified": r["physics_identified_count"],
                                     "heldout_evaluated": r["heldout_comparisons_evaluated"],
                                     "heldout_improved": r["heldout_improvement_positive_count"]} for r in per_asset_rows],
         ["asset_id", "split", "planned", "physical_executed", "full_chain_success"] + [f"{key}_success" for key in STAGES] +
         ["physics_identified", "heldout_evaluated", "heldout_improved"])
    md = ["# Interactive twin benchmark", "", "Simulation-surrogate experiment; DEV is excluded from unseen TEST results.", "",
          f"Frozen manifest: `{manifest['manifest_sha256']}`", "",
          f"TEST: {test_stats['physical_episodes_executed']} physical episodes executed / {test_stats['planned_episodes']} planned; "
          f"{test_stats['full_chain_success']} complete A–I chains.", "",
          "| Stage | Successful | Evaluated | Frozen denominator |", "|---|---:|---:|---:|"]
    md.extend(f"| {key}. {STAGES[key]} | {value['success']} | {value['evaluated']} | {value['planned']} |" for key, value in test_stats["stages"].items())
    md.extend(["", "Missing or blocked episodes remain in the denominator. A stage with no evaluated episodes has no measured success rate.", "",
               "## Physics evidence", "", f"Accepted physics estimates: {test_stats['physics_identified_count']}; held-out comparisons: {test_stats['heldout_comparisons_evaluated']}; "
               f"B2 improvements over B1: {test_stats['heldout_improvement_positive_count']}.", "",
               "No T2 improvement is inferred from training fit quality or missing runs.", "", "## Remaining limits", ""])
    md.extend("- " + value for value in report["not_proven"])
    (output / "README.md").write_text("\n".join(md) + "\n")
    if make_plots:
        plot_stage_counts(report, output / "stage_counts.png")
    return report


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_stage_counts(report, output_path):
    plt = _pyplot()
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(11, 4.5))
    values = report["TEST"]["stages"]
    passed = np.array([values[key]["success"] for key in STAGES])
    evaluated = np.array([values[key]["evaluated"] for key in STAGES])
    planned = np.array([values[key]["planned"] for key in STAGES])
    x = np.arange(len(STAGES))
    ax.bar(x, passed, label="success", color="#278467")
    ax.bar(x, evaluated - passed, bottom=passed, label="evaluated, not successful", color="#cf695d")
    ax.bar(x, planned - evaluated, bottom=evaluated, label="not evaluated / blocked", color="#d9d9de")
    ax.set_xticks(x, list(STAGES)); ax.set_ylabel("TEST episodes (frozen denominator)")
    ax.set_title("A–I stage evidence; missing runs remain visible")
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.1), ncol=3)
    fig.tight_layout(); fig.savefig(output_path, dpi=150); plt.close(fig)
    return str(output_path)


def plot_sensitivity_curves(conditions, output_path, *, report=None):
    """Plot actual LOW/MEDIUM/HIGH observable signals, never hidden GT angle."""
    plt = _pyplot()
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for index, (label, logs) in enumerate(conditions.items()):
        for repeat, log in enumerate(logs):
            validate_log(log)
            t = np.asarray(log["time_s"])
            q = np.asarray(log["signals"]["q_rad"])
            qdot = np.asarray(log["signals"]["qdot_rad_s"])
            p = np.asarray(log["signals"]["ee_T_world_tcp"])[:, :3, 3]
            values = [np.linalg.norm(p - p[0], axis=1) * 1000,
                      np.linalg.norm(np.gradient(p, t, axis=0), axis=1) * 1000,
                      np.linalg.norm(q - q[0], axis=1), np.linalg.norm(qdot, axis=1)]
            for ax, value in zip(axes.flat, values):
                ax.plot(t, value, color=f"C{index % 10}", alpha=.8, label=label if repeat == 0 else None)
    for ax, title, unit in zip(axes.flat, ["Measured EE travel", "Measured EE velocity", "Joint displacement norm", "Joint velocity norm"], ["mm", "mm/s", "rad", "rad/s"]):
        ax.set_title(title); ax.set_ylabel(unit); ax.set_xlabel("Protocol time (s)"); ax.grid(alpha=.2)
    axes[0, 0].legend()
    fig.suptitle("Physics sensitivity: " + (report or {}).get("status", "observable signals only"))
    fig.tight_layout(); fig.savefig(output_path, dpi=150); plt.close(fig)
    return str(output_path)


def plot_heldout_predictions(reference, predictions, output_path, *, comparison=None):
    """Held-out sensor curves with missing models omitted, not synthesized."""
    plt = _pyplot()
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for label, log in [("reference", reference)] + list(predictions.items()):
        validate_log(log)
        if log["split"] != "heldout":
            raise ValueError("Held-out plot received training data")
        t = np.asarray(log["time_s"])
        p = np.asarray(log["signals"]["ee_T_world_tcp"])[:, :3, 3]
        q = np.asarray(log["signals"]["q_rad"])
        values = [(p[:, 0] - p[0, 0]) * 1000, (p[:, 1] - p[0, 1]) * 1000,
                  np.linalg.norm(np.gradient(p, t, axis=0), axis=1) * 1000,
                  np.linalg.norm(q - q[0], axis=1)]
        for ax, value in zip(axes.flat, values):
            ax.plot(t, value, label=label, linewidth=2. if label == "reference" else 1., linestyle="-" if label == "reference" else "--")
    for ax, title, unit in zip(axes.flat, ["EE X displacement", "EE Y displacement", "EE speed", "Joint displacement norm"], ["mm", "mm", "mm/s", "rad"]):
        ax.set_title(title); ax.set_ylabel(unit); ax.set_xlabel("Protocol time (s)"); ax.grid(alpha=.2)
    axes[0, 0].legend()
    fig.suptitle("P4 held-out prediction (not used to fit physics)")
    fig.tight_layout(); fig.savefig(output_path, dpi=150); plt.close(fig)
    return str(output_path)
