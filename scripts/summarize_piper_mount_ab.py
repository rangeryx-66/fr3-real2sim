#!/usr/bin/env python3
"""Summarize the frozen FR3 / original-Piper / mounted-Piper benchmark."""
from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from collections import Counter
from pathlib import Path

OBJECTS = ("mustard", "raisin", "hidden_tuna", "bowl", "banana", "sugar", "soup", "mug")


def load(root: Path) -> dict[tuple[str, int], dict]:
    rows = {}
    for obj in OBJECTS:
        for path in sorted((root / obj).glob("B_seed_*.json")):
            row = json.loads(path.read_text())
            row["_path"] = str(path)
            rows[(obj, int(row["seed"]))] = row
    return rows


def metrics(rows: list[dict]) -> dict:
    checked = sum(len(row.get("candidates", [])) for row in rows)
    statuses = Counter(
        candidate.get("status", "UNKNOWN")
        for row in rows
        for candidate in row.get("candidates", [])
    )
    executable = statuses["VALID"]
    ranks = [row["selected_rank"] for row in rows if row.get("selected_rank") is not None]
    return {
        "episodes": len(rows),
        "success": sum(bool(row.get("success")) for row in rows),
        "categories": dict(Counter(row.get("category", "UNKNOWN") for row in rows)),
        "episodes_with_executable": sum(row.get("selected_rank") is not None for row in rows),
        "candidates_checked": checked,
        "executable_candidates": executable,
        "executable_candidate_ratio": executable / checked if checked else None,
        "candidate_status": dict(statuses),
        "no_ik": statuses["NO_IK"],
        "no_ik_ratio": statuses["NO_IK"] / checked if checked else None,
        "non_target_contact": sum(bool(row.get("non_target_contact")) for row in rows),
        "non_target_disturbance": sum(bool(row.get("non_target_disturbance")) for row in rows),
        "mean_selected_rank": statistics.fmean(ranks) if ranks else None,
        "mean_planning_seconds": statistics.fmean(
            row.get("planning_seconds", 0.0) for row in rows
        ) if rows else None,
    }


def input_digest(row: dict) -> str | None:
    digest = row.get("grasp_sha256")
    if digest:
        return digest
    path = row.get("grasp_input")
    if path and Path(path).is_file():
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fr3", required=True, type=Path)
    parser.add_argument("--piper-original", required=True, type=Path)
    parser.add_argument("--piper-mounted", required=True, type=Path)
    parser.add_argument("--mount-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    runs = {
        "FR3": load(args.fr3),
        "Piper_original": load(args.piper_original),
        "Piper_mounted": load(args.piper_mounted),
    }
    common = sorted(set.intersection(*(set(run) for run in runs.values())))
    hash_match = {
        f"{obj}:{seed}": len({input_digest(run[(obj, seed)]) for run in runs.values()}) == 1
        for obj, seed in common
    }
    report = {
        "schema": "piper_mount_ab/v1",
        "mount": json.loads(args.mount_config.read_text()),
        "paired_episodes": len(common),
        "all_anygrasp_inputs_identical": bool(common) and all(hash_match.values()),
        "input_hash_match": hash_match,
        "overall": {
            name: metrics([run[key] for key in sorted(run)]) for name, run in runs.items()
        },
        "per_object": {
            obj: {
                name: metrics([row for (kind, _), row in run.items() if kind == obj])
                for name, run in runs.items()
            }
            for obj in OBJECTS
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")

    md = [
        "# Piper mounting-pose frozen A/B",
        "",
        f"Paired episodes: {report['paired_episodes']}; identical AnyGrasp inputs: "
        f"{report['all_anygrasp_inputs_identical']}",
        "",
        "|Robot/config|Success|Executable episodes|NO_IK|Executable candidates|Contact|Disturbance|Mean planning (s)|",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, value in report["overall"].items():
        md.append(
            f"|{name}|{value['success']}/{value['episodes']}|"
            f"{value['episodes_with_executable']}/{value['episodes']}|"
            f"{value['no_ik']}/{value['candidates_checked']}|"
            f"{value['executable_candidates']}/{value['candidates_checked']}|"
            f"{value['non_target_contact']}/{value['episodes']}|"
            f"{value['non_target_disturbance']}/{value['episodes']}|"
            f"{value['mean_planning_seconds']:.2f}|"
        )
    md += ["", "|Object|FR3|Piper original|Piper mounted|", "|---|---:|---:|---:|"]
    for obj, values in report["per_object"].items():
        md.append(
            f"|{obj}|{values['FR3']['success']}/{values['FR3']['episodes']}|"
            f"{values['Piper_original']['success']}/{values['Piper_original']['episodes']}|"
            f"{values['Piper_mounted']['success']}/{values['Piper_mounted']['episodes']}|"
        )
    args.output.with_suffix(".md").write_text("\n".join(md) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "input_hash_match"}, indent=2))


if __name__ == "__main__":
    main()
