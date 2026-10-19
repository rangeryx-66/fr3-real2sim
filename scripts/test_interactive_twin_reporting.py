#!/usr/bin/env python3
"""Report accounting tests; synthetic metadata does not prove physical success."""
from pathlib import Path
import hashlib
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from interactive_twin.reporting import build_benchmark_report


def manifest(asset_count=4):
    ids = [str(100 + i) for i in range(asset_count)]
    episodes = [{"episode_id": f"{split.lower()}_{asset}_{j}", "asset_id": asset, "split": split,
                 "configuration_index": j, "seed": j, "included_in_denominator": split == "TEST"}
                for split, assets in [("DEV", ["7320"]), ("TEST", ids)] for asset in assets for j in range(3)]
    value = {"schema": "interactive-twin-manifest-v1", "frozen": True,
             "policy": {"test_asset_count": 4, "episodes_per_asset": 3},
             "test_asset_ids": ids, "episodes": episodes, "selection_complete": asset_count == 4}
    value["manifest_sha256"] = hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    return value


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


class Reporting(unittest.TestCase):
    def test_missing_test_slots_remain_and_dev_excluded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write(root / "episodes/dev_7320_0/episode_summary.json",
                  {"status": "SUCCESS", "stages": {key: {"status": "SUCCESS"} for key in "ABCDEFGHI"}})
            result = build_benchmark_report(manifest(2), root / "episodes", root / "report", make_plots=False)
            self.assertEqual(result["TEST"]["planned_episodes"], 12)
            self.assertEqual(result["TEST"]["full_chain_success"], 0)
            self.assertEqual(result["DEV"]["full_chain_success"], 0)  # Labels without physical/physics evidence are insufficient.
            self.assertEqual(result["TEST"]["physical_episodes_executed"], 0)
            self.assertIsNone(result["TEST"]["stages"]["B"]["success_rate_among_evaluated"])
            self.assertIsNone(result["TEST"]["heldout_mean_relative_loss_reduction"])
            self.assertTrue((root / "report/articulation_errors.csv").is_file())

    def test_wrapper_stage_is_authority_and_failure_not_removed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            own = root / "episodes/test_100_0"
            write(own / "report.json", {"status": "SUCCESS", "success": True, "duration_s": 10.,
                                       "bilateral_hold_established": True})
            write(own / "episode_summary.json", {"status": "NO_IK", "stages": {
                "A": {"status": "COMPLETE"}, "B": {"status": "FAILED", "reason": "NO_IK"}}})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report", make_plots=False)
            row = next(r for r in result["episodes"] if r["episode_id"] == "test_100_0")
            self.assertEqual(row["first_failure_stage"], "B")
            self.assertEqual(row["stages"]["I"]["status"], "BLOCKED_B")
            self.assertEqual(result["TEST"]["planned_episodes"], 12)
            self.assertEqual(result["TEST"]["stages"]["B"]["success"], 0)

    def test_ambiguous_candidates_cannot_select_best(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            own = root / "episodes/test_100_0"
            write(own / "candidate_01/report.json", {"status": "SUCCESS", "success": True})
            write(own / "candidate_02/report.json", {"status": "NO_IK", "success": False})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report", make_plots=False)
            row = next(r for r in result["episodes"] if r["episode_id"] == "test_100_0")
            self.assertIsNone(row["artifacts"]["selected_report"])
            self.assertEqual(row["stages"]["A"]["status"], "BLOCKED_REPORT_AMBIGUITY")

    def test_physics_complete_is_not_final_manipulation_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write(root / "episodes/test_100_0/report.json", {
                "status": "PHYSICS_PROTOCOL_COMPLETE", "success": False,
                "bilateral_hold_established": True, "accepted_estimate_count": 1,
                "estimate": {"joint_type": "revolute"}, "probe": {"distance_m": .01},
                "followed_estimated_articulation": True, "physics_protocol_complete": True})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report", make_plots=False)
            self.assertEqual(result["TEST"]["stages"]["E"]["success"], 1)
            self.assertEqual(result["TEST"]["stages"]["I"]["evaluated"], 0)
            self.assertEqual(result["TEST"]["full_chain_success"], 0)

    def test_kinematic_only_success_counts_physical_progress_not_full_loop(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            own = root / "episodes/test_100_0"
            write(own / "selected/report.json", {
                "status": "SUCCESS", "success": True, "duration_s": 10.,
                "bilateral_hold_established": True, "estimate": {"joint_type": "revolute"},
                "evaluation": {"actual_door_displacement_deg": 5.2, "actual_final_door_angle_deg": 7.2}})
            write(own / "episode_summary.json", {"selected_report": "selected/report.json", "stages": {
                "A": {"status": "SUCCESS"}, "B": {"status": "SUCCESS"}, "D": {"status": "SUCCESS"},
                "E": {"status": "BLOCKED", "reason": "SENSITIVITY_UNIDENTIFIABLE"},
                "I": {"status": "KINEMATIC_ONLY_SUCCESS", "physics_updated": False}}})
            for name in ("T0", "T1", "T2"):
                write(own / "kinematic_update/twins" / name / "twin.json", {"version": name})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report", make_plots=False)
            row = next(r for r in result["episodes"] if r["episode_id"] == "test_100_0")
            self.assertEqual(result["TEST"]["planned_episodes"], 12)
            self.assertEqual(result["TEST"]["interaction_5deg_success_count"], 1)
            self.assertEqual(result["TEST"]["stages"]["I"]["success"], 1)
            self.assertAlmostEqual(result["TEST"]["interaction_5deg_success_fraction_of_frozen_denominator"], 1/12)
            self.assertFalse(row["full_chain_success"])
            self.assertIn("kinematic_update/twins/T1", row["artifacts"]["twins"]["T1"])
            self.assertEqual(row["interaction"]["actual_articulation_displacement_deg"], 5.2)
            self.assertEqual(row["first_failure_reason"], "SENSITIVITY_UNIDENTIFIABLE")

    def test_final_action_report_video_and_safety_are_separate_from_identification(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            own = root / "episodes/test_100_0"
            write(own / "selected/report.json", {
                "status": "SUCCESS", "success": True, "duration_s": 10.,
                "bilateral_hold_established": True, "estimate": {"joint_type": "revolute"},
                "evaluation": {"actual_door_displacement_deg": 5.2, "axis_angular_error_deg": 1.}})
            write(own / "final_updated_interaction/report.json", {
                "status": "PROBE_CARTESIAN_SPEED_LIMIT", "success": False, "duration_s": 11.,
                "bilateral_hold_established": True, "minimum_joint_margin_rad": .07,
                "video": "failure.mp4", "evaluation": {"actual_door_displacement_deg": 1.3,
                "actual_final_door_angle_deg": 3.3, "axis_angular_error_deg": 2.}})
            (own / "final_updated_interaction/failure.mp4").write_bytes(b"test-video-placeholder")
            write(own / "episode_summary.json", {"selected_report": "selected/report.json", "stages": {
                "A": {"status": "SUCCESS"}, "B": {"status": "SUCCESS"}, "D": {"status": "SUCCESS"},
                "I": {"status": "FAILED", "reason": "PROBE_CARTESIAN_SPEED_LIMIT",
                      "report": "final_updated_interaction/report.json"}}})
            for name in ("T0", "T1", "T2"):
                write(own / "twins_final" / name / "twin.json", {"version": name})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report", make_plots=False)
            row = next(r for r in result["episodes"] if r["episode_id"] == "test_100_0")
            self.assertEqual(row["kinematics"]["axis_angular_error_deg"], 1.)
            self.assertEqual(row["interaction"]["actual_door_displacement_deg"], 1.3)
            self.assertEqual(row["interaction"]["minimum_joint_margin_rad"], .07)
            self.assertTrue(row["interaction"]["safety_stop"])
            self.assertEqual(result["TEST"]["safety_stop_count"], 1)
            self.assertEqual(row["overall_status"], "PROBE_CARTESIAN_SPEED_LIMIT")
            self.assertEqual(result["TEST"]["interaction_5deg_success_count"], 0)
            self.assertTrue(row["artifacts"]["continuous_video"].endswith("final_updated_interaction/failure.mp4"))
            self.assertIn("twins_final/T2", row["artifacts"]["twins"]["T2"])
            self.assertTrue(any(r["reason"] == "PROBE_CARTESIAN_SPEED_LIMIT" for r in result["failure_taxonomy"]))

    def test_missing_final_report_cannot_borrow_initial_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            own = root / "episodes/test_100_0"
            write(own / "selected/report.json", {"status": "SUCCESS", "success": True,
                  "duration_s": 10., "bilateral_hold_established": True,
                  "evaluation": {"actual_door_displacement_deg": 5.2}})
            write(own / "episode_summary.json", {"selected_report": "selected/report.json", "stages": {
                "I": {"status": "SUCCESS", "report": "final_updated_interaction/report.json"}}})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report", make_plots=False)
            row = next(r for r in result["episodes"] if r["episode_id"] == "test_100_0")
            self.assertTrue(row["interaction"]["final_report_missing"])
            self.assertIsNone(row["interaction"]["actual_door_displacement_deg"])
            self.assertFalse(row["interaction_5deg_success"])
            self.assertIsNone(row["artifacts"]["continuous_video"])

    def test_full_chain_requires_accepted_physics_and_final_physical_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            own = root / "episodes/test_100_0"
            write(own / "report.json", {"status": "SUCCESS", "success": True,
                  "duration_s": 10., "bilateral_hold_established": True,
                  "evaluation": {"actual_door_displacement_deg": 5.2}})
            write(own / "episode_summary.json", {"status": "SUCCESS", "stages": {
                key: {"status": "SUCCESS"} for key in "ABCDEFGHI"}})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report1", make_plots=False)
            self.assertEqual(result["TEST"]["full_chain_success"], 0)
            write(own / "physics_fit.json", {"status": "IDENTIFIABLE_ON_FROZEN_GRID",
                                             "accepted_parameters": {"tau_c": .001, "b": .01}})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report2", make_plots=False)
            self.assertEqual(result["TEST"]["full_chain_success"], 0)
            write(own / "heldout_comparison.json", {"methods": {name: {"status": "EVALUATED"} for name in ("B0", "B1", "B2")}})
            for name in ("T0", "T1", "T2"):
                write(own / "twins_final" / name / "twin.json", {"version": name})
            result = build_benchmark_report(manifest(), root / "episodes", root / "report3", make_plots=False)
            self.assertEqual(result["TEST"]["full_chain_success"], 1)


if __name__ == "__main__":
    unittest.main()
