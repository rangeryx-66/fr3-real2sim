#!/usr/bin/env python3
"""Contract/analysis tests with synthetic fixtures, not physical validation."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np

from interactive_twin.calibration import calibrate_robot_response
from interactive_twin.sysid import (InvalidLog, NoiseScales, candidate_grid, command_hash,
                                   fit_resistance, heldout_comparison,
                                   noise_normalized_distance, sensitivity_report, validate_log)


NOISE = NoiseScales(position_m=2e-5, rotation_rad=2e-5, joint_rad=np.deg2rad(.001),
                    joint_velocity_rad_s=.001, ee_velocity_m_s=.0001,
                    start_time_s=.02, tracking_m=2e-5, calibration_id="unit_test_cal")


def fixture(c=1., b=1., probe="P1", split="train"):
    """Analytic arrays exercise analysis code only; no simulated success claim."""
    t = np.linspace(0, 2, 101)
    q = np.zeros((len(t), 6))
    q[:, 0] = (.05 - .01 * c) * t
    q[:, 1] = (.025 - .005 * b) * t ** 2
    pose = np.tile(np.eye(4), (len(t), 1, 1))
    pose[:, 0, 3] = q[:, 0] * .4
    pose[:, 1, 3] = q[:, 1] * .3
    commands = {"time_s": t.tolist(),
                "values": np.column_stack((.02 * t, .01 * t, np.zeros(len(t)))).tolist(),
                "fields": ["ee_reference_x_m", "ee_reference_y_m", "ee_reference_z_m"],
                "kind": "unit_test_fixture"}
    return {
        "schema_version": 1, "mode": "SIM_TO_SIM_BLIND_SYSID", "episode_id": "unit_test_only",
        "probe_id": probe, "split": split, "time_s": t.tolist(), "commands": commands,
        "signals": {"q_rad": q.tolist(), "qdot_rad_s": np.gradient(q, t, axis=0).tolist(),
                    "ee_T_world_tcp": pose.tolist(), "gripper_opening_m": [.03] * len(t)},
        "provenance": {"source": "isaac_physics", "robot_model_id": "fixture_robot",
                       "robot_calibration_id": "unit_test_cal", "controller_id": "fixture_controller",
                       "simulator": "isaac_physx", "complete": True, "initialization_only": True,
                       "robot_state_replayed": False, "object_state_replayed": False,
                       "direct_object_actuation": False, "attachment": False,
                       "command_applied_sha256": command_hash(commands)},
    }


class Contracts(unittest.TestCase):
    def test_gt_rejected_recursively(self):
        log = fixture()
        log["signals"]["gt_door_angle"] = [0.] * len(log["time_s"])
        with self.assertRaisesRegex(InvalidLog, "Forbidden"):
            validate_log(log)

    def test_replay_and_changed_command_rejected(self):
        log = fixture()
        log["provenance"]["robot_state_replayed"] = True
        with self.assertRaisesRegex(InvalidLog, "replay violation"):
            noise_normalized_distance(fixture(), log, NOISE)
        log = fixture()
        log["commands"]["values"][2][0] += .1
        log["provenance"]["command_applied_sha256"] = command_hash(log["commands"])
        with self.assertRaisesRegex(InvalidLog, "identical timed command"):
            noise_normalized_distance(fixture(), log, NOISE)

    def test_incomplete_reference_rejected(self):
        reference = fixture()
        reference["provenance"]["complete"] = False
        with self.assertRaisesRegex(InvalidLog, "Reference probe is incomplete"):
            noise_normalized_distance(reference, fixture(), NOISE)

    def test_full_trajectory_not_only_endpoint(self):
        log = fixture()
        t = np.asarray(log["time_s"])
        pose = np.asarray(log["signals"]["ee_T_world_tcp"])
        pose[:, 2, 3] += .002 * np.sin(t * np.pi / 2)
        log["signals"]["ee_T_world_tcp"] = pose.tolist()
        result = noise_normalized_distance(fixture(), log, NOISE)
        self.assertLess(result["metrics"]["final_displacement_error_m"], 1e-12)
        self.assertGreater(result["metrics"]["ee_position_rmse_m"], .0005)
        self.assertGreater(result["normalized_loss"], 1.)

    def test_current_is_not_implicitly_external_torque(self):
        log = fixture()
        log["signals"]["motor_current_a"] = np.zeros((len(log["time_s"]), 6)).tolist()
        with self.assertRaisesRegex(InvalidLog, "B3 unavailable"):
            noise_normalized_distance(log, deepcopy(log), NOISE, use_current=True)

    def test_optional_current_does_not_change_common_motion_metric(self):
        reference, prediction = fixture(), fixture()
        for log, value in ((reference, 1.), (prediction, 2.)):
            log["signals"]["motor_current_a"] = np.full((len(log["time_s"]), 6), value).tolist()
            log["sensor_calibration"] = {"motor_current_a": {"independently_calibrated": True,
                                                            "calibration_id": "fixture_current_cal"}}
        noise = NoiseScales(**{**NOISE.__dict__, "current_a": .1})
        result = noise_normalized_distance(reference, prediction, noise, use_current=True)
        self.assertEqual(result["motion_only_normalized_loss"], 0.)
        self.assertGreater(result["normalized_loss"], 0.)

    def test_mode_b_real_log_supported_not_relabelled_sim(self):
        real, twin = fixture(), fixture()
        real["mode"] = twin["mode"] = "REAL_LOG_TO_SIM"
        real["provenance"]["source"] = "real_robot_log"
        self.assertEqual(noise_normalized_distance(real, twin, NOISE)["normalized_loss"], 0.)
        real["mode"] = "SIM_TO_SIM_BLIND_SYSID"
        with self.assertRaisesRegex(InvalidLog, "relabeled"):
            validate_log(real)

    def test_truncated_rollout_and_frozen_robot_change_rejected(self):
        predicted = fixture()
        predicted["provenance"]["controller_id"] = "different_controller"
        with self.assertRaisesRegex(InvalidLog, "Frozen robot/control"):
            noise_normalized_distance(fixture(), predicted, NOISE)
        predicted = fixture()
        predicted["time_s"][-1] -= .01
        with self.assertRaisesRegex(InvalidLog, "Incomplete observation horizon"):
            validate_log(predicted)

    def test_budget_and_heldout_leakage(self):
        with self.assertRaisesRegex(ValueError, "budget"):
            candidate_grid([0., 1., 2.], [0., 1., 2.], J_eff_prior=1., budget=8)
        with self.assertRaisesRegex(InvalidLog, "P4 is held out"):
            fit_resistance({"P4": fixture(probe="P4", split="heldout")}, [], NOISE, J_eff_prior=1., budget=9)

    def test_fixed_inertial_model_is_not_a_fake_scalar(self):
        prior = {"kind": "fixed_articulated_model", "sha256": "a" * 64,
                 "scalar_J_eff": None, "frozen": True}
        grid = candidate_grid([0., 1.], [0., 1.], J_eff_prior=prior, budget=4)
        self.assertEqual(grid[0]["J_eff_prior"], prior)
        with self.assertRaisesRegex(ValueError, "Malformed"):
            candidate_grid([0., 1.], [0., 1.], J_eff_prior={**prior, "scalar_J_eff": 1.}, budget=4)


class Identification(unittest.TestCase):
    def setUp(self):
        self.refs = {p: fixture(probe=p) for p in ("P1", "P2", "P3")}
        self.grid = candidate_grid([0., 1., 2.], [0., 1., 2.], J_eff_prior=1., budget=9)
        self.rollouts = [{**row, "probes": {p: fixture(row["tau_c"], row["b"], probe=p) for p in self.refs}}
                         for row in self.grid]
        self.sensitivity = sensitivity_report({"LOW": [fixture(0., 0.)] * 2,
                                               "MEDIUM": [fixture(1., 1.)] * 2,
                                               "HIGH": [fixture(2., 2.)] * 2}, NOISE)

    def test_full_rank_grid_accepts_correct_candidate(self):
        result = fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9,
                                sensitivity_evidence=self.sensitivity, expected_grid=self.grid)
        self.assertEqual(result["status"], "IDENTIFIABLE_ON_FROZEN_GRID")
        self.assertEqual(result["accepted_parameters"], {"tau_c": 1., "b": 1.})
        self.assertEqual(result["probe_segments_evaluated"], 27)
        self.assertIsNone(result["physics_rollouts_used"])

    def test_repeat_variability_can_remove_apparent_two_parameter_rank(self):
        quiet = fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9,
                               sensitivity_evidence=self.sensitivity, expected_grid=self.grid)
        self.assertEqual(quiet["sensitivity_rank"], 2)
        # Hold every command, response, loss and grid point fixed. The same
        # apparent parameter response is below a larger independently measured
        # DEV repeat floor; a zero training residual must not override this.
        floor = 2. * max(quiet["normalized_singular_values"])
        noisy_evidence = {**self.sensitivity, "repeat_floor_noise_units": floor}
        result = fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9,
                                sensitivity_evidence=noisy_evidence, expected_grid=self.grid)
        self.assertTrue(result["reference_sensitivity_passed"])
        self.assertEqual(result["rank_repeat_floor_noise_units"], floor)
        np.testing.assert_allclose(result["normalized_singular_values"],
                                   np.asarray(quiet["normalized_singular_values"]) / floor)
        self.assertEqual(result["sensitivity_rank"], 0)
        self.assertEqual(result["status"], "UNIDENTIFIABLE_UNDER_CURRENT_PROBE")
        self.assertIsNone(result["accepted_parameters"])
        self.assertEqual(result["best_grid_candidate_diagnostic"],
                         quiet["best_grid_candidate_diagnostic"])

    def test_flat_response_is_unidentifiable(self):
        for row in self.rollouts:
            row["probes"] = deepcopy(self.refs)
        result = fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9,
                                sensitivity_evidence=self.sensitivity, expected_grid=self.grid)
        self.assertEqual(result["status"], "UNIDENTIFIABLE_UNDER_CURRENT_PROBE")
        self.assertIsNone(result["accepted_parameters"])
        self.assertEqual(result["parameter_intervals"], {"tau_c": [0., 2.], "b": [0., 2.]})

    def test_missing_sensitivity_cannot_claim_identifiability(self):
        result = fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9)
        self.assertEqual(result["status"], "SENSITIVITY_NOT_VALIDATED")
        self.assertIsNone(result["accepted_parameters"])

    def test_sensitivity_includes_repeat_noise(self):
        report = sensitivity_report({"LOW": [fixture(), fixture()], "HIGH": [fixture(), fixture()]}, NOISE)
        self.assertEqual(report["status"], "UNIDENTIFIABLE_UNDER_CURRENT_PROBE")
        self.assertTrue(report["repeat_variability_measured"])

    def test_censored_grid_cannot_create_a_narrow_identifiable_interval(self):
        observed = [row for row in self.rollouts if row["tau_c"] < 2.]
        failed = [{"candidate_id": row["candidate_id"], "status": "SUSTAINED_CONTACT_LOSS"}
                  for row in self.rollouts if row["tau_c"] == 2.]
        result = fit_resistance(self.refs, observed, NOISE, J_eff_prior=1., budget=9,
                                sensitivity_evidence=self.sensitivity, expected_grid=self.grid,
                                censored_candidates=failed)
        self.assertEqual(result["status"], "UNIDENTIFIABLE_UNDER_CURRENT_PROBE")
        self.assertIsNone(result["accepted_parameters"])
        self.assertTrue(result["grid_censored"])
        self.assertEqual(result["parameter_intervals"], {"tau_c": [0., 2.], "b": [0., 2.]})
        self.assertEqual(len(result["censored_candidates"]), 3)
        self.assertEqual(result["best_grid_candidate_diagnostic"]["tau_c"], 1.)

    def test_unexecuted_grid_points_are_automatically_censored(self):
        result = fit_resistance(self.refs, self.rollouts[:-1], NOISE, J_eff_prior=1., budget=9,
                                sensitivity_evidence=self.sensitivity, expected_grid=self.grid)
        self.assertEqual(result["censored_candidates"][0]["status"], "NOT_RUN_OR_INCOMPLETE")
        self.assertIsNone(result["accepted_parameters"])

    def test_unverified_grid_or_wrong_calibration_cannot_claim_identified(self):
        result = fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9,
                                sensitivity_evidence=self.sensitivity)
        self.assertEqual(result["status"], "GRID_COVERAGE_NOT_VALIDATED")
        self.assertIsNone(result["accepted_parameters"])
        bad = {**self.sensitivity, "robot_calibration_id": "different"}
        with self.assertRaisesRegex(InvalidLog, "same frozen"):
            fit_resistance(self.refs, self.rollouts, NOISE, J_eff_prior=1., budget=9,
                           sensitivity_evidence=bad, expected_grid=self.grid)

    def test_heldout_better_and_oracle_missing_not_fabricated(self):
        reference = fixture(probe="P4", split="heldout")
        result = heldout_comparison(reference, {"B0": fixture(0., 0., "P4", "heldout"),
                                                "B1": fixture(0., 1., "P4", "heldout"),
                                                "B2": deepcopy(reference)}, NOISE)
        self.assertEqual(result["methods"]["Oracle"]["status"], "NOT_RUN")
        self.assertEqual(result["B2_improvement"]["B0"]["relative_loss_reduction"], 1.)


class RobotCalibration(unittest.TestCase):
    def test_noise_floors_and_sim_scope(self):
        logs = [fixture(split="calibration"), fixture(split="calibration")]
        for log in logs:
            log["provenance"]["no_contact_supervisor_certified"] = True
        floors = dict(NOISE.__dict__)
        floors.pop("calibration_id")
        report = calibrate_robot_response(logs, noise_floors=floors)
        self.assertFalse(report["hardware_calibration"])
        self.assertEqual(report["noise_scales"]["position_m"], floors["position_m"])
        self.assertIsNone(report["response"]["command_latency_s"])
        self.assertFalse(report["current_or_effort"]["motor_current_a"]["available"])
        self.assertEqual(report["calibration_id"], report["noise_scales"]["calibration_id"])

    def test_contact_calibration_rejected(self):
        floors = dict(NOISE.__dict__)
        floors.pop("calibration_id")
        with self.assertRaisesRegex(InvalidLog, "no-contact"):
            calibrate_robot_response([fixture(split="calibration")] * 2, noise_floors=floors)


if __name__ == "__main__":
    unittest.main()
