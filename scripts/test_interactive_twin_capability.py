"""Capability/data-boundary checks; no robot, SDK import, NumPy or network."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from interactive_twin.capability import (build_capability_report, calibration_eligible,
    inspect_local_sdk, project_observation, validate_controller_inputs, write_capability_report)


class CapabilityBoundaryTests(unittest.TestCase):
    def test_sdk_is_never_executed(self):
        with tempfile.TemporaryDirectory() as tmp:
            package = Path(tmp) / "piper_sdk"
            (package / "interface").mkdir(parents=True)
            (package / "__init__.py").write_text("raise RuntimeError('must not import SDK')")
            (package / "interface/piper_interface.py").write_text(
                "raise RuntimeError('must not execute source')\nclass SDK:\n def GetArmJointMsgs(self): pass\n")
            (package / "version.py").write_text("PIPER_SDK_CURRENT_VERSION = PIPER_SDK_VERSION_0_6_2\n")
            result = inspect_local_sdk(tmp)
            self.assertEqual(result["api_methods"], ["GetArmJointMsgs"])
            self.assertEqual(result["version"], "0.6.2")
            self.assertFalse(result["hardware_connected"])
            self.assertEqual(result["firmware"], "UNKNOWN")

    def test_projection_excludes_truth_and_simulation_tactile(self):
        row = {"t": 1., "q": [0.] * 6, "qdot": [0.] * 6,
               "T_tcp": [[1, 0, 0, .2], [0, 1, 0, .1], [0, 0, 1, .3], [0, 0, 0, 1]],
               "door_angle": .5, "axis": [0, 0, 1], "contacts": [{"force_n": 99}],
               "forces_n": {"left": 1}, "sdk_effort_raw": [123] * 6,
               "motor_current_a": [1.] * 6, "external_torque_nm": [99] * 6}
        projected = project_observation(row)
        self.assertEqual(set(projected), {"t_s", "q_rad", "qdot_rad_s", "T_world_tcp"})
        validate_controller_inputs(projected)
        with self.assertRaisesRegex(ValueError, "NON_HARDWARE_POLICY_INPUT"):
            validate_controller_inputs(["q_rad", "contacts"])
        with self.assertRaisesRegex(ValueError, "NON_HARDWARE_POLICY_INPUT"):
            validate_controller_inputs(["GT_joint_axis"])

    def test_cannot_smuggle_truth_in_command(self):
        with self.assertRaisesRegex(ValueError, "COMMAND_SCHEMA_REJECTED"):
            project_observation({"command": {"door_angle": 1}})
        with self.assertRaisesRegex(ValueError, "COMMAND_SCHEMA_REJECTED"):
            project_observation({"command": {"mode": {"GT": 1}}})

    def test_b3_is_unavailable_without_evidence(self):
        report = build_capability_report("/path/which/does/not/exist")
        self.assertEqual(report["B3"]["status"], "UNAVAILABLE")
        self.assertFalse(calibration_eligible({"calibrated": True}))
        with self.assertRaisesRegex(ValueError, "B3_UNAVAILABLE"):
            project_observation({"motor_current_a": [1.] * 6}, allow_calibrated_effort=True, report=report)
        self.assertFalse(report["signals"]["sdk_effort"]["hardware_verified"])

    def test_invalid_sensor_data_rejected(self):
        with self.assertRaisesRegex(ValueError, "NONFINITE_SENSOR_DATA"):
            project_observation({"q": [float("nan")] * 6})
        with self.assertRaisesRegex(ValueError, "six arm"):
            project_observation({"q": [0.] * 8})

    def test_report_persists_claim_boundary(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_capability_report(tmp, "/absent/sdk")
            report = json.loads((Path(tmp) / "capability_report.json").read_text())
            self.assertFalse(report["claim_boundary"]["real_to_sim_success_proven"])
            self.assertFalse(report["claim_boundary"]["full_relative_slip_observable"])
            self.assertTrue((Path(tmp) / "capability_report.md").is_file())


if __name__ == "__main__":
    unittest.main()
