"""Portable contract tests for the read-only agent handoff wrapper."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("make_handoff.py")


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def put(self, name, value):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(value, dict):
            path.write_text(json.dumps(value))
        else:
            path.write_bytes(value)
        return path

    def run_command(self, *arguments, success=True):
        result = subprocess.run([sys.executable, str(SCRIPT), *map(str, arguments)], capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stderr)
        return result

    def grasp(self):
        result = self.put("grasp.json", {
            "target": "sample", "seed": 7, "mode": "ANYGRASP", "success": True, "category": "SUCCESS",
            "flags": {"PICKED": True, "RETAINED": True, "CLEAR_TABLE": True, "lift_ge_8cm": True, "DROP": False},
            "final_support": {"passed": True, "currently_clear": True},
        })
        output = self.root / "grasp_handoff.json"
        self.run_command("grasp", "--result", result, "--output", output)
        return result, output

    def test_grasp_requires_physical_success(self):
        result, handoff = self.grasp()
        self.assertEqual(json.loads(handoff.read_text())["stage"], "GRASP_STABLE")
        row = json.loads(result.read_text())
        row["flags"]["RETAINED"] = False
        result.write_text(json.dumps(row))
        self.run_command("grasp", "--result", result, "--output", handoff, success=False)
        self.assertEqual(json.loads(handoff.read_text())["stage"], "GRASP_STABLE")

    def test_station_requires_source_frames_and_continuity_evidence(self):
        views = []
        for index in range(2):
            files = {name: str(self.put(f"{name}/{index}.png", b"image")) for name in
                     ("rgb", "depth", "object_mask", "gripper_mask")}
            views.append({**files, "frame_id": index, "quality": {"accepted": True}, "intrinsics": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                          "T_B_camera": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]})
        manifest = self.put("scan_manifest.json", {"schema": "fr3_scan_station/v1", "target": "sample",
                         "state": "placed_released_arm_parked", "config": {"minimum_accepted_views": 2},
                         "accepted_count": 2, "rejected_count": 0, "views": views})
        output = self.root / "scan_handoff.json"
        self.run_command("scan", "--manifest", manifest, "--output", output)
        self.assertEqual(json.loads(output.read_text())["checks"]["identity_continuity"], "INDEPENDENT_SCENE")
        selection = self.put("selection.json", {"schema": "mv_sam3d_input_selection/v1", "object": "sample",
                             "selections": {"2": {"view_count": 2, "selected": [
                                 {"source_frame_id": i, "area_fraction": 0.3, "T_B_camera": views[i]["T_B_camera"]}
                                 for i in range(2)]}}})
        glb = self.put("result.glb", b"glb")
        mv_output = self.root / "mv_handoff.json"
        self.run_command("mv", "--scan-handoff", output, "--selection", selection,
                         "--glb", glb, "--views", 2, "--output", mv_output)
        self.assertFalse(json.loads(mv_output.read_text())["checks"]["physics_ready"])
        Path(views[0]["depth"]).unlink()
        self.run_command("scan", "--manifest", manifest, "--output", output, success=False)

    def test_payload_rejected_fit_is_fallback(self):
        _, grasp_handoff = self.grasp()
        summary = self.put("summary.json", {"sample": {"accepted_poses": 5, "methods": {"drake": {"fit": {
            "accepted": 0, "mass_kg": 1.0, "center_of_mass_m": [0, 0, 0], "reason": "STATIC_UNCERTAIN"}}}}})
        output = self.root / "payload_handoff.json"
        self.run_command("payload", "--grasp-handoff", grasp_handoff, "--summary", summary, "--output", output)
        row = json.loads(output.read_text())
        self.assertEqual(row["stage"], "PAYLOAD_FALLBACK")
        self.assertNotIn("center_of_mass_m", row["checks"])
        fit = json.loads(summary.read_text())
        fit["sample"]["methods"]["drake"]["fit"]["accepted"] = 1
        summary.write_text(json.dumps(fit))
        self.run_command("payload", "--grasp-handoff", grasp_handoff, "--summary", summary, "--output", output)
        self.assertEqual(json.loads(output.read_text())["stage"], "PAYLOAD_IDENTIFIED")


if __name__ == "__main__":
    unittest.main()
