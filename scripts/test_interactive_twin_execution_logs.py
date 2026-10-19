#!/usr/bin/env python3
"""A later held-out failure must not corrupt completed training log coverage."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from interactive_twin.execution import save_observable_logs
from interactive_twin.sysid import InvalidLog, NoiseScales, noise_normalized_distance, validate_log


DT = 1. / 240
N = 24
JOB = {"episode_id": "synthetic_log_contract_test", "robot_model_id": "frozen_model",
       "robot_calibration_id": "frozen_cal", "controller_id": "frozen_ctrl",
       "physics_protocol": [{"probe_id": p, "duration_s": N * DT} for p in ("P1", "P2", "P3", "P4")]}


def samples(counts):
    rows, commands = [], []
    for phase, count in counts:
        for _ in range(count):
            t = len(rows) * DT
            rows.append({"phase": phase, "t": t, "q": [0.] * 8,
                         "T_tcp": np.eye(4).tolist(), "aperture_m": .02, "contacts": []})
            commands.append({"phase": phase, "t": t, "arm_effort": [0.] * 6,
                             "finger_effort": .5, "finger_position": [.01, -.01]})
    return rows, commands


def export(directory, counts, report, *, mutation=None):
    rows, commands = samples(counts)
    if mutation:
        mutation(rows, commands)
    save_observable_logs(directory, rows, commands, list(range(6)), JOB, report)
    index = json.loads((Path(directory) / "observable/index.json").read_text())
    logs = {p: json.loads(Path(record["path"]).read_text()) for p, record in index.items() if record["path"]}
    return index, logs


class PerProbeCompletion(unittest.TestCase):
    def test_p4_failure_keeps_full_training_segments(self):
        with tempfile.TemporaryDirectory() as output:
            index, logs = export(output, [("P1", N), ("P2", N), ("P3", N), ("P4", 6)],
                                 {"status": "SUSTAINED_CONTACT_LOSS", "failure_phase": "P4"})
            for p in ("P1", "P2", "P3"):
                self.assertTrue(logs[p]["provenance"]["complete"])
                validate_log(logs[p], independent_sim=True)
                self.assertAlmostEqual(index[p]["observed_response_horizon_s"], N * DT)
            self.assertFalse(logs["P4"]["provenance"]["complete"])
            self.assertTrue(index["P4"]["explicit_failure_in_segment"])
            self.assertEqual(index["P4"]["episode_terminal_status"], "SUSTAINED_CONTACT_LOSS")

    def test_training_prefix_replay_has_no_fabricated_p4(self):
        for status in ("REPLAY_COMPLETE", "PHYSICS_REPLAY_COMPLETE"):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as output:
                index, logs = export(output, [("P1", N), ("P2", N), ("P3", N)], {"status": status})
                self.assertTrue(logs["P3"]["provenance"]["complete"])
                self.assertEqual(index["P4"]["status"], "NOT_RUN")
                self.assertNotIn("P4", logs)

    def test_success_label_cannot_complete_a_short_segment(self):
        with tempfile.TemporaryDirectory() as output:
            index, logs = export(output, [("P1", N), ("P2", 6)], {"status": "SUCCESS"})
            self.assertFalse(index["P2"]["complete"])
            with self.assertRaisesRegex(InvalidLog, "complete"):
                validate_log(logs["P2"], independent_sim=True)

    def test_failure_at_last_sample_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as output:
            index, logs = export(output, [("P1", N), ("P2", N)],
                                 {"status": "LOW_JOINT_MARGIN", "failure_phase": "P2"})
            self.assertTrue(logs["P1"]["provenance"]["complete"])
            self.assertFalse(logs["P2"]["provenance"]["complete"])
            self.assertEqual(index["P2"]["samples"], N)

    def test_guard_before_first_p4_sample_keeps_p3(self):
        with tempfile.TemporaryDirectory() as output:
            index, logs = export(output, [("P1", N), ("P2", N), ("P3", N)],
                                 {"status": "CUTOFF_05_00", "failure_phase": "P4",
                                  "first_failure_state": {"phase": "P3"}})
            self.assertTrue(logs["P3"]["provenance"]["complete"])
            self.assertTrue(index["P4"]["explicit_failure_in_segment"])

    def test_missing_command_cannot_be_fabricated(self):
        with tempfile.TemporaryDirectory() as output:
            index, logs = export(output, [("P1", N)], {"status": "REPLAY_COMPLETE"},
                                 mutation=lambda rows, commands: commands.pop())
            self.assertFalse(index["P1"]["complete"])
            self.assertFalse(index["P1"]["paired_command_response"])
            self.assertNotIn("P1", logs)

    def test_later_failure_keeps_adaptive_probe_complete(self):
        with tempfile.TemporaryDirectory() as output:
            index, logs = export(output, [("EXPLORATORY", N), ("P1", N), ("P2", 8)],
                                 {"status": "SUSTAINED_RELATIVE_SLIP", "failure_phase": "P2",
                                  "accepted_estimate_count": 1})
            self.assertTrue(logs["EXPLORATORY"]["provenance"]["complete"])
            self.assertTrue(logs["P1"]["provenance"]["complete"])
            self.assertFalse(logs["P2"]["provenance"]["complete"])

    def test_cartesian_input_hash_does_not_freeze_feedback_torque(self):
        def add_cartesian(rows, commands, tau):
            for command in commands:
                command['cartesian_input']={'reference_world_m':[.4,.1,.2], 'direction_world':[1.,0.,0.], 'active':True}
                command['arm_effort']=[tau]*6
        with tempfile.TemporaryDirectory() as first, tempfile.TemporaryDirectory() as second:
            a, logs_a=export(first,[("P1",N)],{'status':'REPLAY_COMPLETE'},
                            mutation=lambda rows,commands:add_cartesian(rows,commands,1.))
            b, logs_b=export(second,[("P1",N)],{'status':'REPLAY_COMPLETE'},
                            mutation=lambda rows,commands:add_cartesian(rows,commands,2.))
            self.assertEqual(a['P1']['command_hash'],b['P1']['command_hash'])
            self.assertEqual(logs_a['P1']['commands']['kind'],'cartesian_constrained_drive')
            self.assertNotIn('arm_effort_j1',logs_a['P1']['commands']['fields'])
            noise=NoiseScales(2e-5,2e-5,2e-5,.001,.0001,.01,2e-5,calibration_id='frozen_cal')
            result=noise_normalized_distance(logs_a['P1'],logs_b['P1'],noise)
            self.assertEqual(result['tracking_source'],'cartesian_reference')
            self.assertIn('tracking_error_profile_rmse_m',result['metrics'])

    def test_actual_runner_frame_projects_world_reference_and_direction(self):
        # Same keys as issued.append in run_interactive_twin_episode.py; this
        # catches interface drift that internally consistent mock schemas miss.
        def actual_frames(rows,commands):
            for command in commands:
                command.update(compliant=True,arm_position=[0.]*6,arm_velocity=[0.]*6,
                               finger_mode='effort',retention_armed=True,diagnostics={})
                command['cartesian_input']={'direction_world':[0.,1.,0.],
                    'reference_world_m':[.123,.456,.789],'active':False}
        with tempfile.TemporaryDirectory() as output:
            index,logs=export(output,[("P1",N)],{'status':'REPLAY_COMPLETE'},mutation=actual_frames)
            self.assertTrue(index['P1']['complete'])
            self.assertEqual(logs['P1']['commands']['values'][0],[.123,.456,.789,0.,1.,0.,0.,.5,.01,-.01])

    def test_mixed_cartesian_input_is_not_silently_torque_replay(self):
        def partial(rows,commands):
            commands[0]['cartesian_input']={'reference':[0.,0.,0.],'direction':[1.,0.,0.],'active':True}
        with tempfile.TemporaryDirectory() as output:
            index,logs=export(output,[("P1",N)],{'status':'REPLAY_COMPLETE'},mutation=partial)
            self.assertFalse(index['P1']['complete'])
            self.assertEqual(index['P1']['projection_error'],'MIXED_OR_MISSING_CARTESIAN_INPUT')
            self.assertNotIn('P1',logs)


if __name__ == "__main__":
    unittest.main()
