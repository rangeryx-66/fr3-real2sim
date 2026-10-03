"""Orchestration contract test with fake rollouts; no simulation-success claim."""
import ast
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_interactive_twin_benchmark import Benchmark, read, write


class ReplayGraspSafety(unittest.TestCase):
    """Execute the real runner's guard blocks without importing Isaac.

    The fixtures supply only grip-window results. This protects the boundary
    between completed command replay and legal physical grasp; it does not
    substitute for the actual contact experiment.
    """
    @staticmethod
    def _runner_branch(predicate):
        path = Path(__file__).with_name("run_interactive_twin_episode.py")
        tree = ast.parse(path.read_text(), filename=str(path))
        matches = [node for node in ast.walk(tree) if isinstance(node, ast.If) and predicate(node)]
        if len(matches) != 1:
            raise AssertionError("Expected exactly one replay safety branch in the actual runner")
        branch = deepcopy(matches[0])
        # Execute only the selected actual branch. Its unrelated robot/Isaac
        # alternatives are intentionally outside this lightweight contract.
        branch.orelse = []
        return compile(ast.fix_missing_locations(ast.Module(body=[branch], type_ignores=[])),
                       str(path), "exec")

    def test_first_armed_frame_requires_bilateral_hold_before_retention(self):
        code = self._runner_branch(lambda node: "retention_armed" in ast.unparse(node.test)
                                   and "reference is None" in ast.unparse(node.test))
        for ready in (False, True):
            with self.subTest(ready=ready):
                retention = Mock()
                scope = {"replay_frame": {"retention_armed": True}, "reference": None,
                         "legal": False, "retention": retention, "tcp": lambda: np.eye(4),
                         "grip_window": lambda: (ready, {"diagnostic": "test-only"})}
                if ready:
                    exec(code, scope)
                    retention.arm.assert_called_once_with()
                    self.assertTrue(scope["reference"])
                    self.assertTrue(scope["legal"])
                else:
                    with self.assertRaisesRegex(RuntimeError, "REPLAY_BILATERAL_HOLD_NOT_ESTABLISHED"):
                        exec(code, scope)
                    retention.arm.assert_not_called()
                    self.assertIsNone(scope["reference"])
                    self.assertFalse(scope["legal"])

    def test_final_replay_hold_failure_cannot_be_labelled_complete(self):
        code = self._runner_branch(lambda node: ast.unparse(node.test) == "tape is not None"
                                   and any(isinstance(child, ast.Constant) and child.value == "REPLAY_COMPLETE"
                                           for statement in node.body for child in ast.walk(statement)))
        for ready in (False, True):
            with self.subTest(ready=ready):
                scope = {"tape": [{}], "tick": 0, "step": Mock(), "legal": False,
                         "status": "STARTED", "success": False,
                         "grip_window": lambda: (ready, {"diagnostic": "test-only"})}
                if ready:
                    exec(code, scope)
                    self.assertEqual(scope["status"], "REPLAY_COMPLETE")
                    self.assertTrue(scope["legal"])
                else:
                    with self.assertRaisesRegex(RuntimeError, "REPLAY_FINAL_HOLD_LOST"):
                        exec(code, scope)
                    self.assertNotEqual(scope["status"], "REPLAY_COMPLETE")
                    self.assertFalse(scope["legal"])
                scope["step"].assert_called_once_with()


class FitHeldoutBoundary(unittest.TestCase):
    def test_training_does_not_execute_or_select_on_p4(self):
        self._boundary()

    def test_nonzero_reference_is_preserved_but_twin_replay_starts_at_zero(self):
        self._boundary(q_initial=.05)

    def test_kinematic_twin_survives_unidentifiable_physics(self):
        self._boundary(physics_available=False)

    def _boundary(self, q_initial=0., physics_available=True):
        from test_interactive_twin_artifacts import URDF
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); asset = root / "asset"
            (asset / "urdf").mkdir(parents=True)
            (asset / "urdf/test.urdf").write_text(URDF)
            write(asset / "manifest.json", {"asset_id": "test", "joint_name": "hinge", "moving_link": "door",
                "door_link": "handle", "moving_links": ["door", "handle", "handle_tip"], "scale_source_to_meters": 1., "moving_source_bounds": [[-.2,-.1,-.1],[.6,.2,.8]]})
            b = Benchmark.__new__(Benchmark)
            b.output = root / "benchmark"
            b.config = {"physics": {"initial_prior": {"tau_c": .007, "b": .6},
                "grid": {"tau_c": [.001, .01], "b": [.1, 1.]}, "maximum_candidates": 4,
                "operational_joint_limits_rad": [-.1745329252, .1745329252]},
                "policy": {"seed": 11, "budgets": {"sysid_simulations": 32}}}
            b.calibration = {"calibration_id": "cal-test", "noise_scales": {}}
            b.sensitivity = {"status": "OBSERVABLE_RESPONSE" if physics_available else "UNIDENTIFIABLE_UNDER_CURRENT_PROBE"}
            b.expired = lambda: False
            episode = {"episode_id": "test_0", "seed": 11, "asset_id": "test",
                "budgets": {"sysid_simulations": 32, "wall_clock_seconds": 3600}}
            out = b.output / "episodes/test_0"
            write(out / "episode_summary.json", {"stages": {key: {"status": "SUCCESS" if key == "D" else "NOT_RUN"} for key in "ABCDEFGHI"},
                "selected_job": {"asset_root": str(asset), "initial_articulation_rad": q_initial}})
            fit = {"joint_type": "revolute", "confidence": .99,
                "revolute": {"axis": [0, 0, 1], "point_on_axis": [.2, .1, .3], "radius_m": .3}}
            selected_folder = out / "selected_kinematics"
            write(selected_folder / "initial_scene_private.json", {"asset_rotation": np.eye(3).tolist(), "asset_xyz": [0, 0, 0], "fixture": {"frozen": True}})
            write(selected_folder / "structured_memory.json", {"fit_history": [{"accepted": True, "fit": fit, "observation_count": 2}], "supporting_observations": [np.eye(4).tolist()] * 5, "attempt_history": []})
            summary = read(out / "episode_summary.json"); summary["selected_job"]["output"] = str(selected_folder); write(out / "episode_summary.json", summary)
            calls = []; fitted = [False]; reference_plant = [None]

            def fake_run(job):
                calls.append(job)
                directory = Path(job["output"])
                if job["mode"] == "physics_reference":
                    self.assertEqual(job["initial_articulation_rad"], q_initial)
                    reference_plant[0] = job["plant"]
                    write(directory / "initial_scene_private.json", {"asset_rotation": np.eye(3).tolist(), "asset_xyz": [0, 0, 0], "fixture": {"frozen": True}})
                    write(directory / "structured_memory.json", {"fit_history": [{"accepted": True, "fit": fit, "observation_count": 2}],
                        "supporting_observations": [np.eye(4).tolist()] * 5, "attempt_history": []})
                    write(directory / "command_tape.json", [{"phase": p} for p in ("SETTLE", "P1", "P2", "P3", "P4", "FINAL_HOLD")])
                    for probe in ("P1", "P2", "P3", "P4"):
                        write(directory / "observable" / (probe + ".json"), {"probe_id": probe, "provenance": {"complete": True}})
                    return {"status": "PHYSICS_PROTOCOL_COMPLETE"}
                if job["mode"] == "updated_interaction":
                    self.assertEqual(job["plant"], reference_plant[0])
                    self.assertEqual(read(job["initial_estimate"]), fit)
                    return {"status": "SUCCESS", "success": True, "evaluation": {"actual_final_door_angle_deg": 5.2}}
                self.assertEqual(job["initial_articulation_rad"], q_initial if directory.name=="Oracle" else 0.)
                self.assertTrue(job.get("reference_safety_memory"))
                tape = read(job["replay_commands"])
                if "physics_candidates" in str(directory):
                    self.assertFalse(fitted[0])
                    self.assertEqual(tape[-1]["phase"], "P3")
                    self.assertNotIn("P4", [frame["phase"] for frame in tape])
                    for probe in ("P1", "P2", "P3"):
                        write(directory / "observable" / (probe + ".json"), {"probe_id": probe, "provenance": {"complete": True}})
                else:
                    self.assertTrue(fitted[0])
                    self.assertIn("P4", [frame["phase"] for frame in tape])
                    if directory.name in ("B0", "B1"):
                        self.assertEqual(job["plant"], {"tau_c": .007, "b": .6})
                    write(directory / "observable/P4.json", {"probe_id": "P4", "provenance": {"complete": True}})
                return {"status": "REPLAY_COMPLETE"}

            def fake_fit(reference, candidates, noise, **kwargs):
                self.assertEqual(set(reference), {"P1", "P2", "P3"})
                self.assertEqual(kwargs["J_eff_prior"]["kind"], "fixed_articulated_model")
                self.assertIsNone(kwargs["J_eff_prior"]["scalar_J_eff"])
                for candidate in candidates:
                    self.assertEqual(set(candidate["probes"]), {"P1", "P2", "P3"})
                    self.assertEqual(candidate["J_eff_prior"], kwargs["J_eff_prior"])
                fitted[0] = True
                return {"status": "IDENTIFIABLE_ON_FROZEN_GRID", "accepted_parameters": {"tau_c": .001, "b": .1},
                    "parameter_intervals": {"tau_c": [.001, .001], "b": [.1, .1]}, "J_eff_prior_fixed": kwargs["J_eff_prior"]}

            def fake_comparison(reference, predictions, noise):
                self.assertTrue(fitted[0]);self.assertEqual(set(predictions), {"B0", "B1", "B2", "Oracle"})
                return {"methods": {name: {"status": "EVALUATED"} for name in predictions}, "B2_improvement": {}}

            b.run = fake_run
            with patch("interactive_twin.sysid.validate_log", side_effect=lambda x: x), \
                    patch("interactive_twin.sysid.NoiseScales", side_effect=lambda **kw: None), \
                    patch("interactive_twin.sysid.fit_resistance", side_effect=fake_fit), \
                    patch("interactive_twin.sysid.heldout_comparison", side_effect=fake_comparison):
                result = b.fit_episode(episode, 1)
            if not physics_available:
                self.assertEqual(result["stages"]["G"]["status"], "SUCCESS")
                self.assertFalse(result["stages"]["G"]["T2_physics_accepted"])
                metadata=read(Path(result["stages"]["G"]["versions"]["T2"]["asset_root"])/"twin.json")
                self.assertEqual(metadata["physics"]["status"], "NOT_IDENTIFIED")
                self.assertEqual(calls, [])
                self.assertEqual(result["stages"]["H"]["status"], "BLOCKED")
                return
            self.assertEqual(result["stages"]["I"]["status"], "SUCCESS")
            self.assertFalse(result["stages"]["I"]["physics_model_used_by_controller"])
            memory = read(out / "twins_final/structured_memory.json")
            self.assertEqual(memory["supporting_ee_observation_count"], 2)
            self.assertEqual(read(out / "heldout_comparison.json")["methods"]["B3"]["status"], "UNAVAILABLE")
            self.assertEqual(len([j for j in calls if "physics_candidates" in j["output"]]), 4)
            self.assertEqual(len([j for j in calls if "/heldout/" in j["output"]]), 4)


class BudgetAndProjection(unittest.TestCase):
    def test_asset_budget_is_shared_between_configurations(self):
        with tempfile.TemporaryDirectory() as tmp:
            b = Benchmark.__new__(Benchmark); b.output = Path(tmp)
            first = {'budget_asset_id': '42', 'asset_wall_clock_budget_s': 10,
                'asset_sysid_simulation_budget': 2, 'episode_sysid_simulation_budget': 2,
                'episode_id': '42_00', 'output': str(Path(tmp) / 'one'), 'mode': 'replay'}
            b._budget(first, 'reserve')
            second = {**first, 'episode_id': '42_01', 'output': str(Path(tmp) / 'two')}
            with self.assertRaisesRegex(RuntimeError, 'ALREADY_HAS_RUNNING_JOB'):
                b._budget(second, 'reserve')
            b._budget(first, 'finish', 3.)
            self.assertEqual(b._budget(second, 'reserve')['remaining_wall_s'], 7.)
            b._budget(second, 'finish', 2.)
            third = {**first, 'episode_id': '42_02', 'output': str(Path(tmp) / 'three')}
            with self.assertRaisesRegex(RuntimeError, 'SYSID_BUDGET_EXHAUSTED'):
                b._budget(third, 'reserve')
            state = read(Path(tmp) / 'asset_budgets/42.json')
            self.assertEqual(state['sysid_launches'], 2)
            self.assertEqual(state['active_wall_seconds'], 5.)
            self.assertEqual(set(state['episode_sysid_launches']), {'42_00', '42_01'})

    def test_raw_projection_preserves_measurements_and_uses_cartesian_u(self):
        from interactive_twin.execution import save_observable_logs
        with tempfile.TemporaryDirectory() as tmp:
            b = Benchmark.__new__(Benchmark); root = Path(tmp); source = root / 'source'; source.mkdir()
            b.calibration = {'calibration_id': 'frozen-cal', 'robot_model_id': 'f5dcc6f-frozen-piper',
                'controller_id': 'cd61660-constrained-probe+physics-protocol-v1'}
            arm = [4, 0, 5, 1, 6, 2]
            rows = []; commands = []
            for i in range(12):
                rows.append({'phase': 'EXPLORATORY', 't': i/240,
                    'q': (np.arange(8)*.1 + i*.00001*np.arange(1,9)).tolist(),
                    'T_tcp': np.eye(4).tolist(), 'aperture_m': .03})
                commands.append({'phase': 'EXPLORATORY', 't': i/240, 'arm_effort': [float(i)]*6,
                    'finger_effort': 1., 'finger_position': [.015, -.015]})
            job = {'episode_id': 'legacy_DEV', 'robot_calibration_id': 'PENDING'}
            report = {'status': 'REPLAY_COMPLETE', 'accepted_estimate_count': 1}
            save_observable_logs(source, rows, commands, arm, job, report)
            original_q = read(source/'observable/EXPLORATORY.json')['signals']['q_rad']
            for frame in commands:
                frame['cartesian_input'] = {'reference_world_m': [.4, .1, .2], 'direction_world': [1,0,0], 'active': True}
            write(source/'observations.json', rows); write(source/'command_tape.json', commands)
            write(source/'job_private.json', job); write(source/'report.json', report)
            before = (source/'observations.json').read_bytes()
            derived = root/'derived'
            audit = b._refresh_observable_import({'run_dir': str(source)}, derived, analysis_episode_id='dev_7320_00')
            projected = read(derived/'observable/EXPLORATORY.json')
            self.assertEqual(audit['arm_indices'], arm)
            self.assertEqual(projected['signals']['q_rad'], original_q)
            self.assertEqual(projected['commands']['kind'], 'cartesian_constrained_drive')
            self.assertNotIn('arm_effort_j1', projected['commands']['fields'])
            self.assertEqual(projected['episode_id'], 'dev_7320_00')
            self.assertEqual(projected['provenance']['robot_calibration_id'], 'frozen-cal')
            self.assertEqual(before, (source/'observations.json').read_bytes())
            self.assertEqual(read(source/'observable/EXPLORATORY.json')['provenance']['robot_calibration_id'], 'PENDING')

    def test_cache_identity_changes_with_plan_and_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = root/'source'; asset = root/'asset'
            write(source/'report.json', {'initial': True}); write(asset/'manifest.json', {'asset_id': '42'})
            plan = root/'plan.json'; write(plan, {'T': [1,2,3]})
            b = Benchmark.__new__(Benchmark); b._code_identity=lambda: {'implementation': 'abc'}
            job = {'source': str(source), 'asset_root': str(asset), 'mode': 'replay',
                'plan': str(plan), 'plant': {'tau_c': 0., 'b': 0.}}
            a = b._job_identity(job)['sha256']
            self.assertNotEqual(a, b._job_identity({**job, 'plant': {'tau_c': .01, 'b': 0.}})['sha256'])
            write(plan, {'T': [1,2,4]})
            self.assertNotEqual(a, b._job_identity(job)['sha256'])


if __name__ == "__main__":
    unittest.main()
