"""Fault injection at the real shared dispatcher; no simulated contact claims."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS,ModuleType
from unittest.mock import Mock,patch
import numpy as np
from agentic_interaction_v81 import recovery_supervisor as supervisor,task_recovery
from interaction_runtime.policy import SafetyStop


def fixture(path):
    r=NS(job=dict(v86_recovery_supervisor=True,v81_active_recovery=True,
        v81_task_goal_native=1.57,v81_minimum_reverse_native=.17,
        articulation_v7=dict(joint_type='revolute')),
        output=Path(path),world=NS(play=Mock()),coordinate=.55,base=np.zeros(4),base_generation=0,
        stopped=False,tick=5,dt=1/240,grasp_reference=None,
        qtarget=np.zeros(13),arm=np.arange(7),filtered=lambda:np.zeros(2),
        rows=[dict(t=1.,aperture_m=.084,wrist_ft=dict(valid=True,raw_wrench_sensor=[0.]*6))],
        robot=NS(get_joint_positions=lambda:np.zeros(13),get_joint_velocities=lambda:np.zeros(13)),
        model=NS(margin=lambda q:1.),policy=NS(data=dict(joint_margin_rad=.05,force=dict(unloaded_pad_n=.05))),
        supervisor=NS(latched=None,force=NS(settled=lambda:True)),
        record=NS(segments=[dict(role='FIT',result=dict(status='TARGET_REACHED'))]),
        station_motion=NS(ready=lambda *args:True),stop_command=Mock(),hold=Mock(),
        episode_budget=NS(remaining=lambda t:dict(wall_s=10000,sim_s=10000)))
    r.actual=lambda:r.coordinate;r.moving=lambda:np.eye(4)
    r.g=dict(recovery_state='UNLOADED_OBSERVATION',reference=None,agent_observations=[])
    ops=NS(r=r,g=r.g,cache={1:'old'},current_plan_ready=True,abc_state='A',_abc_retain=True)
    def park():
        assert ops._abc_retain is False
        r.grasp_reference=None;ops.g['reference']=None
        return dict(safe=True,park=dict(reached=True,unloaded=True,measured_collision_check=dict(valid=True)))
    ops.release_park=Mock(side_effect=park)
    ops.observe=Mock(side_effect=lambda:dict(measured_coordinate_native=r.actual()))
    return ops


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.ops=fixture(self.directory.name)

    def finish(self,op,result):
        command=dict(operation=op,sequence=0)
        return task_recovery.after(self.ops,command,supervisor.complete(self.ops,command,result))

    def test_rejected_ik_collision_and_margin_preserve_world_history_and_goal(self):
        world=self.ops.r.world;history=self.ops.r.record.segments
        for reason in ('NO_CURRENT_PREGRASP_IK','DANGEROUS_GEOMETRY_COLLISION:link5:door',
                       'MOTION_PATH_UNAVAILABLE:EXECUTED_SAMPLE:JOINT_MARGIN'):
            with self.subTest(reason=reason):
                result=self.finish('prepare_grasp',dict(plan=False,reason=reason))
                self.assertEqual(result['outcome'],supervisor.ACTION_REJECTED)
                self.assertFalse(result['task_recovery']['episode_termination_required'])
                self.assertIs(self.ops.r.world,world);self.assertIs(self.ops.r.record.segments,history)
                self.assertEqual(self.ops.r.coordinate,.55);self.assertEqual(self.ops.r.job['v81_task_goal_native'],1.57)
                self.assertEqual(result['recovery']['fresh_observation']['measured_coordinate_native'],.55)
        self.assertEqual(self.ops.observe.call_count,3)

    def test_actual_preflight_margin_error_becomes_action_rejection(self):
        c=dict(operation='reobserve_and_replan',sequence=0)
        raw=supervisor.exception_result(self.ops,c,RuntimeError('MOTION_PATH_UNAVAILABLE:EXECUTED_SAMPLE:JOINT_MARGIN'))
        result=self.finish(c['operation'],raw)
        self.assertEqual(result['outcome'],supervisor.ACTION_REJECTED)
        self.assertFalse(result['task_recovery']['episode_termination_required'])

    def test_transmission_failure_unloads_without_reset_and_requires_fresh_plan(self):
        self.ops.r.grasp_reference=object();self.ops.g['reference']=object()
        self.ops.g['recovery_state']='LOADED_CRITIQUE'
        result=self.finish('acquire_execute',dict(segments=[dict(status='TRANSMISSION_FAILURE')]))
        self.assertEqual(result['outcome'],supervisor.RECOVERABLE_FAILURE)
        self.assertTrue(result['recovery']['park_verified']);self.assertFalse(result['recovery']['object_state_reset'])
        self.assertFalse(self.ops.current_plan_ready);self.assertEqual(self.ops.abc_state,'A')
        self.assertEqual(self.ops.r.coordinate,.55)

    def test_worker_timeout_is_deferred_without_unloading(self):
        c=dict(operation='retry_model',sequence=0)
        result=self.finish(c['operation'],supervisor.exception_result(self.ops,c,TimeoutError('numerical worker')))
        self.assertTrue(result['recovery']['worker_deferred'])
        self.ops.release_park.assert_not_called();self.ops.r.stop_command.assert_not_called()

    def test_no_exit_path_retains_verified_released_hold_for_new_checked_exit(self):
        self.ops.release_park=Mock(return_value=dict(safe=False,reason='TIMED_RETURN_ROUTE_UNAVAILABLE:collision'))
        result=self.finish('prepare_grasp',dict(plan=False,reason='IK'))
        self.assertEqual(result['task_recovery']['status'],'RECOVERY_EXIT_PENDING')
        self.assertFalse(result['task_recovery']['episode_termination_required'])
        self.assertIsNone(supervisor.before(self.ops,dict(operation='observe_view')))
        for op in ('acquire_execute','relocate_station'):
            self.assertFalse(supervisor.before(self.ops,dict(operation=op))['executed'])

    def test_unconfirmed_hold_is_unrecoverable_and_does_not_fake_park(self):
        self.ops.release_park=Mock(return_value=dict(safe=False));self.ops.r.station_motion.ready=lambda *a:False
        result=self.finish('prepare_grasp',dict(plan=False))
        self.assertEqual(result['outcome'],supervisor.UNRECOVERABLE)
        self.assertTrue(result['task_recovery']['episode_termination_required'])
        self.assertFalse(result['recovery']['safe_park_verified'])

    def test_checked_exit_success_clears_only_pending_exit_not_historical_latches(self):
        self.ops._v86_exit_pending=True
        result=supervisor.complete(self.ops,dict(operation='safe_release_retreat'),
            dict(safe=True,park=dict(reached=True,unloaded=True,measured_collision_check=dict(valid=True))))
        self.assertTrue(result['safe']);self.assertFalse(self.ops._v86_exit_pending)
        self.assertIsNone(supervisor.before(self.ops,dict(operation='acquire_execute')))

    def test_hard_stop_latch_and_untrusted_ft_never_auto_clear_or_command(self):
        latch=dict(reason='FORCE_HARD_LIMIT');self.ops.r.supervisor.latched=latch
        result=self.finish('acquire_execute',dict(status='SAFETY_STOP',reason='FORCE_HARD_LIMIT'))
        self.assertTrue(result['task_recovery']['episode_termination_required'])
        self.assertIs(self.ops.r.supervisor.latched,latch);self.ops.release_park.assert_not_called()
        self.ops.r.supervisor.latched=None;self.ops.r.rows[-1]['wrist_ft']['valid']=False
        result=self.finish('prepare_grasp',dict(plan=False))
        self.assertEqual(result['outcome'],supervisor.UNRECOVERABLE)
        self.ops.r.stop_command.assert_not_called()

    def test_unknown_actuator_error_and_native_safety_stop_are_not_reclassified(self):
        c=dict(operation='move_and_observe',sequence=0)
        result=supervisor.exception_result(self.ops,c,RuntimeError('CONTROLLER_LOST'))
        self.assertEqual(result['outcome'],supervisor.UNRECOVERABLE)
        result=supervisor.exception_result(self.ops,c,SafetyStop('NATIVE_COLLISION'))
        self.assertEqual(result['outcome'],supervisor.SAFETY_STOP)

    def test_advisory_warning_does_not_invoke_recovery(self):
        result=dict(status='TARGET_REACHED',advisory='low pad confidence',physical_transmission_valid=True)
        self.assertIs(supervisor.complete(self.ops,dict(operation='acquire_execute'),result),result)
        self.ops.release_park.assert_not_called()

    def test_changed_moving_geometry_or_attachment_rejects_stale_grasp(self):
        from agentic_interaction_v81.scene_stamp import capture
        self.ops._v86_plan_scene=capture(self.ops.r)
        self.assertIsNone(supervisor.before(self.ops,dict(operation='acquire_execute')))
        self.ops.r.coordinate=.8
        self.assertEqual(supervisor.before(self.ops,dict(operation='acquire_execute'))['outcome'],'ACTION_REJECTED')
        self.ops.r.coordinate=.55;self.ops.r.grasp_reference=object()
        self.assertEqual(supervisor.before(self.ops,dict(operation='acquire_execute'))['outcome'],'ACTION_REJECTED')

    def test_real_session_continues_after_initial_view_and_action_failures(self):
        from agentic_interaction_v81.session import serve
        ops=self.ops;r=ops.r;world=r.world;events=[]
        ops.decision_packet=lambda *a:dict(unloaded=True)
        ops.articulation=lambda:NS(units={},validate_target=lambda a:True)
        ops.close_workers=Mock(side_effect=lambda:events.append('closed'))
        ops.prepare_grasp=Mock(side_effect=[RuntimeError('NO_CURRENT_PREGRASP_IK'),dict(plan=False,reason='collision'),dict(plan=True)])
        ops.feedback=lambda:{}
        def attempt(c):
            r.coordinate=1.4
            r.v81_task_progress=dict(maximum_measured_native=1.58,first_goal_record=10)
            r.record.segments.append(dict(role='VALIDATION_NEW',first_step=11,
                result=dict(status='TARGET_REACHED',gain_native=-.18,requested_target_reached=True,physical_transmission_valid=True)))
            return dict(safe_exit=ops.release_park())
        ops.attempt=attempt
        commands=[dict(operation='prepare_grasp'),dict(operation='prepare_grasp',candidate_request=2),
                  dict(operation='finish'),dict(operation='prepare_grasp',candidate_request=3),
                  dict(operation='attempt_grasp_and_probe'),dict(operation='finish')]
        for seq,c in enumerate(commands):
            c['sequence']=seq
        (r.output/'agent_commands').mkdir()
        for i,c in enumerate(commands):(r.output/'agent_commands'/('%03d.json'%i)).write_text(json.dumps(c))
        observed=ModuleType('interaction_runtime.registered_observation_v71')
        observed.initialize=Mock(side_effect=RuntimeError('MOTION_PATH_UNAVAILABLE:EXECUTED_SAMPLE:JOINT_MARGIN'))
        owner=ModuleType('interaction_runtime.probing_v7');owner.OperationsV7=object
        with patch.dict(sys.modules,{'interaction_runtime.registered_observation_v71':observed,'interaction_runtime.probing_v7':owner}), \
             patch('agentic_interaction_v81.session.operations_class',return_value=lambda runtime:ops), \
             patch('agentic_interaction_v81.session.window',return_value=(20,{})), \
             patch('agentic_interaction_v81.session.status'), \
             patch('agentic_interaction_v81.session.receive',side_effect=lambda p,i,res:(commands[i],{})), \
             patch('agentic_interaction_v81.session.complete'):
            result=serve(r)
        self.assertEqual(result['status'],'SESSION_COMPLETE');observed.initialize.assert_called_once()
        self.assertIs(r.world,world);self.assertEqual(len(result['trace']),7)
        self.assertEqual(result['trace'][3]['result']['status'],'CONTINUATION_REQUIRED')
        self.assertEqual(result['trace'][-1]['result']['outcome'],'EPISODE_COMPLETE')
        self.assertEqual(events,['closed']);self.assertEqual(ops.prepare_grasp.call_count,3)


if __name__=='__main__':unittest.main()
