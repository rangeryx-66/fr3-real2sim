import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from test_recovery_supervisor_v86 import fixture
from agentic_interaction_v81 import recovery_supervisor as supervisor,task_recovery
from agentic_interaction_v87.recovery import annotate_interrupted_end
from agentic_interaction_v87.lifecycle import retain
from agentic_interaction_v84.transport import publish
from agent_command_protocol_v12 import ProtocolError


class RecoveryV87Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.ops=fixture(self.tmp.name);r=self.ops.r
        r.job.update(v87_continuous_recovery=True,safe_exit_wall_reserve_s=300,episode_id='unit_v87')
        r.g['qvelocity']=np.zeros(7);r.fingers=np.arange(7,13)
        r.collision=NS(check=Mock(return_value=(True,None)))
        r.model.poses=Mock(return_value={})

    def test_pure_planning_rejection_does_not_repeat_physical_parking(self):
        ops=self.ops;r=ops.r;world=r.world
        out=supervisor.complete(ops,dict(operation='prepare_grasp'),dict(plan=False,reason='IK'))
        self.assertEqual(out['outcome'],'ACTION_REJECTED')
        self.assertTrue(out['recovery']['pure_planning_rejection'])
        r.stop_command.assert_not_called();ops.release_park.assert_not_called();ops.observe.assert_not_called()
        self.assertIs(r.world,world);self.assertEqual(r.coordinate,.55)

    def test_controlled_error_reuses_verified_exit_and_continues_task(self):
        ops=self.ops;r=ops.r;c=dict(operation='acquire_execute',sequence=0)
        ops._abc_retain=False
        parked=ops.release_park();ops.release_park.reset_mock()
        out=supervisor.complete(ops,c,dict(error="RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:test')",safe_exit=parked))
        out=task_recovery.after(ops,c,out)
        self.assertEqual(out['outcome'],'RECOVERABLE_FAILURE')
        self.assertFalse(out['task_recovery']['episode_termination_required'])
        r.stop_command.assert_not_called();ops.release_park.assert_not_called();ops.observe.assert_called_once()
        self.assertIsNone(r.supervisor.latched)

    def test_interrupted_segment_excluded_before_worker_end_callback(self):
        end=Mock();wrapped=annotate_interrupted_end(end)
        try:raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:test')
        except RuntimeError:wrapped({})
        result=end.call_args.args[0]
        self.assertFalse(result['physical_transmission_valid'])
        self.assertTrue(result['FIT_exclude_entire_segment'])
        self.assertFalse(result['complete_independent_validation'])

    def test_untrusted_ft_cannot_use_no_motion_optimization(self):
        self.ops.r.rows[-1]['wrist_ft']['valid']=False
        out=supervisor.complete(self.ops,dict(operation='prepare_grasp'),dict(plan=False))
        self.assertEqual(out['outcome'],'UNRECOVERABLE')
        self.ops.release_park.assert_not_called()

    def test_hard_latch_keeps_world_until_diagnostic_owner_finishes(self):
        ops=self.ops;r=ops.r;latch=dict(reason='PHASE_TCP_VELOCITY');r.supervisor.latched=latch
        playing=[True];r.world.pause=Mock(side_effect=lambda:playing.__setitem__(0,False))
        r.world.is_playing=lambda:playing[0];r.world.close=Mock();r.world.reset=Mock()
        (r.output/'agent_commands').mkdir();(r.output/'agent_responses').mkdir()
        commands=[dict(operation='inspect'),dict(operation='observe'),dict(operation='finish')]
        for i,c in enumerate(commands):
            c.update(protocol_version=1,sequence=i,reason='registered lifecycle fault test')
            (r.output/'agent_commands'/('%03d.json'%i)).write_text(json.dumps(c))
        out=retain(ops,0,[],dict(status='SAFETY_STOP',reason=latch['reason']))
        self.assertEqual(out['status'],'HARD_STOP')
        first=out['trace'][0]['result'];self.assertEqual(first['status'],'SAFETY_LATCHED_WORLD_RETAINED')
        self.assertFalse(first['motion_authorized']);self.assertFalse(first['reset_qualification']['qualified'])
        self.assertEqual(out['trace'][1]['result']['status'],'HARD_LATCH_MOTION_FORBIDDEN')
        self.assertIs(r.supervisor.latched,latch);r.world.close.assert_not_called();r.world.reset.assert_not_called()
        r.stop_command.assert_not_called();ops.release_park.assert_not_called()

    def test_transport_does_not_label_hard_diagnostic_boundary_unloaded(self):
        r=self.ops.r;(r.output/'agent_commands').mkdir()
        import time
        state=dict(sequence=3,state='WAITING',unloaded=False,abc_contact=False,
            diagnostic_only=True,safety_latched=True,deadline_unix_s=time.time()+30)
        (r.output/'AGENT_SESSION_STATUS.json').write_text(json.dumps(state))
        c=dict(operation='observe',protocol_version=1,sequence=3,reason='forbidden motion')
        with self.assertRaisesRegex(ProtocolError,'HARD_LATCH'):publish(r.output,3,c)
        c.update(operation='inspect',reason='read-only diagnosis')
        self.assertIn('published',publish(r.output,3,c))

    def test_unconfirmed_stop_retains_finite_world_without_claiming_hard_force_event(self):
        ops=self.ops;r=ops.r;playing=[True]
        r.world.pause=Mock(side_effect=lambda:playing.__setitem__(0,False))
        r.world.is_playing=lambda:playing[0];r.world.close=Mock()
        (r.output/'agent_commands').mkdir();(r.output/'agent_responses').mkdir()
        for i,name in enumerate(['inspect','finish']):
            (r.output/'agent_commands'/('%03d.json'%i)).write_text(json.dumps(dict(
                operation=name,protocol_version=1,sequence=i,reason='stop-unconfirmed regression')))
        failed=dict(status='CONTROL_STATE_UNTRUSTED',outcome='UNRECOVERABLE',
            recovery=dict(reason='CONTROLLED_EXECUTION_INTERRUPT:TCP_PROJECTION_INFEASIBLE'))
        outcome=retain(ops,0,[],failed)
        report=outcome['trace'][0]['result']
        self.assertEqual(report['World_identity'],id(r.world))
        self.assertFalse(report['motion_authorized']);self.assertFalse(report['physical_park_verified'])
        self.assertEqual(r.supervisor.latched['reason'],'CONTROLLED_STOP_OR_STATE_UNCONFIRMED')
        self.assertIn('not a measured force/speed',r.supervisor.latched['source'])
        r.world.close.assert_not_called();ops.release_park.assert_not_called()

    def test_real_dispatcher_delegates_hard_fault_to_retained_diagnosis(self):
        import sys
        from types import ModuleType
        from interaction_runtime.policy import SafetyStop
        from agentic_interaction_v81.session import serve
        ops=self.ops;r=ops.r;playing=[True]
        r.world.pause=Mock(side_effect=lambda:playing.__setitem__(0,False))
        r.world.is_playing=lambda:playing[0];r.world.close=Mock()
        ops.close_workers=Mock();ops.decision_packet=Mock(return_value=dict(unloaded=True))
        ops.articulation=lambda:NS(units={},validate_target=lambda a:True)
        ops.prepare_grasp=Mock(side_effect=SafetyStop('PHASE_TCP_VELOCITY'))
        commands=[dict(operation='prepare_grasp',planned_segments=[dict(role='VALIDATION_SHORT',
            coordinate_unit='rad',target_q=.1,reference_speed_m_s=.01,
            reference_acceleration_m_s2=.03,profile='constant')]),dict(operation='inspect'),dict(operation='finish')]
        (r.output/'agent_commands').mkdir()
        for i,c in enumerate(commands):
            c.update(protocol_version=1,sequence=i,reason='fault-injection lifecycle regression')
            (r.output/'agent_commands'/('%03d.json'%i)).write_text(json.dumps(c))
        observed=ModuleType('interaction_runtime.registered_observation_v71');observed.initialize=Mock()
        owner=ModuleType('interaction_runtime.probing_v7');owner.OperationsV7=object
        with patch.dict(sys.modules,{'interaction_runtime.registered_observation_v71':observed,
                'interaction_runtime.probing_v7':owner}), \
             patch('agentic_interaction_v81.session.operations_class',return_value=lambda runtime:ops), \
             patch('agentic_interaction_v81.session.window',return_value=(20,{})), \
             patch('agentic_interaction_v81.session.receive',side_effect=lambda p,i,res:(commands[i],{})):
            outcome=serve(r)
        self.assertEqual(outcome['status'],'HARD_STOP')
        first=outcome['trace'][0]['result']
        self.assertFalse(first['task_recovery']['episode_termination_required'])
        self.assertEqual(outcome['trace'][1]['result']['World_identity'],id(r.world))
        self.assertFalse(outcome['trace'][1]['result']['reset_qualification']['qualified'])
        r.world.close.assert_not_called();ops.close_workers.assert_called_once()


if __name__=='__main__':unittest.main()
