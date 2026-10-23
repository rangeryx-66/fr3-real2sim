import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from agentic_interaction_v81.recovery_batch import relocate_observe_prepare,measured_costs
from interaction_runtime.policy import SafetyStop
from agentic_interaction_v81.transport import validate,ProtocolError


class RecoveryBatchTests(unittest.TestCase):
    def ops(self,path):
        r=SimpleNamespace(output=path,job={'articulation_v7':{'joint_type':'prismatic'}},tick=0,dt=1/240,
            episode_budget=SimpleNamespace(remaining=lambda t:dict(wall_s=1000,sim_s=500)))
        return SimpleNamespace(r=r,g={'agent_observations':[{'accepted_for_geometry':True}]},
            articulation=Mock(),evaluate_station=Mock(return_value={'valid':True}),
            relocate_station=Mock(return_value={'reached':True}),reobserve=Mock(return_value={'fresh':True}),
            prepare_grasp=Mock(return_value={'plan':True}))

    def test_batch_proposes_but_never_approves_or_executes_contact(self):
        with tempfile.TemporaryDirectory()as d,patch('agentic_interaction_v81.declaration_preflight.check'):
            ops=self.ops(Path(d));c=dict(planned_segments=[{'role':'FIT'}],reason='fresh inspected view',native_g1_preload_request_n=1.)
            result=relocate_observe_prepare(ops,c)
            self.assertTrue(result['new_observation_must_be_inspected_before_any_grasp'])
            self.assertFalse(result['physical_contact_approved'])
            ops.evaluate_station.assert_called_once_with(c);ops.reobserve.assert_called_once()
            self.assertNotIn('inspected_image_sha256',ops.prepare_grasp.call_args.args[0])

    def test_uncertain_new_target_returns_to_agent_without_grasp_search(self):
        with tempfile.TemporaryDirectory()as d,patch('agentic_interaction_v81.declaration_preflight.check'):
            ops=self.ops(Path(d));ops.g['agent_observations'][0]['accepted_for_geometry']=False
            result=relocate_observe_prepare(ops,dict(planned_segments=[],reason='inspect'))
            self.assertIn('AGENT_TARGET_SELECTION',result['reason']);ops.prepare_grasp.assert_not_called()

    def test_collision_rejection_and_hard_stop_do_not_continue(self):
        with tempfile.TemporaryDirectory()as d,patch('agentic_interaction_v81.declaration_preflight.check'):
            ops=self.ops(Path(d));ops.evaluate_station.return_value={'valid':False}
            self.assertFalse(relocate_observe_prepare(ops,{'planned_segments':[]})['executed'])
            ops.relocate_station.assert_not_called()
            ops.evaluate_station.return_value={'valid':True};ops.relocate_station.side_effect=SafetyStop('COLLISION')
            with self.assertRaises(SafetyStop):relocate_observe_prepare(ops,{'planned_segments':[]})
            ops.reobserve.assert_not_called()

    def test_measured_cost_gate_stops_before_motion_and_retains_exit_reserve(self):
        with tempfile.TemporaryDirectory()as d,patch('agentic_interaction_v81.declaration_preflight.check'):
            p=Path(d);rows=[]
            for op,cost in [('reobserve_and_replan',62),('prepare_grasp',111),('attempt_grasp_and_probe',424)]:
                rows.append(dict(command={'operation':op},wall_s=cost,result={'acquisition':{'acquired':True}}))
            # A later fast refused acquisition cannot erase the measured full-operation cost.
            rows.append(dict(command={'operation':'attempt_grasp_and_probe'},wall_s=1,result={'executed':False}))
            (p/'AGENT_ACTION_TRACE.json').write_text(json.dumps(rows));ops=self.ops(p)
            ops.r.episode_budget.remaining=lambda t:dict(wall_s=430,sim_s=300)
            self.assertEqual(measured_costs(ops)['recovery_preparation_and_attempt_observed_cost_s'],597)
            result=relocate_observe_prepare(ops,{'planned_segments':[]})
            self.assertIn('COST_EXCEEDS',result['reason']);ops.evaluate_station.assert_not_called()

    def test_revolute_never_enters_new_batch(self):
        with tempfile.TemporaryDirectory()as d:
            ops=self.ops(Path(d));ops.r.job['articulation_v7']['joint_type']='revolute'
            self.assertFalse(relocate_observe_prepare(ops,{})['executed']);ops.evaluate_station.assert_not_called()

    def test_transport_requires_inspection_finite_station_and_no_manual_grasp(self):
        c=dict(operation='relocate_observe_prepare',sequence=3,protocol_version=1,
            reason='inspect current safe view',inspected_image_sha256='a'*64,
            target_xy_m=[.2,-.5],planned_segments=[dict(role='FIT_OPEN',target_q=.02,
                coordinate_unit='m',profile='constant',hold=True)])
        self.assertIs(validate(c,3),c)
        with self.assertRaises(ProtocolError):validate(dict(c,T_grasp=[[1,0,0,0]]),3)
        with self.assertRaises(ProtocolError):validate(dict(c,target_xy_m=[float('nan'),0]),3)
        with self.assertRaises(ProtocolError):validate(dict(c,planned_segments=[]),3)
        with self.assertRaises(ProtocolError):validate(dict(c,native_g1_preload_request_n=2.),3)


if __name__=='__main__':unittest.main()
