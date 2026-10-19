import copy,tempfile,unittest
from pathlib import Path
from interaction_runtime.recovery_candidates_v7 import catalog,resolve,order
from agent_command_protocol_v12 import validate,ProtocolError


def candidate(section,roll=0):
    trial=dict(width_m=.01,nominal=dict(sides=[dict(finger='L',lateral_margin_mm=.5)]))
    robust=dict(acquisition_score=1.,retention_score=.4,task_score=.5,robust_combined_score=.01,aperture_trials=[trial],chosen_expected_aperture_m=.01)
    return dict(section=section,insertion_m=.009,roll=roll,pitch=0.,contact=dict(robust_contact=robust,first_contacts={}))


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();p=Path(self.tmp.name)
        for n in ['rgb.png','depth_m.npy','camera.json']:(p/n).write_bytes(b'fresh observation fixture')
        self.observations=[dict(folder=str(p),accepted_for_geometry=True)];self.pool=[candidate(-.3),candidate(-.3,180),candidate(0.)]
    def tearDown(self):self.tmp.cleanup()
    def test_agent_choice_reorders_only_existing_automatic_family(self):
        c=catalog(self.pool,set(),self.observations);request=dict(candidate_id='2',catalog_sha256=c['catalog_sha256'],reason='measured first grasp had lateral-edge contact; inspect existing central alternative')
        preferred=resolve(request,c,self.observations,set());result=order(self.pool,set(),preferred)
        self.assertIs(result[0],self.pool[2]);self.assertEqual(result[0]['contact']['robust_contact']['robust_combined_score'],.01)
        self.assertEqual(len(result),1)
    def test_fresh_observation_or_failed_candidate_invalidates_request(self):
        c=catalog(self.pool,set(),self.observations);request=dict(candidate_id='0',catalog_sha256=c['catalog_sha256'],reason='contact evidence')
        with self.assertRaisesRegex(ValueError,'NOT_ELIGIBLE'):resolve(request,c,self.observations,{(-.3,.009,0.,0.)})
        (Path(self.tmp.name)/'rgb.png').write_bytes(b'new view')
        with self.assertRaisesRegex(ValueError,'NEW_CATALOG'):resolve(request,c,self.observations,set())
    def test_unknown_or_manual_pose_cannot_enter_pool(self):
        c=catalog(self.pool,set(),self.observations)
        with self.assertRaises(ValueError):order(self.pool,set(),(.123,.009,0.,0.))
        with self.assertRaises(ValueError):resolve(dict(candidate_id='100',catalog_sha256=c['catalog_sha256'],reason='bad id'),c,self.observations,set())
        command=dict(operation='build_geometry',protocol_version=1,sequence=0,candidate_request=dict(candidate_id='2',catalog_sha256=c['catalog_sha256'],reason='observed contact'))
        validate(command,0)
        command=copy.deepcopy(command);command['candidate_request']['T']=[[1.,0.,0.,0.]]
        with self.assertRaises(ProtocolError):validate(command,0)
    def test_actual_operation_request_clears_stale_plan_and_preference(self):
        from unittest.mock import patch
        from interaction_runtime.probing_v7 import OperationsV7
        ops=OperationsV7.__new__(OperationsV7);ops.current_plan_ready=True
        c=catalog(self.pool,set(),self.observations);ops.g=dict(contact_attempts=[],agent_observations=self.observations,recovery_candidate_catalog=c)
        bad=ops.request_plan(dict(candidate_request=dict(candidate_id='2',catalog_sha256='0'*64,reason='contact')))
        self.assertFalse(bad['plan']);self.assertFalse(ops.current_plan_ready)
        request=dict(candidate_id='2',catalog_sha256=c['catalog_sha256'],reason='contact')
        with patch.object(OperationsV7,'build_geometry',side_effect=lambda:dict(plan=True,preferred=ops.g['preferred_existing_candidate_family'])):
            self.assertEqual(ops.request_plan(dict(candidate_request=request))['preferred'],(0.,.009,0.,0.))
        self.assertNotIn('preferred_existing_candidate_family',ops.g)
        with patch.object(OperationsV7,'build_geometry',side_effect=[dict(plan=False,reason='native collision'),dict(plan=True,candidate_family=[-.3,.009,180.,0.])]) as plan:
            result=ops.request_plan(dict(candidate_request=request))
            self.assertEqual(plan.call_count,2);self.assertTrue(result['plan'])
            self.assertEqual(result['Agent_requested_family_infeasible']['reason'],'native collision')


if __name__=='__main__':unittest.main()
