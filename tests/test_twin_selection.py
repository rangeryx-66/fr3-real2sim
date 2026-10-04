import copy,unittest
from unittest.mock import patch
import numpy as np
from interactive_twin.sysid import command_hash
from interactive_twin_selection.metrics import score,onset
from interactive_twin_selection.policy import rank,extension_anchors


def log():
    t=np.arange(0,2,.01);T=np.repeat(np.eye(4)[None],len(t),axis=0)
    T[:,0,3]=np.maximum(np.minimum(t,1.5)-.5,0)*.001
    c={'time_s':t.tolist(),'fields':['drive_active'],'kind':'cartesian_constrained_drive',
       'values':(((t>=.5)&(t<1.5)).astype(float)[:,None]).tolist()}
    return {'schema_version':1,'mode':'SIM_TO_SIM_BLIND_SYSID','episode_id':'unit','probe_id':'P1','split':'train',
            'time_s':t.tolist(),'commands':c,'signals':{'q_rad':np.zeros((len(t),6)).tolist(),
            'qdot_rad_s':np.zeros((len(t),6)).tolist(),'ee_T_world_tcp':T.tolist(),'gripper_opening_m':[.03]*len(t)},
            'provenance':{'source':'isaac_physics','robot_model_id':'unit','robot_calibration_id':'unit','controller_id':'unit',
            'simulator':'isaac_physx','complete':True,'initialization_only':True,'robot_state_replayed':False,
            'object_state_replayed':False,'direct_object_actuation':False,'attachment':False,'command_applied_sha256':command_hash(c)}}


class SelectionTests(unittest.TestCase):
    def test_q_not_duplicate_physics_evidence(self):
        a=log();b=copy.deepcopy(a)
        cal={'response_covariance':(np.eye(9)*1e-8).tolist(),'onset_threshold_m':1e-5,
             'onset_hold_s':.05,'start_time_scale_s':.01,'calibration_sha256':'unit'}
        before=score(a,b,cal)
        b['signals']['q_rad']=(np.ones((len(a['time_s']),6))*.1).tolist()
        after=score(a,b,cal)
        self.assertEqual(before['loss'],after['loss']);self.assertGreater(after['q_diagnostic_rmse_rad'],0)

    def test_test_cannot_change_selection(self):
        a=log();a['split']='heldout'
        with self.assertRaisesRegex(ValueError,'HELDOUT_ACCESS_DENIED'):score(a,a,{})

    def test_late_data_overturns_early_winner_without_pruning(self):
        names=['low_forward','high_forward','stop_dwell']
        ref={k:{'split':'train','name':k} for k in names}
        candidates=[{'candidate_id':k,'structure_id':k,'tau_c':.004,'b':0.,'tau_s':.004} for k in ['a','b']]
        bank={k:{n:{'safe':True,'log':{'candidate':k}} for n in names} for k in ['a','b']}
        losses={'a':[0,100,100],'b':[1,1,1]}
        def fake(r,p,c):return {'loss':losses[p['candidate']][names.index(r['name'])]}
        policy={'top_k':5,'support_log_likelihood_delta':3,'adequacy_rms':10}
        with patch('interactive_twin_selection.policy.score',side_effect=fake):
            result=rank(candidates,bank,ref,{},policy)
            reverse=rank(candidates,bank,dict(reversed(list(ref.items()))),{},policy)
        self.assertEqual(result['best']['candidate_id'],'b')
        self.assertEqual(reverse['best']['candidate_id'],'b')
        self.assertEqual({r['candidate_id'] for r in result['support']},{'a','b'})
        self.assertFalse(result['sequential_pruning'])

    def test_onset_does_not_count_pre_drive_drift(self):
        t=np.arange(0,2,.01);p=np.zeros((len(t),3));p[:,0]=np.minimum(t,.5)*.01
        active=(t>=1)&(t<1.8)
        self.assertIsNone(onset(t,p,active,.0001,.05))

    def test_inadequate_model_keeps_full_parameter_set(self):
        names=['low_forward','high_forward','stop_dwell']
        reference={n:{'split':'train','name':n} for n in names}
        candidates=[{'candidate_id':str(i),'structure_id':'fixed','tau_c':i*.004,
                     'b':0.,'tau_s':i*.004} for i in range(7)]
        bank={c['candidate_id']:{n:{'safe':True,'log':{}} for n in names} for c in candidates}
        policy={'top_k':5,'support_log_likelihood_delta':3,'adequacy_rms':10}
        with patch('interactive_twin_selection.policy.score',return_value={'loss':121.}):
            r=rank(candidates,bank,reference,{},policy)
        self.assertEqual(r['status'],'MODEL_MISMATCH')
        self.assertTrue(r['all_candidates_validation_inadequate'])
        self.assertEqual(len(r['retained_parameters']),len(candidates))
        self.assertEqual(r['parameter_intervals']['tau_c'],[0.,.024])

    def test_static_setup_default_and_extension(self):
        import interactive_twin.plant as plant
        from interactive_twin_selection.native import install
        row={'joint':'/door/joint','authored':{'staticFrictionEffort':.004},'static_equals_dynamic':True}
        from unittest.mock import MagicMock
        stage=MagicMock();stage.GetPrimAtPath.return_value.GetAttribute.return_value.IsValid.return_value=True
        with patch.object(plant,'author_passive_resistance',return_value=[copy.deepcopy(row)]):
            install();original=plant.author_passive_resistance(stage,'/door',{'tau_c':.004,'b':0})
            self.assertEqual(original,[row]);stage.GetPrimAtPath.assert_not_called()
            extended=plant.author_passive_resistance(stage,'/door',{'tau_c':.004,'b':0,'tau_s':.012})
            self.assertEqual(extended[0]['authored']['staticFrictionEffort'],.012)
            stage.GetPrimAtPath.return_value.GetAttribute.return_value.Set.assert_called_once_with(.012)
            with self.assertRaises(ValueError):plant.author_passive_resistance(stage,'/door',{'tau_c':.012,'b':0,'tau_s':.004})

if __name__=='__main__':unittest.main()
