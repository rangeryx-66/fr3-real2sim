import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from articulated_interaction_skill.capture import backproject
from articulated_interaction_skill.effort import effective_effort,summarize_effort
from articulated_interaction_skill.export import normalized_pair,export_capture


class SkillTests(unittest.TestCase):
    def test_axial_depth_and_optical_transform(self):
        depth=np.full((2,2),2.);K=np.array([[2.,0,0],[0,2.,0],[0,0,1.]])
        T=np.eye(4);T[:3,3]=[10,20,30]
        P,keep=backproject(depth,K,T,np.ones((2,2)),stride=1)
        np.testing.assert_allclose(P,[[10,20,32],[11,20,32],[10,21,32],[11,21,32]])
        self.assertEqual(keep.sum(),4)

    def test_opposing_preload_cancels_and_object_force_sign(self):
        def contact(F,p):return {'allowed_pad_target':True,'impulse_world_ns':(-np.array(F)*.01).tolist(),'contact_point_world_m':p}
        C=[contact([2,1,0],[1,0,0]),contact([-2,1,0],[1,0,0])]
        E=effective_effort(C,.01,[0,1,0],[0,0,1],np.zeros(3))
        np.testing.assert_allclose(E['force_on_handle_world_n'],[0,2,0])
        self.assertEqual(E['effective_tangential_force_n'],2.)
        self.assertEqual(E['estimated_axis_torque_nm'],2.)

    def test_official_ditto_normalization_and_roundtrip(self):
        A=np.arange(90).reshape(30,3)*.01;B=A+[0,.02,0]
        x,y,c,s=normalized_pair(A,B)
        self.assertEqual(x.shape,(1,8192,3));self.assertEqual(x.dtype,np.float32)
        self.assertGreater(s,0);self.assertLess(float(abs(x).max()),.46)
        x2,y2,c2,s2=normalized_pair(A,B)
        np.testing.assert_array_equal(x,x2)
        reconstructed=x[0]*s+c
        self.assertLess(np.min(np.linalg.norm(reconstructed[0]-A,axis=1)),1e-6)

    def test_continuous_segment_is_not_breakaway(self):
        with tempfile.TemporaryDirectory() as tmp:
            P=Path(tmp);(P/'effort_segments.json').write_text(json.dumps([{'index':1,'start_estimated_state':1.,'start_s':0.,'from_rest':False}]))
            samples=[]
            for i in range(20):
                T=np.eye(4);T[0,3]=i*.0001
                samples.append({'segment':1,'t':i*.2,'T_ee':T.tolist(),'effective_tangential_force_magnitude_n':.1,'estimated_axis_torque_nm':None,'full_tangential_measurement':True,'full_contact_sensor':{'bilateral_buffer_verified':True}})
            (P/'effort_samples.jsonl').write_text('\n'.join(json.dumps(s) for s in samples))
            d=summarize_effort(P)
            self.assertIsNone(d['moving_effort_vs_state'][0]['breakaway_effective_force_n'])
            self.assertAlmostEqual(d['moving_effort_vs_state'][0]['moving_effective_force_n'],.1)

    def test_normal_only_report_does_not_claim_full_effort(self):
        with tempfile.TemporaryDirectory() as tmp:
            P=Path(tmp);(P/'effort_segments.json').write_text(json.dumps([{'index':0,'start_estimated_state':0.,'start_s':0.,'from_rest':True}]))
            samples=[]
            for i in range(20):
                T=np.eye(4);T[0,3]=i*.0001
                samples.append({'segment':0,'t':i*.2,'T_ee':T.tolist(),'effective_tangential_force_magnitude_n':99.,'estimated_axis_torque_nm':None,'command_wrench_world':[.1,0,0,0,0,0],'direction_world':[1,0,0],'full_tangential_measurement':False})
            (P/'effort_samples.jsonl').write_text('\n'.join(json.dumps(s) for s in samples))
            d=summarize_effort(P)['moving_effort_vs_state'][0]
            self.assertEqual(d['effort_measurement_kind'],'COMMAND_EQUIVALENT_PROXY')
            self.assertAlmostEqual(d['moving_effective_force_n'],.1)
            self.assertAlmostEqual(d['breakaway_effective_force_n'],.1)

    def test_force_buffers_survive_following_friction_query(self):
        from articulated_interaction_skill.force_sensor import ForceSensor
        class View:
            sensor_count=2;filter_count=1
            def __init__(self):
                self.count=np.array([[2],[2]]);self.start=np.array([[0],[2]])
            def get_contact_data(self,dt):
                return np.ones((4,1)),np.zeros((4,3)),np.array([[1,0,0],[1,0,0],[-1,0,0],[-1,0,0]]),np.zeros(4),self.count,self.start
            def get_friction_data(self,dt):
                self.count[:]=0;self.start[:]=0
                return np.zeros((4,3)),np.zeros((4,3)),self.count,self.start
        sensor=ForceSensor.__new__(ForceSensor);sensor.view=View();sensor.error=None
        result=sensor.sample(.01)
        self.assertEqual(result['contact_counts_per_finger'],[2,2])
        self.assertTrue(result['bilateral_buffer_verified'])
        self.assertEqual(result['normal_load_sum_n'],4.)
        np.testing.assert_allclose(result['force_on_handle_world_n'],[0,0,0])

    def test_export_preserves_state_groups_and_tensor_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            P=Path(tmp);states=[]
            for i in range(3):
                folder=P/'states'/f'state_{i:03d}'/'view_00';folder.mkdir(parents=True)
                np.savez_compressed(folder/'point_cloud.npz',points_world_m=np.arange(90).reshape(30,3)*.001+[0,i*.01,0])
                states.append({'state_id':i,'estimated_articulation_state':i*.02,'views':[{'directory':str(folder.relative_to(P)),'K':np.eye(3).tolist(),'T_world_camera_optical':np.eye(4).tolist()}]})
            (P/'multistate_capture.json').write_text(json.dumps({'object_id':'fixture','moving_part_id':'drawer','units':'meters','states':states,'articulation_labels':'EE-only'}))
            result=export_capture(P);self.assertEqual(result['pairs'],3)
            pair=np.load(P/'reconstruction/pair_000_001.npz');self.assertEqual(pair['pc_end'].shape,(1,8192,3))
            twin=json.loads((P/'twin_update.json').read_text());self.assertFalse(twin['full_joint_limit_inferred'])


if __name__=='__main__':unittest.main()
