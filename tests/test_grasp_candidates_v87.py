import json,math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from agentic_interaction_v87.grasp_candidates import transforms,pose_id,novel_shortlist
from agentic_interaction_v87.activation import activate


class CandidateTests(unittest.TestCase):
    def payload(self):
        W=np.eye(4);W[:3,3]=[1,2,3]
        C=np.eye(4);C[:3,3]=[.2,.3,.4]
        G=np.eye(4);G[2,3]=.172
        item=dict(rank=0,score=.9,T_C_G=C.tolist(),T_G_TCP=G.tolist())
        return dict(gripper='XARM7_NATIVE_G1',frame='camera_optical',T_B_C=W.tolist(),grasps=[item]),dict(T_world_camera_optical=W.tolist()),W@C@G

    def test_camera_and_tcp_transforms_are_applied_exactly_once(self):
        payload,camera,expected=self.payload();result=transforms(payload,camera)
        np.testing.assert_allclose(result[0]['T'],expected)
        self.assertEqual(result[0]['pose_id'],pose_id(expected))

    def test_foreign_gripper_stale_camera_and_nonrigid_pose_are_rejected(self):
        payload,camera,_=self.payload();payload['gripper']='Dex1'
        with self.assertRaises(ValueError):transforms(payload,camera)
        payload,camera,_=self.payload();camera['T_world_camera_optical'][0][3]+=1
        with self.assertRaises(ValueError):transforms(payload,camera)
        payload,camera,_=self.payload();payload['grasps'][0]['T_C_G'][0][0]=2
        with self.assertRaises(ValueError):transforms(payload,camera)

    def test_duplicate_poses_are_not_repeated_search_candidates(self):
        payload,camera,_=self.payload();payload['grasps']*=2
        self.assertEqual(len(transforms(payload,camera)),1)

    def test_search_diversity_does_not_change_pose_or_safety_score(self):
        candidates=[]
        for x in [0,.001,.02,.04]:
            T=np.eye(4);T[0,3]=x;candidates.append(dict(T=T,contact=dict(rank=1-x)))
        selected=novel_shortlist(candidates,2)
        self.assertIs(selected[0],candidates[0]);self.assertIs(selected[1],candidates[2])
        self.assertEqual(selected[1]['T'][0,3],.02)

    def test_grasp_resources_require_explicit_native_registration(self):
        def existing(spec,job):job['original']=True
        job={};activate(dict(graspgenx=dict(python='/bin/python',source='/models/graspgenx',gpu=0)),job,existing,'base')
        self.assertTrue(job['original']);self.assertEqual(job['v87_graspgenx']['gpu'],0)
        for bad in [dict(python='python',source='/models',gpu=0),dict(python='/bin/python',source='/models',gpu=-1)]:
            with self.assertRaises(ValueError):activate(dict(graspgenx=bad),{},existing,'base')

    def test_rewording_or_sequence_change_does_not_repeat_failed_search(self):
        from agentic_interaction_v87.hybrid_grasp import physical_request
        first=dict(operation='prepare_grasp',sequence=1,reason='first',
            planned_segments=[dict(target_q=.1)],planning_target_q=.1)
        repeated=dict(first,sequence=9,reason='try again',operation='build_geometry')
        self.assertEqual(physical_request(first),physical_request(repeated))
        changed=dict(repeated,planning_target_q=.2)
        self.assertNotEqual(physical_request(first),physical_request(changed))

    def test_fallback_uses_all_current_measured_views_in_one_camera_frame(self):
        import sys
        from types import ModuleType
        from agentic_interaction_v87.hybrid_grasp import generate
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);frame=root/'frame';frame.mkdir();geometry=root/'agent_geometry';geometry.mkdir()
            C=np.eye(4);C[:3,3]=[1,2,3];C[:3,:3]=np.array([[0,-1,0],[1,0,0],[0,0,1]])
            (frame/'camera.json').write_text(json.dumps(dict(T_world_camera_optical=C.tolist())))
            points=np.array([[1,2,4],[2,2,4],[1,3,4]],float)
            np.save(geometry/'contact_supported_points_world.npy',points)
            measured=ModuleType('native_g1_metric_rgbd_geometry')
            measured.preserve=lambda f:(_ for _ in ()).throw(AssertionError('latest-only geometry must not replace fused measurements'))
            ops=SimpleNamespace(r=SimpleNamespace(output=root,job=dict(v8_robot_io=str(root/'robot'),
                v87_graspgenx=dict(python='/bin/python',source='/models',gpu=1))))
            directory=root/'proposal'
            def inference(command,**kw):
                self.assertEqual(command[command.index('--max-input-points')+1],'8192')
                self.assertEqual(kw['env']['CUDA_VISIBLE_DEVICES'],'1')
                (directory/'candidates.json').write_text('{}')
                return SimpleNamespace(returncode=0)
            with patch.dict(sys.modules,{'native_g1_metric_rgbd_geometry':measured}),\
                 patch('agentic_interaction_v87.graspgenx_geometry.derive',return_value={}),\
                 patch('agentic_interaction_v87.hybrid_grasp.subprocess.run',side_effect=inference):
                generate(ops,frame,directory)
            with np.load(directory/'input.npz')as data:
                reconstructed=data['points']@C[:3,:3].T+C[:3,3]
                np.testing.assert_allclose(reconstructed,points)
                np.testing.assert_array_equal(data['T_B_C'],C)

    def test_optional_snapshot_write_failure_does_not_reject_valid_rule_plan(self):
        from agentic_interaction_v87.hybrid_grasp import operations_class
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);(root/'camera.json').write_text('{}');np.save(root/'handle_mask.npy',np.ones((2,2),bool))
            class Parent:
                def request_plan(self,command):return dict(plan=True,geometry=True)
            Ops=operations_class(Parent);ops=Ops()
            ops.g=dict(agent_observations=[dict(accepted_for_geometry=True,folder=str(root),world_sim_s=1.)])
            ops.r=SimpleNamespace(output=root,actual=lambda:0.,base=np.zeros(4),qtarget=np.zeros(7),job=dict(v87_graspgenx={}))
            with patch('agentic_interaction_v87.hybrid_grasp.snapshot',side_effect=OSError('diagnostic disk error')),\
                 patch('agentic_interaction_v87.hybrid_grasp.generate')as worker:
                result=ops.request_plan(dict(operation='prepare_grasp'))
            self.assertTrue(result['plan']);self.assertIn('optional_native_snapshot_error',result);worker.assert_not_called()


if __name__=='__main__':unittest.main()
