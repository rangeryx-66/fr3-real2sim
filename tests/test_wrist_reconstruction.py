"""Meaningful camera-frame, FOV and safety boundaries for the new experiment."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np

class WristCaptureTests(unittest.TestCase):
    def test_operation_memory_rejects_gt_driven_source(self):
        from wrist_reconstruction.operation_memory import load_observed_model
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);poses=[np.eye(4).tolist() for _ in range(12)];poses[-1][0][3]=.01
            memory={'source':'measured EE only','GT_inputs':False,'estimated_articulation':{'joint_type':'prismatic','prismatic':{'axis':[1,0,0]}},'supporting_observations':poses}
            (p/'structured_memory.json').write_text(json.dumps(memory));report={'status':'STAGE_RECORDED','gt_control_inputs':False,'gt_fit_inputs':False,'object_actuation':False,'attachments':False};(p/'report.json').write_text(json.dumps(report))
            estimate,sign,audit=load_observed_model(p/'structured_memory.json','prismatic');self.assertEqual(sign,1.);self.assertIn('NOT new discovery',audit['source_kind'])
            report['gt_control_inputs']=True;(p/'report.json').write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,'FORBIDDEN_PROVENANCE'):load_observed_model(p/'structured_memory.json','prismatic')

    def test_failed_closure_release_precedes_retreat_and_preserves_motion_stop(self):
        from types import SimpleNamespace
        from wrist_reconstruction.recovery import Recovery
        events=[];calls=[0]
        r=SimpleNamespace(drive=SimpleNamespace(active=True),hold=lambda t:None,configure_release=lambda *a:events.append('configure'),release_increment=lambda d:events.append(('release',d)),released=lambda:True,reclose_at_current_pose=lambda:events.append('reclose'))
        recovery=Recovery.__new__(Recovery);recovery.r=r;recovery.released=False;recovery.job={'wrist_experiment':{'retreat':{'maximum_release_increments':2}}}
        recovery.observe=lambda D:(np.eye(4),{},np.array([[0.,0.,0.],[.1,0.,0.]]))
        recovery.release_observation=lambda D:recovery.observe(D)[2]
        def execute(plan):
            self.assertTrue(recovery.released);events.append('retreat');return True
        recovery.retreat=SimpleNamespace(plans=lambda D:[{'opening':.04}],execute=execute)
        recovery.release_failed_closure(np.eye(4));self.assertEqual(events,['configure',('release',.001),'retreat'])
        events.clear();recovery.released=False
        def moving(D):
            T=np.eye(4);T[0,3]=.002 if calls[0] else 0.;calls[0]+=1
            return T,{},np.array([[0.,0.,0.],[.1,0.,0.]])+T[:3,3]
        recovery.observe=moving
        release_calls=[0]
        def release_points(D):
            points=np.array([[0.,0.,0.],[.1,0.,0.]])
            if release_calls[0]:points[:,0]+=.002
            release_calls[0]+=1
            return points
        recovery.release_observation=release_points
        with self.assertRaisesRegex(RuntimeError,'UNSAFE_RELEASE_OBSERVED_OBJECT_MOTION'):recovery.release_failed_closure(np.eye(4))
        self.assertIn('reclose',events);self.assertNotIn('retreat',events);self.assertFalse(recovery.released)

    def test_release_motion_uses_current_observations_and_keeps_safety_limit(self):
        from wrist_reconstruction.recovery import release_motion
        rng=np.random.default_rng(17)
        before=rng.uniform([-.02,-.02,-.03],[.02,.02,.03],size=(3000,3))
        motion,audit=release_motion(before,before.copy())
        self.assertLess(motion,1e-8)
        moved=before+np.array([.002,0,0])
        motion,_=release_motion(before,moved)
        self.assertGreater(motion,.001)
        self.assertEqual(audit['threshold_m'],.001)
        self.assertEqual(audit['reference'],'current stabilized pre-release cloud')

    def test_release_self_mask_uses_robot_q_geometry_and_keeps_front_object(self):
        from types import SimpleNamespace
        from wrist_reconstruction.self_observation import robot_projection_mask
        K=np.array([[50.,0,32],[0,50.,24],[0,0,1.]])
        vertices=np.array([[x,y,z] for x in [-.12,.12] for y in [-.08,.08] for z in [.75,.85]])
        geometries={'finger':SimpleNamespace(vertices=vertices)}
        poses={'finger':np.eye(4)}
        front=np.full((48,64),.5)
        self.assertFalse(robot_projection_mask(front,K,np.eye(4),geometries,poses).any())
        robot_depth=np.full((48,64),.8)
        mask=robot_projection_mask(robot_depth,K,np.eye(4),geometries,poses)
        self.assertTrue(mask[24,32]);self.assertFalse(mask[0,0])
        shifted=np.eye(4);shifted[0,3]=.3
        moved=robot_projection_mask(robot_depth,K,np.eye(4),geometries,{'finger':shifted})
        self.assertFalse(moved[24,32]);self.assertTrue(moved[24,50])

    def test_unknown_regrasp_does_not_forecast_a_missing_model(self):
        from operational_structure.confidence import OperationalMemory
        from wrist_reconstruction.recovery import reset_model_monitor
        memory=OperationalMemory.__new__(OperationalMemory);memory.estimate=None;memory.initial=np.eye(4)
        reset_model_monitor(memory,np.eye(4))
        self.assertIsNone(memory.consistency_error(np.eye(4)))
        memory.estimate={'joint_type':'revolute'};T=np.eye(4);T[0,3]=.4
        reset_model_monitor(memory,T);np.testing.assert_array_equal(memory.monitor_anchor,T)

    def test_stereo_baseline_counts_but_zoom_only_does_not(self):
        from wrist_reconstruction.geometry import distinct_view
        first=np.eye(4);stereo=np.eye(4);stereo[0,3]=.10;zoom=np.eye(4);zoom[2,3]=.10
        self.assertTrue(distinct_view(stereo,[first],.04,8.))
        self.assertFalse(distinct_view(zoom,[first],.04,8.))

    def test_view_groups_preserve_coverage_with_less_backtracking(self):
        from wrist_reconstruction.geometry import coverage_views,calibration,distinct_view
        root=Path(__file__).resolve().parents[1];cal=calibration(root/'configs/wrist_camera_d435_clear_mount_sim.json')
        policy=json.loads((root/'configs/wrist_reconstruction_v2_sensor_bounds.json').read_text())['capture']
        P=np.array([[x,y,z] for x in [0,.5] for y in [0,.3] for z in [0,.3]])
        def first_eight(p):
            chosen=[]
            for v in coverage_views(P,P.mean(0),[0,-1,0],cal,p):
                if distinct_view(v['T_camera'],[x['T_camera'] for x in chosen],.04,8):chosen.append(v)
                if len(chosen)==8:break
            return chosen
        original=first_eight(policy);grouped=first_eight(dict(policy,view_order='azimuth_groups'))
        self.assertEqual(len(grouped),8)
        self.assertEqual({v['azimuth_deg'] for v in grouped},{v['azimuth_deg'] for v in original})
        travel=lambda views:sum(np.linalg.norm(a['T_camera'][:3,3]-b['T_camera'][:3,3]) for a,b in zip(views,views[1:]))
        self.assertLess(travel(grouped),travel(original))

    def test_closed_observation_reuse_never_restores_physics(self):
        from types import SimpleNamespace
        from PIL import Image
        from articulated_interaction_skill.capture import backproject
        from wrist_reconstruction.checkpoint import closed_views
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'previous';initial=source/'initial_sensor_observation';initial.mkdir(parents=True);folder=source/'states/state_000/view_00';folder.mkdir(parents=True)
            K=np.array([[15.,0,6.],[0,15.,4.],[0,0,1.]]);T=np.eye(4);depth=np.ones((8,12))*.5;mask=np.ones((8,12),bool);P,_=backproject(depth,K,T,mask,stride=4)
            np.save(initial/'depth_m.npy',depth);np.save(initial/'sensor_mask.npy',mask);(initial/'camera.json').write_text(json.dumps({'K':K.tolist(),'T_world_camera_optical':T.tolist()}))
            qa={'robot_pixel_ratio':0.,'object_frame_clipped':False,'visible_moving_part_pixels':48,'object_coverage':.5}
            meta={'K':K.tolist(),'resolution_wh':[12,8],'qa':qa};(folder/'camera.json').write_text(json.dumps(meta))
            Image.fromarray(np.full((8,12,3),127,np.uint8)).save(folder/'rgb.png');Image.fromarray(mask.astype(np.uint8)*255).save(folder/'mask.png');np.save(folder/'depth_m.npy',depth);np.savez(folder/'point_cloud.npz',points_world_m=P)
            view={'view_id':0,'directory':'states/state_000/view_00','K':K.tolist(),'T_world_camera_optical':T.tolist(),'qa':qa}
            (source/'states/state_000/views_checkpoint.json').write_text(json.dumps({'estimated_state':0.,'views':[view]}));(source/'multistate_capture.json').write_text(json.dumps({'object_id':'test','capture_mode':'wrist_camera_capture'}))
            output=root/'new';target=output/'states/state_000';target.mkdir(parents=True)
            policy={'resume_observation_max_displacement_m':.001,'maximum_robot_pixel_ratio':.005,'minimum_moving_part_pixels':1,'minimum_object_coverage':.15,'maximum_object_coverage':.85}
            r=SimpleNamespace(job={'resume_closed_capture':str(source)},states=[],metadata={'asset_id':'test'},initial_framing_cloud=P,config={'capture':policy},output=output,cal={'K':K,'resolution_wh':[12,8]})
            reused=closed_views(r,target,0.);self.assertEqual(len(reused),1);self.assertFalse(reused[0]['acquired_in_current_execution']);self.assertEqual((target/'view_00/rgb.png').read_bytes(),(folder/'rgb.png').read_bytes())
            self.assertEqual(closed_views(r,target,.05),[])
            r.initial_framing_cloud=P+[.005,0,0]
            self.assertEqual(closed_views(r,target,0.),[])
            audit=json.loads((output/'observation_resume.json').read_text());self.assertFalse(audit['physical_state_restore']);self.assertEqual(audit['status'],'FRESH_CAPTURE_REQUIRED')

    def test_large_sensor_volume_has_bounded_framing_recovery(self):
        from wrist_reconstruction.geometry import coverage_views,calibration
        root=Path(__file__).resolve().parents[1]
        cal=calibration(root/'configs/wrist_camera_d435_clear_mount_sim.json')
        policy=json.loads((root/'configs/wrist_reconstruction_v2_sensor_bounds.json').read_text())['capture']
        P=np.array([[x,y,z] for x in [0,.7] for y in [0,.7] for z in [0,.6]])
        original=coverage_views(P,P.mean(0),[0,-1,0],cal,policy)
        self.assertLess(sum(v['observed_volume_in_frame']==1. for v in original),8)
        policy['volume_framing_recovery']={'maximum_standoff_m':1.6,'pool_elevations_deg':[0,9],'pool_azimuths_deg':[-80,-60,-40,-20,0,20,40,60,80,100,120,150]}
        recovered=coverage_views(P,P.mean(0),[0,-1,0],cal,policy)
        self.assertEqual(len(recovered),48)
        self.assertGreaterEqual(sum(v['observed_volume_in_frame']==1. for v in recovered),8)
        self.assertTrue(all(v.get('framing_recovery') for v in recovered))

    def test_framing_covers_sparse_edges_despite_dense_front_surface(self):
        from wrist_reconstruction.geometry import observed_volume,coverage_views,calibration,visibility
        root=Path(__file__).resolve().parents[1]
        corners=np.array([[x,y,z] for x in [.2,.8] for y in [-.2,.2] for z in [0.,.6]])
        P=np.vstack([np.repeat(corners,10,axis=0),np.tile([.8,-.2,.6],(1000,1))])
        center,_,bound=observed_volume(P)
        np.testing.assert_allclose(center,[.5,0.,.3])
        self.assertGreater(np.linalg.norm(np.median(P,axis=0)-center),.3)
        cal=calibration(root/'configs/wrist_camera_d435_clear_mount_sim.json')
        policy=json.loads((root/'configs/wrist_reconstruction_v2_low_views.json').read_text())['capture']
        policy['maximum_standoff_m']=2.0
        for view in coverage_views(P,np.median(P,axis=0),[0,-1,0],cal,policy):
            self.assertEqual(visibility(bound,view['T_camera'],cal['K'],cal['resolution_wh']),1.)

    def test_deployed_script_chain_cannot_load_old_root_entry_copy(self):
        import importlib.util,sys
        root=Path(__file__).resolve().parents[1]
        spec=importlib.util.spec_from_file_location('wrist_entry_test',root/'scripts/run_wrist_reconstruction_episode.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertEqual(Path(sys.modules['run_articulated_system_episode'].__file__).resolve(),root/'scripts/run_articulated_system_episode.py')
        self.assertIn('runtime.plan_retreat=plan_retreat',module.source())

    def test_requested_camera_pose_is_realized_by_tcp_and_extrinsic(self):
        from wrist_reconstruction.geometry import calibration,look_at,optical_to_tcp,visibility
        d=calibration(Path(__file__).resolve().parents[1]/'configs/wrist_camera_d435_nominal.json');C=look_at(np.array([.8,-.4,.4]),np.array([.5,0,.2]));E=optical_to_tcp(C,d['X'])
        np.testing.assert_allclose(E@d['X'],C,atol=1e-12)
        self.assertFalse(d['real_hardware_calibrated'])
        self.assertEqual(visibility(np.array([[.5,0,.2]]),C,d['K'],d['resolution_wh']),1.)

    def test_rectangular_capture_preserves_full_horizontal_fov(self):
        from PIL import Image
        from articulated_system.artgs import prepare
        with tempfile.TemporaryDirectory() as tmp:
            r=Path(tmp);root=r/'capture';root.mkdir();states=[]
            for state in range(2):
                views=[]
                for i in range(6):
                    p=root/f's{state}'/f'v{i}';p.mkdir(parents=True)
                    rgb=np.zeros((12,20,3),np.uint8);rgb[:,0]=[255,0,0];rgb[:,-1]=[0,255,0]
                    Image.fromarray(rgb).save(p/'rgb.png');Image.fromarray(np.full((12,20),255,np.uint8)).save(p/'mask.png');np.save(p/'depth_m.npy',np.ones((12,20),np.float32));(p/'camera.json').write_text(json.dumps({'depth':'optical Z in meters','mask_source':'SAM3'}))
                    np.savez(p/'point_cloud.npz',points_world_m=np.array([[0,state*.01,0],[.1,.1,.1]]),rgb=np.zeros((2,3),np.uint8),units='m')
                    views.append({'view_id':i,'directory':str(p.relative_to(root)),'K':[[15.,0,10.],[0,15.,6.],[0,0,1.]],'T_world_camera_optical':np.eye(4).tolist()})
                states.append({'state_id':state,'estimated_articulation_state':state*.01,'views':views})
            (root/'multistate_capture.json').write_text(json.dumps({'states':states,'mask_source':'SAM3'}))
            out=r/'input';meta=prepare(root,out,0,1,size=20);image=np.asarray(Image.open(out/'start/train/rgba/0000.png'))
            self.assertEqual(image.shape,(12,20,4));np.testing.assert_array_equal(image[5,0,:3],[255,0,0]);np.testing.assert_array_equal(image[5,-1,:3],[0,255,0]);self.assertFalse(meta['audit'][0]['masked_robot'])
            import shutil
            extra=root/'s0/v6';shutil.copytree(root/'s0/v0',extra)
            states[0]['views'].append(dict(states[0]['views'][0],view_id=6,directory='s0/v6'))
            (root/'multistate_capture.json').write_text(json.dumps({'states':states,'mask_source':'SAM3'}))
            meta=prepare(root,r/'unpaired',0,1,size=20)
            self.assertIn(6,meta['train_views_by_state']['start'])
            self.assertNotIn(6,meta['train_views_by_state']['end'])
            self.assertEqual(meta['held_out_views_by_state']['start'],[4,5])

    def test_frozen_physical_contact_sources_are_not_edited(self):
        import hashlib,subprocess
        root=Path(__file__).resolve().parents[1]
        expected=json.loads((root/'wrist_reconstruction/frozen_baseline_hashes.json').read_text())
        for name,sha in expected.items():
            self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(),sha,name)
