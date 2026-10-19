"""Metric assembly and sensor/proxy boundaries of reconstruction integration."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np


class ReconstructionIntegrationTests(unittest.TestCase):
    def test_EE_compliance_does_not_create_object_startup_effort(self):
        from articulated_system.effort import profile
        with tempfile.TemporaryDirectory() as tmp:
            r=Path(tmp);(r/'evaluation_private').mkdir()
            (r/'report.json').write_text('{}')
            (r/'evaluation_private/scope.json').write_text(json.dumps({'evaluation_only':True,'controller_readback':False}))
            (r/'effort_segments.json').write_text(json.dumps([{'index':0,'from_rest':True}]))
            samples=[];objects=[]
            for i in range(30):
                T=np.eye(4);T[0,3]=i*.00003
                samples.append({'segment':0,'t':i*.1,'T_ee':T.tolist(),'full_tangential_measurement':True,'effective_tangential_force_n':1.,'direction_world':[1,0,0]})
                objects.append({'t':i*.1,'T_object':np.eye(4).tolist()})
            (r/'effort_samples.jsonl').write_text('\n'.join(json.dumps(s) for s in samples))
            (r/'evaluation_private/object_trajectory.json').write_text(json.dumps(objects))
            self.assertIsNotNone(profile(r)['opening_breakaway_effort'][0]['opening_task_resistance_at_onset_n'])
            d=profile(r,object_motion_evaluation=True)
            self.assertIsNone(d['opening_breakaway_effort'][0]['opening_task_resistance_at_onset_n'])
            self.assertFalse(d['measurement_completed'])
            self.assertTrue(d['simulation_only_object_pose_used_offline'])

    def test_legacy_nominal_plan_preserves_pose_and_shared_recovery_family(self):
        from articulated_system.recovery import normalize_plan
        from interactive_twin.manifest import DEFAULT_POLICY
        legacy={'rows':[{'T':np.eye(4).tolist()}]}
        current=normalize_plan(legacy)
        self.assertEqual(current['candidate_grid'],DEFAULT_POLICY['candidate_grid'])
        self.assertEqual(current['rows'][0]['candidate_index'],0)
        self.assertEqual(current['rows'][0]['T'],legacy['rows'][0]['T'])
        self.assertNotIn('candidate_grid',legacy)
        self.assertNotIn('candidate_index',legacy['rows'][0])

    def test_inferred_joint_recenter_preserves_metric_initial_assembly(self):
        import trimesh,xml.etree.ElementTree as ET
        from articulated_system.twin import write
        with tempfile.TemporaryDirectory() as tmp:
            r=Path(tmp);src=r/'recon';inp=r/'input';capture=r/'capture';out=r/'twin'
            for p in [src,inp,capture]:p.mkdir()
            mesh=trimesh.creation.box(extents=[.2,.3,.4]);mesh.apply_translation([.2,.1,.3])
            for i in range(2):mesh.export(src/f'part_{i}.ply')
            (src/'motion_inferred.json').write_text(json.dumps({'joints':[{'type':'r','axis_direction':[0,1,0],'axis_position':[.1,.2,.3],'theta':[12.]}]}))
            center=np.array([1.,2.,3.]);scale=.7
            (inp/'input_provenance.json').write_text(json.dumps({'normalization':{'world_center_m':center.tolist(),'world_scale_m':scale},'states':[0,1]}))
            (capture/'multistate_capture.json').write_text(json.dumps({'object_id':'test','units':'degrees','states':[{'estimated_articulation_state':i*10.,'views':[]} for i in range(2)]}))
            d=write(src,inp,capture,out);root=ET.parse(out/'reconstructed.urdf').getroot();j=root.find('joint')
            origin=np.array([float(x) for x in j.find('origin').get('xyz').split()]);v=trimesh.load(out/'reconstructed_parts/part_1.obj',process=False).vertices
            original=trimesh.load(src/'part_1.ply',process=False).vertices
            np.testing.assert_allclose(v+origin+center,original*scale+center,atol=1e-8)
            self.assertEqual(j.get('type'),'revolute');self.assertFalse(d['observed_range_estimated']['full_joint_limits_known'])
            self.assertEqual(d['inferred_joint']['motion_between_input_states'],np.deg2rad(12.))
            self.assertFalse(d['GT_mesh_or_axis_substitution'])

    def test_reserved_camera_ids_survive_occlusion_and_do_not_set_scale(self):
        from PIL import Image
        from articulated_system.artgs import prepare
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'capture';root.mkdir();states=[]
            for state_id in [0,1]:
                views=[]
                for camera_id in range(8):
                    if state_id==1 and camera_id==1:continue
                    folder=root/f's{state_id}'/f'v{camera_id}';folder.mkdir(parents=True)
                    Image.fromarray(np.full((8,8,3),128,np.uint8)).save(folder/'rgb.png');Image.fromarray(np.full((8,8),255,np.uint8)).save(folder/'mask.png');np.save(folder/'depth_m.npy',np.full((8,8),2.,np.float32))
                    (folder/'camera.json').write_text(json.dumps({'depth':'optical Z in meters'}))
                    points=np.array([[camera_id*.1,state_id*.05,1.],[camera_id*.1+.01,state_id*.05,1.01]])
                    if camera_id>=6:points+=1000. # Held-out geometry must not leak into normalization.
                    np.savez(folder/'point_cloud.npz',points_world_m=points,rgb=np.full((2,3),128,np.uint8),units='m')
                    views.append({'view_id':camera_id,'directory':str(folder.relative_to(root)),'K':[[8.,0,4.],[0,8.,4.],[0,0,1.]],'T_world_camera_optical':np.eye(4).tolist()})
                states.append({'state_id':state_id,'estimated_articulation_state':state_id*.02,'views':views})
            (root/'multistate_capture.json').write_text(json.dumps({'states':states}))
            out=Path(tmp)/'input';d=prepare(root,out,0,1,size=8)
            self.assertEqual(d['held_out_views'],[6,7]);self.assertEqual(d['train_views'],[0,2,3,4,5]);self.assertLess(d['normalization']['world_scale_m'],1.)
            for name in ['start','end']:
                frames=json.loads((out/f'transforms_test_{name}.json').read_text())['frames'];self.assertEqual([Path(f['file_path']).stem for f in frames],['0006','0007'])
            stored=np.asarray(Image.open(out/'start/train/depth/0000.png'));metric=stored/1000*d['normalization']['world_scale_m'];np.testing.assert_allclose(metric,2.,atol=.001)

    def test_unverified_force_is_not_measurement_or_friction(self):
        from articulated_system.effort import profile
        with tempfile.TemporaryDirectory() as tmp:
            r=Path(tmp);(r/'effort_segments.json').write_text(json.dumps([{'index':0,'from_rest':True}]))
            rows=[]
            for i in range(30):
                T=np.eye(4);T[0,3]=i*.00003
                rows.append({'segment':0,'t':i*.1,'T_ee':T.tolist(),'full_tangential_measurement':False,'effective_tangential_force_n':999.,'direction_world':[1,0,0],'command_wrench_world':[.1,0,0,0,0,0]})
            (r/'effort_samples.jsonl').write_text('\n'.join(json.dumps(s) for s in rows))
            d=profile(r);record=d['moving_effort_vs_state'][0]
            self.assertIsNone(record['opening_task_resistance_at_onset_n']);self.assertIsNone(record['quasistatic_moving_tangential_force_n'])
            self.assertFalse(d['measurement_completed']);self.assertFalse(d['friction_identification_performed'])
            self.assertAlmostEqual(record['command_proxy_range_n']['mean'],.1)


if __name__=='__main__':unittest.main()
