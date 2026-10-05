"""Meaningful camera-frame, FOV and safety boundaries for the new experiment."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np

class WristCaptureTests(unittest.TestCase):
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

    def test_frozen_physical_contact_sources_are_not_edited(self):
        import hashlib,subprocess
        root=Path(__file__).resolve().parents[1]
        expected=json.loads((root/'wrist_reconstruction/frozen_baseline_hashes.json').read_text())
        for name,sha in expected.items():
            self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(),sha,name)
