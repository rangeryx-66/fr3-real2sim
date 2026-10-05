import ast,sys,unittest,tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from operational_structure.prediction import prediction
from operational_structure.confidence import OperationalMemory


class OperationalTests(unittest.TestCase):
    def test_holdout_translation_bias_cannot_be_fitted_away(self):
        th=np.linspace(0,.04,80);T=np.tile(np.eye(4),(80,1,1));T[:,:3,:3]=Rotation.from_rotvec(np.c_[th*0,th*0,th]).as_matrix()
        T[:,:3,3]=np.c_[.5*np.cos(th),.5*np.sin(th),th*0]
        f={'revolute':{'axis':[0,0,1],'point_on_axis':[0,0,0]}}
        self.assertLess(prediction(T,f)['position_rmse_m'],1e-12)
        T[:,2,3]=np.linspace(0,.002,80)
        p=prediction(T,f)
        self.assertGreater(p['endpoint_error_m'],.0019);self.assertFalse(p['reference_fit_on_heldout'])

    def test_model_only_event_is_not_a_physical_failure(self):
        with tempfile.TemporaryDirectory() as d:
            p={'confidence':{'soft_trigger_m':.001,'soft_duration_s':.1}}
            m=OperationalMemory(np.eye(4),Path(d),p)
            for i in range(20):m.monitor(.0011,.01,i*.01,'ESTIMATED_FOLLOW')
            self.assertTrue(m.pending);self.assertFalse(m.events[0]['physical_failure'])

    def test_generated_code_preserves_guards_and_excludes_holdout_fit(self):
        from run_operational_structure_episode import source
        s=source();ast.parse(s)
        self.assertNotIn("max(s['relative_translation_slip_m'],s['articulation_consistency_error_m']",s)
        self.assertIn("s['relative_translation_slip_m']>policy['max_slip_m']",s)
        for x in ['SUSTAINED_CONTACT_LOSS','EXISTING_LOW_PRELOAD_FORCE_LIMIT','LOW_JOINT_MARGIN','EXISTING_ARM_SPEED_LIMIT','EXISTING_ARM_EFFORT_LIMIT']:
            self.assertIn(x,s)
        self.assertIn("if s['margin_rad']<=.05",s)
        held=s[s.index("phase='STRUCTURE_VALIDATION_SETTLE'"):s.index("phase='FINAL_HOLD';drive.active=False;hold(2.)")]
        self.assertNotIn('memory.observe(',held);self.assertNotIn('memory.refresh()',held)

    def test_repeated_static_refits_cannot_trip_consecutive_segment_gate(self):
        with tempfile.TemporaryDirectory() as d:
            m=OperationalMemory(np.eye(4),Path(d),{'confidence':{'hard_window_motion_m':.001}})
            T=np.tile(np.eye(4),(48,1,1))
            for _ in range(20):m.count_window(False,T)
            self.assertEqual(m.failed_refits,0)
            for j in range(1,4):
                X=T.copy();X[:,0,3]=np.linspace((j-1)*.0011,j*.0011,48);m.count_window(False,X)
            self.assertEqual(m.failed_refits,3)
            m.count_window(True,X);self.assertEqual(m.failed_refits,0)

    def test_tangent_requires_one_mm_motion_not_micro_frame_derivative(self):
        th=np.linspace(0,.001,80);T=np.tile(np.eye(4),(80,1,1));T[:,:3,:3]=Rotation.from_rotvec(np.c_[th*0,th*0,th]).as_matrix()
        T[:,:3,3]=np.c_[.5*np.cos(th),.5*np.sin(th),th*0]
        f={'revolute':{'axis':[0,0,1],'point_on_axis':[0,0,0]}}
        self.assertIsNone(prediction(T,f)['tangent_error_deg'])
        self.assertIsNotNone(prediction(T,f)['raw_frame_tangent_error_deg'])


if __name__=='__main__':unittest.main()
