import ast,sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from paper_structure.analysis import object_prediction,trajectory_metrics
from run_paper_structure_episode import source

class PaperTests(unittest.TestCase):
    def test_grasp_prefix_unchanged_across_methods(self):
        s=[source(m) for m in ('B0','B1','B2')]
        # Memory choice is post-grasp only. Same physical approach and closure.
        prefix=[x[x.index("   phase='PREGRASP'"):x.index("   E0=tcp();memory=InteractionMemory")] for x in s]
        self.assertEqual(prefix[0],prefix[1]);self.assertEqual(prefix[1],prefix[2])
        for x in s:
            ast.parse(x)
            for label in ['LOW_JOINT_MARGIN','SUSTAINED_CONTACT_LOSS','EXISTING_ARM_EFFORT_LIMIT','EXISTING_ARM_SPEED_LIMIT','EXISTING_LOW_PRELOAD_FORCE_LIMIT','SUSTAINED_CONTACT_PLANE_DRIFT']:
                self.assertIn(label,x)
    def test_gt_object_logger_has_no_return_channel(self):
        from paper_structure.evaluation_logger import EvaluationLogger
        import inspect
        text=inspect.getsource(EvaluationLogger.append)
        self.assertFalse(any(isinstance(n,ast.Return) for n in ast.walk(ast.parse(__import__('textwrap').dedent(text)))))
        for m in ('B0','B1','B2'):
            x=source(m);self.assertEqual(x.count('evaluation_writer.append('),1)
            self.assertNotIn('evaluation_writer._rows',x)
    def test_object_prediction_does_not_absorb_drift(self):
        theta=np.linspace(0,.08,80);R=Rotation.from_rotvec(np.c_[0*theta,0*theta,theta]).as_matrix()
        E=np.tile(np.eye(4),(80,1,1));E[:,:3,:3]=R;E[:,:3,3]=np.c_[.5*np.cos(theta),.5*np.sin(theta),0*theta]
        O=E.copy();O[:,:3,3]=np.c_[.3*np.cos(theta),.3*np.sin(theta),0*theta]
        f={'revolute':{'axis':[0.,0.,1.],'point_on_axis':[0,0,0]}}
        self.assertLess(object_prediction(E,O,f)['object_position_rmse_m'],1e-12)
        O[:,2,3]=np.linspace(0,.002,80)
        self.assertGreater(object_prediction(E,O,f)['object_endpoint_error_m'],.00199)
    def test_normal_object_motion_is_not_slip(self):
        theta=np.linspace(0,.08,80);R=Rotation.from_rotvec(np.c_[0*theta,0*theta,theta]).as_matrix()
        O=np.tile(np.eye(4),(80,1,1));O[:,:3,:3]=R;O[:,:3,3]=np.c_[.3*np.cos(theta),.3*np.sin(theta),0*theta]
        G=np.eye(4);G[:3,3]=[.01,.02,.03];E=O@G
        rows=[{'phase':'EXPLORATORY','t':i*.01,'T_ee':e.tolist(),'T_object':o.tolist()} for i,(e,o) in enumerate(zip(E,O))]
        self.assertLess(trajectory_metrics(rows)['maximum_true_relative_translation_drift_m'],1e-12)
        E[-1,2,3]+=.002;rows[-1]['T_ee']=E[-1].tolist()
        self.assertGreater(trajectory_metrics(rows)['maximum_true_relative_translation_drift_m'],.00199)
    def test_final_test_not_in_online_model_selection(self):
        for method in ['B1','B2']:
            x=source(method);s=x[x.index("phase='HELDOUT_MANIPULATION'"):x.index("    phase='FINAL_HOLD';drive.active=False;hold(2.)")]
            self.assertNotIn('memory.observe(',s);self.assertNotIn('memory.refresh()',s);self.assertNotIn('refine(rows',s)
        b0=source('B0');s=b0[b0.index('   ap=job'):b0.index(' except BaseException')]
        self.assertNotIn('try_fit(',s);self.assertNotIn('fit_se3(',s);self.assertNotIn('memory.tangent(',s)

if __name__=='__main__':unittest.main()
