import ast,sys,unittest,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from active_structure.discovery import hypotheses

class ActiveStructureTests(unittest.TestCase):
    def setUp(self):self.policy=json.loads((ROOT/'configs/active_structure.yaml').read_text())['active']['discovery']
    def test_linear_motion_does_not_become_provisional_revolute(self):
        T=np.tile(np.eye(4),(120,1,1));T[:,0,3]=np.linspace(0,.02,120)
        self.assertEqual(hypotheses(T,self.policy)['status'],'UNOBSERVABLE')
    def test_short_stable_arc_may_be_provisional_but_not_final(self):
        theta=np.linspace(0,np.deg2rad(.8),120);T=np.tile(np.eye(4),(120,1,1))
        T[:,:3,:3]=Rotation.from_rotvec(np.c_[theta*0,theta*0,theta]).as_matrix()
        T[:,:3,3]=np.c_[1.5*np.cos(theta),1.5*np.sin(theta),theta*0]
        d=hypotheses(T,self.policy)
        self.assertEqual(d['status'],'PROVISIONAL_REVOLUTE');self.assertFalse(d['strict_acceptance'])
        json.dumps(d)  # Native logging must accept NumPy-derived comparisons.
    def test_noisy_axes_cannot_authorize_refinement(self):
        theta=np.linspace(0,.03,120);T=np.tile(np.eye(4),(120,1,1));T[:,0,3]=np.linspace(0,.02,120)
        vectors=np.c_[theta*.8*np.sin(np.linspace(0,5,120)),theta*0,theta]
        T[:,:3,:3]=Rotation.from_rotvec(vectors).as_matrix()
        self.assertEqual(hypotheses(T,self.policy)['status'],'UNOBSERVABLE')
    def test_joint_model_prediction_uses_fitted_reference_pose(self):
        from active_structure.discovery import ProvisionalMemory
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            initial=np.eye(4);initial[0,3]=1.5
            reference=initial.copy();reference[:3,:3]=Rotation.from_euler('z',.001).as_matrix()
            memory=ProvisionalMemory(initial,Path(d),{})
            memory.estimate={'reference_pose':reference.tolist(),'revolute':{'axis':[0,0,1],'point_on_axis':[0,0,0]}}
            E=reference.copy();E[:3,:3]=Rotation.from_euler('z',.021).as_matrix();E[:3,3]=Rotation.from_euler('z',.02).apply(reference[:3,3])
            self.assertLess(memory.consistency_error(E),1e-10)
            bad=E.copy();bad[2,3]=.0011
            self.assertGreater(memory.consistency_error(bad),.001)  # unchanged safety threshold still meaningful

    def test_runtime_hooks_compile_and_keep_safety(self):
        from run_active_structure_episode import source
        s=source();ast.parse(s)
        for guard in ['LOW_JOINT_MARGIN','SUSTAINED_CONTACT_LOSS','SUSTAINED_RELATIVE_SLIP','EXISTING_LOW_PRELOAD_FORCE_LIMIT','EXISTING_ARM_EFFORT_LIMIT']:
            self.assertIn(guard,s)
        self.assertIn("if s['margin_rad']<=.05",s)
        self.assertIn("selected=refine(rows,discovery_snapshot",s)
        self.assertNotIn('job[\'GT_axis\']',s)

if __name__=='__main__':unittest.main()
