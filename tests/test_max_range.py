import unittest,ast,json,hashlib
from pathlib import Path
import numpy as np
from wrist_reconstruction.max_range import ReleaseConsensus,template_variants,effort_validity
ROOT=Path(__file__).resolve().parents[1]
class MaximumRangeTests(unittest.TestCase):
 def test_single_spike_then_stable(self):
  c=ReleaseConsensus();self.assertEqual(c.observe(.001679,[1,0,0]),'PERCEPTION_UNCERTAIN');self.assertEqual(c.observe(.0001,[0,0,0]),'STABLE');self.assertEqual(len(c.vectors),0)
 def test_confirmed_motion_requires_three(self):
  c=ReleaseConsensus();self.assertEqual(c.observe(.002,[1,0,0]),'PERCEPTION_UNCERTAIN');self.assertEqual(c.observe(.002,[1,0,0]),'PERCEPTION_UNCERTAIN');self.assertEqual(c.observe(.002,[1,0,0]),'OBJECT_MOVED_DURING_RELEASE')
 def test_templates_include_proven_pose_and_twelve_unique_trials(self):
  H=np.eye(4);H[:3,3]=[.1,.2,.3];L=np.eye(4);L[:3,3]=[0,0,.01]
  rows=list(template_variants({'T_handle_TCP':L},H));self.assertEqual(len(rows),12);np.testing.assert_allclose(rows[0]['T'],H@L);self.assertEqual([r['candidate_index'] for r in rows],list(range(12)))
 def test_sliding_only_invalidates_effort(self):
  status,why=effort_validity({'phase':'ESTIMATED_FOLLOW','forces_n':{'a':.3,'b':.3},'relative_translation_slip_m':.003},.001);self.assertEqual(status,'INVALID_FOR_EFFORT');self.assertIn('RELATIVE_GRASP_MOTION',why)
 def test_mode_hooks_keep_legacy_and_native_safety(self):
  import sys
  sys.path.insert(0,str(ROOT/'scripts'))
  from run_wrist_reconstruction_episode import source
  legacy=source(False);new=source(True);ast.parse(new)
  self.assertIn("raise RuntimeError('SUSTAINED_CONTACT_PLANE_DRIFT')",legacy);self.assertNotIn("raise RuntimeError('SUSTAINED_CONTACT_PLANE_DRIFT')",new)
  for s in ["raise RuntimeError('SUSTAINED_CONTACT_LOSS')","if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')","raise RuntimeError('EXISTING_LOW_PRELOAD_FORCE_LIMIT')"]:self.assertIn(s,new)
  self.assertIn("runtime.recovery=recovery",new)
  self.assertIn("mode=mode if reference is not None else 'position'",new)
  self.assertIn("runtime.stop_failed_grasp_monitor=stop_failed_grasp_monitor",new)
  self.assertIn("if isinstance(memory,ProvisionalMemory):s['global_reconstruction_consistency_error_m']",new)
  self.assertIn("BASE_ROUTE_REQUIRES_ACTUAL_ZERO_LOAD_AND_RELEASE_APERTURE",new)
 def test_home_not_required_for_clearance_mode(self):
  text=(ROOT/'wrist_reconstruction/retreat.py').read_text();self.assertIn("'home':None",text);self.assertIn("if plan['home'] is not None",text)
 def test_three_failed_real_closures_continue_to_fourth_template(self):
  from types import SimpleNamespace
  import tempfile
  from wrist_reconstruction.max_recovery import MaximumRecovery
  with tempfile.TemporaryDirectory() as d:
   rec=MaximumRecovery.__new__(MaximumRecovery);rec.cycles=0;rec.count=0;rec.history=[];rec.current_D=np.eye(4);rec.grasp_reference_ee=np.eye(4);rec.grasp_reference_D=np.eye(4);rec.released=False;rec.template={}
   rec.policy={'operation_cycles':12,'actual_base_attempts':4,'template_closures':12,'generic_closures':12}
   attempts=[]
   def close(choice,correction):
    self.assertFalse(rec.released)
    attempts.append(choice['plan']['trial_candidates'][0]['candidate_index'])
    if len(attempts)<=3:raise RuntimeError('RECOVERY_BILATERAL_GRASP_FAIL')
   r=SimpleNamespace(tcp=lambda:np.eye(4),base=[0,0,0,0],capture=SimpleNamespace(output=Path(d)),hold=lambda _:None,halt_at_measured_state=lambda:None,set_observed_moving=lambda _:None,regrasp=close,reset_grasp_memory=lambda:None,arm_q=lambda:np.zeros(6))
   rec.r=r;rec.observe=lambda *a,**k:(np.eye(4),{'source':'wrist'},np.zeros((80,3)))
   def escape(D,row):rec.released=True
   rec.escape=escape;rec.remember_success=lambda _:None
   rec.candidates=lambda D,f:[{'candidate_index':i,'family':f} for i in range(12)]
   rec.plan_variant=lambda v,*args:v
   self.assertTrue(rec.run(0));self.assertEqual(attempts,[0,1,2,3]);self.assertTrue(rec.history[-1]['regrasp_completed'])
 def test_release_ignores_newly_revealed_surface_pixels(self):
  from wrist_reconstruction.self_observation import common_release_points
  K=np.array([[20.,0,5],[0,20.,5],[0,0,1.]])
  depth=np.full((12,12),.5);robot=np.zeros((12,12),bool);robot[:,:4]=True
  A={'K':K,'T_world_camera':np.eye(4),'depth_m':depth,'robot_q_self_mask':robot}
  B=dict(A,depth_m=depth.copy(),robot_q_self_mask=np.zeros_like(robot));B['depth_m'][:,:4]=.3
  P,Q=common_release_points(A,B,[0,0,.5],1.);np.testing.assert_allclose(P,Q)
 def test_original_baseline_hashes(self):
  for p,digest in json.loads((ROOT/'wrist_reconstruction/frozen_baseline_hashes.json').read_text()).items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),digest,p)
if __name__=='__main__':unittest.main()
