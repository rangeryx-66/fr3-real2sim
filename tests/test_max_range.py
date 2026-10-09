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
  for s in ["raise RuntimeError('SUSTAINED_CONTACT_LOSS')","if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')","raise RuntimeError(s['force_event']['status'])"]:self.assertIn(s,new)
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
   rec=MaximumRecovery.__new__(MaximumRecovery);rec.cycles=0;rec.count=0;rec.history=[];rec.current_D=np.eye(4);rec.grasp_reference_ee=np.eye(4);rec.grasp_reference_D=np.eye(4);rec.released=False;rec.template={};rec.prior_failures=[]
   rec.policy={'operation_cycles':12,'actual_base_attempts':4,'template_closures':12,'generic_closures':12}
   attempts=[]
   def close(choice,correction):
    self.assertFalse(rec.released)
    attempts.append(choice['plan']['trial_candidates'][0]['candidate_index'])
    if len(attempts)<=3:raise RuntimeError('RECOVERY_BILATERAL_GRASP_FAIL')
   r=SimpleNamespace(tcp=lambda:np.eye(4),base=[0,0,0,0],capture=SimpleNamespace(output=Path(d)),hold=lambda _:None,halt_at_measured_state=lambda:None,set_observed_moving=lambda _:None,regrasp=close,reset_grasp_memory=lambda:None,arm_q=lambda:np.zeros(6))
   r.time=lambda:0.
   rec.r=r;rec.observe=lambda *a,**k:(np.eye(4),{'source':'wrist'},np.zeros((80,3)))
   def escape(D,row,**kwargs):rec.released=True
   rec.escape=escape;rec.remember_success=lambda _:None
   rec.visual_current=lambda D:{'T_world_handle':np.eye(4).tolist()}
   rec.candidates=lambda D,f:[{'candidate_index':i,'family':f,'T':np.eye(4).tolist()} for i in range(12)]
   rec.plan_variant=lambda v,*args:v
   self.assertTrue(rec.run(0));self.assertEqual(attempts,[0,1,2,3]);self.assertTrue(rec.history[-1]['regrasp_completed'])
 def test_release_ignores_newly_revealed_surface_pixels(self):
  from wrist_reconstruction.self_observation import common_release_points
  K=np.array([[20.,0,5],[0,20.,5],[0,0,1.]])
  depth=np.full((12,12),.5);robot=np.zeros((12,12),bool);robot[:,:4]=True
  A={'K':K,'T_world_camera':np.eye(4),'depth_m':depth,'robot_q_self_mask':robot}
  B=dict(A,depth_m=depth.copy(),robot_q_self_mask=np.zeros_like(robot));B['depth_m'][:,:4]=.3
  P,Q=common_release_points(A,B,[0,0,.5],1.);np.testing.assert_allclose(P,Q)
 def test_speed_stop_routes_to_protected_recovery_without_raising_limits(self):
  from wrist_reconstruction.constraint_policy import ConstraintPolicy,Category,State
  import tempfile
  with tempfile.TemporaryDirectory() as d:
   policy=ConstraintPolicy(d)
   result=policy.dispatch('PROBE_CARTESIAN_SPEED_LIMIT','test','physical')
   self.assertEqual(result.category,Category.HARD);self.assertEqual(policy.state,State.SAFE_HOLD)
   policy.transition(State.RELEASE,'checked escape');policy.transition(State.CONTINUE,'safe verified regrasp')
   self.assertEqual(policy.state,State.CONTINUE)
 def test_force_spike_is_warning_sustained_and_emergency_stop(self):
  from wrist_reconstruction.force_policy import TemporalForceGuard
  policy={'soft_force_n':2.,'simulation_emergency_contact_n':10.,'window_s':.05,'sustained_windows':3}
  guard=TemporalForceGuard(policy)
  self.assertEqual(guard.update({'a':2.017,'b':0},1/240,0)['status'],'SOFT_FORCE_WARNING')
  for i in range(48):self.assertFalse(guard.update({'a':.53,'b':.52},1/240,i/240)['status'].startswith('HARD'))
  for i in range(48):status=guard.update({'a':2.2,'b':.5},1/240,i/240)['status']
  self.assertEqual(status,'HARD_FORCE_STOP_SUSTAINED')
  self.assertEqual(guard.update({'a':10.1},1/240,1)['status'],'HARD_FORCE_STOP_EMERGENCY_SIM_ONLY')
 def test_replay_source_reuses_runtime_and_never_object_state(self):
  import sys
  sys.path.insert(0,str(ROOT/'scripts'))
  from run_wrist_reconstruction_episode import source
  from wrist_reconstruction.replay_source import augment
  code=augment(source(True));ast.parse(code)
  self.assertIn('REPLAY_BASE_MOVE_WITH_GRASP_FORBIDDEN',code)
  self.assertIn('recovery.observation_origin_state=0.',code)
  self.assertIn("runtime.initial_grasp_issue='WORKSPACE_RECOVERY'",code)
  self.assertNotIn("scene['articulation'].set_joint_positions",code)
  self.assertIn('if compliant and tape is None:',code)
  self.assertNotIn("if compliant and (tape is None or replay_frame.get('cartesian_input') is not None):",code)
  self.assertIn("tau=np.asarray(replay_frame['arm_effort'])",code)
 def test_empty_or_exhausted_centering_changes_candidate_without_touching_loads(self):
  from types import SimpleNamespace
  from wrist_reconstruction.closure_progress import ClosureProgress
  closure=SimpleNamespace(command=np.eye(4),origin=np.eye(4),p={'max_centering_displacement_m':.004})
  monitor=ClosureProgress()
  for _ in range(4):monitor.update(closure,.0001,[0,0],.1)
  with self.assertRaisesRegex(RuntimeError,'EMPTY_CLOSURE'):monitor.update(closure,.0001,[0,0],.1)
  monitor=ClosureProgress();closure.command=closure.command.copy();closure.command[0,3]=.004
  for _ in range(4):monitor.update(closure,.02,[.1,0],.1)
  with self.assertRaisesRegex(RuntimeError,'CENTERING_EXHAUSTED'):monitor.update(closure,.02,[.1,0],.1)
  monitor=ClosureProgress()
  for _ in range(20):monitor.update(closure,.012,[.5,.5],.1)
 def test_current_handle_frame_controls_direction_without_ee_model(self):
  from types import SimpleNamespace
  from wrist_reconstruction.max_range import operation_tangent
  r=SimpleNamespace(recovery=SimpleNamespace(current_D=np.eye(4),visual_current=lambda D:{'outward_normal_world':[1.,-1.,0.]}))
  np.testing.assert_allclose(operation_tangent(r),np.array([1.,-1.,0.])/np.sqrt(2))
 def test_motion_guess_retains_observed_object_rotation_despite_gripper_rotation(self):
  from types import SimpleNamespace
  from scipy.spatial.transform import Rotation
  from wrist_reconstruction.max_recovery import MaximumRecovery
  rec=MaximumRecovery.__new__(MaximumRecovery);rec.released=False;rec.current_D=np.eye(4);rec.current_D[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()
  rec.grasp_reference_D=np.eye(4);rec.grasp_reference_ee=np.eye(4)
  E=np.eye(4);E[:3,:3]=Rotation.from_euler('z',-20,degrees=True).as_matrix();E[:3,3]=[.1,.2,0]
  rec.r=SimpleNamespace(tcp=lambda:E)
  guess=rec.motion_guess();np.testing.assert_allclose(guess[:3,:3],rec.current_D[:3,:3]);np.testing.assert_allclose(guess[:3,3],E[:3,3])
 def test_original_baseline_hashes(self):
  for p,digest in json.loads((ROOT/'wrist_reconstruction/frozen_baseline_hashes.json').read_text()).items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),digest,p)
if __name__=='__main__':unittest.main()
