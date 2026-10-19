import json,tempfile,unittest
from pathlib import Path
from wrist_reconstruction.constraint_policy import ConstraintPolicy,Event,Category,State,classify,TaskTermination

class PolicyTests(unittest.TestCase):
 def test_recoverable_events_never_finish_task(self):
  codes=['NO_IK','NO_ARM_PATH','WORKSPACE_EXHAUSTED','LOW_JOINT_MARGIN','SCAN_NO_IK','NO_HOME_PATH','OBJECT_MOVED_DURING_RELEASE','RECOVERY_BILATERAL_GRASP_FAIL','SOFT_FORCE_WARNING','CURRENT_HANDLE_REGISTRATION_FAILED']
  with tempfile.TemporaryDirectory() as out:
   p=ConstraintPolicy(out)
   for code in codes:
    d=p.dispatch(code,'recorded guard','operation');self.assertEqual(d.category,Category.RECOVERABLE);self.assertNotIn(p.state,(State.DONE,State.UNRECOVERABLE))
   rows=[json.loads(s) for s in (Path(out)/'constraint_events.jsonl').read_text().splitlines()]
   self.assertEqual([s['event']['code'] for s in rows],codes)
 def test_no_ik_requires_mobile_but_scan_view_tries_pool(self):
  self.assertTrue(classify(Event('NO_INCREMENTAL_IK','control')).force_mobile)
  self.assertEqual(classify(Event('SCAN_NO_IK','view','scan')).next_state,State.WRIST_SCAN)
 def test_native_contact_stops_grasp_but_preflight_rejects_candidate(self):
  self.assertEqual(classify(Event('FINGER_BACK_OR_ROOT_HANDLE_LOAD:gripper_link2','native','physical')).category,Category.HARD)
  self.assertEqual(classify(Event('DANGEROUS_GEOMETRY_COLLISION:panel','planner','preflight')).category,Category.RECOVERABLE)
 def test_motion_metrics_are_diagnostic_and_qa_is_local(self):
  for c in ['GRASP_RELATIVE_MOTION','FINAL_TRUE_RELATIVE_SLIP','MODEL_INCONSISTENCY']:
   d=classify(Event(c,'sensor'));self.assertEqual(d.category,Category.DIAGNOSTIC);self.assertFalse(d.stop_current_action)
  self.assertEqual(classify(Event('QA_OBJECT_CROPPED','camera')).category,Category.DATA)
 def test_unknown_fault_cannot_be_misreported_as_exhaustion(self):
  with tempfile.TemporaryDirectory() as out:
   with self.assertRaisesRegex(TaskTermination,'IMPLEMENTATION_ERROR'):ConstraintPolicy(out).dispatch('unexpected API fault','runtime')
 def test_terminal_outcome_requires_separate_evidence(self):
  with tempfile.TemporaryDirectory() as out:
   p=ConstraintPolicy(out)
   with self.assertRaises(ValueError):p.transition(State.DONE,'NO_IK')
   with self.assertRaises(ValueError):p.finish('NO_IK')
   p.finish('NO_RECOVERY_AVAILABLE',candidates_exhausted=60);self.assertEqual(p.state,State.UNRECOVERABLE)
if __name__=='__main__':unittest.main()

class MobileRecoveryTests(unittest.TestCase):
 def test_no_ik_moves_base_then_reuses_template_before_generic(self):
  import numpy as np
  from types import SimpleNamespace
  from wrist_reconstruction.max_recovery import MaximumRecovery
  with tempfile.TemporaryDirectory() as out:
   rec=MaximumRecovery.__new__(MaximumRecovery);rec.cycles=0;rec.count=0;rec.history=[];rec.current_D=np.eye(4);rec.grasp_reference_D=np.eye(4);rec.grasp_reference_ee=np.eye(4);rec.released=False;rec.template={'saved':True};rec.prior_failures=[]
   rec.policy={'operation_cycles':12,'actual_base_attempts':4,'template_closures':12,'generic_closures':12}
   seen=[];base=[0.,0.,0.,0.]
   r=SimpleNamespace(base=base,tcp=lambda:np.eye(4),time=lambda:1.,capture=SimpleNamespace(output=Path(out)),arm_q=lambda:np.zeros(6),halt_at_measured_state=lambda:None,set_observed_moving=lambda _:None,reset_grasp_memory=lambda:None)
   r.regrasp=lambda choice,_:seen.append(('grasp',choice['plan']['trial_candidates'][0]['family'],list(base)))
   rec.r=r;rec.observe=lambda *a,**k:(np.eye(4),{'wrist':True},np.zeros((80,3)))
   def escape(*a,**k):rec.released=True
   rec.escape=escape;rec.remember_success=lambda _:None
   rec.visual_current=lambda D:{'T_world_handle':np.eye(4).tolist()}
   rec.candidates=lambda D,f:[{'family':f,'candidate_index':0,'T':np.eye(4).tolist()}]
   rec.plan_variant=lambda v,*a:v
   rec.choose_base=lambda *a:{'base':[.1,0.,0.,0.],'route':{}}
   def move(choice):base[:]=choice['base'];seen.append(('base',list(base)));rec.count+=1
   rec.move_base=move
   self.assertTrue(rec.run(0.,reason='NO_INCREMENTAL_IK',force_mobile=True))
   self.assertEqual(seen[0][0],'base');self.assertEqual(seen[1][1],'template');self.assertEqual(rec.count,1)
 def test_registration_motion_is_not_confirmed_fall(self):
  self.assertEqual(classify(Event('OBJECT_MOVED_DURING_RELEASE','ICP','physical')).category,Category.RECOVERABLE)
  self.assertEqual(classify(Event('CONFIRMED_UNCONTROLLED_OBJECT_FALL','verified fall','physical')).category,Category.HARD)

class ProtectiveReleaseTests(unittest.TestCase):
 def test_sustained_force_stops_pull_and_can_only_unload_after_stop(self):
  code='HARD_FORCE_STOP_SUSTAINED'
  pulling=classify(Event(code,'native','physical'))
  unloading=classify(Event(code,'native','protective_release'))
  self.assertEqual(pulling.category,Category.HARD);self.assertEqual(pulling.next_state,State.SAFE_HOLD)
  self.assertEqual(unloading.category,Category.HARD);self.assertEqual(unloading.next_state,State.RELEASE)
  self.assertTrue(unloading.stop_current_action)

class OwnershipTests(unittest.TestCase):
 def test_recovery_reuses_existing_policy_owner(self):
  from types import SimpleNamespace
  from unittest.mock import patch
  from wrist_reconstruction.recovery import Recovery
  from wrist_reconstruction.max_recovery import MaximumRecovery
  with tempfile.TemporaryDirectory() as out:
   p=ConstraintPolicy(out);r=SimpleNamespace(capture=SimpleNamespace(output=Path(out)),constraints=p)
   job={'wrist_experiment':{'maximum_range':{}}}
   def initialize(self,*a):self.r=r;self.job=job
   with patch.object(Recovery,'__init__',initialize):rec=MaximumRecovery()
   self.assertIs(rec.events,p);self.assertIs(r.constraints,p)

class RetreatExecutionTests(unittest.TestCase):
 def test_released_preflight_uses_measured_aperture(self):
  import numpy as np,time
  from types import SimpleNamespace
  from unittest.mock import patch
  from wrist_reconstruction.retreat import RetreatPlanner
  with tempfile.TemporaryDirectory() as out:
   policy={'maximum_alternatives':12,'planning_wall_s':240,'extra_openings_m':[.02,.04],'escape_distance_m':.06,'cartesian_waypoints':2}
   capture=SimpleNamespace(output=Path(out),config={'retreat':policy,'maximum_range':{'enabled':True}})
   E=np.eye(4);goal=E.copy();goal[0,3]=.2
   r=SimpleNamespace(tcp=lambda:E.copy(),arm_q=lambda:np.zeros(6),finger_q=lambda:np.array([.0265,-.0265]),base=[0,0,0,0],capture=capture,deadline=time.time()+1000)
   model=SimpleNamespace(home=np.ones(6),ik=lambda *a,**k:np.zeros(6),margin=lambda _:1.,poses=lambda *a,**k:{'tcp_link':goal})
   widths=[]
   def check(scene,q,b,f):widths.append(float(f[0]-f[1]));return True,'SAFE',1.
   rec=SimpleNamespace(r=r,model=model,released=True,template={'actual_aperture_m':.024},mobile=SimpleNamespace(scene=lambda _:None,check=check),visual_current=lambda _:{'outward_normal_world':[0,0,-1],'anchor_world_m':[0,0,0]})
   planner=RetreatPlanner(rec)
   with patch('wrist_reconstruction.retreat.Model.joint_plan',lambda model,start,end,base,**k:[start.tolist(),end.tolist()]):plans=planner.plans(np.eye(4))
   self.assertTrue(plans);self.assertTrue(all(abs(p['opening']-.053)<1e-10 for p in plans));self.assertTrue(all(abs(w-.053)<1e-10 for w in widths))
 def test_physical_stop_invalidates_release_before_next_plan(self):
  from types import SimpleNamespace
  from wrist_reconstruction.retreat import RetreatPlanner
  from wrist_reconstruction.planner import PlanningExhausted
  with tempfile.TemporaryDirectory() as out:
   def stopped(*a,**k):raise RuntimeError('FINGER_BACK_OR_ROOT_HANDLE_LOAD:gripper_link1')
   r=SimpleNamespace(capture=SimpleNamespace(output=Path(out),config={'retreat':{},'maximum_range':{'enabled':True}}),execute_arm_path=stopped,halt_at_measured_state=lambda:None,return_last_safe=lambda:True)
   rec=SimpleNamespace(r=r,model=None,released=True)
   planner=RetreatPlanner(rec);row={'index':0};planner.rows.append(row)
   with self.assertRaisesRegex(PlanningExhausted,'RETREAT_REPLAN_REQUIRED'):planner.execute({'row':row,'escape':[[0],[1]],'home':None})
   self.assertFalse(rec.released);self.assertTrue(rec.needs_safety_release)

class FailedClosureApertureTests(unittest.TestCase):
 def test_successful_handle_width_is_retained_for_release(self):
  from wrist_reconstruction.retreat import release_aperture
  actual=.00007607030420331284;success=.023941973224282265
  self.assertAlmostEqual(release_aperture(actual,success,.02),success+.02)
  self.assertAlmostEqual(release_aperture(.053,success,.02,certified=True),.053)
  self.assertLessEqual(release_aperture(.09,success,.04),.1)

class BaseClearanceTests(unittest.TestCase):
 def test_base_assisted_clearance_checks_planned_opening_and_tries_next_direction(self):
  import numpy as np
  from types import SimpleNamespace
  from wrist_reconstruction.retreat import RetreatPlanner
  with tempfile.TemporaryDirectory() as out:
   calls=[]
   def route(initial,target,q,fingers):
    calls.append((target,np.asarray(fingers)))
    return {'valid':len(calls)>1,'waypoints':[initial,target]}
   r=SimpleNamespace(base=[.5,-.55,-.1,150.],arm_q=lambda:np.zeros(6),capture=SimpleNamespace(output=Path(out)))
   planner=RetreatPlanner.__new__(RetreatPlanner);planner.r=r;planner.rows=[];planner.policy={'escape_distance_m':.06}
   planner.recovery=SimpleNamespace(mobile=SimpleNamespace(locked_route=route),visual_current=lambda D:{'outward_normal_world':[0.,-1.,0.]})
   plans=planner.base_clearance_plans(np.eye(4),.032)
   self.assertEqual(len(calls),3);self.assertEqual(len(plans),2)
   for base,fingers in calls:
    self.assertEqual(base[2:],r.base[2:]);np.testing.assert_allclose(fingers,[.016,-.016]);self.assertAlmostEqual(np.linalg.norm(np.subtract(base[:2],r.base[:2])),.06)

class LiveRecoveryParkingTests(unittest.TestCase):
 def test_local_exhaustion_keeps_world_then_resumes_same_runtime(self):
  import numpy as np
  from types import SimpleNamespace
  from wrist_reconstruction.live_recovery import wait_for_recovery
  import time
  with tempfile.TemporaryDirectory() as out:
   root=Path(out);calls=[];world=SimpleNamespace(pause=lambda:calls.append('pause'),play=lambda:calls.append('play'),render=lambda:None)
   r=SimpleNamespace(capture=SimpleNamespace(output=root,scene={'world':world}),drive=SimpleNamespace(active=True),halt_at_measured_state=lambda:calls.append('halt'),arm_q=lambda:np.zeros(6),finger_q=lambda:np.array([.02,-.02]),base=[0,0,0,0],time=lambda:123.,deadline=time.time()+5)
   rec=SimpleNamespace(current_D=np.eye(4),template={'saved':True},policy={'operation_cycles':12},cycles=12,visual_current=lambda D:{'source':'sensor'},run=lambda *a,**k:calls.append('existing_recovery') or True)
   (root/'recovery_request.json').write_text(json.dumps({'nonce':'candidate1','action':'retry'}))
   events=ConstraintPolicy(root,r)
   self.assertTrue(wait_for_recovery(r,rec,events,18.6,'NO_ARM_PATH'));self.assertEqual(calls,['halt','pause','play','existing_recovery']);self.assertEqual(events.state,State.CONTINUE)
   self.assertEqual(json.loads((root/'recovery_pending.json').read_text())['status'],'RESUMED');self.assertFalse(r.drive.active)
