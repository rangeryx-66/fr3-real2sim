"""Execute one preplanned released-object base switch in KNOWN_MODEL_DIAGNOSTIC."""
from pathlib import Path
import sys,ast
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_known_model_whole_episode import source as single_source


def source():
 text=single_source()
 def replace(old,new):
  nonlocal text
  if text.count(old)!=1:raise RuntimeError('TWO_STATION_HOOK_CHANGED:'+old[:100])
  text=text.replace(old,new)
 replace("kind=job['skill']['joint_type'];max_state=0.;", "kind=job['skill']['joint_type'];base_moves=0;regrasp_count=0;initial_base=list(whole['base']);max_state=0.;")
 replace("('SETTLE','PREGRASP','APPROACH','NO_OPERATION')", "('SETTLE','PREGRASP','APPROACH','NO_OPERATION','CLEARANCE_RETREAT','BASE_ROUTE','SECOND_APPROACH')")
 replace("if phase in ('CLOSE','SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD','SMALL_PULL','OPEN_5_DEG','FINAL_HOLD') and s['force_event']['status'].startswith", "if s['force_event']['status'].startswith")
 # Preserve the original first closure, clone it for the one planned regrasp.
 baseline=(ROOT/'scripts/run_piper_contact_baseline.py').read_text()
 ca=baseline.index("   closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'")
 cb=baseline.index("   E0=tcp();D0=moving();theta0=",ca)
 closure=baseline[ca:cb]
 # The exact baseline gain/effort transition is reused after the planned release.
 aa=text.index("   E0=tcp();D0=moving();theta0=")
 ab=text.index("   # Smooth spline ends",aa)
 second=text[aa:ab].replace("whole['path']", "whole['second_station']['path']").replace("'aligned_whole_path.json'", "'aligned_second_leg.json'")
 insertion='''   # The whole switch route was certified before starting this trial.
   phase='SWITCH_SAFE_HOLD';qvelocity=np.zeros(6);hold(2.)
   switch_actual=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
   if abs(switch_actual-whole['switch_state'])>np.deg2rad(2.):raise RuntimeError('PLANNED_SWITCH_STATE_NOT_REACHED')
   reference=None;planned_reference=None;phase='RELEASE';effort=0.;mode='position'
   kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kp[fingers]=1000.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True)
   opening=float(whole['switch_retreat']['opening_m']);present=float(rows[-1]['aperture_m'])
   # Release slowly; no stale closed-position command replaces the preload hold.
   for w in np.linspace(present,opening,max(2,int(abs(opening-present)/(.001*dt))+1)):
    qtarget[fingers]=[w/2,-w/2];step()
   hold(.3)
   if abs(rows[-1]['aperture_m']-opening)>.001:raise RuntimeError('RELEASE_APERTURE_TRACKING_IMPLEMENTATION_ERROR')
   if np.any(filtered()>.05):raise RuntimeError('PLANNED_RELEASE_CONTACT_REMAINS')
   phase='CLEARANCE_RETREAT'
   for q in whole['switch_retreat']['q_path'][1:]:
    distance=np.linalg.norm(model.poses(q,base,width=opening)['tcp_link'][:3,3]-tcp()[:3,3]);move(q,max(.25,1.5*distance/.003))
   hold(.3)
   released_state=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
   if abs(released_state-switch_actual)>np.deg2rad(2.):raise RuntimeError('OBJECT_REBOUND_INVALIDATES_PLANNED_SWITCH')
   from isaacsim.core.prims import SingleXFormPrim
   from interactive_twin_recovery.mobile import scene_at
   support=SingleXFormPrim('/World/r1a7_pedestal');chassis=SingleXFormPrim('/World/mobile_chassis');phase='BASE_ROUTE'
   for target_base in whole['base_route']['waypoints'][1:]:
    old_base=np.asarray(base).copy();delta_base=np.asarray(target_base)-old_base;duration=max(np.linalg.norm(delta_base[:2])/.01,abs(delta_base[3])/5.,dt)
    for f in np.linspace(0.,1.,max(2,int(duration/dt)))[1:]:
     pose=old_base+f*delta_base;quat=np.roll(Rotation.from_euler('z',pose[3],degrees=True).as_quat(),1)
     # Existing kinematic SE(2) platform, only after verified release/clearance.
     robot.set_world_pose(pose[:3],quat);support.set_world_pose([pose[0],pose[1],-.33],quat);chassis.set_world_pose([pose[0],pose[1],-.66],quat)
     base[:]=pose.tolist();B=transform(base[:3],[0,0,np.deg2rad(base[3])]);
     if tick%8==0:collision=scene_at(ROOT,export,model,initial_base,base)
     step()
   base_moves+=1;hold(.3)
   now=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
   if abs(now-whole['switch_state'])>np.deg2rad(2.):raise RuntimeError('OBJECT_STATE_CHANGED_DURING_PLANNED_BASE_ROUTE')
   phase='SECOND_APPROACH'
   for q in whole['second_station']['preplan'][1:]:move(q,1.5)
   for q in whole['second_station']['approach'][1:]:move(q,.15)
   # Reclose with the original bilateral verification and same preload command.
   legal=False;loss_s=0.;slip_s=0.
'''+closure+'''   regrasp_count+=1
'''+second
 replace("   # Smooth spline ends",insertion+"   # Smooth spline ends")
 replace("'base_fixed':base", "'base_fixed':initial_base,'final_base':base,'base_moves':base_moves,'regrasp_count':regrasp_count,'bounded_station_switches':1")
 ast.parse(text);return text

if __name__=='__main__':
 text=source();exec(compile(text,str(ROOT/'scripts/run_known_model_two_station_episode.py'),'exec'),globals())
