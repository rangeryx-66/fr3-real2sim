"""Bounded explicit Agent/contact handoff; frozen controllers are called intact.

Only an unloaded, collision-checked observation state may wait for the Agent.
An original hard exception is never ignored to keep pulling or clear history.
"""
import json,os,shutil,time,traceback
from pathlib import Path
from unittest.mock import patch
import numpy as np
import native_g1_agent_observation as observation
import native_g1_agent_incremental_activation as incremental
import native_g1_agent_execution as physical
import native_g1_multiview_input as multi
import native_g1_recovery_telemetry as passive
import native_g1_wrist_input as wrist
from native_g1_wrist_input import install_camera
from native_g1_wrist_views import survey_candidates,select_route
from precontact_trajectory import segment_samples


class TrialFeedback(RuntimeError):pass


def short_outcome(acquired,gain,target_reached,reason):
    # The registered >=2deg transmission criterion is independent of nominal
    # +3deg completion. An original failure/physical hard stop is never PASS.
    return bool(acquired and gain>=2. and (target_reached or 'EXISTING_SOFT_FORCE_WARNING_RETURN_FOR_UNLOAD' in reason))


def write(g,name,data):observation.write(g['a'].output/name,data)
def family(candidate):return tuple(float(candidate[k]) for k in ['section_fraction','insertion_m','roll_deg','pitch_deg'])
def set_state(g,state):g['recovery_state']=state


def observe_current(g):
    save=observation.save_rgbd;generation=len(g['observation_generations'])
    def capture(state,label):return save(state,'generation_%02d_'%generation+label)
    with patch.object(observation,'save_rgbd',capture):return incremental.observe(g)


def refine_current(g):
    save=wrist.save_rgbd
    def capture(state,label):return save(state,'attempt_%02d_'%g['attempt_index']+label)
    with patch.object(wrist,'save_rgbd',capture):return wrist.refine(g)


def build_geometry(g):
    """Same scorer; only episode-local attempted families leave the new pool."""
    attempted={tuple(r['family']) for r in g['contact_attempts'] if r['physical_grasp_started'] and not r['short_success']}
    original=multi.rescore
    def alternatives(*args,**kwargs):
        scored=original(*args,**kwargs)
        remaining=[c for c in scored if (float(c['section']),float(c['insertion_m']),float(c['roll']),float(c['pitch'])) not in attempted]
        write(g,'EPISODE_CANDIDATE_MEMORY.json',dict(failed_families=list(attempted),before=len(scored),after=len(remaining),scope='this episode only; scores unchanged; no object blacklist'))
        return remaining
    with patch.object(multi,'rescore',alternatives):result=incremental.build_geometry(g)
    if result.get('plan'):
        candidate=json.loads((g['a'].output/'visual_plan.json').read_text())
        result['candidate_family']=list(family(candidate))
        result['failed_episode_families']=list(attempted)
    return result


def checked_route(g,path):
    """Existing .05 and collision predicates, using measured released fingers."""
    fingers=np.asarray(g['robot'].get_joint_positions())[g['fingers']];samples=0
    for a,b in zip(path,path[1:]):
        for q in np.linspace(a,b,max(2,int(np.ceil(np.max(abs(np.asarray(b)-a))/.025))+1)):
            samples+=1
            if g['model'].margin(q)<=.05:return dict(valid=False,reason='LOW_JOINT_MARGIN',samples=samples)
            ok,why=g['collision'].check(g['model'].poses(q,g['base'],finger_q=fingers),g['moving'](),False)
            if not ok:return dict(valid=False,reason=why,samples=samples)
    return dict(valid=True,samples=samples,measured_fingers_rad=fingers.tolist())


def release_geometry(g,start_width):
    q=np.asarray(g['robot'].get_joint_positions())[g['arm']];audit=[]
    for w in np.linspace(start_width,.084,35):
        if g['model'].margin(q)<=.05:return dict(valid=False,reason='LOW_JOINT_MARGIN')
        ok,why=g['collision'].check(g['model'].poses(q,g['base'],width=float(w)),g['moving'](),True)
        audit.append(dict(aperture_m=float(w),valid=bool(ok),reason=why))
        if not ok:return dict(valid=False,reason=why,samples=audit)
    return dict(valid=True,samples=audit,arm_motion=False,object_motion_command=False)


def slow_retreat(g,path):
    g['phase']='CLEARANCE_RETREAT'
    for end in path[1:]:
        for q,v in segment_samples(g['solver'],g['qtarget'][g['arm']].copy(),end,.25,g['dt'],.003):
            g['qtarget'][g['arm']]=q;g['qvelocity']=v;g['step']()
    g['qvelocity']=np.zeros(len(g['arm']))


def plan_clearance(g):
    T=g['tcp']().copy();seed=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    directions=[-T[:3,2],np.asarray(g['normal']),np.asarray(g['normal'])+np.array([0.,0.,1.])];alternatives=[]
    for direction in directions:
        direction=direction/np.linalg.norm(direction);path=[seed.tolist()];q=seed.copy();reason=None
        for distance in np.linspace(0,.04,21)[1:]:
            target=T.copy();target[:3,3]+=distance*direction;q=g['ik'](target,q)
            if q is None:reason='NO_RETREAT_IK';break
            path.append(q.tolist())
        check=checked_route(g,path) if reason is None else dict(valid=False,reason=reason)
        alternatives.append(dict(direction=direction.tolist(),check=check))
        if check['valid']:return dict(path=path,candidates=alternatives)
    return dict(path=None,candidates=alternatives)


def safe_release_retreat(g):
    """Guarded jaw-only unload, then checked slow clearance and free return.

    The SAME TemporalForceGuard remains live. Any original hard exception in
    unloading ends this primitive immediately; no retry-through or reset.
    """
    set_state(g,'SAFE_UNLOAD');g['trial_running']=False;g['opening']=False
    g['qvelocity']=np.zeros(len(g['arm']));g['planned_reference']=None
    out=dict(start_sim_s=g['tick']*g['dt'],force_history_reset=False,arm_held_during_release=True)
    try:
        width=float(g['rows'][-1]['aperture_m']);audit=release_geometry(g,width);out['jaw_sweep']=audit
        if not audit['valid']:return dict(out,safe=False,reason='RELEASE_SWEEP:'+str(audit['reason']))
        g['world'].play()
        # Neutralize the existing preload at measured opening, then use the
        # original release/slow-close magnitude. No clamp/gain/effort change.
        g['mode']='position';g['effort']=0.;g['phase']='RELEASE'
        out['stopped_grasp_reference']=None if g['reference'] is None else g['reference'].tolist()
        # The frozen release intentionally ends grasp-loss monitoring. Keep
        # its reference archived, and explicitly retain the SAME velocity
        # protections during jaw-only unloading rather than clearing safety.
        g['reference']=None
        def release_step():
            g['step']()
            if np.any(abs(np.asarray(g['robot'].get_joint_velocities())[g['arm']])>g['vel']/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')
            if len(g['rows'])>1 and np.linalg.norm(np.asarray(g['rows'][-1]['T_tcp'])[:3,3]-np.asarray(g['rows'][-2]['T_tcp'])[:3,3])/g['dt']>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')
        rate=g['policy']['slow_closure_m_s'];steps=max(2,int(np.ceil(abs(.084-width)/(rate*g['dt']))))
        for w in np.linspace(width,.084,steps+1):
            g['jaw_width_command']=float(w);g['qtarget'][g['fingers']]=g['controller'].geometry.angle(w);release_step()
        for _ in range(int(.3/g['dt'])):release_step()
        if np.any(g['filtered']()>.05):
            for _ in range(int(1./g['dt'])):release_step()
        if np.any(g['filtered']()>.05):return dict(out,safe=False,reason='RELEASE_CONTACT_REMAINS')
        if not g['force_guard'].settled():return dict(out,safe=False,reason='ORIGINAL_FORCE_HISTORY_NOT_SETTLED')
        # No load-bearing reset: original force history has naturally settled.
        g['force_guard'].pending=False;g['reference']=None;g['loss_s']=0.;g['slip_s']=0.
        g['telemetry_events'].append(dict(event='RECOVERY_PHYSICAL_RELEASE',t=g['tick']*g['dt'],pad_loads_n=g['filtered']().tolist(),force_history_reset=False))
        set_state(g,'CLEARANCE_RETREAT');planning=plan_clearance(g);chosen=planning['path']
        out['retreat_candidates']=planning['candidates']
        if chosen is None:return dict(out,safe=False,reason='NO_CHECKED_CLEARANCE_RETREAT')
        slow_retreat(g,chosen);g['hold'](.3)
        # Reversal is never presumed safe. Clearance first, then validate the
        # whole measured-state route with the currently released fingers.
        home_path=[np.asarray(g['robot'].get_joint_positions())[g['arm']].tolist(),g['qtarget'][g['arm']].tolist(),g['approach'][0]]+list(reversed(g['chosen']['preplan']))
        check=checked_route(g,home_path);out['unloaded_return_check']=check
        if not check['valid']:return dict(out,safe=False,reason='NO_CHECKED_UNLOADED_RETURN:'+str(check['reason']))
        g['phase']='PREGRASP'
        for q in home_path[2:]:g['move'](np.asarray(q),.8)
        g['hold'](.3);g['wrist_routes']=[];set_state(g,'UNLOADED_OBSERVATION')
        out.update(safe=True,end_sim_s=g['tick']*g['dt'],pad_loads_n=g['filtered']().tolist(),measured_q=np.asarray(g['robot'].get_joint_positions())[g['arm']].tolist(),retreat_path=chosen,home_path=home_path)
        return out
    except Exception as error:
        # No hard safety exception is swallowed to continue another step.
        return dict(out,safe=False,reason=repr(error),original_protection_or_interface_exception=True,traceback=traceback.format_exc())


def attempt_grasp_and_probe(g,command):
    if g['physical_grasps_started']>=2:return dict(executed=False,reason='registered two-grasp episode budget exhausted')
    if any(r['short_success'] for r in g['contact_attempts']):return dict(executed=False,reason='first success: recovery prohibited by registration')
    candidate=json.loads((g['a'].output/'visual_plan.json').read_text());key=family(candidate)
    if any(tuple(r['family'])==key and r['physical_grasp_started'] for r in g['contact_attempts']):return dict(executed=False,reason='same failed family already attempted in this episode; reobserve/replan')
    ix=len(g['contact_attempts']);folder=g['a'].output/'attempts'/('%02d'%ix);folder.mkdir(parents=True,exist_ok=True)
    shutil.copytree(g['a'].output/'agent_geometry',folder/'geometry',dirs_exist_ok=True);shutil.copy2(g['a'].output/'visual_plan.json',folder/'selected_plan.json')
    result=dict(index=ix,folder=str(folder),family=list(key),first_row=len(g['rows']),start_sim_s=g['tick']*g['dt'],physical_grasp_started=False,short_success=False,Agent_command=command,observation_routes=list(g['wrist_routes']))
    g['current_attempt']=result;g['attempt_index']=ix;g['trial_running']=False;start_wall=time.perf_counter();event_start=len(g['telemetry_events'])
    g.update(opening=False,legal=False,preload_position_origin=None,mode='position',effort=0.,loss_s=0.,slip_s=0.,reference=None,planned_reference=None)
    set_state(g,'ATTEMPT_APPROACH')
    try:
        # Existing explicit return/handoff and original pregrasp refinement.
        physical.execute(g,command);g['refine']=refine_current;g['trial_running']=True
        initial=g['initial'].replace("move(qpre,.8);phase='APPROACH'","move(qpre,.8);refine(globals());phase='APPROACH'")
        exec(compile(initial,'frozen_initial_sequence::contact_recovery_attempt','exec'),g)
        g['phase']='VALIDATED_PREFIX_EXTRA_HOLD';g['hold'](.5)
        g['opening_start_angle']=g['actual']();g['goal']=g['actual']()+np.deg2rad(3.)
        g['telemetry_events'].append(dict(event='SHORT_OPENING_START',t=g['tick']*g['dt'],angle_deg=float(np.rad2deg(g['actual']())),filtered_pad_loads_n=g['filtered']().tolist(),attempt_index=ix))
        g['grasp_relative']=np.linalg.inv(g['moving']())@g['tcp']();g['reference']=g['grasp_relative'].copy();g['ref']=g['tcp']().copy();g['seed']=np.asarray(g['robot'].get_joint_positions())[g['arm']];g['bias']=g['qtarget'][g['arm']]-g['seed'];g['angle_reference']=g['actual']();g['opening']=True;g['drive_correction']=np.eye(4)
        source=Path(g['spec']['shared_policy']).read_text().replace("g['qvelocity']=np.zeros(6)","g['qvelocity']=np.zeros(7)")
        namespace={};exec(compile(source,'unchanged_shared_opening::joint_count_adapter','exec'),namespace)
        ok,status,_=namespace['open_with_contact_feedback'](g);result['termination_reason']=status;result['controller_target_reached']=bool(ok)
    except Exception as error:result.update(termination_reason=repr(error),traceback=traceback.format_exc(),controller_target_reached=False)
    finally:
        g['trial_running']=False;g['opening']=False;g['qvelocity']=np.zeros(len(g['arm']));set_state(g,'ATTEMPT_STOPPED')
        result['last_row']=len(g['rows']);result['end_sim_s']=g['tick']*g['dt'];passive.snapshot(g,folder,'stopped')
    events=g['telemetry_events'][event_start:]
    for event in events:event['contact_recovery_attempt_index']=ix
    acquired=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None);rows=g['rows'][result['first_row']:result['last_row']]
    after=[r for r in rows if acquired and r['t']>=acquired['t']]
    result.update(acquisition=acquired,opening_gain_deg=max((r['door_angle_deg']-acquired['angle_deg'] for r in after),default=0.),
        force_events=[r['force_event'] for r in rows if r.get('force_event',{}).get('status')!='NORMAL'],
        hold_degradation=dict(single_pad_zero_observed=any(min(r['forces_n'].values())==0 for r in after),not_an_execution_veto=True),
        peak_pad_load_n=max((max(r['forces_n'].values()) for r in rows),default=0.),minimum_joint_margin_rad=min((r['margin_rad'] for r in rows),default=None),
        physical_attempt_wall_s=time.perf_counter()-start_wall)
    result['short_success']=short_outcome(acquired,result['opening_gain_deg'],result['controller_target_reached'],result['termination_reason'])
    result['transmission_gain_ge_2_deg']=bool(acquired and result['opening_gain_deg']>=2.)
    for name in ['visual_correction.json','MEASURED_STATE_APPROACH_CHECK.json','AGENT_EXECUTION_TRANSITION_CHECK.json']:
        if (g['a'].output/name).exists():shutil.copy2(g['a'].output/name,folder/name)
    g['contact_attempts'].append(result)
    # Failure never waits under load. A safety/unknown-configuration exception
    # is preserved; only a guarded, jaw-only release can possibly follow.
    reason=result['termination_reason']
    if any(x in reason for x in ['PHYSICS_CLOCK_MISMATCH','LOW_JOINT_MARGIN','EXISTING_COMMAND_JOINT_MARGIN','SIMULATOR_CLOSED']):
        exit_result=dict(safe=False,reason='No verified unloading route after invalid physics/configuration stop: '+reason)
    else:exit_result=safe_release_retreat(g)
    result['safe_exit']=exit_result
    if exit_result['safe']:
        g['world'].pause();result['inspection']=passive.inspect_result(g,result)
    else:set_state(g,'SAFE_TERMINATED_NO_RECOVERY');g['world'].pause()
    observation.write(folder/'RESULT.json',result);write(g,'CONTACT_ATTEMPTS.json',g['contact_attempts'])
    return result


def initial_observation(g):
    set_state(g,'UNLOADED_OBSERVATION');g['world'].play()
    proposal=next(survey_candidates(g['model'],g['wrist_config']));q=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    route,audit=select_route(g['model'],g['solver'],g['collision'],g['base'],q,[proposal],g['wrist_config'],g['wrist_X'])
    if route is None:g['world'].pause();return dict(observation=False,reason='INITIAL_WRIST_VIEW_UNAVAILABLE',audit=audit)
    observation.actuate(g,route,'initial_current_generation');result=observe_current(g);g['world'].pause();return result


def reobserve_and_replan(g,command):
    if g.get('recovery_state')!='UNLOADED_OBSERVATION':return dict(executed=False,reason='Not in verified unloaded observation state')
    if not g['contact_attempts'] or g['contact_attempts'][-1]['short_success']:return dict(executed=False,reason='No failed first attempt requiring recovery')
    if g['physical_grasps_started']>=2:return dict(executed=False,reason='recovery attempt budget exhausted')
    g['observation_generations'].append(g['agent_observations']);g['agent_observations']=[];g['wrist_routes']=[];g['job']['_visual_proposal_used']=False
    feedback=g['a'].output/'incremental_feedback'
    if feedback.exists():feedback.rename(g['a'].output/('incremental_feedback_generation_%02d'%(len(g['observation_generations'])-1)))
    write(g,'OBSERVATION_GENERATIONS.json',g['observation_generations'])
    result=initial_observation(g);result['episode_failure_memory']=g['contact_attempts'];return result


def run(g):
    install_camera(g);g.update(agent_observations=[],agent_moves=0,contact_attempts=[],physical_grasps_started=0,observation_generations=[],trial_running=False,attempt_index=0)
    g['job']['_visual_proposal_used']=False;passive.install(g);frozen_step=g['step']
    def feedback_step():
        attempt=g.get('current_attempt')
        if g.get('trial_running') and attempt is not None and g['phase'] in ['SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD'] and not attempt['physical_grasp_started']:
            attempt['physical_grasp_started']=True;g['physical_grasps_started']+=1;g['regrasp_count']=g['physical_grasps_started']-1
        row=frozen_step()
        if g.get('trial_running') and attempt is not None and g['phase']=='FORCE_HOLD' and not attempt.get('acquisition_image_saved'):
            passive.snapshot(g,Path(attempt['folder']),'acquisition');attempt['acquisition_image_saved']=True
        if g.get('trial_running') and g['force_guard'].last.get('status')=='SOFT_FORCE_WARNING':raise TrialFeedback('EXISTING_SOFT_FORCE_WARNING_RETURN_FOR_UNLOAD')
        return row
    g['step']=feedback_step
    root=g['a'].output;commands=root/'agent_commands';responses=root/'agent_responses';commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True)
    first=initial_observation(g);write(g,'INITIAL_OBSERVATION_READY.json',first)
    cache={};trace=[];sequence=0;termination='AGENT_FINISHED';started=time.perf_counter()
    while True:
        write(g,'WAITING_FOR_AGENT.json',dict(sequence=sequence,pid=os.getpid(),world_paused=True,recovery_state=g['recovery_state'],verified_unloaded=True,physical_grasps_used=g['physical_grasps_started'],adjustments_used=g['agent_moves'],adjustment_budget=4))
        path=commands/('%03d.json'%sequence);wait=time.perf_counter()
        while not path.exists():
            if time.perf_counter()-wait>g['job']['agent_wait_timeout_s']:raise KeyboardInterrupt('AGENT_TIMEOUT_UNLOADED_SAFE_EXPORT')
            if (root/'STOP_AND_EXPORT').exists():raise KeyboardInterrupt('OWNER_REQUESTED_UNLOADED_EXPORT')
            if not g['app'].is_running():raise RuntimeError('SIMULATOR_CLOSED')
            g['app'].update();time.sleep(.2)
        command=json.loads(path.read_text());op=command['operation'];began=time.perf_counter()
        try:
            if op=='observe':result=observe_current(g)
            elif op=='inspect':result=observation.inspect(Path(g['agent_observations'][-1]['folder']),g['agent_observations'])
            elif op=='evaluate_view':result=observation.evaluate_view(g,command['target']);cache[sequence]=result
            elif op=='move_and_observe':
                evaluated=cache.get(command['evaluated_command'])
                if g['agent_moves']>=4:result=dict(executed=False,reason='registered four-adjustment budget exhausted')
                elif not evaluated or not evaluated['feasible']:result=dict(executed=False,reason='No checked route for requested view')
                else:
                    g['world'].play();observation.actuate(g,evaluated['route'],'Agent_adjustment');g['agent_moves']+=1;result=observe_current(g);g['world'].pause()
            elif op=='build_geometry':result=build_geometry(g)
            elif op=='validate_execution':result=physical.validate_execution(g)
            elif op=='attempt_grasp_and_probe':result=attempt_grasp_and_probe(g,command)
            elif op=='inspect_contact_result':result=passive.inspect_result(g,g['contact_attempts'][-1])
            elif op=='safe_release_retreat':result=dict(safe=True,already_unloaded=True) if g['recovery_state']=='UNLOADED_OBSERVATION' else safe_release_retreat(g)
            elif op=='reobserve_and_replan':result=reobserve_and_replan(g,command)
            elif op=='finish':result=dict(completed=True,reason=command.get('reason'))
            else:result=dict(error='Unknown explicit recovery operation')
        except Exception as error:
            result=dict(error=repr(error),traceback=traceback.format_exc(),scene_preserved=True)
            if g['recovery_state']!='UNLOADED_OBSERVATION':termination='INTERFACE_OR_SAFETY_FAILURE';write(g,'INTERFACE_FAILURE.json',result);break
        entry=dict(sequence=sequence,command=command,result=result,agent_wait_wall_s=began-wait,operation_wall_s=time.perf_counter()-began)
        trace.append(entry);write(g,'AGENT_ACTION_TRACE.json',trace);observation.write(responses/('%03d.json'%sequence),entry)
        if g['recovery_state']=='SAFE_TERMINATED_NO_RECOVERY':termination='SAFE_EXIT_UNAVAILABLE';break
        if op=='finish':break
        sequence+=1
    success=any(r['short_success'] and r['safe_exit']['safe'] for r in g['contact_attempts'])
    result=dict(success=success,status='SHORT_OPENING_SUCCESS' if success else termination,attempts=g['contact_attempts'],physical_grasp_attempts=g['physical_grasps_started'],recovery_grasps=max(0,g['physical_grasps_started']-1),Agent_adjustments=g['agent_moves'],observation_generations=g['observation_generations']+[g['agent_observations']],scene_restarts=0,GT_grasp_fallback=False,simulation_s=g['tick']*g['dt'],wall_s=time.perf_counter()-g['_profile_entry'],interface_wall_s=time.perf_counter()-started)
    write(g,'RECOVERY_EPISODE_RESULT.json',result);return result
