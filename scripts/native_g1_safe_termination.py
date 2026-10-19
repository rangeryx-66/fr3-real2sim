"""Explicit V1.1 exit dependency; frozen V1 controller/Agent loop stay intact."""
import traceback
from unittest.mock import patch
import numpy as np
import native_g1_contact_recovery as recovery
from native_g1_contact_recovery import set_state,release_geometry,slow_retreat,checked_route,write
from native_g1_safe_park import plan_exit,route_to_joint,verify_park


def checked_motion(g,end,clearance):
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    audit=checked_route(g,[measured,g['qtarget'][g['arm']],end])
    if not audit['valid']:raise RuntimeError('SAFE_EXIT_SEGMENT_UNAVAILABLE:'+str(audit['reason']))
    if clearance:slow_retreat(g,[measured,np.asarray(end)])
    else:g['phase']='PREGRASP';g['move'](np.asarray(end),.8)
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    check=checked_route(g,[measured,g['qtarget'][g['arm']]])
    if not check['valid']:raise RuntimeError('SAFE_EXIT_MEASURED_REVALIDATION:'+str(check['reason']))
    return dict(t=g['tick']*g['dt'],measured_q=measured.tolist(),check=check,moving_pose=np.asarray(g['moving']()).tolist())


def safe_release_park(g):
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
        out.update(released=True,terminal_state='RELEASED')
        planning=plan_exit(g);out['joint_exit_plan']=planning
        write(g,'SAFE_EXIT_PLAN_%02d.json'%g['attempt_index'],planning)
        if not planning['valid']:return dict(out,safe=False,terminal_state='SAFE_EXIT_UNAVAILABLE',reason=planning['reason'])
        set_state(g,'CLEARANCE_RETREAT');out['executed_segment_checks']=[]
        for q in planning['clearance_path'][1:]:out['executed_segment_checks'].append(checked_motion(g,q,True))
        g['hold'](.3);out.update(clearance_reached=True,terminal_state='CLEARANCE_REACHED')
        g['telemetry_events'].append(dict(event='CLEARANCE_REACHED',t=g['tick']*g['dt']))
        # Actual tracking and naturally moving door can invalidate a previously
        # checked segment. Revalidate before return, not after a collision.
        measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
        onward=planning['parking_path'];audit=checked_route(g,[measured,g['qtarget'][g['arm']]]+onward[1:])
        out['post_clearance_return_audit']=audit
        if not audit['valid']:
            onward,why=route_to_joint(g,measured,planning['parking_q'])
            out['post_clearance_local_replan']=dict(valid=onward is not None,reason=why,path=onward)
            if onward is None:return dict(out,safe=False,terminal_state='SAFE_EXIT_UNAVAILABLE',reason='CURRENT_STATE_PARK_ROUTE:'+why)
        set_state(g,'SAFE_PARK_MOTION')
        for q in onward[1:]:out['executed_segment_checks'].append(checked_motion(g,q,False))
        g['qvelocity']=np.zeros(len(g['arm']));g['hold'](.3)
        park=verify_park(g,planning);out['parking_verification']=park
        if not park['reached']:return dict(out,safe=False,terminal_state='SAFE_EXIT_UNAVAILABLE',reason='MEASURED_PARK_NOT_REACHED')
        g['wrist_routes']=[];set_state(g,'UNLOADED_OBSERVATION')
        out.update(safe=True,terminal_state='SAFE_PARK_REACHED',safe_park_reached=True,end_sim_s=g['tick']*g['dt'],
            pad_loads_n=g['filtered']().tolist(),measured_q=park['measured_q'],retreat_path=planning['clearance_path'],home_path=onward)
        g['telemetry_events'].append(dict(event='SAFE_PARK_REACHED',t=g['tick']*g['dt'],region=park['region'],measured_q=park['measured_q']))
        return out
    except Exception as error:
        return dict(out,safe=False,terminal_state='SAFE_EXIT_UNAVAILABLE',reason=repr(error),original_protection_or_interface_exception=True,traceback=traceback.format_exc())


def run(g):
    # A single explicit operation dependency, tested before launch. No dynamic
    # function rebinding chain or edits to frozen controller/observation code.
    with patch.object(recovery,'safe_release_retreat',safe_release_park):
        result=recovery.run(g)
    result.update(exit_version='V1.1',
        manipulation_success=any(r['short_success'] for r in result['attempts']),
        full_lifecycle_success=any(r['short_success'] and r['safe_exit'].get('safe_park_reached',False) for r in result['attempts']))
    write(g,'RECOVERY_EPISODE_RESULT.json',result)
    return result
