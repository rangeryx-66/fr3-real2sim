"""Diagnostic-only unloaded replay of measured interaction TCP motion.

Archived poses are CALIBRATION fixtures, never online grasp/view fallbacks.
Shift the complete measured trajectory away from the observed panel; check all
native geometry with NO handle-contact waiver. Normal actuators execute it.
No object force, joint command, gain change or estimator training is permitted.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def fixture(episode):
    from diagnose_probing_v3 import columns
    from update_probing_v2_model import matrix
    p=Path(episode);a,index=columns(p);normal=np.asarray(json.loads((p/'agent_geometry/observed_geometry.json').read_text())['normal_world'])
    normal/=np.linalg.norm(normal);shift=.04*normal;blocks=[]
    for s in index['segments']:
        if not s['role'].startswith('FIT') or not s.get('result',{}).get('requested_target_reached'):continue
        ids=np.flatnonzero(a['segment']==s['id'])
        if len(ids)<500:continue
        if np.any(np.diff(a['step'][ids])!=1):raise ValueError('Matched fixture needs complete native-rate measured source')
        poses=np.array([matrix(a['tcp_pose'][i])for i in ids]);poses[:,:3,3]+=shift
        blocks.append(dict(role='CALIBRATION_NOT_FIT',source_role=s['role'],poses=poses,
            source_t=a['t'][ids]-a['t'][ids[0]],jaw_width_m=float(np.median(a['aperture'][ids])),
            source_steps=[int(a['step'][ids[0]]),int(a['step'][ids[-1]])]))
        if len(blocks)==2:break
    if len(blocks)!=2:raise ValueError('Two complete measured FIT motions required; no invented profile')
    return blocks,dict(source_episode=p.name,source_index_sha256=hashlib.sha256((p/'telemetry/index.json').read_bytes()).hexdigest(),
        normal_source='archived observed RGB-D panel normal',translation_away_m=shift.tolist(),
        diagnostic_cached_robot_motion=True,online_grasp_or_view_fallback=False,
        same_timing_and_orientation=True,exact_loaded_dynamics_not_reproduced=True)


def check_paths(model,solver,collision,base,moving,blocks,seed):
    checked=0
    for b in blocks:
        for goal in b['poses'][::8]:
            q=solver.solve(goal,base,seed,starts=1)
            if q is None:raise RuntimeError('MATCHED_FREE_PATH_IK_UNAVAILABLE')
            ok,why=collision.check(model.poses(q,base,width=b['jaw_width_m']),moving,False);checked+=1
            if not ok:raise RuntimeError('MATCHED_FREE_PATH_COLLISION:'+why)
            seed=q
    return dict(valid=True,checked=checked,handle_contact_waiver=False)


def preflight(job,episode,output):
    from interaction_runtime.kinematics import FastNativeModel
    from interaction_runtime.prismatic_planning_v73 import native_precision_ik_class
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    from xarm7_native_execution import mapped_export
    p=Path(episode);blocks,meta=fixture(p);source=json.loads((Path(job['source'])/'report.json').read_text())
    cfg=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());model=FastNativeModel(None,Path(job['asset_root']),source)
    solver=native_precision_ik_class()(cfg['gripper_config']);export=json.loads((p/'planning_current_scene.json').read_text())
    allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])
    scene=physical_scene_class(ROOT)(mapped_export(export),model,allowed)
    try:result=check_paths(model,solver,scene,source['robot_base_pose'],scene.moving_reference,blocks,np.asarray(job['robot_start_q']))
    except Exception as e:result=dict(valid=False,reason=repr(e))
    result.update(meta,physics_started=False,scope='offline unloaded path geometry; measured-state route revalidation still required')
    Path(output).parent.mkdir(parents=True,exist_ok=True);Path(output).write_text(json.dumps(result_packet(result),indent=2)+'\n')
    print(json.dumps(result_packet(result)));return result


def run(job,episode):
    from interaction_runtime.prismatic_v73 import PrismaticRuntimeV73,audit_frozen
    from interaction_runtime.policy import SafetyStop
    from native_g1_safe_park import route_to_joint,plan_exit,verify_park
    audit_frozen(ROOT);blocks,registration=fixture(episode);out=Path(job['output'])
    if out.exists():raise ValueError('Preserve diagnostic attempts; use a fresh fixture ID')
    out.mkdir(parents=True);atomic_json(out/'REGISTRATION.json',dict(registration,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        object_actuation_or_force=False,thresholds_changed=False,not_training_or_validation=True))
    r=None;began=time.perf_counter();result=dict(status='NOT_STARTED',blocks=[])
    try:
        r=PrismaticRuntimeV73(job);initial=r.actual();base=r.base;seed=np.asarray(r.robot.get_joint_positions())[r.arm]
        r.g['normal']=np.asarray(registration['translation_away_m'])/.04
        def jaw_to(width):
            r.g['phase']='RELEASE';step=r.policy.data['jaw_release_m_s']*r.dt
            while abs(r.jaw_width-width)>1e-10:
                r.jaw_width+=float(np.clip(width-r.jaw_width,-step,step));r.step()
            r.hold(.5)
        # Preflight a compact geometric sample of BOTH measured paths before any
        # fixture travel. Runtime collision guards remain authoritative at 240Hz.
        result['measured_state_preflight']=check_paths(r.model,r.solver,r.collision,base,r.moving(),blocks,seed)
        for b in blocks:
            jaw_to(b['jaw_width_m'])
            q=np.asarray(r.robot.get_joint_positions())[r.arm];target=r.solver.solve(b['poses'][0],base,q,starts=1)
            if target is None:raise RuntimeError('MATCHED_START_IK_UNAVAILABLE')
            route,reason=route_to_joint(r.g,q,target)
            if route is None:raise RuntimeError('MATCHED_START_ROUTE:'+str(reason))
            r.move_path(route[1:],'PREGRASP')
            r.g['phase']='FORCE_HOLD';r.hold(.6);r.reference_drive=None
            r.record.begin(dict(role='CALIBRATION_NOT_FIT',target_q=r.actual(),coordinate_unit='m',profile='measured_unloaded_replay'),'UNLOADED')
            first=r.tick;tracking=[]
            for goal in b['poses']:
                # Explicitly disallow intended handle contact in the forecast;
                # original actuator, collision and six-axis guards still execute.
                r.cartesian_step(goal,'OPEN_CONTINUE')
                tracking.append(float(np.linalg.norm(r.tcp()[:3,3]-goal[:3,3])))
                if max(r.filtered())>0:raise RuntimeError('CALIBRATION_CONTAMINATED_BY_CONTACT')
            r.stop_command();r.g['phase']='FORCE_HOLD';r.hold(.6)
            r.record.end(dict(status='CALIBRATION_COMPLETED',not_FIT=True))
            result['blocks'].append(dict(source_role=b['source_role'],first_step=first,last_step_exclusive=r.tick,
                source_steps=b['source_steps'],source_duration_s=float(b['source_t'][-1]),
                measured_duration_s=(r.tick-first)*r.dt,jaw_width_m=b['jaw_width_m'],
                position_reference_tracking_RMS_m=float(np.sqrt(np.mean(np.square(tracking)))),
                position_reference_tracking_max_m=float(max(tracking))))
        if abs(r.actual()-initial)>1e-6:raise RuntimeError('UNLOADED_FIXTURE_OBJECT_MOTION')
        jaw_to(.084)
        plan=plan_exit(r.g)
        if not plan['valid']:raise RuntimeError('CALIBRATION_EXIT_UNAVAILABLE')
        r.move_path(plan['clearance_path'][1:],'CLEARANCE_RETREAT');r.move_path(plan['parking_path'][1:],'SAFE_PARK_MOTION')
        r.hold(.3);park=verify_park(r.g,plan)
        result.update(status='MATCHED_UNLOADED_COMPLETED'if park['reached']else'CALIBRATION_PARK_NOT_VERIFIED',park=park)
    except SafetyStop as e:result.update(status='PROTECTED_STOP',reason=e.reason,evidence=e.evidence)
    except BaseException as e:
        import traceback
        result.update(status='CALIBRATION_UNAVAILABLE_OR_INFRASTRUCTURE_FAILURE',reason=repr(e),traceback=traceback.format_exc())
    finally:
        result['wall_s']=time.perf_counter()-began
        if r is not None:
            result.update(simulation_s=r.tick*r.dt,object_coordinate_m=r.actual(),pad_loads_N=r.filtered().tolist())
            atomic_json(out/'CALIBRATION_RESULT.json',result_packet(result))
            try:
                if r.record.segment and len(r.record.segments)>=r.record.segment:r.record.end(dict(status=result['status'],not_FIT=True))
            finally:r.close()
        else:atomic_json(out/'CALIBRATION_RESULT.json',result_packet(result))
    print(json.dumps(result_packet(result)));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--preflight-output',type=Path);a=p.parse_args();job=json.loads(a.job.read_text())
    if a.preflight_output:preflight(job,a.episode,a.preflight_output)
    else:run(job,a.episode)
