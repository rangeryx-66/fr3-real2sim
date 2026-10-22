"""Short independent TOOL-load calibration; never object assistance or FIT.

Unmodified native actuators, sensor and every protection. A known externally
applied tool load is confined to this engineering fixture. No manipulation,
cached grasp, direct object actuation or changed plant parameter is permitted.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def run(job,mode):
    from interaction_runtime.prismatic_v73 import PrismaticRuntimeV73,audit_frozen
    from interaction_runtime.policy import SafetyStop
    audit_frozen(ROOT)
    if mode not in ('normal','sustained_stop','sustained_ramp_stop'):raise ValueError('Registered fixture modes only')
    output=Path(job['output'])
    if output.exists():raise ValueError('Preserve every attempt; fresh output required')
    output.mkdir(parents=True);atomic_json(output/'REGISTRATION.json',dict(mode=mode,
        engineering_TOOL_load_only=True,object_contact=False,not_estimator_training=True,
        robot_physics_or_limits_changed=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    r=None;began=time.perf_counter();result=dict(mode=mode,physics_fixture=True,object_contact=False)
    force=np.zeros(3);torque=np.zeros(3);label='unloaded';blocks=[]
    try:
        r=PrismaticRuntimeV73(job);plant=r.engineering_before_physics
        view=r.actual_views['xarm_gripper_base_link']
        def apply():
            if plant is not None:plant()
            position=r.pose('xarm_gripper_base_link')[:3,3]
            view.apply_forces_and_torques_at_pos(forces=force[None],torques=torque[None],positions=position[None],is_global=True)
        r.engineering_before_physics=apply;r.engineering_load=force
        r.g['phase']='FORCE_HOLD'
        # Native 240 Hz numeric capture; role can never enter FIT/VALIDATION.
        r.record.begin(dict(role='CALIBRATION_NOT_FIT',target_q=r.actual(),coordinate_unit='m',profile='known_tool_load'), 'UNLOADED')
        def hold(seconds,name,f=None,m=None):
            nonlocal label
            label=name;force[:]=0 if f is None else f;torque[:]=0 if m is None else m
            start=r.tick
            try:r.hold(seconds)
            finally:blocks.append(dict(label=label,first_step=start,last_step_exclusive=r.tick,
                applied_world_force_N=force.tolist(),applied_world_torque_at_gripper_base_Nm=torque.tolist()))
        hold(.8,'unloaded_orientation_A')
        if mode=='normal':
            home=r.qtarget[r.arm].copy();target=home.copy();target[5]+=.03
            # Original timed path and native collision guards, normal actuators.
            start=r.tick;r.move_path([target],'FORCE_HOLD');blocks.append(dict(label='unloaded_smooth_motion_A_B',first_step=start,last_step_exclusive=r.tick))
            hold(.8,'unloaded_orientation_B')
            start=r.tick;r.move_path([home],'FORCE_HOLD');blocks.append(dict(label='unloaded_smooth_motion_B_A',first_step=start,last_step_exclusive=r.tick))
            hold(.5,'unloaded_after_motion')
            for i in range(3):
                f=np.zeros(3);f[i]=1.;hold(.6,'known_force_axis_%d'%i,f=f);hold(.35,'zero_after_force_%d'%i)
            for i in range(3):
                m=np.zeros(3);m[i]=.05;hold(.6,'known_torque_axis_%d'%i,m=m);hold(.35,'zero_after_torque_%d'%i)
            result['status']='NORMAL_CALIBRATION_COMPLETED'
        else:
            if mode=='sustained_ramp_stop':
                # The abrupt 9 N fixture correctly hit the unchanged TCP speed
                # guard before a full overload window. Register a bounded ramp
                # to isolate the force channel without disabling that guard.
                for level in np.arange(.05,9.001,.05):hold(.05,'known_TOOL_load_ramp',f=[0,0,float(level)])
            hold(.5,'known_TOOL_sustained_overload',f=[0,0,9.])
            result['status']='FAIL: sustained overload was not stopped'
    except SafetyStop as e:
        result.update(status='PROTECTED_STOP',reason=e.reason,evidence=e.evidence)
    except BaseException as e:
        import traceback
        result.update(status='INFRASTRUCTURE_OR_FIXTURE_FAILURE',error=repr(e),traceback=traceback.format_exc())
    finally:
        if r is not None:
            force[:]=0;torque[:]=0
            result.update(blocks=blocks,physics_steps=r.tick,simulation_s=r.tick*r.dt,
                current_coordinate_m=r.actual(),pad_loads_N=r.filtered().tolist(),hard_latch=r.supervisor.latched)
            # Preserve the FIRST error even if a fixture fails during begin().
            # This is recorder cleanup, never a physical safety-latch reset.
            atomic_json(output/'CALIBRATION_RESULT.json',result_packet(dict(result,wall_s=time.perf_counter()-began)))
            try:
                if r.record.segment and len(r.record.segments)>=r.record.segment:
                    r.record.end(dict(status=result['status'],calibration_only=True))
                r.record.flush();r.record.queue.join()
            finally:r.close()
        else:atomic_json(output/'CALIBRATION_RESULT.json',result_packet(dict(result,wall_s=time.perf_counter()-began)))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--mode',choices=['normal','sustained_stop','sustained_ramp_stop'],required=True);a=p.parse_args()
    print(json.dumps(result_packet(run(json.loads(a.job.read_text()),a.mode))))
