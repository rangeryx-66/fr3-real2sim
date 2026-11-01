"""Bounded native free-space check before any new controller contact trial."""
import time
import numpy as np
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def cancellation_check(r):
    """Small unloaded XY/opposing-arm input from the CURRENT robot Jacobian.

    Engineering test input only, through the same owner/drive and native
    supervision. No fake contact, grasp, object command or robot state reset.
    Linearized cancellation is a prediction; actual TCP motion is measured.
    """
    initial=r._measured_position();T0=r.tcp().copy()
    J=r.coupled.jacobian(initial);base=np.array([.0005,0.])
    arm=np.linalg.lstsq(J[:,2:],-J[:,:2]@base,rcond=None)[0]
    wanted=np.r_[base,arm]*min(1.,.002/max(np.max(abs(arm)),1e-30))
    if np.linalg.norm(J@wanted)>1e-7 or np.linalg.norm(wanted[:2])<5e-5:
        raise RuntimeError('FREE_CANCELLATION_INPUT_NOT_RESOLVED')
    r.g['phase']=r.phase='PREGRASP';r.base_active=r._v89_mobile=True
    bounds=r._bounds('PREGRASP');excursion=[];command_excursion=[]
    for _ in range(round(2./r.dt)):
        # Joint feedforward alone cannot compensate native drive stiction/lag.
        # Hold the CURRENT measured TCP with the same DLS feedback as normal
        # Cartesian execution. The arm supplies correction; XY feedforward and
        # arm reference enter one proposal, never separate drive submissions.
        correction=r.kernel.velocity(T0,r._measured_position(),bounds,mobile=False)
        proposal=r.command_owner.propose(wanted+correction,bounds,r._geometry())
        r._last_proposal=proposal
        if not proposal.accepted:
            r.stop_command()
            raise RuntimeError('FREE_CANCELLATION_REJECTED:'+proposal.reason)
        r.command_owner.commit(proposal)
        r._last_bounds=bounds;r._last_phase='PREGRASP';r._write_command()
        r.step();excursion.append(float(np.linalg.norm(r.tcp()[:3,3]-T0[:3,3])))
        command_excursion.append(float(np.linalg.norm(
            r.coupled.fk(r.command_owner.command.position)[:3,3]-T0[:3,3])))
    r.stop_command();change=r._measured_position()-initial
    return dict(base_net_m=change[:2].tolist(),arm_net_rad=change[2:].tolist(),
        command_input_velocity=wanted.tolist(),linearized_TCP_twist=(J@wanted).tolist(),
        actual_TCP_excursion_peak_m=max(excursion),command_zero=r.command_owner.command_zero,
        command_TCP_excursion_peak_m=max(command_excursion),
        measured_TCP_feedback=True,feedforward_is_not_actual_cancellation=True,
        measured_stationary=r.command_owner.measured_stationary,
        unloaded=True,object_commands=False,
        passed=bool(np.linalg.norm(change[:2])>5e-5 and np.linalg.norm(change[2:])>1e-5
            and max(excursion)<.001 and r.command_owner.measured_stationary))


def free_stop_restart(r):
    from interaction_runtime.prismatic_radial_session_v751 import PrismaticRadialOperationsV751
    from agentic_interaction_v81.operations import operations_class
    from agentic_interaction_v87.staged_exit import operations_class as staged
    from agentic_interaction_v81.terminal_safety import persist
    ops = staged(operations_class(PrismaticRadialOperationsV751))(r)
    r.world.play()
    began = time.perf_counter()
    initial = r.tcp().copy()
    qinitial = float(r.actual())
    results = []
    result = dict(status='NOT_PASS', contact_experiment=False, object_reset=False)
    try:
        # An engineering movement from the CURRENT measured pose, not a grasp
        # waypoint or recorded successful configuration. Native check each step.
        for direction in (1., -1.):
            r.reference_drive = None
            target = initial.copy()
            target[1, 3] += .005 * direction
            start = r.tick
            for _ in range(round(2. / r.dt)):
                r.cartesian_step(target, 'PREGRASP')
            r.stop_command()
            results.append(dict(t_sim=r.tick*r.dt, steps=r.tick-start,
                actual_TCP=r.tcp().tolist(), requested_TCP=target.tolist(),
                tracking_error_m=float(np.linalg.norm(r.tcp()[:3, 3]-target[:3, 3])),
                command_zero=r.command_owner.command_zero,
                measured_stationary=r.command_owner.measured_stationary))
        result['motion_checks'] = results
        # Conservative SUPERVISION-input fault, never a physical state reset:
        # simulate loss of readiness after a qualified zero-command hold.
        # The runtime must reacquire REAL native dwell before committing again.
        owner=r.command_owner;before=owner.command.generation;began_rearm=r.tick
        owner.settling();owner.measured_stationary=False
        r.station_motion.stable_s=r.station_motion.physical_stable_s=0.
        r.station_motion.latest=dict(r.station_motion.latest,ready=False,stable_s=0.)
        r.cartesian_step(r.tcp().copy(),'PREGRASP')
        restarted=owner.state.value=='RUNNING' and owner.command.generation>before
        r.stop_command()
        result['readiness_loss_injection']=dict(
            kind='conservative readiness invalidation; no physical disturbance',
            real_physics_steps=r.tick-began_rearm,explicit_restart_accepted=restarted,
            measured_stationary=owner.measured_stationary,command_zero=owner.command_zero,
            physics_state_changed_by_fixture=False,latch_cleared=False,
            passed=bool(restarted and owner.measured_stationary and owner.command_zero))
        result['coordinated_cancellation_check'] = cancellation_check(r)
    except Exception as error:
        result['error'] = repr(error)
    finally:
        if r.supervisor.latched is None:
            try: result['safe_exit'] = ops.release_park()
            except Exception as error: result['exit_error'] = repr(error)
        else: result['hard_latch'] = r.supervisor.latched
        result.update(wall_s=time.perf_counter()-began, sim_s=r.tick*r.dt,
            object_coordinate_initial=qinitial, object_coordinate_final=float(r.actual()),
            profile=r.profile.report(r.tick*r.dt))
        passed = (len(results)==2 and all(x['measured_stationary'] and x['tracking_error_m']<.001 for x in results)
                  and result.get('readiness_loss_injection',{}).get('passed') is True
                  and result.get('coordinated_cancellation_check',{}).get('passed') is True
                  and result.get('safe_exit',{}).get('safe') is True and r.supervisor.latched is None)
        result['status'] = 'PASS' if passed else 'NOT_PASS'
        atomic_json(r.output/'V89_FREE_SPACE_CHECK.json', result_packet(result))
        persist(r, result, pause=True)
    return result
