"""Bounded native free-space check before any new controller contact trial."""
import time
import numpy as np
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


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
                  and result.get('safe_exit',{}).get('safe') is True and r.supervisor.latched is None)
        result['status'] = 'PASS' if passed else 'NOT_PASS'
        atomic_json(r.output/'V89_FREE_SPACE_CHECK.json', result_packet(result))
        persist(r, result, pause=True)
    return result
