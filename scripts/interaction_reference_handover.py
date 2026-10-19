"""Versioned reference handover only; original guards and physical settings stay live.

Pink integrates from a virtual measured-state seed, then adds an equilibrium
joint bias. At any handover its reference must equal FK of that SAME seed.
A Cartesian IK target solved only to tolerance is not an exact seed FK.
"""
import numpy as np
import pinocchio as pin
from articulated_demo.kinematics import transform


def seed_pose(solver, base, seed):
    q=np.asarray(seed)[solver.indices]
    data=solver.robot.createData()
    pin.framesForwardKinematics(solver.robot,data,q)
    frame=data.oMf[solver.robot.getFrameId('tcp_link')]
    local=np.eye(4);local[:3,:3]=frame.rotation;local[:3,3]=frame.translation
    return transform(base[:3],[0,0,np.deg2rad(base[3])])@local


def handover(g, reason):
    """Keep joint position command and grasp reference; rebase virtual drive.

    No object-state edit, no slip-reference reset, no safety acceptance check.
    Existing temporal generator ramps from zero twist to its existing rate.
    """
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']].copy()
    command=np.asarray(g['qtarget'][g['arm']]).copy()
    before=np.asarray(g['ref']).copy() if g.get('ref') is not None else None
    g['seed']=measured;g['bias']=command-measured
    g['ref']=seed_pose(g['solver'],g['base'],measured)
    g['planned_reference']=g['ref'].copy();g['angle_reference']=float(g['actual']())
    g['twist']=np.zeros(6);g['qvelocity']=np.zeros(len(g['arm']))
    event=dict(event='REFERENCE_HANDOVER',reason=reason,t=g['tick']*g['dt'],
        seed=measured.tolist(),equilibrium_bias=g['bias'].tolist(),
        previous_reference=None if before is None else before.tolist(),reference=g['ref'].tolist(),
        unchanged_joint_command=command.tolist(),grasp_slip_reference_preserved=True)
    g['telemetry_events'].append(event)
    return event


def contact_policy_source():
    """Single pinned change in the existing policy, no duplicate controller."""
    from pathlib import Path
    source=(Path(__file__).parent/'shared_door_execution.py').read_text()
    anchor="        g['opening']=feasible;return feasible # one-pad unloading never blocks continuation"
    if source.count(anchor)!=1:raise ValueError('Frozen centering handover boundary changed')
    source=source.replace(anchor,"        handover(g,'LOCAL_CENTERING_TO_PROBE')\n"+anchor)
    return "from interaction_reference_handover import handover\n"+source


def open_with_contact_feedback(g):
    ns={};exec(compile(contact_policy_source(),__file__+'::versioned_contact_policy','exec'),ns)
    return ns['open_with_contact_feedback'](g)
