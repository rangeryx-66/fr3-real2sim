"""Rest-to-rest timing on the existing joint-space segments; no new guards."""
import numpy as np
import pinocchio as pin


def segment_samples(ik, start, end, minimum_duration, dt, tcp_cruise_m_s):
    start = np.asarray(start); delta = np.asarray(end) - start
    data = ik.robot.createData(); frame = ik.robot.getFrameId('tcp_link')
    # This sets the trajectory clock, not a measured-speed acceptance gate.
    tangent = max(np.linalg.norm(pin.computeFrameJacobian(
        ik.robot, data, start + f * delta, frame,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] @ delta)
        for f in np.linspace(0., 1., 65))
    duration = max(float(minimum_duration), 1.875 * tangent / tcp_cruise_m_s,
                   float(np.max(1.875 * abs(delta) / ik.robot.velocityLimit)))
    steps = max(2, int(np.ceil(duration / dt))); duration = steps * dt
    for i in range(1, steps + 1):
        if i == steps:
            yield np.asarray(end).copy(), np.zeros_like(delta)
            continue
        u = i / steps
        position = 10*u**3 - 15*u**4 + 6*u**5
        rate = 30*u**2*(1-u)**2 / duration
        yield start + position * delta, rate * delta


def repair_initial_sequence(initial):
    """Only retain the equilibrium offset at the approach/closure handover."""
    entry = "closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'"
    update = "qvelocity=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q;step()"
    # Source-edit verification is offline, before a physical scene exists.
    if initial.count(entry) != 1 or initial.count(update) != 1:
        raise ValueError('Pinned initial sequence changed; no physical trial started')
    initial = initial.replace(entry, "closure_handover_bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm]\n  " + entry)
    return initial.replace(update, "q=q+closure_handover_bias\n   " + update)
