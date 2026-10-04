"""One setup initialization, no contact cache or execution-time state injection."""
import json
from pathlib import Path

def install():
    import interactive_twin.plant as plant
    old=plant.adapt_loader_source
    def adapt(source,job):
        source=old(source,job)
        path=job.get('conditional_snapshot')
        if not path:return source
        s=json.loads(Path(path).read_text())
        if s['provenance']['joint_truth_used'] or s['provenance']['moving_trajectory_used']:
            raise ValueError('CONDITIONAL_SNAPSHOT_IS_NOT_OBSERVABLE')
        start='q = np.zeros(len(names)); q[arm] = HOME; q[fingers] = [.05,-.05]'
        if source.count(start)!=1:raise RuntimeError('CONDITIONAL_Q_INIT_MARKER')
        source=source.replace(start,'q = np.array('+repr(s['q_rad'])+',dtype=float)')
        applied='robot.set_joint_positions(q); robot.apply_action(ArticulationAction(joint_positions=q))'
        replacement='''robot.set_joint_positions(q)
robot.set_joint_velocities(np.array(SNAPSHOT_VELOCITY,dtype=float))
robot.apply_action(ArticulationAction(joint_positions=np.array(SNAPSHOT_COMMAND,dtype=float),joint_indices=arm))
kp[fingers]=0.;kd[fingers]=40.
controller.set_gains(kps=kp,kds=kd,save_to_usd=False)
robot.apply_action(ArticulationAction(joint_efforts=np.array([-SNAPSHOT_FORCE,SNAPSHOT_FORCE]),joint_indices=fingers))'''.replace('SNAPSHOT_VELOCITY',repr(s['qdot_rad_s'])).replace('SNAPSHOT_COMMAND',repr(s['arm_position_command'])).replace('SNAPSHOT_FORCE',repr(s['finger_effort_n']))
        if source.count(applied)!=1:raise RuntimeError('CONDITIONAL_APPLY_INIT_MARKER')
        source=source.replace(applied,replacement)
        # The original one-second unobserved warmup is excluded from A only.
        # All contact rebuilding is now logged by the normal guarded step loop.
        warm='for i in range(240):'
        if source.count(warm)!=1:raise RuntimeError('CONDITIONAL_WARMUP_MARKER')
        source=source.replace(warm,'for i in range(0):')
        return source
    plant.adapt_loader_source=adapt
