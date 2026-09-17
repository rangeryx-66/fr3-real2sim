"""Safe excitation and data capture layered on the frozen MoveIt backend."""
from __future__ import annotations
import copy
from pathlib import Path
import numpy as np
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .safety import require_free_space_stable


AMPLITUDE_RAD = np.array([.025, .030, .020, .035, .025, .040, .045])
FREQUENCY_HZ = np.array([.20, .27, .33, .40, .47, .53, .60])


def excitation(center, duration_s=15., sample_hz=30.):
    """Deterministic, smooth, low-amplitude FR3 payload excitation."""
    center=np.asarray(center,float);t=np.linspace(0,duration_s,int(duration_s*sample_hz)+1)
    envelope=np.sin(np.pi*t/duration_s)**2
    q=center[None,:]+envelope[:,None]*AMPLITUDE_RAD[None,:]*np.sin(2*np.pi*t[:,None]*FREQUENCY_HZ[None,:])
    return t,q


class PayloadIDSkill:
    def __init__(self,backend,plant):self.backend=backend;self.plant=plant

    def _checked_trajectory(self,center):
        times,positions=excitation(center);names=[f"fr3_joint{i}" for i in range(1,8)]
        measured=self.backend.measured();finger_names=[n for n in measured.joint_state.name if 'finger' in n]
        finger_values=[measured.joint_state.position[measured.joint_state.name.index(n)] for n in finger_names]
        # Check every 5th command plus the terminal state through MoveIt collision validity.
        for q in positions[::5]:
            state=RobotState();state.joint_state.name=names+finger_names;state.joint_state.position=q.tolist()+finger_values
            self.backend.validate(state)
        trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=names
        for t,q in zip(times,positions):
            point=JointTrajectoryPoint();point.positions=q.tolist();point.time_from_start=Duration(sec=int(t),nanosec=int((t%1)*1e9));trajectory.joint_trajectory.points.append(point)
        return trajectory

    def run(self,path:Path,grasp_result:dict|None,mode='payload'):
        if mode not in {'baseline','payload'}:raise ValueError(mode)
        if mode=='payload':require_free_space_stable(grasp_result or {})
        state=self.plant.state();center=np.asarray(state['q'])[[state['names'].index(f'fr3_joint{i}') for i in range(1,8)]]
        trajectory=self._checked_trajectory(center)
        started=self.plant.command({'op':'payload_record_start','mode':mode})
        if not started.get('ok'):raise RuntimeError(str(started))
        try:
            self.backend.phase('PAYLOAD_ID_'+mode.upper())
            item={'stage':self.backend.stage,'joint_names':list(trajectory.joint_trajectory.joint_names),
                  'points':[{'t':p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                             'q':list(p.positions)} for p in trajectory.joint_trajectory.points]}
            # 240 Hz Isaac contact/Jacobian acquisition can run substantially
            # slower than wall time.  Keep the frozen execution bridge semantics
            # while allowing the complete 15 s excitation to finish.
            if hasattr(self.backend,'executions'):self.backend.executions.append(item)
            answer=self.plant.command({'op':'trajectory','names':item['joint_names'],'points':item['points']},timeout=1200)
            if not answer.get('ok'):raise RuntimeError('PAYLOAD_ID_SAFETY_ABORT: '+str(answer))
        finally:
            stopped=self.plant.command({'op':'payload_record_stop','path':str(Path(path).resolve())})
        if not stopped.get('ok'):raise RuntimeError('PAYLOAD_ID_SAFETY_ABORT: '+str(stopped))
        return stopped
