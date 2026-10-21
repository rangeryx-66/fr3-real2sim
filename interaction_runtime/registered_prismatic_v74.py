"""Authorized registered entry. Frozen engineering guard remains unchanged.

Only constructor wiring is mirrored from V7.3, with AST parity regression.
Step, closure, planning, recovery and estimator operations are inherited unchanged.
No job is relabelled engineering and no actuator/controller method is replaced.
"""
import json
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from .prismatic_v73 import PrismaticRuntimeV73,serve_v73
from .prismatic_preload_v71 import PrismaticPreloadRuntime
from .prismatic_planning_v73 import representable_joint_bounds,closing_envelope,predicted_aperture
from prismatic_registration_v74 import verify_job


class RegisteredPrismaticRuntimeV74(PrismaticRuntimeV73):
    ALLOW_REGISTERED_PRISMATIC=True

    def __init__(self,job):
        verify_job(Path(__file__).resolve().parents[1],job)
        PrismaticPreloadRuntime.__init__(self,job);self.g['runtime_policy']=self.policy
        low,high=representable_joint_bounds(self.solver.original_limits,self.policy.data['joint_margin_rad'])
        self.solver.robot.lowerPositionLimit=low;self.solver.robot.upperPositionLimit=high
        atomic_json(self.output/'NATIVE_COMMAND_PRECISION.json',dict(dtype=str(self.qtarget.dtype),strict_joint_margin_rad=self.policy.data['joint_margin_rad'],IK_lower=low.tolist(),IK_upper=high.tolist(),reason='represent inclusive IK bounds strictly inside unchanged guard at actual float32 command precision; no safety relaxation'))
        previous=self.record.coordinate_fields
        def measured_motion(row):
            from .compact_v2 import pose7
            value=previous(row)
            value['moving_link_pose']=pose7(row['T_moving_link'])
            return value
        self.record.coordinate_fields=measured_motion
        from .sensor_lighting_v1 import install
        atomic_json(self.output/'LIGHTING_PROVENANCE.json',install(self.scene['stage'],self.root/'configs/sensor_lighting_v1.json'))
        original=self.Closure
        def checked_closure(T,policy):
            candidate=self.g.get('chosen',{})
            measured=np.asarray(self.robot.get_joint_positions())[self.arm]
            audit=closing_envelope(self.g,measured,predicted_aperture(candidate),policy)
            audit.update(measured_arm_q=measured.tolist(),measured_TCP=np.asarray(T).tolist(),refined_target_TCP=np.asarray(self.g['T']).tolist(),selected_family=[candidate.get(k) for k in ('section_fraction','insertion_m','roll_deg','pitch_deg')])
            atomic_json(self.output/('CLOSING_REVALIDATION_%02d.json'%len(self.g['contact_attempts'])),audit)
            if not audit['valid']:raise RuntimeError('PRECONTACT_CLOSING_ROUTE_UNAVAILABLE:'+audit['reason'])
            return original(T,policy)
        self.Closure=checked_closure

