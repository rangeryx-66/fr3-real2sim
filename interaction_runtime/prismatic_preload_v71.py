"""Prismatic-only preload study; native actuator authority and guards unchanged.

The preload value is the original closure's requested normal-load setpoint,
NOT a measured force nor a new effort cap. The existing master-position mapping
and nonlinear aperture Jacobian remain owned by Operations.acquire/NativeDrive.
"""
import json,hashlib
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from .prismatic_feedback_v7 import PrismaticFeedbackRuntime,PrismaticFeedbackOperations


def preload_policy(original,value):
    if isinstance(value,(bool,np.bool_)):raise ValueError('Numeric preload required, not a boolean')
    value=float(value)
    if not np.isfinite(value) or value<=0 or value>original['finger_effort_limit_n']:
        raise ValueError('Preload must be finite, positive and within existing closure request authority')
    result=dict(original);result['preload_n']=value
    return result


class PrismaticPreloadRuntime(PrismaticFeedbackRuntime):
    def __init__(self,job):
        if job.get('prismatic_integrated_ik_feedback') or job.get('prismatic_contact_retention'):
            raise ValueError('Isolate preload from unvalidated integral/retention prototypes')
        super().__init__(job)
        self.optional_recording_disabled=True # checkpoint RGB-D remains active
        original=self.closure_policy
        path=self.root/'configs/prismatic_preload_v71.json'
        defaults=json.loads(path.read_text())
        self.closure_policy=preload_policy(original,job.get('native_g1_preload_request_n',defaults['preload_request_n']))
        self.g['policy']=self.closure_policy
        atomic_json(self.output/'PRELOAD_CONFIGURATION.json',dict(
            baseline_policy=original,active_policy=self.closure_policy,
            semantics='original closure normal-load request, translated to finite master position bias; not commanded motor force',
            actuator_caps_gains_or_physics_changed=False,hard_safety_policy_sha256=self.policy.sha256,
            prismatic_configuration_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            motor_drive_output='unavailable; explicit zero effort and projected reaction are not motor output',
            reference=job.get('preload_fixture'),engineering=bool(job.get('preload_fixture'))))
        self.preload_record_fields=self.record.coordinate_fields
        self.record.coordinate_fields=self.preload_fields
        if job.get('preload_fixture'):
            path=self.output/'SOURCE_PROVENANCE.json'
            provenance=json.loads(path.read_text())
            provenance.update(fresh_grasp_generation=False,saved_grasp_input=True,
                saved_successful_grasp_input=False,controlled_observed_fixture=True,
                fresh_observation='pregrasp checkpoint only; no new geometry/pose selection')
            atomic_json(path,provenance)

    def preload_fields(self,row):
        values=self.preload_record_fields(row)
        values.update(preload_request_n=np.float32(self.closure_policy['preload_n']),
            master_target_rad=np.float32(self.controller.geometry.angle(self.jaw_width)),
            master_effort_cap_nm=np.float32(self.controller.caps[self.controller.master]))
        return values


class PreloadOperations(PrismaticFeedbackOperations):
    def attempt(self,command):
        # The Agent may choose a different legal preload only before a new
        # acquisition, at the existing verified unloaded decision boundary.
        # No loaded force-history clearing or actuator-authority modification.
        if 'native_g1_preload_request_n' in command:
            if (self.r.supervisor.latched is not None or self.r.grasp_reference is not None
                    or max(self.r.filtered())>self.r.policy.data['force']['unloaded_pad_n']
                    or not self.r.supervisor.force.settled()):
                return dict(executed=False,recoverable=True,reason='PRELOAD_CHANGE_REQUIRES_VERIFIED_UNLOADED_STATE')
            self.r.closure_policy=preload_policy(self.r.closure_policy,command['native_g1_preload_request_n'])
            self.g['policy']=self.r.closure_policy
            self.r.event('PRISMATIC_PRELOAD_SELECTED',request_n=self.r.closure_policy['preload_n'],
                         reason=command.get('reason','Agent-selected legal preload before acquisition'),
                         actuator_authority_changed=False)
        return super().attempt(command)


class PreloadFixtureOperations(PreloadOperations):
    """Matched engineering only: one frozen observed pose, no online GT fallback."""
    def prepare_fixture(self):
        from native_g1_safe_park import route_to_joint
        fixture=Path(self.r.job['preload_fixture'])
        data=json.loads(fixture.read_text());T=np.asarray(data['T'])
        q=np.asarray(self.r.robot.get_joint_positions())[self.r.arm]
        pre=T.copy();pre[:3,3]-=.04*T[:3,2]
        seed=self.r.solver.solve(pre,self.r.base,np.asarray(data['seed_q']))
        if seed is None:raise RuntimeError('FIXTURE_PREGRASP_IK_UNAVAILABLE')
        route,reason=route_to_joint(self.g,q,seed)
        if route is None:raise RuntimeError('FIXTURE_PREGRASP_ROUTE:'+str(reason))
        plan=dict(data['candidate']);plan.update(T=T.tolist(),T_grasp=T.tolist(),preplan=route,approach=[seed.tolist()])
        atomic_json(self.r.output/'visual_plan.json',plan)
        self.g.update(chosen=plan,whole=plan,T=T.copy(),normal=np.asarray(data['normal_world']),
            axis=np.asarray(data['bar_axis_world']),wrist_routes=[],
            anchor_world=np.asarray(data['anchor_world']),
            anchor_link=(np.linalg.inv(self.r.moving())@np.r_[data['anchor_world'],1])[:3])
        atomic_json(self.r.output/'MATCHED_FIXTURE_ACTUAL_START.json',dict(
            measured_q=q.tolist(),object_q=self.r.actual(),base=self.r.base.tolist(),
            T=T.tolist(),fixture_sha256=data['fixture_content_sha256'],
            observation_source=data['observation_source'],fresh_grasp_generation=False,
            saved_successful_or_GT_pose=False,complete_approach_still_checked=True))
        self.current_plan_ready=True

    def acquire(self):
        from unittest.mock import patch
        import native_g1_agent_execution as execution
        import native_g1_wrist_input as wrist
        self.prepare_fixture()
        def assign(state,command):
            # Initial route was freshly checked. Original acquire executes it
            # through actuators and checks its timed approach as usual.
            return dict(engineering_fixed_observed_fixture=True)
        def checkpoint(state):
            folder=wrist.save_rgbd(state,'matched_pregrasp')
            atomic_json(self.r.output/'MATCHED_PREGRASP_CHECKPOINT.json',dict(
                folder=str(folder),T=self.g['T'].tolist(),
                geometric_refinement='disabled only in matched fixture to keep the declared grasp pose identical',
                fresh_RGBD=True,GT_used=False))
        with patch.object(execution,'execute',assign),patch.object(wrist,'refine',checkpoint):
            return super().acquire()


def serve_preload_fixture(r):
    """One preapproved finite comparison, then unload/park; no silent retries."""
    ops=PreloadFixtureOperations(r);r.optional_recording_disabled=True
    try:
        r.g['phase']='SETTLE';r.hold(.5)
        segments=r.job['engineering_segments']
        for a in segments:
            from .segments import validate_action
            validate_action(a);ops.articulation().validate_target(a)
        ops.current_plan_ready=True
        # attempt reads the declared family before acquire creates current paths.
        data=json.loads(Path(r.job['preload_fixture']).read_text())
        atomic_json(r.output/'visual_plan.json',data['candidate'])
        result=ops.attempt(dict(segments=segments))
        atomic_json(r.output/'PRELOAD_CONDITION_RESULT.json',result)
        return dict(status='MATCHED_PRELOAD_ENGINEERING_COMPLETE',condition_n=r.closure_policy['preload_n'],result=result,
            not_a_fresh_perception_result=True,benchmark=False)
    finally:ops.close_workers()


def serve_preload(r):
    if r.job.get('preload_fixture'):return serve_preload_fixture(r)
    from .probing_v2 import serve
    from .segments import validate_action
    from .articulation_v7 import Articulation
    def check(a):
        validate_action(a)
        Articulation('prismatic',(1,0,0),(0,0,0),tuple(r.job['articulation_v7']['limits_native'])).validate_target(a)
        if not a['role'].startswith(('FIT','VALIDATION')):raise ValueError('Predeclare FIT/VALIDATION')
    return serve(r,operations_factory=PreloadOperations,action_validator=check)
