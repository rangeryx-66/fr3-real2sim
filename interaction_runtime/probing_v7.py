"""V7 SI adapter on the existing perception/acquisition/recovery/motion owners."""
import json
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from .articulation_v7 import Articulation
from .probing_v2 import ProbingRuntime,serve
from .probing_v3 import RuntimeV3,RecorderV3
from .probing_v5 import OperationsV5
from .segments import ExecutionBudgetExhausted,validate_action
from .policy import SafetyStop
from xarm_ft_v7_estimator import VERSION


class RecorderV7(RecorderV3):
    def begin(self,action,mode):
        self.segment=len(self.segments)+1
        self.segments.append(dict(id=self.segment,role=action['role'],mode=mode,target_q=action['target_q'],coordinate_unit=action['coordinate_unit'],profile=action.get('profile','constant'),first_step=self.count,completed=False,declared_action=action.copy()))
    def coordinate_fields(self,row):
        from .compact_v2 import pose7
        c=row.get('contact_diagnostic',{})
        return dict(joint_q=np.float64(row['joint_coordinate']),joint_dq=np.float64(row['joint_coordinate_velocity']),
            command_TCP_pose=pose7(row.get('command_TCP',np.full((4,4),np.nan))) if 'command_TCP' in row else np.full(7,np.nan),
            normal_force_on_fingers_world_N=np.asarray(c.get('native_normal_force_on_fingers_world_N',np.zeros((2,3))),np.float32),
            friction_force_on_fingers_world_N=np.asarray(c.get('native_friction_force_on_fingers_world_N',np.zeros((2,3))),np.float32),
            friction_api_valid=np.asarray(c.get('friction_api_valid',[False,False]),np.uint8),
            friction_anchor_counts=np.asarray(c.get('friction_anchor_counts',[0,0]),np.uint16),
            relative_tangent_speed_m_s=np.float32(c.get('relative_tangent_speed_m_s',np.nan)))


class RuntimeV7(RuntimeV3):
    def __init__(self,job):
        ProbingRuntime.__init__(self,job,record_factory=RecorderV7)
        from xarm_ft_v7_scene import install_native_law
        if self.engineering_before_physics is not None:raise RuntimeError('Passive plant owner conflict')
        self.engineering_before_physics=install_native_law(self.g)
        self.record.metadata['articulation']=self.job['articulation_v7']
        from xarm_ft_known_joint_prior import native_joint_prior
        p=native_joint_prior(job['asset_root'],self.moving())
        if p['joint_type']!=job['articulation_v7']['joint_type']:raise ValueError('Native/registered articulation types disagree')
        a=Articulation(p['joint_type'],tuple(p['axis_world']),tuple(p['origin_world_m']),tuple(job['articulation_v7']['limits_native']))
        atomic_json(self.output/'ARTICULATION_INTERFACE.json',dict(prior=p,SI=a.packet(),selected_native_DOF=self.scene.get('selected_asset_dof',0),object_joint_commands=False))
        from .contact_evidence_v7 import ContactEvidence
        self.contact_evidence=ContactEvidence(self.output/'contact_evidence',p['axis_world'],self.dt,record_points=bool(job.get('engineering_contact_diagnostics')))

    def enrich_measurement(self,row,pre_contact_poses):
        from interaction_reference_handover import seed_pose
        try:row['command_TCP']=seed_pose(self.solver,self.base,self.qtarget[self.arm])
        except Exception as error:row['command_TCP_unavailable']=repr(error)
        evidence=getattr(self,'contact_evidence',None)
        if evidence is not None:
            if row['contacts'] or self.policy.phase(row['phase']) in ('ESTABLISH','LOADED','RELEASE'):
                try:row['contact_diagnostic']=evidence.read(self,row,pre_contact_poses)
                except Exception as error:row['contact_diagnostic']=dict(traction_available=False,slip_velocity_available=False,unavailable=repr(error))
            else:evidence.previous={k:v.copy() for k,v in pre_contact_poses.items()}

    def close(self):
        evidence=getattr(self,'contact_evidence',None)
        if evidence is not None:evidence.close()
        return super().close()

    def motion_limits(self,phase):
        if getattr(self,'prismatic_preloading',False) and self.policy.phase(phase)=='LOADED':
            m=dict(self.policy.motion(phase));m['speed_m_s']=min(m['speed_m_s'],self.active_prismatic_startup.preload_speed)
            m['acceleration_m_s2']=min(m['acceleration_m_s2'],self.probe_profile.get('reference_acceleration_m_s2',m['acceleration_m_s2']))
            return m
        return super().motion_limits(phase)


class OperationsV7(OperationsV5):
    worker_script='scripts/update_probing_v7_model.py'
    worker_version=VERSION

    def __init__(self,r):
        super().__init__(r);self.current_plan_ready=False;self.g['expose_recovery_candidates_v7']=True

    def reobserve(self):
        self.current_plan_ready=False
        self.g.pop('recovery_candidate_catalog',None)
        return super().reobserve()

    def request_plan(self,command):
        self.current_plan_ready=False
        request=command.get('candidate_request')
        if request is not None:
            from .recovery_candidates_v7 import resolve
            attempted={tuple(r['family']) for r in self.g['contact_attempts'] if r['physical_grasp_started'] and r.get('failed_contact_candidate',not r['short_success'])}
            try:self.g['preferred_existing_candidate_family']=resolve(request,self.g.get('recovery_candidate_catalog'),self.g['agent_observations'],attempted)
            except ValueError as error:return dict(plan=False,reason=str(error),recoverable=True,robot_action_executed=False)
        try:
            result=self.build_geometry()
            if request is not None:
                selected=self.g.pop('preferred_existing_candidate_family')
                if not result.get('plan'):
                    rejected=result;result=self.build_geometry()
                    result['Agent_requested_family_infeasible']=dict(family=list(selected),reason=rejected.get('reason'),automatic_eligible_fallback=True)
                else:result['Agent_selected_existing_family']=list(selected)
            return result
        finally:self.g.pop('preferred_existing_candidate_family',None)

    def build_geometry(self):
        # Planning failure remains an unloaded Agent result in the inherited
        # interface. Never execute a stale visual_plan.json from a prior grasp.
        self.current_plan_ready=False
        result=super().build_geometry()
        self.current_plan_ready=bool(result.get('plan'))
        if getattr(self,'g',{}).get('recovery_candidate_catalog'):
            c=self.g['recovery_candidate_catalog'];result['candidate_catalog']=dict(path=str(self.r.output/'AGENT_CANDIDATE_CATALOG.json'),catalog_sha256=c['catalog_sha256'],candidates=c['candidates'])
        return result

    def validate_execution(self):
        if not self.current_plan_ready:
            return dict(valid=False,reason='NO_CURRENT_AUTOMATIC_PLAN',recoverable=True)
        return super().validate_execution()

    def attempt(self,command):
        if not self.current_plan_ready:
            return dict(executed=False,reason='NO_CURRENT_AUTOMATIC_PLAN; reobserve/build/evaluate alternatives',recoverable=True)
        self.inputs()  # Contact feedback remains renderable even if acquisition fails before probing.
        result=super().attempt(command)
        self.current_plan_ready=False
        result['failed_contact_candidate']=bool(result.get('error')) or not result.get('acquisition',{}).get('acquired') or any(s.get('status') in ('TRANSMISSION_FAILURE','SAFETY_STOP') for s in result.get('segments',[]))
        result.pop('short_success',None)
        # Legacy memory readers retain a boolean compatibility field; its V7
        # meaning is explicit, never a drawer-displacement/degree comparison.
        result['short_success']=bool(not result['failed_contact_candidate'] and any(s.get('ok') for s in result.get('segments',[])))
        result['short_success_semantics']='V7 transmitting completed native-coordinate segment; not the legacy >=2deg metric'
        return result

    def plan_push(self,command):
        if self.articulation().joint_type=='prismatic':
            return dict(valid=False,reason='PANEL_PUSH_REVOLUTE_ONLY; use current-state handle reversal or unloaded fresh regrasp',recoverable=True)
        return super().plan_push(command)

    def rotate_segment(self,target_deg,*args,**kwargs):
        if 'articulation' not in kwargs:
            # Existing panel primitive explicitly declares degrees and is
            # revolute-only. Convert at its boundary, not in any rail path.
            a=self.articulation()
            if a.joint_type!='revolute':raise ValueError('Angular panel primitive cannot execute rail motion')
            kwargs.update(articulation=a,target_q=float(np.deg2rad(target_deg)))
        return super().rotate_segment(target_deg,*args,**kwargs)

    def push(self,command=None):
        command=command or {}
        if len(command.get('segments',[]))!=1:return dict(executed=False,reason='Declare one complete native-coordinate panel segment')
        self.articulation().validate_target(command['segments'][0])
        return super().push(command)

    def articulation(self):
        from native_g1_ft_identification import kinematic_prior
        p=kinematic_prior(self.g)
        return Articulation(p['joint_type'],tuple(p['axis_world']),tuple(p['origin_world_m']),tuple(self.r.job['articulation_v7']['limits_native']),self.r.model.manifest['joint_name'])

    def inputs(self):
        from native_g1_ft_identification import kinematic_prior
        atomic_json(self.r.output/'identification_input.json',dict(prior=kinematic_prior(self.g),bias=self.r.job['ft_calibration']['bias_sensor'],estimation=self.r.job['estimation_v7'],hidden_object_dynamics=False))

    def action_target(self,action):return self.articulation().validate_target(action)
    def measured_coordinate_fields(self):return dict(joint_coordinate=self.r.actual(),units=self.articulation().units)

    def prismatic_startup(self,action,lookahead):
        from .prismatic_startup import PrismaticStartup
        self._startup_axis=np.asarray(self.articulation().axis_world)
        initial=self.startup_sample(self.r.rows[-1],0.)
        quiet=[self.startup_sample(r,r['t']-self.r.rows[-1]['t']) for r in self.r.rows if r['t']>=self.r.rows[-1]['t']-1. and r['phase']=='FORCE_HOLD']
        startup=PrismaticStartup(initial,quiet,action,self.r.policy.motion('LOADED'),lookahead,self.r.dt)
        self.r.prismatic_preloading=True;self.r.active_prismatic_startup=startup
        return startup

    def startup_sample(self,row,elapsed):
        if 'command_TCP' not in row:raise RuntimeError('Command TCP FK unavailable for rail startup; no silent tracking fallback')
        axis=self._startup_axis
        return dict(t=float(elapsed),q=float(row['joint_coordinate']),dq=float(row['joint_coordinate_velocity']),
            command_rail=float(row['command_TCP'][:3,3]@axis),tcp_rail=float(row['T_tcp'][:3,3]@axis),
            rail_effort=float(np.asarray(row['wrist_ft']['gravity_compensated_wrench_world'])[:3]@axis),contact=row.get('contact_diagnostic',{}))

    def check_successor(self,action):
        a=self.articulation();target=a.validate_target(action);start=self.r.actual();T=self.r.tcp().copy();M=self.r.moving().copy()
        measured=np.asarray(self.r.robot.get_joint_positions());q=measured[self.r.arm];fingers=measured[self.r.fingers]
        for displacement in np.linspace(0,target-start,21):
            D=a.delta(displacement);q=self.r.solver.solve(D@T,self.r.base,q)
            if q is None:return dict(valid=False,reason='SUCCESSOR_CURRENT_STATE_IK',actual_start_q=start)
            if self.r.model.margin(q)<=self.r.policy.data['joint_margin_rad']:return dict(valid=False,reason='JOINT_MARGIN')
            ok,why=self.r.collision.check(self.r.model.poses(q,self.r.base,finger_q=fingers),D@M,True)
            if not ok:return dict(valid=False,reason=why,actual_start_q=start)
        return dict(valid=True,actual_start_q=start,target_q=target,units=a.units,measured_state_route=True)

    def interact(self,target,max_sim_s=None,action=None):
        a=self.actions.pop(0) if self.actions else action
        if a is None:raise ValueError('Predeclared native-coordinate action required')
        motion=self.articulation();target=motion.validate_target(a)
        self.inputs();self.r.record.begin(a,'HANDLE');self.r.probe_profile=a;self.r.profile_started=self.r.tick
        result={}
        try:
            from native_g1_ft_identification import kinematic_prior
            result=self.rotate_segment(None,max_sim_s,a,prior=kinematic_prior(self.g),articulation=motion,target_q=target)
            for old in ['requested_target_rad','selected_target_rad']:
                if old in result.get('planning',{}):result['planning'][old.replace('_rad','_native')]=result['planning'].pop(old)
            result['planning']['coordinate_unit']=motion.units['coordinate']
            for key in ['transmission','transmission_failure']:
                if result.get(key):
                    for old,new in [('progress_rad','progress_native'),('requested_travel_rad','requested_travel_native'),('lead_rad','lead_native')]:
                        if old in result[key]:result[key][new]=result[key].pop(old)
            return result
        except ExecutionBudgetExhausted as error:
            self.r.stop_command();result=dict(ok=False,status='BUDGET_EXHAUSTED',reason=str(error),safely_stopped=True,physical_transmission_valid=False);return result
        except SafetyStop as error:
            result=dict(ok=False,status='SAFETY_STOP',reason=error.reason,evidence=error.evidence,safely_stopped=False,physical_transmission_valid=False);raise
        finally:
            startup=getattr(self.r,'active_prismatic_startup',None)
            if startup is not None:result['prismatic_startup']=startup.packet()
            self.r.prismatic_preloading=False;self.r.active_prismatic_startup=None
            self.r.probe_profile=None;self.r.record.end(result)

    def feedback(self):
        from render_contact_feedback_v7 import render
        if self.r.contact_evidence is not None:self.r.contact_evidence.flush()
        try:contact=render(self.r.output)
        except Exception as error:contact=dict(available=False,reason=repr(error))
        p=self.r.output/'model/estimate.json'
        return dict(attempts=self.g['contact_attempts'],contact_feedback=contact,model=json.loads(p.read_text()) if p.exists() else dict(status='worker pending'),plot=str(self.r.output/'model/feedback.png'),events=self.r.events[-30:],**self.measured_coordinate_fields())


def serve_v7(r):
    def check(a):
        validate_action(a)
        Articulation(r.job['articulation_v7']['joint_type'],(1,0,0),(0,0,0),tuple(r.job['articulation_v7']['limits_native'])).validate_target(a)
        if not a['role'].startswith(('FIT','VALIDATION')):raise ValueError('Predeclare FIT/VALIDATION role')
    return serve(r,operations_factory=OperationsV7,action_validator=check)
