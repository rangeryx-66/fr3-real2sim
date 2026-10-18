"""Thin research integration on the validated phase-aware runtime."""
import concurrent.futures,json,math,os,subprocess,sys,time
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json,receive,complete
from .runtime import Runtime
from .operations import Operations
from .compact_v2 import CompactRecorder,storage,prune_optional
from .serialization import result_packet

class ProbingRuntime(Runtime):
    def __init__(self,job,record_factory=CompactRecorder):
        self.probe_profile=None;self.profile_started=0;self.clip_events=set();self.panel_contact_seen=False
        super().__init__(job,'FAST_DEV',record_factory=record_factory)
        self.record.context=lambda:self.g
    def event(self,name,**data):
        previous=self.render.until;super().event(name,**data)
        keep=name in ['GRASP_VERIFIED','RELEASED','PANEL_FIRST_CONTACT','SAFETY_STOP'] and name not in self.clip_events
        if keep:self.clip_events.add(name)
        else:self.render.until=previous
    def step(self):
        row=super().step()
        if self.phase=='PANEL_PUSH' and not self.panel_contact_seen and any(c['force_n']>0 for c in row['contacts']):
            self.panel_contact_seen=True;self.event('PANEL_FIRST_CONTACT',angle_deg=row['door_angle_deg'])
        return row
    def motion_limits(self,phase):
        m=dict(self.policy.motion(phase));a=self.probe_profile
        if a and self.policy.phase(phase)=='LOADED':
            factor=1.
            if a.get('profile')=='smooth_variation':factor=.55+.45*math.sin(math.pi*(self.tick-self.profile_started)*self.dt/1.5)**2
            m['speed_m_s']=min(m['speed_m_s'],a.get('reference_speed_m_s',m['speed_m_s'])*factor)
            m['acceleration_m_s2']=min(m['acceleration_m_s2'],a.get('reference_acceleration_m_s2',m['acceleration_m_s2']))
        return m
    def record_frame(self):
        video=self.output/'events.mp4'
        if getattr(self,'optional_recording_disabled',False) or len(self.video_index)>=60 or (video.exists() and video.stat().st_size>=12000000):return
        return super().record_frame()

class ProbingOperations(Operations):
    def __init__(self,r):
        super().__init__(r);self.actions=[];self.worker=concurrent.futures.ThreadPoolExecutor(max_workers=1);self.updates=[]
        self.r.record.completed=self.update_async
    def observe(self):
        from native_g1_contact_recovery import observe_current
        with self.r.profile.measure('RGBD_SAM3_and_incremental_geometry'):return observe_current(self.g)
    def _update(self,ready):
        if not ready.wait(30):return dict(error='numeric packet export not ready')
        path=self.r.output/'identification_input.json'
        if not path.exists():return dict(error='no interaction prior yet')
        folder=self.r.output/'model';folder.mkdir(exist_ok=True)
        with (folder/'worker.log').open('w') as log:
            p=subprocess.run([sys.executable,str(self.r.root/'scripts/update_probing_v2_model.py'),str(self.r.output)],stdout=log,stderr=subprocess.STDOUT,timeout=60)
        return dict(returncode=p.returncode)
    def update_async(self,ready):self.updates.append(self.worker.submit(self._update,ready))
    def inputs(self):
        from native_g1_ft_identification import kinematic_prior
        atomic_json(self.r.output/'identification_input.json',dict(prior=kinematic_prior(self.g),bias=self.r.job['ft_calibration']['bias_sensor'],hidden_object_dynamics=False))
    def attempt(self,command):
        self.actions=list(command['segments']);self.r.world.play()
        return super().attempt(command)
    def interact(self,target_deg,max_sim_s=45):
        a=self.actions.pop(0) if self.actions else dict(target_deg=target_deg,role='FIT_INTERACTION',profile='constant')
        self.inputs();self.r.record.begin(a,'HANDLE');self.r.probe_profile=a;self.r.profile_started=self.r.tick
        result={}
        try:result=super().interact(target_deg,max_sim_s);return result
        finally:self.r.probe_profile=None;self.r.record.end(result)
    def push(self,command=None):
        command=command or {};self.inputs();self.r.world.play();self.r.record.router=list(command.get('segments',[]));self.r.probe_profile=command.get('motion_profile',dict(profile='constant'));self.r.profile_started=self.r.tick
        if not self.r.record.router:self.r.record.router=[dict(target_deg=self.panel_plan['target_deg'],role='VALIDATION_PANEL_PUSH',profile='constant')]
        try:return super().push()
        finally:
            self.r.record.end();self.r.record.router=[];self.r.probe_profile=None
    def feedback(self):
        folder=self.r.output/'model';model=folder/'estimate.json'
        return dict(attempts=self.g['contact_attempts'],model=json.loads(model.read_text()) if model.exists() else dict(status='worker pending'),plot=str(folder/'feedback.png'),
            events=self.r.events[-30:],actual_angle_deg=math.degrees(self.r.actual()),storage=storage(self.r.output))
    def close_workers(self):
        self.worker.shutdown(wait=True)

def action_check(a):
    if not np.isfinite(a['target_deg']):raise ValueError('finite target required')
    if not a['role'].startswith(('FIT','VALIDATION')):raise ValueError('declare FIT/VALIDATION before motion')
    for k in ['reference_speed_m_s','reference_acceleration_m_s2']:
        if k in a and (not np.isfinite(a[k]) or a[k]<=0):raise ValueError('positive finite profile required')

def serve(r,operations_factory=ProbingOperations):
    ops=operations_factory(r);g=r.g;commands=r.output/'agent_commands';responses=r.output/'agent_responses'
    commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True);trace=[]
    try:
        from native_g1_wrist_views import survey_candidates,select_route
        proposal=next(survey_candidates(r.model,g['wrist_config']));route,audit=select_route(r.model,r.solver,r.collision,r.base,np.asarray(r.robot.get_joint_positions())[r.arm],[proposal],g['wrist_config'],g['wrist_X'])
        if route is None:return dict(status='INITIAL_VIEW_UNAVAILABLE',audit=audit)
        r.move_path(route['q_path'][1:],'PREGRASP');r.hold(g['wrist_config']['settle_s']);g['wrist_routes'].append(route)
        first=ops.observe();atomic_json(r.output/'INITIAL_OBSERVATION_READY.json',result_packet(first))
        for seq in range(22):
            if g['recovery_state']!='UNLOADED_OBSERVATION':return dict(status='SAFE_UNLOADED_BOUNDARY_UNAVAILABLE',trace=trace)
            # Unloaded communication deadline; independent of all motion/force limits.
            wait_s=float(r.job.get('agent_wait_timeout_s',120))
            atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,unloaded=True,timeout_s=wait_s))
            path=commands/('%03d.json'%seq);began=time.perf_counter()
            while not path.exists():
                if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.perf_counter()-began>wait_s:return dict(status='BOUNDED_AGENT_STOP',trace=trace)
                time.sleep(.05)
            command,ack=receive(path,seq,responses)
            if command is None:trace.append(ack);continue
            op=command['operation'];started=time.perf_counter()
            try:
                for a in command.get('segments',[]):action_check(a)
                if op=='observe':result=ops.observe()
                elif op=='inspect':
                    from native_g1_agent_observation import inspect
                    result=inspect(Path(g['agent_observations'][-1]['folder']),g['agent_observations'])
                elif op=='evaluate_view':result=ops.evaluate_view(command['target']);ops.cache[seq]=result
                elif op=='move_and_observe':result=ops.move_and_observe(ops.cache.get(command['evaluated_command'],{}))
                elif op=='build_geometry':result=ops.build_geometry()
                elif op=='validate_execution':result=ops.validate_execution()
                elif op=='attempt_grasp_and_probe':result=ops.attempt(command)
                elif op=='inspect_contact_result':result=ops.feedback()
                elif op in ['reobserve_and_replan','fresh_mode_observation']:result=ops.reobserve()
                elif op=='plan_panel_push':result=ops.plan_push(command)
                elif op=='execute_panel_push':g['recovery_state']='LOADED_OPERATION';result=ops.push(command)
                elif op=='safe_release_retreat':r.world.play();result=ops.release_park()
                elif op=='finish':r.world.play();result=dict(safe_exit=ops.release_park());result['completed']=bool(result['safe_exit']['safe'])
                else:raise ValueError('Operation not exposed')
            except BaseException as error:
                result=dict(error=repr(error),operation_stopped=True);complete(responses,seq,result_packet(dict(result=result)));trace.append(dict(command=command,result=result));return dict(status='OPERATION_STOP',trace=trace)
            entry=dict(command=command,result=result,wall_s=time.perf_counter()-started,unloaded_wait_s=started-began);complete(responses,seq,result_packet(entry));trace.append(entry)
            atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
            footprint=prune_optional(r.output)
            if footprint['total_bytes']>900000000:r.optional_recording_disabled=True
            atomic_json(r.output/'STORAGE_SUMMARY.json',footprint)
            if op=='finish':return dict(status='SESSION_COMPLETE',trace=trace)
        return dict(status='BOUNDED_COMMAND_BUDGET',trace=trace)
    finally:ops.close_workers()
