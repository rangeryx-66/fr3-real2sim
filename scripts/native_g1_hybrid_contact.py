"""One bounded pull/push/planned-regrasp owner; frozen controls are reused.

Agent commands run only at the original verified unloaded decision boundary.
The sole new collision permission is an explicit native fingertip-cap/panel
pair during PANEL_PUSH. There is no object force, target or state command.
"""
import copy, inspect, json, pickle, subprocess, sys, time, traceback
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
import fcl
from scipy.spatial.transform import Rotation
import native_g1_contact_recovery as recovery
import native_g1_safe_termination as termination
import native_g1_agent_observation as observation
from native_g1_ft_identification import kinematic_prior
from native_g1_ft_multistate import segment
from native_g1_wrist_input import save_rgbd
from native_g1_wrist_views import path_to
from xarm_wrist_ft import install_passive, wrench_at, generalized_effort
from xarm_ft_v1_scene import install_native_law
from wrist_reconstruction.force_policy import TemporalForceGuard
from hybrid_panel_preflight import patch_from_rgbd, scoped_panel_check, FINGERS, JOINTS, PAD
from piper_mobile_demo.owned_scene import meshes


def guard_wrench_world(ft,bias):
    """Original calibrated native sign: transmitted wrench + tool gravity.

    Residual inertia stays in this conservative envelope, while the passive
    estimator separately computes complete segment-local inertia compensation.
    """
    T=np.asarray(ft['T_world_sensor']);w=np.asarray(ft['raw_wrench_sensor'])-np.asarray(bias)
    return wrench_at(w,T,T[:3,3])+np.asarray(ft['tool_gravity_wrench_world'])


def current_view_scene(g):
    return SimpleNamespace(check=g['collision'].check,moving_reference=np.asarray(g['moving']()).copy())


def evaluate_current_view(g,target):
    # The existing pure view evaluator reads this dictionary; it does not
    # mutate the actor. Its path checks must use today's opened door, not the
    # cooked closed-reference transform used for articulation kinematics.
    state=dict(g);state['collision']=current_view_scene(g)
    return observation.evaluate_view(state,target)


def initial_observation(g):
    from native_g1_wrist_views import survey_candidates,select_route
    recovery.set_state(g,'UNLOADED_OBSERVATION');g['world'].play()
    proposal=next(survey_candidates(g['model'],g['wrist_config']))
    q=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    route,audit=select_route(g['model'],g['solver'],current_view_scene(g),g['base'],q,[proposal],g['wrist_config'],g['wrist_X'])
    if route is None:g['world'].pause();return dict(observation=False,reason='INITIAL_CURRENT_STATE_WRIST_VIEW_UNAVAILABLE',audit=audit)
    observation.actuate(g,route,'initial_current_generation');result=recovery.observe_current(g);g['world'].pause();return result


def write(g,name,data):
    observation.write(g['a'].output/name,data)


class PanelScene:
    """Mode-scoped exception; closest actual raw face must be a distal cap."""
    def __init__(self, original, raw_fingers):
        self.original=original;self.active=False;self.paths=set();self.raw=raw_fingers
        # In the existing logical pad frames native finger +Z (distal) is
        # logical +Y. Logical Z is the jaw-closing normal, not the tip axis.
        self.cap_faces={n:np.max(abs(m.triangles[:,:,1]-m.vertices[:,1].max()),axis=1)<1e-6
                        for n,m in self.raw.items()}
    def __getattr__(self,key):return getattr(self.original,key)
    def on_cap(self,n,point,poses,separation=0.):
        local=(np.linalg.inv(poses[n])@np.r_[point,1])[:3]
        mesh=self.raw[n];triangles=mesh.triangles[self.cap_faces[n]]
        points=np.repeat(local[None,:],len(triangles),axis=0)
        closest=trimesh.triangles.closest_point(triangles,points)
        # Contact solvers report a point within the penetration interval, not
        # necessarily on the tool boundary. Use that existing interval only
        # for face ownership, never move the TCP. Roundoff is float32 precision,
        # not an additional allowed physical penetration or pad-edge margin.
        epsilon=8*np.finfo(np.float32).eps*max(1.,np.linalg.norm(point))
        return bool(np.min(np.linalg.norm(closest-local,axis=1))<=abs(separation)+epsilon)
    def check(self,poses,moving_pose,allow_handle):
        if not self.active:return self.original.check(poses,moving_pose,allow_handle)
        # No handle or cabinet exemption. The two actual cap identities are
        # checked separately; every other native pair uses the frozen checker.
        ok,why=scoped_panel_check(self.original,poses,moving_pose,self.paths)
        if not ok:return ok,why
        for robot in self.original.robot:
            if robot.body not in self.raw:continue
            robot.place(poses[robot.body])
            for target in self.original.scene:
                if target.path not in self.paths:continue
                result=fcl.CollisionResult()
                count=fcl.collide(robot.object,target.object,fcl.CollisionRequest(num_max_contacts=64,enable_contact=True),result)
                if count and any(not self.on_cap(robot.body,c.pos,poses,c.penetration_depth) for c in result.contacts):
                    return False,'PANEL_PUSH_GEOMETRY_NOT_ON_DISTAL_CAP:'+robot.body
        return True,'ONLY_EXPLICIT_EXISTING_CAP_PANEL_PAIR_ALLOWED'
    def contact_guard(self,contacts,poses):
        if not self.active:return self.original.contact_guard(contacts,poses)
        remaining=[]
        for c in contacts:
            if c['force_n']>0 and c['finger'] in self.raw and c['target'] in self.paths:
                p=c.get('contact_point_world_m')
                measured=poses if not hasattr(self,'measured_poses') else self.measured_poses(poses)
                if p is None or not self.on_cap(c['finger'],p,measured,c.get('separation_m',0.)):
                    return False,'PANEL_PUSH_NOT_ON_EXISTING_DISTAL_CAP:'+c['finger']
            elif c['force_n']>0 and c['finger'] in self.raw and c['target'] in self.original.allowed:
                return False,'HANDLE_CONTACT_NOT_AUTHORIZED_DURING_PANEL_PUSH:'+c['finger']
            else:remaining.append(c)
        return self.original.contact_guard(remaining,poses)


def save_segment(g,first,label,result):
    """Persist before any later segment; processing occurs after safe unload."""
    folder=g['a'].output/'hybrid_identification'/('%02d'%len(g['hybrid_segments']))
    folder.mkdir(parents=True,exist_ok=True)
    rows=g['rows'][first:]
    with (folder/'measurements.pkl').open('wb') as f:pickle.dump(rows,f,protocol=5)
    settings=dict(prior=kinematic_prior(g),bias=g['job']['ft_calibration']['bias_sensor'],
        previous_packets=[r['folder'] for r in g['hybrid_segments']],candidate_blocks={})
    observation.write(folder/'INPUT.json',settings)
    record=dict(label=label,folder=str(folder),result=result,records=len(rows),
        start_sim_s=rows[0]['t'] if rows else None,end_sim_s=rows[-1]['t'] if rows else None,
        measured_angle_extent_deg=[min(r['door_angle_deg'] for r in rows),max(r['door_angle_deg'] for r in rows)] if rows else None,
        full_wrist_wrench_primary=True,unloaded_rows_excluded_by_segment_labels=True)
    g['hybrid_segments'].append(record);write(g,'HYBRID_SEGMENTS.json',g['hybrid_segments'])


def update_models_unloaded(g):
    if g['recovery_state']!='UNLOADED_OBSERVATION':return dict(updated=False,reason='no verified unloaded decision state')
    results=[]
    for record in g['hybrid_segments']:
        folder=Path(record['folder'])
        if (folder/'PACKET_READY.json').exists():continue
        with (folder/'worker.log').open('w') as log:
            worker=subprocess.Popen([sys.executable,str(g['ROOT']/'scripts/run_xarm_ft_v1_estimation.py'),str(folder)],stdout=log,stderr=subprocess.STDOUT)
            began=time.perf_counter()
            while worker.poll() is None:
                g['app'].update();time.sleep(.1)
                if time.perf_counter()-began>90:worker.terminate();worker.wait(timeout=10);break
        status=dict(returncode=worker.returncode,physics_wait='verified unloaded only',raw_segment_preserved=True)
        observation.write(folder/'PACKET_READY.json',status);results.append(status)
    return dict(updated=True,packets=results)


def pull(g):
    first=len(g['rows']);ix=len(g['hybrid_segments']);target=g['hybrid_target_deg']
    action=dict(target_deg=target,speed_scale=.6,profile='smooth_variation',hold=True,role='FIT_HANDLE_PULL')
    result=dict(ok=False,status='segment interrupted')
    try:
        result=segment(g,action,ix)
        return result['ok'],result['status'],[result]
    finally:
        save_segment(g,first,'HANDLE_PULL',result)


def handle_attempt(g,command):
    planned=bool(g.get('hybrid_push_completed'))
    if g['physical_grasps_started']>=2:return dict(executed=False,reason='two planned acquisitions used; recovery acquisitions prohibited')
    if g['contact_attempts'] and not planned:return dict(executed=False,reason='second acquisition requires completed planned panel transition')
    g['interaction_mode']='HANDLE_REGRASP' if planned else 'HANDLE_PULL'
    g['hybrid_target_deg']=float(command.get('target_deg',15. if planned else 10.))
    # Pinned approach/closure/hold/measurement/guarded exit are unchanged.
    # Only the explicit action and planned-vs-recovery permission differ.
    source=inspect.getsource(recovery.attempt_grasp_and_probe)
    source=source.replace("    if any(r['short_success'] for r in g['contact_attempts']):return dict(executed=False,reason='first success: recovery prohibited by registration')\n",'')
    source=source.replace("    if any(tuple(r['family'])==key and r['physical_grasp_started'] for r in g['contact_attempts']):return dict(executed=False,reason='same failed family already attempted in this episode; reobserve/replan')\n",'')
    anchor="ok,status,_=namespace['open_with_contact_feedback'](g)"
    if source.count(anchor)!=1:raise RuntimeError('Frozen acquisition operation boundary mismatch')
    source=source.replace(anchor,'ok,status,_=pull(g)')
    ns=dict(recovery.__dict__,pull=pull,safe_release_retreat=safe_exit)
    exec(compile(source,__file__+'::planned_handle_acquisition','exec'),ns)
    result=ns['attempt_grasp_and_probe'](g,command)
    result['acquisition_kind']='PLANNED_POST_PUSH_REGRASP' if planned else 'INITIAL_HANDLE_GRASP'
    result['unexpected_recovery_attempts']=0;result['model_updates']=update_models_unloaded(g)
    write(g,'CONTACT_ATTEMPTS.json',g['contact_attempts']);return result


def safe_exit(g):
    g['interaction_mode']='SAFE_EXIT'
    result=termination.safe_release_park(g)
    if result.get('safe'):g['interaction_mode']='UNLOADED_REPOSITION'
    return result


def fresh_reobserve(g,command):
    if g['recovery_state']!='UNLOADED_OBSERVATION':return dict(executed=False,reason='not verified unloaded')
    g['observation_generations'].append(g['agent_observations']);g['agent_observations']=[];g['wrist_routes']=[]
    g['job']['_visual_proposal_used']=False
    feedback=g['a'].output/'incremental_feedback'
    if feedback.exists():feedback.rename(g['a'].output/('incremental_feedback_generation_%02d'%(len(g['observation_generations'])-1)))
    return initial_observation(g)


def close_geometry(g):
    """Native official finger solids in TCP coordinates at compact joint angle."""
    P=g['model'].poses(g['model'].home,g['base'],width=g['controller'].geometry.width(.85))
    vertices=[]
    for n,logical in zip(FINGERS,('gripper_link1','gripper_link2')):
        mesh=g['model'].native.meshes[n].copy()
        mesh.apply_transform(np.linalg.inv(P['tcp_link'])@P[n]);vertices.append(mesh.vertices)
    v=np.concatenate(vertices);tip=v[v[:,2]>v[:,2].max()-1e-5].mean(0)
    return tip


def panel_plan(g,command):
    if g['recovery_state']!='UNLOADED_OBSERVATION':return dict(valid=False,reason='not verified unloaded')
    folder=Path(g['agent_observations'][-1]['folder'])
    patch=patch_from_rgbd(folder,command['rectangle']);center=patch['center'];normal=patch['normal']
    direction=-normal;x=np.cross([0.,0.,1.],direction);x/=np.linalg.norm(x)
    R=np.column_stack([x,np.cross(direction,x),direction])@Rotation.from_euler('z',command.get('roll_deg',0),degrees=True).as_matrix()
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=center-R@close_geometry(g)
    # Observed contact point is immutable. Native scene supplies only collider
    # identity and path validation, never GT snapping of the point.
    from hybrid_panel_preflight import rays
    target_paths=set()
    for shape in g['collision'].original.scene:
        if shape.body!=g['collision'].moving_link:continue
        entry=g['collision'].entries[shape.path]
        for mesh in meshes(entry):
            local=np.linalg.inv(entry['rigid_body_world_transform'])@entry['world_transform'];mesh.apply_transform(local);mesh.apply_transform(g['moving']())
            d=rays(mesh,[center+.01*normal],direction)[0]
            if np.isfinite(d) and abs(d-.01)<.003:target_paths.add(shape.path)
    if not target_paths:return dict(valid=False,reason='observed patch has no corresponding current native moving surface')
    compact=g['controller'].geometry.width(.85);start=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    def check(q):
        return g['collision'].original.check(g['model'].poses(q,g['base'],width=compact),g['moving'](),False)
    for a in np.linspace(float(g['robot'].get_joint_positions()[g['controller'].master]),.85,35):
        ok,why=g['collision'].original.check(g['model'].poses(start,g['base'],finger_q=np.full(len(g['fingers']),a)),g['moving'](),False)
        if not ok:return dict(valid=False,reason='free-space compact sweep:'+why)
    proxy=SimpleNamespace(poses=lambda q,b,width:g['model'].poses(q,b,width=compact),margin=g['model'].margin,limits=g['model'].limits)
    scene=SimpleNamespace(check=g['collision'].original.check,moving_reference=g['moving']())
    pre=T.copy();pre[:3,3]+=.04*normal
    route,why=path_to(proxy,g['solver'],scene,g['base'],start,pre,dict(ik_starts=4,route_iterations=600),np.eye(4))
    if route is None:return dict(valid=False,reason=why)
    prior=kinematic_prior(g);q0=g['actual']();target_deg=float(command['target_deg']);seed=np.asarray(route[-1]);path=[]
    entry=[]
    for backoff in np.linspace(.04,.001,21):
        goal=T.copy();goal[:3,3]+=backoff*normal;seed=g['solver'].solve(goal,g['base'],seed)
        if seed is None:return dict(valid=False,reason='PANEL_ENTRY_IK')
        ok,why=check(seed)
        if not ok:return dict(valid=False,reason=why)
        entry.append(seed.tolist())
    contact_scene=PanelScene(g['collision'].original,g['collision'].raw)
    contact_scene.paths=target_paths;contact_scene.active=True
    # Validate actual intended contact and separation at successive current
    # known-hinge states. Contact is not enabled during unloaded approach.
    for angle in np.linspace(q0,np.deg2rad(target_deg),max(2,int(abs(np.rad2deg(q0)-target_deg)/.25)+1)):
        rot=Rotation.from_rotvec(np.asarray(prior['axis_world'])*(angle-q0)).as_matrix();D=np.eye(4);D[:3,:3]=rot;D[:3,3]=prior['origin_world_m']-rot@np.asarray(prior['origin_world_m'])
        goal=D@T;seed=g['solver'].solve(goal,g['base'],seed)
        if seed is None:return dict(valid=False,reason='PUSH_IK')
        ok,why=contact_scene.check(g['model'].poses(seed,g['base'],width=compact),D@g['moving'](),False)
        if not ok:return dict(valid=False,reason=why)
        path.append(dict(angle=float(angle),q=seed.tolist()))
    final_moving=D@g['moving']();separation=[]
    for distance in np.linspace(.001,.04,21):
        separating=goal.copy();separating[:3,3]-=distance*goal[:3,2];seed=g['solver'].solve(separating,g['base'],seed)
        if seed is None:return dict(valid=False,reason='FUTURE_PANEL_SEPARATION_IK')
        ok,why=g['collision'].original.check(g['model'].poses(seed,g['base'],width=compact),final_moving,False)
        if not ok:return dict(valid=False,reason=why)
        separation.append(seed.tolist())
    future_scene=SimpleNamespace(check=g['collision'].original.check,moving_reference=final_moving)
    exact=SimpleNamespace(solve=lambda *args,**kwargs:np.asarray(g['model'].home))
    parking,why=path_to(proxy,exact,future_scene,g['base'],seed,np.eye(4),dict(ik_starts=1,route_iterations=600),np.eye(4))
    if parking is None:return dict(valid=False,reason='FUTURE_PANEL_PARK:'+why)
    result=dict(valid=True,rectangle=command['rectangle'],observation=str(folder),center_world=center.tolist(),normal_world=normal.tolist(),T_contact=T.tolist(),
        target_paths=sorted(target_paths),route=route,entry_path=entry,push_path=path,separation_path=separation,parking_path=parking,
        starting_angle_rad=q0,target_deg=target_deg,prior=prior,
        compact_aperture_m=compact,point_source='fresh wrist RGB-D; Agent inspected ROI',native_use='narrow contact identity and original safety scene only')
    write(g,'HYBRID_PANEL_PLAN.json',result);g['hybrid_panel_plan']=result;return result


def push(g,command):
    plan=g.get('hybrid_panel_plan')
    if not plan or not plan['valid']:return dict(executed=False,reason='no current checked panel plan')
    if g['recovery_state']!='UNLOADED_OBSERVATION':return dict(executed=False,reason='not verified unloaded')
    if g.get('panel_push_attempted'):return dict(executed=False,reason='registered one panel-push attempt already used')
    g['panel_push_attempted']=True;g['panel_contact_tcp']=close_geometry(g)
    g['world'].play();g['interaction_mode']='UNLOADED_REPOSITION';g['phase']='PREGRASP';g['reference']=None;g['opening']=False
    start_width=g['controller'].feedback()['aperture_m'];rate=g['policy']['slow_closure_m_s'];compact=plan['compact_aperture_m']
    for width in np.linspace(start_width,compact,max(2,int(abs(start_width-compact)/(rate*g['dt']))+1)):
        g['jaw_width_command']=float(width);g['qtarget'][g['fingers']]=g['controller'].geometry.angle(width);g['step']()
    for q in plan['route'][1:]:g['move'](np.asarray(q),.8)
    g['hold'](.3);g['interaction_mode']='PANEL_PUSH';g['collision'].paths=set(plan['target_paths']);g['collision'].active=True
    g['identification_segment']='VALIDATION_PANEL_PUSH';g['phase']='PANEL_PUSH';first=len(g['rows']);result=dict(success=False,started_angle_deg=float(np.rad2deg(g['actual']())))
    q0=plan['starting_angle_rad'];T0=np.asarray(plan['T_contact']);axis=np.asarray(plan['prior']['axis_world']);origin=np.asarray(plan['prior']['origin_world_m']);reference=q0;g['push_wrench_guard']=TemporalForceGuard(g['job']['wrist_experiment']['force_policy'])
    try:
        # Slow entry from the measured pose, then measured-progress-bounded
        # closing. No object target/force and no gripping-force requirement.
        start=np.asarray(g['tcp']()).copy();radius=np.linalg.norm(np.cross(T0[:3,3]-origin,axis))
        for f in np.linspace(0,1,max(2,int(np.linalg.norm(T0[:3,3]-start[:3,3])/(.0015*g['dt'])))):
            goal=T0.copy();goal[:3,3]=(1-f)*start[:3,3]+f*T0[:3,3];push_step(g,goal)
        began=g['tick']*g['dt'];last_progress=began;best=g['actual']()
        while g['actual']()>np.deg2rad(plan['target_deg']):
            theta=g['actual']();reference=max(theta-np.deg2rad(.05),reference-.0015/max(radius,1e-9)*g['dt'])
            rot=Rotation.from_rotvec(axis*(reference-q0)).as_matrix();D=np.eye(4);D[:3,:3]=rot;D[:3,3]=origin-rot@origin
            push_step(g,D@T0)
            now=g['tick']*g['dt']
            if theta<best-np.deg2rad(.01):best=theta;last_progress=now
            if now-last_progress>6.:result['reason']='PANEL_TRANSMISSION_STALL';break
            if now-began>180.:result['reason']='REGISTERED_PUSH_TIME_BUDGET';break
        result.update(success=g['actual']()<=np.deg2rad(plan['target_deg']),final_angle_deg=float(np.rad2deg(g['actual']())))
    except Exception as error:result.update(reason=repr(error),traceback=traceback.format_exc())
    finally:
        g['qvelocity']=np.zeros(len(g['arm']));g['opening']=False;save_segment(g,first,'PANEL_PUSH',result);g['identification_segment']=None
    # No Agent wait under panel load. A normal-direction, checked separation
    # is attempted under the SAME guards. Hard stops never resume pushing.
    result['separation']=separate_panel(g)
    if result['separation'].get('safe'):
        g['hybrid_push_completed']=bool(result.get('success'));g['world'].pause();result['model_updates']=update_models_unloaded(g)
    else:recovery.set_state(g,'SAFE_TERMINATED_NO_RECOVERY');g['world'].pause()
    write(g,'HYBRID_PANEL_RESULT.json',result);return result


def push_step(g,goal):
    measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
    q=g['solver'].velocity_step(goal,g['base'],measured,g['dt'])
    if q is None:raise RuntimeError('PANEL_PUSH_NO_IK')
    if g['model'].margin(q)<=.05:raise RuntimeError('EXISTING_COMMAND_JOINT_MARGIN')
    ok,why=g['collision'].check(g['model'].poses(q,g['base'],finger_q=np.asarray(g['robot'].get_joint_positions())[g['fingers']]),g['moving'](),False)
    if not ok:raise RuntimeError(why)
    g['qtarget'][g['arm']]=q;g['qvelocity']=(q-measured)/g['dt'];g['step']()


def separate_panel(g):
    result=dict(safe=False)
    try:
        # Back off opposite the measured intended contact normal. Validate all
        # segments at the CURRENT door; no reverse historical approach replay.
        T=np.asarray(g['tcp']()).copy();path=[];seed=np.asarray(g['robot'].get_joint_positions())[g['arm']]
        for d in np.linspace(.001,.04,21):
            goal=T.copy();goal[:3,3]-=d*T[:3,2];seed=g['ik'](goal,seed)
            if seed is None:return dict(result,reason='PANEL_SEPARATION_NO_IK')
            ok,why=g['collision'].check(g['model'].poses(seed,g['base'],finger_q=np.asarray(g['robot'].get_joint_positions())[g['fingers']]),g['moving'](),False)
            if not ok:return dict(result,reason=why)
            path.append(seed.tolist())
        g['interaction_mode']='UNLOADED_REPOSITION';g['phase']='CLEARANCE_RETREAT'
        recovery.slow_retreat(g,[np.asarray(g['robot'].get_joint_positions())[g['arm']].tolist()]+path)
        g['hold'](.3)
        contacts=g['rows'][-1]['contacts'];loaded=[c for c in contacts if c['force_n']>0 and c['target'] in g['collision'].paths]
        if loaded:return dict(result,reason='PANEL_CONTACT_REMAINS')
        if not g['push_wrench_guard'].settled():return dict(result,reason='PANEL_WRENCH_NOT_UNLOADED')
        g['collision'].active=False;g['push_wrench_guard']=None
        # Original guarded release+joint retreat/park, from a separated compact
        # hand; opening sweep is checked before executing it.
        return safe_exit(g)
    except Exception as error:return dict(result,reason=repr(error),traceback=traceback.format_exc())


def run(g):
    install_passive(g);update=install_native_law(g);g.update(interaction_mode='UNLOADED_REPOSITION',hybrid_segments=[],hybrid_push_completed=False,push_wrench_guard=None)
    original_scene=g['collision'];raw={}
    for n,logical in zip(FINGERS,('gripper_link1','gripper_link2')):
        mesh=g['model'].native.meshes[n].copy();mesh.apply_transform(np.linalg.inv(PAD[n]));raw[logical]=mesh
    g['collision']=PanelScene(original_scene,raw)
    def measured_fingers(poses):
        result=dict(poses)
        for native,logical in zip(FINGERS,('gripper_link1','gripper_link2')):
            p,q=g['actual_views'][native].get_world_poses()
            result[logical]=g['matrix'](p[0],q[0])@PAD[native]
        return result
    g['collision'].measured_poses=measured_fingers
    observe=g['observe_step']
    def sample(state,contacts):
        observe(state,contacts);state['interaction_mode']=g['interaction_mode'];state['identification_segment']=g.get('identification_segment')
    g['observe_step']=sample;original_step=g['step']
    def step():
        update();result=original_step()
        guard=g.get('push_wrench_guard')
        if guard is not None:
            row=g['rows'][-1];ft=row.get('wrist_ft',{})
            if not ft.get('valid'):raise RuntimeError('PANEL_PUSH_WRIST_SENSOR_UNAVAILABLE')
            T=np.asarray(ft['T_world_sensor']);world=guard_wrench_world(ft,g['job']['ft_calibration']['bias_sensor'])
            # Conservative unfiltered wrist envelope includes residual inertial
            # loads; complete inertia compensation remains in passive estimator.
            tcp=np.asarray(g['tcp']());point=tcp[:3,:3]@g['panel_contact_tcp']+tcp[:3,3]
            lever=np.linalg.norm(point-T[:3,3]);equivalent=max(np.linalg.norm(world[:3]),np.linalg.norm(world[3:])/max(lever,1e-9))
            status=guard.update({'wrist_equivalent_n':float(equivalent)},g['dt'],row['t']);row['panel_wrist_guard']=status
            row['panel_wrench_gravity_compensated_world']=world.tolist()
            if np.any(abs(np.asarray(g['robot'].get_joint_velocities())[g['arm']])>g['vel']/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')
            if len(g['rows'])>1 and np.linalg.norm(np.asarray(row['T_tcp'])[:3,3]-np.asarray(g['rows'][-2]['T_tcp'])[:3,3])/g['dt']>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')
            if status['status'].startswith('HARD'):raise RuntimeError(status['status'])
        return result
    g['step']=step
    # One explicit high-level operation table, all original observation and
    # physical dependencies retained in their original namespace.
    source=inspect.getsource(recovery.run)
    anchor="elif op=='finish':result=dict(completed=True,reason=command.get('reason'))"
    addition="elif op=='plan_panel_push':result=panel_plan(g,command)\n            elif op=='execute_panel_push':result=push(g,command)\n            elif op=='fresh_mode_observation':result=fresh_reobserve(g,command)\n            "+anchor
    if source.count(anchor)!=1:raise RuntimeError('Explicit hybrid operation binding mismatch')
    source=source.replace(anchor,addition)
    source=source.replace("observation.evaluate_view(g,command['target'])","evaluate_current_view(g,command['target'])")
    ns=dict(recovery.__dict__,attempt_grasp_and_probe=handle_attempt,safe_release_retreat=safe_exit,panel_plan=panel_plan,push=push,
        fresh_reobserve=fresh_reobserve,initial_observation=initial_observation,evaluate_current_view=evaluate_current_view)
    exec(compile(source,__file__+'::explicit_hybrid_owner','exec'),ns)
    result=ns['run'](g);result.update(interaction_modes_version='hybrid_v0',planned_acquisitions=g['physical_grasps_started'],unexpected_recovery_attempts=0,
        panel_push_completed=g['hybrid_push_completed'],segments=g['hybrid_segments'],hybrid_success=bool(g['hybrid_push_completed'] and len(g['contact_attempts'])==2 and g['contact_attempts'][-1]['short_success']))
    write(g,'HYBRID_EPISODE_RESULT.json',result);return result
