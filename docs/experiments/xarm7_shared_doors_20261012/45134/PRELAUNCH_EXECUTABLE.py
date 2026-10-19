"""Pre-contact continuity trial; frozen closure-fix actor copied from e1e534c. Frozen setup/opening/guards are executed literally.

The active closure module is generated and verified before Isaac is started.
Owner stop uses a cooperative marker (or a Python signal flag), followed by
pause/export. There are no new sensor or trajectory execution conditions.
"""
import sys,json,time,hashlib,importlib.util,signal,os,ast,gzip,pickle
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
_owner_stop_requested=False
closure_commands=[];closure_repair_calls=0;closure_reopenings=0


def request_owner_stop(signum=None,frame=None):
    global _owner_stop_requested
    _owner_stop_requested=True


def owner_step():
    if _owner_stop_requested or (a.output/'STOP_AND_EXPORT').exists():
        raise KeyboardInterrupt('OWNER_REQUESTED_PAUSE_EXPORT')
    return _frozen_step()


def finish_trial():
    """Used by the real actor and the no-manipulation lifecycle self-test."""
    try:world.pause()
    except Exception as error:diagnostic_issue('world_pause',error)
    for name,data in [('observations',rows),('physics_steps',native.physics_steps),('closure_commands',closure_commands)]:
        try:export_raw_records(name,data)
        except Exception as error:diagnostic_issue('export:'+name,error)
    try:
        if video is not None:video.stdin.close();video.wait(timeout=30)
    except Exception as error:diagnostic_issue('video_finalize',error)
    diagnostic_write(a.output/'events.json',json.dumps(telemetry_events,indent=2))
    saved={}
    for name,data in [('observations',rows),('physics_steps',native.physics_steps),('closure_commands',closure_commands)]:
        try:saved[name]={'buffer_records':len(data),'index':json.loads((a.output/(name+'_chunks')/'index.json').read_text())}
        except Exception as error:diagnostic_issue('export_index:'+name,error);saved[name]={'buffer_records':len(data),'index':None}
    diagnostic_write(a.output/'EXPORT_COMPLETE.json',json.dumps({'world_pause_requested':True,'streams':saved,'telemetry_issues':telemetry_issues,'owner_stop_requested':_owner_stop_requested or (a.output/'STOP_AND_EXPORT').exists()},indent=2))


def lifecycle_selftest():
    """Archived buffers, optionally empty Isaac app. No robot/object/physics."""
    import argparse,subprocess,shutil
    from types import SimpleNamespace
    p=argparse.ArgumentParser();p.add_argument('--lifecycle-selftest',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--isaac-empty',action='store_true');p.add_argument('--gpu',type=int,default=3);args=p.parse_args()
    global a,world,native,rows,video,telemetry_events,telemetry_issues,chunked_export,export_raw_records,closure_commands,_frozen_step
    a=SimpleNamespace(output=args.lifecycle_selftest);a.output.mkdir(parents=True,exist_ok=True)
    telemetry_issues={};telemetry_events=[{'event':'EXPORT_SELFTEST_ONLY','not_physical_evidence':True}];chunked_export=True
    def issue(k,e):telemetry_issues[k]=repr(e)
    def write(p,v):p.write_text(v)
    globals().update(diagnostic_issue=issue,diagnostic_write=write)
    tree=ast.parse(args.baseline.read_text());function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='export_raw_records')
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(args.baseline)+'::unchanged_export','exec'),globals())
    rows=[];nr=[]
    for name,dest in [('observations',rows),('physics_steps',nr)]:
        for f in sorted((args.fixture/(name+'_chunks')).glob('*.gz'))[:2]:dest.extend(pickle.load(gzip.open(f,'rb')))
    native=SimpleNamespace(physics_steps=nr);closure_commands=[{'fixture_only':True,'t':i/240} for i in range(2048)]
    video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','16x16','-r','30','-i','-','-an','-c:v','libx264',str(a.output/'export_fixture.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(a.output/'ffmpeg.log').open('w'))
    video.stdin.write(bytes(16*16*3*8))
    app=None
    if args.isaac_empty:
        os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'active_gpu':args.gpu,'physics_gpu':args.gpu,'multi_gpu':False})
    paused=[]
    if app is not None:
        from isaacsim.core.api import World
        empty_world=World(physics_dt=1/240,rendering_dt=1/30)
        empty_world.pause()
        world=SimpleNamespace(pause=lambda:(empty_world.pause(),paused.append(True)))
    else:world=SimpleNamespace(pause=lambda:paused.append(True))
    _frozen_step=lambda:None
    signal.signal(signal.SIGINT,request_owner_stop);signal.signal(signal.SIGTERM,request_owner_stop)
    write(a.output/'SELFTEST_READY.json',json.dumps({'pid':os.getpid(),'empty_Isaac':args.isaac_empty,'no_robot_or_object':True,'no_physics_steps_executed':True}))
    try:
        while True:
            owner_step()
            if app is not None:app.update()
            time.sleep(.02)
    except KeyboardInterrupt:
        finish_trial()
    def exact_values(x,y):
        import struct
        if type(x) is not type(y):return False
        if isinstance(x,float):return struct.pack('!d',x)==struct.pack('!d',y)
        if isinstance(x,dict):return x.keys()==y.keys() and all(exact_values(x[k],y[k]) for k in x)
        if isinstance(x,(list,tuple)):return len(x)==len(y) and all(exact_values(a,b) for a,b in zip(x,y))
        if hasattr(x,'dtype'):return x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes()
        return x==y
    checks={}
    for name,data in [('observations',rows),('physics_steps',nr),('closure_commands',closure_commands)]:
        restored=[]
        for f in sorted((a.output/(name+'_chunks')).glob('*.gz')):restored.extend(pickle.load(gzip.open(f,'rb')))
        checks[name]={'records':len(data),'restored_records':len(restored),'lossless':exact_values(data,restored),'comparison':'exact types/values and float bits; cross-chunk Python string aliasing is not sensor data'}
    write(a.output/'SELFTEST_RESULT.json',json.dumps({'classification':'EXPORT_LIFECYCLE_SELFTEST_ONLY','checks':checks,'pause_called':bool(paused),'video_returncode':video.returncode,'physical_trials_consumed':0,'empty_Isaac':args.isaac_empty,'telemetry_issues':telemetry_issues},indent=2))
    if app is not None:app.close()


if '--lifecycle-selftest' in sys.argv:
    lifecycle_selftest();sys.exit(0)

spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
active_path=Path(spec['active_closure_source']).resolve()
module_spec=importlib.util.spec_from_file_location('verified_active_closure',active_path)
active_module=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(active_module)
FixedClosure=active_module.JawCenteredClosure
actor_output=Path(spec['output']);actor_output.mkdir(parents=True,exist_ok=True)
provenance={'actual_runner_path':str(Path(__file__).resolve()),'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'active_closure_path':str(active_path),'active_closure_sha256':hashlib.sha256(active_path.read_bytes()).hexdigest(),'active_update_code_file':FixedClosure.update.__code__.co_filename,'argv':sys.argv,'pid':os.getpid()}
provenance['trajectory_helper_sha256']=hashlib.sha256((ROOT/'scripts/precontact_trajectory.py').read_bytes()).hexdigest()
# These are requested pre-launch provenance checks, before scene initialization.
if provenance['runner_sha256']!=spec['expected_runner_sha256'] or provenance['active_closure_sha256']!=spec['expected_active_closure_sha256'] or provenance['trajectory_helper_sha256']!=spec['expected_trajectory_helper_sha256']:
    raise RuntimeError('PRELAUNCH_SOURCE_MISMATCH_NO_ISAAC_STARTED')
(actor_output/'actor_start_provenance.json').write_text(json.dumps(provenance,indent=2))

from cross_robot_adapter import RobotPinkIK
from xarm7_runtime import configuration
PinkIK=lambda ignored,home:RobotPinkIK(configuration())
baseline=Path(spec['baseline_runner']);text=baseline.read_text()
from xarm7_door_adapter import adapt_baseline
text=adapt_baseline(text,spec)
text=text.replace("allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]", "from revolute10_fixed import authored_handle_colliders\nallowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])")
text=text.replace("if len(allowed)!=len(model.manifest['interaction_geometry']['pieces']):raise RuntimeError('FROZEN_PROXY_ASSOCIATION_FAILED')", "diagnostic_write(a.output/'authored_collider_association.json',json.dumps({'allowed':allowed,'geometry_changed':False}))")
setup,sequence=text.split("try:\n phase='SETTLE';hold(.5)",1)
exec(compile(setup,str(baseline)+'::unchanged_setup','exec'),globals())
_fixed_update=FixedClosure.update

def observed_closure_update(self,T,opening,forces,pad_centers,dt):
    global closure_repair_calls,closure_reopenings
    before=self.opening
    command=_fixed_update(self,T,opening,forces,pad_centers,dt)
    try:
        unilateral=max(forces)>=.05 and min(forces)<.05
        if unilateral:closure_repair_calls+=1
        reopening=bool(before is not None and command['mode']=='position' and command['opening_m']>before+1e-7)
        closure_reopenings+=int(reopening)
        closure_commands.append({'t':tick*dt,'physics_step_index':tick,'previous_aperture_command_m':before,'measured_aperture_m':float(opening),'aperture_command_m':command.get('opening_m'),'mode':command['mode'],'filtered_pad_loads_n':list(map(float,forces)),'unilateral_centering_input':bool(unilateral),'state':command['state'],'command_reopening':reopening,'active_update_source_sha256':provenance['active_closure_sha256']})
    except Exception as error:diagnostic_issue('closure_execution_observer',error)
    return command

FixedClosure.update=observed_closure_update;JawCenteredClosure=FixedClosure
solver=PinkIK(ROOT/'config/piper.urdf',model.home);model.ik=solver.solve
_frozen_step=step;original_step=owner_step
# The next controller definitions are literal frozen Pink source.
grasp_relative=None;ref=None;twist=np.zeros(6)
goal=float(whole['goal_state']);motion_scale=float(spec.get('pink_motion_scale',1.));angle_reference=0.;bias=None;seed=None;opening=False
pink_source=Path(spec['pink_runner']).read_text()
controller_source='def body_at(theta):'+pink_source.split('def body_at(theta):',1)[1].split('initial_grasp=False',1)[0]
drive_correction=np.eye(4)
controller_source=controller_source.replace('moving()@grasp_relative','moving()@grasp_relative@drive_correction')
exec(compile(controller_source,spec['pink_runner']+'::unchanged_opening_control','exec'),globals())
from precontact_trajectory import segment_samples,repair_initial_sequence

def move(q,duration):
    global qvelocity,_profile_activity
    _profile_activity='motion'
    start=qtarget[arm].copy()
    cruise=.003 if phase=='APPROACH' else .030
    for command,velocity in segment_samples(solver,start,q,duration,dt,cruise):
        qtarget[arm]=command;qvelocity=velocity;step()
    qvelocity=np.zeros(7);_profile_activity='auto'

initial="if True:\n phase='SETTLE';hold(.5)"+sequence.split("  profile_mark('OPEN_1')",1)[0]
initial=repair_initial_sequence(initial)
provenance.update(opening_controller_sha256=hashlib.sha256(controller_source.encode()).hexdigest(),initial_sequence_sha256=hashlib.sha256(initial.encode()).hexdigest(),active_closure_alias=JawCenteredClosure is FixedClosure)
diagnostic_write(a.output/'active_source_provenance.json',json.dumps(provenance,indent=2))
try:
    signal.signal(signal.SIGINT,request_owner_stop);signal.signal(signal.SIGTERM,request_owner_stop)
    diagnostic_write(a.output/'EXECUTION_READY.json',json.dumps({'pid':os.getpid(),'owner_stop_marker':'STOP_AND_EXPORT','signal_handler_installed_after_bootstrap':True,'provenance':provenance},indent=2))
    exec(compile(initial,str(baseline)+'::unchanged_initial_sequence','exec'),globals())
    grasp_relative=np.linalg.inv(moving())@tcp();reference=grasp_relative.copy();ref=tcp().copy()
    seed=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-seed;angle_reference=actual();opening=True
    diagnostic_write(a.output/'acquired_grasp_reference.json',json.dumps({'T_body_tcp':grasp_relative.tolist(),'base':base,'q':seed.tolist(),'t':tick*dt}))
    policy_source=Path(spec['shared_policy']).read_text().replace("g['qvelocity']=np.zeros(6)","g['qvelocity']=np.zeros(7)")
    policy_namespace={};exec(compile(policy_source,spec['shared_policy']+'::joint_count_adapter','exec'),policy_namespace)
    success,status,adaptation_records=policy_namespace['open_with_contact_feedback'](globals())
    opening=False;qvelocity=np.zeros(7);phase='FINAL_HOLD' if success else 'TRANSMISSION_FAILURE_SAFE_HOLD'
except BaseException as error:
    import traceback
    status=repr(error);print(traceback.format_exc(),flush=True);opening=False;qvelocity=np.zeros(7)
finally:
    finish_trial()

grasp_event=next((e for e in telemetry_events if e.get('event')=='GRASP_VERIFIED'),None)
valid_rows=[r for r in rows if grasp_event is not None and r['t']>=grasp_event['t']]
result={'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT / XARM7_APPROXIMATE_ACTUATOR_SIMULATION','asset_id':model.manifest['asset_id'],'success':success,'status':status,'initial_grasp':bool(legal),'target_deg':30.0,'max_actual_angle_deg':max_state,'max_actual_angle_after_valid_grasp_deg':max((r['door_angle_deg'] for r in valid_rows),default=None),'final_actual_angle_deg':float(np.rad2deg(actual())),'initial_actual_angle_deg':rows[0]['door_angle_deg'] if rows else None,'grasp_acquisition':grasp_event,'base_fixed':list(base),'base_moves_after_grasp':0,'regrasps':0,'intentional_releases':0,'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'physics_steps':len(native.physics_steps),'observation_records':len(rows),'closure_update_calls':len(closure_commands),'unilateral_fix_branch_calls':closure_repair_calls,'aperture_command_reopenings':closure_reopenings,'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=0),'motion_scale':motion_scale,'IK':'Pinocchio3.9.0/Pink3.3.0/DAQP','telemetry_issues':telemetry_issues,'first_divergence':first_divergence,'active_sources':provenance,'GT_dependencies':['oracle handle mesh geometry','known articulation geometry','actual moving-body/articulation feedback','collision model'],'scene_restart_or_stitching':False,'object_physics_or_geometry_changed':False}
diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))
diagnostic_write(a.output/'closure_fix_execution.json',json.dumps({'active_update_file':str(active_path),'active_update_source_sha256':provenance['active_closure_sha256'],'underlying_method_code_file':_fixed_update.__code__.co_filename,'update_calls':len(closure_commands),'unilateral_fix_branch_calls':closure_repair_calls,'command_reopenings_diagnostic_count':closure_reopenings,'proof_is_passive':True},indent=2))
diagnostic_write(a.output/'scene_preserved_after_result.json',json.dumps({'pid':os.getpid(),'status':status,'world_paused':True,'export_completed_before_owner_cleanup':True},indent=2))
print('PHYSICAL_RESULT',json.dumps(result),flush=True)
app.close()
