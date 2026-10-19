"""Native position-hand interface around frozen shared approach/closure/Pink.

No new manipulation policy. Logical labels identify real native fingers;
contact normals/points/impulses and collider paths retain the native values.
"""
import copy,inspect,json
from pathlib import Path
import numpy as np
import xarm7_door_adapter as frozen
from xarm7_native_gripper import configuration,GRIPPER_JOINTS,FINGER_BODIES,ROOT
from xarm7_native_planning import NativeModel,PAD_FRAMES,LOGICAL

def robot_model(ignored,asset,source):return NativeModel(ignored,asset,source)

def adapt_loader(source,c,output):
 source=frozen.adapt_xarm_loader(source,c,output)
 source=source.replace('from xarm7_runtime import configure_stage,configuration,XArm7Drive','from xarm7_native_gripper import configure_stage,configuration,NativeDrive,GRIPPER_JOINTS,NativeGripperGeometry').replace('XArm7Drive(robot,','NativeDrive(robot,')
 source=source.replace("[names.index('gripper_joint1'), names.index('gripper_joint2')]","[names.index(n) for n in GRIPPER_JOINTS]").replace("[names.index('gripper_joint1'),names.index('gripper_joint2')]","[names.index(n) for n in GRIPPER_JOINTS]")
 source=source.replace('q[fingers] = [.05,-.05]',"q[fingers] = NativeGripperGeometry(xarm_config).angle(.084)")
 source=source.replace("joint_positions=q[fingers],joint_efforts=np.zeros(2),joint_indices=fingers","joint_positions=q[fingers],joint_efforts=np.zeros(len(fingers)),joint_indices=fingers")
 source=source.replace('controller.apply_action(ArticulationAction(joint_positions=q[fingers],joint_efforts=np.zeros(len(fingers)),joint_indices=fingers))','controller.command_width(.084)')
 source=source.replace("['link1','link2','link3','link4','link5','link6','link7','gripper_base','gripper_link1','gripper_link2']","['link1','link2','link3','link4','link5','link6','link7','xarm_gripper_base_link','left_outer_knuckle','left_inner_knuckle','right_outer_knuckle','right_inner_knuckle','left_finger','right_finger']")
 for old,new in [('gripper_link1','left_finger'),('gripper_link2','right_finger'),('gripper_base','xarm_gripper_base_link')]:
  # Complete quoted link names only; never corrupt xarm_gripper_base_link.
  source=source.replace(repr(old),repr(new))
 source=source.replace("'multi_gpu': False","'multi_gpu': False,'anti_aliasing':0").replace("'multi_gpu':False","'multi_gpu':False,'anti_aliasing':0")
 return source

def bootstrap_job(args,base,job):
 code=inspect.getsource(frozen.bootstrap_job);ns=dict(frozen.__dict__,configuration=configuration,adapt_xarm_loader=adapt_loader)
 exec(compile(code,__file__+'::frozen_scene_interface','exec'),ns)
 return ns['bootstrap_job'](args,base,job)

def mapped_export(export):
 e=copy.deepcopy(export)
 for x in e['shapes']:
  body=x.get('rigid_body_path','').split('/')[-1]
  if body in FINGER_BODIES:
   logical=LOGICAL[FINGER_BODIES.index(body)]
   x['native_rigid_body_path']=x['rigid_body_path'];x['native_body_name']=body
   x['rigid_body_world_transform']=(np.asarray(x['rigid_body_world_transform'])@PAD_FRAMES[body]).tolist()
   x['rigid_body_path']=x['rigid_body_path'].rsplit('/',1)[0]+'/'+logical
 return e

def collision_scene(export,model,allowed):
 from articulated_interaction.physical_baseline import PhysicalScene
 return PhysicalScene(mapped_export(export),model,allowed)

def native_reports(stage,export,allowed,world,dt,provider):
 from articulated_interaction.physical_baseline import WholeFingerReports
 native=WholeFingerReports(stage,export,allowed,world,dt,provider)
 for e in export['shapes']:
  body=e.get('rigid_body_path','').split('/')[-1]
  if body in FINGER_BODIES:
   label=LOGICAL[FINGER_BODIES.index(body)];native.paths[e['path']]=(label,'whole_finger');native.finger_body_paths[e['rigid_body_path']]=label
 return native

def adapt_baseline(text,spec):
 text=frozen.adapt_baseline(text,spec)
 text=text.replace('from xarm7_door_adapter import bootstrap_job','from xarm7_native_execution import bootstrap_job').replace('from xarm7_door_adapter import robot_model as Model','from xarm7_native_execution import robot_model as Model')
 text=text.replace('from xarm7_runtime import configuration','from xarm7_native_gripper import configuration')
 text=text.replace('collision=PhysicalScene(export,model,allowed)','from xarm7_native_execution import collision_scene,native_reports\ncollision=collision_scene(export,model,allowed)')
 text=text.replace('native=WholeFingerReports(stage,export,allowed,world,dt,sample)','native=native_reports(stage,export,allowed,world,dt,sample)')
 text=text.replace("chain.root_to_link('gripper_base',positions)","chain.root_to_link('xarm_gripper_base_link',positions)").replace("actual_views['gripper_base']","actual_views['xarm_gripper_base_link']")
 text=text.replace('qtarget[fingers]=[.05,-.05]','jaw_width_command=.084;preload_position_origin=None;qtarget[fingers]=controller.geometry.angle(jaw_width_command)')
 text=text.replace('float(q[fingers[0]]-q[fingers[1]])',"controller.geometry.width(float(q[controller.master]))")
 text=text.replace('float(qtarget[fingers[0]]-qtarget[fingers[1]])','jaw_width_command')
 old=" controller.apply_action(ArticulationAction(joint_positions=qtarget[fingers],joint_efforts=np.zeros(2),joint_indices=fingers) if mode=='position' else ArticulationAction(joint_efforts=np.array([-effort,effort]),joint_indices=fingers))"
 if old not in text:raise ValueError('PRELAUNCH_NATIVE_HAND_ACTION_ANCHOR')
 text=text.replace(old," controller.command_width(jaw_width_command) # native finite position hold in both logical closure states")
 text=text.replace("if cmd['mode']=='position':qtarget[fingers]=[cmd['opening_m']/2,-cmd['opening_m']/2]","if cmd['mode']=='position':jaw_width_command=float(cmd['opening_m']);qtarget[fingers]=controller.geometry.angle(jaw_width_command)")
 text=text.replace("    effort=cmd['closing_effort_n']", """    effort=cmd['closing_effort_n'] # requested legacy preload, never a native effort command
    if preload_position_origin is None:preload_position_origin=float(robot.get_joint_positions()[controller.master])
    preload_angle=preload_position_origin+effort*abs(controller.geometry.width_derivative(preload_position_origin))/xarm_config['gripper_kp_nm_rad']
    jaw_width_command=controller.geometry.width(float(np.clip(preload_angle,0,.85)))
    qtarget[fingers]=controller.geometry.angle(jaw_width_command)""")
 text=text.replace('width=.1','width=.084')
 text=text.replace("'command_finger_efforts':([0.,0.] if mode=='position' else [-effort,effort]),'gripper_control_mode':mode","'command_finger_efforts':[0.]*len(fingers),'legacy_closure_effort_request_not_applied':effort,'gripper_control_mode':'NATIVE_FINITE_POSITION_HOLD','native_gripper_feedback':controller.feedback()")
 text=text.replace("'gripper_position_gains':{'kp':1000.,'kd':40.,'effort_explicitly_zero':True}","'gripper_position_gains':{'kp':10.,'kd':.1,'effort_explicitly_zero':True,'native_position_hold':True,'hardware_force_feedback':False}")
 text=text.replace("'gripper_effort_gains':{'kp':0.,'kd':40.}","'gripper_effort_gains':None")
 # Hardware contact reports are not invented; all original native records
 # and physical mappings remain in the exported source and sensor metadata.
 return text

def executable(spec):
 text=frozen.executable(spec)
 text=text.replace('from xarm7_runtime import configuration','from xarm7_native_gripper import configuration')
 text=text.replace('from xarm7_door_adapter import adapt_baseline','from xarm7_native_execution import adapt_baseline')
 text=text.replace("XARM7_APPROXIMATE_ACTUATOR_SIMULATION","XARM7_NATIVE_GRIPPER_APPROXIMATE_ACTUATOR_SIMULATION")
 # Recording verification requested by this experiment, before any approach.
 text=text.replace("initial=repair_initial_sequence(initial)","""initial=repair_initial_sequence(initial)
initial=initial.replace("phase='SETTLE';hold(.5)","phase='SETTLE';hold(.5)\\n if video is None or video.poll() is not None:raise RuntimeError('RECORDING_PREFLIGHT_FAILED_BEFORE_APPROACH')\\n frame=near.get_rgba()\\n if frame is None:raise RuntimeError('CAMERA_PREFLIGHT_FAILED_BEFORE_APPROACH')\\n cv2.imwrite(str(a.output/'RECORDING_FIRST_FRAME.png'),cv2.cvtColor(np.asarray(frame)[:,:,:3],cv2.COLOR_RGB2BGR))\\n diagnostic_write(a.output/'RECORDING_READY.json',json.dumps({'encoder_alive':True,'ffmpeg':shutil.which('ffmpeg'),'before_approach':True,'frame_shape':list(frame.shape)}))")""")
 return text
