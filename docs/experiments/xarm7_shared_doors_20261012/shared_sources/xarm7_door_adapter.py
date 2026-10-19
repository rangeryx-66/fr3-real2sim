"""Thin robot-specific scene and joint-count adapter to the frozen shared skill."""
import inspect, hashlib, json
from pathlib import Path
from cross_robot_adapter import RobotModel,RobotPinkIK
from xarm7_runtime import configuration

def robot_model(ignored,asset,source):return RobotModel(configuration(),asset,source)

def bootstrap_job(args,base,job):
    import piper_mobile_execute as original
    import interactive_twin.plant as plant
    c=configuration();code=inspect.getsource(original.bootstrap)
    # Skip the PiPER preparation/writer entirely; preserve frozen robot inputs.
    start=code.index('    # Fold the massless virtual opening command')
    end=code.index('    monitors=',start)
    code=code[:start]+"    source=source.replace(\"subprocess.run([sys.executable,str(ROOT/'scripts/prepare_piper_description.py')],check=True)\",\"pass # xArm model prepared in isolated output\")\n"+code[end:]
    for option,field in (('asset-x','x_m'),('asset-y','y_m'),('asset-yaw-deg','yaw_deg'),('fixture-height-m','fixture_height_m')):
        code=code.replace(f"'--{option}',str(place['{field}'])",f"'--{option}='+str(place['{field}'])")
    marker="    exec(compile(source,str(legacy),'exec'),scene);sys.argv=old;"
    replacement="    source=adapt_xarm_loader(plant_adapt(source,benchmark_job),runtime_config,a.output)\n    (a.output/'ACTUAL_XARM_LOADER.py').write_text(source)\n"+marker
    if code.count(marker)!=1:raise ValueError('PRELAUNCH_LOADER_ANCHOR_MISMATCH')
    code=code.replace(marker,replacement)
    ns=dict(original.__dict__,plant_adapt=plant.adapt_loader_source,benchmark_job=job,runtime_config=c,adapt_xarm_loader=adapt_xarm_loader)
    exec(compile(code,str(Path(__file__))+'::robot_scene_adapter','exec'),ns)
    scene=ns['bootstrap'](args,base)
    for name,key in [('baked_mass_setup_private.json','baked_mass_audit'),('visual_import_audit.json','visual_import_audit'),('plant_setup_private.json','plant_resistance_audit')]:
        try:(args.output/name).write_text(json.dumps(scene[key],indent=2))
        except Exception:pass
    return scene

def adapt_xarm_loader(source,c,output):
    source=source.replace("ROOT / 'config/piper_sim.urdf'","Path("+repr(c['sim_urdf'])+")")
    source=source.replace('nargs=6','nargs=7').replace("names.index(f'joint{i}') for i in range(1, 7)","names.index(f'joint{i}') for i in range(1, 8)")
    source=source.replace("['link1','link2','link3','link4','link5','link6','flange_link','gripper_base','gripper_link1','gripper_link2']","['link1','link2','link3','link4','link5','link6','link7','gripper_base','gripper_link1','gripper_link2']")
    source=source.replace('kp[arm] = 10000.', 'kp[arm] = '+repr(c['arm_kp'])).replace('kd[arm] = 400.','kd[arm] = '+repr(c['arm_kd']))
    reset='world.reset(); camera.initialize()'
    stage="from xarm7_runtime import configure_stage,configuration,XArm7Drive\nxarm_config=configuration()\nconfigure_stage(stage,'/World/Piper',xarm_config,Path(os.environ['R1A7_RUN_DIR']))\n"
    source=source.replace(reset,stage+reset)
    old="robot.set_joint_positions(q); robot.apply_action(ArticulationAction(joint_positions=q))"
    new="""robot.set_joint_positions(q); robot.set_joint_velocities(np.zeros(len(q)))
controller=XArm7Drive(robot,xarm_config,Path(os.environ['R1A7_RUN_DIR']))
controller.apply_action(ArticulationAction(joint_positions=q[arm],joint_velocities=np.zeros(7),joint_indices=arm))
controller.apply_action(ArticulationAction(joint_positions=q[fingers],joint_efforts=np.zeros(2),joint_indices=fingers))
"""
    if source.count(old)!=1:raise ValueError('PRELAUNCH_ACTUATOR_INSTALL_ANCHOR_MISMATCH')
    source=source.replace(old,new)
    source=source.replace('startup_angles=[]','startup_angles=[];xarm_startup_records=[]')
    source=source.replace('    world.step(render=i % 8 == 0)',"""    controller.apply_action(ArticulationAction(joint_positions=q[arm],joint_velocities=np.zeros(7),joint_indices=arm))
    world.step(render=i % 8 == 0)
    try:xarm_startup_records.append({'step':i,'t':float(world.current_time),'q':robot.get_joint_positions().tolist(),'dq':robot.get_joint_velocities().tolist(),'projected_efforts':robot.get_measured_joint_efforts().tolist(),'joint_reactions':robot.get_measured_joint_forces().tolist(),'actuator':controller.last.copy()})
    except Exception:pass""")
    source += "\nimport gzip,pickle\nwith gzip.open(Path(os.environ['R1A7_RUN_DIR'])/'startup_240hz.pkl.gz','wb',compresslevel=1) as f:pickle.dump(xarm_startup_records,f,protocol=5)\n"
    # The final imported joint/body mappings and mass tensor channels are passive.
    source += "\ntry:\n native_tensor={}\n for method in ['get_masses','get_inertias','get_coms']:\n  native_tensor[method]=np.asarray(getattr(robot._articulation_view._physics_view,method)()).tolist()\n (Path(os.environ['R1A7_RUN_DIR'])/'NATIVE_MASS_TENSOR_READBACK.json').write_text(json.dumps(native_tensor,indent=2))\nexcept Exception:pass\n"
    return source

def adapt_baseline(text,spec):
    text=text.replace('from interactive_twin.plant import bootstrap_job','from xarm7_door_adapter import bootstrap_job')
    text=text.replace('from piper_mobile_demo.model import Model','from xarm7_door_adapter import robot_model as Model')
    text=text.replace('qvelocity=np.zeros(6)','qvelocity=np.zeros(7)')
    old="urdf=ET.parse(ROOT/'config/piper.urdf').getroot();vel=np.array([float(urdf.find(f\"joint[@name='joint{i}']/limit\").get('velocity')) for i in range(1,7)]);limits=model.limits"
    new="from xarm7_runtime import configuration\nxarm_config=configuration();vel=np.array(xarm_config['arm_speed_limits']);limits=model.limits"
    if text.count(old)!=1:raise ValueError('PRELAUNCH_JOINT_LIMIT_SOURCE_ANCHOR_MISMATCH')
    text=text.replace(old,new)
    text=text.replace("'arm_gains':{'kp':10000.,'kd':400.}","'arm_gains':{'kp':xarm_config['arm_kp'],'kd':xarm_config['arm_kd'],'classification':'APPROXIMATE_XARM_SIMULATION'}")
    text=text.replace("s['force_event']=force_guard.update", "s['xarm_actuator']=controller.last.copy();s['force_event']=force_guard.update")
    return text

def executable(spec):
    """Compile one shared adapter before any scene exists, for both cases."""
    frozen=Path(spec['frozen_execution_runner']);text=frozen.read_text()
    anchor="baseline=Path(spec['baseline_runner']);text=baseline.read_text()"
    text=text.replace(anchor,anchor+"\nfrom xarm7_door_adapter import adapt_baseline\ntext=adapt_baseline(text,spec)")
    text=text.replace("from piper_pink_fixed import PinkIK", "from cross_robot_adapter import RobotPinkIK\nfrom xarm7_runtime import configuration\nPinkIK=lambda ignored,home:RobotPinkIK(configuration())")
    text=text.replace('qvelocity=np.zeros(6)','qvelocity=np.zeros(7)')
    needle="exec(compile(controller_source,spec['pink_runner']+'::unchanged_opening_control','exec'),globals())"
    replacement="drive_correction=np.eye(4)\ncontroller_source=controller_source.replace('moving()@grasp_relative','moving()@grasp_relative@drive_correction')\n"+needle
    text=text.replace(needle,replacement)
    old="""    while actual()<goal:
        phase='FIXED_BASE_PINK_OPEN';step()
    opening=False;qvelocity=np.zeros(7);phase='FINAL_HOLD';status='ORACLE_HANDLE_30_DEG_SUCCESS';success=True"""
    new="""    policy_source=Path(spec['shared_policy']).read_text().replace("g['qvelocity']=np.zeros(6)","g['qvelocity']=np.zeros(7)")
    policy_namespace={};exec(compile(policy_source,spec['shared_policy']+'::joint_count_adapter','exec'),policy_namespace)
    success,status,adaptation_records=policy_namespace['open_with_contact_feedback'](globals())
    opening=False;qvelocity=np.zeros(7);phase='FINAL_HOLD' if success else 'TRANSMISSION_FAILURE_SAFE_HOLD'"""
    if text.count(old)!=1:raise ValueError('PRELAUNCH_SHARED_OPENING_ANCHOR_MISMATCH')
    text=text.replace(old,new).replace('ONE_PRECONTACT_CONTINUITY_TRIAL','XARM7_APPROXIMATE_ACTUATOR_SIMULATION')
    # Keep normal pause/export behavior, then close the completed actor promptly.
    text=text[:text.index('while app.is_running():')]+"app.close()\n"
    return text
