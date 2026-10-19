"""Versioned virtual-sensor assembly + existing Agent/recovery owner, frozen files intact."""
import hashlib,json,sys
from pathlib import Path
from xarm_wrist_ft import prepare

ROOT=Path(__file__).resolve().parents[1]
job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
c=prepare(out/'sensor_assembly')
# Bind configuration before importing modules that retain the official hand
# configuration function. The files/controllers themselves remain unchanged.
import xarm7_native_gripper as hand
hand.configuration=lambda:c
import xarm7_native_execution as execution
old_loader=execution.adapt_loader

def sensor_scene_loader(source,config,output):
    text=old_loader(source,config,output)
    anchor='world.reset(); camera.initialize()'
    if text.count(anchor)!=1:raise ValueError('Frozen scene assembly boundary changed before initialization')
    addition="\nfrom xarm_ft_id_scene import author as author_identification_scene\nimport json as _id_json\n_id_case=_id_json.loads(Path("+repr(job['private_physics_configuration'])+").read_text())["+str(job['identification_case'])+"]\n_id_physics_audit=author_identification_scene(stage,globals(),_id_case,Path(os.environ['R1A7_RUN_DIR']))\ndel _id_case\n"
    return text.replace(anchor,addition+anchor)

execution.adapt_loader=sensor_scene_loader
from run_native_g1_safe_termination import source
text=source(job)
text=text.replace('from native_g1_safe_termination import run as run_contact_recovery','from native_g1_ft_identification import run as run_contact_recovery')
anchor="controller_source=controller_source.replace('moving()@grasp_relative','moving()@grasp_relative@drive_correction')"
addition="""
id_direction=1.
controller_source=controller_source.replace('lag=max(0.,angle_reference-theta)', 'lag=max(0.,id_direction*(angle_reference-theta))')
controller_source=controller_source.replace('angle_reference=min(goal+np.deg2rad(.5),max(theta+np.deg2rad(.05),angle_reference+rate*dt))', 'angle_reference=id_direction*min(id_direction*goal+np.deg2rad(.5),max(id_direction*theta+np.deg2rad(.05),id_direction*angle_reference+rate*dt))')
"""
if text.count(anchor)!=1:raise ValueError('Frozen directional reference boundary changed before initialization')
text=text.replace(anchor,anchor+addition)
report_anchor="diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))"
text=text.replace(report_anchor,"result.update(classification='XARM7_NATIVE_G1 / WRIST_FT_IDENTIFICATION_V0 / AGENT_WRIST_RGBD / KNOWN_HINGE_STATE / IDEALIZED_LOAD_CELL / APPROXIMATE_ACTUATOR_SIMULATION',object_physics_or_geometry_changed=True,object_geometry_changed=False,object_physics_version='seeded density/resistance V0',sensor_model='idealized fixed-interface load cell; hardware sensor mass/geometry not modeled')\n"+report_anchor)
compile(text,__file__+'::ft_id_v0_owner','exec')
(out/'ACTUAL_FT_ID_EXECUTABLE.py').write_text(text)
files=job['activation_source_files']+['scripts/xarm_wrist_ft.py','scripts/xarm_ft_id_scene.py','scripts/xarm_ft_id_estimator.py','scripts/native_g1_ft_identification.py','scripts/run_xarm_ft_estimation.py','scripts/run_native_g1_ft_identification.py']
hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in files}
(out/'FT_ID_SOURCE_HASHES.json').write_text(json.dumps(dict(files=hashes,executable_sha256=hashlib.sha256(text.encode()).hexdigest(),reference='136b6e0',assembly='idealized virtual cut v0',object_physics_version='seeded density/resistance v0'),indent=2))
if '--preflight' in sys.argv:
    print(json.dumps(dict(compiled=True,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),files=len(hashes),physical_episodes=0)))
    sys.exit(0)
exec(compile(text,__file__+'::ft_id_v0_owner','exec'),globals())
