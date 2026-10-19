"""Short frozen-owner cost/timing control; zero grasp or Agent interaction.

Only the top-level operation is replaced. Active controller/physics/guards,
full-rate original logging and video remain from the archived executable.
"""
import hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
from xarm_wrist_ft import prepare
c=prepare(out/'sensor_assembly')
import xarm7_native_gripper as hand
hand.configuration=lambda:c
import xarm7_native_execution as execution
original=execution.adapt_loader

def loader(source,config,output):
    text=original(source,config,output);anchor='world.reset(); camera.initialize()'
    addition="\nfrom xarm_ft_v1_scene import author\ncase=json.loads(Path("+repr(job['private_physics_configuration'])+").read_text())["+str(job.get('identification_case',0))+"]\n_id_physics_audit=author(stage,globals(),case,Path(os.environ['R1A7_RUN_DIR']))\ndel case\n"
    if text.count(anchor)!=1:raise ValueError('Frozen loader changed')
    return text.replace(anchor,addition+anchor)
execution.adapt_loader=loader
path=Path(job['legacy_executable']);text=path.read_text();old='from native_g1_pull_push_v12 import run as run_contact_recovery'
if text.count(old)!=1:raise ValueError('Frozen archived owner boundary changed')
text=text.replace(old,'from legacy_runtime_short_operation import run as run_contact_recovery')
(out/'ACTUAL_LEGACY_SHORT_EXECUTABLE.py').write_text(text)
ns=dict(__name__='legacy_short_native_owner',__file__=str(ROOT/'scripts/run_native_g1_pull_push_v12.py'))
exec(compile(text,str(out/'ACTUAL_LEGACY_SHORT_EXECUTABLE.py'),'exec'),ns)
