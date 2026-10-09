"""Normal closed setup/cooked export for KNOWN_MODEL_DIAGNOSTIC planning."""
from piper_mobile_execute import *
from types import SimpleNamespace
p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);args=p.parse_args();job=json.loads(args.job.read_text())
a=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),output=Path(job['output']),gpu=job['gpu'],ownership=None)
a.output.mkdir(parents=True,exist_ok=True);base=json.loads((a.source/'report.json').read_text())['robot_base_pose']
from interactive_twin.plant import bootstrap_job
from interactive_twin_refinement.assembly import install_setup_capture
install_setup_capture();scene=bootstrap_job(a,base,job)
from piper_mobile_demo.cooked_geometry import export_cooked
export_cooked(scene['stage'],a.output/'cooked_initial.json')
scene['world'].render()
import cv2
for i,camera in enumerate(scene['overview']):cv2.imwrite(str(a.output/f'overview_{i}.png'),cv2.cvtColor(np.asarray(camera.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
(a.output/'setup.json').write_text(json.dumps({'mode':'KNOWN_MODEL_DIAGNOSTIC','purpose':'closed scene geometry export only; no manipulation claim','actual_initial_joint':scene['articulation'].get_joint_positions().tolist(),'base':base},indent=2))
scene['app'].close()
