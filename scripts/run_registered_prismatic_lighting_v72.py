"""Explicit relit entry; no automatic benchmark continuation or action policy."""
import argparse,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from launch_job_v7 import verify_registration
from run_efficient_probing_v2 import main
from interaction_runtime.prismatic_lighting_v72 import LitRegisteredPrismaticRuntimeV72
from interaction_runtime.registered_observation_v71 import serve_registered
from run_angle_resistance_v5 import SOURCES


if __name__=='__main__':
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--job',type=Path,required=True)
    args,_=parser.parse_known_args();verify_registration(ROOT,json.loads(args.job.read_text()))
    external=json.loads((ROOT/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_EXTERNAL_FREEZE.json').read_text())
    bad=[p for p,h in external['external_robot_camera_closure_uncertainty_sha256'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    if bad:raise ValueError('Frozen external dependencies changed: '+str(bad))
    sys.exit(main(LitRegisteredPrismaticRuntimeV72,serve_registered,extra_sources=SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/native_g1_prismatic_ranking.py','configs/prismatic_preload_v71.json',
        'configs/sensor_lighting_v1.json','scripts/run_registered_prismatic_lighting_v72.py')))
