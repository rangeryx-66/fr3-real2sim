"""Registered Agent execution; controllers, serve loop and worker are frozen V7.3."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.registered_prismatic_v74 import RegisteredPrismaticRuntimeV74,serve_v73
from interaction_runtime.prismatic_v73 import audit_frozen
from run_angle_resistance_v5 import SOURCES
from prismatic_registration_v74 import ACTIVATION_SOURCES

if __name__=='__main__':
    audit_frozen(ROOT)
    sys.exit(main(RegisteredPrismaticRuntimeV74,serve_v73,extra_sources=SOURCES+ACTIVATION_SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/native_g1_prismatic_ranking.py','scripts/prismatic_visual_v73.py','scripts/prismatic_quality_v73.py',
        'scripts/prismatic_inspection_v73.py','scripts/update_prismatic_v73_model.py','configs/prismatic_preload_v71.json','configs/sensor_lighting_v1.json')))
