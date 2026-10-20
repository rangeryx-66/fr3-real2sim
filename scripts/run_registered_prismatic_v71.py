"""Versioned benchmark entry for prismatic cases; original door runner stays frozen."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.prismatic_benchmark_v71 import RegisteredPrismaticRuntimeV71
from interaction_runtime.registered_observation_v71 import serve_registered
from run_angle_resistance_v5 import SOURCES
if __name__=='__main__':
    sys.exit(main(RegisteredPrismaticRuntimeV71,serve_registered,extra_sources=SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/native_g1_prismatic_ranking.py','scripts/run_registered_prismatic_v71.py',
        'scripts/launch_registered_prismatic_v71.py','configs/prismatic_preload_v71.json')))
