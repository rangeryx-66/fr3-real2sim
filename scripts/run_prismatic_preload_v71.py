"""Isolated legal-preload entry; frozen revolute and native drive unchanged."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.prismatic_preload_v71 import PrismaticPreloadRuntime,serve_preload
from run_angle_resistance_v5 import SOURCES
if __name__=='__main__':
    sys.exit(main(PrismaticPreloadRuntime,serve_preload,extra_sources=SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/native_g1_prismatic_ranking.py','scripts/run_prismatic_preload_v71.py')))
