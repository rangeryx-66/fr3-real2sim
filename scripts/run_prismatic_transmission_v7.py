"""Isolated engineering entry; all frozen revolute implementation stays intact."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime,serve_prismatic
from run_angle_resistance_v5 import SOURCES
if __name__=='__main__':
    sys.exit(main(PrismaticFeedbackRuntime,serve_prismatic,extra_sources=SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py',
        'scripts/update_probing_v7_model.py','scripts/native_g1_prismatic_ranking.py',
        'scripts/run_prismatic_transmission_v7.py','scripts/launch_prismatic_transmission_v7.py')))
