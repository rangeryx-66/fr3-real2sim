"""Original revolute owners, versioned initial-wrist-image infrastructure only."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v7 import RuntimeV7
from interaction_runtime.registered_observation_v71 import serve_registered
from run_angle_resistance_v5 import SOURCES
if __name__=='__main__':
    sys.exit(main(RuntimeV7,serve_registered,extra_sources=SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/run_registered_revolute_v71.py','scripts/launch_registered_revolute_v71.py')))
