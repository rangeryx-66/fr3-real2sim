"""V5 research entry: frozen V4.4 Skill plus native passive law and V5 worker."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v5 import RuntimeV5,serve_v5
from run_informative_probing_v31 import ESTIMATOR_SOURCES
SOURCES=ESTIMATOR_SOURCES+('scripts/update_probing_v5_model.py','scripts/xarm_ft_v5_estimator.py','scripts/xarm_ft_v1_scene.py','scripts/run_angle_resistance_v5.py','scripts/native_g1_execution_candidates.py','scripts/native_g1_agent_execution.py','scripts/native_g1_panel_support.py','scripts/native_g1_visual_input.py','scripts/xarm7_native_planning.py','scripts/plan_shared_door_skill.py')
if __name__=='__main__':sys.exit(main(RuntimeV5,serve_v5,extra_sources=SOURCES))
