"""Prepared next research entry; V3.1 mathematics, V4.4 execution feasibility."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v3 import RuntimeV3
from interaction_runtime.recovery_v44 import serve_v44
from run_informative_probing_v31 import ESTIMATOR_SOURCES
if __name__=='__main__':sys.exit(main(RuntimeV3,serve_v44,extra_sources=ESTIMATOR_SOURCES+('scripts/native_g1_execution_candidates.py','scripts/native_g1_agent_execution.py','scripts/native_g1_panel_support.py','scripts/native_g1_visual_input.py','scripts/xarm7_native_planning.py','scripts/plan_shared_door_skill.py')))
