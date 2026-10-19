"""One V6 Agent-guided research entry; fixed RGB-D is write-only, not Agent vision."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v6 import RuntimeV6,serve_v6
from run_angle_resistance_v5 import SOURCES
SOURCES=SOURCES+('scripts/xarm_ft_v6_estimator.py','scripts/update_probing_v6_model.py','scripts/run_large_range_v6.py','configs/probing_v6_registration.json')
if __name__=='__main__':sys.exit(main(RuntimeV6,serve_v6,extra_sources=SOURCES))
