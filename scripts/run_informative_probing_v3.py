"""One V3 Agent episode, using the existing lifecycle and explicit factory wiring."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v3 import RuntimeV3,serve_v3
if __name__=='__main__':
    sys.exit(main(RuntimeV3,serve_v3,extra_sources=('configs/informative_probing_v3.json','scripts/update_probing_v3_model.py','scripts/diagnose_probing_v3.py','scripts/run_informative_probing_v3.py')))
