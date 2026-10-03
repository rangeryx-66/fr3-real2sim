"""Independent mobile scene adapter around the unchanged physical-contact loop."""
import runpy
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from interactive_twin_recovery.native import install
install()
runpy.run_path(str(ROOT/'scripts/run_interactive_twin_episode.py'),run_name='__main__')
