"""Use the untouched physical runner, with optional passive tau_s at setup."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from interactive_twin_selection.native import install
install()
from run_interactive_twin_conditional_episode import main
if __name__=='__main__':main()
