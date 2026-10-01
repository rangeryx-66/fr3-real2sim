"""Load the unchanged shared scene, without any robot or ROS dependency."""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
if '--door-sanity-only' not in sys.argv:raise ValueError('--door-sanity-only output.json is required')
legacy=ROOT/'src/r1a7_articulated_sim_server.py'
source=legacy.read_text().partition('commands = queue.Queue(); state = {}; lock = threading.Lock()')[0]
source=source.replace("subprocess.run([sys.executable,str(ROOT/'scripts/prepare_r1a7_description.py')],check=True)","pass # no robot description needed")
begin=source.index("asset_cache = ROOT / 'assets'");end=source.index('def one_prim(',begin)
source=source[:begin]+source[end:]
source=source.replace('from articulated_demo.isolation_diagnostics import run_door_without_robot','from piper_mobile_demo.door_sanity import run_door_without_robot')
exec(compile(source,str(legacy),'exec'),{'__name__':'piper_no_robot_sanity','__file__':str(legacy)})
