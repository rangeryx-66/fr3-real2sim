"""Scene-only adapter; execute the frozen simulator with manifest target reset poses."""
import os,json
from pathlib import Path
source=Path(__file__).with_name('sim_server.py')
text=source.read_text()
old='box.set_world_pose(BOX,[1,0,0,0]);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])'
new="heldout_pose=json.loads(Path(os.environ['FR3_TARGET_MANIFEST']).read_text())[str(cmd.get('seed',0))];box.set_world_pose(heldout_pose['position'],heldout_pose['quaternion_wxyz']);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])"
assert text.count(old)==1
# Only the reset pose expression changes. All dynamics, capture and control code stays original.
exec(compile(text.replace(old,new),str(source),'exec'),dict(__name__='__main__',__file__=str(source),os=os))
