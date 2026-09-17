"""Single original Arena asset on original table; no grasp perception."""
import os,sys,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import arena_scene as arena
TARGET=os.environ.get('FR3_ARENA_TARGET','soup')
BOUNDS=np.array(arena.INVENTORY[TARGET]['bounds'])
POSITION=np.array([.5,0,-BOUNDS[0,2]+.001])
QUATERNION=np.array([1.,0,0,0])
SCENE_SEED=os.environ.get('CALIBRATION_SCENE_SEED')
if SCENE_SEED:
    protocol=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())
    episode=next(x for x in protocol['episodes'] if x['seed']==int(SCENE_SEED) and x['target']==TARGET)
    target=next(x for x in episode['objects'] if x['asset']==TARGET)
    POSITION=np.asarray(target['position'],dtype=float)
    QUATERNION=np.asarray(target['quaternion_wxyz'],dtype=float)
SPEC=dict(asset=TARGET,position=POSITION.tolist(),quaternion_wxyz=QUATERNION.tolist())
arena.DEFAULT={**arena.DEFAULT,'objects':[SPEC]}
def table(world,stage,material):return arena.table(world,stage,material)
def reset(box):
    box.set_world_pose(POSITION,QUATERNION);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])
