"""Data adapter: existing Arena spawn/monitor with a frozen independent manifest."""
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'src/arena_scene.py').read_text()
changes={
    "ROOT/'assets/arena_complex/inventory.json'":"Path(os.environ['FR3_ASSET_DIRECTORY'])/'inventory.json'",
    "ROOT/'ARENA_COMPLEX_PROTOCOL.json'":"Path(os.environ['UNSEEN_PROTOCOL'])",
    "DEFAULT=next(e for e in EPISODES.values() if e['target']==TARGET)":"DEFAULT=EPISODES[int(os.environ['CALIBRATION_SCENE_SEED'])]",
    "range(5)":"range(3)",
    "forces.shape==(5,len(self.filters))":"forces.shape==(3,len(self.filters))",
}
for old,new in changes.items():
    assert old in source,old
    source=source.replace(old,new)
ns={'__file__':str(ROOT/'src/arena_scene.py')}
exec(compile(source,str(ROOT/'src/arena_scene.py'),'exec'),ns)
TARGET=ns['TARGET'];INVENTORY=ns['INVENTORY'];DEFAULT=ns['DEFAULT']
table=ns['table'];ArenaMonitor=ns['ArenaMonitor'];target_mask=ns['target_mask']
def reset(box):ns['reset_target'](box,int(os.environ['CALIBRATION_SCENE_SEED']))
