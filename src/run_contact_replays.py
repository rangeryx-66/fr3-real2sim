"""Sequential fixed-pose replay suite, never overlaps plant executions."""
import subprocess
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
bad=[0,1,6,8,13,14,17];controls=[3,4,5,7,9,10,11]
for name,seeds,empty in [('contact_replay_clutter_r1',bad,False),('contact_replay_empty_r1',sorted(bad+controls),True),('contact_replay_empty_r2',bad,True)]:
    print('START',name,flush=True)
    cmd=[sys.executable,'-u',str(ROOT/'src/contact_backend.py'),'--replay','--seeds',','.join(map(str,seeds)),'--output',str(ROOT/'results'/name)]
    if empty:cmd.append('--empty')
    with (ROOT/'logs'/f'{name}.log').open('a') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
    print('FINISHED',name,flush=True)
