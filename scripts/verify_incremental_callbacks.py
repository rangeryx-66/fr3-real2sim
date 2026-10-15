"""Robot-free callback wiring check. This is not physical validation."""
import argparse,hashlib,json,time
from pathlib import Path
from unittest.mock import patch
import native_g1_agent_incremental_activation as adapter


def verify():
    fn=adapter.move_and_observe
    required=['actuate','observe']
    available={name:callable(fn.__globals__.get(name)) for name in required}
    if not all(available.values()):raise RuntimeError('Missing callback dependencies: '+repr(available))
    events=[]
    class World:
        def play(self):events.append('play')
        def pause(self):events.append('pause')
    def actuate(g,route,label):events.append(['actuate',route,label])
    def observe(g):events.append(['observe',g['agent_moves']]);return {'fresh_observation':'synthetic only'}
    g=dict(world=World(),agent_moves=0)
    with patch.dict(fn.__globals__,actuate=actuate,observe=observe):result=fn(g,{'test_route':True})
    expected=['play',['actuate',{'test_route':True},'agent_adjustment_1'],['observe',1],'pause']
    if events!=expected or g['agent_moves']!=1:raise RuntimeError('Incorrect observation transition: '+repr(events))
    return dict(passed=True,dependencies=available,events=events,result=result,
                physics_executed=False,real_robot_motion_verified=False,
                source_sha256=hashlib.sha256(Path(adapter.__file__).read_bytes()).hexdigest(),
                checked_unix_s=time.time())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=verify();a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r))
