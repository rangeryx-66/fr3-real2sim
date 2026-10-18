"""Read-only capture of existing online estimates; never fits or controls motion."""
import argparse,hashlib,json,time
from pathlib import Path
from agent_command_protocol_v12 import atomic_json


def watch(root,limit_s=3300):
    root=Path(root);model=root/'model/estimate.json';started=time.monotonic();seen=None;history=[]
    while time.monotonic()-started<limit_s:
        if model.exists():
            blob=model.read_bytes();digest=hashlib.sha256(blob).hexdigest()
            if digest!=seen:
                value=json.loads(blob);seen=digest;m=value.get('constant_model',{})
                if m.get('parameters'):
                    entry=dict(captured_UTC=time.time(),model_sha256=digest,input_index_sha256=value.get('input_index_sha256'),training_bins=value['training_bins'],validation_bins=value['validation_bins'],parameters=m['parameters'],conditional_std=m.get('conditional_std'),information=m.get('information'),worker_wall_s=value.get('worker_wall_s'))
                    history.append(entry);atomic_json(root/'ONLINE_ESTIMATE_CAPTURE.json',dict(read_only=True,updates=history,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
                    if value['validation_bins']==0:
                        atomic_json(root/'PREVALIDATION_ONLINE_MODEL.json',value)
                        atomic_json(root/'PREVALIDATION_CAPTURE.json',entry)
        if (root/'PROCESS_COMPLETION.json').exists():break
        time.sleep(.5)
    atomic_json(root/'ONLINE_CAPTURE_FINISHED.json',dict(updates=len(history),wall_s=time.monotonic()-started,never_reads_hidden_truth=True))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);a=p.parse_args();watch(a.episode)
