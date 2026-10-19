"""Private segment worker; all filtering is partition-local before fitting."""
import argparse,json,pickle
from pathlib import Path
import numpy as np
from torque_components_v1 import local_process,analyze
from xarm_ft_v1_estimator import KEYS

def main(folder):
    folder=Path(folder);settings=json.loads((folder/'INPUT.json').read_text());rows=pickle.load((folder/'measurements.pkl').open('rb'))
    # Partition known correction/unloaded intervals before derivative windows.
    # A one-pad zero report by itself is NOT removed or a physical failure.
    processed=[]
    for row in rows:
        item=dict(row)
        if row.get('phase') in ['LOCAL_CENTERING','RELEASE','CLEARANCE_RETREAT'] or row.get('force_event',{}).get('status') not in [None,'NORMAL']:
            item['identification_segment']=None
        processed.append(item)
    data,provenance=local_process(processed,settings['prior'],settings['bias'])
    if data is None:
        (folder/'TORQUE_COMPONENTS.json').write_text(json.dumps(dict(status='insufficient local derivative support',provenance=provenance),indent=2));return
    pieces=[];samples=[]
    for p in settings.get('previous_packets',[]):
        p=Path(p)
        if (p/'TORQUE_BLOCK.npz').exists():
            pieces.append(dict(np.load(p/'TORQUE_BLOCK.npz',allow_pickle=True)));samples+=json.loads((p/'TORQUE_SAMPLES.json').read_text())
    np.savez_compressed(folder/'TORQUE_BLOCK.npz',**{k:data[k] for k in KEYS});(folder/'TORQUE_SAMPLES.json').write_text(json.dumps(data['samples']))
    pieces.append(data);samples+=data['samples'];joined={k:np.concatenate([d[k] for d in pieces]) for k in KEYS};joined['samples']=samples
    result=analyze(joined,settings['prior']);result['segment_provenance']=provenance
    (folder/'TORQUE_COMPONENTS.json').write_text(json.dumps(result,indent=2))
    np.savez_compressed(folder/'TORQUE_CUMULATIVE.npz',**{k:joined[k] for k in KEYS})
    print(json.dumps(dict(status=result.get('status'),identifiability=result.get('identifiability'),training_bins=result['training_bins'],validation_bins=result['validation_bins'])))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');main(p.parse_args().folder)
