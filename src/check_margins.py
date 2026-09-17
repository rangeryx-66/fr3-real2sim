"""Fixed-output, planning-only 0/1/2 mm comparison; no inference or trajectory execution."""
import json
import rclpy
from clutter_backend import ClutterBackend,ROOT
import plant
rclpy.init();n=ClutterBackend(ROOT/'results/margin_sweep');reports=[]
for seed in range(20):
    plant.command(dict(op='reset',seed=seed));plant.settle(1.)
    n.mode='B';n.attached=None;n.initial=plant.state()
    data=json.loads((ROOT/f'results/clutter_ab_20_v1/inputs/seed_{seed:04d}_grasps.json').read_text())
    for margin in [0.,.001,.002]:
        n.margin=margin;n.reset_scene();details=[]
        for g in data['grasps']:
            d,_=n.candidate(g,data);details.append(d)
        valid=[d['rank'] for d in details if d['status']=='VALID']
        record=dict(seed=seed,margin_m=margin,total=len(details),remaining=len(valid),valid_ranks=valid,all_filtered=not valid,candidates=details)
        reports.append(record);(n.output/'sweep.json').write_text(json.dumps(reports,indent=2))
        print(json.dumps({k:v for k,v in record.items() if k!='candidates'}),flush=True)
n.margin=.001;n.reset_scene();n.destroy_node();rclpy.shutdown()
