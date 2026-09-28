"""Plan every R1 candidate passing Dex1 geometry and continuous-path checks."""
import json
import sys
from collections import Counter
from pathlib import Path
import numpy as np
import rclpy

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from r1a7_backend import R1A7Backend, Failure, ROOT
import r1a7_plant as plant

REFERENCE=Path('/data1/home/rangeryx/fr3_moveit_grasp/results/run_1788943864166403373/trial_01_grasps.json')
OUT=ROOT/'results/r1a7_fr3_exact_ab/r1_candidate_audit.json'

def main():
    data=json.loads(REFERENCE.read_text())
    rclpy.init();node=R1A7Backend()
    report={}
    try:
        plant.command({'op':'reset'});plant.settle(1.)
        node.check_fk();node.scene();node.gripper(.09)
        center=plant.state()['box']
        for mode in ('raw','adapted'):
            details,feasible=node.select_candidates(data,mode,center)
            for detail,g,T_list in feasible:
                try:
                    T=np.asarray(T_list)
                    evaluated=node.evaluate_variant(T,np.asarray(detail['raw_T_B_TCP']),center,node.measured())
                    node.plan(node.measured(),evaluated['pre_state'])
                    detail['status']='EXECUTABLE'
                except Failure as error:
                    detail.update(status=error.category,detail=str(error))
            report[mode]=dict(total=len(details),path_valid=len(feasible),
                              executable=sum(d['status']=='EXECUTABLE' for d in details),
                              categories=dict(Counter(d['status'] for d in details)),
                              rows=[dict(rank=d['rank'],status=d['status'],variant=d.get('variant'),
                                         detail=d.get('detail')) for d in details])
    finally:
        node.destroy_node();rclpy.shutdown()
    OUT.parent.mkdir(parents=True,exist_ok=True);OUT.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
