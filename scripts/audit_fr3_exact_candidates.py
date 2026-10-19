"""Non-executing FR3 MoveIt audit of every original benchmark candidate."""
import json
import sys
from collections import Counter
from pathlib import Path

FR3=Path('/data1/home/rangeryx/fr3_moveit_grasp')
REFERENCE=FR3/'results/run_1788943864166403373/trial_01_grasps.json'
sys.path.insert(0,str(FR3/'src'))
import rclpy
from backend import Backend, Failure
from frames import grasp_to_tcp
import plant

def main():
    data=json.loads(REFERENCE.read_text())
    rclpy.init();node=Backend()
    rows=[]
    try:
        plant.command({'op':'reset'});plant.settle(1.)
        node.check_fk();node.scene();node.gripper(.08)
        for g in data['grasps']:
            row=dict(rank=g['rank'],score=g['score'],status='UNKNOWN')
            try:
                if not 0<g['width']<=.08:raise Failure('GRIPPER_WIDTH')
                pre,grasp,_=grasp_to_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'])
                current=node.measured();gs=node.ik(grasp,current);ps=node.ik(pre,gs)
                node.cartesian(ps,grasp);node.plan(current,ps)
                row['status']='EXECUTABLE'
            except Failure as error:
                row.update(status=error.category,detail=str(error))
            rows.append(row)
    finally:
        node.destroy_node();rclpy.shutdown()
    report=dict(reference=str(REFERENCE),total=len(rows),executable=sum(r['status']=='EXECUTABLE' for r in rows),
                categories=dict(Counter(r['status'] for r in rows)),rows=rows)
    out=Path(__file__).resolve().parents[1]/'results/r1a7_fr3_exact_ab/fr3_candidate_audit.json'
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
