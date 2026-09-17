"""Certify geometric graspability with read-only reference poses, never used by the executor."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from clutter_backend import ClutterBackend, ROOT, collision_box
from clutter_scene import layout,SIZES
from moveit_msgs.srv import ApplyPlanningScene
import plant
rclpy.init();n=ClutterBackend(ROOT/'results/clutter_preflight')
plant.command(dict(op='reset',seed=0));plant.settle(1.);n.mode='B';n.reset_scene();n.initial=plant.state()
results=[]
for seed in range(20):
    centers,qs=layout(seed);attempts=[]
    for yaw in [0,45,90,135]:
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        for i,(center,q) in enumerate(zip(centers,qs)):
            R=Rotation.from_quat(np.roll(q,-1)).as_matrix();T=np.eye(4);T[:3,3]=center
            req.scene.world.collision_objects.append(collision_box(f'clutter_{i}',np.abs(R)@SIZES[i],T))
        n.apply(req)
        H=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
        G=H@np.array([[0,0,1],[0,1,0],[-1,0,0.]]).T
        g=dict(rank=0,score=0.,width=.07,depth=.01,rotation=G.tolist(),translation=(np.array([.5,0,.049])-G@np.array([.0005,0,0])).tolist())
        detail,plan=n.candidate(g,{'T_B_C':np.eye(4).tolist()})
        attempts.append(dict(yaw_degrees=yaw,status=detail['status'],detail=detail.get('detail'),checks=detail['checks']))
        if plan is not None:break
    results.append(dict(seed=seed,geometrically_graspable=plan is not None,attempts=attempts))
    print(json.dumps(results[-1]),flush=True)
report=dict(scope='Read-only geometric feasibility; reference poses never enter A/B candidate sets and are never executed',passed=all(r['geometrically_graspable'] for r in results),seeds=results)
(ROOT/'results/clutter_preflight/layout_feasibility.json').write_text(json.dumps(report,indent=2))
n.reset_scene();n.destroy_node();rclpy.shutdown()
