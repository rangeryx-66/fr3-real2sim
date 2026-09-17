"""Read-only MoveIt/GT preflight; sends no robot or gripper trajectory."""
import json
import numpy as np
import rclpy,plant
from arena_backend import ArenaBackend,ROOT,mesh,original_box
from clutter_backend import transform
from moveit_msgs.srv import GetPlanningScene,GetStateValidity,ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
def main():
    rclpy.init();n=ArenaBackend(str(ROOT/'results/arena_preflight'),geometry_filter=True,lift_gain_scale=2.)
    req=ApplyPlanningScene.Request();req.scene.is_diff=True
    probe=original_box('clutter_probe',[.15,.15,.20],np.eye(4));probe.operation=probe.REMOVE;req.scene.world.collision_objects=[probe];n.apply(req)
    n.reset_scene();s=plant.state();H=transform(s['tcp'],s['tcp_quat']);O=transform(s['box'],s['box_quat'])
    n.attach(H,O)
    get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene=n.call('get_planning_scene',get).scene;att=scene.robot_state.attached_collision_objects[0]
    assert len(att.object.meshes)==1 and len(att.object.meshes[0].triangles)==len(mesh.triangles)
    # Collision-only negative control: surround the attached object's surface with a
    # virtual non-target cube far from the robot. Restore scene immediately below.
    req=ApplyPlanningScene.Request();req.scene.is_diff=True
    T=O.copy();req.scene.world.collision_objects=[original_box('clutter_probe',[.15,.15,.20],T)];n.apply(req)
    q=GetStateValidity.Request();q.robot_state=n.measured();q.group_name='fr3_arm'
    v=n.call('check_state_validity',q)
    pairs=[(c.contact_body_1,c.contact_body_2) for c in v.contacts]
    assert any(set(pair)=={'box','clutter_probe'} for pair in pairs),pairs
    req.scene.world.collision_objects[0].operation=req.scene.world.collision_objects[0].REMOVE;n.apply(req);n.reset_scene()
    capture=plant.command(dict(op='capture'))
    assert capture['ok'] and capture['object_points']>=30,capture
    result=dict(attached_mesh_triangles=len(mesh.triangles),attached_link=att.link_name,negative_control_contacts=pairs,capture=capture,robot_trajectories_sent=0,physics_trials=0,obstacle_count=len(s['clutter']['obstacles']))
    (ROOT/'results/arena_preflight/adapter_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
