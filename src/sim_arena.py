"""Data-only scene adapter around the hash-frozen physics/control server."""
from pathlib import Path
source=Path(__file__).with_name('sim_server.py');text=source.read_text()
changes={
"world.scene.add_default_ground_plane(z_position=-.06)":"world.scene.add_default_ground_plane(z_position=-.76)",
"world.scene.add(FixedCuboid('/World/table',name='table',position=[.5,0,-.025],scale=[.7,.7,.05],physics_material=mat))\nbox=world.scene.add(DynamicCuboid('/World/box',name='box',position=BOX,scale=SIZE,mass=.06,color=np.array([.8,.12,.08]),physics_material=mat))":"import arena_scene\nbox=arena_scene.table(world,omni.usd.get_context().get_stage(),mat)",
"clutter=ClutterMonitor(world,stage,mat) if a.clutter else None":"clutter=arena_scene.ArenaMonitor(world,stage,mat)",
"camera.add_distance_to_image_plane_to_frame()":"camera.add_distance_to_image_plane_to_frame()\ncamera.add_instance_id_segmentation_to_frame()",
"box.set_world_pose(BOX,[1,0,0,0]);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])":"arena_scene.reset_target(box,cmd['seed'])",
"mask=(np.abs(xyz-bp)<=SIZE/2+.002).all(axis=1)":"mask=arena_scene.target_mask(camera,valid)",
"np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K)":"np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K,rgb=camera.get_rgba())",
"world.reset()":"stage.Flatten().Export(str(ROOT/'assets/arena_complex'/f'{arena_scene.TARGET}_scene.usdc'))\nworld.reset()",
"trace.sample(tick*DT,clutter.phase if clutter else 'IDLE',robot.get_joint_positions(),target,bp,bq,tp,tq,pp,pq,f)":"trace.sample(tick*DT,clutter.phase if clutter else 'IDLE',robot.get_joint_positions(),target,bp,bq,tp,tq,pp,pq,f)\n            obp,obq=clutter.contacts.get_world_poses();trace.records[-1]['non_target_poses']=dict(positions=obp.tolist(),quaternions_wxyz=obq.tolist())",
}
for old,new in changes.items():
    assert text.count(old)==1,old
    text=text.replace(old,new)
exec(compile(text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))
