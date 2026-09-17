"""Same material coupon with the original soup convex-decomposition geometry."""
from pathlib import Path
p=Path(__file__).with_name('friction_coupon.py');s=p.read_text()
s=s.replace("from isaacsim.core.prims import RigidPrim", "from isaacsim.core.prims import RigidPrim,SingleRigidPrim\nfrom isaacsim.core.utils.stage import add_reference_to_stage\nfrom pxr import UsdPhysics,UsdShade\nimport omni.usd")
s=s.replace("box=world.scene.add(DynamicCuboid('/World/target',name='target',position=[0,0,.051],scale=[.1,.1,.1],mass=1.,physics_material=target))", """add_reference_to_stage(json.loads((Path(__file__).resolve().parents[1]/'assets/arena_complex/inventory.json').read_text())['soup']['usd_path'],'/World/target')
stage=omni.usd.get_context().get_stage()
for prim in stage.Traverse():
 if str(prim.GetPath()).startswith('/World/target/') and prim.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(target.prim),bindingStrength='strongerThanDescendants',materialPurpose='physics')
box=world.scene.add(SingleRigidPrim('/World/target',name='target',position=np.array([0.,0.,.035]),orientation=np.array([.7071067811865476,0.,.7071067811865476,0.])))""")
s=s.replace('max_contact_count=128','max_contact_count=8192')
s=s.replace('[1.,2.,3.,4.,5.,6.]','[.5,1.,1.5,2.,2.5,3.]').replace("box.set_world_pose([0,0,.051],[1,0,0,0])", "box.set_world_pose([0,0,.035],[.7071067811865476,0,.7071067811865476,0])")
s=s.replace("'friction_coupon.json'", "'friction_soup_coupon.json'").replace('mass_kg=1.','mass_kg=.45')
exec(compile(s,str(p),'exec'),dict(__name__='__main__',__file__=str(p)))
