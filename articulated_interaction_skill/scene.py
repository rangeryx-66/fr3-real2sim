"""Import support for passive prismatic/multi-DOF dataset assets, scene-only."""
def install():
 import interactive_twin.plant as plant
 original=plant.adapt_loader_source
 def adapt(source,job):
  source=original(source,job)
  if job.get('skill',{}).get('joint_type')!='prismatic':return source
  # Only remove importer's object drives for both passive articulation families.
  source=source.replace("prim.IsA(UsdPhysics.RevoluteJoint):\n        drive=UsdPhysics.DriveAPI.Get(prim,'angular')", "(prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):\n        drive=UsdPhysics.DriveAPI.Get(prim,'angular' if prim.IsA(UsdPhysics.RevoluteJoint) else 'linear')")
  source=source.replace('if asset_names != [asset_joint_name]:','if asset_joint_name not in asset_names:')
  source=source.replace('articulation.set_joint_positions(np.array([0.]))',"articulation.set_joint_positions(np.zeros(len(asset_names)))\nselected_asset_dof=asset_names.index(asset_joint_name)")
  source=source.replace('articulation.set_joint_velocities(np.array([0.]))','articulation.set_joint_velocities(np.zeros(len(asset_names)))')
  # original adaptor may spell its zero explicitly as 0.0.
  source=source.replace('articulation.set_joint_positions(np.array([0.0]))',"articulation.set_joint_positions(np.zeros(len(asset_names)))\nselected_asset_dof=asset_names.index(asset_joint_name)")
  return source
 plant.adapt_loader_source=adapt
