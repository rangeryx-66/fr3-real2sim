"""One registered nonvertical-hinge input; no grasp pose or physics trial generation."""
import copy, json, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from articulated_demo.kinematics import transform
from articulated_demo.fixture_geometry import fixture_box
from piper_mobile_demo.model import geometry
from xarm7_native_gripper import configuration
from xarm7_native_planning import NativeModel
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/xarm_large_range_torque_v1_20261017'


def prepare(asset_id):
    prepared=OUT/'prepared'/asset_id;asset=OUT/'asset';asset.mkdir(exist_ok=True)
    m=json.loads((prepared/'manifest.json').read_text());root=ET.parse(prepared/'urdf'/f'{asset_id}.urdf').getroot()
    # Same generic contact-body mapping repair as earlier transfer inputs.
    joint=root.find("joint[@name='%s']"%m['joint_name']);child=joint.find('child').get('link')
    descendants={child}
    for _ in root.findall('joint'):
        descendants|={j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in descendants}
    bodies=[l.get('name') for l in root.findall('link') if l.get('name') in descendants and l.findall('visual')]
    if len(bodies)!=1:raise ValueError('Ambiguous moving contact body; inspect input before initialization')
    selection=dict(asset_id=asset_id,mesh=m['grasp_mesh_stems'][0],handle_link=bodies[0],moving_link=child,joint_name=m['joint_name'])
    m.update(door_link=bodies[0],object_name='Nonvertical articulated cabinet',interaction_geometry=dict(method='moving body identity for safety only; visual grasp input',selection=selection,pieces=[]))
    for d in ['urdf','meshes','collision']:
        if not (asset/d).exists():(asset/d).symlink_to(prepared/d,target_is_directory=True)
    (asset/'manifest.json').write_text(json.dumps(m,indent=2))
    original=ROOT/'results/native_g1_agent_transfer_20261015/46427'
    source=json.loads((original/'source/report.json').read_text());source.update(asset_id=asset_id,saved_grasp_poses=[],task='Fresh Agent wrist geometry / physical large-range bidirectional interaction')
    src=OUT/'source';src.mkdir(exist_ok=True);(src/'report.json').write_text(json.dumps(source,indent=2))
    model=NativeModel(None,asset,source);export=dict(schema='authored-closed-geometry-offline/v1',not_native_PhysX_cooking=True,shapes=[])
    for l in root.findall('link'):
        name=l.get('name');T=model.asset_T@model.asset.root_to_link(name,{m['joint_name']:0.})
        for col in l.findall('collision'):
            h=geometry(col,prepared/'urdf').convex_hull;stem=Path(col.find('geometry/mesh').get('filename')).stem
            export['shapes'].append(dict(path=f'/World/articulated_pose/asset/Geometry/{name}/{stem}'.replace('-',''),type='Mesh',rigid_body_path=f'/World/articulated_pose/asset/{name}',world_transform=T.tolist(),rigid_body_world_transform=T.tolist(),convexes=[dict(vertices=h.vertices.tolist(),polygons=h.faces.tolist())]))
    fix=fixture_box(m,model.asset,model.asset_urdf,model.asset_T[:3,:3],model.asset_T[:3,3],source['asset_installation']['fixture_height_m']);T=transform(fix['center']);T[:3,:3]=Rotation.from_quat(np.roll(fix['quaternion_wxyz'],-1)).as_matrix()
    for name,h,X in [('fixture',trimesh.creation.box(fix['size']),T),('table',trimesh.creation.box([.9,.9,.05]),transform([.5,0,-.025]))]:
        export['shapes'].append(dict(path='/World/'+name,type='Mesh',rigid_body_path='/World/'+name,world_transform=X.tolist(),rigid_body_world_transform=X.tolist(),convexes=[dict(vertices=h.vertices.tolist(),polygons=h.faces.tolist())]))
    (OUT/'authored_offline_export.json').write_text(json.dumps(export));base=source['robot_base_pose'];home=source['home_q']
    (OUT/'BOOTSTRAP_NO_GRASP.json').write_text(json.dumps(dict(trial_candidates=[dict(base=base,T=np.eye(4).tolist(),preplan=[home],approach=[home])]),indent=2))
    (OUT/'BOOTSTRAP_NO_PATH.json').write_text(json.dumps(dict(base=base,goal_state=float(np.deg2rad(15)),path=[],end_retreat=None),indent=2))
    job=json.loads((ROOT/'results/xarm_hybrid_v0_20261017/job.json').read_text())
    job.update(asset_id=asset_id,identification_asset=asset_id,asset_root=str(asset),source=str(src),output=str(OUT/'episode'),plan=str(OUT/'BOOTSTRAP_NO_GRASP.json'),whole_task_plan=str(OUT/'BOOTSTRAP_NO_PATH.json'),collision_export=str(OUT/'authored_offline_export.json'),episode_id='large_range_torque_v1_'+asset_id,grasp_geometry_candidates=[selection],authored_target_rad=m['source_joint_limits_rad']['upper'])
    job['skill_inputs']['articulation_model']=dict(type='known_revolute',joint=m['joint_name'],axis=joint.find('axis').get('xyz'),authored_limits=m['source_joint_limits_rad'],urdf_origin=joint.find('origin').attrib,origin_frame='parent',axis_frame='joint')
    (OUT/'INPUT_JOB.json').write_text(json.dumps(job,indent=2))
    axis=model.asset_T[:3,:3]@model.asset.root_to_link(child,{})[:3,:3]@np.fromstring(joint.find('axis').get('xyz'),sep=' ')
    (OUT/'INPUT_AUDIT.json').write_text(json.dumps(dict(asset=asset_id,axis_world=axis.tolist(),moving_contact_body=bodies[0],registered_station=base,no_saved_grasp_or_view=True,no_physics_started=True),indent=2))
    print(json.dumps(dict(asset=asset_id,axis_world=axis.tolist(),base=base)))

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--asset',required=True);prepare(p.parse_args().asset)
