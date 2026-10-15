"""Prepare registered metadata/scene inputs only; never plan a grasp or view."""
import copy, hashlib, json, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from xarm7_native_gripper import ROOT, configuration
from xarm7_native_planning import NativeModel
from piper_mobile_demo.model import geometry
from articulated_demo.fixture_geometry import fixture_box
from articulated_demo.kinematics import transform

if __name__ == '__main__':
    out=ROOT/'results/native_g1_agent_transfer_20261015'
    registration=json.loads((ROOT/'docs/experiments/native_g1_agent_transfer_20261015/REGISTERED_CASES.json').read_text())
    template=json.loads((ROOT/'results/native_g1_agent_manipulation_20261015/job.json').read_text())
    baseline={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in registration['frozen_source_sha256']}
    if baseline!=registration['frozen_source_sha256']:raise SystemExit('Frozen baseline differs before any scene')
    for row in registration['cases']:
        aid=row['asset_id'];case=out/aid;asset=case/'asset';asset.mkdir(parents=True,exist_ok=True)
        prepared=out/'geometry_assets'/aid;m=json.loads((prepared/'manifest.json').read_text())
        # Generic input normalization: active joint's visual descendant, not importer l_1 default.
        selection=dict(asset_id=aid,mesh=row['handle_mesh'],handle_link=row['handle_link'],moving_link=row['moving_link'],joint_name=row['joint'])
        m.update(door_link=row['moving_link'],moving_link=row['moving_link'],object_name='Registered hinged cabinet',interaction_geometry=dict(method='registered collider identity only; no grasp geometry',selection=selection,pieces=[]))
        for directory in ['urdf','meshes','collision']:
            p=asset/directory
            if not p.exists():p.symlink_to(prepared/directory,target_is_directory=True)
        (asset/'manifest.json').write_text(json.dumps(m,indent=2))
        src=case/'source';src.mkdir(exist_ok=True)
        source=dict(schema='registered-agent-wrist-transfer-source/v1',asset_id=aid,asset_installation=registration['installation'],home_q=configuration()['home_q'],camera_offset_m=[.2,-.75,.36],robot_base_pose=registration['registered_common_base_xyzyawdeg'],support_bottom_z_m=-.56,saved_grasp_poses=[],task='Fresh Agent wrist visual grasp / original 1s hold / +3deg opening')
        (src/'report.json').write_text(json.dumps(source,indent=2))
        model=NativeModel(None,asset,source);tree=ET.parse(model.asset_urdf).getroot()
        export=dict(schema='authored-closed-geometry-offline/v1',not_native_PhysX_cooking=True,shapes=[])
        for link in tree.findall('link'):
            n=link.get('name');T=model.asset_T@model.asset.root_to_link(n,{m['joint_name']:0.})
            for col in link.findall('collision'):
                hull=geometry(col,model.asset_urdf.parent).convex_hull;mesh=Path(col.find('geometry/mesh').get('filename')).stem
                export['shapes'].append(dict(path=f'/World/articulated_pose/asset/Geometry/{n}/{mesh}'.replace('-',''),type='Mesh',rigid_body_path=f'/World/articulated_pose/asset/{n}',world_transform=T.tolist(),rigid_body_world_transform=T.tolist(),convexes=[dict(vertices=hull.vertices.tolist(),polygons=hull.faces.tolist())]))
        fix=fixture_box(m,model.asset,model.asset_urdf,model.asset_T[:3,:3],model.asset_T[:3,3],.04);T=transform(fix['center']);T[:3,:3]=Rotation.from_quat(np.roll(fix['quaternion_wxyz'],-1)).as_matrix()
        for n,h,X in [('fixture',trimesh.creation.box(fix['size']),T),('table',trimesh.creation.box([.9,.9,.05]),transform([.5,0,-.025]))]:
            export['shapes'].append(dict(path='/World/'+n,type='Mesh',rigid_body_path='/World/'+n,world_transform=X.tolist(),rigid_body_world_transform=X.tolist(),convexes=[dict(vertices=h.vertices.tolist(),polygons=h.faces.tolist())]))
        (case/'authored_offline_export.json').write_text(json.dumps(export))
        base=registration['registered_common_base_xyzyawdeg'];home=source['home_q']
        (case/'BOOTSTRAP_NO_GRASP.json').write_text(json.dumps(dict(trial_candidates=[dict(base=base,T=np.eye(4).tolist(),preplan=[home],approach=[home])]),indent=2))
        (case/'BOOTSTRAP_NO_PATH.json').write_text(json.dumps(dict(base=base,goal_state=float(np.deg2rad(3)),path=[],end_retreat=None),indent=2))
        job=copy.deepcopy(template);job.update(asset_id=aid,asset_root=str(asset),source=str(src),output=str(case/'episode'),plan=str(case/'BOOTSTRAP_NO_GRASP.json'),whole_task_plan=str(case/'BOOTSTRAP_NO_PATH.json'),collision_export=str(case/'authored_offline_export.json'),episode_id='native_g1_agent_transfer_'+aid,grasp_geometry_candidates=[selection],authored_target_rad=m['source_joint_limits_rad']['upper'],classification='CODEX_AGENT_WRIST / RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION')
        joint=tree.find("joint[@name='%s']"%m['joint_name'])
        job['skill_inputs']['articulation_model']=dict(type='known_revolute',joint=m['joint_name'],axis=joint.find('axis').get('xyz'),authored_limits=m['source_joint_limits_rad'],urdf_origin=joint.find('origin').attrib,origin_frame='parent',axis_frame='joint')
        (case/'job.json').write_text(json.dumps(job,indent=2));print('PREPARED_NO_GRASP_OR_VIEW_PLANNING',aid,flush=True)
    (out/'FROZEN_SOURCE_CHECK.json').write_text(json.dumps(dict(reference='34607b4',sha256=baseline,all_equal=True),indent=2))
