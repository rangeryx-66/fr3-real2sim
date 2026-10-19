"""Prephysical station/IK eligibility and explicit scene inputs; no saved grasp.

The diagnostic mesh-centre IK below is NEVER an executable grasp. Its only
output used online is the declared known-model base station, as in V6.
"""
import argparse,copy,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def prepare(root):
    from xarm7_native_gripper import configuration
    from xarm7_native_planning import NativeModel
    from cross_robot_adapter import RobotPinkIK
    from piper_mobile_demo.model import geometry
    from xarm7_native_planning import scene_factory
    from revolute10_fixed import authored_handle_colliders
    cases=json.loads((root/'preflight/PROVISIONAL_CASES.json').read_text())['cases']
    template=json.loads((ROOT/'results/large_range_v6_20261019/episode_00/job.json').read_text())
    source_template=json.loads((Path(template['source'])/'report.json').read_text());config=configuration();solver=RobotPinkIK(config)
    results=[]
    for row in cases:
        aid=row['asset_id'];case=root/'inputs'/aid;asset=case/'asset';source=case/'source';asset.mkdir(parents=True,exist_ok=True);source.mkdir(exist_ok=True)
        original=Path(row['asset_root']);manifest=json.loads((original/'manifest.json').read_text());kind=row['joint_type']
        joint=ET.parse(original/'urdf'/f'{aid}.urdf').getroot().find("joint[@name='%s']"%row['joint_name'])
        mesh=manifest['grasp_mesh_stems'][0]
        # Original preparation may select a shelf as generic door_link. Resolve
        # the selected mesh's actual visual-bearing rigid link, by identity only.
        tree=ET.parse(original/'urdf'/f'{aid}.urdf').getroot()
        matches=[link.get('name') for link in tree.findall('link') for vis in link.findall('visual') if Path(vis.find('geometry/mesh').get('filename')).stem==mesh]
        if len(set(matches))!=1:raise ValueError('Ambiguous registered contact-bearing link '+aid)
        handle_link=matches[0];selection=dict(asset_id=aid,mesh=mesh,handle_link=handle_link,moving_link=manifest['moving_link'],joint_name=row['joint_name'])
        manifest.update(joint_type=kind,door_link=handle_link,joint_limits_native=row['limits_native'],joint_coordinate_unit=row['unit'],interaction_geometry=dict(method='registered collider identities only; no online geometry',selection=selection,pieces=[]))
        for directory in ['urdf','meshes','collision']:
            dst=asset/directory
            if not dst.exists():dst.symlink_to((original/directory).resolve(),target_is_directory=True)
        (asset/'manifest.json').write_text(json.dumps(manifest,indent=2))
        src=copy.deepcopy(source_template);src.update(asset_id=aid,saved_grasp_poses=[],task='V7 fresh wrist geometry, SI articulated interaction',home_q=config['home_q'])
        (source/'report.json').write_text(json.dumps(src,indent=2))
        model=NativeModel(None,asset,src);points=[]
        export=dict(shapes=[])
        for link in tree.findall('link'):
            n=link.get('name');X=model.asset_T@model.asset.root_to_link(n,{})
            for col in link.findall('collision'):
                hull=geometry(col,original/'urdf').convex_hull;stem=Path(col.find('geometry/mesh').get('filename')).stem
                export['shapes'].append(dict(path=f'/World/articulated_pose/asset/{n}/{stem}'.replace('-',''),type='Mesh',rigid_body_path=f'/World/articulated_pose/asset/{n}',world_transform=X.tolist(),rigid_body_world_transform=X.tolist(),convexes=[dict(vertices=hull.vertices.tolist(),polygons=hull.faces.tolist())]))
        safety=scene_factory(ROOT,export,model,authored_handle_colliders(export,selection))
        for link in tree.findall('link'):
            for vis in link.findall('visual'):
                if Path(vis.find('geometry/mesh').get('filename')).stem==mesh:
                    h=geometry(vis,original/'urdf');h.apply_transform(model.asset_T@model.asset.root_to_link(link.get('name'),{}));points.extend(h.vertices)
        points=np.asarray(points);anchor=(points.min(0)+points.max(0))/2;_,_,V=np.linalg.svd(points-points.mean(0),full_matrices=False);axis=V[0]
        normal=model.asset_T[:3,:3]@np.array([0,0,1.]);axis-=normal*(axis@normal);axis/=np.linalg.norm(axis)
        target=np.eye(4);target[:3,:3]=np.column_stack([axis,np.cross(-normal,axis),-normal]);target[:3,3]=anchor-normal*.025
        base0=np.asarray(src['robot_base_pose']);station=None;audit=[]
        stations=[base0]+[base0+np.array([dx,dy,0,yaw]) for dx,dy,yaw in [(0,-.10,0),(-.10,0,0),(.10,0,0),(0,.10,0),(0,0,-15),(0,0,15)]]
        for base in stations:
            success=False
            home_valid,home_reason=safety.check(model.poses(model.home,base,width=.084),safety.moving_reference,False)
            if not home_valid:
                audit.append(dict(base=base.tolist(),coarse_IK=False,home_collision_valid=False,reason=home_reason));continue
            for roll in [0,180]:
                T=target.copy();T[:3,:3]=T[:3,:3]@Rotation.from_euler('z',roll,degrees=True).as_matrix();q=solver.solve(T,base,starts=2)
                if q is not None and model.margin(q)>.05:success=True;break
            audit.append(dict(base=base.tolist(),coarse_IK=success,home_collision_valid=True))
            if success:station=base.tolist();break
        if station is not None:src['robot_base_pose']=station
        (source/'report.json').write_text(json.dumps(src,indent=2))
        goal=min(row['limits_native']['upper'],np.deg2rad(30) if kind=='revolute' else .15)
        domain=[-.02*goal,1.5*goal]
        units=dict(coordinate='rad',effort='N m',inertial='kg m^2',damping='N m s/rad') if kind=='revolute' else dict(coordinate='m',effort='N',inertial='kg',damping='N s/m')
        job=copy.deepcopy(template)
        for k in ['passive_dataset_output','passive_evaluation_output','private_physics_configuration','deadline_shanghai']:job.pop(k,None)
        job.update(asset_id=aid,asset_root=str(asset),source=str(source),experiment='V7',research_reference='25e2189',runtime_acquisition_budget=4,maximum_agent_operations=40,agent_wait_timeout_s=360,
            articulation_v7=dict(joint_type=kind,limits_native=[row['limits_native'][k] for k in ['lower','upper']],units=units),
            estimation_v7=dict(joint_type=kind,domain_native=domain,units=units,parameter_scales=[1.,.25,.045] if kind=='revolute' else [5.,10.,1.],intervals=[2,4,8],smoothness=[.0001,.001,.01],ridge=.001),
            benchmark_goal_rad=goal,benchmark_goal_native=goal,authored_target_rad=row['limits_native']['upper'],skill=dict(joint_type=kind),grasp_geometry_candidates=[selection],
            storage_target_bytes=100000000,scientific_blinding='private scene generator only; no hidden dynamics/seed in Agent or estimator inputs',
            episode_wall_budget_s=3600.,episode_sim_budget_s=2400.,safe_exit_wall_reserve_s=300.,safe_exit_sim_reserve_s=90.)
        (case/'job_template.json').write_text(json.dumps(job,indent=2))
        results.append(dict(asset_id=aid,joint_type=kind,mesh=mesh,handle_link=handle_link,station=station,coarse_IK=station is not None,audit=audit,
            saved_grasp_input=False,saved_viewpoints=False,GT_station_dependency=True,status='NATIVE_COLLISION_AND_WRIST_VIEW_PREFLIGHT_PENDING'))
    (root/'preflight/STATION_IK.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();prepare(a.root.resolve())
