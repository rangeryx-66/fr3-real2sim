"""Metric, provenance-preserving URDF from inferred ArtGS parts and motion."""
import json,shutil,hashlib
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET


def attach_observations(capture_root, output):
    """Refresh completed collection records without retraining or altering mesh/URDF."""
    root=Path(capture_root);out=Path(output)
    capture=json.loads((root/'multistate_capture.json').read_text())
    effort=root/'effort_profile.json' if (root/'effort_profile.json').exists() else root/'effort_summary.json'
    measured=json.loads(effort.read_text()) if effort.exists() else {'status':'UNAVAILABLE','measurement_completed':False}
    (out/'effort_profile.json').write_text(json.dumps(measured,indent=2))
    for name in ('effort_samples.jsonl','effort_segments.json','effort_sensor_capability.json','effort_profile_object_evaluation.json'):
        if (root/name).exists():shutil.copy2(root/name,out/name)
    (out/'state_observations').mkdir(exist_ok=True)
    (out/'state_observations'/'multistate_capture.json').write_text(json.dumps(capture,indent=2))
    for state in capture['states']:
        for view in state['views']:
            target=out/'state_observations'/view['directory'];target.parent.mkdir(parents=True,exist_ok=True)
            if not target.exists():target.symlink_to((root/view['directory']).resolve(),target_is_directory=True)
    (out/'state_observations'/'PORTABILITY.txt').write_text('State directories reference durable source observations via absolute symlinks. Use tar --dereference when transferring this twin package.\n')


def write(reconstruction, input_dir, capture_root, output):
    import trimesh
    src=Path(reconstruction);inp=Path(input_dir);root=Path(capture_root);out=Path(output);parts=out/'reconstructed_parts';parts.mkdir(parents=True,exist_ok=True)
    audit=json.loads((inp/'input_provenance.json').read_text());motion=json.loads((src/'motion_inferred.json').read_text());capture=json.loads((root/'multistate_capture.json').read_text())
    center=np.array(audit['normalization']['world_center_m']);scale=audit['normalization']['world_scale_m'];j=motion['joints'][0]
    axis=np.asarray(j['axis_direction'],dtype=float);axis/=np.linalg.norm(axis);origin=np.asarray(j['axis_position'])*scale
    kind={'r':'revolute','p':'prismatic'}[j['type']]
    theta=float(np.asarray(j['theta']).reshape(-1)[0])
    low,high=0.,float(np.deg2rad(abs(theta))) if kind=='revolute' else float(abs(theta)*scale)
    if not np.isfinite(high) or high<=0:raise RuntimeError('INFERRED_ZERO_OR_INVALID_MOTION')
    robot=ET.Element('robot',name='artgs_'+capture['object_id']);meshes=[]
    def vec(x):return ' '.join(f'{float(v):.10g}' for v in x)
    for i in range(2):
        mesh=trimesh.load(src/f'part_{i}.ply',force='mesh',process=False)
        if not len(mesh.faces):raise RuntimeError('EMPTY_RECONSTRUCTED_PART')
        mesh.vertices=np.asarray(mesh.vertices)*scale-(origin if i else 0);filename=f'reconstructed_parts/part_{i}.obj';mesh.export(out/filename)
        link=ET.SubElement(robot,'link',name=f'part_{i}')
        # Simulation preview inertial prior only, not an inferred real mass/COM.
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',xyz=vec(mesh.centroid),rpy='0 0 0');ET.SubElement(inertial,'mass',value='1')
        inertia=np.diag(np.maximum(np.sum(mesh.extents**2)-mesh.extents**2,1e-4)/12)
        ET.SubElement(inertial,'inertia',ixx=str(inertia[0,0]),iyy=str(inertia[1,1]),izz=str(inertia[2,2]),ixy='0',ixz='0',iyz='0')
        for tag in ['visual','collision']:
            item=ET.SubElement(link,tag);ET.SubElement(item,'origin',xyz='0 0 0',rpy='0 0 0');g=ET.SubElement(item,'geometry');ET.SubElement(g,'mesh',filename=filename,scale='1 1 1')
        meshes.append({'part':i,'file':filename,'vertices':len(mesh.vertices),'faces':len(mesh.faces),'bounds_link_m':mesh.bounds.tolist(),'sha256':hashlib.sha256((out/filename).read_bytes()).hexdigest()})
    joint=ET.SubElement(robot,'joint',name='inferred_joint',type=kind);ET.SubElement(joint,'parent',link='part_0');ET.SubElement(joint,'child',link='part_1');ET.SubElement(joint,'origin',xyz=vec(origin),rpy='0 0 0');ET.SubElement(joint,'axis',xyz=vec(axis));ET.SubElement(joint,'limit',lower=str(low),upper=str(high),effort='1',velocity='.2')
    ET.indent(robot);ET.ElementTree(robot).write(out/'reconstructed.urdf',encoding='utf-8',xml_declaration=True)
    states=[capture['states'][i]['estimated_articulation_state'] for i in audit['states']]
    attach_observations(root,out)
    update={'backend':'ArtGS official per-scene optimization','backend_commit':'7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a','asset_id':capture['object_id'],'GT_mesh_or_axis_substitution':False,'normalization':audit['normalization'],'T_world_root':np.block([[np.eye(3),center[:,None]],[np.zeros((1,3)),np.ones((1,1))]]).tolist(),'inferred_joint':{'type':kind,'axis_parent':axis.tolist(),'origin_parent_m':origin.tolist(),'motion_between_input_states':high,'uncertainty':'not calibrated; geometry/output quality must be independently validated'},'observed_range_estimated':{'units':capture['units'],'range':[min(states),max(states)],'full_joint_limits_known':False},'URDF_limit_semantics':'bounded preview range from inferred state-pair motion; not full asset joint limit','effort_attachment':'effort_profile.json; NOT mapped to friction','inertial_provenance':'neutral 1kg preview prior with geometry extent inertia, NOT inferred physics','assembly':'root frame aligned world axes at shared visual bbox center; moving mesh recentered by inferred parent joint origin, preserving reconstructed initial assembly','parts':meshes,'initial_reference_asset_overwritten':False}
    (out/'twin_update.json').write_text(json.dumps(update,indent=2));return update
