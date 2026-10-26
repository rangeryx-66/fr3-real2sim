"""Prepare the two preregistered PartNet joints through existing V7 geometry IO.

Offline assembly only; never chooses an asset/grasp based on a physical result.
The empty-root basis change preserves all relative physical joints. Other DOFs
are retained, without locking or resetting. Contact identity is semantic only.
"""
import argparse,copy,hashlib,json,shutil,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def prepare(raw,output,registration):
    from articulated_demo.kinematics import URDFChain
    from piper_mobile_demo.model import geometry
    root=Path(__file__).resolve().parents[1];reg=json.loads(Path(registration).read_text())
    output=Path(output);output.mkdir(exist_ok=False,parents=True)
    records=[]
    for case in reg['fixed_cases']:
        aid=case['asset_id'];name=case['candidate_id'].split('__',1)[1]
        src=Path(raw)/aid;original=src/'mobility.urdf';tree=ET.parse(original).getroot()
        initial=copy.deepcopy(tree);selected=tree.find("joint[@name='%s']"%name)
        if selected is None or selected.get('type')!=case['joint_type']:raise ValueError('FROZEN_SELECTED_JOINT_MISMATCH')
        child=selected.find('child').get('link');children={j.find('child').get('link')for j in tree.findall('joint')}
        roots={l.get('name')for l in tree.findall('link')}-children
        if len(roots)!=1:raise ValueError('ONE_EMPTY_ROOT_REQUIRED')
        source_root=next(iter(roots));root_link=tree.find("link[@name='%s']"%source_root)
        upstream=[j for j in tree.findall('joint')if j.find('parent').get('link')==source_root]
        if root_link.findall('visual') or root_link.findall('collision') or len(upstream)!=1 or upstream[0].get('type')!='fixed':
            raise ValueError('SOURCE_ROOT_BASIS_NOT_RELOCATABLE_WITHOUT_PHYSICAL_CHANGE')
        # PartNet's empty root turns its native Y-up model into viewer Z-up.
        # Existing V7 asset_T already supplies the Y-up installation transform.
        basis=upstream[0].find('origin');basis_before=dict(basis.attrib)
        basis.set('xyz','0 0 0');basis.set('rpy','0 0 0')
        stage=output/'normalized_source'/aid
        (stage/'urdf').mkdir(parents=True);(stage/'finaljson').mkdir()
        (stage/'partseg'/aid).mkdir(parents=True)
        (stage/'partseg'/aid/'objs').symlink_to((src/'textured_objs').resolve(),target_is_directory=True)
        handle_stems=[];parts=[]
        for link in tree.findall('link'):
            visuals=link.findall('visual')
            if visuals and link.find('inertial') is None:
                inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',xyz='0 0 0',rpy='0 0 0')
                ET.SubElement(inertial,'mass',value='1')
                ET.SubElement(inertial,'inertia',ixx='.01',iyy='.01',izz='.01',ixy='0',ixz='0',iyz='0')
            for visual in visuals:
                mesh=visual.find('geometry/mesh');filename=Path(mesh.get('filename')).name
                mesh.set('filename','../partseg/'+aid+'/objs/'+filename)
                label=visual.get('name','unknown');stem=Path(filename).stem
                parts.append(dict(name=label,obj=[stem]))
                if link.get('name')==child and any(k in label.lower()for k in ('handle','knob')):handle_stems.append(stem)
            for collision in link.findall('collision'):
                mesh=collision.find('geometry/mesh')
                if mesh is not None:mesh.set('filename','../partseg/'+aid+'/objs/'+Path(mesh.get('filename')).name)
        if not handle_stems:raise ValueError('NO_SELECTED_LINK_HANDLE_SEMANTICS; DO_NOT_SWAP_OBJECT')
        normalized=stage/'urdf'/(aid+'.urdf');ET.indent(tree);ET.ElementTree(tree).write(normalized,encoding='utf-8',xml_declaration=True)
        (stage/'finaljson'/(aid+'.json')).write_text(json.dumps(dict(object_name=json.loads((src/'meta.json').read_text()).get('model_cat','PartNetObject'),parts=parts)))
        destination=output/'assets'/aid
        subprocess.run([sys.executable,str(root/'scripts/prepare_physx_microwave.py'),'--source',str(stage),'--asset-id',aid,'--joint-type',case['joint_type'],'--joint-name',name,'--height-m',str(reg['asset_preparation']['height_m']),'--output',str(destination)],check=True,stdout=subprocess.DEVNULL)
        manifest=json.loads((destination/'manifest.json').read_text());prepared=destination/'urdf'/(aid+'.urdf');final=ET.parse(prepared).getroot();collider_count=0
        for link in final.findall('link'):
            if link.get('name')!=child:continue
            for col in link.findall('collision'):
                m=col.find('geometry/mesh');old=prepared.parent/m.get('filename')
                if any(old.stem==child+'_'+stem for stem in handle_stems):
                    new=old.with_name(child+'_targethandle_'+old.name[len(child)+1:]);old.rename(new)
                    m.set('filename','../collision/'+new.name);collider_count+=1
        if collider_count!=len(set(handle_stems)):raise ValueError('SEMANTIC_COLLIDER_GROUP_INCOMPLETE')
        ET.indent(final);ET.ElementTree(final).write(prepared,encoding='utf-8',xml_declaration=True)
        chain=URDFChain(normalized);points=[]
        for link in tree.findall('link'):
            if link.get('name')!=child:continue
            for vis in link.findall('visual'):
                if Path(vis.find('geometry/mesh').get('filename')).stem not in handle_stems:continue
                h=geometry(vis,normalized.parent);h.apply_transform(chain.root_to_link(child,{}));points.extend(h.vertices)
        points=np.asarray(points)
        selection=dict(asset_id=aid,mesh='targethandle',handle_link=child,moving_link=child,joint_name=name)
        manifest.update(door_link=child,grasp_mesh_stems=sorted(set(handle_stems)),grasp_mesh_bounds_source=[points.min(0).tolist(),points.max(0).tolist()],
            interaction_geometry=dict(method='source semantic collider identity group only; fresh wrist supplies all online grasp geometry',selection=selection,pieces=[]),
            source_urdf_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),source_dataset='PartNet-Mobility user selected20',
            source_root_basis_change=basis_before,other_articulations_retained=True,
            baseline_inertial_note='missing source masses initialized1kg per visual-bearing link; existing V7 private positive-density law owns selected plant. Synthetic simulation, not measured object dynamics.')
        (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        # Exact relative joints are preserved except uniform SI scaling by the
        # already registered common height convention; no object-specific cap.
        before={j.get('name'):j for j in initial.findall('joint')};after={j.get('name'):j for j in final.findall('joint')}
        if before.keys()!=after.keys():raise ValueError('JOINT_SET_CHANGED')
        for n,j in before.items():
            a=after[n]
            if j.get('type')!=a.get('type') or j.find('parent').attrib!=a.find('parent').attrib or j.find('child').attrib!=a.find('child').attrib:raise ValueError('JOINT_STRUCTURE_CHANGED')
            if j.find('axis') is not None and j.find('axis').attrib!=a.find('axis').attrib:raise ValueError('JOINT_AXIS_CHANGED')
        records.append(dict(asset_id=aid,target_joint=name,joint_type=case['joint_type'],asset_root=str(destination),selection=selection,
            nonfixed_joints_retained=[j.get('name')for j in final.findall('joint')if j.get('type')!='fixed'],
            source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),prepared_sha256=hashlib.sha256(prepared.read_bytes()).hexdigest(),
            native_handle_collider_count=collider_count,source_root_basis_change_only=True,relative_joint_structure_and_axes_preserved=True,
            scale=manifest['scale_source_to_meters'],limits_native=manifest['joint_limits_native']))
        print('PREPARED_FIXED_CASE',aid,name,flush=True)
    (output/'PREPARATION.json').write_text(json.dumps(dict(cases=records,registration_sha256=hashlib.sha256(Path(registration).read_bytes()).hexdigest(),no_grasp_or_view_import=True),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--registration',type=Path,required=True)
    a=p.parse_args();prepare(a.raw,a.output,a.registration)
