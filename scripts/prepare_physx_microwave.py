"""Prepare the existing PhysX-Mobility microwave without replacing its joints."""
import argparse,hashlib,json,shutil,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from articulated_demo.kinematics import URDFChain


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--asset-id',default='7236')
    p.add_argument('--height-m',type=float,default=.30)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();source=args.source/'urdf'/f'{args.asset_id}.urdf'
    meta=args.source/'finaljson'/f'{args.asset_id}.json';root=ET.parse(source).getroot()
    chain=URDFChain(source)
    movable=[j for j in root.findall('joint') if j.get('type')=='revolute']
    if len(movable)!=1:raise ValueError('minimal microwave requires one revolute door joint')
    joint=movable[0];moving=joint.find('child').get('link')
    # Resolve descendants from the immutable source link tree.
    moving_links={moving}
    while True:
        children={j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in moving_links}
        if children<=moving_links:break
        moving_links|=children
    pieces=[]
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            spec=visual.find('geometry/mesh');path=(source.parent/spec.get('filename')).resolve()
            mesh=trimesh.load(path,force='mesh',process=False)
            scale=np.fromstring(spec.get('scale','1 1 1'),sep=' ');mesh.apply_scale(scale)
            origin=visual.find('origin');T=np.eye(4)
            if origin is not None:
                T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            mesh.apply_transform(chain.root_to_link(link.get('name'),{})@T)
            pieces.append((link.get('name'),path.stem,mesh,visual))
    all_bounds=np.stack((np.min([m.bounds[0] for _,_,m,_ in pieces],0),np.max([m.bounds[1] for _,_,m,_ in pieces],0)))
    factor=args.height_m/(all_bounds[1,1]-all_bounds[0,1])
    target=args.output.resolve();(target/'urdf').mkdir(parents=True,exist_ok=True)
    shutil.copytree(args.source/'partseg'/args.asset_id,target/'meshes',dirs_exist_ok=True)
    (target/'collision').mkdir(exist_ok=True)
    repairs=[];collisions=0;bounds_by_link={}
    for link in root.findall('link'):
        name=link.get('name');local=[m.copy() for n,_,m,_ in pieces if n==name]
        if local:
            for m in local:m.apply_transform(np.linalg.inv(chain.root_to_link(name,{})))
            bounds=np.stack((np.min([m.bounds[0] for m in local],0),np.max([m.bounds[1] for m in local],0)))*factor
            bounds_by_link[name]=bounds.tolist()
        for old in list(link.findall('collision')):link.remove(old)
        for n,stem,mesh,visual in [r for r in pieces if r[0]==name]:
            spec=visual.find('geometry/mesh');original=(source.parent/spec.get('filename')).resolve()
            copied=target/'meshes/objs'/original.name
            lines=copied.read_text().splitlines();vt=sum(l.startswith('vt ') for l in lines)
            if vt==0 and any('/' in token for l in lines if l.startswith('f ') for token in l.split()[1:]):
                lines=[('f '+' '.join(t.split('/')[0] for t in l.split()[1:])) if l.startswith('f ') else l for l in lines if not l.startswith('vn ')]
                copied.write_text('\n'.join(lines)+'\n');repairs.append(stem)
            spec.set('filename',str(Path('../meshes/objs')/original.name))
            prior=np.fromstring(spec.get('scale','1 1 1'),sep=' ');spec.set('scale',' '.join(map(str,prior*factor)))
            origin=visual.find('origin')
            if origin is not None:origin.set('xyz',' '.join(map(str,np.fromstring(origin.get('xyz','0 0 0'),sep=' ')*factor)))
            localmesh=mesh.copy();localmesh.apply_transform(np.linalg.inv(chain.root_to_link(name,{})));localmesh.apply_scale(factor)
            try:hull=localmesh.convex_hull
            except Exception:continue
            if hull.volume<1e-12:continue
            file=target/'collision'/f'{name}_{stem}.stl';hull.export(file,file_type='stl_ascii')
            col=ET.SubElement(link,'collision');geo=ET.SubElement(col,'geometry');ET.SubElement(geo,'mesh',filename=str(Path('../collision')/file.name),scale='1 1 1');collisions+=1
        inertial=link.find('inertial')
        if inertial is not None and name in bounds_by_link:
            bounds=np.asarray(bounds_by_link[name]);size=bounds[1]-bounds[0];mass=float(inertial.find('mass').get('value'))
            origin=inertial.find('origin');origin.set('xyz',' '.join(map(str,bounds.mean(0))));origin.set('rpy','0 0 0')
            inertia=mass/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])
            inertial.find('inertia').attrib.update(ixx=str(max(inertia[0],1e-6)),iyy=str(max(inertia[1],1e-6)),izz=str(max(inertia[2],1e-6)),ixy='0',ixz='0',iyz='0')
    for j in root.findall('joint'):
        origin=j.find('origin')
        if origin is not None:origin.set('xyz',' '.join(map(str,np.fromstring(origin.get('xyz','0 0 0'),sep=' ')*factor)))
    prepared=target/'urdf'/f'{args.asset_id}.urdf';ET.indent(root);ET.ElementTree(root).write(prepared,encoding='utf-8',xml_declaration=True)
    moving_pieces=[m for name,_,m,_ in pieces if name in moving_links];static_pieces=[m for name,_,m,_ in pieces if name not in moving_links]
    def bounds(items):return np.stack((np.min([m.bounds[0] for m in items],0),np.max([m.bounds[1] for m in items],0))).tolist()
    door_pieces=[(stem,m) for n,stem,m,_ in pieces if n in moving_links]
    # Geometry metadata is only for camera placement and post-hoc mask evaluation.
    # The grasp input still comes from RGB + SAM3; no mesh-derived grasp is used.
    handle=max(door_pieces,key=lambda r:r[1].bounds[1,2])
    data={'source_url':'https://huggingface.co/datasets/Caoza/PhysX-Mobility','license':'CC-BY-NC-4.0',
        'asset_id':args.asset_id,'object_name':'Microwave','prepared_urdf':str(prepared),
        'source_urdf_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'source_metadata_sha256':hashlib.sha256(meta.read_bytes()).hexdigest(),
        'scale_source_to_meters':factor,'prepared_height_m':args.height_m,'source_up_axis':'+Y',
        'static_source_bounds':bounds(static_pieces),'moving_source_bounds':bounds(moving_pieces),
        'moving_links':sorted(moving_links),'moving_link':moving,'door_link':'l_1',
        'joint_name':joint.get('name'),'source_joint_axis':joint.find('axis').get('xyz'),
        'source_joint_limits_rad':{k:float(joint.find('limit').get(k)) for k in ('lower','upper')},
        'grasp_mesh_stems':[handle[0]],'grasp_mesh_bounds_source':handle[1].bounds.tolist(),
        'handle_mesh_selection':'provisional most protruding door mesh; verify visually; evaluation/camera only',
        'uv_repaired_visuals':repairs,'collision_mesh_count':collisions,
        'physics_note':'source masses preserved; link COM and inertia from uniform box approximation; not measured hardware properties'}
    digest=hashlib.sha256(prepared.read_bytes())
    for path in sorted((target/'meshes/objs').glob('*.obj')):digest.update(path.read_bytes())
    data['prepared_geometry_sha256']=digest.hexdigest()
    (target/'manifest.json').write_text(json.dumps(data,indent=2));print(json.dumps(data,indent=2))


if __name__=='__main__':main()
