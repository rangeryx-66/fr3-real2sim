"""Deterministic dataset inventory and geometric handle ranking (not grasp acceptance).

Uses transformed visual AND authored collision meshes. Native cooked/raw/closure
checks remain mandatory; these ray measurements cannot certify a real grasp.
"""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import geometry
from articulated_demo.kinematics import URDFChain

def load(urdf):
    root=ET.parse(urdf).getroot();chain=URDFChain(urdf);pieces=[]
    for link in root.findall('link'):
        for kind in ('visual','collision'):
            for item in link.findall(kind):
                mesh=geometry(item,urdf.parent);mesh.apply_transform(chain.root_to_link(link.get('name'),{}))
                spec=item.find('geometry/mesh')
                pieces.append((link.get('name'),kind,Path(spec.get('filename')).stem if spec is not None else 'primitive',mesh))
    return root,pieces

def hits(mesh,point,normal):
    extent=np.linalg.norm(mesh.extents)+1
    triangles=mesh.triangles;origin=point+normal*extent;direction=-normal
    e1=triangles[:,1]-triangles[:,0];e2=triangles[:,2]-triangles[:,0]
    h=np.cross(np.broadcast_to(direction,e2.shape),e2);det=np.einsum('ij,ij->i',e1,h)
    valid=abs(det)>1e-12;inv=np.zeros_like(det);inv[valid]=1/det[valid]
    s=origin-triangles[:,0];u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1)
    v=inv*(q@direction);t=inv*np.einsum('ij,ij->i',e2,q)
    keep=valid&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)&(t>=0)
    return sorted(set(np.round(extent-t[keep],9)),reverse=True)

def scan(directory):
    rows=[];inventory=[]
    for manifest in sorted(directory.glob('*/manifest.json')):
        meta=json.loads(manifest.read_text());urdf=manifest.parent/'urdf'/f"{meta['asset_id']}.urdf"
        if not urdf.exists():continue
        root,pieces=load(urdf);joints=[j for j in root.findall('joint') if j.get('type') in ('revolute','prismatic')]
        inventory.append({'directory':manifest.parent.name,'asset_id':meta['asset_id'],'joints':[{'name':j.get('name'),'type':j.get('type')} for j in joints]})
        for joint in joints:
            moving={joint.find('child').get('link')}
            while True:
                nxt=moving|{j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in moving}
                if nxt==moving:break
                moving=nxt
            visuals=[p for p in pieces if p[0] in moving and p[1]=='visual']
            if not visuals:continue
            panel=max(visuals,key=lambda p:np.prod(np.sort(p[3].extents)[-2:]))
            # PCA panel normal, with sign determined per protruding part.
            _,_,v=np.linalg.svd(np.vstack([p[3].vertices for p in visuals])-np.vstack([p[3].vertices for p in visuals]).mean(0),full_matrices=False);normal=v[-1]
            for link,kind,stem,mesh in visuals:
                if stem==panel[2]:continue
                _,_,hv=np.linalg.svd(mesh.vertices-mesh.vertices.mean(0),full_matrices=False)
                axis=hv[0];axis-=normal*(axis@normal)
                if np.linalg.norm(axis)<1e-6:continue
                axis/=np.linalg.norm(axis);n=normal.copy()
                if (mesh.vertices.mean(0)-panel[3].vertices.mean(0))@n<0:n=-n
                closing=np.cross(n,axis);closing/=np.linalg.norm(closing)
                center=mesh.bounds.mean(0);dims=np.ptp(mesh.vertices@np.column_stack((axis,closing,n)),axis=0)
                other=[p[3] for p in pieces if p[1]=='collision' and not (p[0]==link and p[2].split('.obj')[0].endswith(stem))]
                own=[p[3] for p in pieces if p[1]=='collision' and p[0]==link and p[2].split('.obj')[0].endswith(stem)]
                records=[]
                for f in (-.3,0,.3):
                    anchor=center+axis*dims[0]*f;visual_hits=hits(mesh,anchor,n)
                    if len(visual_hits)<2:continue
                    # First bar encountered from outside; disconnected supports
                    # must not be mistaken for a central bar's back surface.
                    front,back=visual_hits[:2];panel_hits=[x for p in visuals if p[2]!=stem for x in hits(p[3],anchor,n)]
                    obstacles=[x for m in other for x in hits(m,anchor,n) if x<=front]
                    cooked_proxy=[x for m in own for x in hits(m,anchor,n)]
                    cb=min(cooked_proxy) if cooked_proxy else back
                    collision_gap=cb-max(obstacles,default=cb)
                    visual_gap=back-max([x for x in panel_hits if x<=front],default=back)
                    patch=abs((mesh.triangles_center-anchor)@axis)<.02543/2
                    projection=mesh.face_normals@closing
                    areas=[float(mesh.area_faces[patch&(projection> .8)].sum()),float(mesh.area_faces[patch&(projection< -.8)].sum())]
                    records.append({'opposing_patch_areas_m2':areas,'fraction':f,'anchor_root_m':(anchor+n*(front+back)/2).tolist(),'bar_depth_m':front-back,'visual_rear_gap_m':visual_gap,'collision_rear_gap_m':collision_gap})
                gap=min([r['collision_rear_gap_m'] for r in records],default=0)
                reasons=[]
                if dims[0]<.02543:reasons.append('SHORTER_THAN_OFFICIAL_PAD')
                if dims[1]>.10:reasons.append('EXCEEDS_OFFICIAL_OPENING')
                if dims[0]/max(dims[1],1e-9)<1.6:reasons.append('NOT_BAR_LIKE')
                if gap<.00501:reasons.append('INSUFFICIENT_REAR_HALF_PAD_DEPTH_CLEARANCE')
                if min([r['visual_rear_gap_m'] for r in records],default=0)<.00501:reasons.append('NOT_A_PROTRUDING_BAR')
                if not own:reasons.append('NO_IDENTIFIABLE_AUTHORED_HANDLE_COLLIDER')
                rows.append({'asset_id':meta['asset_id'],'asset_root':str(manifest.parent),'joint_name':joint.get('name'),'joint_type':joint.get('type'),'moving_link':joint.find('child').get('link'),'handle_link':link,'mesh':stem,'panel_mesh':panel[2],'axis_root':axis.tolist(),'outward_normal_root':n.tolist(),'dimensions_m':dims.tolist(),'sections':records,'minimum_collision_rear_gap_m':gap,'screen_rejections':reasons,'score':(100 if joint.get('type')=='prismatic' else 0)+gap*100+min(dims[0],.15)-len(reasons)*10})
    rows.sort(key=lambda r:r['score'],reverse=True)
    return {'inventory':inventory,'ranking':rows,'geometry_only':True,'rear_gap_definition':'signed axial ray separation, not Euclidean clearance or a grasp feasibility proof; three central sections only','acceptance':'actual PhysX closure + native ownership + cooked AND official raw audit mandatory','ranking_constants_source':'unchanged official PiPER aperture 100mm, distal pad 25.43 x 10.02mm; screening only'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--source-dataset',type=Path);a=p.parse_args();result=scan(a.prepared);
    if a.source_dataset:
        result['source_inventory']=[{'asset_id':f.stem,'joints':[{'name':j.get('name'),'type':j.get('type')} for j in ET.parse(f).getroot().findall('joint') if j.get('type')!='fixed']} for f in sorted((a.source_dataset/'urdf').glob('*.urdf'))]
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));print(json.dumps({'inventory':result['inventory'],'best':result['ranking'][:12]},indent=2))
if __name__=='__main__':main()
