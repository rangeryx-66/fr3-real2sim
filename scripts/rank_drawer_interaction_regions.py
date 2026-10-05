"""New prismatic setup adapter: exterior bar observation, no GT axis inputs.

The revolute baseline's scene-wide panel PCA is ambiguous for box-shaped
drawers: its thinnest dimension may describe the drawer bottom. Here only the
initial visual geometry supplies an exterior normal. Existing proxy builder,
grasp family, controller and collision acceptance remain unchanged.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from rank_prepared_handle_assets import load,hits


def scan(directories):
    rows=[]
    for directory in directories:
        for path in sorted(Path(directory).glob('*/manifest.json')):
            meta=json.loads(path.read_text());urdf=path.parent/'urdf'/f"{meta['asset_id']}.urdf"
            root,pieces=load(urdf)
            joints=[j for j in root.findall('joint') if j.get('type')=='prismatic']
            for joint in joints:
                moving={joint.find('child').get('link')}
                while True:
                    nxt=moving|{j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in moving}
                    if nxt==moving:break
                    moving=nxt
                static=[m for l,k,s,m in pieces if k=='visual' and l not in moving]
                if not static:continue
                center_body=np.vstack([m.vertices for m in static]).mean(0)
                visuals=[x for x in pieces if x[0] in moving and x[1]=='visual']
                for link,kind,stem,mesh in visuals:
                    center=mesh.bounds.mean(0);_,_,V=np.linalg.svd(mesh.vertices-mesh.vertices.mean(0),full_matrices=False);axis=V[0]
                    normal=center-center_body;normal-=axis*(normal@axis)
                    if np.linalg.norm(normal)<.001:continue
                    normal/=np.linalg.norm(normal)
                    # A planar handle's own surface normal is better observed
                    # than a drawer-wide PCA. Sign comes only from visual body.
                    face=V[-1].copy()
                    if face@normal<0:face=-face
                    spread=np.std(mesh.vertices@V.T,axis=0)
                    if spread[2]/max(spread[1],1e-9)<.6:normal=face
                    closing=np.cross(normal,axis);closing/=np.linalg.norm(closing)
                    dims=np.ptp(mesh.vertices@np.column_stack((axis,closing,normal)),axis=0)
                    # Initial-region screening only, never a grasp certification.
                    if dims[0]<.02543 or dims[1]>=.1 or dims[2]>.03 or dims[0]/max(dims[1],1e-6)<1.6:continue
                    sections=[]
                    for fraction in [-.3,0,.3]:
                        anchor=center+axis*dims[0]*fraction; own=hits(mesh,anchor,normal)
                        if len(own)<2:continue
                        front,back=own[:2]
                        all_other=[h for l,k,s,m in pieces if k=='visual' and not (l==link and s==stem) for h in hits(m,anchor,normal)]
                        if any(h>front+1e-8 for h in all_other):continue # rail/internal wall is occluded
                        other=[h for h in all_other if h<back-1e-8]
                        if not other:continue
                        gap=back-max(other)
                        sections.append({'fraction':fraction,'anchor_root_m':anchor.tolist(),'bar_depth_m':front-back,'visual_rear_gap_m':gap,'collision_rear_gap_m':gap})
                    if len(sections)!=3 or min(x['visual_rear_gap_m'] for x in sections)<.005:continue
                    height=center[1]-meta['scale_source_to_meters']*meta['static_source_bounds'][0][1]
                    rows.append({'asset_id':meta['asset_id'],'asset_root':str(path.parent.resolve()),'joint_name':joint.get('name'),'joint_type':'prismatic','moving_link':joint.find('child').get('link'),'handle_link':link,'mesh':stem,'axis_root':axis.tolist(),'outward_normal_root':normal.tolist(),'dimensions_m':dims.tolist(),'sections':sections,'initial_region_height_m':float(height),'screen_rejections':[], 'region_semantics':'exterior bar' if dims[1]<.03 else 'drawer front edge pinch, NOT a protruding handle', 'source':'initial visual exterior pinch-region hypothesis, not GT joint axis'})
    rows.sort(key=lambda r:(r['initial_region_height_m'], -min(x['visual_rear_gap_m'] for x in r['sections']),r['asset_id'],r['mesh']))
    return {'ranking':rows,'geometry_only':True,'acceptance':'actual frozen IK/approach/PhysX closure still mandatory','source_family':'prismatic dataset assets; no joint axis exported'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=scan(a.prepared);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result['ranking'][:5],indent=2))
