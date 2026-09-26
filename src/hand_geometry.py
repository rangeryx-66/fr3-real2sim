"""Franka pad geometry diagnostics; original network pose and score are immutable."""
import itertools
import numpy as np
from frames import grasp_to_tcp
from robot_profile import get_profile
_PROFILE=get_profile()

SIZE=np.array([.045,.045,.05])
PAD_X=(-.028,.028) if _PROFILE.name=='piper' else (-.00875,.00875)
PAD_Z=(-.070,-.006) if _PROFILE.name=='piper' else (-.009,.0095)  # .0584 + (.04525 +/-.0185/2) - .1034

def clip(poly,axis,bound,sign):
    out=[]
    for a,b in zip(poly,poly[1:]+poly[:1]):
        ia=sign*(a[axis]-bound)>=-1e-12;ib=sign*(b[axis]-bound)>=-1e-12
        if ia:out.append(a)
        if ia!=ib:out.append(a+(b-a)*((bound-a[axis])/(b[axis]-a[axis])))
    return out

def area(poly):
    if len(poly)<3:return 0.
    a=np.array(poly);return float(abs(np.dot(a[:,0],np.roll(a[:,1],1))-np.dot(a[:,1],np.roll(a[:,0],1)))/2)

def geometry(T_B_H,T_B_O):
    T=np.linalg.inv(T_B_H)@T_B_O;R=T[:3,:3];c=T[:3,3]
    axis=int(np.argmax(np.abs(R[1])));others=[i for i in range(3) if i!=axis]
    patches=[]
    for side in [-1,1]:
        vertices=[]
        for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]:
            p=np.zeros(3);p[axis]=side*SIZE[axis]/2;p[others]=[u*SIZE[others[0]]/2,v*SIZE[others[1]]/2]
            vertices.append((R@p+c)[[0,2]])
        clipped=vertices
        for ax,lo,hi in [(0,*PAD_X),(1,*PAD_Z)]:
            clipped=clip(clipped,ax,lo,1);clipped=clip(clipped,ax,hi,-1)
        patches.append(dict(face_side=side,projected_area_m2=area(vertices),pad_overlap_m2=area(clipped),pad_coverage=area(clipped)/((PAD_X[1]-PAD_X[0])*(PAD_Z[1]-PAD_Z[0]))))
    verts=np.array([R@(np.array(s)*SIZE/2)+c for s in itertools.product([-1,1],repeat=3)])
    return dict(T_TCP_target=T.tolist(),target_center_TCP_m=c.tolist(),closing_axis_target_axis=axis,closing_axis_alignment=float(abs(R[1,axis])),pad_faces=patches,min_pad_coverage=min(p['pad_coverage'] for p in patches),target_TCP_bounds_m=[verts.min(0).tolist(),verts.max(0).tolist()],closing_plane_offset_m=float(abs(c[0])),target_depth_center_m=float(c[2]),pad_bounds_m=dict(x=PAD_X,z=PAD_Z),definition='Projected opposite target faces on pad x/z rectangle; diagnostic geometric overlap, not a force-closure guarantee')

if __name__=='__main__':
    import json
    from pathlib import Path
    from scipy.spatial.transform import Rotation
    root=Path(__file__).resolve().parents[1];rows=[]
    for p in sorted((root/'results/clutter_ab_20_v1').glob('B_seed*.json')):
        r=json.loads(p.read_text())
        if r['selected_rank'] is None:continue
        d=json.loads((root/f"results/clutter_ab_20_v1/inputs/seed_{r['seed']:04d}_grasps.json").read_text());g=d['grasps'][r['selected_rank']]
        _,H,_=grasp_to_tcp(d['T_B_C'],g['rotation'],g['translation'],g['depth'])
        O=np.eye(4);O[:3,3]=r['initial_target']['position'];O[:3,:3]=Rotation.from_quat(np.roll(r['initial_target']['quaternion_wxyz'],-1)).as_matrix()
        row=dict(seed=r['seed'],category=r['category'],rank=g['rank'],geometry=geometry(H,O));rows.append(row)
        print(row['seed'],row['category'],row['rank'],round(row['geometry']['min_pad_coverage'],4),row['geometry']['target_center_TCP_m'])
    out=root/'results/contact_geometry';out.mkdir(exist_ok=True);(out/'original_geometry.json').write_text(json.dumps(rows,indent=2))
