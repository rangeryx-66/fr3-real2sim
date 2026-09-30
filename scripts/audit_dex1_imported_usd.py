"""Inspect composed collider instance proxies against the official link frames.

Run using Isaac's Python (pxr required). This audits source geometry before
PhysX convex cooking, not contact forces or the cooked hull itself.
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--usd',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();stage=Usd.Stage.Open(str(a.usd));cache=UsdGeom.XformCache()
    urdf=ET.parse(ROOT/'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1/dex1_1.urdf').getroot()
    names=('Link1_2','Link1_3','Link2_2','Link2_3');rows=[]
    for prim in Usd.PrimRange(stage.GetDefaultPrim(),Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        parent=prim.GetParent()
        while parent and parent.GetName() not in ('dex1_'+n for n in names):parent=parent.GetParent()
        if not parent:continue
        name=parent.GetName().removeprefix('dex1_');link=urdf.find(f"link[@name='{name}']")
        collision=link.find('collision');node=collision.find('geometry/mesh');o=collision.find('origin')
        origin=dict(o.attrib) if o is not None else {}
        expected=np.eye(4);expected[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        expected[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        actual=np.array(cache.ComputeRelativeTransform(prim,parent)[0]).T
        points=np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        joint=next(j for j in urdf.findall('joint') if j.find('child').get('link')==name)
        jo=joint.find('origin');jo=jo.attrib if jo is not None else {}
        fixed_expected=np.eye(4);fixed_expected[:3,3]=np.fromstring(jo.get('xyz','0 0 0'),sep=' ')
        fixed_expected[:3,:3]=Rotation.from_euler('xyz',np.fromstring(jo.get('rpy','0 0 0'),sep=' ')).as_matrix()
        fixed_actual=np.array(UsdGeom.Xformable(parent).GetLocalTransformation()).T
        rows.append({'link':name,'usd_path':str(prim.GetPath()),'approximation':prim.GetAttribute('physics:approximation').Get(),
                     'source_mesh':str((ROOT/'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1'/node.get('filename')).resolve()),
                     'source_scale':node.get('scale','1 1 1'),'collision_origin':origin,'collider_to_link':actual.tolist(),
                     'collider_transform_max_error':float(np.max(np.abs(actual-expected))),
                     'fixed_joint_origin':jo,'fixed_link_transform_max_error':float(np.max(np.abs(fixed_actual-fixed_expected))),
                     'usd_mesh_bounds':np.stack((points.min(0),points.max(0))).tolist(),'usd_mesh_vertices':points.tolist()})
    if len(rows)!=4:raise RuntimeError(f'expected four finger colliders, got {len(rows)}')
    if any(r['collider_transform_max_error']>1e-6 or r['fixed_link_transform_max_error']>1e-6 for r in rows):raise RuntimeError('import frame mismatch')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'usd':str(a.usd),'colliders':rows},indent=2))
    print([(r['link'],r['approximation'],r['collider_transform_max_error'],r['fixed_link_transform_max_error']) for r in rows])
if __name__=='__main__':main()
