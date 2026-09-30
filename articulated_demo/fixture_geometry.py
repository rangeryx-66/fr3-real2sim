"""Place a support under static geometry, clear of the moving-part sweep."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def fixture_box(manifest,chain,asset_urdf,asset_rotation,asset_xyz,height,clearance=.005):
    static=np.asarray(manifest['static_source_bounds'])*manifest['scale_source_to_meters']
    root=ET.parse(asset_urdf).getroot();pieces=[]
    moving_links=set(manifest['moving_links'])
    for link in root.findall('link'):
        if link.get('name') not in moving_links:continue
        for collision in link.findall('collision'):
            spec=collision.find('geometry/mesh')
            if spec is None:continue
            path=(asset_urdf.parent/spec.get('filename')).resolve()
            vertices=[list(map(float,line.split()[1:4])) for line in path.read_text().splitlines() if line.strip().startswith('vertex ')]
            if not vertices:continue
            origin=collision.find('origin');T=np.eye(4)
            if origin is not None:
                T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            v=np.asarray(vertices)*np.fromstring(spec.get('scale','1 1 1'),sep=' ')
            pieces.append((link.get('name'),v@T[:3,:3].T+T[:3,3]))
    if not pieces:raise ValueError('support requires actual moving collision geometry')
    closed=np.concatenate([v@chain.root_to_link(n,{})[:3,:3].T+chain.root_to_link(n,{})[:3,3] for n,v in pieces])
    # Source +Y is up. Select the static box face nearest the closed moving
    # panel; carve the support footprint away from its complete joint sweep.
    displacement=(closed.mean(0)-static.mean(0))/np.maximum(static[1]-static[0],1e-8)
    axis=max((0,2),key=lambda k:abs(displacement[k]));sign=1 if displacement[axis]>=0 else -1
    limits=manifest['source_joint_limits_rad'];joint=manifest['joint_name']
    sweep=[]
    for angle in np.linspace(limits['lower'],limits['upper'],max(2,math.ceil((limits['upper']-limits['lower'])/math.radians(1))+1)):
        for name,vertices in pieces:
            T=chain.root_to_link(name,{joint:float(angle)});sweep.append(vertices@T[:3,:3].T+T[:3,3])
    swept=np.concatenate(sweep);footprint=static.copy()
    if sign>0:footprint[1,axis]=min(footprint[1,axis],swept[:,axis].min()-clearance)
    else:footprint[0,axis]=max(footprint[0,axis],swept[:,axis].max()+clearance)
    if np.any((footprint[1]-footprint[0])[[0,2]]<=0):raise ValueError('moving sweep leaves no safe rectangular support footprint')
    center=footprint.mean(0);center[1]=static[0,1]-height/2
    size=np.array([footprint[1,2]-footprint[0,2],footprint[1,0]-footprint[0,0],height])
    # Fixture local X/Y correspond to source Z/X, local Z to source Y.
    fixture_rotation=asset_rotation@np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
    return {'center':(asset_xyz+asset_rotation@center).tolist(),'size':size.tolist(),
        'quaternion_wxyz':np.roll(Rotation.from_matrix(fixture_rotation).as_quat(),1).tolist(),
        'audit':{'static_root_bounds_m':static.tolist(),'support_root_bounds_m':footprint.tolist(),
                 'swept_root_bounds_m':[swept.min(0).tolist(),swept.max(0).tolist()],
                 'cut_axis_source':axis,'cut_sign':sign,'sweep_clearance_m':clearance,
                 'note':'support geometry corrected; asset, robot and grasp poses unchanged'}}
