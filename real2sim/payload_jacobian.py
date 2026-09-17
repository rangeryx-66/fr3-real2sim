"""Correct the legacy PhysX hand-COM to TCP Jacobian origin conversion.
Only robot model geometry is read. No payload GT is consumed.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

CONVENTION='TCP_ORIGIN_BASE_AXES_V2'

def tcp_jacobian_record(record, urdf=None):
    """Return a copy; old records retain their bytes and provenance."""
    convention=str(np.asarray(record.get('jacobian_convention',['LEGACY_HAND_COM'])).reshape(-1)[0])
    if convention==CONVENTION:
        return dict(record)
    if 'T_B_hand' not in record:
        raise ValueError('JACOBIAN_ORIGIN_UNKNOWN: missing T_B_hand; explicit convention required')
    path=Path(urdf) if urdf else Path(__file__).resolve().parents[1]/'config/fr3.urdf'
    origin=ET.parse(path).getroot().find("link[@name='fr3_hand']/inertial/origin")
    if origin is None:raise ValueError('FR3_HAND_COM_MISSING')
    c=np.fromstring(origin.attrib['xyz'],sep=' ')
    J=np.asarray(record['jacobian_TCP'],float).copy()
    v=np.asarray(record['T_B_hand'],float)[:,:3,:3]@c
    S=np.zeros((len(v),3,3));S[:,0,1]=-v[:,2];S[:,0,2]=v[:,1];S[:,1,0]=v[:,2];S[:,1,2]=-v[:,0];S[:,2,0]=-v[:,1];S[:,2,1]=v[:,0]
    J[:,:3]+=S@J[:,3:]
    result=dict(record);result['jacobian_TCP']=J
    result['jacobian_convention']=np.asarray([CONVENTION])
    result['jacobian_origin_correction_source']=np.asarray([str(path)])
    return result
