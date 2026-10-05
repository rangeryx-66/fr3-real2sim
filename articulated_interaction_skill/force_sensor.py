"""Optional read-only PhysX normal+friction tensor observation.

No collision/material/drive edits. Never feeds contact acceptance or control.
Contact reports alone expose normal impulses; friction needs its own buffer.
"""
import json
from pathlib import Path
import numpy as np


def array(x):
    if hasattr(x,'detach'):x=x.detach().cpu().numpy()
    elif hasattr(x,'numpy'):x=x.numpy()
    # Native APIs may reuse count/start buffers on the next query.
    return np.array(x,copy=True)


class ForceSensor:
    def __init__(self,scene,export,output):
        self.view=None;self.error=None;self.output=Path(output)
        try:
            bodies=sorted({e['rigid_body_path'] for e in export['shapes'] if e.get('rigid_body_path','').split('/')[-1] in ('gripper_link1','gripper_link2')})
            self.view=scene['world'].physics_sim_view.create_rigid_contact_view(bodies,[[scene['contact_target_path']]]*len(bodies),max_contact_data_count=128)
            if self.view.sensor_count!=2:raise RuntimeError('FINGER_SENSOR_COUNT_MISMATCH')
        except Exception as error:self.error=str(error);self.view=None
        self.save()

    def save(self):
        (self.output/'effort_sensor_capability.json').write_text(json.dumps({'available':self.view is not None,'error':self.error,'read_only':True,'controller_input':False,'source':'PhysX tensor normal contact + get_friction_data buffers','source_docs':'https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html'},indent=2))

    def sample(self,dt,axis=None,point=None):
        if self.view is None:return None
        try:
            f,p,n,separation,count,start=map(array,self.view.get_contact_data(dt));t,tp,tc,ts=map(array,self.view.get_friction_data(dt))
            F=np.zeros(3);M=np.zeros(3);N=0.;friction_count=0;loads=[];counts=[]
            for i in range(self.view.sensor_count):
                load=0.;contacts=0
                for j in range(self.view.filter_count):
                    a=int(start[i,j]);k=int(count[i,j]);v=-(f[a:a+k].reshape(-1,1)*n[a:a+k]);F+=v.sum(0);N+=float(np.linalg.norm(v,axis=1).sum());load+=float(np.linalg.norm(v,axis=1).sum());contacts+=k
                    if point is not None:M+=np.cross(p[a:a+k]-point,v).sum(0)
                    a=int(ts[i,j]);k=int(tc[i,j]);v=-t[a:a+k];F+=v.sum(0);friction_count+=k
                    if point is not None:M+=np.cross(tp[a:a+k]-point,v).sum(0)
                loads.append(load);counts.append(contacts)
            return {'normal_loads_per_finger_n':loads,'contact_counts_per_finger':counts,'bilateral_buffer_verified':len(counts)==2 and min(counts)>0,'force_on_handle_world_n':F.tolist(),'normal_load_sum_n':N,'friction_patch_count':friction_count,
                    'estimated_axis_torque_nm':None if axis is None else float(np.asarray(axis)@M),
                    'source':'native PhysX normal+friction tensors; forces on handle = minus on finger'}
        except Exception as error:self.error=str(error);self.view=None;self.save();return None
