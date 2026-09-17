"""Passive 240 Hz contact/pose telemetry; does not change physics or controls."""
import gzip
import json
from pathlib import Path
import numpy as np

class ContactTrace:
    def __init__(self,contacts):self.contacts=contacts;self.path=None;self.records=[]
    def start(self,path):self.path=Path(path);self.records=[]
    def sample(self,t,phase,q,command,bp,bq,tp,tq,palm_p,palm_q,forces):
        if self.path is None:return
        force,point,normal,separation,count,index=[np.asarray(v) for v in self.contacts.get_contact_force_data(dt=1/240)]
        pairs=[]
        for i in range(2):
            start=int(index[i,0]);n=int(count[i,0]);sl=slice(start,start+n)
            pairs.append(dict(points_world_m=point[sl].tolist(),normals_world=normal[sl].tolist(),normal_force_N=force[sl].reshape(-1).tolist(),separation_m=separation[sl].reshape(-1).tolist()))
        self.records.append(dict(t=t,phase=phase,q=q.tolist(),command_q=command.tolist(),box=bp.tolist(),box_quat=bq.tolist(),tcp=tp.tolist(),tcp_quat=tq.tolist(),palm=palm_p.tolist(),palm_quat=palm_q.tolist(),forces=forces.tolist(),finger_contacts=pairs))
    def stop(self,names):
        if self.path is None:return dict(ok=True,path=None,count=0)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        with gzip.open(self.path,'wt') as f:json.dump(dict(names=names,finger_paths=self.contacts.prim_paths,dt=1/240,contact_frame='world=fr3_link0; signed normals as reported by PhysX',records=self.records),f)
        out=dict(ok=True,path=str(self.path),count=len(self.records));self.path=None;self.records=[];return out
