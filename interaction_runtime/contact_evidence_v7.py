"""Passive native normal impulses + separate friction-anchor measurements.

No inferred friction force from normal reports. Sign is preserved on the
sensor/finger body; action-reaction projection is explicitly negated. The
independent engineering load check must validate this convention in situ.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent_command_protocol_v12 import atomic_json


def active_friction(packet):
    if packet is None:raise ValueError('Native friction API returned no packet')
    force,point,count,start=[np.asarray(x) for x in packet]
    force=force.reshape(-1,3);point=point.reshape(-1,3)
    ids=[]
    for n,i in zip(count.ravel(),start.ravel()):
        n=int(n);i=int(i)
        if n<0 or i<0 or i+n>len(force):raise ValueError('Invalid native friction packet bounds')
        ids.extend(range(i,i+n))
    if len(ids)!=len(set(ids)):raise ValueError('Overlapping native pair ranges')
    return force[ids],point[ids]


class ContactEvidence:
    def __init__(self,folder,axis,dt,record_points=True):
        self.folder=Path(folder);self.folder.mkdir(exist_ok=True);self.axis=np.asarray(axis,float);self.dt=dt
        self.rows=[];self.files=[];self.identities=[];self.previous={};self.failures={};self.record_points=record_points;self.pad_bounds={}

    def identity(self,name):
        if name not in self.identities:self.identities.append(name)
        return self.identities.index(name)

    def read(self,r,row,poses):
        if not self.pad_bounds:self.pad_bounds={k:np.asarray(v).tolist() for k,v in r.model.pads.items()}
        sums=np.zeros((2,3));normals=np.zeros((2,3));friction_ok=np.zeros(2,bool);anchor_count=np.zeros(2,int);slip=[]
        delta_q=row['joint_coordinate_velocity'];moving_velocity=self.axis*delta_q
        def append(side,kind,p,J,n,collider,target):
            T=poses[side];local=T[:3,:3].T@(p-T[:3,3]);bounds=r.model.pads[side]
            tangent_speed=np.nan
            if side in self.previous and np.isfinite(n).all() and np.linalg.norm(n)>0:
                old=self.previous[side];v=(T[:3,3]-old[:3,3])/self.dt
                omega=Rotation.from_matrix(T[:3,:3]@old[:3,:3].T).as_rotvec()/self.dt
                relative=v+np.cross(omega,p-T[:3,3])-moving_velocity;normal=n/np.linalg.norm(n)
                tangent=relative-normal*(relative@normal);tangent_speed=float(np.linalg.norm(tangent));slip.append(tangent_speed)
            if not self.record_points:return
            self.rows.append(dict(step=np.uint32(r.tick),attempt=np.uint8(r.g.get('physical_grasps_started',0)+1),finger=np.uint8(r.g['FINGERS'].index(side)),kind=np.uint8(kind),collider=np.uint16(self.identity(collider)),target=np.uint16(self.identity(target)),
                point_world=np.asarray(p,np.float32),point_pad=np.asarray(local,np.float32),normal_world=np.asarray(n,np.float32),normal_pad=np.asarray(T[:3,:3].T@n,np.float32),impulse_world=np.asarray(J,np.float64),
                longitudinal_margin_m=np.float32(min(local[1]-bounds[0,1],bounds[1,1]-local[1])),lateral_margin_m=np.float32(min(local[0]-bounds[0,0],bounds[1,0]-local[0])),
                inner_plane_gap_m=np.float32(local[2]-bounds[0,2]),relative_tangential_speed_m_s=np.float32(tangent_speed)))
        for c in row['contacts']:
            side=c['finger']
            if side not in r.g['FINGERS'] or not c.get('allowed_pad_target') or c['force_n']<=0:continue
            J=np.asarray(c['impulse_world_ns']);normals[r.g['FINGERS'].index(side)]+=J/self.dt
            if 'contact_point_world_m' in c:
                append(side,0,np.asarray(c['contact_point_world_m']),J,np.asarray(c.get('native_normal_world',[np.nan]*3)),c['collider'],c['target'])
        for j,(view,side) in enumerate(zip(r.scene['finger_views'],r.g['FINGERS'])):
            try:
                f,p=active_friction(view.get_friction_data(dt=self.dt));sums[j]=f.sum(0);anchor_count[j]=len(f);friction_ok[j]=True
                for force,point in zip(f,p):append(side,1,point,force*self.dt,np.full(3,np.nan),str(view.prim_paths[0])+':friction_anchor',str(r.scene['contact_target_path']))
            except Exception as error:self.failures[side]=repr(error)
        self.previous={k:v.copy() for k,v in poses.items()}
        if len(self.rows)>=8192:self.flush()
        # Empty anchor buffers are observable absence of reports, not measured
        # zero friction at an otherwise loaded interface.
        available=bool(np.all(friction_ok) and anchor_count.sum()>0)
        return dict(native_normal_force_on_fingers_world_N=normals,native_friction_force_on_fingers_world_N=sums,
            friction_api_valid=friction_ok,friction_anchor_counts=anchor_count,
            traction_available=available,traction_on_object_rail_N=float(-(sums+normals).sum(0)@self.axis) if available else np.nan,
            slip_velocity_available=bool(slip),relative_tangent_speed_m_s=float(np.median(slip)) if slip else np.nan,
            unavailable=dict(self.failures),normal_report_is_not_complete_wrench=True)

    def flush(self):
        if not self.rows:return
        p=self.folder/('%04d.npz'%len(self.files));data={k:np.asarray([r[k] for r in self.rows]) for k in self.rows[0]}
        np.savez_compressed(p,**data);self.files.append(dict(file=p.name,records=len(self.rows)));self.rows=[]
        atomic_json(self.folder/'index.json',dict(physics_dt_s=self.dt,files=self.files,identities=self.identities,axis_world=self.axis.tolist(),pad_bounds=self.pad_bounds,
            kinds=['native_normal_report','native_tangential_friction_anchor'],impulse_units='N s',position_units='m',
            normal_sign='original native collider ordering, preserved',friction_sign='native force on sensor finger body; object reaction explicitly negated',
            relative_tangent_speed='derived rigid material-point velocity estimate; pre-contact finger poses and measured native rail velocity; not a direct friction measurement',
            unavailable=self.failures,raw_private=True))

    def close(self):self.flush()
