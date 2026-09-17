"""Offline telemetry analysis, independent of planning and execution decisions."""
import argparse
import gzip
import itertools
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from hand_geometry import geometry,SIZE

ROOT=Path(__file__).resolve().parents[1]
def T(p,q):
    m=np.eye(4);m[:3,3]=p;m[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();return m
def angle(R):return float(Rotation.from_matrix(R).magnitude())

def closing_axis_features(H,O):
    relative=np.linalg.inv(H)@O;R=relative[:3,:3];c=relative[:3,3]
    origin=R.T@(np.array([0.,0.,.00025])-c);direction=R.T@np.array([0.,1.,0.]);lo=-np.inf;hi=np.inf
    for i in range(3):
        if abs(direction[i])<1e-12:
            if abs(origin[i])>SIZE[i]/2:lo,hi=1.,0.;break
        else:
            ends=sorted([(-SIZE[i]/2-origin[i])/direction[i],(SIZE[i]/2-origin[i])/direction[i]])
            lo=max(lo,ends[0]);hi=min(hi,ends[1])
    profile=[]
    for offset in [-.005,0.,.005]:
        shifted=H.copy();shifted[:3,3]+=offset*H[:3,2]
        profile.append(dict(diagnostic_depth_shift_m=offset,pad_coverage=geometry(shifted,O)['min_pad_coverage']))
    return dict(closing_axis_intersects_target=bool(hi>lo),closing_axis_target_chord_m=float(max(0.,hi-lo)),target_center_to_closing_plane_m=float(abs(c[0])),depth_sensitivity_not_executed=profile)

def fk(names,q):
    joints={j.find('child').attrib['link']:j for j in ET.parse(ROOT/'config/fr3.urdf').findall('joint')}
    values=dict(zip(names,q));chain=[];link='fr3_hand_tcp'
    while link!='fr3_link0':
        j=joints[link];chain.append(j);link=j.find('parent').attrib['link']
    out=np.eye(4)
    for j in reversed(chain):
        origin=j.find('origin');a=np.eye(4)
        if origin is not None:
            a[:3,3]=list(map(float,origin.attrib.get('xyz','0 0 0').split()));a[:3,:3]=Rotation.from_euler('xyz',list(map(float,origin.attrib.get('rpy','0 0 0').split()))).as_matrix()
        out=out@a
        if j.attrib['type']!='fixed':
            axis=np.array(list(map(float,j.find('axis').attrib['xyz'].split())));b=np.eye(4);v=values.get(j.attrib['name'],0.)
            if j.attrib['type']=='prismatic':b[:3,3]=axis*v
            else:b[:3,:3]=Rotation.from_rotvec(axis*v).as_matrix()
            out=out@b
    return out

def palm_clearance(H,O):
    import trimesh
    mesh=trimesh.load_mesh(ROOT/'franka_description/meshes/robot_ee/franka_hand_white/collision/hand.stl')
    P=H.copy();P[:3,3]-=.1034*P[:3,2];local=np.linalg.inv(P)@O;points=[];max_step=0.
    for axis in range(3):
        other=[i for i in range(3) if i!=axis]
        lines=[np.linspace(-SIZE[j]/2,SIZE[j]/2,int(np.ceil(SIZE[j]/.003))+1) for j in other]
        max_step=max(max_step,*[float(np.max(np.diff(l))) for l in lines])
        for sign in [-1,1]:
            for u,v in itertools.product(*lines):
                p=np.zeros(3);p[axis]=sign*SIZE[axis]/2;p[other]=[u,v];points.append(local[:3,:3]@p+local[:3,3])
    _,distance,_=trimesh.proximity.closest_point(mesh,np.array(points))
    d=float(distance.min());bound=float(np.sqrt(2)*max_step/2)
    return dict(sampled_surface_distance_m=d,lower_bound_m=max(0.,d-bound),surface_sampling_error_bound_m=bound,method='Target surface grid <=3 mm against official palm triangle mesh; unsigned separation estimate, collision checked separately by MoveIt')

def analyze(path):
    r=json.loads(path.read_text());trace=Path(r['trace']['path'])
    if not trace.exists():trace=path.parent/trace.name
    d=json.load(gzip.open(trace,'rt'));records=d['records'];names=d['names'];fi=[names.index('fr3_finger_joint'+str(i)) for i in [1,2]]
    close=[v for v in records if v['phase']=='CLOSE'];micro=[v for v in records if v['phase']=='MICRO_LIFT'];endpoints=[]
    for e in r['telemetry']['motion_endpoints']:
        s=e['actual'];actual=T(s['tcp'],s['tcp_quat']);measured=fk(names,s['q']);cmd=np.array(e['commanded_EE'])
        endpoints.append(dict(stage=e['stage'],position_error_m=e['position_error_m'],rotation_error_rad=e['rotation_error_rad'],measured_joint_FK_to_sim_TCP_m=float(np.linalg.norm(measured[:3,3]-actual[:3,3])),measured_joint_FK_to_sim_TCP_rad=angle(measured[:3,:3].T@actual[:3,:3]),command_to_measured_joint_FK_m=float(np.linalg.norm(cmd[:3,3]-measured[:3,3])),actual_pad_coverage=e['actual_hand_geometry']['min_pad_coverage']))
    first=[next((v['t']-close[0]['t'] for v in close if v['forces'][i]>.1),None) if close else None for i in range(2)]
    dots=[];examples=[];example_counts=[0,0];pad_force=[0.,0.];total_force=[0.,0.]
    for v in close:
        R=Rotation.from_quat(np.roll(v['tcp_quat'],-1)).as_matrix();normals=[]
        for i,c in enumerate(v['finger_contacts']):
            f=np.abs(np.array(c['normal_force_N']));ns=np.array(c['normals_world']).reshape(-1,3);ps=np.array(c['points_world_m']).reshape(-1,3)
            if f.sum()>.1:
                normal=(f[:,None]*ns).sum(0);normal/=max(1e-12,np.linalg.norm(normal));normals.append(normal)
                hp=(ps-np.array(v['tcp']))@R
                valid=(abs(hp[:,0])<=.00875+.0005)&(hp[:,2]>=-.009-.0005)&(hp[:,2]<=.0095+.0005)&(abs(ns@R[:,1])>.7)
                pad_force[i]+=float(f[valid].sum());total_force[i]+=float(f.sum())
                if example_counts[i]<4:
                    examples.append(dict(t=v['t'],finger=i,points_world_m=ps[f>.01].tolist(),normal_world=ns[f>.01].tolist(),force_N=f[f>.01].tolist()));example_counts[i]+=1
            else:normals.append(None)
        if all(n is not None for n in normals):dots.append(float(np.dot(*normals)))
    micro_summary=None
    if micro:
        rel=[np.linalg.inv(T(v['tcp'],v['tcp_quat']))@T(v['box'],v['box_quat']) for v in micro]
        dp=max(float(np.linalg.norm(t[:3,3]-rel[0][:3,3])) for t in rel);dr=max(angle(rel[0][:3,:3].T@t[:3,:3]) for t in rel)
        micro_summary=dict(relative_translation_max_m=dp,relative_rotation_max_rad=dr,target_rise_m=micro[-1]['box'][2]-micro[0]['box'][2],bilateral_fraction=float(np.mean([min(v['forces'])>.1 for v in micro])),relative_motion_gt_2mm_or_2deg=bool(dp>.002 or dr>np.deg2rad(2)))
        tail=[v for v in micro if v['t']>=micro[-1]['t']-.1]
        micro_summary['bilateral_tail_fraction']=float(np.mean([min(v['forces'])>.1 for v in tail]))
        micro_summary['lost_contact_without_target_following']=bool(micro_summary['bilateral_tail_fraction']<.5 and micro_summary['target_rise_m']<.001)
    approach=next((e for e in r['telemetry']['motion_endpoints'] if e['stage']=='APPROACH'),None)
    clearance=None
    if approach:
        s=approach['actual'];clearance=palm_clearance(T(s['tcp'],s['tcp_quat']),T(s['box'],s['box_quat']))
    result=dict(seed=r['seed'],success=r['success'],category=r['category'],rank=r['selected_rank'],physical_clutter=r['physical_clutter'],geometry_filter=r['geometry_filter'],commanded_pad_coverage=r['telemetry'].get('commanded_geometry',{}).get('min_pad_coverage'),motion_endpoints=endpoints,contact_start_after_close_s=first,bilateral_close_fraction=float(np.mean([min(v['forces'])>.1 for v in close])) if close else 0.,close_end_width_m=sum(close[-1]['q'][i] for i in fi) if close else None,close_end_finger_positions_m=[close[-1]['q'][i] for i in fi] if close else None,contact_normal_dot_median=float(np.median(dots)) if dots else None,force_on_closing_pad_fraction=[a/b if b else 0. for a,b in zip(pad_force,total_force)],contact_examples=examples,micro_lift=micro,palm_target_clearance=clearance)
    result['micro_lift']=micro_summary
    result['contact_first_events']=[next((dict(phase=v['phase'],after_trace_start_s=v['t']-records[0]['t']) for v in records if v['forces'][i]>.1),None) for i in range(2)]
    if r['telemetry'].get('TF'):
        initial=r['initial_target'];O=T(initial['position'],initial['quaternion_wxyz']);H=np.array(r['telemetry']['TF']['T_B_TCP'])
        result['closing_axis_geometry']=closing_axis_features(H,O)
    out=path.with_name('diagnostic_'+path.name);out.write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
    rows=[]
    for path in sorted(a.directory.glob('[AB]_seed*.json')):
        if not json.loads(path.read_text()).get('trace'):continue
        rows.append(analyze(path));print(path.name,rows[-1]['category'],rows[-1]['commanded_pad_coverage'],flush=True)
    (a.directory/'diagnostics.json').write_text(json.dumps(rows,indent=2))
