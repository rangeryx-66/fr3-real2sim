"""Native-owned collision audit and unchanged IK for measured closed grasps."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
from piper_mobile_demo.owned_scene import OwnedFingerScene,arm_check


def main():
    request,response=map(Path,sys.argv[1:]);r=json.loads(request.read_text())
    source=json.loads((Path(r['source'])/'report.json').read_text())
    model=Model(ROOT/'config/piper.urdf',r['asset_root'],source)
    model.deadline_timestamp=r.get('deadline_timestamp',float('inf'))
    owner=OwnedFingerScene(ROOT,r['export'],r['ownership'],r['allowed_targets'],model.manifest['moving_link'])
    owner.reference_angle_rad=np.deg2rad(r.get('export_door_angle_deg',0.))
    if r.get('export_moving_pose') is not None:owner.moving_reference=np.asarray(r['export_moving_pose'])
    out={'kind':r['kind'],'collision_source':'native PhysX cooked convexes plus unchanged official raw nonpad faces','geometry_export_sha256':hashlib.sha256(Path(r['export']).read_bytes()).hexdigest(),'rows':[]}
    if r['kind']=='audit':
        states=json.loads(Path(r['observations']).read_text());mismatches=0
        registry={e['path']:e.get('owner') for e in owner.data['shapes']}
        for i,s in enumerate(states):
            safe,why=owner.check({k:np.asarray(v) for k,v in s['finger_world_poses'].items()},np.asarray(s['T_moving_link']))
            if safe and s.get('robot_world_poses'):
                safe,why=owner.check_self({k:np.asarray(v) for k,v in s['robot_world_poses'].items()},model.allowed)
            if safe:
                safe,why,_=arm_check(model,np.asarray(s['q'])[:6],r['base'],np.deg2rad(s.get('door_angle_deg',0)),np.asarray(s['q'])[6:])
            native=s['ownership'];contacts=[c for c in native['contacts'] if c['force_n']>0]
            mismatch=sum(c['owner']!=registry.get(c['collider']) for c in contacts);mismatches+=mismatch
            loaded=s['phase'] in ['CLOSURE_HOLD','PULL_DIAGNOSTIC','PULL_HOLD','OPEN_DOOR','STAGE_HOLD']
            violations=[]
            if not safe:violations.append(why)
            if mismatch:violations.append('NATIVE_OWNER_MISMATCH')
            if native['metal_contacts']:violations.append('METAL_CONTACT')
            if native['pad_target_violations']:violations.append('PAD_WRONG_TARGET')
            if s['margin_rad']<=.05:violations.append('LOW_JOINT_MARGIN')
            if loaded and min(native['pad_forces_n'].values())<.2:violations.append('NO_BILATERAL_PAD_CONTACT')
            if s.get('relative_slip_m',0)>r.get('slip_limit_m',.001):violations.append('GRASP_SLIP')
            valid=not violations
            out['rows'].append({'sample':i,'t':s['t'],'phase':s['phase'],'valid':valid,'geometry_safe':safe,'reason':why,'violations':violations,'metal_contacts':native['metal_contacts'],'pad_force_n':native['pad_forces_n'],'margin_rad':s['margin_rad'],'slip_m':s.get('relative_slip_m',0)})
            if not valid:
                out.update(status='OWNED_TRAJECTORY_REJECTED',failed_sample=i,failure=out['rows'][-1]);break
        else:out['status']='OWNED_TRAJECTORY_VALID'
        out['native_owner_mismatches']=mismatches;out['recorded_sample_count']=len(states);out['validated_sample_count']=len(out['rows'])
    else:
        base=np.asarray(r['base']);current=np.asarray(r['actual_q'])[:6];fingers=np.asarray(r['actual_q'])[6:]
        angle=float(r['actual_door_angle_deg']);origin=float(r['reference_door_angle_deg']);T0=np.asarray(r['reference_tcp'])
        target=lambda deg:model.door_target(T0,np.deg2rad(deg))@np.linalg.inv(model.door_target(T0,np.deg2rad(origin)))@T0
        # Express the relative articulation transform around the measured grasp,
        # without editing the asset's joint or driving that joint in simulation.
        if r['kind']=='pull':
            T=target(angle);V=target(angle+.001)[:3,3]-T[:3,3];radius=np.linalg.norm(V)/np.deg2rad(.001)
            if radius<.001:raise RuntimeError('HANDLE_AT_ROTATION_CENTER')
            goals=[angle+np.rad2deg(mm/1000/radius) for mm in [.25,.5,1.,2.]];out['tangent_world']=(V/np.linalg.norm(V)).tolist();out['radius_m']=float(radius)
        else:goals=list(np.arange(angle+.25,float(r['goal_deg']),.25))+[float(r['goal_deg'])]
        for goal in goals:
            T=target(goal);q=model.ik(T,base,seed=current,starts=5)
            if q is None:out.update(status='NO_IK',failed_angle_deg=goal);break
            if np.max(np.abs(q-current))>.4:out.update(status='IK_BRANCH_JUMP',failed_angle_deg=goal);break
            for fraction in np.linspace(0,1,max(2,int(np.ceil(np.max(abs(q-current))/.005))+1)):
                qr=current+fraction*(q-current);deg=angle+fraction*(goal-angle)
                safe,why=owner.check_robot(model,qr,base,np.deg2rad(deg),fingers)
                if not safe:out.update(status=why,failed_angle_deg=deg);break
            if not safe:break
            out['rows'].append({'angle_deg':goal,'q':q.tolist(),'T':T.tolist(),'margin_rad':model.margin(q)});current=q;angle=goal
        else:out['status']='OWNED_PATH_PLANNED'
        out['minimum_joint_margin_rad']=min([model.margin(np.asarray(r['actual_q'])[:6])]+[x['margin_rad'] for x in out['rows']])
    response.write_text(json.dumps(out,indent=2));print(out['status'],len(out['rows']),flush=True)

if __name__=='__main__':main()
