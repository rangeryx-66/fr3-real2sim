"""Archived-state regression only; no Isaac imports or physical state replay."""
import argparse,gzip,json,pickle,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from native_g1_contact_recovery import checked_route
from native_g1_safe_park import plan_exit,verify_park,PARK_DEFINITION

ROOT=Path(__file__).resolve().parents[1]


def archived_row(ep,t=None):
    idx=json.loads((ep/'observations_chunks/index.json').read_text())
    n=idx['records']-1 if t is None else max(0,int(round(t*240))-1)
    chunk=next(c for c in idx['chunks'] if c['first_record']<=n<c['first_record']+c['records'])
    # Trusted experiment-generated archives, not arbitrary downloaded pickle.
    rows=pickle.load(gzip.open(ep/'observations_chunks'/chunk['file'],'rb'))
    return rows[n-chunk['first_record']]


def archived_scene(root):
    from xarm7_native_planning import NativeModel
    from xarm7_native_gripper import configuration
    from cross_robot_adapter import RobotPinkIK
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    ep=root/'episode';job=json.loads((root/'job.json').read_text())
    model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    solver=RobotPinkIK(configuration());model.ik=solver.solve
    scene=physical_scene_class(ROOT)(mapped_export(json.loads((ep/'cooked_initial.json').read_text())),model,
        json.loads((ep/'sensor_metadata.json').read_text())['allowed_handle_colliders'])
    events=json.loads((ep/'events.json').read_text())
    row=archived_row(ep,[e for e in events if e.get('event')=='RECOVERY_PHYSICAL_RELEASE'][-1]['t'])
    q=np.asarray(row['q']);g=dict(model=model,solver=solver,collision=scene,
        robot=SimpleNamespace(get_joint_positions=lambda:q),arm=np.arange(7),fingers=np.array([7,11,8,9,12,10]),
        base=row['base'],moving=lambda:np.asarray(row['T_moving_link']),tcp=lambda:np.asarray(row['T_tcp']),
        qtarget=np.asarray(row['telemetry']['command_q_all']),
        wrist_config=json.loads((ROOT/'configs/xarm7_wrist_rgbd_sim.json').read_text()),
        normal=np.asarray(json.loads((ep/'agent_geometry/observed_geometry.json').read_text())['normal_world']))
    g['ik']=lambda T,seed:solver.solve(T,g['base'],seed,starts=1)
    return ep,row,archived_row(ep),g


def old_route(ep,last):
    result=json.loads((ep/'CONTACT_ATTEMPTS.json').read_text())[-1]
    if result['safe_exit'].get('home_path'):return result['safe_exit']['home_path']
    # refine() replaces approach[0] with measured pregrasp q, while chosen's
    # preplan retains the current-episode measured-start approach check.
    ix=result['index'];pre=json.loads((ep/'rgbd'/('attempt_%02d_pregrasp'%ix)/'camera.json').read_text())['robot_q'][:7]
    measured=json.loads((ep/'attempts'/('%02d'%ix)/'MEASURED_STATE_APPROACH_CHECK.json').read_text())
    return [last['q'][:7],last['command_q_arm'],pre]+list(reversed(measured['preplan']))


def collision_sample(g,path):
    for segment,(a,b) in enumerate(zip(path,path[1:])):
        for f in np.linspace(0,1,max(2,int(np.ceil(np.max(abs(np.asarray(b)-a))/.025))+1)):
            q=(1-f)*np.asarray(a)+f*np.asarray(b);check=checked_route(g,[q,q])
            if not check['valid']:return dict(segment=segment,fraction=float(f),q=q.tolist(),reason=check['reason'])
    return None


def render(g,old,new,hit,path):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    scene=g['collision'];model=g['model'];fig=plt.figure(figsize=(15,5))
    colors=['#c63838','#246ec4','#23915e']
    for panel,(route,title) in enumerate([(old,'Rejected old pregrasp reversal'),(new['clearance_path']+new['parking_path'][1:],'Jointly checked clearance + park'),([hit['q']] if hit else [old[0]],'First predicted collision (not executed)')]):
        ax=fig.add_subplot(1,3,panel+1,projection='3d');fingers=np.asarray(g['robot'].get_joint_positions())[g['fingers']]
        points=[]
        for a,b in zip(route,route[1:]):
            points.extend(model.poses(q,g['base'],finger_q=fingers)['tcp_link'][:3,3] for q in np.linspace(a,b,25))
        if points:
            P=np.asarray(points);ax.plot(*P.T,color=colors[panel],lw=2)
        for s in scene.scene:
            if not (s.body==scene.moving_link):continue
            X=np.asarray(g['moving']());V=s.vertices@X[:3,:3].T+X[:3,3]
            from scipy.spatial import ConvexHull
            faces=ConvexHull(V).simplices
            handle=s.path in scene.allowed
            ax.add_collection3d(Poly3DCollection(V[faces],alpha=.35 if handle else .08,facecolor='#df9b24' if handle else '#999999',edgecolor='none'))
            if handle:points.extend(V)
        drawq=np.asarray(route[0]);P=model.poses(drawq,g['base'],finger_q=fingers)
        for n in ['gripper_link1','gripper_link2']:
            m=model.robot_hulls[n].copy();m.apply_transform(P[n]);ax.add_collection3d(Poly3DCollection(m.triangles,alpha=.6,facecolor='#b53db3',edgecolor='none'));points.extend(m.vertices)
        if points:
            P=np.asarray(points);center=(P.max(0)+P.min(0))/2;radius=max(np.ptp(P,axis=0))/2+.03
            ax.set_xlim(center[0]-radius,center[0]+radius);ax.set_ylim(center[1]-radius,center[1]+radius);ax.set_zlim(center[2]-radius,center[2]+radius)
        ax.set_title(title,fontsize=10);ax.set_xlabel('world x (m)');ax.set_ylabel('y (m)');ax.set_zlabel('z (m)');ax.view_init(25,-75)
    fig.tight_layout();fig.savefig(path,dpi=170);plt.close(fig)


def verify(root,out):
    out.mkdir(parents=True,exist_ok=True);results=dict(physics_executed=False,parking_definition=PARK_DEFINITION,cases={})
    for asset in ['45936','46427']:
        began=time.perf_counter();ep,row,last,g=archived_scene(root/asset)
        old=old_route(ep,last)
        gl=dict(g,robot=SimpleNamespace(get_joint_positions=lambda:np.asarray(last['q'])),moving=lambda:np.asarray(last['T_moving_link']))
        audit=checked_route(gl,old);hit=collision_sample(gl,old)
        closed=checked_route(dict(gl,moving=lambda:g['collision'].moving_reference),old)
        plan=plan_exit(g)
        record=dict(old_route=old,old_final_door_audit=audit,closed_door_counterfactual_audit=closed,first_rejected_sample=hit,
            released_state=dict(q=row['q'],q_command=row['telemetry']['command_q_all'],door_deg=row['door_angle_deg'],t=row['t'],base=row['base']),
            final_state=dict(q=last['q'],q_command=last['telemetry']['command_q_all'],door_deg=last['door_angle_deg'],t=last['t']),joint_plan=plan)
        if plan['valid']:render(gl,old,plan,hit,out/(asset+'_routes.png'))
        record['wall_s']=time.perf_counter()-began;results['cases'][asset]=record
        print(asset,json.dumps(dict(old_valid=audit['valid'],new_valid=plan['valid'],method=plan.get('route_method'),wall_s=record['wall_s'])),flush=True)
    # Terminal parking cannot be awarded to a locally retreated configuration.
    ep,row,last,g=archived_scene(root/'45936');g.update(filtered=lambda:np.zeros(2),force_guard=SimpleNamespace(settled=lambda:True),actual=lambda:np.deg2rad(row['door_angle_deg']))
    assert not verify_park(g,dict(parking_q=g['model'].home.tolist(),parking_region='native_rest'))['reached']
    import native_g1_safe_termination as runtime
    # Original HARD stop must abort unload at its first guarded step.
    guard=SimpleNamespace(samples=['not cleared'],pending=True);calls=[]
    mock=dict(a=SimpleNamespace(output=out),tick=0,dt=1/240,rows=[dict(aperture_m=.015)],
        arm=np.arange(7),fingers=np.arange(7,13),qtarget=np.zeros(13),reference=np.eye(4),force_guard=guard,
        policy={'slow_closure_m_s':.001},controller=SimpleNamespace(geometry=SimpleNamespace(angle=lambda w:w)),world=SimpleNamespace(play=lambda:None))
    def hard():calls.append(1);raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
    mock['step']=hard
    with patch.object(runtime,'release_geometry',return_value={'valid':True}):r=runtime.safe_release_park(mock)
    assert not r['safe'] and len(calls)==1 and guard.pending and guard.samples==['not cleared']
    results['hard_stop_preserved_no_continuation']=True;results['clearance_alone_not_park']=True
    results['passed']=bool(not results['cases']['45936']['old_final_door_audit']['valid'] and results['cases']['46427']['old_final_door_audit']['valid'] and all(c['joint_plan']['valid'] for c in results['cases'].values()))
    (out/'OFFLINE_REGRESSION.json').write_text(json.dumps(results,indent=2));return results


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--archive-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=verify(a.archive_root,a.output);print('passed',result['passed'])
