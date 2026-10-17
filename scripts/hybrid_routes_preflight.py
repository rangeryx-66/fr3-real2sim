"""Bounded complete unloaded/push route audit from observation-based proposals.

No simulator, no physical success claim, no new geometry or grasp selection.
The archived visual grasp is used solely to check transition feasibility;
the live episode must obtain a fresh wrist reconstruction and grasp.
"""
import argparse, copy, json, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from hybrid_panel_preflight import native_model, scoped_panel_check
from interactive_twin_recovery.mobile import physical_scene_class
from cross_robot_adapter import RobotPinkIK
from native_g1_wrist_views import path_to
from piper_mobile_demo.owned_scene import intersects


def run(episode, preflight, prior, output):
    started=time.perf_counter();root=Path(__file__).resolve().parents[1]
    export=json.loads((episode/'cooked_initial.json').read_text());model,mapped=native_model(episode,export)
    allowed=json.loads((episode/'sensor_metadata.json').read_text())['allowed_handle_colliders']
    scene=physical_scene_class(root)(mapped,model,allowed);solver=RobotPinkIK(model.config)
    base=json.loads((episode/'attempts/00/selected_plan.json').read_text())['base']
    axis=np.asarray(prior['axis_world']);origin=np.asarray(prior['origin_world_m'])
    def D(deg):
        R=Rotation.from_rotvec(axis*np.deg2rad(deg)).as_matrix();X=np.eye(4);X[:3,:3]=R;X[:3,3]=origin-R@origin;return X
    def route(start,end,angle,finger_angle=.85):
        proxy=SimpleNamespace(poses=lambda q,b,width:model.poses(q,b,angle=finger_angle),margin=model.margin,limits=model.limits)
        s=SimpleNamespace(check=scene.check,moving_reference=D(angle)@scene.moving_reference)
        ik=SimpleNamespace(solve=lambda *args,**kwargs:np.asarray(end))
        return path_to(proxy,ik,s,base,start,np.eye(4),dict(ik_starts=1,route_iterations=600),np.eye(4))
    tests=[];selected=None
    for region in preflight['region_audits']:
        paths=set(region['native_surface_audit']['cooked']['matching_shapes'])
        for family in region['candidates']:
            for entry in family['paths']:
                if entry['reason']!='STANDOFF_AND_APPROACH_ONLY':continue
                record=dict(region=region['name'],roll_deg=family['roll_deg'],open_deg=entry['open_deg'],valid=False)
                tests.append(record);angle=entry['open_deg'];seed=np.asarray(entry['approach_q'][-1])
                incoming,why=route(model.home,entry['approach_q'][0],angle)
                record['park_to_standoff']=why
                if incoming is None:continue
                reference=np.linalg.inv(D(angle))@entry['target_tcp'];push=[];reason=None;contact_pairs=set()
                # Intended native fingertip contact is the only pair exception.
                # Point comes from RGB-D; it is never snapped to native geometry.
                for theta in np.linspace(angle,2.,int((angle-2.)/.25)+1):
                    T=D(theta)@reference;seed=solver.solve(T,base,seed)
                    if seed is None:reason='PUSH_IK';break
                    P=model.poses(seed,base);moving=D(theta)@scene.moving_reference
                    ok,why=scoped_panel_check(scene,P,moving,paths)
                    if not ok:reason=why;break
                    for robot in scene.robot:
                        if robot.body not in ('gripper_link1','gripper_link2'):continue
                        robot.place(P[robot.body])
                        for target in scene.scene:
                            if target.path in paths and intersects(robot,target):contact_pairs.add((robot.body,target.path))
                    push.append(dict(angle_deg=float(theta),q=seed.tolist(),T_tcp=T.tolist(),margin_rad=model.margin(seed)))
                record.update(push_reason=reason,push_q=push,native_nominal_intersections=[list(x) for x in contact_pairs])
                if reason:continue
                T=np.asarray(push[-1]['T_tcp']);separation=[seed.tolist()]
                for distance in np.linspace(.001,.04,21):
                    goal=T.copy();goal[:3,3]-=distance*T[:3,2];q=solver.solve(goal,base,seed)
                    if q is None:reason='SEPARATION_IK';break
                    ok,why=scene.check(model.poses(q,base),D(2.)@scene.moving_reference,False)
                    if not ok:reason=why;break
                    separation.append(q.tolist());seed=q
                record.update(separation_reason=reason,separation_q=separation)
                if reason:continue
                outgoing,why=route(seed,model.home,2.);record['separation_to_park']=why
                if outgoing is None:continue
                # Check physical jaw sweep at the no-contact park, not in object.
                sweep=[]
                for a in np.linspace(0,.85,35):
                    ok,why=scene.check(model.poses(model.home,base,angle=a),D(angle)@scene.moving_reference,False)
                    sweep.append(dict(driver_rad=float(a),valid=ok,reason=why))
                record['unloaded_park_jaw_sweep']=sweep
                if not all(x['valid'] for x in sweep):continue
                record.update(valid=True,incoming_q=incoming,outgoing_q=outgoing)
                selected=record;break
            if selected:break
        if selected:break
    # Guarded pull release/park and fresh-pose feasibility are kept distinct
    # from the panel contact certificate above. No historical plan is executed.
    result=dict(physical_episodes_started=0,selected=selected,candidates=tests,
        panel_route_valid=selected is not None,live_grasp_or_loaded_range_guarantee=False,
        actual_cap_contact_authorization_still_required=True,wall_s=time.perf_counter()-started)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--preflight',type=Path,required=True)
    p.add_argument('--prior',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=run(a.episode,json.loads(a.preflight.read_text()),json.loads(a.prior.read_text()),a.output)
    print(json.dumps(dict(panel_route_valid=r['panel_route_valid'],wall_s=r['wall_s'],candidates=len(r['candidates']))))
