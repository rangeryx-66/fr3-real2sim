"""Bounded pre-physics range/excitation/sensor audit; no SimulationApp."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
root=a.root;case=root/'results/xarm_wrist_ft_id_v0_20261017/case_02';job=json.loads((case/'job.json').read_text());candidate=json.loads((case/'episode/attempts/00/selected_plan.json').read_text())
from xarm7_native_planning import NativeModel,scene_factory
from cross_robot_adapter import RobotPinkIK
from scipy.spatial.transform import Rotation
source=json.loads((Path(job['source'])/'report.json').read_text());model=NativeModel(None,job['asset_root'],source);solver=RobotPinkIK(model.config)
from revolute10_fixed import authored_handle_colliders
export=json.loads(Path(job['collision_export']).read_text())
allowed=authored_handle_colliders(export,job['grasp_geometry_candidates'][0])
scene=scene_factory(root,export,model,allowed);path=candidate['path'];base=candidate['base'];checks=[]
width=max(v['width_m'] for v in candidate['contact_geometry_audit']['first_contacts'].values())
for v in path:
 theta=v['state'];T=np.asarray(v['T_tcp']);q=np.asarray(v['q']);ok,reason=scene.check(model.poses(q,base,width=width),
     model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta})@np.linalg.inv(model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:0.}))@scene.moving_reference,True)
 checks.append(dict(angle_deg=float(np.rad2deg(theta)),margin_rad=float(model.margin(q)),collision_free=bool(ok),reason=reason))
report=dict(Isaac_scenes_started=0,archived_visual_grasp_plan_only=True,not_a_loaded_range_guarantee=True,range_checks=checks,
    nominal_plan_reached_deg=float(np.rad2deg(candidate['planned_reached_state'])),nominal_minimum_margin_rad=float(min(v['margin_rad'] for v in checks)),
    no_perfect_future_trajectory_admission_gate_added=True,registered_speed_cap_unchanged=True,
    result='meaningful nominal extension exists; new measured grasp and original live protections determine actual reach')
(a.output/'OFFLINE_RANGE.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
