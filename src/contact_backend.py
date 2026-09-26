"""Instrumented existing executor: fixed JSON, 1 mm scene margin, optional pad filter."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from moveit_msgs.srv import GetPositionFK
from clutter_backend import ClutterBackend,ROOT,BASE,TCP,PROFILE,transform
from frames import grasp_to_robot_tcp
from hand_geometry import geometry
import plant

SOURCE=ROOT/'results/clutter_ab_20_v1'
BAD=[0,1,6,8,13,14,17]
CONTROL=[3,4,5,7,9,10,11]

def snapshot():
    s=plant.state();return {k:s[k] for k in ['t','names','q','box','box_quat','tcp','tcp_quat','forces']}

class ContactBackend(ClutterBackend):
    def __init__(self,output,geometry_filter=False,replay=False,empty=False,selected_run=None,min_pad_coverage=.045,arm_kp=None,arm_kd=None,lift_gain_scale=1.):
        super().__init__(output);self.margin=.001;self.geometry_filter=geometry_filter;self.replay=replay;self.empty=empty
        self.selected_run=Path(selected_run) if selected_run else SOURCE;self.min_pad_coverage=min_pad_coverage
        self.arm_kp=arm_kp;self.arm_kd=arm_kd
        self.lift_gain_scale=lift_gain_scale
    def perception(self,seed):
        path=SOURCE/f'inputs/seed_{seed:04d}_grasps.json';data=json.loads(path.read_text())
        self.original_data=copy.deepcopy(data)
        if self.replay:
            original=json.loads((self.selected_run/f'B_seed_{seed:04d}.json').read_text());rank=original['selected_rank']
            assert rank is not None
            data['grasps']=[g for g in data['grasps'] if g['rank']==rank]
        return data,str(path)
    def candidate(self,g,data):
        detail,plan=super().candidate(g,data)
        pre,H,lift=grasp_to_robot_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'],PROFILE.grasp_tip_offset_m)
        geom=geometry(H,transform(self.initial['box'],self.initial['box_quat']));detail['hand_geometry']=geom
        # Robot-specific geometric gate; no score changes or pose shifts.
        # The coverage threshold is explicitly calibrated on the diagnostic box cohort.
        if plan is not None and self.geometry_filter and geom['min_pad_coverage']<self.min_pad_coverage:
            detail.update(status='INSUFFICIENT_PAD_OVERLAP',detail=f"pad coverage {geom['min_pad_coverage']:.6f} < {self.min_pad_coverage:.6f}");plan=None
        if plan is not None and self.selected_targets is None:
            self.selected_targets=dict(PREGRASP=pre,APPROACH=H,LIFT=lift)
            T_C_G=np.eye(4);T_C_G[:3,:3]=g['rotation'];T_C_G[:3,3]=g['translation']
            T_G_H=np.eye(4);T_G_H[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]];T_G_H[0,3]=g['depth']-PROFILE.grasp_tip_offset_m
            self.telemetry['grasp']=copy.deepcopy(g)
            self.telemetry['TF']=dict(T_B_C=data['T_B_C'],T_C_G=T_C_G.tolist(),T_G_TCP=T_G_H.tolist(),T_B_TCP=H.tolist())
            self.telemetry['commanded_geometry']=geom
        return detail,plan
    def phase(self,name):
        if name=='MICRO_LIFT' and self.lift_gain_scale!=1:
            self.lift_gains=plant.command(dict(op='arm_gain_scale',scale=self.lift_gain_scale))
        if name=='PERCEPTION':
            self.trace_path=self.output/f'trace_seed_{self.seed:04d}.json.gz'
            plant.command(dict(op='trace_start',path=str(self.trace_path)))
        super().phase(name)
        self.telemetry['phase_starts'].append(dict(phase=name,state=snapshot()))
    def execute(self,traj,category):
        start=self.measured();goal=copy.deepcopy(start)
        for name,q in zip(traj.joint_trajectory.joint_names,traj.joint_trajectory.points[-1].positions):goal.joint_state.position[goal.joint_state.name.index(name)]=q
        req=GetPositionFK.Request();req.header.frame_id=BASE;req.fk_link_names=[TCP];req.robot_state=goal
        fk=self.call('compute_fk',req).pose_stamped[0].pose
        T=transform([fk.position.x,fk.position.y,fk.position.z],[fk.orientation.w,fk.orientation.x,fk.orientation.y,fk.orientation.z])
        wanted=self.selected_targets.get(self.stage,T)
        try:super().execute(traj,category)
        finally:
            actual=snapshot();A=transform(actual['tcp'],actual['tcp_quat'])
            self.telemetry['motion_endpoints'].append(dict(stage=self.stage,commanded_EE=wanted.tolist(),trajectory_endpoint_EE=T.tolist(),actual=actual,position_error_m=float(np.linalg.norm(wanted[:3,3]-A[:3,3])),rotation_error_rad=float(Rotation.from_matrix(wanted[:3,:3].T@A[:3,:3]).magnitude()),actual_hand_geometry=geometry(A,transform(actual['box'],actual['box_quat']))))
    def gripper(self,width):
        before=snapshot()
        try:return super().gripper(width)
        finally:self.telemetry['gripper_commands'].append(dict(commanded_width_m=width,before=before,after=snapshot()))
    def episode(self,seed,mode='B'):
        self.seed=seed;self.trace_path=None;self.selected_targets=None
        self.telemetry=dict(phase_starts=[],motion_endpoints=[],gripper_commands=[])
        self.lift_gains=None
        gains=plant.command(dict(op='arm_gain_scale',scale=1.))
        if self.arm_kp is not None:gains=plant.command(dict(op='arm_gains',kp=self.arm_kp,kd=self.arm_kd))
        plant.command(dict(op='clutter_enabled',enabled=not self.empty))
        result=super().episode(seed,'A' if self.empty else 'B')
        if self.trace_path:result['trace']=plant.command(dict(op='trace_stop'))
        if self.geometry_filter and result['selected_rank'] is None and any(c['status']=='INSUFFICIENT_PAD_OVERLAP' for c in result['candidates']):
            result.update(category='INSUFFICIENT_PAD_OVERLAP',detail='All scene-valid candidates fail the robot-specific pad overlap gate')
        result.update(telemetry=self.telemetry,margin_m=self.margin,geometry_filter=self.geometry_filter,min_pad_coverage=self.min_pad_coverage,pose_replay=self.replay,physical_clutter=not self.empty,controller_gains=gains,lift_controller_gains=self.lift_gains,lift_gain_scale=self.lift_gain_scale)
        (self.output/f"{result['mode']}_seed_{seed:04d}.json").write_text(json.dumps(result,indent=2))
        return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--geometry-filter',action='store_true');p.add_argument('--replay',action='store_true');p.add_argument('--empty',action='store_true');p.add_argument('--selected-run');p.add_argument('--min-pad-coverage',type=float,default=.045);p.add_argument('--arm-kp',type=float);p.add_argument('--arm-kd',type=float);p.add_argument('--lift-gain-scale',type=float,default=1.);p.add_argument('--seeds',default=','.join(map(str,range(20))));a=p.parse_args()
    rclpy.init();n=ContactBackend(a.output,a.geometry_filter,a.replay,a.empty,a.selected_run,a.min_pad_coverage,a.arm_kp,a.arm_kd,a.lift_gain_scale)
    for seed in map(int,a.seeds.split(',')):
        mode='A' if a.empty else 'B'
        if (n.output/f'{mode}_seed_{seed:04d}.json').exists():continue
        result=n.episode(seed)
        if result['category']=='SYSTEM_ERROR':raise RuntimeError(result['detail'])
    n.destroy_node();rclpy.shutdown()
