"""Orchestrate frozen grasp -> ObjectScan -> PayloadID capture in one live process."""
from __future__ import annotations
import argparse,json,os,sys,time
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'calibration')]
import plant
from unseen_backend import UnseenBackend
from settling_gate import GateThresholds
from real2sim.object_scan import ObjectScan
from real2sim.payload_skill import PayloadIDSkill
from real2sim.payload_skill_v2 import PayloadIDV2Skill


def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,required=True);p.add_argument('--mode',choices=['GT','ANYGRASP'],default='GT');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gt-path');p.add_argument('--family',action='store_true');p.add_argument('--scan-pass',type=int,default=0);p.add_argument('--scan-profile',choices=['sparse','continuous','held'],default='sparse')
    p.add_argument('--held-regrasp',action='store_true',help='two physical held-object passes with place/regrasp')
    p.add_argument('--held-keyframes',type=int,default=90)
    p.add_argument('--held-frames-per-pose',type=int,default=10)
    p.add_argument('--capture-payload',action='store_true');p.add_argument('--baseline-only',action='store_true');p.add_argument('--center-q',type=Path)
    p.add_argument('--payload-v2',action='store_true',help='two-stage static gravity + dynamic inertia PayloadID; legacy capture remains the default')
    p.add_argument('--mass-com-only',action='store_true',help='calibration-only: record static mass/CoM data and skip dynamic inertia capture')
    p.add_argument('--dynamic-only',action='store_true',help='calibration-only: record only the official-style Fourier dynamic protocol')
    p.add_argument('--gripper-opening-mm',type=float,help='calibration-only Franka Hand opening (0..80 mm)')
    p.add_argument('--skip-scan',action='store_true',help='preserve an existing valid scan and only acquire payload data')
    p.add_argument('--fixed-replay-npz',type=Path,help='calibration-only: replay a captured T_TCP_object stream with a rigid diagnostic attachment')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();backend=UnseenBackend(a.output,GateThresholds())
    try:
        if a.baseline_only:
            plant.command({'op':'reset','seed':a.seed});plant.settle(.8)
            state_now=plant.state();O=ObjectScan._T(state_now['box'],state_now['box_quat']);backend.reset_scene_at(O)
            if a.payload_v2:
                # Baseline data are collected with an empty robot.  The
                # physical target remains in Isaac on the table, but it must
                # not reject harmless open-hand calibration orientations in
                # MoveIt's world-state validity check.  This calibration-only
                # ACM change is reverted when the payload process starts and
                # does not affect the frozen grasp executor.
                backend.target_policy(['fr3_leftfinger','fr3_rightfinger','fr3_hand'])
            if a.gripper_opening_mm is not None:
                opening=float(a.gripper_opening_mm)
                if not (0.0 <= opening <= 80.0):
                    raise ValueError('--gripper-opening-mm must be in [0,80]')
                half=opening/2000.0
                answer=plant.command({'op':'trajectory','names':['fr3_finger_joint1','fr3_finger_joint2'],
                                      'points':[{'t':1.,'q':[half,half]}],'gripper':True})
                if not answer.get('ok'): raise RuntimeError(f'gripper opening command failed: {answer}')
                plant.settle(.2)
            if a.center_q:
                center=json.loads(a.center_q.read_text());target=np.asarray(center['q'])
                state=backend.measured();names=[f'fr3_joint{i}' for i in range(1,8)]
                for n,q in zip(names,target):state.joint_state.position[state.joint_state.name.index(n)]=float(q)
                # The empty-arm baseline must execute the identical measured
                # center pose as the payload leg.  MoveIt's controller
                # execution can report a large tracking error immediately
                # after cold start even when the planned state is valid;
                # that makes the paired baseline unusable and is unrelated to
                # the mass/CoM estimator.  Use the same deterministic center
                # trajectory used by the payload branch below, while keeping
                # the trajectory itself unchanged and calibration-only.
                backend.phase('BASELINE_POSITION')
                current=np.asarray(plant.state()['q'],dtype=float)
                indices=[plant.state()['names'].index(n) for n in names]
                # The empty-arm baseline is a calibration reference, so set
                # the arm to the already measured payload center state and
                # settle before recording.  A force-driven cold-start ramp
                # can leave a large final tracking error even for a valid
                # MoveIt state, which would silently make the pair invalid.
                # This direct state placement is isolated to the empty-arm
                # diagnostic branch; no production execution path uses it.
                answer=plant.command({'op':'calibration_set_joints','q':target.tolist()})
                if not answer.get('ok'):
                    raise RuntimeError(f'baseline center placement failed: {answer}')
                plant.settle(.8)
                actual=np.asarray(plant.state()['q'],dtype=float)[indices]
                if np.max(np.abs(actual-target)) > 0.005:
                    raise RuntimeError(f'baseline center placement tracking error: {float(np.max(np.abs(actual-target)))}')
                if center.get('finger_q') and a.gripper_opening_mm is None:
                    plant.command({'op':'trajectory','names':['fr3_finger_joint1','fr3_finger_joint2'],
                      'points':[{'t':1.,'q':center['finger_q']}],'gripper':True})
                    plant.settle(.2)
            if a.payload_v2:
                skill=PayloadIDV2Skill(backend,plant)
                protocol=(skill.prepare_dynamic_protocol(a.output/'payload_id_v2_protocol.json')
                          if a.dynamic_only else skill.prepare_protocol(a.output/'payload_id_v2_protocol.json'))
                result=skill.run(a.output,protocol,None,'baseline',include_dynamic=not a.mass_com_only,
                                 include_static=not a.dynamic_only)
                (a.output/'baseline_result.json').write_text(json.dumps(result,indent=2));return
            result=PayloadIDSkill(backend,plant).run(a.output/'system_id_baseline.npz',None,'baseline')
            (a.output/'baseline_result.json').write_text(json.dumps(result,indent=2));return
        if a.fixed_replay_npz:
            # Diagnostic-only FixedJoint replay.  This branch deliberately
            # bypasses the frozen grasp executor: it places the arm at the
            # paired calibration center, reconstructs the measured
            # TCP/object transform from an existing normal capture, places the
            # target at that measured pose, and then creates the rigid joint.
            # It is used only to separate contact dynamics from the torque
            # baseline; no GT mass/CoM or GT pose is consumed.
            if not a.center_q:
                raise RuntimeError('--fixed-replay-npz requires --center-q')
            source_npz=Path(a.fixed_replay_npz).resolve()
            if not source_npz.exists():
                raise FileNotFoundError(source_npz)
            center=json.loads(a.center_q.read_text())
            plant.command({'op':'reset','seed':a.seed});plant.settle(.8)
            arm_q=np.asarray(center['q'],dtype=float).reshape(7)
            answer=plant.command({'op':'calibration_set_joints','q':arm_q.tolist()})
            if not answer.get('ok'):
                raise RuntimeError(f'fixed replay arm placement failed: {answer}')
            plant.settle(.8)
            if center.get('finger_q'):
                plant.command({'op':'trajectory','names':['fr3_finger_joint1','fr3_finger_joint2'],
                               'points':[{'t':1.,'q':center['finger_q']}],'gripper':True})
                plant.settle(.3)
            with np.load(source_npz,allow_pickle=True) as loaded:
                rel=np.asarray(loaded['T_TCP_object'],dtype=float)
                valid=np.isfinite(rel).all(axis=(1,2))
                if 'static_pose_id' in loaded:
                    pid=np.asarray(loaded['static_pose_id']).reshape(-1)
                    hold=np.asarray(loaded.get('static_hold',np.ones(len(rel))),dtype=bool).reshape(-1)
                    candidates=np.flatnonzero(valid & hold & (pid==0))
                    if len(candidates)==0:
                        candidates=np.flatnonzero(valid & hold & (pid>=0))
                else:
                    candidates=np.flatnonzero(valid)
                if len(candidates)==0:
                    raise RuntimeError('fixed replay has no valid T_TCP_object samples')
                # Use a robust center-window transform, preserving the
                # measured attachment orientation without using GT.
                candidates=candidates[:min(len(candidates),240)]
                t_rel=np.median(rel[candidates,:3,3],axis=0)
                r_rel=Rotation.from_matrix(rel[candidates,:3,:3]).mean().as_matrix()
            current=plant.state()
            H_tcp=ObjectScan._T(current['tcp'],current['tcp_quat'])
            T_rel=np.eye(4);T_rel[:3,:3]=r_rel;T_rel[:3,3]=t_rel
            T_object=H_tcp@T_rel
            q_wxyz=np.roll(Rotation.from_matrix(T_object[:3,:3]).as_quat(),1)
            answer=plant.command({'op':'calibration_set_object_pose',
                                  'position':T_object[:3,3].tolist(),
                                  'quat_wxyz':q_wxyz.tolist()})
            if not answer.get('ok'):
                raise RuntimeError(f'fixed replay object placement failed: {answer}')
            # Attach immediately after placement.  A free dynamic object would
            # otherwise fall/rotate during a settling delay before the
            # diagnostic joint is created, changing the replay transform.
            attachment=plant.command({'op':'diagnostic_fixed_joint'})
            if not attachment.get('ok'):
                raise RuntimeError(f'fixed replay attachment failed: {attachment}')
            # Arm the support monitor at the placed object's current origin;
            # the object is already in free space and the rigid attachment is
            # the only diagnostic aid.  The ordinary PayloadID safety checks
            # still require bilateral contact and a settled free-space state.
            plant.command({'op':'support_begin','initial_z':float(T_object[2,3])})
            plant.settle(.8)
            replay_protocol=source_npz.parent/'payload_id_v2_protocol.json'
            if not replay_protocol.exists():
                replay_protocol=a.output/'payload_id_v2_protocol.json'
            protocol=json.loads(replay_protocol.read_text())
            fake_result={'success':True,
                         'flags':{'PICKED':True,'RETAINED':True,
                                  'CLEAR_TABLE':True,'lift_ge_8cm':True},
                         'final_support':{'passed':True,'currently_clear':True,
                                           'category':'STABLE',
                                           'flags':{'PICKED':True,'RETAINED':True,
                                                    'CLEAR_TABLE':True,
                                                    'lift_ge_8cm':True}}}
            skill=PayloadIDV2Skill(backend,plant)
            payload=skill.run(a.output,protocol,fake_result,'payload',include_dynamic=not a.mass_com_only)
            summary={'grasp':'diagnostic_fixed_replay','scan_frames':0,'scan_pass':a.scan_pass,
                     'scan_profile':a.scan_profile,'held_regrasp':False,
                     'held_regrasp_info':None,'payload':payload,
                     'excitation_center':arm_q.tolist(),
                     'diagnostic_attachment':attachment,
                     'fixed_replay_source':str(source_npz),
                     'fixed_replay_T_TCP_object':T_rel.tolist(),
                     'gt_used_for_estimation':False}
            (a.output/'acquisition_result.json').write_text(json.dumps(summary,indent=2))
            print(json.dumps(summary));return
        result=backend.episode(a.seed,a.mode,a.gt_path,a.family)
        if not result.get('success'):raise RuntimeError('frozen grasp failed: '+str(result.get('category')))
        if a.capture_payload and os.environ.get('PAYLOAD_ATTACHMENT_MODE','NORMAL').upper() == 'FIXED':
            # Diagnostic-only comparison.  The production executor never
            # sets PAYLOAD_ATTACHMENT_MODE=FIXED and therefore never creates
            # this joint.
            result['diagnostic_attachment']=plant.command({'op':'diagnostic_fixed_joint'})
        records=[]
        held_info=None
        if not a.skip_scan:
            scan=ObjectScan(backend,plant,a.output/'scan',result['target'])
            if a.held_regrasp:
                pass0,pass1,held_info=scan.run_held_regrasp(
                    result, keyframes=a.held_keyframes,
                    frames_per_pose=a.held_frames_per_pose,
                )
                (a.output/'scan'/'pass_00.json').write_text(json.dumps(pass0,indent=2))
                (a.output/'scan'/'pass_01.json').write_text(json.dumps(pass1,indent=2))
                records=pass0+pass1
                (a.output/'held_regrasp.json').write_text(json.dumps(held_info,indent=2))
            else:
                records=scan.run_pass(result,a.scan_pass,profile=a.scan_profile)
                pass_file=a.output/'scan'/f'pass_{a.scan_pass:02d}.json';pass_file.write_text(json.dumps(records,indent=2))
            pass_files=sorted((a.output/'scan').glob('pass_*.json'))
            passes=[json.loads(path.read_text()) for path in pass_files]
            ObjectScan.merge_passes(a.output/'scan',result['target'],passes,sources={'grasp_result':str(a.output/(result['id']+'.json'))})
        if a.center_q:
            center=json.loads(a.center_q.read_text());target=np.asarray(center['q'])
            names=[f'fr3_joint{i}' for i in range(1,8)]
            # The frozen grasp already ends at (or very near) the calibration
            # center.  Calling MoveIt's global planner again while the target
            # is attached can block on a stale world/ACM state.  Calibration
            # is allowed to use the deterministic, already validated center
            # trajectory; this branch is payload-only and never changes the
            # grasp executor.  Keep the original MoveIt move for baseline/
            # empty-arm captures.
            current=np.asarray(plant.state()['q'],dtype=float)
            indices=[plant.state()['names'].index(n) for n in names]
            current_arm=current[indices]
            if np.max(np.abs(current_arm-target)) > 0.02:
                diagnostic_fixed = os.environ.get('PAYLOAD_ATTACHMENT_MODE','NORMAL').upper() == 'FIXED'
                if a.payload_v2 and diagnostic_fixed:
                    # FixedJoint is a diagnostic-only rigid attachment.  Put
                    # its arm directly at the paired center-q before the
                    # static protocol so empty/payload samples start from the
                    # same measured pose; this removes a cold-start transient
                    # from the first static windows without touching the
                    # normal grasp executor.
                    answer=plant.command({'op':'calibration_set_joints','q':target.tolist()})
                    if not answer.get('ok'): raise RuntimeError(f'fixed diagnostic center placement failed: {answer}')
                    plant.settle(.8)
                    actual=np.asarray(plant.state()['q'],dtype=float)[indices]
                    if np.max(np.abs(actual-target)) > 0.005:
                        raise RuntimeError(f'fixed diagnostic center tracking error: {float(np.max(np.abs(actual-target)))}')
                elif a.payload_v2:
                    backend.phase('PAYLOAD_ID_CENTER')
                    points=[{'t':0.0,'q':current_arm.tolist()},
                            {'t':1.5,'q':target.tolist()}]
                    answer=plant.command({'op':'trajectory','names':names,'points':points},timeout=120)
                    if not answer.get('ok'): raise RuntimeError(f'payload center trajectory failed: {answer}')
                else:
                    state=backend.measured()
                    for n,q_target in zip(names,target):state.joint_state.position[state.joint_state.name.index(n)]=float(q_target)
                    backend.phase('PAYLOAD_ID_CENTER');backend.execute(backend.plan(backend.measured(),state),'NO_PLAN')
            plant.settle(.3)
        arm_names=[f'fr3_joint{i}' for i in range(1,8)];state=plant.state();q=[state['q'][state['names'].index(n)] for n in arm_names]
        finger_names=['fr3_finger_joint1','fr3_finger_joint2'];finger_q=[state['q'][state['names'].index(n)] for n in finger_names]
        (a.output/'excitation_center.json').write_text(json.dumps({'q':q,'names':arm_names,'finger_q':finger_q,'finger_names':finger_names},indent=2))
        payload=None
        calibration_contact=None
        # The production grasp has already completed and remains frozen.  For
        # the calibration-only PayloadID branch, switch the hand to the
        # explicit bilateral force controller before excitation.  This avoids
        # conflating a weak position-only squeeze with robot-torque bias; the
        # setting is supplied by launch_real2sim via CALIBRATION_FORCE_N and is
        # never used by AnyGrasp/MoveIt execution.
        if a.capture_payload and a.payload_v2:
            force=float(os.environ.get('CALIBRATION_FORCE_N','30.0'))
            if not (0.0 < force <= 60.0):
                raise ValueError(f'CALIBRATION_FORCE_N must be in (0,60], got {force}')
            answer=plant.command({'op':'calibration_force','force_N':force})
            if not answer.get('ok'):
                raise RuntimeError(f'calibration force command failed: {answer}')
            start_force=plant.state()['t']; stable_since=None; last_force=[0.0,0.0]
            while plant.state()['t']-start_force < 5.0:
                cs=plant.state('calibration'); last_force=np.asarray(cs['calibration']['filtered_force_N'],float).reshape(-1)
                bilateral=last_force.size>=2 and bool(np.min(last_force[:2])>.1)
                on_target=bilateral and abs(float(np.sum(last_force[:2]))-force)<=max(2.0,.2*force)
                stable_since=cs['t'] if on_target and stable_since is None else stable_since if on_target else None
                if stable_since is not None and cs['t']-stable_since>=.3: break
                time.sleep(.01)
            calibration_contact={'mode':'CALIBRATION_FORCE','target_total_force_N':force,
                                 'filtered_force_N':np.asarray(last_force).tolist(),
                                 'bilateral':bool(last_force.size>=2 and np.min(last_force[:2])>.1),
                                 'target_reached':bool(stable_since is not None),
                                 'mu':float(os.environ.get('CALIBRATION_MU','0.7'))}
            if not calibration_contact['target_reached']:
                raise RuntimeError(f'calibration force target not reached: {calibration_contact}')
        if a.capture_payload:
            if a.payload_v2:
                skill=PayloadIDV2Skill(backend,plant)
                protocol=(skill.prepare_dynamic_protocol(a.output/'payload_id_v2_protocol.json')
                          if a.dynamic_only else skill.prepare_protocol(a.output/'payload_id_v2_protocol.json'))
                payload=skill.run(a.output,protocol,result,'payload',include_dynamic=not a.mass_com_only,
                                  include_static=not a.dynamic_only)
            else:
                payload=PayloadIDSkill(backend,plant).run(a.output/'system_id_payload.npz',result,'payload')
        summary={'grasp':result['id'],'scan_frames':len(records),'scan_pass':a.scan_pass,'scan_profile':a.scan_profile,'held_regrasp':bool(a.held_regrasp),'held_regrasp_info':held_info,'payload':payload,'excitation_center':q,'calibration_contact':calibration_contact}
        (a.output/'acquisition_result.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
    finally:backend.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
