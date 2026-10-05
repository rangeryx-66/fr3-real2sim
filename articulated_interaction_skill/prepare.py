"""Reuse frozen deployment/planning/mobile recovery for a prepared proxy asset."""
import copy,json,time,subprocess,sys,hashlib
import xml.etree.ElementTree as ET
from pathlib import Path


def prepare_job(root,reference_job,asset,output,kind,gpu,deadline,skill_policy):
    from interactive_twin.planning import make_deployment_template,prepare_deployment
    from interactive_twin_recovery.mobile import recover,eligible
    root=Path(root);asset=Path(asset).resolve();out=Path(output).resolve();out.mkdir(parents=True,exist_ok=True)
    J=copy.deepcopy(json.loads(Path(reference_job).read_text()));meta=json.loads((asset/'manifest.json').read_text())
    selected=meta['interaction_geometry']['selection'];tree=ET.parse(asset/'urdf'/f"{meta['asset_id']}.urdf").getroot()
    joint=next(j for j in tree.findall('joint') if j.get('name')==selected['joint_name'])
    if joint.get('type')!=kind:raise ValueError('PREPARED_JOINT_FAMILY_MISMATCH')
    moving={joint.find('child').get('link')}
    while True:
        nxt=moving|{j.find('child').get('link') for j in tree.findall('joint') if j.find('parent').get('link') in moving}
        if nxt==moving:break
        moving=nxt
    # Fix selection metadata in this NEW asset copy only. Geometry untouched.
    if meta['joint_name']!=joint.get('name'):
        meta.update(joint_name=joint.get('name'),joint_type=kind,moving_links=sorted(moving))
        (asset/'manifest.json').write_text(json.dumps(meta,indent=2))
    template=make_deployment_template(J['asset_root'],Path(J['source'])/'report.json')
    episode={'episode_id':'multistate_'+str(meta['asset_id']),'deployment':{'yaw_perturbation_deg':0.,'translation_in_initial_handle_frame_m':[0,0,0],'along_handle_offset_m':0.},'initialization_only':{'joint_position_rad':0.}}
    D=prepare_deployment(episode,asset,template,out/'deployment')
    # A different asset must retain its own native passive parameters, not
    # inherit a DEV physics-experiment plant override from the reference job.
    discarded=J.pop('plant',None)
    J.pop('mobile_route',None);J.pop('mobile_platform',None)
    J.update(frozen_proxy_sha256=hashlib.sha256((asset/'manifest.json').read_bytes()).hexdigest(),source=D['source'],asset_root=str(asset),output=str(out/'preflight'),plan=str(out/'preflight/plan.json'),
             initial_visual=str(out/'deployment/initial_visual_handle_world.json'),mode='plan',episode_id=episode['episode_id'],gpu=gpu,
             deadline_shanghai=deadline,skill=skill_policy,planning_budget_s=600,wall_clock_budget_s=2400)
    path=out/'preflight_job.json';path.write_text(json.dumps(J,indent=2))
    result=subprocess.run([sys.executable,str(root/'scripts/run_articulated_skill_episode.py'),'--job',str(path)],cwd=root)
    plan_path=Path(J['plan'])
    if result.returncode or not plan_path.exists():raise RuntimeError('PREFLIGHT_INFRASTRUCTURE_FAILURE')
    plan=json.loads(plan_path.read_text());initial=D['source_report']['robot_base_pose'];recovery=None
    if not plan.get('trial_candidates') and eligible(plan):
        policy=json.loads((root/'configs/interactive_twin_recovery.yaml').read_text())['mobile']['search']
        recovery=recover(root,json.loads((Path(J['output'])/'cooked_initial.json').read_text()),plan,D['initial_visual_handle_world'],initial,policy,seed=732061660,deadline=time.time()+600)
        (out/'mobile_preflight.json').write_text(json.dumps(recovery,indent=2))
        if recovery['selected']:
            choice=recovery['selected'];plan=choice['plan'];plan_path=out/'recovered_plan.json';plan_path.write_text(json.dumps(plan,indent=2))
            source=out/'recovered_source';source.mkdir(exist_ok=True);report=copy.deepcopy(D['source_report']);report['robot_base_pose']=choice['base'];(source/'report.json').write_text(json.dumps(report,indent=2))
            J.update(source=str(source),plan=str(plan_path),mobile_platform=True,mobile_route=choice['route']);J['mobile_route']['deadline_unix_s']=__import__('datetime').datetime.fromisoformat(deadline).timestamp()
    J.update(mode='skill_multistate',output=str(out/'execution'),candidate=0)
    job=out/'execution_job.json';job.write_text(json.dumps(J,indent=2))
    status={'status':'READY' if plan.get('trial_candidates') else 'DEPLOYMENT_UNREACHABLE','execution_job':str(job),
            'discarded_unrelated_reference_plant':discarded,'dataset_geometry_changed':False,
            'mobile_recovered':bool(recovery and recovery['selected']),'grasp_family_changed':False}
    (out/'prepare_result.json').write_text(json.dumps(status,indent=2));return status
