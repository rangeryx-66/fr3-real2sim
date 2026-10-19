"""Predeclared matched cases; copy scene inputs, retain only registered stations."""
from pathlib import Path
import copy,datetime,hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[1];old=ROOT/'results/native_g1_robust_rank_20261013';out=ROOT/'results/native_g1_visual_input_20261014';out.mkdir(parents=True,exist_ok=True)
runner=ROOT/'scripts/run_native_g1_rgbd_short.py'
manifest=dict(reference='30f5b30',cases=['45671','45638','45600','45606'],mode='RGBD_SAM3_HANDLE / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION',new_held_out_objects=False,station_source='same registered oracle station; copied base only',attempts_per_case=1,short_target_deg=3.,comparison_post_acquisition_deg=2.,hold_s=1.,visual_prompt='cabinet door handle',visual_reobservations=1,ranking_weights_unchanged=True,perturbations_calibrated_for_vision=False,no_between_case_tuning=True,sources={})
for asset in manifest['cases']:
 d=out/asset;d.mkdir(exist_ok=True);job=json.loads((old/asset/'job.json').read_text());candidate=json.loads((old/asset/'planning/candidate.json').read_text());base=candidate['base'];home=job['robot_start_q']
 for key in ('source','asset_root'):
  src=Path(job[key]);dest=d/('source' if key=='source' else 'asset')
  if not dest.exists():shutil.copytree(src,dest)
  job[key]=str(dest)
 job.update(episode_id='native_g1_visual_'+asset,gpu=3,visual_gpu=6,visual_prompt=manifest['visual_prompt'],visual_reobservations=1,visual_planning_wall_s=450,deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat(),classification=manifest['mode'],expected_runner_sha256=hashlib.sha256(runner.read_bytes()).hexdigest())
 job['wrist_experiment']['sam3']=dict(python='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',pythonpath='/data1/home/rangeryx/sam3_overlay:/data1/home/rangeryx/sam3_official',checkpoint='/data1/home/rangeryx/sam3_1_weights/sam3.1_multiplex.pt')
 job.pop('cached_closing_geometry_audit',None);job['planning_base_seeds']=[]
 job['skill_inputs']['handle_region']=dict(source='RGBD_SAM3',prompt=manifest['visual_prompt'],GT_grasp_fallback=False)
 stub={'base':base,'T':__import__('numpy').eye(4).tolist(),'preplan':[home],'approach':[home]}
 (d/'BOOTSTRAP_NO_GRASP.json').write_text(json.dumps({'trial_candidates':[stub]},indent=2));(d/'BOOTSTRAP_NO_PATH.json').write_text(json.dumps(dict(base=base,goal_state=__import__('numpy').deg2rad(3.),path=[],end_retreat=None),indent=2))
 job['plan']=str(d/'BOOTSTRAP_NO_GRASP.json');job['whole_task_plan']=str(d/'BOOTSTRAP_NO_PATH.json')
 for capture in (True,False):
  j=copy.deepcopy(job);j['capture_shadow_only']=capture;j['output']=str(d/('shadow_01' if capture else 'physical_01'));(d/('shadow_job.json' if capture else 'job.json')).write_text(json.dumps(j,indent=2))
 manifest.setdefault('stations',{})[asset]=base
for f in ['native_g1_visual_input.py','run_native_g1_rgbd_short.py','native_g1_robust_contact_ranking.py','run_native_g1_short_open.py','shared_door_execution.py']:
 manifest['sources'][f]=hashlib.sha256((ROOT/'scripts'/f).read_bytes()).hexdigest()
(out/'PREREGISTRATION.json').write_text(json.dumps(manifest,indent=2));print(out)
