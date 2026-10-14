"""Three user-registered paired cases. Original files/results stay untouched."""
from pathlib import Path
import copy,datetime,hashlib,json,shutil
ROOT=Path(__file__).resolve().parents[1];old=ROOT/'results/native_g1_visual_input_20261014';out=ROOT/'results/native_g1_visual_reliability_20261015';out.mkdir(parents=True,exist_ok=True)
CASES=['45600','45606','45671'];manifest=dict(reference='9ae41fb',cases=CASES,mode='RGBD_SAM3_HANDLE / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION / INPUT_RELIABILITY_DEVELOPMENT',registered_by_user=True,paired_development_not_held_out=True,physical_attempt_limit_per_case=1,no_between_case_tuning=True,short_target_deg=3.,outcome_post_acquisition_deg=2.,hold_s=1.,sam3_prompt='cabinet door handle',sam3_threshold=.25,one_observed_depth_crop_recovery=True,camera_reobservations=1,hidden_depth_scales=[.75,1.,1.25],hypotheses_are_not_calibrated_confidence_bounds=True,scoring_weights_unchanged=True,controls_unchanged=True,stations={},sources={})
for a in CASES:
 d=out/a;d.mkdir(exist_ok=True);j=json.loads((old/a/'job.json').read_text())
 for key,label in [('source','source'),('asset_root','asset')]:
  dest=d/label
  if not dest.exists():shutil.copytree(j[key],dest)
  j[key]=str(dest)
 for key,name in [('plan','BOOTSTRAP_NO_GRASP.json'),('whole_task_plan','BOOTSTRAP_NO_PATH.json')]:
  shutil.copyfile(old/a/name,d/name);j[key]=str(d/name)
 j.update(episode_id='native_g1_visual_reliability_'+a,output=str(d/'physical_01'),capture_shadow_only=False,visual_reobservations=1,visual_planning_wall_s=450,visual_input_version='observed_depth_crop_completion_3hyp',deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat(),expected_runner_sha256=hashlib.sha256((ROOT/'scripts/run_native_g1_rgbd_short.py').read_bytes()).hexdigest())
 j.pop('_visual_proposal_used',None);(d/'job.json').write_text(json.dumps(j,indent=2));manifest['stations'][a]=json.loads((d/'BOOTSTRAP_NO_GRASP.json').read_text())['trial_candidates'][0]['base']
for name in ['native_g1_visual_input.py','native_g1_observed_regions.py','infer_sam3_observed.py','native_g1_visual_hypothesis_rank.py','run_native_g1_rgbd_short.py','run_native_g1_short_open.py','native_g1_robust_contact_ranking.py','xarm7_native_gripper.py','xarm7_native_execution.py','shared_door_execution.py','precontact_trajectory.py']:
 manifest['sources'][name]=hashlib.sha256((ROOT/'scripts'/name).read_bytes()).hexdigest()
(out/'PREREGISTRATION.json').write_text(json.dumps(manifest,indent=2));print(out)
