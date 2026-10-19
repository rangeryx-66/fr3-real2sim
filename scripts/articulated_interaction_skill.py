"""Agent-callable segmented interaction, effort and reconstruction inputs."""
import argparse
import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime,timedelta
from pathlib import Path
from zoneinfo import ZoneInfo
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def policy(kind,goal):
    targets=([0,5,10,15,20] if kind=='revolute' else [0,.02,.04,.06,.08]) if goal=='open_for_multistate_capture' else ([0,2.5,5] if kind=='revolute' else [0,.01,.02])
    return {'joint_type':kind,'goal':goal,'targets':targets,
            'capture_interval':2.5 if kind=='revolute' else .01,
            'minimum_capture_separation':.5 if kind=='revolute' else .002,
            'hold_s':2.,'segment_m':.001,'segment_timeout_s':5.,'maximum_segments':120,
            'maximum_sim_s':450.,'maximum_path_m':.12}


def finish(output):
    from articulated_interaction_skill.effort import summarize_effort
    from articulated_interaction_skill.export import export_capture
    root=Path(output)
    if (root/'effort_segments.json').exists():summarize_effort(root)
    from articulated_interaction_skill.evaluation import evaluate_capture
    evaluation=evaluate_capture(root)
    capture=json.loads((root/'multistate_capture.json').read_text())
    if capture.get('states') and capture['status']!='RUNNING' and not capture['states'][-1].get('stop_reason'):
        capture['states'][-1]['stop_reason']=capture['status']
        last=capture['states'][-1];(root/'states'/f"state_{last['state_id']:03d}"/'state.json').write_text(json.dumps(last,indent=2))
        (root/'multistate_capture.json').write_text(json.dumps(capture,indent=2))
    for state in capture['states']:
        state.setdefault('object_id',capture['object_id']);state.setdefault('moving_part_id',capture['moving_part_id'])
        state.setdefault('robot_base_pose_units',['m','m','m','deg'])
        if not state.get('grasp_pose_kind'):
            state['T_world_initial_grasp']=state.get('T_world_grasp')
            state['T_world_grasp']=state['T_world_ee']
            state['grasp_pose_kind']='current measured official TCP proxy; contact centroid is not separately measured'
        (root/'states'/f"state_{state['state_id']:03d}"/'state.json').write_text(json.dumps(state,indent=2))
    (root/'multistate_capture.json').write_text(json.dumps(capture,indent=2))
    result=export_capture(root)
    effort=json.loads((root/'effort_summary.json').read_text()) if (root/'effort_summary.json').exists() else {}
    rows=['# Articulated interaction capture','',f"Object {capture['object_id']} / {capture['joint_family_requested']}; status: {capture['status']}",'',
          '| State | Estimated value | Unit | Stable hold | Views |','|---|---:|---|---|---:|']
    for s in capture['states']:rows.append(f"| {s['state_id']} | {s['estimated_articulation_state']:.4f} | {capture['units']} | {s['grasp_still_stable']} | {len(s['views'])} |")
    rows+=['','## Effort','', '| Segment | Start state | Breakaway proxy N | Moving effective N | Stop |','|---|---:|---|---|---|']
    for s in effort.get('moving_effort_vs_state',[]):rows.append(f"| {s['index']} | {s['start_estimated_state']:.4f} | {s['breakaway_effective_force_n']} | {s['moving_effective_force_n']} | {s.get('stop_reason')} |")
    rows+=['','Effort source is recorded per segment: full PhysX normal+friction measurement when available, otherwise explicitly labelled commanded-effort proxy. Normal-only contact reports are diagnostic, not full tangential force. Neither source proves exact minimum breakaway or friction.','',
           f"Independent post-run check: {evaluation.get('physically_distinct_captured_states',0)} physically distinct captured states; >=3 verified: {evaluation.get('three_state_capture_verified',False)}.",'',
           '## Reconstruction inputs','',f"{result['states']} states; {result['pairs']} Ditto tensor pairs.",
           'ART-style multi-state RGB/camera manifest is ready; official ART parser/model execution is not claimed. Ditto tensors match official pc_start/pc_end input shape. HouseDitto pairs and optional image/scan manifest are supplied.',
           'Instance masks come from the simulator render annotator. Estimated labels are not GT labels. Partial/failed captures remain in the package.',
           'Twin update stores estimated articulation, observed range and effort only; no backend reconstruction has yet run and full joint limits are not inferred.']
    (root/'REPORT.md').write_text('\n'.join(rows)+'\n')
    result.update(schema='articulated-interaction-skill-result-v1',mode='SIMULATION_MULTISTATE_CAPTURE',physics_fitting=False,status=capture['status'],capture_goal_satisfied=len(capture['states'])>=3 and all(x['grasp_still_stable'] for x in capture['states']) and evaluation.get('three_state_capture_verified',False),all_requested_targets_reached=capture['status']=='MULTISTATE_TARGET_COMPLETE',physical_state_check=evaluation,effort_sources=sorted({x['effort_measurement_kind'] for x in effort.get('moving_effort_vs_state',[])}),mid_grasp_base_motion=False,reconstruction_executed=False)
    (root/'skill_result.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--job',type=Path,help='Existing physical-contact job; preserves geometry, grasp, controller and deployment')
    p.add_argument('--joint-type',choices=['revolute','prismatic'])
    p.add_argument('--goal',choices=['open_for_multistate_capture','probe_articulation_and_effort'],default='open_for_multistate_capture')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--stage',choices=['prepare','run','export'],default='run')
    p.add_argument('--asset-root',type=Path,help='New prepared proxy asset; --job supplies the frozen deployment template for prepare')
    p.add_argument('--candidate',type=int,help='Index within the already planned safe candidate list; never generates new poses');p.add_argument('--gpu',type=int);p.add_argument('--targets',type=float,nargs='+')
    p.add_argument('--deadline-shanghai');p.add_argument('--ffmpeg',type=Path);p.add_argument('--manifest',type=Path,help='Frozen list of object jobs; all failures remain in summary');a=p.parse_args()
    if a.manifest:
        manifest=json.loads(a.manifest.read_text());out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);rows=[]
        (out/'frozen_capture_manifest.json').write_text(json.dumps(manifest,indent=2))
        for e in manifest['episodes']:
            target=out/e['episode_id'];cmd=[sys.executable,str(Path(__file__).resolve()),'--job',e['job'],'--joint-type',e['joint_type'],'--goal',e.get('goal',a.goal),'--stage',a.stage,'--output',str(target)]
            if a.ffmpeg:cmd+=['--ffmpeg',str(a.ffmpeg)]
            if a.deadline_shanghai:cmd+=['--deadline-shanghai',a.deadline_shanghai]
            if a.gpu is not None:cmd+=['--gpu',str(a.gpu)]
            code=subprocess.run(cmd,cwd=ROOT).returncode
            data=json.loads((target/'multistate_capture.json').read_text()) if (target/'multistate_capture.json').exists() else {}
            rows.append({'episode_id':e['episode_id'],'returncode':code,'status':data.get('status','NO_CAPTURE'),'captured_states':len(data.get('states',[])),'output':str(target)})
            (out/'batch_summary.json').write_text(json.dumps(rows,indent=2))
        lines=['# Articulated skill batch','','| Episode | Captured states | Physical goal verified | Status |','|---|---:|---|---|']
        for row in rows:
            file=Path(row['output'])/'skill_result.json';data=json.loads(file.read_text()) if file.exists() else {}
            lines.append(f"| {row['episode_id']} | {row['captured_states']} | {data.get('capture_goal_satisfied',False)} | {row['status']} |")
        lines+=['','All failed episodes remain in batch_summary.json. Targets are requested states; safe partial capture is distinct from reaching every target. Effort provenance is recorded per segment. No backend reconstruction or exact friction identification is claimed.']
        (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
        print(json.dumps(rows));return
    if a.stage=='export':print(json.dumps(finish(a.output)));return
    if not a.job or not a.joint_type:p.error('--job and --joint-type are required for run')
    if a.ffmpeg:
        if not a.ffmpeg.is_file():p.error('--ffmpeg binary not found')
        os.environ['PATH']=str(a.ffmpeg.resolve().parent)+os.pathsep+os.environ.get('PATH','')
    if not shutil.which('ffmpeg'):p.error('ffmpeg required for continuous video; supply --ffmpeg /path/to/ffmpeg')
    job=copy.deepcopy(json.loads(a.job.read_text()));out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    job.update(output=str(out),mode='skill_multistate',episode_id='multistate_'+str(job.get('episode_id','object')),skill=policy(a.joint_type,a.goal))
    if a.targets:
        if a.targets[0]!=0 or any(y<=x for x,y in zip(a.targets,a.targets[1:])):p.error('targets must start at 0 and increase; units degrees/meters')
        job['skill']['targets']=a.targets
    if a.gpu is not None:job['gpu']=a.gpu
    if a.candidate is not None:
        if not 0<=a.candidate<12:p.error('candidate index must be within the frozen twelve-candidate budget')
        planned=json.loads(Path(job['plan']).read_text()).get('trial_candidates',[])
        if a.candidate>=len(planned):p.error('candidate not in preflight-approved list')
        job['candidate']=a.candidate
    now=datetime.now(ZoneInfo('Asia/Shanghai'));deadline=now.replace(hour=5,minute=0,second=0,microsecond=0)
    if deadline<=now:deadline+=timedelta(days=1)
    job['deadline_shanghai']=a.deadline_shanghai or deadline.isoformat()
    if datetime.fromisoformat(job['deadline_shanghai']).tzinfo is None:p.error('deadline must include timezone, e.g. +08:00')
    if a.stage=='prepare':
        if not a.asset_root:p.error('--asset-root required for prepare')
        from articulated_interaction_skill.prepare import prepare_job
        print(json.dumps(prepare_job(ROOT,a.job,a.asset_root,out,a.joint_type,a.gpu or 0,job['deadline_shanghai'],job['skill'])))
        return
    job['wall_clock_budget_s']=min(float(job.get('wall_clock_budget_s',2400)),2400)
    path=out/'job_private.json';path.write_text(json.dumps(job,indent=2))
    files=list((ROOT/'articulated_interaction_skill').glob('*.py'))+[ROOT/'scripts/run_articulated_skill_episode.py']
    (out/'skill_code_hashes.json').write_text(json.dumps({str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
    result=subprocess.run([sys.executable,str(ROOT/'scripts/run_articulated_skill_episode.py'),'--job',str(path)],cwd=ROOT)
    if (out/'multistate_capture.json').exists():finish(out)
    else:(out/'skill_result.json').write_text(json.dumps({'status':'PROCESS_FAILED_BEFORE_CAPTURE','returncode':result.returncode,'capture_goal_satisfied':False,'job':str(path),'physical_success_claimed':False},indent=2))
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
