"""Existing records plus external wall snapshots; no simulation execution."""
from pathlib import Path
import json,collections,datetime
import numpy as np
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_20261013');reg=json.loads((O/'REGISTERED_OBJECTS.json').read_text());observer=collections.defaultdict(list)
p=O/'LAST_FOUR_EXTERNAL_WALL_SAMPLES.jsonl'
if p.exists():
 for line in p.read_text().splitlines():
  x=json.loads(line);observer[x['asset_id']].append(x)
results=[]
for c in reg['objects']:
 a=c['asset_id'];d=O/a;p=d/'physical_01';r=json.loads((p/'report.json').read_text());metrics=json.loads((p/'ROUND_ANALYSIS.json').read_text());profile=json.loads((p/'timing_profile.json').read_text());plan=json.loads((d/'planning/planning_summary.json').read_text());spans=[];previous=None
 for x in observer[a]:
  unix=x['source_file_mtime_ns']/1e9
  if not spans or spans[-1]['phase']!=x['phase']:
   if spans:spans[-1]['end_boundary_unix_s']=[previous,unix]
   spans.append({'phase':x['phase'],'start_boundary_unix_s':None if previous is None else [previous,unix],'first_observed_unix_s':unix,'last_observed_unix_s':unix,'first_sim_s':x['physics_t_s'],'last_sim_s':x['physics_t_s']})
  spans[-1]['last_observed_unix_s']=unix;spans[-1]['last_sim_s']=x['physics_t_s'];previous=unix
 for span in spans:
  s=span['start_boundary_unix_s'];e=span.get('end_boundary_unix_s');span['observed_wall_span_s']=span['last_observed_unix_s']-span['first_observed_unix_s'];span['wall_duration_bounds_s']=None if s is None or e is None else [max(0,e[0]-s[1]),max(0,e[1]-s[0])]
 chunks=[f for folder in ['observations_chunks','physics_steps_chunks','closure_commands_chunks'] for f in (p/folder).glob('*.gz')];first_chunk=min(f.stat().st_mtime for f in chunks);export_done=(p/'EXPORT_COMPLETE.json').stat().st_mtime;latest_progress=(p/'progress.json').stat().st_mtime;initwall=profile['phases'].get('initialization',{}).get('wall_s');tail=max(0,(p/'report.json').stat().st_mtime-latest_progress);exportspan=max(0,export_done-first_chunk)
 init_cost=profile.get('costs_wall_s',{});totalcost=sum(init_cost.values());stepwall=profile['phases'].get('initialization',{}).get('stationary_command_step_wall_s',0.)
 results.append({'asset_id':a,'planning_wall_s':plan['wall_s'],'simulation_s':r['simulation_s'],'runner_wall_s':r['wall_s'],'physical_steps':r['physics_steps'],'simulation_realtime_factor':r['simulation_s']/r['wall_s'],'mean_step_wall_ms_including_initialization_and_export':1000*r['wall_s']/r['physics_steps'],'phase_simulation_s':metrics['phase_simulation_s'],'initialization_measured_profile_wall_s':initwall,'saved_profile_is_only_initialization_snapshot':profile['active_phase']=='initial_approach_grasp','initialization_step_costs_wall_s':init_cost,'initialization_instrumented_cost_fraction':{k:v/totalcost for k,v in init_cost.items()} if totalcost else {},'initialization_step_wall_measured_s':stepwall,'initialization_step_costs_unaccounted_wall_s':stepwall-totalcost,'external_phase_wall_snapshots':spans,'buffer_export_file_write_span_s_lower_bound':exportspan,'final_recording_step_plus_export_and_result_write_s_upper_bound':tail,'runner_wall_minus_initialization_and_final_tail_s_approx':None if initwall is None else max(0,r['wall_s']-initwall-tail),'video_frames':int(metrics['video']['streams'][0]['nb_frames']),'video_duration_s':float(metrics['video']['streams'][0]['duration'])})
summary={'cases':results,'total_planning_wall_s':sum(x['planning_wall_s'] for x in results),'total_physical_simulation_s':sum(x['simulation_s'] for x in results),'total_runner_wall_s':sum(x['runner_wall_s'] for x in results),'aggregate_simulation_realtime_factor':sum(x['simulation_s'] for x in results)/sum(x['runner_wall_s'] for x in results),'total_initialization_profile_wall_s':sum(x['initialization_measured_profile_wall_s'] or 0 for x in results),'total_final_recording_step_export_write_upper_bound_s':sum(x['final_recording_step_plus_export_and_result_write_s_upper_bound'] for x in results),'notes':['Simulation phase durations are exact sample counts / 240 Hz.','First six lack per-phase host timestamps beyond initialization; unavailable values are not allocated proportional to simulation time.','External last-four snapshots observe progress updated once per simulation second: duration bounds exclude unknown boundary position inside each update interval; first/last observed phases can be censored.','Saved subsystem timers cover only the initialization hold, not the complete episode. Do not extrapolate subsystem proportions as full-task measurements.','Export lower bound is first buffered chunk mtime to EXPORT_COMPLETE; final-tail upper bound includes up to one progress-update interval, export and video finalization. No physics fidelity or logging rate was changed.']}
(O/'TEN_CASE_RUNTIME_ANALYSIS.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='cases'},indent=2))
