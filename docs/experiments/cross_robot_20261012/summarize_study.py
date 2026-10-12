"""Read-only aggregation of the registered offline study and archived physics."""
from pathlib import Path
import json,csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/cross_robot_20261012';robots=['piper','nero','xarm7','fr3'];assets=['45134','45443','45176','45415'];rows=[];absrows=[];decision=json.loads((O/'PHYSICAL_MODEL_DECISION.json').read_text())
for robot in robots:
 for asset in assets:
  p=O/'offline'/robot/asset;s=json.loads((p/'SUMMARY.json').read_text());f=json.loads((p/'same_grasp_same_station.json').read_text());b=json.loads((p/'same_grasp_station_search.json').read_text())['best'];ownpath=p/'own_plan/candidate.json';own=json.loads(ownpath.read_text()) if ownpath.exists() else None
  row={'robot':robot,'asset':asset,'dof':6 if robot=='piper' else 7,'same_grasp_same_PiPER_station_range_deg':s['same_grasp_fixed_station'],'same_station_approach':f.get('approach'),'same_station_blocker':f['blocker'],'same_station_limiting_joint':f.get('limiting'),'same_grasp_optimized_station_range_deg':s['same_grasp_optimized_station'],'same_grasp_optimized_approach':b.get('approach') if b else None,'same_grasp_optimized_base':b['base'] if b else None,'own_grasp_plan_found':bool(own),'own_grasp_range_deg':float(np.rad2deg(own['planned_reached_state'])) if own else None,'own_grasp_base':own['base'] if own else None,'own_grasp_contact_support':own['contact_geometry_audit']['support_score'] if own else None,'own_grasp_contact_aperture_m':own['contact_geometry_audit']['physical_aperture_m'] if own else None,'own_grasp_minimum_approach_arc_margin_rad':own['minimum_approach_arc_margin_rad'] if own else None,'own_grasp_blocker':own.get('planned_blocker') if own else json.loads((p/'own_plan/planning_summary.json').read_text())['blocker'],'own_grasp_limiting_joint':None,'wall_s':s['wall_s'],'mount_status':s['mount_status'],'physical_status':decision['robots'][robot]['physical_status'],'new_physical_attempts':0,'physical_metrics':None}
  if own and own['path']:
   c=json.loads((O/'ROBOT_MODELS.json').read_text())[robot];import xml.etree.ElementTree as ET
   root=ET.parse(c['urdf']).getroot();limits=np.array([[float(root.find(f"joint[@name='{n}']/limit").get(k)) for k in ['lower','upper']] for n in c['arm_joint_names']]);q=np.array(own['path'][-1]['q']);margin=np.minimum(q-limits[:,0],limits[:,1]-q);idx=int(margin.argmin());row['own_grasp_limiting_joint']={'joint':c['arm_joint_names'][idx],'last_q_rad':float(q[idx]),'last_margin_rad':float(margin[idx]),'limits_rad':limits[idx].tolist()}
  if robot=='piper':
   m=json.loads((ROOT/'results/door_transfer4_20261012'/asset/'physical_01/PHYSICAL_METRICS.json').read_text());row['physical_metrics']={'source':'ARCHIVED_81ab4f5_NOT_A_NEW_TRIAL','actual_deg':m['max_actual_angle_after_valid_grasp_deg'],'valid_grasp':m['bilateral_grasp_verified'],'slip_mm':m['max_handle_relative_translation_m_after_grasp']*1000,'minimum_margin_rad':m['minimum_joint_margin_rad_after_grasp'],'initial_pad_loads_n':m['grasp_event']['pad_loads_n'],'peak_pad_loads_n':m['peak_pad_loads_n_all'],'max_loaded_tcp_mm_s':m['max_TCP_velocity_m_s_by_phase']['after_valid_grasp']*1000,'sim_s':m['simulation_s'],'wall_s':m['wall_s'],'physical_30deg_PASS':m['physical_PASS_30deg'],'contact_loss_intervals':m['contact_loss_intervals_after_grasp']}
  rows.append(row)
  a=p/'redundancy_ablation.json'
  if a.exists():
   d=json.loads(a.read_text())
   for v in d['variants']:absrows.append({'robot':robot,'asset':asset,'joint_fixed':v['joint'],'fixed_q_rad':v['fixed_rad'],'fixed_source':v['value_source'],'full7_range_deg':d['full7_range_deg'],'fixed6_range_deg':v['range_deg'],'initial_ik':v['initial_ik'],'approach':v['approach']['feasible'],'blocker':v['blocker'],'limiting':v['limiting'],'minimum_margin_rad':v.get('minimum_margin_rad'),'same_base':d['base'],'fallback_station':d.get('fallback_to_registered_PiPER_station_due_to_no_full7_feasible_station',False)})
result={'classification':'ORACLE_HANDLE / KNOWN_MODEL / CROSS_ROBOT_ABLATION','reference_commit':'81ab4f5','cases':rows,'ablations':absrows,'new_physical_attempts':0,'not_a_physical_success_study':True,'physical_contact_stability_for_7DoF':'UNMEASURED','no_attribution_of_cross_platform_gain_to_DoF_alone':True};(O/'COMPARISON.json').write_text(json.dumps(result,indent=2))
for name,data in [('OFFLINE_COMPARISON.csv',rows),('REDUNDANCY_ABLATION.csv',absrows)]:
 with (O/name).open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(data[0]),lineterminator='\n');w.writeheader();w.writerows(data)
fig,axs=plt.subplots(1,3,figsize=(13,4));modes=['same_grasp_same_PiPER_station_range_deg','same_grasp_optimized_station_range_deg','own_grasp_range_deg'];titles=['Same grasp / PiPER station','Same grasp / optimized station','Own grasp / optimized station']
for ax,mode,title in zip(axs,modes,titles):
 data=np.array([[next(r for r in rows if r['robot']==robot and r['asset']==asset)[mode] if next(r for r in rows if r['robot']==robot and r['asset']==asset)[mode] is not None else np.nan for asset in assets] for robot in robots]);im=ax.imshow(np.nan_to_num(data,nan=-1),vmin=0,vmax=30,cmap='viridis');ax.set_xticks(range(4),assets);ax.set_yticks(range(4),robots);ax.set_title(title)
 for y in range(4):
  for x in range(4):ax.text(x,y,'—' if np.isnan(data[y,x]) else str(round(data[y,x])),ha='center',va='center',color='white' if data[y,x]<15 or np.isnan(data[y,x]) else 'black')
fig.suptitle('OFFLINE ONLY — degrees within 0–30°; common gripper, conditional foreign mounts\nNo native contact trials on 7DoF arms; — = no collision-free plan');fig.tight_layout(rect=[0,0,1,.85]);fig.savefig(O/'OFFLINE_RANGES.png',dpi=150)
fig,ax=plt.subplots(figsize=(12,4));labels=[];full=[];fixed=[]
for robot in ['nero','xarm7','fr3']:
 for asset in assets:
  vs=[r for r in absrows if r['robot']==robot and r['asset']==asset];labels.append(robot+'\n'+asset);full.append(vs[0]['full7_range_deg']);matched=next((v for v in vs if v['fixed_source']=='matched_full7_initial_configuration'),None);fixed.append(matched['fixed6_range_deg'] if matched else np.nan)
x=np.arange(len(labels));ax.bar(x-.17,full,.34,label='full 7');ax.bar(x+.17,fixed,.34,label='joint 3 fixed at matched initial q');ax.set_xticks(x,labels);ax.set_ylabel('offline collision-free range (deg)');ax.set_ylim(0,34);ax.legend();ax.set_title('Redundancy ablation — same robot geometry, tool, base and grasp\nNo physical validation; missing matched bars mean no feasible full7 initial pose');fig.tight_layout();fig.savefig(O/'REDUNDANCY_ABLATION.png',dpi=150)
print([(r['robot'],r['asset'],r['own_grasp_range_deg']) for r in rows]);print('ablations',len(absrows))
