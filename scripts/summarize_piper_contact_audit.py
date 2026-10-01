"""Compact contact audit evidence, plots and subset exports for reproducibility."""
import argparse,json,hashlib,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
def read(path):return json.loads((a.results/path).read_text())
real='piper_measured_cooked_closure_final' if (a.results/'piper_measured_cooked_closure_final'/'report.json').exists() else 'piper_measured_cooked_closure';current=real if (a.results/real/'cooked_shapes.json').exists() else 'piper_cooked_audit'
v=read(Path(current)/('contact_validation.json' if current==real else 'validation.json'));summary={'base_unchanged':[.5,-.55,-.1,150.],'friction_unchanged':[.5,.5],'finger_effort_limit_n':10.,'joint_margin_requirement_rad':.05,'canonical_geometry_m':[.20,.024,.018],'real_closure_report':read(Path(real)/'report.json') if (a.results/real/'report.json').exists() else None,'formal_closure_replay':read(Path(real)/'closure_replay.json')['summary'] if (a.results/real/'closure_replay.json').exists() else read(Path('piper_cooked_audit')/'full_replay.json')['summary'],'canonical_complete_replay':read(Path('piper_canonical_box')/'full_replay.json')['summary'],'canonical_strict_report':read(Path('piper_canonical_box_strict')/'report.json'),'cooking_fidelity_ablation':{'hull_vertex_limit':255,'result':'unchanged exported vertices/polygons, not adopted in production'},'files':{},'legal_pad_only_canonical_grasp':False,'legitimate_formal_grasp':False,'mobile_base_used':False,'door_opening_executed':False}
summary['geometry']=[x for x in v['geometry'] if 'gripper_link' in x['path'] or 'original19' in x['path']]
summary['closure_cross_ablation']={mode:{'pairs':sum(x['mode']==mode for x in v['intersection_audit']),'nonpad_witnesses':sum(len(x['nonpad_witnesses']) for x in v['intersection_audit'] if x['mode']==mode)} for mode in ['physx_cooked','raw_finger_raw_target','raw_finger_cooked_target','cooked_finger_raw_target','old_full_hull_raw_target']}
for directory in ['piper_cooked_audit','piper_cooked_audit_255','piper_canonical_box','piper_canonical_box_strict',real]:
 d=a.results/directory
 for file in d.glob('cooked*.json'):
  data=json.loads(file.read_text());subset=[s for s in data['shapes'] if s.get('rigid_body_path','').split('/')[-1] in ['l_1','gripper_link1','gripper_link2'] or s['path']=='/World/canonical_handle'];data['shapes']=subset;data['source_export_sha256']=hashlib.sha256(file.read_bytes()).hexdigest();data['subset_note']='Only contact-relevant colliders; vertices/transforms unchanged from native export'
  dest=a.output/directory/file.name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(json.dumps(data));summary['files'][str(dest.relative_to(a.output))]={'source_sha256':data['source_export_sha256']}
# Plot observations, retaining tiny non-pad edge crossings, never smoothing them.
rows=read(Path('piper_canonical_box')/'observations.json');t=np.array([x['t'] for x in rows]);ap=np.array([x['aperture_m'] for x in rows])*1000;force=np.array([x['finger_forces_n'] for x in rows]);replay=read(Path('piper_canonical_box')/'full_replay.json')['states']
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True);axes[0].plot(t,ap);axes[0].axhline(24,color='gray',ls='--',label='analytic width 24 mm');axes[0].set_ylabel('Aperture (mm)');axes[0].legend();axes[1].plot(t,force[:,0],label='finger 1');axes[1].plot(t,force[:,1],label='finger 2');axes[1].set_ylabel('Force (N)');axes[1].legend()
for mode,color in [('physx_cooked','tab:blue'),('raw_finger_raw_target','tab:red')]:
 ys=[max((f['max_distance_from_pad_m']*1e6 for f in x['failures'] if f['mode']==mode),default=0.) for x in replay];axes[2].plot([x['t'] for x in replay],ys,label=mode,color=color)
axes[2].axhline(1,color='gray',ls='--',label='unchanged strict 1 um threshold');axes[2].set_ylabel('Non-pad edge distance (um)');axes[2].set_xlabel('Simulation time (s)');axes[2].legend();fig.suptitle('Canonical box: diagnostic pull, NOT a certified pad-only grasp');fig.tight_layout();fig.savefig(a.output/'canonical_contact_trajectory.png',dpi=160);plt.close(fig)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k not in ['real_closure_report','canonical_strict_report','geometry','files']},indent=2))
