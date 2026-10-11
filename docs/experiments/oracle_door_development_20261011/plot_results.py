import json,pickle,gzip,sys,numpy as np
from pathlib import Path
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);fig,axes=plt.subplots(5,2,figsize=(12,11),sharex='col')
for col,i in enumerate(['45176','45415']):
 p=root/i/'development_01'
 if not (p/'report.json').exists():continue
 rr=[]
 for f in sorted((p/'observations_chunks').glob('*.gz')):
  with gzip.open(f,'rb') as stream:rr.extend(pickle.load(stream))
 t=np.array([r['t'] for r in rr]);angle=np.array([r['door_angle_deg'] for r in rr]);E=np.array([r['T_tcp'] for r in rr]);v=np.r_[0,np.linalg.norm(np.diff(E[:,:3,3],axis=0),axis=1)/np.diff(t)*1000]
 data=[(angle,'Actual opening (degrees)'),(v,'Measured TCP speed (mm/s)'),(np.array([r['relative_translation_slip_m'] for r in rr])*1000,'Handle-relative drift (mm)'),(np.array([r['margin_rad'] for r in rr]),'Joint margin (rad)')]
 for row,(y,label) in enumerate(data):
  ax=axes[row,col];ax.plot(t,y,lw=.65);ax.set_ylabel(label);ax.grid(alpha=.2)
 axes[0,col].axhline(30,color='green',ls='--',lw=.8);axes[1,col].axhline(5,color='red',ls='--',lw=.8);axes[3,col].axhline(.05,color='red',ls='--',lw=.8)
 loads=np.array([list(r['forces_n'].values()) for r in rr]);axes[4,col].plot(t,loads[:,0],lw=.45,label='left raw');axes[4,col].plot(t,loads[:,1],lw=.45,label='right raw');axes[4,col].legend();axes[4,col].set_ylabel('Pad load (N)');axes[4,col].set_xlabel('Simulation time (s)');axes[4,col].grid(alpha=.2)
 report=json.loads((p/'report.json').read_text());axes[0,col].set_title(i+' — '+('PASS' if report['success'] else 'FAIL'))
 events=json.loads((p/'events.json').read_text());g=next((e for e in events if e['event']=='GRASP_VERIFIED'),None)
 if g:
  for ax in axes[:,col]:ax.axvline(g['t'],color='gray',ls=':',lw=.8)
 # Offline evidence of all loaded pad contacts belonging to the intended mesh.
 contacts=[c for r in rr for c in r['contacts'] if c.get('allowed_pad_target') and c.get('force_n',0)>0]
 (p/'CONTACT_EVIDENCE.json').write_text(json.dumps({'classification':'OFFLINE_DIAGNOSTIC_ONLY','contact_targets':sorted({c['target'] for c in contacts}),'positive_handle_contact_records':len(contacts),'grasp_event':g,'first_loaded_handle_contacts':contacts[:8],'sample_timing_note':'Native pre-step and post-step observation records retained separately.','execution_gates_added':False},indent=2))
fig.suptitle('ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — unchanged contact controller and protections');fig.tight_layout();fig.savefig(root/'physical_results.png',dpi=160)
