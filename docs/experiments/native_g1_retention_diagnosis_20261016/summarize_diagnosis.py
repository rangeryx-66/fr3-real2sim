"""Compact publication products from archived-only numerical diagnostics."""
import argparse,hashlib,json,gzip,pickle
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def read(p):return json.loads(p.read_text())
def write(p,d):p.write_text(json.dumps(d,indent=2))
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def at(ep,t):
    d=ep/'physics_steps_chunks'
    for c in read(d/'index.json')['chunks']:
        rr=pickle.load(gzip.open(d/c['file'],'rb'))
        if rr[-1]['t']+1/240<t:continue
        return min(rr,key=lambda x:abs(x['t']-t))
    return rr[-1]
def delta(A,B):
    A=np.asarray(A);B=np.asarray(B)
    return dict(translation_mm=((B[:3,3]-A[:3,3])*1000).tolist(),translation_norm_mm=float(np.linalg.norm(B[:3,3]-A[:3,3])*1000),
                rotation_vector_deg=np.rad2deg(Rotation.from_matrix(B[:3,:3]@A[:3,:3].T).as_rotvec()).tolist(),rotation_deg=float(np.rad2deg(Rotation.from_matrix(B[:3,:3]@A[:3,:3].T).magnitude())))

def main(a):
    full=read(a.private/'remote/FIXED_POSE_SCORES.json');rows=[]
    for r in full['rows']:
        score=r['contact']['robust_contact'];chosen=next(t for t in score['aperture_trials'] if t['width_m']==score['chosen_expected_aperture_m'])
        compact={k:score[k] for k in ['acquisition_score','retention_score','task_score','robust_mean','robust_lower_quartile','robust_combined_score','chosen_expected_aperture_m','nominal_best_expected_aperture_m','uncertainty_changes_aperture_choice']}
        rows.append(dict(geometry=r['geometry'],pose=r['pose'],scores=compact,first_contacts=r['contact']['first_contacts'],
                         nominal_sides=chosen['nominal']['sides'],nominal_rear_hypotheses=chosen['nominal']['hidden_geometry_samples'],
                         perturbations=[dict(label=s['perturbation'],combined=s['combined'],contact=s['contact_score'],retention=s['retention_score'],task=s['task_score']) for s in chosen['samples']],
                         first_contact_predictions=r['predictions']))
    compact=dict(no_new_poses=True,no_GT_in_scoring=True,unchanged_configuration=full['config'],rows=rows,
                 separate_GT_native_diagnostic=full['GT_separate_evaluation_only'],source_hashes=full['source_hashes'])
    write(a.public/'FIXED_POSE_COMPARISON.json',compact)
    fig,axs=plt.subplots(1,2,figsize=(14,5),sharey=True)
    for ax,geometry in zip(axs,['first_seed','incremental']):
        for pose,color in [('success','#2769a8'),('failed','#d15a24')]:
            row=next(r for r in rows if r['geometry']==geometry and r['pose']==pose)
            ax.plot(range(15),[r['combined'] for r in row['perturbations']],'.-',lw=.8,label=pose+' frozen pose',color=color)
        ax.set_xticks(range(15),[r['label'].replace('translation','trans').replace('rotation','rot') for r in row['perturbations']],rotation=75,fontsize=7)
        ax.set_title(geometry+' geometry');ax.set_ylabel('Frozen geometric combined sample score');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Two frozen poses, two archived representations; unchanged 15 perturbations\nSoft geometric ranking features, not probabilities or contact-retention guarantees')
    fig.tight_layout();fig.savefig(a.public/'figures/FIXED_POSE_PERTURBATIONS.png',dpi=160);plt.close(fig)
    poses={};timelines={};provenance={};velocity_consistency={};cooked_hashes={}
    roots={'success':a.success,'failed':a.failed}
    for label,root in roots.items():
        ep=root/'episode';report=read(ep/'report.json');zero=report['grasp_acquisition']['t'];corr=read(ep/'visual_correction.json');plan=read(ep/'visual_plan.json');sample=at(ep,zero)
        poses[label]=dict(selected=plan['T'],refined=corr['T_after'],acquired=sample['T_tcp'],acquired_body=(np.linalg.inv(sample['T_moving_link'])@sample['T_tcp']).tolist())
        poses[label]['refinement_shift_at_TCP']=delta(plan['T'],corr['T_after']);poses[label]['refined_to_acquired']=delta(corr['T_after'],sample['T_tcp'])
        analysis_label='success_a47fe09' if label=='success' else 'failed_f980076'
        d=np.load(a.private/'analysis'/f'{analysis_label}_NATIVE_RATE.npz');stats=read(a.private/'analysis'/f'{analysis_label}_SUMMARY_FULL.json');t=d['t'];hold=(t>=0)&(t<stats['opening_start_after_verification_s']);ix=np.flatnonzero(hold);minimum=ix[np.argmin(d['aperture_mm'][hold])]
        timelines[label]=dict(aperture_minimum_aligned_s=float(t[minimum]),aperture_minimum_mm=float(d['aperture_mm'][minimum]),
            first_raw_left_below_existing_0p05_n_s=next((float(t[i]) for i in ix if d['raw'][i,0]<.05),None),
            first_filtered_left_below_existing_0p05_n_s=next((float(t[i]) for i in ix if d['filtered'][i,0]<.05),None),
            raw_hold_end_n=d['raw'][ix[-1]].tolist(),filtered_hold_end_n=d['filtered'][ix[-1]].tolist(),
            acquisition_master_error_rad=float(d['qcmd'][np.argmin(abs(t)),7]-d['q'][np.argmin(abs(t)),7]),
            hold_end_master_error_rad=float(d['qcmd'][ix[-1],7]-d['q'][ix[-1],7]),
            hold_max_TCP_speed_mm_s=float(np.nanmax(d['tcp_speed_mm_s'][hold])))
        fd=np.diff(d['q'],axis=0)/np.diff(t)[:,None];hv=(t[1:]>=0)&(t[1:]<stats['opening_start_after_verification_s']);native=d['velocity'][1:]
        velocity_consistency[label]=dict(joint_names=read(ep/'sensor_metadata.json')['joint_names'][7:],
            hold_native_minus_position_difference_RMSE_rad_s=np.sqrt(np.mean((native[hv,7:]-fd[hv,7:])**2,axis=0)).tolist(),
            hold_mean_native_rad_s=native[hv,7:].mean(0).tolist(),hold_mean_position_difference_rad_s=fd[hv,7:].mean(0).tolist(),
            interpretation='Instantaneous native velocities and post-step position differences disagree strongly on mimic joints; neither channel is overwritten or filtered. Constraint/projection/timing contribution is unresolved.')
        cooked_hashes[label]={s['path'].split('/')[-1]:hashlib.sha256(json.dumps(s['convexes'],sort_keys=True).encode()).hexdigest() for s in read(ep/'cooked_initial.json')['shapes'] if s.get('rigid_body_path','').split('/')[-1] in ['left_finger','right_finger'] or 'original39' in s['path']}
        provenance[label]={str(f.relative_to(ep)):dict(sha256=digest(f),bytes=f.stat().st_size) for f in [ep/'report.json',ep/'sensor_metadata.json',ep/'ACTUATOR_METADATA.json',ep/'visual_plan.json',ep/'visual_correction.json',ep/'cooked_initial.json',ep/'contact_baseline.mp4',ep/'observations_chunks/index.json',ep/'physics_steps_chunks/index.json']}
    across={k:delta(poses['success'][k],poses['failed'][k]) for k in ['selected','refined','acquired','acquired_body']}
    write(a.public/'POSE_AND_TIMING.json',dict(across_runs=across,per_run_pose_differences={k:{z:p[z] for z in ['refinement_shift_at_TCP','refined_to_acquired']} for k,p in poses.items()},timelines=timelines))
    write(a.public/'VELOCITY_CONSISTENCY.json',velocity_consistency)
    write(a.public/'EVIDENCE_INDEX.json',dict(private_roots={k:str(v) for k,v in roots.items()},inputs=provenance,raw_published=False,
          video_inspection=dict(segments_relative_to_verification_s=[[-.2,1.5]],frames_aligned_s=[0,.23,.5,1.005],
          finding='Fingers are partly occluded; video does not resolve 10-500 micrometre gaps or prove separation. No gross impact observed.',private_directory=str(a.private/'video_inspection')),cooked_payload_hashes=cooked_hashes,cooked_finger_and_handle_payloads_identical=cooked_hashes['success']==cooked_hashes['failed']))
    # Explicit contact-status and velocity/tracking channels at the native rate.
    fig,axs=plt.subplots(4,2,figsize=(13,10),sharex=True)
    for col,(label,root) in enumerate(roots.items()):
        name='success_a47fe09' if label=='success' else 'failed_f980076';d=np.load(a.private/'analysis'/f'{name}_NATIVE_RATE.npz');t=d['t'];tp=d['tp'];keep=(t>=-.3)&(t<=1.4);kp=(tp>=-.3)&(tp<=1.4)
        for side,color in [(0,'#2769a8'),(1,'#d15a24')]:
            axs[0,col].step(tp[kp],(d['normal_load'][kp,side]>0).astype(float)-side*1.3,where='post',color=color,label=['left positive','right positive (offset)'][side])
            axs[1,col].plot(t[keep],d['relative_mm'][keep,side],color=color,label=['body x','body y'][side])
        axs[1,col].plot(t[keep],d['relative_mm'][keep,2],label='body z',color='grey')
        axs[2,col].plot(t[keep],d['tcp_speed_mm_s'][keep],lw=.8,label='measured finite difference')
        axs[3,col].plot(t[keep],(d['aperture_mm'][keep]-d['command_mm'][keep]),lw=.8,label='master-derived aperture error mm')
        for i,title in enumerate(['Native positive-normal report','TCP/body displacement mm','World TCP speed mm/s','Position-hold aperture error mm']):
            ax=axs[i,col];ax.set_ylabel(title);ax.axvline(0,color='grey',ls=':');ax.axvline(1.0041666667,color='grey',ls='--');ax.grid(alpha=.2);ax.legend(fontsize=7);ax.set_xlim(-.3,1.4)
        axs[0,col].set_title(label)
    fig.suptitle('Contact report, measured motion and holding error: no new filtering\nPositive normal report != complete contact state; dashed = original opening start')
    fig.tight_layout();fig.savefig(a.public/'figures/CONTACT_STATUS_AND_TRACKING.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(13,7),sharex=True)
    for col,label in enumerate(roots):
        name='success_a47fe09' if label=='success' else 'failed_f980076';d=np.load(a.private/'analysis'/f'{name}_NATIVE_RATE.npz');t=d['t'];fd=np.diff(d['q'],axis=0)/np.diff(t)[:,None];keep=(t[1:]>=0)&(t[1:]<1.33)
        for row,(index,n) in enumerate([(7,'drive_joint'),(11,'left_finger_joint')]):
            ax=axs[row,col];ax.plot(t[1:][keep],d['velocity'][1:,index][keep],lw=.7,label='native velocity');ax.plot(t[1:][keep],fd[keep,index],lw=.7,label='one-step position difference');ax.set_ylabel(n+' rad/s');ax.set_title(label);ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Both archived velocity interpretations retained; no smoothing\nDisagreement limits inference about mimic constraint dynamics')
    fig.tight_layout();fig.savefig(a.public/'figures/NATIVE_VS_POSITION_VELOCITY.png',dpi=160);plt.close(fig)
    write(a.public/'COOKED_CLEARANCE.json',read(a.private/'remote/COOKED_CLEARANCE.json'))
    print('COMPACT_DIAGNOSIS_PRODUCTS_COMPLETE')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--success',type=Path,required=True);p.add_argument('--failed',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--public',type=Path,required=True);main(p.parse_args())
