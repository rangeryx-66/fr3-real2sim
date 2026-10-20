"""Read-only engineering tracking audit; never starts Isaac or reads object truth.

Implicit spring estimates are diagnostics, NOT measured motor output. Each
interval excludes its final boundary sample, which can belong to the next
reference handover in the compact recorder.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from diagnose_probing_v3 import columns
from cross_robot_adapter import RobotPinkIK
from interaction_reference_handover import seed_pose


def audit(folder):
    p=Path(folder);a,index=columns(p)
    runtime=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text())
    job=json.loads((p/'job.json').read_text())
    base=json.loads((Path(job['source'])/'report.json').read_text())['robot_base_pose']
    solver=RobotPinkIK(runtime['gripper_config']);arms=runtime['arm_indices']
    result=json.loads((p/'MICROTRACKING_RESULT.json').read_text())
    out=dict(hidden_object_truth_used=False,audit_starts_new_physics=False,
             fixture='unloaded robot-only; archived observed unexecuted pregrasp reference',
             independent_contact_validation=False,segments=[])
    for s in result['segments']:
        ix=np.flatnonzero((a['t']>=s['start_sim_s'])&(a['t']<s['end_sim_s']-1e-8))
        d=np.asarray(s['reference_world_direction'])
        native=a['tcp_pose'][ix,:3]
        fk=np.array([seed_pose(solver,base,q[arms])[:3,3] for q in a['q'][ix]])
        cmd=a['command_TCP_pose'][ix,:3]
        row=dict(axis=s['axis'],leg=s['leg'],samples=len(ix),
            requested_um=s['requested_displacement_m']*1e6,
            actual_projected_um=float((native[-1]-native[0])@d*1e6),
            nominal_command_projected_um=float((cmd[-1]-cmd[0])@d*1e6),
            max_native_minus_FK_um=float(np.linalg.norm(native-fk,axis=1).max()*1e6),
            max_pad_load_N=float(a['pad_load'][ix].max()),
            max_object_coordinate_m=float(abs(a['joint_q'][ix]).max()),
            safety_samples=int(a['safety_stop'][ix].sum()))
        if 'physical_reference_TCP_pose' in a:
            desired=a['physical_reference_TCP_pose'][ix,:3]
            row.update(final_desired_minus_measured_um=((desired[-1]-native[-1])*1e6).tolist(),
                       integral_limited_samples=int(a['tracking_integral_limited'][ix].sum()))
        out['segments'].append(row)
    out.update(safe_park=result.get('safe_exit',{}).get('safe',False),
        process=json.loads((p/'PROCESS_COMPLETION.json').read_text()),
        raw_telemetry_sha256=[c['sha256'] for c in index['chunks']],
        limitation='No-contact test cannot certify traction, contact retention, motor output or nonlinear contact stability.')
    return out


def compare_plot(folders,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(len(folders),2,figsize=(11,3.5*len(folders)),squeeze=False)
    for ri,folder in enumerate(folders):
        p=Path(folder);a,_=columns(p)
        result=json.loads((p/'MICROTRACKING_RESULT.json').read_text())
        for ci,name in enumerate(['closing_axis','bar_tangent']):
            ss=[s for s in result['segments'] if s['axis']==name]
            ix=np.flatnonzero((a['t']>=ss[0]['start_sim_s'])&(a['t']<ss[-1]['end_sim_s']-1e-8))
            d=np.asarray(ss[0]['reference_world_direction']);t=a['t'][ix]-a['t'][ix[0]]
            actual=a['tcp_pose'][ix,:3]@d;cmd=a['command_TCP_pose'][ix,:3]@d
            ax=axs[ri,ci];ax.plot(t,(actual-actual[0])*1e6,label='Measured TCP',lw=1.5)
            ax.plot(t,(cmd-cmd[0])*1e6,label='Nominal command FK',lw=1,alpha=.75)
            if 'physical_reference_TCP_pose' in a:
                target=a['physical_reference_TCP_pose'][ix,:3]@d
                ax.plot(t,(target-actual[0])*1e6,label='Physical reference',lw=1)
            ax.axhline(100,color='gray',ls='--',lw=.8,label='Requested outward +100 um')
            ax.axvline(ss[0]['end_sim_s']-ss[0]['start_sim_s'],color='gray',ls=':',lw=.8)
            dwell=ss[0].get('diagnostic_dwell_s',json.loads((p/'job.json').read_text()).get('microtracking_dwell_s',.3))
            ax.set(title=f'{p.name}\n{name}; dwell {dwell} s',xlabel='Simulation time (s)',ylabel='Projected displacement (um)')
            ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Unloaded microtracking: unchanged robot drives/friction; no contact or object motion\nDifferent dwell durations: not a time-matched controller comparison',fontsize=10)
    fig.tight_layout(rect=(0,0,1,.93));fig.savefig(output,dpi=150);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder');parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--compare',type=Path);parser.add_argument('--plot',type=Path)
    args=parser.parse_args();args.output.write_text(json.dumps(audit(args.folder),indent=2))
    if args.plot:
        compare_plot([Path(args.folder)]+([args.compare] if args.compare else []),args.plot)
