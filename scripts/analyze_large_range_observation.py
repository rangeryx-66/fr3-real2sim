"""Post-run unloaded wrist data only; no object-parameter fit or Isaac launch."""
import argparse,json,pickle,gzip
from pathlib import Path
import numpy as np
from summarize_hybrid_episode import rows_at,main as summarize
from xarm_ft_id_estimator import process
from xarm_ft_known_joint_prior import native_joint_prior


def analyze(root):
    root=Path(root);ep=root/'episode';summarize(root,root/'PRIVATE_EPISODE_SUMMARY.json')
    rows=list(rows_at(ep));job=json.loads((root/'job.json').read_text());prior=native_joint_prior(job['asset_root'],rows[0]['T_moving_link'])
    routes=json.loads((ep/'WRIST_ROBOT_TRAJECTORIES.json').read_text());pieces=[];stats=[]
    for route in routes:
        block=[r for r in rows if route['t_start']<=r['t']<=route['t_end']]
        data=process(block,prior,job['ft_calibration']['bias_sensor'])
        if data is None:continue
        # Calibration interval only; no train/validation derivative leakage.
        good=np.ones(len(data['t']),bool);good[:121]=False;good[-121:]=False
        stats.append(dict(label=route['label'],records=len(block),local_calibration_samples=int(good.sum()),
            compensated_world_RMS=np.sqrt(np.mean(data['compensated_world'][good]**2,axis=0)).tolist(),
            compensated_world_peak_abs=np.max(abs(data['compensated_world'][good]),axis=0).tolist(),
            calibration_only_not_object_resistance=True))
        pieces.append({k:data[k] for k in ['t','raw','compensated_world','gravity_world','inertial_world']})
    out=root/'private_analysis';out.mkdir(exist_ok=True)
    if pieces:
        data={k:np.concatenate([p[k] for p in pieces]) for k in pieces[0]};np.savez_compressed(out/'UNLOADED_WRIST_WRENCHES.npz',**data)
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
        for j,ax in enumerate(axes.flat):
            ax.plot(data['t'],data['raw'][:,j],lw=.5,label='raw SENSOR frame')
            ax.plot(data['t'],data['compensated_world'][:,j],lw=.5,label='compensated WORLD frame')
            ax.set_ylabel(['Fx N','Fy N','Fz N','Tx Nm','Ty Nm','Tz Nm'][j]);ax.grid(alpha=.2);ax.legend(fontsize=7)
        fig.suptitle('Unloaded wrist observation ONLY; different raw/processed frames labeled\nNo object interaction, no hinge identification or held-out dynamics claim')
        fig.tight_layout();fig.savefig(out/'UNLOADED_WRIST_WRENCHES.png',dpi=140);plt.close(fig)
    result=dict(motion_blocks=stats,interaction_segments=0,physical_model_updates=0,identified_inertia=None,identified_friction=None,identified_gravity=None,
        heldout_motion_prediction=None,parameter_accuracy_not_evaluated=True,unloaded_compensation_is_not_object_identification=True)
    (out/'UNLOADED_ANALYSIS.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(motion_blocks=len(stats),interaction_segments=0)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);analyze(p.parse_args().root)
