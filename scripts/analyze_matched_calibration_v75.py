"""Read-only matched free-space sensor analysis. Never estimator training."""
import argparse,json,hashlib,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v31_estimator import process


def analyze(folder,source,output):
    p=Path(folder);a,index=columns(p)
    settings=json.loads((Path(source)/'identification_input.json').read_text())
    axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    report=dict(kind='unloaded sensor diagnostic; not FIT or VALIDATION',
        blocks=[],production_parameters_modified=False,hidden_truth_read=False,
        source_index_sha256=hashlib.sha256((p/'telemetry/index.json').read_bytes()).hexdigest(),
        limitation='Translated measured references and timing; actual free-space motion and contact compliance differ. No hardware noise bound.')
    for s in index.get('segments',[]):
        if s['role']!='CALIBRATION_NOT_FIT':raise ValueError('Unexpected training role')
        ids=np.flatnonzero(a['segment']==s['id'])
        for run in np.split(ids,np.flatnonzero(np.diff(a['step'][ids])!=1)+1):
            if len(run)<247:continue
            rows=rows_at(a,run,index['metadata'],'UNLOADED_MATCHED_'+str(s['id']))
            d=process(rows,settings['prior'],settings['bias'],a['joint_dq'][run],native_coordinate=a['joint_q'][run])
            if d is None:continue
            inside=np.arange(121,len(run)-121)
            sensor=a['sensor_pose'][run,:3];origin=np.asarray(settings['prior']['origin_world_m'])
            raw=d['raw_world_hinge'].copy();gravity=raw+d['gravity_world_hinge'];full=gravity-d['inertial_world_hinge']
            row=dict(segment_id=s['id'],samples=len(run),reliable_samples=len(inside),
                duration_s=float(d['t'][-1]-d['t'][0]),
                object_range_m=float(np.ptp(a['joint_q'][run])),
                pad_peak_N=a['pad_load'][run].max(0).tolist(),
                reference_tracking_RMS_m=float(np.sqrt(np.mean(np.sum((a['tcp_pose'][run,:3]-a['command_TCP_pose'][run,:3])**2,axis=1)))))
            for name,w in [('raw_bias_removed',raw),('gravity_compensated',gravity),('fully_compensated',full)]:
                w=w.copy();w[:,3:]-=np.cross(sensor-origin,w[:,:3])
                reliable=w[inside];rail=reliable[:,:3]@axis
                bins=np.floor((d['t'][inside]-d['t'][inside][0])/.1).astype(int)
                b=np.array([rail[bins==i].mean()for i in np.unique(bins)if np.sum(bins==i)>=12])
                row[name]=dict(world_sensor_origin_mean=reliable.mean(0).tolist(),
                    world_sensor_origin_RMS=np.sqrt(np.mean(reliable**2,axis=0)).tolist(),
                    rail_RMS_N=float(np.sqrt(np.mean(rail**2))),rail_mean_N=float(rail.mean()),
                    rail_bin100ms_RMS_N=float(np.sqrt(np.mean(b**2)))if len(b)else None)
            row['common_SG121_fully_compensated_rail_RMS_N']=float(np.sqrt(np.mean(d['torque'][inside]**2)))
            report['blocks'].append(row)
    result=p/'CALIBRATION_RESULT.json'
    if result.exists():report['physical_result']=json.loads(result.read_text())
    Path(output).parent.mkdir(parents=True,exist_ok=True)
    Path(output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(blocks=len(report['blocks']),result=report.get('physical_result',{}).get('status'))))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path)
    parser.add_argument('--source',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();analyze(args.folder,args.source,args.output)
