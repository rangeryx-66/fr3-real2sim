"""Archived six-axis load chain and native cross-check; no truth or control."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v31_estimator import process
from interaction_runtime.serialization import result_packet


def stats(w,axis):
    w=np.asarray(w);rail=w[:,:3]@axis;trans=w[:,:3]-rail[:,None]*axis
    return dict(mean_world_six=w.mean(0).tolist(),RMS_world_six=np.sqrt(np.mean(w*w,axis=0)).tolist(),
        mean_force_norm_N=float(np.linalg.norm(w[:,:3],axis=1).mean()),
        mean_rail_force_N=float(rail.mean()),mean_transverse_norm_N=float(np.linalg.norm(trans,axis=1).mean()),
        mean_sensor_origin_torque_norm_Nm=float(np.linalg.norm(w[:,3:],axis=1).mean()),
        peak_sensor_origin_torque_norm_Nm=float(np.linalg.norm(w[:,3:],axis=1).max()))


def diagnose(folder):
    folder=Path(folder);a,index=columns(folder);settings=json.loads((folder/'identification_input.json').read_text())
    axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    cfg=json.loads((folder/'EFFECTIVE_RUNTIME.json').read_text())['policy'];phases=np.array(index['metadata']['phases'])[a['phase']]
    result=dict(hidden_truth_read=False,production_or_safety_changed=False,
        policy_sha256=hashlib.sha256((folder/'EFFECTIVE_RUNTIME.json').read_bytes()).hexdigest(),segments=[])
    for segment in index['segments']:
        ix=np.flatnonzero(a['segment']==segment['id'])
        if len(ix)<247:continue
        # Split gaps, never differentiate across missing/non-native samples.
        runs=np.split(ix,np.flatnonzero((np.diff(a['step'][ix])!=1)|(abs(np.diff(a['t'][ix])-1/240)>1e-7))+1)
        for run in runs:
            if len(run)<247:continue
            rows=rows_at(a,run,index['metadata'],'ARCHIVED_LOAD_ONLY');p=process(rows,settings['prior'],settings['bias'],a['joint_dq'][run],native_coordinate=a['joint_q'][run])
            raw=p['raw_world_hinge'].copy();gravity=raw+p['gravity_world_hinge'];full=gravity-p['inertial_world_hinge']
            offset=a['sensor_pose'][run,:3]-settings['prior']['origin_world_m']
            for w in (raw,gravity,full):w[:,3:]-=np.cross(offset,w[:,:3])
            # Last fully observed causal guard window. Dynamic compensation is
            # diagnostic only and cannot use the unavailable future at STOP.
            duration=cfg['force']['sustained_s'];end=p['t'][-1];window=p['t']>=end-duration+1/480
            reliable=(np.arange(len(run))>=121)&(np.arange(len(run))<len(run)-121)
            reliable_window=(p['t']>=p['t'][-122]-duration+1/480)&reliable
            native=-(a['normal_force_on_fingers_world_N'][run]+a['friction_force_on_fingers_world_N'][run]).sum(1)
            raw_unbiased=p['raw']
            normal=-a['normal_force_on_fingers_world_N'][run].sum(1)@axis
            friction=-a['friction_force_on_fingers_world_N'][run].sum(1)@axis
            tcp=a['tcp_pose'][run,:3]@axis;command=a['command_TCP_pose'][run,:3]@axis
            r=dict(id=segment['id'],role=segment['role'],original_result=segment.get('result'),
                native_interval_s=[float(p['t'][0]),float(end)],
                last_guard_window_s=[float(p['t'][window][0]),float(end)],last_guard_native_samples=int(window.sum()),
                raw_bias_removed_sensor_to_world_at_sensor=stats(raw[window],axis),
                gravity_compensated_at_sensor=stats(gravity[window],axis),
                last_guard_pad_max_mean_N=float(a['pad_load'][run][window].max(1).mean()),
                full_dynamic_last_guard_window='UNAVAILABLE: centered derivative needs future samples; do not invent boundary torque',
                last_reliable_dynamic_window_s=[float(p['t'][reliable_window][0]),float(p['t'][reliable_window][-1])],
                full_dynamic_reliable_window=stats(full[reliable_window],axis),
                gravity_reliable_window=stats(gravity[reliable_window],axis),
                native_diagnostic_rail_normal_mean_N=float(normal[window].mean()),
                native_diagnostic_rail_friction_mean_N=float(friction[window].mean()),
                friction_api_valid_fraction=a['friction_api_valid'][run][window].mean(0).tolist(),
                guard_mean_components_N_N_Nm=[float(a['pad_load'][run][window].max(1).mean()),float(np.linalg.norm(gravity[window,:3],axis=1).mean()),float(np.linalg.norm(gravity[window,3:],axis=1).mean())],
                actual_drawer_delta_m=float(p['q'][-1]-p['q'][0]),
                command_minus_measured_TCP_delta_m=float(command[-1]-command[0]-tcp[-1]+tcp[0]),
                measured_TCP_minus_drawer_delta_m=float(tcp[-1]-tcp[0]-p['q'][-1]+p['q'][0]),
                aperture_delta_m=float(a['aperture'][run[-1]]-a['aperture'][run[0]]),
                diagnostic_full_wrench_native_force_residual_RMS_N=np.sqrt(np.mean((full[reliable,:3]-native[reliable])**2,axis=0)).tolist())
            result['segments'].append(r)
    result['source_numeric_sha256']=[dict(file=c['file'],sha256=c['sha256'])for c in index['chunks']]
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episodes',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    value={p.name:diagnose(p)for p in a.episodes};a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result_packet(value),indent=2)+'\n');print(json.dumps({k:len(v['segments'])for k,v in value.items()}))
