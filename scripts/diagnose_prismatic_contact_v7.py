"""Read-only command/servo/contact/rail audit. No GT or plant parameters.

Output contains compact derived metrics only. Raw contact points remain private.
Full downstream-tool compensation uses the frozen production operator once;
its derivative boundaries are excluded from reliable-interior summaries.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v31_estimator import process


def audit(folder):
    folder=Path(folder);a,index=columns(folder);settings=json.loads((folder/'identification_input.json').read_text())
    if settings['prior']['joint_type']!='prismatic':raise ValueError('Metre-valued rail diagnostic only')
    axis=np.array(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    report=dict(hidden_dynamics_used=False,units=dict(distance='m',velocity='m/s',effort='N'),segments=[],contact_measurements_available='friction_force_on_fingers_world_N' in a)
    for segment in index['segments']:
        ix=np.flatnonzero(a['segment']==segment['id'])
        if not len(ix):continue
        t=a['t'][ix]-a['t'][ix[0]];q=a['joint_q'][ix];tcp=a['tcp_pose'][ix,:3]@axis
        command=a['command_TCP_pose'][ix,:3]@axis if 'command_TCP_pose' in a else None
        row=dict(id=segment['id'],role=segment['role'],original_result=segment.get('result'),records=len(ix),duration_s=float(t[-1]),drawer_gain_m=float(q[-1]-q[0]),measured_TCP_gain_m=float(tcp[-1]-tcp[0]),measured_TCP_to_drawer_relative_travel_m=float((tcp[-1]-tcp[0])-(q[-1]-q[0])),raw_pad_peak_N=a['pad_load'][ix].max(0).tolist(),aperture_range_m=[float(a['aperture'][ix].min()),float(a['aperture'][ix].max())],max_relative_translation_m=float(a['relative_slip_m'][ix].max()),force_warning_samples=int(a['load_warning'][ix].sum()),safety_stop_samples=int(a['safety_stop'][ix].sum()))
        if command is not None:row.update(command_TCP_gain_m=float(command[-1]-command[0]),incremental_command_to_TCP_deficit_m=float((command[-1]-command[0])-(tcp[-1]-tcp[0])),absolute_command_to_TCP_start_end_m=[float(command[0]-tcp[0]),float(command[-1]-tcp[-1])])
        d=process(rows_at(a,ix,index['metadata'],'DIAGNOSTIC_NOT_FIT'),settings['prior'],settings['bias'],a['joint_dq'][ix],native_coordinate=q)
        if d is not None:
            edge=d['processing']['boundary_exclusion_samples'];inside=np.arange(edge,len(ix)-edge)
            row.update(compensated_wrist_force_reliable_interior_quantiles_N=np.quantile(d['torque'][inside],[0,.1,.5,.9,1]).tolist(),unfiltered_compensated_force_peak_excluding_two_edge_samples_N=float(abs(d['unfiltered_torque'][2:-2]).max()),tool_inertia_force_RMS_reliable_interior_N=float(np.sqrt(np.mean((d['inertial_world_hinge'][inside,:3]@axis)**2))),force_processing=d['processing'])
            if report['contact_measurements_available']:
                normal=-a['normal_force_on_fingers_world_N'][ix].sum(1)@axis;friction=-a['friction_force_on_fingers_world_N'][ix].sum(1)@axis;native=normal+friction
                available=np.all(a['friction_api_valid'][ix]>0,axis=1)&np.any(a['friction_anchor_counts'][ix]>0,axis=1)
                j=inside[available[inside]]
                row.update(native_anchor_valid_samples=int(available.sum()),native_normal_rail_mean_N=float(normal[j].mean()) if len(j) else None,native_friction_rail_mean_N=float(friction[j].mean()) if len(j) else None,native_total_vs_compensated_wrist_RMS_N=float(np.sqrt(np.mean((native[j]-d['unfiltered_torque'][j])**2))) if len(j) else None,friction_sign='native finger-body force negated for object reaction; compared against wrist measurement, not GT torque')
        report['segments'].append(row)
    p=folder/'contact_evidence/index.json'
    if p.exists():
        meta=json.loads(p.read_text());parts={}
        for chunk in meta['files']:
            with np.load(p.parent/chunk['file']) as x:
                for k in ['attempt','finger','kind','longitudinal_margin_m','lateral_margin_m','normal_pad','relative_tangential_speed_m_s']:parts.setdefault(k,[]).append(x[k])
        if parts:
            c={k:np.concatenate(v) for k,v in parts.items()};summaries=[]
            for attempt in np.unique(c['attempt']):
                for finger in [0,1]:
                    j=(c['attempt']==attempt)&(c['finger']==finger)&(c['kind']==0)
                    if j.any():summaries.append(dict(attempt=int(attempt),finger=int(finger),normal_point_records=int(j.sum()),longitudinal_margin_quantiles_m=np.quantile(c['longitudinal_margin_m'][j],[0,.5,1]).tolist(),lateral_margin_quantiles_m=np.quantile(c['lateral_margin_m'][j],[0,.5,1]).tolist(),median_native_normal_pad=np.median(c['normal_pad'][j],axis=0).tolist(),derived_material_tangent_speed_quantiles_m_s=np.nanquantile(c['relative_tangential_speed_m_s'][j],[.5,.9,1]).tolist()))
            report['contact_point_summary']=summaries
    report['limitations']=['A positive/absent normal report alone does not prove holding/separation','Material-point relative velocity is derived kinematics, not directly measured friction','Contact points are not pressure areas; individual contact force sign preserves native collider ordering','FT compensation retains finite-difference/temporal error; unfiltered peaks are not hidden','No geometric replacement grasp or controller change is computed by this audit']
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args();r=audit(args.folder);args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(r,indent=2));print(json.dumps(dict(segments=len(r['segments']),output=str(args.output),hidden_dynamics_used=False)))
