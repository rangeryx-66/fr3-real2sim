"""Native measurement-only contact bounds; no requested-force substitution.

Used traction gives a lower bound, not an identified friction point value.
Transferring a normal-load calibration assumes the same G1/handle geometry;
held-out actual loads/slip must test that assumption. No failure is scripted.
"""
import hashlib,json
from pathlib import Path
import numpy as np


def point_normal_sums(steps,point_steps,sides,impulses,normals,axis,dt):
    """Sum scalar normal impulses, avoiding cancellation of contact normals."""
    normal=np.zeros((len(steps),2));tangent_budget=normal.copy();seen=normal.astype(bool)
    ix=np.searchsorted(steps,point_steps);keep=(ix<len(steps))&(sides>=0)&(sides<2)
    keep&=steps[np.minimum(ix,len(steps)-1)]==point_steps
    ix=ix[keep];side=sides[keep];force=np.linalg.norm(impulses[keep],axis=1)/dt
    n=normals[keep];length=np.linalg.norm(n,axis=1)
    resolved=np.isfinite(force)&np.isfinite(n).all(1)&(length>0)
    ix=ix[resolved];side=side[resolved];force=force[resolved];n=n[resolved]/length[resolved,None]
    projection=np.sqrt(np.maximum(0.,1.-(n@axis)**2))
    np.add.at(normal,(ix,side),force);np.add.at(tangent_budget,(ix,side),force*projection)
    seen[ix,side]=True
    return normal,tangent_budget,seen


def residual_axis_capacity(tangent_budget,off_axis_friction,mu):
    """Empirical friction budget after observed transverse tangential loading."""
    return np.sqrt(np.maximum((mu*np.asarray(tangent_budget))**2
                              -np.asarray(off_axis_friction)**2,0.)).sum(1)


def bounds(normal,tangent_budget,friction,pads,mask,*,resolved_normal_n,axis=None):
    ratios=[]
    for side in (0,1):
        m=mask&(normal[:,side]>resolved_normal_n)
        if not m.any():raise ValueError('NATIVE_NORMAL_AND_FRICTION_SUPPORT_REQUIRED')
        ratios.append(float(np.percentile(np.linalg.norm(friction[m,side],axis=1)/normal[m,side],95)))
    result=dict(used_traction_coefficient_lower=min(ratios),per_finger_used_ratio_p95=ratios,
        normal_scalar_sum_lower_n=float(np.percentile(normal[mask].sum(1),5)),
        axis_tangent_normal_budget_lower_n=float(np.percentile(tangent_budget[mask].sum(1),5)),
        measured_pad_normal_lower_n=np.percentile(normal[mask],5,axis=0).tolist(),
        supervisor_pad_load_upper_n=np.max(pads[mask],axis=0).tolist(),
        supervisor_pad_load_median_n=np.median(pads[mask],axis=0).tolist(),
        friction_point_value_identified=False,geometry_and_normal_mapping_assumed_stationary=True)
    if axis is not None:
        f=friction[mask];other=f-np.einsum('nsi,i->ns',f,axis)[:,:,None]*axis
        result['joint_axis_tangent_normal_budget_n']=tangent_budget[mask].tolist()
        result['joint_off_axis_tangent_friction_n']=np.linalg.norm(other,axis=2).tolist()
        result['axis_capacity_scope']='per-finger residual friction budget; joint native samples preserve pad-load redistribution; empirical transfer, not a future guarantee'
    return result


def observe(episode,request_n):
    from diagnose_probing_v3 import columns
    p=Path(episode);a,index=columns(p)
    if json.loads((p/'PROCESS_COMPLETION.json').read_text()).get('actor_exited') is not True:
        raise ValueError('CLOSED_MEASUREMENT_EPISODE_REQUIRED')
    prior=json.loads((p/'identification_input.json').read_text())['prior']
    if prior['joint_type']!='prismatic':raise ValueError('DRAWER_CONTACT_CALIBRATION_ONLY')
    axis=np.asarray(prior['axis_world'],float);axis/=np.linalg.norm(axis)
    normal=np.zeros((len(a['step']),2));budget=normal.copy();seen=normal.astype(bool)
    contact=json.loads((p/'contact_evidence/index.json').read_text())
    for record in contact['files']:
        with np.load(p/'contact_evidence'/record['file']) as d:
            m=d['kind']==0
            n,b,s=point_normal_sums(a['step'],d['step'][m],d['finger'][m],
                d['impulse_world'][m],d['normal_world'][m],axis,contact['physics_dt_s'])
            normal+=n;budget+=b;seen|=s
    # Independent task VALIDATION must not improve contact/model selection.
    eligible=[s['id'] for s in index['segments'] if s['role'].startswith('FIT')
        and s.get('result',{}).get('status')=='TARGET_REACHED'
        and s.get('result',{}).get('physical_transmission_valid') is True]
    mask=np.isin(a['segment'],eligible)&(a['phase']==index['metadata']['phases'].index('OPEN_CONTINUE'))
    attempts=[int(f.stem)+1 for f in (p/'attempt_inputs').glob('*.json')
        if f.stem.isdigit() and json.loads(f.read_text()).get('requested_preload_n')==request_n]
    if not attempts:raise ValueError('RECORDED_REQUESTED_PRELOAD_ASSOCIATION_REQUIRED')
    mask&=np.isin(a['attempt_id_v75'],attempts)
    mask&=a['friction_api_valid'].astype(bool).all(1)&(a['friction_anchor_counts'].sum(1)>0)
    mask&=(a['safety_stop']==0)&a['ft_valid'].astype(bool)&seen.any(1)
    if not mask.any():raise ValueError('NO_COMPLETE_NATIVE_TRANSMITTING_CONTACT_CALIBRATION')
    threshold=json.loads((Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json').read_text())['force']['unloaded_pad_n']
    result=bounds(normal,budget,a['friction_force_on_fingers_world_N'],a['pad_load'],mask,
        resolved_normal_n=threshold,axis=axis)
    if 'FORCE_HOLD' in index['metadata']['phases']:
        hold=(a['phase']==index['metadata']['phases'].index('FORCE_HOLD'))&seen.any(1)
        hold&=np.isin(a['attempt_id_v75'],attempts)&(a['safety_stop']==0)
        if hold.any():
            result['stationary_grip_hold_calibration']=dict(samples=int(hold.sum()),
                holding_contact_is_not_an_unloaded_boundary=True,
                summed_native_normal_median_n=float(np.median(normal[hold].sum(1))),
                per_finger_native_normal_median_n=np.median(normal[hold],axis=0).tolist(),
                supervisor_pad_median_n=np.median(a['pad_load'][hold],axis=0).tolist(),
                aperture_median_m=float(np.median(a['aperture'][hold])))
    result.update(schema='native-measured-contact-calibration/v1',requested_preload_n=float(request_n),
        samples=int(mask.sum()),independent_validation_used=False,hidden_dynamics_used=False,
        normal_force_is_requested_preload=False,
        normal_measurement='sum of individual positive native normal impulse magnitudes / dt',
        supervisor_measurement='original normal-only pad report; retained separately',
        source_telemetry_sha256=hashlib.sha256((p/'telemetry/index.json').read_bytes()).hexdigest(),
        source_contact_index_sha256=hashlib.sha256((p/'contact_evidence/index.json').read_bytes()).hexdigest(),
        source_contact_files=[dict(file=row['file'],sha256=hashlib.sha256(
            (p/'contact_evidence'/row['file']).read_bytes()).hexdigest()) for row in contact['files']],
        actual_slip_not_inferred_from_request_or_coefficient=True)
    return result


def capabilities(normal_calibrations,current_contact):
    mu=float(current_contact['used_traction_coefficient_lower'])
    if not np.isfinite(mu) or mu<=0:raise ValueError('OBSERVED_POSITIVE_TRACTION_BOUND_REQUIRED')
    rows=[]
    commitment=lambda d:hashlib.sha256(json.dumps(d,sort_keys=True,allow_nan=False).encode()).hexdigest()
    for cal in normal_calibrations:
        capacity=residual_axis_capacity(cal['joint_axis_tangent_normal_budget_n'],
            cal['joint_off_axis_tangent_friction_n'],mu)
        rows.append(dict(request_n=cal['requested_preload_n'],
            transmitted_axis_force_lower_n=float(np.percentile(capacity,5)),
            measured_pad_normal_lower_n=cal['measured_pad_normal_lower_n'],
            measured_pad_normal_upper_n=cal['supervisor_pad_load_upper_n'],
            source='native measured traction lower bound '+commitment(current_contact)+
                '; same-geometry measured normal calibration '+commitment(cal)+
                '; transfer is empirical, not a guarantee of future contact load'))
    return rows
