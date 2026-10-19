"""Post-run evidence summary only. Never imported by the physical owner."""
import argparse, gzip, json, pickle
from pathlib import Path
import numpy as np


def load(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default


def rows_at(episode):
    index=load(episode/'observations_chunks/index.json',{})
    for chunk in index.get('chunks',[]):
        with gzip.open(episode/'observations_chunks'/chunk['file'],'rb') as f:
            yield from pickle.load(f)


def main(root,output):
    ep=root/'episode';report=load(ep/'report.json',{})
    attempts=load(ep/'CONTACT_ATTEMPTS.json',[]);segments=load(ep/'HYBRID_SEGMENTS.json',[])
    push=load(ep/'HYBRID_PANEL_RESULT.json',{});plan=load(ep/'HYBRID_PANEL_PLAN.json',{})
    phases={};modes={};events=[];contacts=[];paths=set(plan.get('target_paths',[]))
    count=0;ft_valid=0;maximum=None;final=None;positive=0.;negative=0.;margin=None;peak_pad=0.;max_speed=0.;loaded_speed=0.;previous=None;step_dt=[]
    first_positive={};first_threshold={}
    loaded=[(r['acquisition']['t'],r['end_sim_s']) for r in attempts if r.get('acquisition')]
    for r in rows_at(ep):
        count+=1;ft_valid+=bool(r.get('wrist_ft',{}).get('valid',False));angle=r['door_angle_deg'];final=angle
        maximum=angle if maximum is None else max(maximum,angle)
        margin=r['margin_rad'] if margin is None else min(margin,r['margin_rad'])
        peak_pad=max(peak_pad,max(r.get('forces_n',{}).values(),default=0.))
        if previous is not None:
            delta=angle-previous['door_angle_deg'];positive+=max(delta,0.);negative+=max(-delta,0.)
            dt=r['t']-previous['t'];step_dt.append(dt)
            speed=float(np.linalg.norm(np.asarray(r['T_tcp'])[:3,3]-np.asarray(previous['T_tcp'])[:3,3])/dt*1000)
            max_speed=max(max_speed,speed)
            if any(lo<=r['t']<=hi for lo,hi in loaded):loaded_speed=max(loaded_speed,speed)
        for finger,force in r.get('forces_n',{}).items():
            contact=dict(t=r['t'],phase=r['phase'],measured_aperture_mm=1000*r['aperture_m'],
                commanded_aperture_mm=1000*r.get('telemetry',{}).get('command_aperture_m',r['aperture_m']),normal_load_n=force)
            if force>0 and finger not in first_positive:first_positive[finger]=contact
            if force>=.05 and finger not in first_threshold:first_threshold[finger]=contact
        for field,destination in [('phase',phases),('interaction_mode',modes)]:
            label=r.get(field,'UNLABELED');v=destination.setdefault(label,dict(samples=0,first_s=r['t'],last_s=r['t'],angle_min_deg=r['door_angle_deg'],angle_max_deg=r['door_angle_deg']))
            v['samples']+=1;v['last_s']=r['t'];v['angle_min_deg']=min(v['angle_min_deg'],r['door_angle_deg']);v['angle_max_deg']=max(v['angle_max_deg'],r['door_angle_deg'])
        if r.get('force_event',{}).get('status') not in [None,'NORMAL']:events.append(dict(t=r['t'],phase=r['phase'],kind='original_force',value=r['force_event']))
        if r.get('panel_wrist_guard',{}).get('status') not in [None,'NORMAL']:events.append(dict(t=r['t'],phase=r['phase'],kind='panel_wrist',value=r['panel_wrist_guard']))
        if r.get('interaction_mode')=='PANEL_PUSH':
            hits=[c for c in r.get('contacts',[]) if c['target'] in paths and c['force_n']>0]
            if hits:contacts.append(dict(t=r['t'],angle_deg=angle,contacts=hits))
        previous=r
    intervals=[]
    for contact in contacts:
        if not intervals or contact['t']-intervals[-1]['end_s']>1/240+1e-6:
            intervals.append(dict(start_s=contact['t'],end_s=contact['t'],samples=1))
        else:intervals[-1]['end_s']=contact['t'];intervals[-1]['samples']+=1
    model_updates=[]
    for segment in segments:
        # Actor folders may have moved into a private archive. Resolve using
        # the local segment sequence, never by loading an unrelated old run.
        folder=ep/'hybrid_identification'/('%02d'%len(model_updates))
        m=load(folder/'IDENTIFIED_MODEL.json',{})
        data=folder/'CUMULATIVE_DATA.npz';effort={}
        if data.exists():
            with np.load(data,allow_pickle=True) as z:
                effort=dict(valid_samples=int(z['valid'].sum()),raw_samples=len(z['t']),
                    F_T_derived_torque_range_nm=[float(z['torque'].min()),float(z['torque'].max())],
                    measured_angle_range_deg=np.rad2deg([z['q'].min(),z['q'].max()]).tolist(),
                    velocity_range_rad_s=[float(z['qd'].min()),float(z['qd'].max())],
                    acceleration_range_rad_s2=[float(z['qdd'].min()),float(z['qdd'].max())])
        model_updates.append(dict(label=segment['label'],result=segment['result'],measurement=effort,
            model_status=m.get('status'),training_samples=m.get('training_samples'),validation_samples=m.get('validation_samples'),
            selected_model=m.get('selected_model'),models=m.get('models'),coverage=m.get('coverage'),
            heldout_truth_used_for_fitting=False))
    dt=1/240
    result=dict(schema='hybrid-posthoc-evidence-v0',valid_physical_grasp_attempts=report.get('physical_grasp_attempts',0),
        maximum_actual_angle_deg=maximum,final_actual_angle_deg=final,
        raw_240hz_positive_travel_deg=positive,raw_240hz_negative_travel_deg=negative,
        travel_note='raw total variation includes physical/solver jitter; segment endpoint travel reported separately, no smoothing of peaks',
        handle_acquisitions=sum(r.get('acquisition') is not None for r in attempts),
        handle_release_completions=sum(bool(r.get('acquisition') and r.get('safe_exit',{}).get('released')) for r in attempts),
        free_space_compact_hand_opening=bool(push.get('separation',{}).get('released')),
        planned_regrasp_attempts=max(0,report.get('physical_grasp_attempts',0)-1),unexpected_recovery_acquisitions=0,
        attempts=attempts,panel_push=push,panel_contact_intervals=intervals,panel_contact_samples=len(contacts),
        modes=modes,phases=phases,force_events=events,model_updates=model_updates,
        minimum_joint_margin_rad=margin,maximum_measured_TCP_speed_mm_s=max_speed,
        maximum_post_acquisition_TCP_speed_mm_s=loaded_speed,peak_raw_pad_n=peak_pad,
        first_positive_normal_contacts=first_positive,first_original_0p05N_contacts=first_threshold,
        native_records=count,native_dt_s=dt,wrist_ft_valid=ft_valid,
        measured_step_dt_range_s=[min(step_dt),max(step_dt)] if step_dt else None,
        simulation_s=report.get('simulation_s'),wall_s=report.get('wall_s'),
        final_status=report.get('status'),hybrid_result=load(ep/'HYBRID_EPISODE_RESULT.json'),
        known_dependencies=report.get('GT_dependencies'),private_raw_evidence=True)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:result[k] for k in ['maximum_actual_angle_deg','final_actual_angle_deg','handle_acquisitions','panel_contact_samples','native_records','wrist_ft_valid','simulation_s','wall_s','final_status']},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.root,a.output)
