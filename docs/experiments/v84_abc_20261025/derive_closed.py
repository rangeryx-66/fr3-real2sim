"""Post-hoc closed-episode audit. Whitelisted measured evidence only; no plant reads."""
import argparse, collections, hashlib, json, math, pathlib
import numpy as np

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def audit(ep, dest):
    def read(name, default=None):
        p=ep/name
        return json.loads(p.read_text()) if p.exists() else default
    completion=read('PROCESS_COMPLETION.json')
    if not completion or not completion.get('actor_exited'):
        raise ValueError('Only closed episodes may receive a final audit')
    idx=read('telemetry/index.json'); chunks=[]; hashes={}
    cols=['step','t','segment','phase','joint_q','joint_dq','reference_q',
          'tcp_pose','command_TCP_pose','pad_load','aperture','relative_slip_m',
          'safety_stop','load_warning','ft_valid','dq']
    for row in idx['chunks']:
        p=ep/'telemetry'/row['file']; h=sha(p)
        if h!=row['sha256']:raise ValueError('chunk hash mismatch: '+str(p))
        hashes[row['file']]=h
        with np.load(p) as a:chunks.append({k:a[k] for k in cols if k in a})
    data={k:np.concatenate([a[k] for a in chunks]) for k in chunks[0]}
    phases=idx['metadata']['phases']; active=phases.index('OPEN_CONTINUE')
    kind=idx['metadata']['articulation']['joint_type']; unit='deg' if kind=='revolute' else 'mm'
    scale=180/math.pi if kind=='revolute' else 1000
    def stats(a):
        a=np.asarray(a);a=a[np.isfinite(a)]
        return dict(mean=float(np.mean(a)),p95=float(np.percentile(a,95)),max=float(np.max(a))) if len(a) else None
    segments=[]
    for s in idx['segments']:
        mask=(data['segment']==s['id'])&(data['phase']==active)
        ii=np.flatnonzero(mask);r=s.get('result',{});fb=r.get('abc_feedback',{})
        row=dict(id=s['id'],role=s['role'],completed=s.get('completed'),target=s['target_q']*scale,
                 end=(r['joint_coordinate']*scale if r.get('joint_coordinate') is not None else None),
                 gain=(r['gain_native']*scale if r.get('gain_native') is not None else None),
                 status=r.get('status'),reason=r.get('reason'),sim_s=r.get('elapsed_sim_s'),
                 tracking_limited_fraction=r.get('tracking_limited_fraction'),
                 advisory_windows=sum(bool(x.get('advisory')) for x in fb.get('windows',[])),
                 actual_bad_windows=sum(bool(x.get('suspect')) for x in fb.get('windows',[])),
                 FIT_exclude_entire_segment=fb.get('FIT_exclude_entire_segment',False),
                 declared_action=s.get('declared_action'),samples=int(len(ii)))
        if len(ii):
            err=np.linalg.norm(data['tcp_pose'][ii,:3]-data['command_TCP_pose'][ii,:3],axis=1)*1000
            row.update(measured_last_coordinate=float(data['joint_q'][ii[-1]])*scale,
                       recorded_phase_span_s=float(data['t'][ii[-1]]-data['t'][ii[0]]),lead=stats(np.abs(data['reference_q'][ii]-data['joint_q'][ii])*scale),
                       TCP_servo_mm=stats(err),pad_N=[stats(data['pad_load'][ii,j]) for j in range(2)],
                       aperture_minmax_mm=[float(np.min(data['aperture'][ii])*1000),float(np.max(data['aperture'][ii])*1000)],
                       contact_relative_slip_mm=stats(data['relative_slip_m'][ii]*1000),
                       actual_velocity=stats(np.abs(data['joint_dq'][ii])*scale))
            if len(ii)>=3:
                row['derived_acceleration_abs']=stats(np.abs(np.gradient(data['joint_dq'][ii].astype(float),data['t'][ii]))*scale)
                row['acceleration_scope']='Segment-local derivative of measured velocity; noisy diagnostic, not an independent acceleration sensor or estimator input.'
        segments.append(row)
    trace=read('AGENT_ACTION_TRACE.json',[]); waits=read('ABC_WAITS.json',{}).get('waits',[])
    byop=collections.defaultdict(float);calls=collections.Counter()
    for row in trace:
        op=row['command']['operation'];byop[op]+=row.get('wall_s',0);calls[op]+=1
    waittot=collections.defaultdict(float)
    for row in waits:waittot[row['reason']]+=row['wall_s']
    ledger=[]; checkout=pathlib.Path(read('PROCESS.json')['checkout'])
    for name in ['V81_LOADED_SOURCES.json','SOURCE_PROVENANCE.json','V81_PRELAUNCH.json']:
        d=read(name)
        if d is not None:
            entries=d.get('sources',d.get('source_sha256',{}));issues=[]
            for rel,expected in entries.items():
                source=checkout/rel
                if not source.is_file() or sha(source)!=expected:issues.append(rel)
            ledger.append(dict(file=name,sha256=sha(ep/name),entries=len(entries),issues=issues))
    seals=[]
    for p in sorted((ep/'validation_model_seals').glob('*.json')):
        d=json.loads(p.read_text());seals.append(dict(file=p.name,sha256=sha(p),seal=d))
    vreq=[]
    for row in read('VALIDATION_REQUESTS.json',{}).get('requests',[]):
        result=row.get('result',{})
        vreq.append(dict(id=row['id'],action=row['action'],status=row['status'],
                         approved_before_execution=row.get('approved_before_execution'),
                         created_unix_s=row.get('created_unix_s'),seal=row.get('seal'),
                         failed_attempts_count=len(row.get('failed_attempts',[])),
                         result={k:result.get(k) for k in ['status','reason','joint_coordinate','gain_native','elapsed_sim_s','requested_target_reached','safely_stopped','physical_transmission_valid']}))
    final_exit=trace[-1].get('result',{}).get('safe_exit',{}) if trace else {}
    park=final_exit.get('park',{})
    compact_exit=dict(safe=final_exit.get('safe'),reason=final_exit.get('reason'),stop=final_exit.get('stop'),park={k:park.get(k) for k in ['reached','region','unloaded','pad_loads_n','joint_cell_error_rad']},
                      measured_collision_valid=park.get('measured_collision_check',{}).get('valid'))
    out=dict(schema='v84-closed-measured-audit-v2',derivation_source_sha256=sha(pathlib.Path(__file__)),episode_id=ep.name,source_commit=read('PROCESS.json',{}).get('source_commit'),
             coordinate_display_unit=unit,completion=completion,task_progress=read('TASK_PROGRESS.json'),
             final_state=read('AGENT_SESSION_STATUS.json'),physics_steps=idx['physics_steps_seen'],records=idx['records'],
             chunk_sha256=hashes,recorder_issues=idx.get('issues'),hard_stop_records=int(np.count_nonzero(data['safety_stop'])),
             invalid_FT_records=int(np.count_nonzero(~data['ft_valid'].astype(bool))),
             max_actual=float(np.max(data['joint_q'])*scale),final_actual=float(data['joint_q'][-1]*scale),
             final_pad_N=data['pad_load'][-1].tolist(),segments=segments,action_calls=dict(calls),
             operation_wall_s=dict(byop),unloaded_decision_wait_s=sum(x.get('unloaded_wait_s',0) for x in trace),
             waits_by_reason_s=dict(waittot),waits=waits,
             trace_partition_unattributed_s=completion['wall_s']-sum(byop.values())-sum(x.get('unloaded_wait_s',0) for x in trace),
             interaction_sim_s=sum(s.get('result',{}).get('elapsed_sim_s',0) for s in idx['segments']),
             interaction_reported_duration_scope='Sum of result-reported elapsed seconds; hard-interrupted actions may omit a duration; recorded count separately includes them.',
             recorded_interaction_240hz_samples=int(np.count_nonzero(data['phase']==active)),
             recorded_interaction_time_s=float(np.count_nonzero(data['phase']==active)/240),
             sim_end_s=float(data['t'][-1]),validation_requests=vreq,validation_model_seals=seals,
             runtime_profile=read('RUNTIME_RESULT.json',{}).get('profile'),
             final_safe_exit=compact_exit,
             source_evidence_ledgers=ledger,raw_evidence_private=True,
             timing_scope='operation walls include physics/render/planning within call; sim is not wall; wait subledgers may overlap operation categories, never add all ledgers')
    dest.mkdir(parents=True,exist_ok=True)
    (dest/'FINAL_AUDIT.json').write_text(json.dumps(out,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(11,9),sharex=True,layout='constrained')
    ax[0].plot(data['t'],data['joint_q']*scale,lw=1,label='Actual object')
    for s in segments:
        ii=np.flatnonzero((data['segment']==s['id'])&(data['phase']==active))
        if len(ii):
            ax[0].axvspan(data['t'][ii[0]],data['t'][ii[-1]],alpha=.12)
            ax[0].text(data['t'][ii[0]],s['end'] if s['end'] is not None else s['measured_last_coordinate'],str(s['id']),fontsize=8)
    ax[0].set_ylabel('Actual joint ('+unit+')');ax[0].legend()
    mm=data['phase']==active
    for k,col in enumerate(['reference_q']):
        ax[1].scatter(data['t'][mm],np.abs(data[col][mm]-data['joint_q'][mm])*scale,s=.4,label='Reference lead')
    ax[1].set_ylabel('Reference lead ('+unit+')');ax[1].legend()
    ax[2].scatter(data['t'][mm],data['pad_load'][mm,0],s=.3,label='Left pad normal')
    ax[2].scatter(data['t'][mm],data['pad_load'][mm,1],s=.3,label='Right pad normal')
    ax[2].set_ylabel('Normal load (N)');ax[2].set_xlabel('Advancing physics time (s), excludes paused Agent wall time');ax[2].legend()
    for a in ax:a.grid(alpha=.2)
    fig.suptitle(ep.name+' | measured states; all failed segments retained')
    fig.savefig(dest/'MOTION.png',dpi=150);plt.close(fig)
    print(json.dumps({k:out[k] for k in ['episode_id','source_commit','max_actual','final_actual','hard_stop_records','final_pad_N','completion','action_calls','unloaded_decision_wait_s','interaction_sim_s']},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=pathlib.Path);p.add_argument('destination',type=pathlib.Path)
    a=p.parse_args();audit(a.episode,a.destination)
