"""Post-shutdown derived metrics only. No physics truth or action publication."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')


def summarize(root):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    process=read(root/'PROCESS_COMPLETION.json');status=read(root/'AGENT_SESSION_STATUS.json')
    assert process['actor_exited'] and status['state']=='CLOSED'
    out=root/'derived_replanning_gate';out.mkdir(exist_ok=True)
    trace=read(root/'AGENT_ACTION_TRACE.json');attempt=trace[2]['result']
    numeric=read(root/'telemetry/index.json');rows=[];motion=[];integrity=[];stops=warnings=0
    for chunk in numeric['chunks']:
        p=root/'telemetry'/chunk['file'];actual=digest(p)
        integrity.append(dict(file=chunk['file'],sha256=actual,matched=actual==chunk['sha256']))
        with np.load(p,allow_pickle=False) as a:
            stops+=int(a['safety_stop'].sum());warnings+=int(a['load_warning'].sum())
            motion.append(np.c_[a['t'],a['joint_q'],a['reference_q'],a['segment']])
    assert all(r['matched'] for r in integrity)
    motion=np.vstack(motion);motion=motion[np.argsort(motion[:,0])]
    for segment in numeric['segments']:
        r=segment.get('result',{})
        rows.append(dict(role=segment['role'],declared_target_deg=math.degrees(segment['target_q']),
            status=r.get('status'),actual_end_deg=math.degrees(r['joint_coordinate']),
            actual_gain_deg=math.degrees(r['gain_native']),elapsed_sim_s=r['elapsed_sim_s'],
            measured_stopped=r['safely_stopped'],transmission_valid=r['physical_transmission_valid']))
    rejection=attempt['continuations'][0]
    park=attempt['safe_exit']['park'];events=read(root/'events.json')
    counts=Counter(e['event'] for e in events)
    source=read(root/'V81_PRELAUNCH.json');loaded=read(root/'V81_LOADED_SOURCES.json')['sources']
    expected=source['source_sha256']
    loaded_checks={k:loaded.get(k)==v for k,v in expected.items() if k in loaded}
    assert loaded_checks and all(loaded_checks.values())
    assert 'agentic_interaction_v81/replanning.py' in loaded_checks
    work=motion[motion[:,3]>0]
    start=work[0,0]
    fig,ax=plt.subplots(figsize=(8,3.7),layout='constrained')
    for i,segment in enumerate(numeric['segments'],1):
        m=motion[motion[:,3]==i]
        ax.plot(m[:,0]-start,np.degrees(m[:,1]),label=segment['role'].replace('FIT_',''))
        ax.plot(m[:,0]-start,np.degrees(m[:,2]),color='gray',linestyle=':',linewidth=.7)
    selected=motion[motion[:,3]==2][0]
    ax.annotate('12° request rejected (injected)\n9° alternative passes native check',
        xy=(selected[0]-start,math.degrees(selected[1])),xytext=(8,10.3),
        arrowprops=dict(arrowstyle='->'),fontsize=8)
    ax.axhline(12,color='crimson',linestyle='--',linewidth=.8,label='12° request: NOT executed')
    ax.set(xlabel='Simulation seconds from first probe',ylabel='Measured door angle (deg)',ylim=(-.4,13))
    ax.grid(alpha=.2);ax.legend(fontsize=7,loc='lower left');fig.savefig(out/'MOTION.png',dpi=160);plt.close(fig)
    physical=dict(status='PASS_SHORT_REPLANNING_GATE',episode_id=status['episode_id'],
        physical_source_commit=source['source_commit'],one_fresh_scene=True,
        actual_acquisitions=sum(e['event']=='PHYSICAL_GRASP_START' for e in events),
        acquisition=attempt['acquisition'],maximum_actual_angle_deg=float(np.degrees(motion[:,1]).max()),
        reverse_travel_deg=sum(-r['actual_gain_deg'] for r in rows if r['actual_gain_deg']<0),segments=rows,
        rejection=dict(reason=rejection['rejected_request']['reason'],
            engineering_injected=rejection['rejected_request']['native_rejection_preserved']['engineering_injected'],
            rejected_requested_target_deg=math.degrees(rejection['rejected_request']['requested_action']['target_q']),
            selected_target_deg=math.degrees(rejection['selected_action']['target_q']),
            selected_original_native_check_valid=rejection['valid'],
            original_rejected_request_completed=False),
        complete_independent_validation=False,validation_status=attempt['segments'][-1]['status'],
        validation_metrics=None,full_range_pass=False,
        park=dict(reached=park['reached'],unloaded=park['unloaded'],
            measured_collision_valid=park['measured_collision_check']['valid'],
            pad_loads_n=park['pad_loads_n'],measured_q=park['measured_q'],door_angle_deg=park['door_angle_deg']),
        hard_safety_stop_samples=stops,informational_warning_samples=warnings,event_counts=dict(counts),
        wall_s=process['wall_s'],simulation_s=numeric['physics_steps_seen']/240,
        operation_wall_s=[dict(operation=r['command']['operation'],wall_s=r['wall_s'],
            unloaded_wait_s=r['unloaded_wait_s'])for r in trace],
        full_target_coverage=trace[1]['result']['target_coverage'],
        raw_chunks_verified=integrity,loaded_source_matches=loaded_checks,actor_exited=True)
    assert physical['actual_acquisitions']==1
    assert all(physical['park'][k] for k in ('reached','unloaded','measured_collision_valid'))
    assert not stops and rejection['valid'] and rows[1]['role']=='FIT_REPLAN_SHORTER_OPEN'
    dump(out/'METRICS.json',physical)
    # A small derived trace, not another copy of the full telemetry/commands.
    dump(out/'AGENT_TRACE.json',dict(observations=read(root/'inspected_boards/INDEX.json'),
        decisions=[dict(sequence=i,operation=r['command']['operation'],reason=r['command'].get('reason'),
            status=r['result'].get('status'),wall_s=r['wall_s'])for i,r in enumerate(trace)],
        deterministic_rejection_and_alternative=rejection))
    excluded={'PRIVATE_EVIDENCE_SHA256.json','derived_replanning_gate/STORAGE.json'}
    files={str(p.relative_to(root)):digest(p) for p in sorted(root.rglob('*'))
           if p.is_file() and str(p.relative_to(root)) not in excluded}
    dump(root/'PRIVATE_EVIDENCE_SHA256.json',dict(files=files,post_shutdown=True,
        RGBD_FT_telemetry_private=True,no_raw_copy=True,excluded_late_index_files=sorted(excluded)))
    for _ in range(3):
        categories=Counter()
        for p in root.rglob('*'):
            if p.is_file():categories[p.relative_to(root).parts[0] if p.relative_to(root).parts[0] in
                ('telemetry','rgbd','contact_evidence','inspected_boards')else'derived_source_metadata']+=p.stat().st_size
        storage=dict(bytes_by_category=dict(categories),total_bytes=sum(categories.values()))
        dump(out/'STORAGE.json',storage)
    return dict(metrics=str(out/'METRICS.json'),storage=storage,
        private_evidence_sha256=digest(root/'PRIVATE_EVIDENCE_SHA256.json'))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path)
    print(json.dumps(summarize(p.parse_args().episode),indent=2))
