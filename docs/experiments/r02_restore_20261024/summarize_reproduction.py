"""Read-only R02 reproduction summary; never fit or change an online model."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(episode, output):
    completion = episode / 'PROCESS_COMPLETION.json'
    if not completion.exists():
        raise ValueError('ACTOR_MUST_FINISH_FIRST')
    if output.is_relative_to(episode):
        raise ValueError('DERIVED_OUTPUT_MUST_NOT_OVERWRITE_EPISODE')
    output.mkdir(parents=True, exist_ok=True)
    estimate = episode / 'model/estimate.json'
    original_model_hash = sha(estimate)
    index = json.loads((episode / 'telemetry/index.json').read_text())
    chunks = index['chunks']
    arrays = []
    for item in chunks:
        path = episode / 'telemetry' / item['file']
        if sha(path) != item['sha256']:
            raise ValueError('NUMERIC_CHUNK_INTEGRITY_FAILURE:' + item['file'])
        arrays.append(np.load(path))
    keys = ['t', 'joint_q', 'segment', 'tcp_pose', 'command_TCP_pose',
            'relative_slip_m', 'aperture', 'command_aperture', 'pad_load', 'safety_stop']
    data = {k: np.concatenate([a[k] for a in arrays]) for k in keys}
    prior = json.loads((episode / 'identification_input.json').read_text())['prior']
    axis, origin = np.array(prior['axis_world']), np.array(prior['origin_world_m'])
    axis = axis / np.linalg.norm(axis)
    segments = []
    curves = []
    for record in index['segments']:
        mask = data['segment'] == record['id']
        q = data['joint_q'][mask]
        if not len(q):
            continue
        position = data['tcp_pose'][mask, :3]
        commanded = data['command_TCP_pose'][mask, :3]
        rotation = Rotation.from_rotvec((q-q[0])[:, None] * axis)
        expected = rotation.apply(np.broadcast_to(position[0]-origin, position.shape).copy()) + origin
        tracking = np.linalg.norm(commanded-position, axis=1) * 1000
        rigid_error = np.linalg.norm(expected-position, axis=1) * 1000
        result = record.get('result', {})
        complete = result.get('status') == 'TARGET_REACHED' and result.get('requested_target_reached') is True
        segments.append(dict(id=record['id'], role=record['role'], status=result.get('status'),
            reason=result.get('reason'), target_reached=complete,
            start_deg=float(np.degrees(q[0])), end_deg=float(np.degrees(q[-1])),
            maximum_deg=float(np.degrees(q.max())), net_deg=float(np.degrees(q[-1]-q[0])),
            recorded_motion_s=float(data['t'][mask][-1]-data['t'][mask][0]),
            command_measured_TCP_error_mm_p50_p95_max=np.nanpercentile(tracking, [50,95,100]).tolist(),
            per_segment_rigid_arc_error_mm_p50_p95_max=np.nanpercentile(rigid_error, [50,95,100]).tolist(),
            grasp_relative_drift_mm_max=float(data['relative_slip_m'][mask].max()*1000),
            measured_aperture_mm_min_max=(np.array([data['aperture'][mask].min(),data['aperture'][mask].max()])*1000).tolist(),
            commanded_aperture_mm_min_max=(np.array([data['command_aperture'][mask].min(),data['command_aperture'][mask].max()])*1000).tolist()))
        curves.append((record, mask, tracking, rigid_error))
    trace = json.loads((episode / 'AGENT_ACTION_TRACE.json').read_text())
    attempts = [e['result'] for e in trace if e['command']['operation'] == 'attempt_grasp_and_probe']
    finished = trace[-1]['result'] if trace[-1]['command']['operation'] == 'finish' else {}
    seals = [{k: s.get(k) for k in ['model_sha256','declared_action','object_coordinate']}
             for s in (json.loads(f.read_text()) for f in sorted((episode/'validation_model_seals').glob('*.json')))]
    storage = {}
    for f in episode.rglob('*'):
        if f.is_file():
            category = f.relative_to(episode).parts[0] if f.parent != episode else 'root'
            storage[category] = storage.get(category, 0) + f.stat().st_size
    result = dict(episode_id=episode.name, status='R02_COMPLETE_REPRODUCTION_NOT_PASS',
        maximum_actual_opening_deg=float(np.degrees(data['joint_q'].max())),
        largest_completed_closing_deg=max((-s['net_deg'] for s in segments if s['target_reached'] and s['net_deg']<0), default=0),
        segments=segments, physical_acquisitions=sum(a.get('acquisition',{}).get('acquired',False) for a in attempts),
        verified_one_second_holds=sum(a.get('acquisition',{}).get('hold_s') == 1.0 for a in attempts),
        guarded_attempt_exits=[a.get('safe_exit',{}).get('safe') for a in attempts],
        final_safe_exit=finished.get('safe_exit',{}).get('safe'),
        final_safe_park=finished.get('safe_exit',{}).get('park',{}).get('reached'),
        final_angle_deg=finished.get('safe_exit',{}).get('park',{}).get('door_angle_deg'),
        final_pad_loads_n=finished.get('safe_exit',{}).get('park',{}).get('pad_loads_n'),
        hard_stop_records=int(data['safety_stop'].sum()),
        qualified_FIT=json.loads((episode/'model/FIT_READINESS_V81.json').read_text()),
        validation_seals=seals,
        complete_independent_validation_segments=sum(s['role'].startswith('VALIDATION') and s['target_reached'] for s in segments),
        independent_heldout_torque_RMSE=None, independent_forward_angle_RMSE=None,
        independent_prediction_reason='INCOMPLETE_PHYSICAL_VALIDATION; no success metric from a partial segment',
        online_model_sha256=original_model_hash, numeric_chunks_verified=len(chunks),
        storage_bytes_by_category=storage, total_episode_bytes=sum(storage.values()),
        process=json.loads(completion.read_text()), source_modified=False,
        optional_correctness_ports_applied=False, ninety_degree_episode_started=False)
    if sha(estimate) != original_model_hash:
        raise ValueError('ONLINE_MODEL_CHANGED_DURING_READ_ONLY_SUMMARY')
    (output/'METRICS.json').write_text(json.dumps(result, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    for record, mask, tracking, rigid_error in curves:
        label = f"{record['id']}: {record['role']}"
        axes[0].plot(data['t'][mask], np.degrees(data['joint_q'][mask]), label=label)
        if record['id'] in (1, 4):
            axes[1].plot(np.degrees(data['joint_q'][mask]), rigid_error, label=f"segment{record['id']} rigid-arc deviation")
            axes[1].plot(np.degrees(data['joint_q'][mask]), tracking, '--', label=f"segment{record['id']} command tracking")
    axes[0].axhline(63.086079212149805, color='black', alpha=.4, linestyle=':', label='historical R02 maximum')
    axes[0].set(xlabel='simulation time (s)', ylabel='measured door angle (deg)')
    axes[1].set(xlabel='measured door angle (deg)', ylabel='position residual (mm)')
    for ax in axes:
        ax.grid(alpha=.2); ax.legend(fontsize=7)
    fig.suptitle('Exact R02 source, fresh observations: range recovered; validation transmission failed')
    fig.tight_layout(); fig.savefig(output/'MOTION_AND_CONTACT.png', dpi=140); plt.close(fig)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    r = summarize(a.episode.resolve(), a.output.resolve())
    print(json.dumps({k:r[k] for k in ['maximum_actual_opening_deg','largest_completed_closing_deg',
        'complete_independent_validation_segments','final_safe_park','hard_stop_records','total_episode_bytes']}))
