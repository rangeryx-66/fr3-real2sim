"""Post-episode derived engineering metrics; no simulator or raw-data copies."""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts')]
from audit_prismatic_v73 import audit, read, digest
from diagnose_probing_v3 import columns
from update_probing_v7_model import load_segments
from update_prismatic_v73_model import qualify
from xarm_ft_v31_estimator import common_filter


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2)))


def model_summary(model, reference):
    m = model.get('selected', {})
    if not m:
        return None
    beta = m['parameters']
    return dict(parameters=beta, parameter_names=m['parameter_names'],
                conditional_std=m['conditional_std'], scaled_condition=m['scaled_condition'],
                support_intervals_native=m['support_intervals_native'],
                mass_relative_error_percent=100*(beta[0]/reference['mass_kg']-1),
                damping_relative_error_percent=100*(beta[1]/reference['damping_N_s_m']-1),
                constant_friction_relative_error_percent=(100*(beta[2]/reference['friction_N']-1) if len(beta)==3 else None),
                heldout=m.get('heldout_motion'), unfiltered_input_sensitivity=m.get('heldout_unfiltered_input_sensitivity'),
                FIT_qualification=model.get('FIT_qualification_v73'),
                uncertainty=m.get('uncertainty'), validation_used_for_fitting=m.get('validation_used'))


def summarize(root):
    root = Path(root)
    checked = audit(root)
    result = dict(schema='prismatic-v73-engineering-derived', frozen_revolute=checked['frozen_revolute'],
                  V72=checked['V72'], cases=[],
                  interpretation='All attempts preserved; selected engineering successes are not an unbiased benchmark rate. Authored references are not independently measured effective PhysX dynamics.')
    for e in checked['engineering_attempts']:
        p = root/'episodes'/e['id']
        truth = e['authored_reference_post_episode']
        ref = dict(mass_kg=sum(b['mass_kg'] for b in truth['bodies']),
                   damping_N_s_m=truth['configuration']['b'], friction_N=truth['configuration']['tau_c'])
        trace = read(p/'AGENT_ACTION_TRACE.json')
        row = dict(id=e['id'], terminal_status=e['status'], actor_exited=e['actor_exited'],
                   wall_s=e['wall_s'], storage_bytes=e['storage_bytes'],
                   authored_reference_post_episode=ref, attempts=[],
                   source_sha256=e['source_sha256'], private_evidence_index=e['private_evidence_index'],
                   model_online=model_summary(read(p/'model/estimate.json'), ref),
                   worker=read(p/'model/WORKER_STATUS.json'))
        row['Agent_operations'] = [dict(sequence=x['command']['sequence'], operation=x['command']['operation']) for x in trace]
        decisions = p/'CODEX_DECISIONS.jsonl'
        row['Agent_decisions'] = []
        if decisions.exists():
            for line in decisions.read_text().splitlines():
                decision = json.loads(line)
                inspected = decision.pop('inspected', [])
                decision['inspected_private_images'] = [dict(reference=f, retained=(p/f).exists(), sha256=digest(p/f) if (p/f).exists() else None) for f in inspected]
                row['Agent_decisions'].append(decision)
        for x in trace:
            if x['command']['operation'] != 'attempt_grasp_and_probe':
                continue
            a = x['result']; exit_ = a.get('safe_exit') or {}; park = exit_.get('park') or {}
            row['attempts'].append(dict(acquisition=a.get('acquisition'), requested_preload_n=x['command'].get('native_g1_preload_request_n'),
                family=a.get('family'), physical_grasp_started=a.get('physical_grasp_started', False), error=a.get('error'),
                actions_declared_before_execution=x['command'].get('segments'),
                segments=[{k:s.get(k) for k in ('status','reason','gain_native','joint_coordinate','elapsed_sim_s','requested_target_reached','physical_transmission_valid','final_tracking_lag_native')} for s in a.get('segments',[])],
                safe_exit=exit_.get('safe'), parked=park.get('reached'), final_pad_loads_N=park.get('pad_loads_n'), final_joint_q=park.get('measured_q'), final_drawer_m=park.get('joint_coordinate')))
        index = read(p/'telemetry/index.json')
        row['simulation_s'] = index.get('physics_steps_seen',0)/240
        row['numeric_records'] = index.get('records')
        row['observation_count'] = len(list((p/'rgbd').glob('generation*/*rgb.png')))
        if not row['observation_count']:
            row['observation_count'] = len(list((p/'rgbd').glob('generation*/rgb.png')))
        if row['model_online']:
            a, _ = columns(p); pieces, settings, _ = load_segments(p, True)
            axis = np.array(settings['prior']['axis_world']); axis /= np.linalg.norm(axis)
            row['motion_diagnostics'] = []
            for d in pieces:
                if read(p/'model/estimate.json').get('FIT_qualification_v73'):
                    qualify(d)
                v = d['valid']; ix = np.minimum(np.searchsorted(a['t'],d['t']), len(a['t'])-1)
                tcp = a['tcp_pose'][ix,:3]@axis; cmd = a['command_TCP_pose'][ix,:3]@axis
                normal = -a['normal_force_on_fingers_world_N'][ix].sum(1)@axis
                friction = -a['friction_force_on_fingers_world_N'][ix].sum(1)@axis
                row['motion_diagnostics'].append(dict(block=d['block_id'], coordinate_range_m=[float(d['q'].min()),float(d['q'].max())],
                    qualified_samples=int(v.sum()), velocity_rms_m_s=rms(d['qd'][v]), acceleration_rms_m_s2=rms(d['qdd'][v]),
                    incremental_command_to_measured_TCP_rail_RMS_m=rms((cmd-cmd[0])-(tcp-tcp[0])),
                    incremental_measured_TCP_to_drawer_rail_RMS_m=rms((tcp-tcp[0])-(d['q']-d['q'][0])),
                    command_measured_tracking_delta_m=float((cmd[-1]-cmd[0])-(tcp[-1]-tcp[0])),
                    measured_TCP_drawer_relative_delta_m=float((tcp[-1]-tcp[0])-(d['q'][-1]-d['q'][0])),
                    pad_load_peak_N=a['pad_load'][ix].max(0).tolist(), native_normal_rail_mean_N=float(normal[v].mean()),
                    native_friction_rail_mean_N=float(friction[v].mean()), wrist_force_rms_N=rms(d['torque'][v]),
                    wrist_minus_native_rail_traction_RMS_N=rms((d['torque']-common_filter(normal+friction))[v]),
                    friction_API_valid_fraction=a['friction_api_valid'][ix].mean(0).tolist()))
            row['FIT_consistency'] = read(p/'model/FIT_CONSISTENCY_V73.json')
        alias = e['id'].split('_')[0]
        replay = root/'qualified_replays'/alias/'estimate.json'
        if replay.exists() and not read(p/'model/estimate.json').get('FIT_qualification_v73') and row['model_online']:
            row['qualified_offline_replay_not_online_replacement'] = model_summary(read(replay),ref)
            row['qualified_offline_replay_sha256'] = digest(replay)
        result['cases'].append(row)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('root',type=Path); parser.add_argument('--output',type=Path,required=True)
    a = parser.parse_args(); a.output.write_text(json.dumps(summarize(a.root),indent=2,allow_nan=False))
