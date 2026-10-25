"""Closed-episode, exact pre-execution seal prediction. No fitting or scene truth.

Run with the frozen runtime checkout on PYTHONPATH (including scripts/).
Only original measured-input processing and prediction functions are called.
Outputs derived metrics; does not write episode models or raw data.
"""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from diagnose_probing_v3 import columns, rows_at
from interaction_runtime.segment_quality import selection
from xarm_ft_v7_estimator import process, samples, prediction


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def audit(episode, output):
    assert (episode/'PROCESS_COMPLETION.json').exists(), 'closed episodes only'
    a, index = columns(episode)
    phases = np.array(index['metadata']['phases'])[a['phase']]
    packets = {sha(p): p for p in sorted((episode/'numeric_packets').glob('*/model/estimate.json'))}
    out = dict(schema='v84-exact-frozen-heldout-v1', fitting_called=False,
               hidden_truth_used=False, validation_used_for_selection=False,
               source_sha256=sha(Path(__file__)), segments=[])
    for segment in index['segments']:
        if not segment['role'].startswith('VALIDATION'):
            continue
        sid = segment['id']
        record = dict(id=sid, role=segment['role'], blocks=[])
        out['segments'].append(record)
        ix = np.flatnonzero((a['segment']==sid) & np.isin(phases, ['OPEN_CONTINUE', 'PANEL_PUSH']))
        ix, quality = selection(segment, a, ix, True)
        record['quality'] = quality
        if not len(ix):
            record['status'] = 'INCOMPLETE_VALIDATION_EXCLUDED'
            continue
        sealpath = episode/'validation_model_seals'/('%02d.json'%sid)
        seal = json.loads(sealpath.read_text())
        modelpath = packets[seal['model_sha256']]
        model = json.loads(modelpath.read_text())
        assert model['selected']['parameters'] == seal['selected']['parameters']
        assert not model['validation_used_for_fit_or_selection']
        settingspath = modelpath.parent.parent/'identification_input.json'
        settings = json.loads(settingspath.read_text())
        assert settings['prior']['joint_type'] == seal['selected']['joint_type']
        record.update(status='SEALED_COMPLETE_VALIDATION', model_sha256=seal['model_sha256'],
                      seal_sha256=sha(sealpath), compensation_settings_sha256=sha(settingspath),
                      immutable_packet=modelpath.parent.parent.name,
                      physical_duration_s=segment['result'].get('elapsed_sim_s'),
                      requested_target_reached=segment['result'].get('requested_target_reached'),
                      coordinate_unit=segment['coordinate_unit'])
        good = (np.isfinite(a['joint_q'][ix]) & np.isfinite(a['joint_dq'][ix])
                & np.isfinite(a['raw_ft'][ix]).all(1) & np.isfinite(a['sensor_pose'][ix]).all(1)
                & np.isfinite(a['tool_poses'][ix]).all((1,2)) & (a['ft_valid'][ix]>0))
        ix = ix[good]
        cuts = np.flatnonzero((np.diff(a['step'][ix])!=1) | (abs(np.diff(a['t'][ix])-1/240)>1e-7))+1
        for block, run in enumerate(np.split(ix, cuts)):
            if not len(run):
                continue
            label = segment['role']+'_%02d'%sid
            rows = rows_at(a, run, index['metadata'], label)
            piece = process(rows, settings['prior'], settings['bias'], a['joint_dq'][run],
                            native_coordinate=a['joint_q'][run])
            if piece is None:
                record['blocks'].append(dict(block=block, status='insufficient filter support', records=len(run)))
                continue
            piece['block_id'] = label+'#%d'%block
            m = seal['selected']
            samples(piece, m['domain_native'], [m['intervals']] if m['intervals'] else [])
            result = prediction(piece, m)
            if result.get('status') == 'predicted':
                factor = 180/np.pi if segment['coordinate_unit']=='rad' else 1000
                result['display_coordinate_rmse'] = result['coordinate_rmse']*factor
                result['display_final_coordinate_error'] = result['final_coordinate_error']*factor
                result['display_unit'] = 'deg' if segment['coordinate_unit']=='rad' else 'mm'
            record['blocks'].append(dict(block=block, records=len(run), measured_duration_s=float(piece['t'][-1]-piece['t'][0]),
                                         processing=piece['processing'], prediction=result))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(out, indent=2, allow_nan=False)+'\n')
    return out


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path)
    v=p.parse_args();print(json.dumps(audit(v.episode,v.output),indent=2))
