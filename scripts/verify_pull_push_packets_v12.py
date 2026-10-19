"""Replay two archived raw packets with reversed worker start order, no physics."""
import argparse
import concurrent.futures
import json
from pathlib import Path
import shutil
import time
from identification_packet_v12 import publish_raw, process, verified_marker


def verify(archive, output):
    output.mkdir(parents=True, exist_ok=True)
    folders = [output / ('%02d' % i) for i in range(2)]
    if any((p/'RAW_READY.json').exists() for p in folders):
        raise ValueError('Replay already exists; preserve evidence rather than overwrite')
    for i, folder in enumerate(folders):
        folder.mkdir(exist_ok=True)
        source = archive / ('%02d' % i)
        shutil.copyfile(source/'measurements.pkl', folder/'measurements.pkl')
        settings = json.loads((source/'INPUT.json').read_text())
        settings['previous_packets'] = [str(p.resolve()) for p in folders[:i]]
        (folder/'INPUT.json').write_text(json.dumps(settings, indent=2))
    began = time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        # Later worker starts with neither raw producer marked ready.
        later = pool.submit(process, folders[1], None, 120)
        time.sleep(.15); publish_raw(folders[1])
        earlier = pool.submit(process, folders[0], None, 120)
        time.sleep(.15); publish_raw(folders[0])
        earlier.result(); later.result()
    result = dict(physics_actors=0, reversed_worker_start=True, output_packets=[])
    for folder in folders:
        ready = verified_marker(folder, 'MODEL_READY.json')
        model = json.loads((folder/'TORQUE_COMPONENTS.json').read_text())
        result['output_packets'].append(dict(folder=str(folder), ready=ready,
            training_bins=model.get('training_bins'),validation_bins=model.get('validation_bins'),
            parameters=model.get('constant_model', model.get('fit'))))
    assert result['output_packets'][1]['ready']['dependencies'][0]['status'] == 'READY'
    assert result['output_packets'][1]['training_bins'] >= result['output_packets'][0]['training_bins']
    result['wall_s'] = time.monotonic()-began
    (output/'REPLAY_RESULT.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=verify(a.archive,a.output);print(json.dumps(r,indent=2))
