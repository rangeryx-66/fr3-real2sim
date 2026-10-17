"""Passive packet completion/ordered estimation; never waits in loaded physics.

RAW_READY is published after the frozen producer has closed all raw files.
MODEL_READY is published only after the numeric worker and all dependencies
finish. Failed/deferred estimates remain explicit diagnostics, not robot gates.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
from agent_command_protocol_v12 import atomic_json


def hashes(folder, names):
    return {n: hashlib.sha256((Path(folder) / n).read_bytes()).hexdigest() for n in names}


def publish_raw(folder):
    folder = Path(folder)
    atomic_json(folder / 'RAW_READY.json', dict(status='READY',
        files=hashes(folder, ('INPUT.json', 'measurements.pkl'))))


def verified_marker(folder, name):
    folder = Path(folder); path = folder / name
    if not path.exists(): return None
    record = json.loads(path.read_text())
    for n, digest in record.get('files', {}).items():
        if not (folder / n).exists() or hashlib.sha256((folder / n).read_bytes()).hexdigest() != digest:
            raise ValueError('Packet changed after completion: ' + str(folder / n))
    return record


def wait_ready(folder, name, timeout_s=120):
    began = time.monotonic()
    while time.monotonic() - began < timeout_s:
        record = verified_marker(folder, name)
        if record is not None: return record
        time.sleep(.05)
    raise TimeoutError('Passive packet dependency unavailable: ' + str(Path(folder) / name))


def process(folder, worker=None, timeout_s=120):
    folder = Path(folder)
    with (folder / '.model.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if verified_marker(folder, 'MODEL_READY.json') is not None: return
        began = time.monotonic()
        try:
            wait_ready(folder, 'RAW_READY.json', timeout_s)
            settings = json.loads((folder / 'INPUT.json').read_text())
            dependencies = []
            for previous in settings.get('previous_packets', []):
                status = wait_ready(previous, 'MODEL_READY.json', timeout_s)
                if status['status'] not in ('READY', 'NO_LOCAL_DATA'):
                    raise ValueError('Dependency did not complete: ' + str(previous))
                dependencies.append(dict(folder=previous, status=status['status']))
            if worker is None:
                from run_cabinet_torque_v11 import main as worker
            worker(folder)
            names = ['TORQUE_COMPONENTS.json']
            has_data = (folder / 'TORQUE_BLOCK.npz').exists()
            if has_data: names += ['TORQUE_BLOCK.npz', 'TORQUE_SAMPLES.json', 'TORQUE_CUMULATIVE.npz']
            atomic_json(folder / 'MODEL_READY.json', dict(status='READY' if has_data else 'NO_LOCAL_DATA',
                dependencies=dependencies, files=hashes(folder, names), wall_s=time.monotonic() - began))
        except Exception as error:
            atomic_json(folder / 'MODEL_ERROR.json', dict(status='DEFERRED_OR_FAILED', reason=repr(error),
                raw_preserved=True, used_in_execution=False, wall_s=time.monotonic() - began))
            raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('folder', type=Path)
    process(parser.parse_args().folder)
