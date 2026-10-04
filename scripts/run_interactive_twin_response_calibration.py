"""Shared robot calibration -> bounded M2/M3 fit -> two frozen native tests."""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from interactive_twin_robot_response.calibration import read


def write(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='configs/interactive_twin_robot_response.yaml')
    a = p.parse_args(); c = read(ROOT/a.config); out = ROOT/c['output']; out.mkdir(parents=True, exist_ok=True)
    if not (out/'robot_calibration.json').exists():
        subprocess.run([sys.executable, str(ROOT/'scripts/run_interactive_twin_robot_response.py'), '--config', a.config], check=True)
    robot = read(out/'robot_calibration.json')
    if not (robot['numerical_adequacy_pass'] and robot['cross_asset_robot_validation']):
        print('STOP: shared robot response failed independent calibration/transfer'); return
    files = sorted((ROOT/'interactive_twin_response_selection').glob('*.py'))+[Path(__file__).resolve()]
    hashes = {str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    target = out/'physics_method_frozen.json'
    if target.exists() and read(target)['source_hashes'] != hashes: raise ValueError('PHYSICS_METHOD_CHANGED')
    if not target.exists(): write(target, {'source_hashes': hashes, 'saved_unix_s': time.time(), 'test_read': False,
                                          'robot_calibration_sha256': hashlib.sha256((out/'robot_calibration.json').read_bytes()).hexdigest()})
    from interactive_twin_response_selection.experiment import run
    result = run(ROOT, c, read, write)
    write(out/'final_results.json', result)
    print(json.dumps(result['decisions'], indent=2))


if __name__ == '__main__':
    main()
