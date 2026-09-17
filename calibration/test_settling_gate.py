import json, gzip
from pathlib import Path
from settling_gate import compute_metrics, evaluate, GateThresholds

ROOT = Path(__file__).resolve().parents[1]

def trace(name):
    path = ROOT/f'results/hand_calibration/formal/{name}/{name}_force_F30_mu0.7_r0.trace.json.gz'
    rows = json.load(gzip.open(path))['records']
    return [x for x in rows if x['phase'] == 'MICRO_LIFT']

def main():
    soup = evaluate(trace('soup'))
    bowl = evaluate(trace('bowl'))
    assert soup['passed'], soup
    assert not bowl['passed'], bowl
    assert 0.003 < bowl['metrics']['max_cumulative_translation_m'] < 0.004
    # Its tail has settled, but the target followed only ~34% of the commanded
    # 5 mm TCP rise.  The new gate must not mistake table-supported settling for
    # a transportable grasp.
    assert bowl['category'] == 'CONTINUOUS_SLIP'
    strict = evaluate(trace('bowl'), GateThresholds(1e-7, 1e-7, 1e-7, .9999))
    assert not strict['passed']
    print('SETTLING_GATE_TEST_OK')

if __name__ == '__main__': main()
