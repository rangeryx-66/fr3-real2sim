"""Single reproducible command for calibration, native tests and final tables."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='configs/interactive_twin_robot_response.yaml')
    args = p.parse_args()
    subprocess.run([sys.executable, str(ROOT/'scripts/run_interactive_twin_response_calibration.py'),
                    '--config', args.config], cwd=ROOT, check=True)
    config = json.loads((ROOT/args.config).read_text()); out = ROOT/config['output']
    if (out/'final_results.json').exists():
        subprocess.run([sys.executable, str(ROOT/'scripts/report_interactive_twin_response_calibration.py'),
                        '--output', str(out)], cwd=ROOT, check=True)


if __name__ == '__main__': main()
