"""Isolated V1.2 owner; no frozen controller source is edited."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
path = ROOT / 'scripts/run_native_g1_large_range_torque.py'
source = path.read_text()
old = 'from native_g1_large_range_contact import run as run_contact_recovery'
if source.count(old) != 1: raise ValueError('Pinned owner import changed')
source = source.replace(old, 'from native_g1_pull_push_v12 import run as run_contact_recovery')
exec(compile(source, str(path) + '::pull_push_v12', 'exec'), globals())
