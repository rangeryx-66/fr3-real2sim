"""Pinned load-cell/scene owner; explicit V1.1 interaction adapter."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'scripts/run_native_g1_large_range_torque.py';source=path.read_text()
source=source.replace('from native_g1_large_range_contact import run as run_contact_recovery','from native_g1_cabinet_torque_contact import run as run_contact_recovery')
exec(compile(source,str(path)+'::cabinet_v11','exec'),globals())
