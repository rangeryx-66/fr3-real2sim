"""Isolated reference-handover version of the pinned sensor/Agent owner."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'scripts/run_native_g1_ft_v1.py';source=path.read_text()
old='from native_g1_ft_multistate import run as run_contact_recovery'
if source.count(old)!=1:raise ValueError('Pinned owner operation boundary changed')
source=source.replace(old,'from native_g1_large_range_contact import run as run_contact_recovery')
source=source.replace("'scripts/run_native_g1_ft_v1.py']", "'scripts/run_native_g1_ft_v1.py','scripts/native_g1_large_range_contact.py','scripts/run_native_g1_large_range_torque.py','scripts/interaction_reference_handover.py','scripts/torque_components_v1.py','scripts/run_torque_components_v1.py']")
source=source.replace("out/'ACTUAL_FT_ID_V1_EXECUTABLE.py'","out/'ACTUAL_LARGE_RANGE_TORQUE_EXECUTABLE.py'")
source=source.replace('WRIST_FT_MULTISTATE_V1 /','LARGE_RANGE_TORQUE_COMPONENTS_V1 /')
exec(compile(source,str(path)+'::isolated_handover_owner','exec'),globals())
