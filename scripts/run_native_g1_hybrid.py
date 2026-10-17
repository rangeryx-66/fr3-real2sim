"""Pinned sensor owner, explicit hybrid operation substitution, frozen files intact."""
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'scripts/run_native_g1_ft_v1.py'
source=path.read_text()
old='from native_g1_ft_multistate import run as run_contact_recovery'
if source.count(old)!=1:raise ValueError('Pinned V1 owner boundary changed before initialization')
source=source.replace(old,'from native_g1_hybrid_contact import run as run_contact_recovery')
source=source.replace("'scripts/run_native_g1_ft_v1.py']", "'scripts/run_native_g1_ft_v1.py','scripts/native_g1_hybrid_contact.py','scripts/run_native_g1_hybrid.py','scripts/hybrid_panel_preflight.py','scripts/hybrid_routes_preflight.py']")
source=source.replace("out/'ACTUAL_FT_ID_V1_EXECUTABLE.py'","out/'ACTUAL_HYBRID_EXECUTABLE.py'")
source=source.replace('WRIST_FT_MULTISTATE_V1 /','HYBRID_PULL_PUSH_REGRASP_V0 /')
exec(compile(source,str(path)+'::explicit_hybrid_owner','exec'),globals())
