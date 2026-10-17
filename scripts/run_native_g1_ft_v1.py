"""Versioned sensor/passive-plant V1 owner. All frozen source files unchanged."""
import ast,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'scripts/run_native_g1_ft_identification.py';source=path.read_text()
REFERENCE_BINDING="""
identification_speed_m_s=.0015
identification_acceleration_m_s2=.002
controller_source=controller_source.replace('rate=.0015*motion_scale/radius','rate=identification_speed_m_s/radius')
controller_source=controller_source.replace('acc=.002*motion_scale**2','acc=identification_acceleration_m_s2')
controller_source=controller_source.replace('min(.0015*motion_scale,np.sqrt','min(identification_speed_m_s,np.sqrt')
"""
source=source.replace('text=text.replace(anchor,anchor+addition)','text=text.replace(anchor,anchor+addition+REFERENCE_BINDING)')
# Keep the verified generated owner/loader/directional reference exactly;
# substitute only the versioned scene generator and interaction operation.
source=source.replace('from xarm_ft_id_scene import author as author_identification_scene','from xarm_ft_v1_scene import author as author_identification_scene')
source=source.replace('from native_g1_ft_identification import run as run_contact_recovery','from native_g1_ft_multistate import run as run_contact_recovery')
source=source.replace("WRIST_FT_IDENTIFICATION_V0 /","WRIST_FT_MULTISTATE_V1 /")
source=source.replace("seeded density/resistance V0","seeded passive resistance V1")
source=source.replace("out/'ACTUAL_FT_ID_EXECUTABLE.py'","out/'ACTUAL_FT_ID_V1_EXECUTABLE.py'")
# The reference wrapper's report/hash must include these explicit V1 inputs too.
source=source.replace("hashes={n:hashlib.sha256", "files+=['scripts/xarm_ft_v1_scene.py','scripts/xarm_ft_v1_estimator.py','scripts/native_g1_ft_multistate.py','scripts/run_xarm_ft_v1_estimation.py','scripts/run_native_g1_ft_v1.py']\nhashes={n:hashlib.sha256")
compile(source,__file__+'::pinned_V0_owner_with_V1_inputs','exec')
exec(compile(source,__file__+'::pinned_V0_owner_with_V1_inputs','exec'),globals())
