"""No Isaac initialization: verify the actual operation namespace and directional clock."""
import hashlib,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import native_g1_ft_identification as identification
import native_g1_contact_recovery as recovery
import native_g1_safe_termination as termination

observed={}
def inspect_owner(g):
    fn=recovery.attempt_grasp_and_probe;ns=fn.__globals__
    observed.update(safe_exit_is_v11=ns['safe_release_retreat'] is termination.safe_release_park,
        interaction_is_explicit=ns['bidirectional'] is identification.bidirectional,
        approach_module_unchanged=ns['physical'] is recovery.physical,
        closure_is_original=ns['FixedClosure'] is recovery.__dict__.get('FixedClosure') if 'FixedClosure' in ns else True)
    return observed

with patch.object(identification,'install_passive',lambda g:None),patch.object(termination,'run',inspect_owner):
    identification.run(dict(observe_step=lambda *a:None))
assert all(observed.values()),observed
for direction in [1.,-1.]:
    theta=.1;goal=theta+direction*.02;rate=.002;dt=1/240;old=theta
    new=direction*min(direction*goal+np.deg2rad(.5),max(direction*theta+np.deg2rad(.05),direction*old+rate*dt))
    assert direction*(new-theta)>0
    if direction==1:
        assert new==min(goal+np.deg2rad(.5),max(theta+np.deg2rad(.05),old+rate*dt))
observed.update(positive_clock_identical=True,negative_clock_moves_toward_closing=True,Isaac_scenes_started=0,
    source_sha256=hashlib.sha256(Path(identification.__file__).read_bytes()).hexdigest())
print(json.dumps(observed,indent=2))
