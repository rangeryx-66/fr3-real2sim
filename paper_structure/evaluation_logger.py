"""Write-only simulator truth logger. No value is returned to the controller."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

class EvaluationLogger:
    def __init__(self, scene, output):
        # Capture before GroundTruthGate seals controller-facing handles.
        self._link=scene['door_link'];self._rows=[];self._output=Path(output)
    def append(self, t, phase, measured_ee):
        p,q=self._link.get_world_pose();T=np.eye(4)
        T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();T[:3,3]=p
        self._rows.append({'t':t,'phase':phase,'T_object':T.tolist(),'T_ee':np.asarray(measured_ee).tolist()})
        # Explicitly no pose, joint state, or error return channel.
    def close(self):
        self._output.mkdir(exist_ok=True)
        (self._output/'object_trajectory.json').write_text(json.dumps(self._rows))
        (self._output/'scope.json').write_text(json.dumps({'evaluation_only':True,'controller_readback':False,'GT_inputs_to_fitter':False,'sample_every_physics_steps':8}))
