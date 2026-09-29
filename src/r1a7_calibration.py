"""R1+Dex1 assembly and simulation calibration, separate from Unitree source URDFs."""
import json
from functools import lru_cache
from pathlib import Path

CALIBRATION_PATH = Path(__file__).resolve().parents[1] / 'config/end_effector_calibration.yaml'


@lru_cache(maxsize=1)
def load_calibration():
    data = json.loads(CALIBRATION_PATH.read_text())  # JSON is valid YAML 1.2.
    for key in ('link7_to_dex1_base', 'dex1_base_to_tcp'):
        item = data[key]
        if len(item['xyz_m']) != 3 or len(item['rpy_rad']) != 3:
            raise ValueError(f'{key} must contain 3D xyz/rpy')
    return data


def width_to_finger_q(width_m, model_path=None):
    """Map simulated pad gap to each symmetric URDF prismatic joint."""
    import xml.etree.ElementTree as ET
    path = model_path or Path(__file__).resolve().parents[1] / 'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1/dex1_1.urdf'
    root = ET.parse(path).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    a, b = joints['Joint1_1'], joints['Joint2_1']
    if a.find('axis').get('xyz') != '-1 0 0' or b.find('axis').get('xyz') != '1 0 0':
        raise ValueError('Dex1 finger axes changed; rederive width convention')
    origin_a = float(a.find('origin').get('xyz').split()[0])
    origin_b = float(b.find('origin').get('xyz').split()[0])
    fixed_a = sum(float(joints[n].find('origin').get('xyz').split()[0]) for n in ('Joint1_2', 'Joint1_3'))
    fixed_b = sum(float(joints[n].find('origin').get('xyz').split()[0]) for n in ('Joint2_2', 'Joint2_3'))
    zero_width = (origin_a + fixed_a) - (origin_b + fixed_b)
    q = (zero_width - width_m) / 2
    lo, hi = (float(a.find('limit').get(k)) for k in ('lower', 'upper'))
    if not lo - 1e-9 <= q <= hi + 1e-9:
        raise ValueError(f'opening {width_m} outside official URDF finger limits')
    return min(hi, max(lo, q))
