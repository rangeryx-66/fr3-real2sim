"""Reuse the pinned inference adapter with explicitly exported native G1 geometry.

Only the geometry interface is replaced, before inference. The Dex1 constructor
and its calibration are never executed; the default legacy adapter stays intact.
"""
import argparse,hashlib,importlib.util,json,sys
from pathlib import Path
from unittest.mock import patch
import numpy as np


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--native-geometry',required=True,type=Path)
    args,remaining=parser.parse_known_args()
    # Resource cancellation is checked in each new worker, before importing
    # the CUDA/model adapter. The live physics owner is never hot-reloaded.
    directory=args.native_geometry.parent
    if (directory.parent.name=='hybrid_grasp'
            and (directory.parent.parent/'GRASPGENX_DISABLED_BY_USER.json').exists()):
        raise RuntimeError('GRASPGENX_DISABLED_BY_USER_RESOURCE_POLICY')
    payload=args.native_geometry.read_bytes();geometry=json.loads(payload)
    if geometry.get('schema')!='xarm-native-g1-graspgenx-geometry-v1':
        raise ValueError('EXPLICIT_NATIVE_G1_GEOMETRY_REQUIRED')
    if geometry['provenance'].get('dex1_or_r1_calibration_used')is not False:
        raise ValueError('FOREIGN_GRIPPER_CALIBRATION_FORBIDDEN')
    transform=np.asarray(geometry['T_GX_TCP'],float)
    if transform.shape!=(4,4)or not np.isfinite(transform).all():raise ValueError('INVALID_TCP_TRANSFORM')
    output=Path(remaining[remaining.index('--output')+1])
    if output.exists():raise ValueError('PRESERVE_EXISTING_INFERENCE_RESULT')
    class NativeGeometry:
        T_GX_TCP=transform
        def sweep_params(self):return geometry['sweep_params']
        def provenance(self):return geometry['provenance']
    path=Path(__file__).resolve().parents[1]/'src/infer_graspgenx.py'
    spec=importlib.util.spec_from_file_location('v87_pinned_grasp_inference',path)
    adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
    with patch.object(adapter,'Dex1Geometry',NativeGeometry),patch.object(sys,'argv',[str(path),*remaining]):
        adapter.main()
    result=json.loads(output.read_text())
    result['native_g1_sweep_params']=result.pop('dex1_sweep_params')
    result['native_g1_provenance']=result.pop('dex1_provenance')
    result.update(native_geometry_sha256=hashlib.sha256(payload).hexdigest(),
        gripper='XARM7_NATIVE_G1',inference_adapter_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        collision_and_IK_validation_pending=True)
    for candidate in result['grasps']:
        if not np.array_equal(candidate['T_G_TCP'],transform):raise ValueError('FOREIGN_TCP_MAPPING')
    temp=output.with_suffix('.native.tmp');temp.write_text(json.dumps(result,indent=2));temp.replace(output)


if __name__=='__main__':main()
