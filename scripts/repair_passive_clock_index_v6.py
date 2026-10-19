"""Metadata-only repair/regression from recorded clocks. No simulator or GT inputs."""
import json,sys,hashlib,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.passive_clock_v6 import check_pair
from agent_command_protocol_v12 import atomic_json

def repair(dataset):
    original=dataset/'acquisition_index.json'
    if not original.exists():shutil.copy2(dataset/'index.json',original)
    saved=json.loads(original.read_text());rows=[];origin=None;previous=None
    for row in saved['pairs']:
        c=check_pair(row['cameras'],row['physics_tick'],row['simulation_timestamp_s'],origin,previous)
        if origin is None:origin=c['reference_physics_origin_s']
        previous=c['reference_timestamp_s'];rows.append(dict(row,clock_diagnostics=c,synchronized=c['synchronized'],valid=bool(c['synchronized'] and all(v['valid'] for v in row['cameras']))))
    assert all(v['valid'] for v in rows),'Do not force a mismatched/stale pair to pass'
    test=rows[-1];a=[dict(c) for c in test['cameras']];a[1]=dict(a[1],reference_time=dict(a[1]['reference_time'],referenceTimeNumerator=a[1]['reference_time']['referenceTimeNumerator']+1));assert not check_pair(a,test['physics_tick'],test['simulation_timestamp_s'],origin)['synchronized']
    assert not check_pair(test['cameras'],test['physics_tick']+1,test['simulation_timestamp_s'],origin)['synchronized']
    assert not check_pair(test['cameras'],test['physics_tick'],test['simulation_timestamp_s'],origin,test['render_reference_timestamp_s'])['synchronized']
    result=dict(original_index_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),original_valid_pairs=saved['valid_synchronized_pairs'],recalibrated_valid_pairs=len(rows),maximum_world_clock_difference_s=max(abs(r['clock_diagnostics']['world_clock_difference_s']) for r in rows),maximum_nominal_tick_residual_s=max(abs(r['clock_diagnostics']['reference_minus_nominal_tick_s']) for r in rows),negative_tests=['mismatched paired reference','wrong physics tick','stale reference'],no_image_bytes_changed=True,no_object_state_or_hidden_dynamics_read=True,source_sha256=hashlib.sha256((ROOT/'interaction_runtime/passive_clock_v6.py').read_bytes()).hexdigest())
    atomic_json(dataset/'index.json',dict(saved,pairs=rows,valid_synchronized_pairs=len(rows),clock_metadata_repaired_offline=True,acquisition_index='acquisition_index.json'))
    atomic_json(dataset/'clock_calibration.json',dict(reference='renderer rational ReferenceTime, canonical dataset clock',physics_hz=240,reference_physics_origin_s=origin,world_clock='float32 dt accumulation; retained, not an inter-camera sync gate'))
    atomic_json(dataset/'CLOCK_DIAGNOSTICS.json',result);print(json.dumps(result,indent=2));return result
if __name__=='__main__':repair(Path(sys.argv[1]))
