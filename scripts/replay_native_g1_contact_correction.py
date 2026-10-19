"""Recorded-input closure comparison only; never launches Isaac or predicts physics."""
from pathlib import Path
import argparse,gzip,pickle,json,importlib.util
import numpy as np

def load_class(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m.JawCenteredClosure

def records(folder):
    out=[]
    for f in sorted(folder.glob('*.gz')):
        with gzip.open(f,'rb') as stream:out.extend(pickle.load(stream))
    return out

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--archive-root',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--old-closure',type=Path,required=True);parser.add_argument('--new-closure',type=Path,required=True);parser.add_argument('--cases',nargs='+',required=True);args=parser.parse_args()
    Old=load_class(args.old_closure,'recorded_old');New=load_class(args.new_closure,'recorded_new');pads=json.loads((args.archive_root/'TEN_CASE_CONTACT_DIAGNOSTICS.json').read_text())['pad_bounds_logical_frame_m'];results=[]
    for asset in args.cases:
        p=args.archive_root/asset/'physical_01';rows=records(p/'observations_chunks');commands=records(p/'closure_commands_chunks');policy=json.loads((p/'sensor_metadata.json').read_text())['original_control_policy'];origin=np.asarray(rows[commands[0]['physics_step_index']-1]['T_tcp']);old=Old(origin,dict(policy));new=New(origin,dict(policy));reproduction=0.;same_targets=True;nonreopening=True;previous=None;held=advanced=0;phase_counts={}
        for c in commands:
            x=rows[c['physics_step_index']-1];P=x['finger_world_poses'];centers=[(np.asarray(P[n])@np.r_[np.asarray(pads[n]).mean(0),1])[:3] for n in ['gripper_link1','gripper_link2']];inputs=(np.asarray(x['T_tcp']),c['measured_aperture_m'],c['filtered_pad_loads_n'],centers,1/240);a=old.update(*inputs);b=new.update(*inputs)
            if a['mode']=='position':reproduction=max(reproduction,abs(a['opening_m']-c['aperture_command_m']))
            same_targets &= bool(np.array_equal(a['T'],b['T']))
            if b['mode']=='position':
                if previous is not None:nonreopening &= b['opening_m']<=previous+1e-14
                previous=b['opening_m']
            if b['state']=='CENTER':
                phase_counts[new.center_phase]=phase_counts.get(new.center_phase,0)+1
                advanced+=int(new.detail['closing_advanced']);held+=int(not new.detail['closing_advanced'])
        responses=new.response_records;r={'asset_id':asset,'updates':len(commands),'old_aperture_reproduction_error_m':reproduction,'new_nonreopening':bool(nonreopening),'same_centering_targets_with_fixed_recorded_inputs':bool(same_targets),'CENTER_aperture_held_updates':held,'CENTER_aperture_advanced_updates':advanced,'response_windows':len(responses),'unloading_with_expected_measured_translation_windows':sum(int(e['unloading_observed'] and e['actual_TCP_toward_loaded_m']>0) for e in responses),'last_center_phase':new.center_phase,'last_position_command_m':previous,'maximum_command_displacement_m':new.detail['command_displacement_m'],'note':'Counterfactual command comparison using OLD measured poses/loads; not physical validation.'};results.append(r);print(json.dumps(r),flush=True)
    args.output.write_text(json.dumps({'classification':'RECORDED_INPUT_REPLAY_ONLY','cases':results},indent=2)+'\n')
