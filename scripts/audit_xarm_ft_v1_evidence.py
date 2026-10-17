"""Post-episode native-record/video/provenance audit, never an execution gate."""
import argparse,collections,gzip,hashlib,json,pickle,subprocess
from pathlib import Path
import numpy as np


def audit(case):
    case=Path(case);episode=case/'episode';export=json.loads((episode/'EXPORT_COMPLETE.json').read_text())
    counts=collections.Counter();phases={};previous=None;max_gap=0.;events=collections.Counter();clock=[]
    for part in sorted((episode/'observations_chunks').glob('*.gz')):
        with gzip.open(part,'rb') as stream:rows=pickle.load(stream)
        for row in rows:
            counts['records']+=1;ft=row.get('wrist_ft',{});tele=row.get('telemetry',{})
            counts['valid_native_FT']+=bool(ft.get('valid'));counts['raw_six_axis_FT']+=len(ft.get('raw_wrench_sensor',[]))==6
            counts['sensor_world_transform']+=np.asarray(ft.get('T_world_sensor',[])).shape==(4,4)
            counts['downstream_tool_states']+=bool(ft.get('body_states'));counts['native_contact_telemetry']+='raw_contact_records' in tele
            counts['native_G1_interface']+='native_g1_interface' in row
            counts['independent_motor_drive_force_available']+=bool(row.get('native_g1_interface',{}).get('independent_drive_force_available'))
            counts['identification_segment_records']+=bool(row.get('identification_segment'))
            if ft.get('timestamp_s') is not None:clock.append(float(ft['timestamp_s'])-row['t'])
            status=row.get('force_event',{}).get('status','UNAVAILABLE');events[status]+=1
            phase=row['phase'];entry=phases.setdefault(phase,dict(records=0,max_TCP_speed_m_s=0.,max_pad_normal_N=0.,max_raw_pad_normal_N=0.,minimum_margin_rad=float('inf')))
            raw_pad=tele.get('pad_load_raw_n',{});entry['max_raw_pad_normal_N']=max(entry['max_raw_pad_normal_N'],max(raw_pad.values(),default=0.))
            entry['records']+=1;entry['max_pad_normal_N']=max(entry['max_pad_normal_N'],max(row['forces_n'].values(),default=0.));entry['minimum_margin_rad']=min(entry['minimum_margin_rad'],row['margin_rad'])
            if previous is not None:
                dt=row['t']-previous['t'];max_gap=max(max_gap,abs(dt-1/240))
                if dt>0:entry['max_TCP_speed_m_s']=max(entry['max_TCP_speed_m_s'],float(np.linalg.norm(np.asarray(row['T_tcp'])[:3,3]-np.asarray(previous['T_tcp'])[:3,3])/dt))
            previous=row
    for entry in phases.values():entry['simulation_s']=entry['records']/240
    videos=[]
    for path in sorted(episode.glob('*.mp4')):
        command=['ffprobe','-v','error','-show_entries','format=duration,size:stream=codec_name,width,height,avg_frame_rate,nb_frames','-of','json',str(path)]
        r=subprocess.run(command,capture_output=True,text=True)
        videos.append(dict(file=path.name,probe=json.loads(r.stdout) if r.returncode==0 else None,error=r.stderr if r.returncode else None))
    expected=export['streams']['observations']['buffer_records'];frozen=json.loads((case.parent/'FROZEN.json').read_text());repo=Path(__file__).resolve().parents[1]
    source={name:dict(expected=h,actual=hashlib.sha256((repo/name).read_bytes()).hexdigest()) for name,h in frozen['source_sha256'].items()}
    active=json.loads((episode/'FT_ID_SOURCE_HASHES.json').read_text());generated=hashlib.sha256((episode/'ACTUAL_FT_ID_V1_EXECUTABLE.py').read_bytes()).hexdigest()
    result=dict(active_generated_executable_sha256=generated,generated_hash_matches=generated==active['executable_sha256'],job_sha256=hashlib.sha256((case/'job.json').read_bytes()).hexdigest(),native_hz=240,counts=dict(counts),expected_records=expected,record_count_matches=counts['records']==expected,
        maximum_native_cadence_error_s=max_gap,sensor_minus_row_timestamp_range_s=[min(clock),max(clock)] if clock else None,
        phases=phases,force_status_sample_counts=dict(events),videos=videos,
        frozen_shared_source_files=len(source),all_frozen_sources_match=all(v['actual']==v['expected'] for v in source.values()),source=source,
        measured_joint_effort_is_not_isolated_drive_force=True,contact_normals_not_complete_wrench=True,
        audit_only_no_execution_or_fitting_gates=True,hidden_dynamics_not_parsed=True)
    (case/'EVIDENCE_AUDIT.json').write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');a=p.parse_args();r=audit(a.case)
    print(json.dumps({k:v for k,v in r.items() if k not in ['source','phases']},indent=2))
