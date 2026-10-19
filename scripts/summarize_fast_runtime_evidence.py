"""Stream private 240 Hz evidence into compact, auditable engineering metrics."""
import argparse,gzip,hashlib,json,pickle
from collections import defaultdict
from pathlib import Path
import numpy as np

def summarize(folder):
    index=json.loads((folder/'telemetry/index.json').read_text());phases=defaultdict(lambda:dict(records=0,max_tcp_mm_s=0.,max_pad_n=0.,max_compensated_wrist_n=0.,minimum_joint_margin_rad=float('inf'),max_tracking_rad=0.,positive_native_contact_records=0))
    count=0;previous=None;max_step_error=0.;first=last=None;verified=True;previous_row=None
    for chunk in index['chunks']:
        path=folder/'telemetry'/chunk['file'];verified &= hashlib.sha256(path.read_bytes()).hexdigest()==chunk['sha256']
        assert chunk['first_record']==count
        with gzip.open(path,'rb') as stream:rows=pickle.load(stream)
        assert len(rows)==chunk['records']
        for row in rows:
            if first is None:first=row
            last=row;count+=1
            if previous is not None:max_step_error=max(max_step_error,abs(row['t']-previous-1/240))
            previous=row['t'];p=phases[row['phase']];p['records']+=1
            p.setdefault('first_angle_deg',row['door_angle_deg']);p['last_angle_deg']=row['door_angle_deg']
            p['max_tcp_mm_s']=max(p['max_tcp_mm_s'],row['tcp_speed_m_s']*1000)
            p['max_pad_n']=max(p['max_pad_n'],max(row['forces_n'].values(),default=0.))
            p['max_compensated_wrist_n']=max(p['max_compensated_wrist_n'],float(np.linalg.norm(row['wrist_ft']['gravity_compensated_wrench_world'][:3])))
            p['minimum_joint_margin_rad']=min(p['minimum_joint_margin_rad'],row['margin_rad'])
            p['max_tracking_rad']=max(p['max_tracking_rad'],float(np.max(abs(np.asarray(row['q'])[:7]-row['command_q_arm']))))
            if previous_row is not None:
                acceleration=float(np.max(abs(np.asarray(row['command_q_velocity_arm'])-previous_row['command_q_velocity_arm']))*240)
                p['max_command_joint_acceleration_rad_s2']=max(p.get('max_command_joint_acceleration_rad_s2',0.),acceleration)
                p['max_measured_TCP_acceleration_mm_s2']=max(p.get('max_measured_TCP_acceleration_mm_s2',0.),abs(row['tcp_speed_m_s']-previous_row['tcp_speed_m_s'])*240000)
            p['positive_native_contact_records']+=any(c['force_n']>0 for c in row['contacts'])
            p['max_relative_translation_mm']=max(p.get('max_relative_translation_mm',0.),row['relative_translation_slip_m']*1000)
            previous_row=row
    assert count==index['records']
    for p in phases.values():p['simulation_s']=p['records']/240
    result=dict(records=count,simulation_s=count/240,chunk_hashes_verified=bool(verified),max_timestamp_step_error_s=max_step_error,export_issues=index['issues'],phases=dict(phases),final=dict(angle_deg=last['door_angle_deg'],pad_loads_n=last['forces_n'],joint_state=last['q'].tolist()),private_raw_evidence=True)
    for name in ['TEST_RESULT.json','PROCESS_COMPLETION.json','EXECUTABLE_PROVENANCE.json']:
        path=folder/name
        if path.exists():result[name.removesuffix('.json')]=json.loads(path.read_text())
    path=folder/'COMPACT_EVIDENCE_SUMMARY.json';path.write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);args=parser.parse_args();s=summarize(args.folder)
    print(json.dumps(dict(records=s['records'],simulation_s=s['simulation_s'],hashes_verified=s['chunk_hashes_verified'],export_issues=s['export_issues'],final=s['final']),indent=2))
