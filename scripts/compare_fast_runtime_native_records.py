"""Compare identical native safety excitation before/after diagnostic-render changes.

This is an offline evidence comparison, never an execution admission check.
"""
import argparse,gzip,json,pickle
from pathlib import Path
import numpy as np

def load(folder):
    index=json.loads((folder/'telemetry/index.json').read_text());rows=[]
    for chunk in index['chunks']:
        with gzip.open(folder/'telemetry'/chunk['file'],'rb') as stream:rows.extend(pickle.load(stream))
    return rows,index

def compare(first,second):
    a,ia=load(first);b,ib=load(second)
    result=dict(first=str(first),second=str(second),records=[len(a),len(b)],native_hz=[ia['native_rate_hz'],ib['native_rate_hz']],equal_record_count=len(a)==len(b),export_issues=[ia['issues'],ib['issues']])
    if len(a)==len(b):
        metrics={}
        for name,get in [('timestamp_s',lambda r:r['t']),('joint_position_rad',lambda r:r['q']),('joint_velocity_rad_s',lambda r:r['joint_velocities']),('TCP_world_matrix',lambda r:r['T_tcp']),('raw_sensor_wrench',lambda r:r['wrist_ft']['raw_wrench_sensor']),('compensated_wrench_world',lambda r:r['wrist_ft']['gravity_compensated_wrench_world'])]:
            metrics[name]=max(float(np.max(abs(np.asarray(get(x))-get(y)))) for x,y in zip(a,b))
        result['maximum_absolute_difference']=metrics
        result['contact_records_equal']=all(x['contacts']==y['contacts'] for x,y in zip(a,b))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('first',type=Path);p.add_argument('second',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();r=compare(a.first,a.second);a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
