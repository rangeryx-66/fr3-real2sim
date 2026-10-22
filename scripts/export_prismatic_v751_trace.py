"""Publish compact decisions and outcomes; raw observations remain private."""
import argparse,hashlib,json
from pathlib import Path


def export(episode,output):
    p=Path(episode);seal=json.loads((p/'ONLINE_SEAL_V751.json').read_text())
    assert seal['actor_exited'] and seal['online_actions_finished']
    trace_path=p/'AGENT_ACTION_TRACE.json'
    assert hashlib.sha256(trace_path.read_bytes()).hexdigest()==seal['sha256']['AGENT_ACTION_TRACE.json']
    rows=[]
    for e in json.loads(trace_path.read_text()):
        c=e['command'];r=e['result'];safe=r.get('safe_exit')or{}
        contacts=(r.get('candidate_contact_support')or{}).get('first_contacts',{})
        rows.append(dict(sequence=c.get('sequence'),operation=c['operation'],Agent_reason=c.get('reason'),
            operation_wall_s=e.get('wall_s'),unloaded_wait_s=e.get('unloaded_wait_s'),
            requested_preload_N=c.get('native_g1_preload_request_n'),transverse_unloading=c.get('transverse_unloading'),
            observation_hypothesis=c.get('observed_handle_hypothesis'),grasp_family=c.get('grasp_family'),
            candidate_family=r.get('candidate_family',r.get('family')),
            planned_segments=[{k:s.get(k)for k in ['role','target_q','reference_speed_m_s','reference_acceleration_m_s2','velocity_program']}for s in c.get('planned_segments',c.get('segments',[]))],
            predicted_first_contact_m={k:v.get('width_m')for k,v in contacts.items()},
            plan=r.get('plan'),valid=r.get('valid'),feasible=r.get('feasible'),reason=r.get('reason'),error=r.get('error'),
            physical_acquisition=r.get('acquisition'),physical_grasp_started=r.get('physical_grasp_started'),
            physical_segments=[{k:s.get(k)for k in ['status','reason','gain_native','joint_coordinate','elapsed_sim_s','physical_transmission_valid']}for s in r.get('segments',[])],
            safe_exit=safe.get('safe'),verified_park=safe.get('park',{}).get('reached')))
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(dict(episode=p.name,trace_sha256=seal['sha256']['AGENT_ACTION_TRACE.json'],raw_images_and_telemetry_private=True,decisions=rows),indent=2)+'\n')
    print(p.name,len(rows))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode');ap.add_argument('--output',required=True);a=ap.parse_args();export(a.episode,a.output)
