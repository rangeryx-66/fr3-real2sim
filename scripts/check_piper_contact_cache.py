"""Replay real SDK contact payloads through old/new callbacks; check exact output."""
import argparse,importlib.util,json,sys,time
from pathlib import Path
from types import SimpleNamespace as N
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.contact_ownership import NativeOwnershipReports


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['before','trial','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();spec=importlib.util.spec_from_file_location('before_cache',a.before);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    exported=json.loads((a.trial/'cooked_initial.json').read_text());owners={e['path']:(e['finger'],e['owner']) for e in exported['shapes'] if e.get('finger')};target=json.loads((a.trial/'target_collider_association.json').read_text())['allowed_pad_targets'][0];body=next(e['rigid_body_path'] for e in exported['shapes'] if e['path']==target)
    states=json.loads((a.trial/'physics_steps.json').read_text());sample=next(s for s in states if s['ownership']['metal_contacts']);contacts=sample['ownership']['contacts'];ids={};paths={}
    def identifier(path):
        if path not in ids:ids[path]=len(ids)+1;paths[ids[path]]=path
        return ids[path]
    def reporter(cls):
        o=cls.__new__(cls);o.dt=1/240;o.targets={body};o.allowed_pad_targets={target};o.paths=owners.copy();o.rows=[];o.physics_steps=[];o.decode=lambda key:paths[key]
        o.finger_body_paths={path.rsplit('/contact_owned/',1)[0]:finger for path,(finger,owner) in o.paths.items()};o.decoded_paths={};o.contact_header_count=0
        return o
    headers=[];data=[]
    for i,c in enumerate(contacts):
        impulse=c['impulse_world_ns'];reverse=i%2
        headers.append(N(collider0=identifier(c['target'] if reverse else c['collider']),collider1=identifier(c['collider'] if reverse else c['target']),contact_data_offset=len(data),num_contact_data=1))
        data.append(N(impulse=N(**dict(zip(['x','y','z'],[-v for v in impulse] if reverse else impulse))),separation=c['separation_m'],face_index0=c['face0'],face_index1=c['face1']))
    finger_body=next(iter(reporter(NativeOwnershipReports).finger_body_paths));pad=next(path for path,(finger,owner) in owners.items() if owner=='pad')
    for collider,other in [(finger_body+'/unmapped_source_collider',target),(pad,body+'/non_target_door_panel')]:
        headers.append(N(collider0=identifier(collider),collider1=identifier(other),contact_data_offset=len(data),num_contact_data=1));data.append(N(impulse=N(x=.001,y=0.,z=0.),separation=0.,face_index0=0,face_index1=0))
    before=reporter(module.NativeOwnershipReports);after=reporter(NativeOwnershipReports)
    before.callback(headers,data);after.callback(headers,data)
    assert before.rows==after.rows,'callback outputs changed'
    assert before.summarize(before.rows,before.dt)==after.summarize(after.rows,after.dt),'forces/classification changed'
    assert any(r['owner']=='unknown' for r in after.rows),'unknown must remain forbidden'
    assert any(r['owner']=='pad' and not r['allowed_pad_target'] for r in after.rows),'wrong pad target guard lost'
    owned=next(iter(owners));unrelated='/World/Piper/Geometry/world/base_link/link4';ignored=[N(collider0=identifier(owned),collider1=identifier(unrelated),contact_data_offset=0,num_contact_data=0)]*2000;times=[]
    for o in [before,after]:
        begin=time.perf_counter();o.callback(ignored,[]);times.append(time.perf_counter()-begin)
    assert before.rows==after.rows
    result={'identical_contact_rows':True,'identical_force_summary':True,'unknown_forbidden':True,'wrong_pad_target_forbidden':True,'real_contact_payloads_replayed':len(contacts),'ignored_headers_benchmark':len(ignored),'before_seconds':times[0],'cached_seconds':times[1],'speedup':times[0]/max(times[1],1e-12),'note':'Callback replay/cache benchmark, not a new physical grasp success.'}
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
