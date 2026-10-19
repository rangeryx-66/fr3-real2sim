"""Recheck manipulation or base-reposition edges with actual finger positions."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model


def main():
    p=argparse.ArgumentParser();p.add_argument('request',type=Path);p.add_argument('response',type=Path);a=p.parse_args();r=json.loads(a.request.read_text());source=json.loads((Path(r['source'])/'report.json').read_text())
    model=Model(ROOT/'config/piper.urdf',r['asset_root'],source);model.target_tree=cKDTree(np.load(r['target_points']))
    previous=np.array(r['actual_q'][:6]);fingers=r['actual_q'][6:];angle=r['actual_door_angle_deg'];minimum=model.margin(previous);out={'status':'CLOSED_PATH_VALID','finger_positions':fingers,'initial_state_valid':False,'rows':[]}
    if r.get('kind')=='base_reposition':
        begin=np.asarray(r['initial_base']);end=np.asarray(r['selected']['base']);delta=end-begin;delta[3]=(delta[3]+180)%360-180
        steps=max(2,int(np.ceil(np.linalg.norm(delta[:2])/.005))+1,int(np.ceil(abs(delta[3])))+1)
        for fraction in np.linspace(0,1,steps):
            base=begin+fraction*delta;valid,reason,_=model.check(previous,base,finger_q=fingers)
            if not valid:
                out.update(status='BASE_ROUTE_'+reason,failed_fraction=float(fraction));a.response.write_text(json.dumps(out,indent=2));return
            out['rows'].append({'fraction':float(fraction),'base':base.tolist()})
        out['status']='BASE_ROUTE_VALID';a.response.write_text(json.dumps(out,indent=2));return
    for step in [{'q':previous.tolist(),'angle_deg':angle},*r['selected']['arc']]:
        q=np.array(step['q']);goal=step['angle_deg']
        for alpha in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(q-previous))/.005))+1)):
            actual=previous+alpha*(q-previous);qa=np.deg2rad(angle+alpha*(goal-angle));valid,reason,_=model.check(actual,r['selected']['base'],qa,allow_pad=True,finger_q=fingers)
            minimum=min(minimum,model.margin(actual))
            if not valid:
                out.update(status='CLOSED_PATH_'+reason,failed_angle_deg=float(np.rad2deg(qa)));a.response.write_text(json.dumps(out,indent=2));return
        out['initial_state_valid']=True
        out['rows'].append({'angle_deg':goal,'margin_rad':model.margin(q)});previous=q;angle=goal
    out['minimum_joint_margin_rad']=minimum;a.response.write_text(json.dumps(out,indent=2))

if __name__=='__main__':main()
