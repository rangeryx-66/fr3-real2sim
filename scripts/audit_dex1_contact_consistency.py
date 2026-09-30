"""Replay real closure through the same full-size hulls used by Isaac/MoveIt."""
import argparse
import json
import os
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from grasp_compare.scene import load_scene
from grasp_compare.collision import Dex1SceneCollision

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',required=True,type=Path)
    p.add_argument('--search',required=True,type=Path)
    p.add_argument('--episodes',type=Path,nargs='*',default=[])
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--skip-nominal',action='store_true',help='Reuse the completed nominal search for measured-closure replay')
    args=p.parse_args();os.environ['DEX1_COLLISION_PROFILE']='isaac_convex_hull'
    checker=Dex1SceneCollision(load_scene(args.scene));search=json.loads(args.search.read_text())
    output={'profile':'isaac_convex_hull','scene_sha256':checker.scene.sha256,'nominal_search':[],'actual_episodes':[]}
    if args.skip_nominal:
        output=json.loads(args.output.read_text());output['actual_episodes']=[]
    for item in ([] if args.skip_nominal else search['decisions']):
        if item['status']=='OUTSIDE_SEARCH_BOUNDS':continue
        T=np.array(item['T_B_TCP']);approach=checker.check(T)
        closure=checker.check_closure(T) if approach['status']=='FREE' else None
        output['nominal_search'].append({'variant':item['variant'],'approach':approach,'estimated_closure_diagnostic':closure,
                                        'actual_closure_verified':False})
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(output,indent=2));print(item['variant'],approach['status'],flush=True)
    for directory in args.episodes:
        report=json.loads((directory/'report.json').read_text())
        load=report.get('isolation',{}).get('isolated_grasp',{})
        if 'actual_closure_samples' not in load:continue
        # The frozen input must be aligned to the live locked door, identically
        # to the saved-grasp transform applied before MoveIt planning.
        # For this fixed scene, record drift and reject an unaligned replay.
        alignment=report['hinge_frame_alignment']
        if abs(alignment['source_joint_q_rad']-alignment['live_joint_q_rad'])>1e-3:
            raise ValueError('frozen scene requires hinge transform before closure replay')
        episode={'directory':str(directory),'variant':report['local_search']['rows'][-1]['variant'],
                 'estimated_aperture_m':report['local_search']['rows'][-1]['estimated_contact_width_m'],
                 'actual_aperture_m':load['actual_aperture_m'],
                 'actual_official_pad_gap_m':checker.geometry.measured_pad_aperture(load['actual_closure_samples'][-1]['finger_q']),
                 'moveit_replay':load['actual_closure_moveit_replay'],
                 'pointcloud_replay':checker.check_actual_closure(load['actual_closure_samples']),
                 'actual_pad_forces_n':load['close_forces_n'],'actual_body_forces_n':load['close_finger_body_contact_n'],
                 'physical_status':load['status']}
        episode['aperture_error_m']=episode['actual_aperture_m']-episode['estimated_aperture_m']
        output['actual_episodes'].append(episode)
        args.output.write_text(json.dumps(output,indent=2))
    from collections import Counter
    output['approach_counts']=dict(Counter(r['approach']['status'] for r in output['nominal_search']))
    output['estimated_closure_counts']=dict(Counter(r['estimated_closure_diagnostic']['status'] for r in output['nominal_search'] if r['estimated_closure_diagnostic']))
    output['pads_only_actual_count']=sum(all(f>.2 for f in r['actual_pad_forces_n']) and not any(f>1e-6 for f in r['actual_body_forces_n']) and all(s['valid'] for s in r['moveit_replay']) and r['pointcloud_replay']['status']=='FREE' for r in output['actual_episodes'])
    args.output.write_text(json.dumps(output,indent=2));print(output['approach_counts'],output['estimated_closure_counts'],flush=True)
if __name__=='__main__':main()
