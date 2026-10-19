"""Compatibility entry for native collider-ownership validation (no projection)."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import trimesh,fcl
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import fcl_geometry,obj


def mesh_from(entry,cooked=True):
    pieces=[]
    if cooked:
        if not entry.get('convexes') or entry.get('export_error'):raise RuntimeError('missing native cooking export: '+entry['path'])
        for c in entry['convexes']:
            faces=[[p[0],p[k],p[k+1]] for p in c['polygons'] for k in range(1,len(p)-1)]
            pieces.append(trimesh.Trimesh(vertices=c['vertices'],faces=faces,process=False))
    else:
        faces=[];start=0
        for count in entry['raw_counts']:
            p=entry['raw_indices'][start:start+count];start+=count
            faces.extend([[p[0],p[k],p[k+1]] for k in range(1,count-1)])
        pieces=[trimesh.Trimesh(vertices=entry['raw_points'],faces=faces,process=True)]
    return pieces


def main():
    # Retire distance/projection-based ownership. Retain mesh_from as a native
    # cooked-shape reader for historical audit consumers.
    from validate_piper_owned_contact import main as owned_main
    owned_main()

if __name__=='__main__':main()
