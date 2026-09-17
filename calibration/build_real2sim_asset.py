"""Offline, restartable Real2Sim reconstruction/ID/asset assembly."""
from __future__ import annotations
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from real2sim.bundlesdf_adapter import run as bundlesdf
from real2sim.payload_id import identify
from real2sim.frame_transform import transform
from real2sim.inertia_regularization import regularize
from real2sim.collision_mesh import decompose
from real2sim.asset_builder import build

def main():
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--official-root',type=Path,default=Path('/data1/home/rangeryx/scalable-real2sim'));p.add_argument('--force-reconstruct',action='store_true');a=p.parse_args()
    recon_dir=a.run/'reconstruction/bundlesdf';recon_json=recon_dir/'reconstruction.json'
    if recon_json.exists() and not a.force_reconstruct:
        recon=json.loads(recon_json.read_text())
    else:
        recon=bundlesdf(a.run/'scan',recon_dir,a.official_root)
    tcp=identify(a.run/'system_id_baseline.npz',a.run/'system_id_payload.npz',a.run/'inertial_tcp.json')
    object_inertia=transform(a.run/'inertial_tcp.json',a.run/'scan/scan_manifest.json',Path(recon['products']['tracking']),a.run/'inertial_object.json')
    visual_mesh=Path(recon['products']['textured_mesh'])
    simulation_inertia=regularize(a.run/'inertial_object.json',visual_mesh,a.run/'inertial_object_simulation.json')
    collision=decompose(visual_mesh,a.run/'collision')
    asset=build(a.name,visual_mesh,Path(recon['products']['texture']),a.run/'collision/collision_manifest.json',a.run/'inertial_object_simulation.json',a.run/'asset_final')
    summary={'reconstruction':recon,'inertial_tcp':tcp.__dict__,'inertial_object_raw':object_inertia,'inertial_object_simulation':simulation_inertia,'collision':collision,'asset':asset}
    (a.run/'asset_pipeline.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
