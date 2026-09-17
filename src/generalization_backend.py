"""Held-out harness: frozen v2 execution, original inference once per new scene."""
import argparse,json,hashlib,shutil
from pathlib import Path
import rclpy
from contact_backend import ContactBackend
from clutter_backend import ClutterBackend
class HeldOut(ContactBackend):
    def perception(self,seed):
        data,path=ClutterBackend.perception(self,seed)
        capture=Path(path).with_name(f'seed_{seed:04d}_capture.json')
        if capture.exists():
            cloud=Path(json.loads(capture.read_text())['path'])
            saved=Path(path).with_name(f'seed_{seed:04d}_cloud.npz')
            if not saved.exists():shutil.copy2(cloud,saved)
        return data,path
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--start',type=int,required=True);p.add_argument('--count',type=int,required=True);a=p.parse_args()
    rclpy.init();n=HeldOut(a.output,geometry_filter=True,min_pad_coverage=.045,lift_gain_scale=2.)
    for seed in range(a.start,a.start+a.count):
        if (n.output/f'B_seed_{seed:04d}.json').exists():continue
        r=n.episode(seed)
        if r['category']=='SYSTEM_ERROR':raise RuntimeError(r['detail'])
    n.destroy_node();rclpy.shutdown()
