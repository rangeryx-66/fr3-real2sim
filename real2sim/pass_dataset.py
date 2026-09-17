"""Extract one pass from a prepared dataset and renumber it for independent tracking."""
from __future__ import annotations
import argparse,json,shutil
from pathlib import Path


def extract(source:Path,output:Path,pass_id:int):
    if output.exists():shutil.rmtree(output)
    for d in ('rgb','depth','masks','gripper_masks','masks_hand','poses'):(output/d).mkdir(parents=True,exist_ok=True)
    shutil.copy2(source/'cam_K.txt',output/'cam_K.txt');rows=[]
    for pose in sorted((source/'poses').glob('*.json')):
        row=json.loads(pose.read_text())
        if int(row['pass_id'])!=pass_id:continue
        stem=f'{len(rows):06d}'
        for d in ('rgb','depth','masks','gripper_masks','masks_hand'):
            src=source/d/f'{pose.stem}.png';shutil.copy2(src,output/d/f'{stem}.png')
        row['combined_prepared_stem']=pose.stem;row['prepared_stem']=stem
        (output/'poses'/f'{stem}.json').write_text(json.dumps(row,indent=2));rows.append(row)
    if len(rows)<8:raise RuntimeError(f'pass {pass_id} only has {len(rows)} frames')
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('pass_id',type=int);a=p.parse_args();print(len(extract(a.source,a.output,a.pass_id)))
