"""Reject BundleSDF pose outliers using stable-grasp kinematic consistency."""
from __future__ import annotations
import argparse,json,shutil
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation


def center(Ts,keep=None):
    use=Ts if keep is None else [T for T,k in zip(Ts,keep) if k]
    M=np.eye(4);M[:3,3]=np.median([x[:3,3] for x in use],axis=0);M[:3,:3]=Rotation.from_matrix([x[:3,:3] for x in use]).mean().as_matrix();return M


def robust_limit(values,floor):
    v=np.asarray(values);med=np.median(v);mad=np.median(np.abs(v-med))*1.4826
    return float(max(floor,med+3*mad))


def run(dataset:Path,tracking:Path,out_dataset:Path,out_bundle:Path):
    records=[];groups={}
    for p in sorted((dataset/'poses').glob('*.json')):
        row=json.loads(p.read_text());P=np.loadtxt(tracking/'ob_in_cam'/f'{p.stem}.txt');T=np.linalg.inv(np.asarray(row['T_B_TCP']))@np.asarray(row['T_B_camera'])@P
        item={'pose':p,'row':row,'P':P,'T':T};records.append(item);groups.setdefault(int(row['pass_id']),[]).append(item)
    report={'schema':'fr3_tracking_quality/v1','gt_used':False,'method':'robust constancy of T_TCP_BundleSDF_object within each stable grasp','passes':{}}
    for pid,items in groups.items():
        Ts=[x['T'] for x in items];M=center(Ts);dt=[np.linalg.norm((np.linalg.inv(M)@T)[:3,3])*1000 for T in Ts];dr=[np.degrees(Rotation.from_matrix((np.linalg.inv(M)@T)[:3,:3]).magnitude()) for T in Ts]
        lt=robust_limit(dt,3.);lr=robust_limit(dr,8.);keep=[a<=lt and b<=lr for a,b in zip(dt,dr)]
        # Recenter once after gross outliers are removed.
        M=center(Ts,keep);dt=[np.linalg.norm((np.linalg.inv(M)@T)[:3,3])*1000 for T in Ts];dr=[np.degrees(Rotation.from_matrix((np.linalg.inv(M)@T)[:3,:3]).magnitude()) for T in Ts]
        keep=[a<=lt and b<=lr for a,b in zip(dt,dr)]
        for x,a,b,k in zip(items,dt,dr,keep):x.update(accepted=bool(k),translation_residual_mm=float(a),rotation_residual_deg=float(b))
        report['passes'][str(pid)]={'translation_limit_mm':lt,'rotation_limit_deg':lr,'accepted':int(sum(keep)),'input':len(keep)}
    if out_dataset.exists():shutil.rmtree(out_dataset)
    if out_bundle.exists():shutil.rmtree(out_bundle)
    for d in ('rgb','depth','masks','gripper_masks','masks_hand','poses'):(out_dataset/d).mkdir(parents=True,exist_ok=True)
    for d in ('ob_in_cam','color_segmented'):(out_bundle/d).mkdir(parents=True,exist_ok=True)
    shutil.copy2(dataset/'cam_K.txt',out_dataset/'cam_K.txt');shutil.copy2(dataset/'cam_K.txt',out_bundle/'cam_K.txt');accepted=[]
    for item in records:
        audit={k:v for k,v in item.items() if k not in ('pose','row','P','T')};item['row']['tracking_quality']=audit
        if not item['accepted']:continue
        src=item['pose'].stem;dst=f'{len(accepted):06d}'
        (out_bundle/dst).mkdir(exist_ok=True)
        for d in ('rgb','depth','masks','gripper_masks','masks_hand'):shutil.copy2(dataset/d/f'{src}.png',out_dataset/d/f'{dst}.png')
        (out_dataset/'poses'/f'{dst}.json').write_text(json.dumps(item['row'],indent=2));np.savetxt(out_bundle/'ob_in_cam'/f'{dst}.txt',item['P'])
        rgb=cv2.imread(str(dataset/'rgb'/f'{src}.png'));mask=cv2.imread(str(dataset/'masks'/f'{src}.png'),0)>0;rgb[~mask]=0;cv2.imwrite(str(out_bundle/'color_segmented'/f'{dst}.png'),rgb);accepted.append(item)
    report['input_frames']=len(records);report['accepted_frames']=len(accepted);report['rejected_frames']=[{'source_stem':x['pose'].stem,'pass_id':x['row']['pass_id'],'translation_residual_mm':x['translation_residual_mm'],'rotation_residual_deg':x['rotation_residual_deg']} for x in records if not x['accepted']]
    (out_dataset/'tracking_quality_report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('tracking',type=Path);p.add_argument('out_dataset',type=Path);p.add_argument('out_bundle',type=Path);a=p.parse_args();print(json.dumps(run(a.dataset,a.tracking,a.out_dataset,a.out_bundle),indent=2))
