from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1]/'results/real2sim_soup_scan_v2_dual/scan'
rows=[]
for pid,stems in [(0,['000000','000004','000008']),(1,['010000','010004','010008'])]:
    cells=[]
    for stem in stems:
        im=cv2.imread(str(ROOT/'rgb'/f'{stem}.png'));mask=cv2.imread(str(ROOT/'masks'/f'{stem}.png'),0);hand=cv2.imread(str(ROOT/'gripper_masks'/f'{stem}.png'),0)
        edge=cv2.Canny(mask,1,2)>0;hedge=cv2.Canny(hand,1,2)>0;im[edge]=[0,255,0];im[hedge]=[0,0,255]
        cv2.putText(im,f'pass {pid} / {stem}',(15,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2);cells.append(im)
    rows.append(np.hstack(cells))
out=ROOT.parent/'orthogonal_pass_views.png';cv2.imwrite(str(out),np.vstack(rows));print(out)
