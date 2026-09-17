"""Perspective visualization of stored AnyGrasp output and real rendered depth points."""
import json
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'videos';OUT.mkdir(exist_ok=True)
RUN=ROOT/'results/run_1788943864166403373'
r=json.loads((RUN/'trial_01.json').read_text());data=json.loads((RUN/'trial_01_grasps.json').read_text())
cloud=np.load(r['capture']['path']);T=np.array(data['T_B_C'])
points=cloud['points']@T[:3,:3].T+T[:3,3]
mask=cloud['mask'].astype(bool)
roi=(np.abs(points[:,0]-.5)<.15)&(np.abs(points[:,1])<.15)&(points[:,2]<.16)
ids=np.where(roi)[0];rng=np.random.default_rng(0)
if len(ids)>8000:ids=np.sort(rng.choice(ids,8000,replace=False))
points=points[ids];mask=mask[ids]
writer=cv2.VideoWriter(str(OUT/'anygrasp_raw.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),30,(1280,720));assert writer.isOpened()
center=np.array([.5,0,.035])
for frame_index in range(360):
    frame=np.full((720,1280,3),(24,30,39),dtype=np.uint8)
    angle=np.deg2rad(-60+frame_index/359*120)
    eye=center+np.array([.31*np.cos(angle),.31*np.sin(angle),.24])
    z=center-eye;z=z/np.linalg.norm(z);x=np.cross(z,[0,0,1]);x=x/np.linalg.norm(x);y=np.cross(z,x)
    R=np.column_stack([x,y,z])
    def project(p):
        q=(np.asarray(p)-eye)@R
        return np.rint(q[:,:2]/q[:,2,None]*850+[465,390]).astype(int)
    def line(a,b,color,thick=2):
        uv=project(np.array([a,b]));cv2.line(frame,tuple(uv[0]),tuple(uv[1]),color,thick,cv2.LINE_AA)
    uv=project(points);order=np.argsort(((points-eye)@R)[:,2])[::-1]
    for i in order:
        u,v=uv[i]
        if 0<u<940 and 110<v<640:
            cv2.circle(frame,(u,v),2 if mask[i] else 1,(70,148,239) if mask[i] else (103,95,79),-1)
    selected_only=frame_index>=210
    grasps=data['grasps'][:1] if selected_only else data['grasps']
    for g in reversed(grasps):
        rg=T[:3,:3]@np.array(g['rotation']);t=T[:3,:3]@g['translation']+T[:3,3]
        w=g['width'];d=g['depth'];color=(112,241,91) if g['rank']==0 else (229,193,92)
        def world(p):return rg@np.array(p)+t
        # SDK convention: fingers extend from x=-0.02 to x=predicted depth.
        for yy in [-w/2,w/2]:
            corners=[[-.02,yy,-.002],[d,yy,-.002],[d,yy,.002],[-.02,yy,.002]]
            for j in range(4):line(world(corners[j]),world(corners[(j+1)%4]),color,3 if g['rank']==0 else 1)
        line(world([-.02,-w/2,0]),world([-.02,w/2,0]),color,3 if g['rank']==0 else 1)
        line(world([-.06,0,0]),world([-.02,0,0]),color,2)
        if g['rank']==0:
            pp=project(np.array([world([-.075,0,0]),world([-.032,0,0])]))
            cv2.arrowedLine(frame,tuple(pp[0]),tuple(pp[1]),(112,241,91),3,cv2.LINE_AA,tipLength=.22)
            u,v=project(np.array([world([-.07,0,.012])]))[0]
            cv2.putText(frame,'#0',(u,v),cv2.FONT_HERSHEY_SIMPLEX,.65,color,2,cv2.LINE_AA)
    # Actual robot-base axes, with a 4 cm scale.
    origin=np.array([.39,-.10,.001])
    for axis,color,label in [(0,(90,95,245),'X'),(1,(108,224,120),'Y'),(2,(243,166,90),'Z')]:
        end=origin+np.eye(3)[axis]*.04;line(origin,end,color,2)
        u,v=project(np.array([end]))[0];cv2.putText(frame,label,(u+3,v-3),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
    cv2.rectangle(frame,(0,0),(1280,103),(16,22,30),-1)
    cv2.putText(frame,'AnyGrasp | Actual grasp candidates',(30,42),cv2.FONT_HERSHEY_SIMPLEX,1,(245,246,248),2,cv2.LINE_AA)
    cv2.putText(frame,'Stored trial 01 depth cloud and SDK poses / robot-base coordinates',(31,79),cv2.FONT_HERSHEY_SIMPLEX,.6,(182,193,209),1,cv2.LINE_AA)
    cv2.rectangle(frame,(947,104),(1280,650),(19,25,34),-1)
    cv2.putText(frame,'CANDIDATES',(974,142),cv2.FONT_HERSHEY_SIMPLEX,.7,(230,233,240),2,cv2.LINE_AA)
    for i,g in enumerate(data['grasps']):
        color=(112,241,91) if i==0 else (185,194,204)
        cv2.putText(frame,f"#{i:<2}   score {g['score']:.3f}",(976,184+i*29),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1,cv2.LINE_AA)
    cv2.putText(frame,'Green: selected #0',(976,559),cv2.FONT_HERSHEY_SIMPLEX,.55,(112,241,91),1,cv2.LINE_AA)
    cv2.putText(frame,'Orange: box depth',(976,590),cv2.FONT_HERSHEY_SIMPLEX,.55,(70,148,239),1,cv2.LINE_AA)
    cv2.rectangle(frame,(0,650),(1280,720),(16,22,30),-1)
    label='Selected grasp / approach direction' if selected_only else f"Top-K output: {len(data['grasps'])} candidates returned (K=20)"
    cv2.putText(frame,label,(30,686),cv2.FONT_HERSHEY_SIMPLEX,.72,(229,233,240),1,cv2.LINE_AA)
    writer.write(frame)
    if frame_index in [60,260]:cv2.imwrite(str(OUT/f'anygrasp_{frame_index:03d}.jpg'),frame)
writer.release();print('ANYGRASP_VIDEO_COMPLETE',flush=True)
