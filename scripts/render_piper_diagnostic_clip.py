"""Trim actual simulation footage and label its retrospective validation status."""
import argparse,shutil,subprocess
from pathlib import Path
import cv2

p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('output',type=Path);p.add_argument('--start',type=float,default=38.);a=p.parse_args()
c=cv2.VideoCapture(str(a.input));fps=c.get(cv2.CAP_PROP_FPS);w=int(c.get(3));h=int(c.get(4))
if not c.isOpened() or fps<=0:raise RuntimeError('cannot read actual video')
encoder=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','bgr24','-s',f'{w}x{h}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','4','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(a.output)],stdin=subprocess.PIPE)
i=0
try:
    while True:
        ok,frame=c.read()
        if not ok:break
        t=i/fps;i+=1
        if not (t<3 or t>=a.start):continue
        cv2.rectangle(frame,(0,h-80),(w,h),(0,0,0),-1)
        cv2.putText(frame,'DIAGNOSTIC ONLY - NOT A VALIDATED GRASP',(15,h-49),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
        cv2.putText(frame,'Final strict mesh audit rejects body contact at closure',(15,h-17),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1)
        encoder.stdin.write(frame.tobytes())
finally:
    c.release();encoder.stdin.close();encoder.wait(timeout=60)
if encoder.returncode:raise RuntimeError('video encoding failed')
