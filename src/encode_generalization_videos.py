from pathlib import Path
import json,subprocess
import imageio_ffmpeg
import cv2
root=Path(__file__).resolve().parents[1];out=root/'videos/generalization'
manifest=json.loads((out/'manifest.json').read_text());assert len(manifest)==6
ff=imageio_ffmpeg.get_ffmpeg_exe()
for r in manifest:
 stem=f"{r['group']}_seed_{r['seed']}";dest=out/(stem+'.mp4')
 subprocess.run([ff,'-y','-loglevel','error','-i',str(out/(stem+'_raw.mp4')),'-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(dest)],check=True)
 cap=cv2.VideoCapture(str(dest));assert cap.isOpened();assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==r['frames'];cap.release()
 r['video']=dest.name;r['duration_s']=r['frames']/r['fps'];print('ENCODED',dest.name,r['duration_s'],flush=True)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
