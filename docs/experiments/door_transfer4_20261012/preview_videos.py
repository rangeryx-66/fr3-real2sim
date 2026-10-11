from pathlib import Path
import subprocess,json,sys
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/door_transfer4_20261012')
ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg'
for ident in sys.argv[1:]:
 p=root/ident/'physical_01';src=p/'contact_baseline.mp4'
 subprocess.run([ff,'-y','-i',str(src),'-vf','setpts=PTS/20','-an','-c:v','libx264','-preset','fast','-crf','22',str(p/'summary_20x.mp4')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 subprocess.run([ff,'-y','-sseof','-0.25','-i',str(src),'-frames:v','1',str(p/'final_frame.png')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 print(ident,flush=True)
