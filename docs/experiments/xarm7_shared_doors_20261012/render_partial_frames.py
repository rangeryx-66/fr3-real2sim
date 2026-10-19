"""Actual progress PNGs; watermarked partial evidence, no physics/replay."""
from pathlib import Path
import json,subprocess,cv2,numpy as np
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012/45443/physical_01');D=O/'actual_progress_frames';d=json.loads((D/'INDEX.json').read_text());all_rows=d['frames'];rows=[];unreadable=[]
for item in all_rows:
 if cv2.imread(str(D/item['file'])) is None:unreadable.append(item['file'])
 else:rows.append(item)
ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';out=O/'PARTIAL_ACTUAL_PROGRESS.mp4'
v=subprocess.Popen([ff,'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','10','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(out)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(O/'partial_video_ffmpeg.log').open('w'))
for i,r in enumerate(rows):
 image=cv2.imread(str(D/r['file']));cv2.rectangle(image,(0,0),(1280,85),(0,0,0),-1);cv2.putText(image,'PARTIAL ACTUAL PROGRESS IMAGES / NOT FULL NATIVE VIDEO',(12,30),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);cv2.putText(image,'sim %.2f s | actual angle %.3f deg'%(r['reported_sim_time_s'],r['actual_angle_deg']),(12,65),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
 rgb=np.ascontiguousarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)).tobytes();seconds=rows[i+1]['reported_sim_time_s']-r['reported_sim_time_s'] if i+1<len(rows) else 1.
 for _ in range(max(1,round(seconds*10))):v.stdin.write(rgb)
v.stdin.close();code=v.wait(timeout=60);(O/'PARTIAL_VIDEO_METADATA.json').write_text(json.dumps({'source':'actual saved progress PNGs; no new physics or state replay','native_continuous_video_available':False,'first_angle_deg':rows[0]['actual_angle_deg'],'last_angle_deg':rows[-1]['actual_angle_deg'],'first_reported_sim_s':rows[0]['reported_sim_time_s'],'last_reported_sim_s':rows[-1]['reported_sim_time_s'],'image_count':len(rows),'unreadable_original_progress_files_preserved':unreadable,'ffmpeg_returncode':code,'timing_qualification':rows[0]['timing_qualification']},indent=2));print('Partial actual image clip:',code,len(rows))
