"""Full RGB or one depth-proposed crop; raw scores retained before filtering."""
import argparse,json,hashlib,contextlib,time
from pathlib import Path
import numpy as np
from PIL import Image
import torch
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor
p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--prompt',default='cabinet door handle');p.add_argument('--allow-proposal',action='store_true');a=p.parse_args()
f=a.folder;image=Image.open(f/'rgb.png').convert('RGB');model=build_sam3_image_model(checkpoint_path=str(a.checkpoint),load_from_HF=False);processor=Sam3Processor(model,confidence_threshold=.25);attempts=[];succeeded=False;captured={};original=model.forward_grounding
def observe(*args,**kwargs):
 out=original(*args,**kwargs);captured['scores']=(out['pred_logits'].sigmoid()*out['presence_logit_dec'].sigmoid().unsqueeze(1)).detach().float().cpu().numpy().reshape(-1);return out
model.forward_grounding=observe
for attempt in range(2 if a.allow_proposal else 1):
 started=time.perf_counter();box=[0,0,image.width,image.height];proposal=None
 if attempt:
  from native_g1_observed_regions import depth_proposal
  try:proposal=depth_proposal(f);box=proposal['crop_xyxy']
  except Exception as error:
   attempts.append(dict(kind='depth_proposal',error=repr(error)))
   (f/'visual_detection_attempts.json').write_text(json.dumps(dict(prompt=a.prompt,threshold=.25,input_sha256=hashlib.sha256((f/'rgb.png').read_bytes()).hexdigest(),attempts=attempts,GT_fallback=False),indent=2))
   break
 with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
  state=processor.set_image(image.crop(box));state=processor.set_text_prompt(state=state,prompt=a.prompt)
 scores=state['scores'].detach().float().cpu().numpy().reshape(-1);masks=state['masks'].detach().cpu().numpy();valid=[];reject=[]
 for i,(score,m) in enumerate(zip(scores,masks)):
  m=m.squeeze().astype(bool);border=m[0].any() or m[-1].any() or m[:,0].any() or m[:,-1].any();reason='too_few_pixels' if m.sum()<30 else ('ROI_border' if border else None);reject.append(dict(index=i,score=float(score),pixels=int(m.sum()),rejection=reason))
  if reason is None:valid.append(i)
 attempts.append(dict(kind='full_frame' if not attempt else 'observed_depth_crop',crop_xyxy=box,proposal=proposal,raw_query_scores=captured['scores'].tolist(),raw_max=float(captured['scores'].max()),threshold=.25,confidence_retained_scores=scores.tolist(),mask_filter=reject,accepted=bool(valid),wall_s=time.perf_counter()-started))
 (f/'visual_detection_attempts.json').write_text(json.dumps(dict(prompt=a.prompt,threshold=.25,input_sha256=hashlib.sha256((f/'rgb.png').read_bytes()).hexdigest(),attempts=attempts,GT_fallback=False),indent=2))
 if valid:
  best=max(valid,key=lambda i:scores[i]);mask=np.zeros((image.height,image.width),bool);x0,y0,x1,y1=box;mask[y0:y1,x0:x1]=masks[best].squeeze().astype(bool);np.save(f/'handle_mask.npy',mask);np.save(f/'sam3_mask_raw.npy',mask);arr=np.array(image);arr[mask]=(.45*arr[mask]+.55*np.array([0,255,50])).astype(np.uint8);Image.fromarray(arr).save(f/'handle_mask.overlay.png');succeeded=True;print(json.dumps(dict(success=True,recovery_used=bool(attempt),score=float(scores[best]),mask_pixels=int(mask.sum()))));break
else:raise RuntimeError('BOUNDED_VISUAL_DETECTION_FAILED_NO_GT_FALLBACK')
if not succeeded:raise RuntimeError('BOUNDED_VISUAL_DETECTION_FAILED_NO_GT_FALLBACK')
