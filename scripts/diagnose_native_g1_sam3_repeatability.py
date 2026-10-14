"""Saved-byte SAM3 diagnostic only: no Isaac imports or execution feedback."""
import argparse,contextlib,gc,hashlib,json,time
from pathlib import Path
import numpy as np
from PIL import Image
import torch
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
frames=[a.root/'45600'/phase/'rgbd'/view/'rgb.png' for phase,view in [('shadow_01','initial'),('physical_01','initial'),('physical_01','initial_retry_1')]]
def digest(t):return hashlib.sha256(t.detach().float().cpu().contiguous().numpy().tobytes()).hexdigest()
def run(model,processor,file,rep):
 captured={};original=model.forward_grounding
 def observe(*args,**kwargs):
  out=original(*args,**kwargs);captured['native_scores']=(out['pred_logits'].sigmoid()*out['presence_logit_dec'].sigmoid().unsqueeze(1)).detach().float().cpu().numpy().reshape(-1);captured.update({k:out[k].detach().cpu().float().numpy() for k in ['pred_logits','presence_logit_dec','pred_boxes']});return out
 model.forward_grounding=observe
 with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
  state=processor.set_image(Image.open(file).convert('RGB'));state=processor.set_text_prompt(state=state,prompt='cabinet door handle')
 model.forward_grounding=original
 raw=1/(1+np.exp(-captured['pred_logits']));presence=1/(1+np.exp(-captured['presence_logit_dec']));scores=captured['native_scores']
 masks=state['masks'].detach().cpu().numpy();retained=state['scores'].detach().float().cpu().numpy().reshape(-1);rejections=[]
 for i,m in enumerate(masks):
  m=m.squeeze().astype(bool);rejections.append(dict(index=i,score=float(retained[i]),pixels=int(m.sum()),border=bool(m[0].any() or m[-1].any() or m[:,0].any() or m[:,-1].any())))
 return dict(frame=str(file),input_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),repetition=rep,raw_scores=scores.tolist(),raw_max=float(scores.max()),raw_logit_sha256=hashlib.sha256(captured['pred_logits'].tobytes()).hexdigest(),retained_scores=retained.tolist(),mask_rejections_after_confidence=rejections,raw_query_count=len(scores),threshold=.25)
records=[];models=[];start=time.time()
for load in range(3):
 with (a.output/f'load_{load}.log').open('w') as log,contextlib.redirect_stdout(log):model=build_sam3_image_model(checkpoint_path=str(a.checkpoint),load_from_HF=False)
 processor=Sam3Processor(model,confidence_threshold=.25);missing={n:v for n,v in model.state_dict().items() if n.startswith('backbone.vision_backbone.convs.3.')};info=dict(load=load,training=model.training,scalp=model.backbone.scalp,missing_branch_parameters={n:dict(shape=list(v.shape),sha256=digest(v)) for n,v in missing.items()})
 for frame in frames:
  for rep in range(3):
   row=run(model,processor,frame,rep);row['load']=load;records.append(row);print(load,frame.parent.parent.parent.name,frame.parent.name,rep,row['raw_max'],row['retained_scores'],flush=True)
 if load==0:
  baseline=run(model,processor,frames[1],99)
  old={n:v.detach().clone() for n,v in model.named_parameters() if n in missing}
  with torch.no_grad():
   for n,v in model.named_parameters():
    if n in old:v.zero_()
  zero=run(model,processor,frames[1],100);info['missing_branch_zero_ablation']=dict(raw_scores_max_abs_difference=float(np.max(abs(np.asarray(zero['raw_scores'])-baseline['raw_scores']))),identical_raw_logits=baseline['raw_logit_sha256']==zero['raw_logit_sha256'],baseline=baseline['retained_scores'],zero=zero['retained_scores'])
  with torch.no_grad():
   for n,v in model.named_parameters():
    if n in old:v.copy_(old[n])
 models.append(info);del processor,model;gc.collect();torch.cuda.empty_cache()
(a.output/'REPEATABILITY.json').write_text(json.dumps(dict(models=models,inferences=records,wall_s=time.time()-start,configuration=dict(prompt='cabinet door handle',threshold=.25,precision='CUDA bfloat16 autocast',model_checkpoint=str(a.checkpoint),RNG_seeds_not_overridden=True)),indent=2))
