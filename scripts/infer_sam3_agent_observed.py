"""Observe frozen SAM3 filtering, retaining rejected masks as view cues only.

No threshold/prompt/valid-mask rule change. Unlike grasp initialization, an
observation without a valid mask is a result for the Agent to inspect.
"""
from pathlib import Path
original=Path(__file__).with_name('infer_sam3_observed.py')
text=original.read_text()
anchor=" scores=state['scores'].detach().float().cpu().numpy().reshape(-1);masks=state['masks'].detach().cpu().numpy();valid=[];reject=[]"
observer='''
 import cv2
 cues=json.loads((f/'candidate_cues.json').read_text()) if (f/'candidate_cues.json').exists() else []
 for ci,(score,local_mask) in enumerate(zip(scores,masks)):
  local_mask=local_mask.squeeze().astype(bool);full=np.zeros((image.height,image.width),bool);x0,y0,x1,y1=box;full[y0:y1,x0:x1]=local_mask
  name='candidate_%02d_%03d'%(attempt,ci);np.save(f/(name+'.npy'),full)
  yy,xx=np.where(full)
  borders=dict(left=bool(local_mask[:,0].any()),right=bool(local_mask[:,-1].any()),top=bool(local_mask[0].any()),bottom=bool(local_mask[-1].any()))
  arr=np.array(image);arr[full]=(.45*arr[full]+.55*np.array([255,150,0] if any(borders.values()) else [0,255,70])).astype(np.uint8)
  Image.fromarray(arr).save(f/(name+'.overlay.png'))
  cues.append(dict(name=name,score=float(score),kind='full_frame' if not attempt else 'observed_depth_crop',pixels=int(full.sum()),bbox_xyxy=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)] if len(xx) else None,borders=borders,partial=any(borders.values()),eligible_for_grasp=bool(full.sum()>=30 and not any(borders.values())),role='partial cue for viewpoint reasoning; existing border filter still applies to grasp input'))
 (f/'candidate_cues.json').write_text(json.dumps(cues,indent=2))
'''
if text.count(anchor)!=1:raise ValueError('Frozen SAM observation hook changed')
text=text.replace(anchor,anchor+observer)
text=text.replace("else:raise RuntimeError('BOUNDED_VISUAL_DETECTION_FAILED_NO_GT_FALLBACK')","else:print('NO_VALID_MASK_OBSERVATION_RETAINED_FOR_AGENT')")
text=text.replace("if not succeeded:raise RuntimeError('BOUNDED_VISUAL_DETECTION_FAILED_NO_GT_FALLBACK')","if not succeeded:print('NO_VALID_MASK_OBSERVATION_RETAINED_FOR_AGENT')")
exec(compile(text,str(original)+'::unchanged_filter_with_partial_observer','exec'),globals())
try:(f/'SAM3_PARTIAL_OBSERVER_EXECUTED.py').write_text(text)
except Exception as error:print('optional_observer_source_export',repr(error))
