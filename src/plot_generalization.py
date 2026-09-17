"""Saved-data plots only; run after both formal batches."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]/'results/generalization_summary'
s=json.loads((R/'summary.json').read_text());rows=list(csv.DictReader((R/'episodes.csv').open()));cands=list(csv.DictReader((R/'candidates.csv').open()))
fig,ax=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
labels=['Development\n16/20','Held-out clutter\n50 scenes','Held-out target pose\n30 scenes'];groups=['development','generalization_clutter50','generalization_pose30']
for i,g in enumerate(groups):
 d=s['development'] if i==0 else s['groups'][g];p=d['success']/d['n'];lo,hi=d['ci95'];ax[0].errorbar(i,100*p,yerr=[[100*(p-lo)],[100*(hi-p)]],fmt='o',capsize=6,color=['gray','#176aa3','#c56727'][i])
ax[0].set(xticks=range(3),xticklabels=labels,ylabel='Pick success (%)',ylim=(0,105),title='Observed success with 95% Wilson CI');ax[0].axhline(75,color='gray',ls='--',lw=1)
for g,color in zip(groups[1:],['#176aa3','#c56727']):
 v=sorted(float(c['coverage'])*100 for c in cands if c['group']==g and c['status']=='INSUFFICIENT_PAD_OVERLAP')
 if v:ax[1].step(v,np.arange(1,len(v)+1)/len(v),where='post',label=g.replace('generalization_','')+f' (n={len(v)})',color=color)
ax[1].axvline(4.5,color='black',ls='--');ax[1].set(xlabel='Rejected candidate pad coverage (%)',ylabel='Cumulative fraction',xlim=(-.2,5),ylim=(0,1.05),title='Candidates rejected by frozen 4.5% gate');ax[1].legend();fig.savefig(R/'generalization.png',dpi=170)
