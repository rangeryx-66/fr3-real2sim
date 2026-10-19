"""Visual-only uncertain-surface wrapper around the frozen contact scorer."""
import inspect
import numpy as np
import native_g1_robust_contact_ranking as frozen

class HypothesisContactRank(frozen.RobustContactRank):
 def sample(self,T,width,mu):
  original=self.surfaces;results=[]
  for hypothesis in original[2]:
   self.surfaces=hypothesis
   results.append(super().sample(T,width,mu))
  self.surfaces=original
  nominal=dict(results[1]);aggregate=lambda key:float(self.cfg['mean_weight']*np.mean([s[key] for s in results])+self.cfg['lower_quartile_weight']*np.quantile([s[key] for s in results],.25))
  for key in ['combined','contact_score','retention_score','task_score']:nominal[key]=aggregate(key)
  nominal['hidden_geometry_samples']=[dict(scale=scale,combined=r['combined'],contact=r['contact_score'],retention=r['retention_score'],task=r['task_score']) for scale,r in zip([.75,1.,1.25],results)]
  nominal['hypotheses_are_soft_ranking_not_vetoes']=True
  return nominal

def rescore_visual_geometries(*args,**kwargs):
 # Independent namespace: frozen Oracle scorer/class and weights are untouched.
 ns=dict(frozen.__dict__,RobustContactRank=HypothesisContactRank)
 exec(compile(inspect.getsource(frozen.rescore_geometries),__file__+'::unchanged_scoring_structure','exec'),ns)
 return ns['rescore_geometries'](*args,**kwargs)
