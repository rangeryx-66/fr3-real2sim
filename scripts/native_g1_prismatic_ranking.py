"""Observed-only bidirectional rail capacity on the existing native-G1 ranker.

Existing acquisition, perturbations, retention, priors and scalar coefficients
are retained. An optimistic best-finger / opening-only cone feature is replaced
by the weaker direction's mean two-pad capacity. Equal unit pad-normal loading
and isotropic mu are declared relative ranking assumptions, not force estimates.
No GT surface, calibrated friction, hard quality gate or changed candidate pose.
"""
import inspect,json,time,types
from pathlib import Path
import numpy as np


def bidirectional_capacity(forward,reverse):
    return min(float(np.mean([s['opening_capacity_assumed_mu'] for s in r['sides']]))
               for r in (forward,reverse))


def rescore(geometries,model,surfaces,base,origin,axis,config,output):
    import native_g1_robust_contact_ranking as frozen
    from native_g1_multiview_input import fast_ray
    if model.manifest.get('joint_type')!='prismatic':
        raise ValueError('Rail rank adapter must never replace revolute ranking')
    sample=types.FunctionType(frozen.RobustContactRank.sample.__code__,dict(frozen.__dict__,ray_surface=fast_ray))
    class RailRank(frozen.RobustContactRank):
        def sample(self,T,width,mu):
            original=self.surfaces;axis=self.axis.copy();results=[]
            try:
                for h in original[2]:
                    self.surfaces=h;forward=sample(self,T,width,mu)
                    self.axis=-axis;reverse=sample(self,T,width,mu);self.axis=axis
                    r=dict(forward);r['task_score']=bidirectional_capacity(forward,reverse)
                    r['combined']=r['contact_score']*r['retention_score']*r['task_score']
                    r['bidirectional_capacity']={'opening_per_pad':[s['opening_capacity_assumed_mu'] for s in forward['sides']],'closing_per_pad':[s['opening_capacity_assumed_mu'] for s in reverse['sides']],'normal_loading_assumption':'equal unit normal per pad; approximate score only'}
                    results.append(r)
            finally:self.surfaces=original;self.axis=axis
            nominal=dict(results[1])
            for key in ['combined','contact_score','retention_score','task_score']:
                values=[r[key] for r in results]
                nominal[key]=float(self.cfg['mean_weight']*np.mean(values)+self.cfg['lower_quartile_weight']*np.quantile(values,.25))
            nominal['hidden_geometry_samples']=[dict(scale=s,combined=r['combined'],contact=r['contact_score'],retention=r['retention_score'],task=r['task_score']) for s,r in zip([.75,1.,1.25],results)]
            return nominal
    namespace=dict(frozen.__dict__,RobustContactRank=RailRank)
    exec(inspect.getsource(frozen.rescore_geometries),namespace)
    ranked=namespace['rescore_geometries'](geometries,model,surfaces,base,origin,axis,config,output)
    p=Path(output)/'prismatic_ranking_assumptions.json'
    p.write_text(json.dumps(dict(joint_type='prismatic',observed_geometry_only=True,unchanged_weights=True,unchanged_shortlist_budget=True,force_direction='both signs of known rail axis',friction_mu=config['friction_cone_assumption_mu'],measured_friction=False,GT_used=False,new_execution_gates=False),indent=2))
    return ranked
