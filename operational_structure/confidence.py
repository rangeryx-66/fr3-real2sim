"""Model warnings and physical safety have independent state and labels."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from active_structure.discovery import ProvisionalMemory
from interactive_twin_refinement.fitting import fit_se3
from operational_structure.prediction import prediction


class OperationalMemory(ProvisionalMemory):
    def __init__(self,T,out,policy):
        super().__init__(T,out,policy)
        self.pending=False;self.inconsistent_s=0.;self.failed_refits=0;self.events=[];self.step_scale=1.
        self.monitor_anchor=None;self.unexplained_origin=None

    def consistency_error(self,T):
        if self.monitor_anchor is None:return super().consistency_error(T)
        return prediction(np.asarray([self.monitor_anchor,T]),self.estimate)['endpoint_error_m']

    def count_window(self,explained,recent):
        if explained:self.failed_refits=0;self.unexplained_origin=None
        else:
            if self.unexplained_origin is None:self.unexplained_origin=recent[0,:3,3].copy()
            if np.linalg.norm(recent[-1,:3,3]-self.unexplained_origin)>=self.policy['confidence']['hard_window_motion_m']:
                self.failed_refits+=1;self.unexplained_origin=recent[-1,:3,3].copy()

    def monitor(self,error,dt,t,phase):
        p=self.policy['confidence']
        self.inconsistent_s=self.inconsistent_s+dt if (error or 0)>p['soft_trigger_m'] else 0.
        if self.inconsistent_s>=p['soft_duration_s'] and not self.pending:
            self.pending=True
            self.events.append({'event':'SOFT_MODEL_INCONSISTENCY','t':t,'phase':phase,'model_error_m':error,'physical_failure':False})
            self.save_events()
        return self.pending

    def save_events(self):
        (self.output/'model_confidence.json').write_text(json.dumps({'events':self.events,'step_scale':self.step_scale,
          'failed_refits':self.failed_refits,'not_a_slip_measurement':True},indent=2))

    def resolve(self,t):
        p=self.policy['confidence'];X=np.asarray(self.poses)
        f=fit_se3(X,**self.policy['fitting']) # identical frozen fitter
        recent=X[-min(len(X),p['recent_samples']):]
        candidates=[f,self.estimate,*self.hypotheses]
        audits=[{'position_error_m':prediction(recent,x)['maximum_position_error_m'],
                 'rotation_error_rad':prediction(recent,x)['rotation_rmse_rad']} for x in candidates]
        explained=any(r['position_error_m']<=p['explanation_position_m'] and r['rotation_error_rad']<=p['explanation_rotation_rad'] for r in audits)
        if f['optimizer']['success']:
            if np.asarray(f['revolute']['axis'])@np.asarray(self.estimate['revolute']['axis'])<0:self.follow_sign*=-1
            self.estimate=f;self.save()
        # Dwell/repeated fits at one pose cannot be independent failed segments.
        self.count_window(explained,recent)
        # Keep latent reference and full reconstruction error unchanged; the
        # incremental confidence forecast starts at the latest measured pose.
        self.monitor_anchor=recent[-1].copy()
        self.step_scale=max(p['minimum_step_scale'],self.step_scale*p['step_reduction'])
        self.pending=False;self.inconsistent_s=0.
        self.events.append({'event':'REFIT_REDUCE_STEP','t':t,'explained':bool(explained),'candidate_predictions':audits,
                            'optimizer_converged':f['optimizer']['success'],'independent_failed_windows':self.failed_refits,
                            'physical_failure':False})
        self.save_events()
        if self.failed_refits>=p['maximum_unexplained_refits']:raise RuntimeError('MODEL_CONFIDENCE_UNRESOLVED')
        return bool(explained)

    def choose_segment(self,*args):
        old=self.policy['refinement']['segment_m']
        self.policy['refinement']['segment_m']=old*self.step_scale
        try:return super().choose_segment(*args)
        finally:self.policy['refinement']['segment_m']=old
