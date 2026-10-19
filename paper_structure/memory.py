"""One-shot ablation changes memory scheduling, not controller or fitter."""
import numpy as np
from operational_structure.confidence import OperationalMemory
from operational_structure.prediction import prediction

class OnceMemory(OperationalMemory):
    def refresh(self):
        self.save() # no structure optimization after discovery
    def resolve(self,t):
        p=self.policy['confidence'];X=np.asarray(self.poses);recent=X[-min(len(X),p['recent_samples']):]
        v=prediction(recent,self.estimate)
        explained=v['maximum_position_error_m']<=p['explanation_position_m'] and v['rotation_rmse_rad']<=p['explanation_rotation_rad']
        self.count_window(explained,recent);self.monitor_anchor=recent[-1].copy()
        self.step_scale=max(p['minimum_step_scale'],self.step_scale*p['step_reduction'])
        self.pending=False;self.inconsistent_s=0.
        self.events.append({'event':'FROZEN_MODEL_REDUCE_STEP','t':t,'explained':bool(explained),'fit_called':False,'physical_failure':False})
        self.save_events()
        if self.failed_refits>=p['maximum_unexplained_refits']:raise RuntimeError('MODEL_CONFIDENCE_UNRESOLVED')
