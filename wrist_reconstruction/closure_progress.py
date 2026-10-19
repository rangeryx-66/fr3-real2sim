"""Progress checks for recovery closure; nominal successful prefix is untouched."""
import numpy as np

class ClosureProgress:
    def __init__(self):
        self.empty_s=0.;self.bound_s=0.
    def update(self,closure,opening,forces,dt):
        loads=np.asarray(forces,float)
        if loads.min()>=.05:
            self.empty_s=0.;self.bound_s=0.;return
        self.empty_s=self.empty_s+dt if opening<.001 else 0.
        displacement=np.linalg.norm(closure.command[:3,3]-closure.origin[:3,3])
        at_bound=displacement>=closure.p['max_centering_displacement_m']-.0001
        self.bound_s=self.bound_s+dt if at_bound else 0.
        if self.empty_s>=.5:raise RuntimeError('GRASP_TEMPLATE_EMPTY_CLOSURE')
        if self.bound_s>=.5:raise RuntimeError('GRASP_TEMPLATE_CENTERING_EXHAUSTED')
