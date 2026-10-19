"""Frozen complete actions; unseen reverse pulse is never used for selection."""
from interactive_twin.execution import PhysicsProtocol as OriginalProtocol
import numpy as np

class PhysicsProtocol(OriginalProtocol):
    def start(self,s,T):
        super().start(s,T)
        if s['pattern']=='reverse_pulse':
            self.drive.set_direction(-self.memory.tangent(T),T)
    def update(self,s,t,T):
        if s['pattern']!='reverse_pulse':
            return super().update(s,t,T)
        self.drive.active=1. <= t < 4.8
        if np.linalg.norm(T[:3,3]-self.segment_start)>=.001:
            self.drive.refresh_tangent(-self.memory.tangent(T),T)
            self.segment_start=T[:3,3].copy()
