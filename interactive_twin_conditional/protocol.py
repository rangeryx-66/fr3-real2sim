"""Frozen actions selected before any new held-out response exists."""
from interactive_twin.execution import PhysicsProtocol as BaseProtocol
import numpy as np

class PhysicsProtocol(BaseProtocol):
    def start(self,s,T):
        super().start(s,T)
        self.sign = -1. if s['pattern']=='closing' else 1.
        self.drive.set_direction(self.sign*self.memory.tangent(T),T)
    def update(self,s,t,T):
        self.drive.active=any(lo<=t<hi for lo,hi in s['active_windows_s'])
        if np.linalg.norm(T[:3,3]-self.segment_start)>=.001:
            self.drive.refresh_tangent(self.sign*self.memory.tangent(T),T)
            self.segment_start=T[:3,3].copy()
