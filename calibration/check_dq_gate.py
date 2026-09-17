"""Regression tests that the static gate rejects motion and bad clocks."""
import numpy as np
from quasistatic_velocity_gate import configuration_velocity
from strict_pair_protocol import window_quality

t=np.arange(240)/240
base=dict(t=t,q=np.zeros((240,7)),dq=np.full((240,7),.04),tau=np.zeros((240,7)),actual_opening_m=np.full(240,.04),guard_passed=np.ones(240,bool))
assert window_quality(base,np.zeros(7),.04,payload=False)['accepted']
moving=dict(base,q=np.tile((t*.003)[:,None],(1,7)),dq=np.zeros((240,7)))
assert 'NONZERO_DQ' in window_quality(moving,np.zeros(7),.04,payload=False)['reasons']
for bad_t in [t[::-1],np.where(np.arange(240)>100,t+1/240,t),t*2]:
 try:configuration_velocity(bad_t,base['q'])
 except ValueError:pass
 else:raise AssertionError('Bad clock accepted')
print('PASS: stationary actual q, moving actual q, reversed clock, missing step, wrong sample period')
