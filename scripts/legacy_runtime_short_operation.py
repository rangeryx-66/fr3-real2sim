"""Matched unloaded endpoints under the old loaded/unloaded timing and evidence."""
import numpy as np
from xarm_wrist_ft import install_passive
from native_g1_recovery_telemetry import install

def run(g):
    install_passive(g);install(g)
    home=g['qtarget'][g['arm']].copy();other=home.copy();other[0]+=.12;other[2]-=.08
    g['phase']='SETTLE';g['hold'](.3)
    for _ in range(int(g['job'].get('engineering_cycles',2))):
        g['phase']='PREGRASP';g['move'](other,.8)
        g['phase']='PREGRASP';g['move'](home,.8)
    g['hold'](.3)
    return dict(success=True,status='LEGACY_UNLOADED_CONTROL_COMPLETE',physical_grasps=0,full_Agent_episode=False)
