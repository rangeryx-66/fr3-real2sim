"""Method A/evaluator only. Method B never imports this F/T reader.

Use the exact same pure kinematic interface as B, with original F/T processing,
without editing archived settings/models or borrowing an F/T-derived geometry.
"""
from unittest.mock import patch
from .effort import known_geometry


def load_ft_reference(episode,allow_interrupted_fit=False):
    import update_probing_v7_model as owner
    prior=known_geometry(episode);original=owner.process
    def process(rows,unused_prior,bias,*args,**kwargs):
        return original(rows,prior,bias,*args,**kwargs)
    with patch.object(owner,'process',process):
        return owner.load_segments(episode,allow_interrupted_fit)
