"""Prismatic-only engineering hypothesis: rate-consistent causal unloading.

Reuse the existing force direction, gain, jaw-axis projection, centering ball,
per-native-step displacement and speed caps. Change only integration cadence:
the old implementation applies one native-step cap per filtered-load window.
The runtime still supplies the same trailing (not future) filtered wrench.
Not enabled by default; matched physical comparisons precede activation.
"""
from interaction_runtime.prismatic_unloading_v751 import TransverseUnloading


class RateConsistentFollowing(TransverseUnloading):
    def __init__(self,policy,rail,period_steps,closure_offset):
        self.load_window_steps=int(period_steps)
        super().__init__(policy,rail,1,closure_offset)


def selection(command,joint_type):
    mode=command.get('contact_following','legacy_window')
    if mode not in ('legacy_window','rate_consistent'):
        raise ValueError('KNOWN_CONTACT_FOLLOWING_MODE_REQUIRED')
    if joint_type!='prismatic' and mode!='legacy_window':
        raise ValueError('CONTACT_FOLLOWING_EXPERIMENT_IS_PRISMATIC_ONLY')
    if mode=='rate_consistent' and not command.get('transverse_unloading',False):
        raise ValueError('CONTACT_FOLLOWING_REQUIRES_EXPLICIT_UNLOADING_SELECTION')
    if mode=='rate_consistent':
        # Retain the committed engineering ablation for reproducibility, but
        # its native matched result was worse than the baseline. The fixture
        # selects it explicitly without this full-Agent admission interface.
        raise ValueError('MATCHED_NATIVE_FOLLOWING_ABLATION_NOT_APPROVED_FOR_AGENT_EXECUTION')
    return mode
