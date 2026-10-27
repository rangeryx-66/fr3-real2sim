"""Engineering action authority, separate from immutable emergency supervision.

Only an explicitly registered V87 actor can use this policy. No threshold is
increased. The provisional sustained band requests bounded reference braking; it is
never permission to continue pulling above that band. Emergency measurements,
invalid state, actual collision, joint/speed/effort guards remain authoritative.
"""
from types import SimpleNamespace
from interaction_runtime.policy import ForceMonitor

POLICY = 'classified_continuous_v2'
PREFIX = 'CONTROLLED_EXECUTION_INTERRUPT:'


def enabled(r):
    return bool(getattr(r,'job',{}).get('v87_constraint_policy') == POLICY)


def actionable(r):
    """No state/geometry uncertainty may authorize a protective movement."""
    from agentic_interaction_v81.recovery_supervisor import trusted
    return trusted(SimpleNamespace(r=r))


class ActionForceMonitor(ForceMonitor):
    """Reuse exact native filtering and emergency precedence, never clear a latch."""
    def __init__(self, policy, runtime, previous=None):
        super().__init__(policy)
        if previous is not None:
            self.history = previous.history
            self.last = previous.last
            self.events = previous.events
            self.pending = previous.pending
        self.runtime = runtime
        self.regulation_active = False
        self.feedback = dict(scale=1., authority='RECORD')

    def update(self, pads, dt, t, wrench=None):
        event = super().update(pads, dt, t, wrench)
        # Native trailing duration/weights are retained, including across phases.
        duration = sum(x[1] for x in self.history)
        means = [sum(x[1]*x[k] for x in self.history)/max(duration, dt)
                 for k in (2, 3, 4)]
        bands = [('pad', 'n'), ('wrist', 'n'), ('torque', 'nm')]
        fractions = [(value-self.p[name+'_warning_'+unit]) /
            (self.p[name+'_sustained_'+unit]-self.p[name+'_warning_'+unit])
            for value, (name, unit) in zip(means, bands)]
        fraction = max(0., min(1., max(fractions)))
        scale = 1.-fraction
        loaded = self.runtime.policy.phase(self.runtime.g['phase']) == 'LOADED'
        qualified = event['status'] == 'SUSTAINED_OVERLOAD' and actionable(self.runtime)
        original = event['status']
        if original == 'SUSTAINED_OVERLOAD' and qualified and (loaded or self.regulation_active):
            # This event has not reached SafetySupervisor.check's latch branch.
            # An existing latch is never cleared or rewritten here.
            self.regulation_active = True
            event = dict(event, status='CONTINUOUS_LOAD_REGULATION', stop=False,
                original_status=original, authority='CONTINUOUS_REGULATION',
                emergency_limits_unchanged=True)
            self.last = event.copy()
            self.events[-1] = event.copy()
        elif self.regulation_active and self.settled() and not loaded:
            self.regulation_active = False
        self.feedback = dict(scale=scale if loaded else 1.,
            authority='CONTINUOUS_REGULATION' if self.regulation_active or loaded and scale < 1. else 'RECORD',
            trailing_pad_n=means[0], trailing_wrist_n=means[1],
            trailing_torque_nm=means[2], original_status=original,
            load_regulation_active=self.regulation_active)
        return event


def motion(r, limits):
    """Smooth bounded reference adaptation using existing warning/sustained bands.

    Acceleration is still bounded by the original reference and final-command
    governors; a reduced desired speed is never imposed as an instantaneous
    final-command cap. Keep original acceleration bounds and scale both speeds.
    """
    monitor = getattr(r.supervisor, 'force', None)
    feedback = getattr(monitor, 'feedback', {})
    scale = float(feedback.get('scale', 1.))
    measured = float(r.rows[-1].get('tcp_speed_m_s', 0.)) if r.rows else 0.
    hard = float(limits['measured_speed_stop_m_s'])
    # Existing phase command speed is the regulation onset; no new 90% gate.
    span = hard-limits['speed_m_s']
    speed_scale = max(0., min(1., (hard-measured)/span)) if span > 0 else 0.
    r.constraint_feedback = dict(feedback, measured_speed_scale=speed_scale)
    predictive = float(getattr(r, 'v87_predictive_feedback', {}).get('scale', 1.))
    combined = min(scale, speed_scale, predictive)
    return dict(limits, speed_m_s=limits['speed_m_s']*combined,
        angular_speed_rad_s=limits['angular_speed_rad_s']*combined)


class PredictiveLoadRegulator:
    """Retain the old causal forecast as bounded feedback, without its latch.

    A linear continuation of raw norms is not a measured overload. The SAME
    measured warning/sustained bands and independent emergency owner remain.
    This adapter is installed only under the existing classified policy.
    """
    def __init__(self, runtime, original):
        self.runtime=runtime;self.original=original;self.last_event=-float('inf')

    def update(self,t,dt,pads,wrench,stopping_time_s):
        import numpy as np
        r=self.runtime
        base=np.asarray(getattr(r,'base_velocity',np.zeros(2)))
        acc=r.motion_limits(r.phase)['acceleration_m_s2']
        horizon=max(stopping_time_s,float(np.max(abs(base)))/acc)
        risk=self.original.update(t,dt,pads,wrench,horizon)
        scale=1.
        if risk is not None:
            projected=np.asarray(risk['projected_window_mean'])
            limits=np.asarray(risk['unchanged_sustained_limits'])
            scale=float(min(1.,np.min(limits/np.maximum(projected,limits))))
            if t-self.last_event>=1.:
                r.event('CLASSIFIED_PREDICTIVE_LOAD_ADVISORY',forecast=risk,
                    reference_scale=scale,authority='BOUNDED_REFERENCE_REGULATION',
                    action_stop_authorized=False,hard_limits_unchanged=True)
                self.last_event=t
        r.v87_predictive_feedback=dict(scale=scale,active=risk is not None,
            authority='CONTINUOUS_REFERENCE_REGULATION',forecast=risk)
        return None


def action_fault(r, error):
    """Predictive refusal/pad-report absence owns an action, not the World.

    Actual collision and hard measurements never match these exact codes.
    Returning a typed action error invokes the existing trusted stop/unload
    recovery. Failure there is still terminal and cannot automatically resume.
    """
    if (enabled(r) and actionable(r) and
            (error.reason == 'CONTACT_LOST_PAUSE_IN_SAME_SCENE'
             or error.reason == 'COMMAND_JOINT_MARGIN'
             or error.reason.startswith('COMMAND_COLLISION:'))):
        if (error.reason == 'CONTACT_LOST_PAUSE_IN_SAME_SCENE'
                and getattr(r, '_decelerating', False)):
            r.event('PAD_REPORT_ABSENT_DURING_GUARDED_BRAKE',
                authority='COMPLETE_EXISTING_BRAKE', original_reason=error.reason)
            return None
        r.event('CLASSIFIED_ACTION_BOUNDARY', original_reason=error.reason,
            authority='STOP_ACTION_AND_UNLOAD', hard_latch_cleared=False)
        return RuntimeError(PREFIX+error.reason)
    return error
