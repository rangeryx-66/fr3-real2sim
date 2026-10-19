"""Measured rail loading/breakaway state, independent of hidden resistance.

This is an operation contract, not a force controller or a replacement safety
supervisor. Reference lead separately budgets measured servo deflection and
contact loading; actuator authority and original safety limits are unchanged.
No grasp/slip origin is reset at the loading-to-sliding transition.
"""
from collections import deque
import math
import numpy as np


class PrismaticStartup:
    def __init__(self,initial,quiet,action,motion,lookahead,dt):
        self.initial=initial;self.direction=1. if action['target_q']>=initial['q'] else -1.
        self.window=.5  # same measured-progress window as the existing detector
        self.velocity_resolution=1e-4  # existing estimator stiction resolution, m/s
        self.distance_resolution=self.velocity_resolution*self.window
        self.lookahead=float(lookahead);self.dt=float(dt)
        self.preload_speed=min(.001,float(action.get('reference_speed_m_s',motion['speed_m_s'])),motion['speed_m_s'])
        # Finite contact + capped measured servo travel/rate budget, two observed settling windows and initial
        # command/measurement mismatch time. No arbitrary global timeout change.
        self.tracking_allowance=self.lookahead
        self.max_loading_s=(self.lookahead+self.tracking_allowance+abs(initial['command_rail']-initial['tcp_rail']))/self.preload_speed+2*self.window
        self.maximum_tracking_offset=0.
        quiet=list(quiet) or [initial]
        self.force_noise=max(1e-9,5*self.mad([r['rail_effort'] for r in quiet]))
        self.q_noise=max(self.distance_resolution,5*self.mad([r['q'] for r in quiet]))
        self.velocity_noise=max(self.velocity_resolution,5*self.mad([r['dq'] for r in quiet]))
        self.tracking_resolution=4*self.q_noise
        self.phase='CONTACT_PRELOAD';self.classification='COMPLIANT_LOADING_UNCONFIRMED'
        self.samples=deque();self.history=[];self.last=None;self.sliding_started_s=None;self.bad_windows=0;self.stop_reason=None;self.first_suspect_s=None;self.observed_mechanisms=set()

    @staticmethod
    def mad(values):
        x=np.asarray(values,float);return float(1.4826*np.median(abs(x-np.median(x))))

    @property
    def sliding(self):return self.phase=='OPEN_CONTINUE'

    def clock_feedback(self,sample):
        """Separate finite servo deflection from actual contact-relative lead.

        The old rail bound spent the same allowance on BOTH servo tracking and
        contact loading. Compensate only measured incremental command-to-TCP
        deficit, capped by one existing stopping-distance allowance. Thus the
        total reference/object lead remains <= two allowances; actual progress,
        grasp origin, slip/force/joint guards and completion tests are untouched.
        The progress clock still bounds command speed/acceleration continuously.
        """
        deficit=self.direction*((sample['command_rail']-self.initial['command_rail'])-(sample['tcp_rail']-self.initial['tcp_rail']))
        offset=float(np.clip(deficit,0.,self.tracking_allowance))
        self.maximum_tracking_offset=max(self.maximum_tracking_offset,offset)
        return float(sample['q']+self.direction*offset),dict(measured_q_m=sample['q'],incremental_servo_deficit_m=deficit,tracking_offset_m=offset,total_reference_lead_bound_m=self.lookahead+self.tracking_allowance)

    def observe(self,sample):
        self.samples.append(sample)
        if sample['t']-self.samples[0]['t']<self.window-self.dt/2:return None
        a=self.samples[0];b=sample;values=list(self.samples);duration=b['t']-a['t']
        progress=self.direction*(b['q']-a['q']);tcp=self.direction*(b['tcp_rail']-a['tcp_rail']);command=self.direction*(b['command_rail']-a['command_rail'])
        track=abs((b['command_rail']-self.initial['command_rail'])-(b['tcp_rail']-self.initial['tcp_rail']))
        relative=tcp-progress
        effort=float(np.median([self.direction*r['rail_effort'] for r in values]))
        force_change=effort-self.direction*self.initial['rail_effort']
        moving=progress>=self.q_noise and np.median([self.direction*r['dq'] for r in values])>=self.velocity_noise
        ratio=progress/max(tcp,self.q_noise)
        friction=[r.get('contact',{}) for r in values]
        native_traction=[self.direction*r['traction_on_object_rail_N'] for r in friction if r.get('traction_available')]
        slip_speeds=[r['relative_tangent_speed_m_s'] for r in friction if r.get('slip_velocity_available')]
        # Surface-relative velocity is a kinematic diagnostic, not proof inferred
        # from one unloaded pad or a migrating contact-point centroid alone.
        slip_observed=bool(slip_speeds and np.median(slip_speeds)>self.velocity_noise and relative>self.q_noise)
        if slip_observed:self.observed_mechanisms.add('KINEMATIC_TANGENTIAL_SLIP_EVIDENCE')
        if track>self.tracking_resolution:self.observed_mechanisms.add('COMMAND_TO_TCP_TRACKING_DEFICIT')
        settled_tracking=track<=self.tracking_resolution or abs(command-tcp)<=self.q_noise
        window=dict(t_s=b['t'],duration_s=duration,drawer_progress_m=progress,measured_TCP_travel_m=tcp,command_TCP_travel_m=command,
            incremental_tracking_error_m=track,TCP_drawer_relative_travel_m=relative,rail_effort_N=effort,effort_change_N=force_change,
            observed_native_traction_N=float(np.median(native_traction)) if native_traction else None,
            observed_tangential_relative_speed_m_s=float(np.median(slip_speeds)) if slip_speeds else None,
            motion_resolved=bool(moving),tracking_settled=bool(settled_tracking),transmission_ratio_actual_TCP=ratio,
            phase=self.phase,classification=self.classification)
        if not self.sliding:
            if moving and ratio>=.1:
                self.phase='OPEN_CONTINUE';self.sliding_started_s=b['t'];self.classification='BREAKAWAY_CONFIRMED'
            elif slip_observed and settled_tracking and not moving:
                self.classification='KINEMATIC_TANGENTIAL_SLIP_EVIDENCE'
            elif track>self.tracking_resolution:
                self.classification='COMMAND_TO_TCP_TRACKING_LIMITED'
            elif force_change>self.force_noise:
                self.classification='COMPLIANT_LOADING_WITH_RISING_REACTION'
            elif relative>self.q_noise:
                self.classification='INEFFECTIVE_TRACTION_UNRESOLVED'
            else:self.classification='CONTACT_LOADING_PENDING'
            # Allow feedback-driven loading within the SAME finite reference
            # allowance, including a settled dwell. No unlimited reference pull.
            if b['t']>=self.max_loading_s:
                if moving:self.classification='INCIPIENT_MOTION_NOT_YET_SUSTAINED'
                elif 'KINEMATIC_TANGENTIAL_SLIP_EVIDENCE' in self.observed_mechanisms:self.classification='KINEMATIC_TANGENTIAL_SLIP_EVIDENCE'
                elif track<=self.tracking_resolution and settled_tracking and force_change>self.force_noise:
                    self.classification='STATIC_RESISTANCE_OR_CONSTRAINT_UNRESOLVED'
                self.stop_reason='PRISMATIC_STARTUP_'+self.classification
        else:
            # Actual TCP demand, not virtual lead, separates poor transmission
            # from a stationary reference/servo still catching up.
            bad=progress<self.q_noise and (relative>self.q_noise or settled_tracking)
            self.bad_windows=self.bad_windows+1 if bad else 0
            if bad and self.bad_windows==1:self.first_suspect_s=b['t']-self.window
            if not bad:self.first_suspect_s=None
            self.classification='KINEMATIC_TANGENTIAL_SLIP_EVIDENCE' if slip_observed else ('PERSISTENT_SLIDING_STALL' if bad else 'SLIDING_TRANSMISSION')
            if self.bad_windows>=3:self.stop_reason='PRISMATIC_'+self.classification
        window.update(classification=self.classification,next_phase=self.phase,stop_reason=self.stop_reason)
        self.history.append(window);self.last=window;self.samples.clear();self.samples.append(sample)
        return window

    def packet(self):
        return dict(phase=self.phase,classification=self.classification,sliding_started_s=self.sliding_started_s,max_loading_s=self.max_loading_s,
            preload_speed_m_s=self.preload_speed,contact_reference_allowance_m=self.lookahead,
            servo_tracking_allowance_m=self.tracking_allowance,total_reference_lead_bound_m=self.lookahead+self.tracking_allowance,maximum_measured_tracking_offset_m=self.maximum_tracking_offset,
            thresholds=dict(motion_resolution_m=self.q_noise,velocity_resolution_m_s=self.velocity_noise,force_change_resolution_N=self.force_noise,tracking_resolution_m=self.tracking_resolution),
            observed_windows=self.history,observed_mechanisms=sorted(self.observed_mechanisms),stop_reason=self.stop_reason,first_suspect_s=self.first_suspect_s,hidden_resistance_used=False,grasp_reference_reset=False,
            force_measurement='bias/gravity-compensated wrist wrench; dynamic tool loading retained for conservative live trend; completed-segment estimator compensates it once',
            safety='original independent force/collision/joint/actuator/velocity supervisor remains authoritative every physics step')
