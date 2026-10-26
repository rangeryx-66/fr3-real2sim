"""Fault injection through the actual supervisor and recovery classification seams."""
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
from interaction_runtime.policy import Policy, SafetySupervisor, SafetyStop
from agentic_interaction_v87.constraint_feedback import (
    POLICY, ActionForceMonitor, motion, require_recovery, action_fault)
from agentic_interaction_v87.activation import activate

ROOT = Path(__file__).resolve().parents[1]


def fixture(audited=True):
    policy = Policy.load(ROOT/'configs/interaction_runtime_v1.json')
    r = NS(job={'v87_constraint_policy': POLICY} if audited else {},
        policy=policy, stopped=False, rows=[{'wrist_ft': {'valid': True, 'raw_wrench_sensor': [0.]*6}}],
        qtarget=np.zeros(7), arm=np.arange(7), model=NS(margin=lambda q:1.),
        filtered=lambda:np.zeros(2), actual=lambda: .7, g={'phase': 'OPEN_CONTINUE'},
        robot=NS(get_joint_positions=lambda: np.zeros(7),
                 get_joint_velocities=lambda: np.zeros(7)), event=Mock(),
        _decelerating=False)
    r.supervisor = SafetySupervisor(policy, [(-3., 3.)]*7, np.ones(7)*3, np.ones(7)*10)
    if audited: r.supervisor.force = ActionForceMonitor(policy, r)
    return r


def step(r, t, torque=.0, pad=.1, force=0., **changes):
    fields = dict(phase=r.g['phase'], q=np.zeros(7), dq=np.zeros(7),
        tcp_speed=.012, pads={'left': pad, 'right': pad}, dt=1/240, t=t,
        wrench=[force, 0., 0., 0., 0., torque])
    fields.update(changes)
    return r.supervisor.check(**fields)


class ClassifiedConstraintTests(unittest.TestCase):
    def test_recorded_torque_mean_owns_action_not_latch(self):
        # Exact published trailing mean, not a fabricated new physical run.
        for audited in (False, True):
            r = fixture(audited)
            if not audited:
                with self.assertRaisesRegex(SafetyStop, 'SUSTAINED_OVERLOAD'):
                    for i in range(36): step(r, i/240, torque=.6001997743592373)
                self.assertIsNotNone(r.supervisor.latched)
            else:
                for i in range(36): out = step(r, i/240, torque=.6001997743592373)
                self.assertEqual(out['status'], 'CONTROLLED_LOAD_UNLOAD_REQUIRED')
                self.assertIsNone(r.supervisor.latched)
                with self.assertRaisesRegex(RuntimeError, 'REQUIRES_UNLOAD'): require_recovery(r)
                r._decelerating=True
                require_recovery(r)  # only the existing stop owner can brake
                r._decelerating=False
                r.g['phase']='RELEASE'
                require_recovery(r)
                for i in range(36, 110): step(r, i/240, torque=0., pad=0.)
                self.assertFalse(r.supervisor.force.unload_required)

    def test_warning_noise_does_not_interrupt_long_B(self):
        r=fixture()
        for i in range(12000):
            step(r, i/240, torque=.30001 + .00001*np.sin(i))
            require_recovery(r)
        self.assertIsNone(r.supervisor.latched)
        self.assertGreater(r.supervisor.force.feedback['scale'], .99)

    def test_load_regulation_uses_all_original_bands(self):
        for key, value in [('torque', .45), ('pad', 3.), ('force', 6.)]:
            with self.subTest(key=key):
                r=fixture()
                for i in range(36): step(r, i/240, **{key: value})
                out=motion(r,r.policy.motion('LOADED'))
                self.assertAlmostEqual(out['speed_m_s'], .006)
                self.assertEqual(out['acceleration_m_s2'], .04)
                require_recovery(r)

    def test_speed_90_percent_is_continuously_regulated(self):
        r=fixture();r.rows[-1]['tcp_speed_m_s']=.027001
        out=motion(r,r.policy.motion('LOADED'))
        self.assertGreater(out['speed_m_s'],0.)
        self.assertLess(out['speed_m_s'],.012)
        require_recovery(r)
        with self.assertRaisesRegex(SafetyStop,'PHASE_TCP_VELOCITY'):
            step(r,0.,tcp_speed=.03212595152927349)

    def test_all_emergencies_and_actual_hazards_keep_authority(self):
        cases=[({'torque':1.},'EMERGENCY_WRIST_TORQUE'),
            ({'pad':10.},'EMERGENCY_PAD_OVERLOAD'),
            ({'force':10.},'EMERGENCY_WRIST_FORCE'),
            ({'collision_ok':False,'collision_reason':'cabinet'},'COLLISION'),
            ({'q':np.ones(7)*3.},'JOINT_MARGIN'),
            ({'dq':np.ones(7)*1.001},'JOINT_VELOCITY'),
            ({'actuator_efforts':np.ones(7)*11},'ACTUATOR_EFFORT'),
            ({'q':np.ones(7)*np.nan},'INVALID_ROBOT_STATE'),
            ({'wrench':[0,0,0,0,0,float('nan')]},'INVALID_WRIST_WRENCH')]
        for changes, reason in cases:
            with self.subTest(reason=reason):
                r=fixture()
                with self.assertRaisesRegex(SafetyStop,reason): step(r,0.,**changes)

    def test_emergency_during_unload_still_latches(self):
        r=fixture()
        for i in range(36):step(r,i/240,torque=.61)
        r.g['phase']='RELEASE'
        with self.assertRaisesRegex(SafetyStop,'EMERGENCY_WRIST_TORQUE'):
            step(r,.2,torque=1.)
        latch=r.supervisor.latched
        with self.assertRaisesRegex(SafetyStop,'LATCHED_'):step(r,.3)
        self.assertIs(r.supervisor.latched,latch)

    def test_untrusted_or_unregistered_phase_cannot_downgrade(self):
        for phase,valid in [('PREGRASP',True),('OPEN_CONTINUE',False)]:
            r=fixture();r.g['phase']=phase;r.rows[-1]['wrist_ft']['valid']=valid
            with self.assertRaisesRegex(SafetyStop,'SUSTAINED_OVERLOAD'):
                for i in range(36):step(r,i/240,torque=.61)

    def test_predictive_refusal_and_pad_absence_return_action_boundary(self):
        for reason in ['COMMAND_COLLISION:predicted','COMMAND_JOINT_MARGIN',
                       'CONTACT_LOST_PAUSE_IN_SAME_SCENE']:
            r=fixture();error=SafetyStop(reason)
            out=action_fault(r,error)
            self.assertIsInstance(out,RuntimeError)
            self.assertNotIsInstance(out,SafetyStop)
            self.assertIsNone(r.supervisor.latched)
        for reason in ['COLLISION:measured','JOINT_MARGIN','STOP_ROUTE_COLLISION:x',
                       'CONTROLLED_STOP_NOT_VERIFIED','INVALID_COMMAND']:
            r=fixture();error=SafetyStop(reason)
            self.assertIs(action_fault(r,error),error)

    def test_fault_mapping_never_clears_existing_latch(self):
        r=fixture();latch={'reason':'COLLISION'};r.supervisor.latched=latch
        error=SafetyStop('COMMAND_COLLISION:predicted')
        self.assertIs(action_fault(r,error),error)
        self.assertIs(r.supervisor.latched,latch)

    def test_explicit_registration_and_staged_recovery_required(self):
        for phase,staged in [('independent_repetition',True),('engineering',False)]:
            with self.assertRaisesRegex(ValueError,'CLASSIFIED_CONSTRAINT'):
                activate({'constraint_policy':POLICY,'phase':phase,'staged_exit':staged},
                         {},lambda *a:None,'base')
        job={}
        activate({'constraint_policy':POLICY,'phase':'engineering','staged_exit':True},
                 job,lambda *a:None,'base')
        self.assertEqual(job['v87_constraint_policy'],POLICY)


class BrakingAuthorityTests(unittest.TestCase):
    def test_absent_pad_report_cannot_abort_existing_brake(self):
        r=fixture();r._decelerating=True
        self.assertIsNone(action_fault(r,SafetyStop('CONTACT_LOST_PAUSE_IN_SAME_SCENE')))
        error=SafetyStop('COLLISION:measured')
        self.assertIs(action_fault(r,error),error)

    def test_feedback_reference_can_brake_to_zero_without_division_or_snap(self):
        from interaction_runtime.motion import CartesianReference
        r=fixture();reference=CartesianReference(np.eye(4),np.r_[.012,0,0,0,0,0])
        goal=np.eye(4);goal[0,3]=1.
        r.supervisor.force.feedback['scale']=0.
        limits=motion(r,r.policy.motion('LOADED'))
        last=.012
        for _ in range(100):
            reference.advance(goal,1/240,limits)
            speed=float(np.linalg.norm(reference.velocity[:3]))
            self.assertLessEqual(abs(speed-last),.04/240+1e-12)
            last=speed
        self.assertAlmostEqual(last,0.)

    def test_constructor_instrumentation_preserves_force_history(self):
        from interaction_runtime.policy import ForceMonitor
        r=fixture();old=ForceMonitor(r.policy)
        for i in range(35):old.update({'left':.1},1/240,i/240,[0,0,0,0,0,.61])
        new=ActionForceMonitor(r.policy,r,previous=old)
        r.supervisor.force=new
        out=new.update({'left':.1},1/240,35/240,[0,0,0,0,0,.61])
        self.assertEqual(out['status'],'CONTROLLED_LOAD_UNLOAD_REQUIRED')
        self.assertIs(new.history,old.history)

    def test_untrusted_command_margin_cannot_claim_recovery_qualification(self):
        r=fixture();r.model.margin=lambda q:0.
        error=SafetyStop('COMMAND_JOINT_MARGIN')
        self.assertIs(action_fault(r,error),error)


class LoadedCDispatchTests(unittest.TestCase):
    def test_controlled_load_or_resource_end_in_C_reuses_guarded_exit(self):
        import tempfile
        from test_agentic_interaction_v84 import ContactTests
        from agentic_interaction_v84.session import loaded_command
        from interaction_runtime.segments import ExecutionBudgetExhausted
        for error in [RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:PROVISIONAL_LOAD'),
                      ExecutionBudgetExhausted('EXIT_RESERVE_REACHED')]:
            with self.subTest(error=str(error)), tempfile.TemporaryDirectory() as folder:
                ops=ContactTests().fixture();r=ops.r;r.output=Path(folder);r.tick=0;r.dt=1/240
                r.job={'episode_id':'fault_C','v87_constraint_policy':POLICY,'safe_exit_wall_reserve_s':300.}
                r.base_generation=0;r.g={'observation_generations':[]}
                r.episode_budget=NS(remaining=lambda t:{'wall_s':100.,'sim_s':100.})
                r.stopped=False;r.hold=Mock(side_effect=error);ops.packet=lambda:{'state':'C'}
                self.assertIsNone(loaded_command(ops,4))
                self.assertTrue(ops.called)
                self.assertIsNone(r.supervisor.latched)
                self.assertTrue((r.output/'ABC_C_CONTROLLED_RECOVERY_004.json').exists())
