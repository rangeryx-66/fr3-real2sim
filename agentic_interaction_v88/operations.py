"""Replace fixed-base reachability proofs with explicit coupled proofs only."""
from contextlib import contextmanager
from unittest.mock import patch


@contextmanager
def planning_scope():
    import agentic_interaction_v87.prismatic_motion as rail
    import agentic_interaction_v81.revolute_certificate as door
    from .planning import evaluate
    with patch.object(rail, 'evaluate_prismatic_motion', evaluate), patch.object(door, 'evaluate', evaluate):
        yield


def operations_class(parent):
    class CoordinatedOperations(parent):
        def request_plan(self, command):
            with planning_scope(): return super().request_plan(command)

        def attempt(self, command):
            with planning_scope(): return super().attempt(command)

        def redeclare_actions(self, command):
            with planning_scope(): return super().redeclare_actions(command)

        def check_successor(self, action):
            from .planning import current
            from agentic_interaction_v81.replanning import reject
            proof = current(self, [action])
            if not proof['valid']: reject(self, proof, action)
            return proof

        def rotate_segment(self, *args, **kwargs):
            from .planning import current
            from agent_command_protocol_v12 import atomic_json
            action = kwargs.get('action') or (args[2] if len(args) > 2 else {})
            proof = current(self, [action])
            atomic_json(self.r.output / ('V88_MEASURED_COUPLED_PREFLIGHT_%02d.json' % len(self.r.record.segments)), proof)
            if not proof['valid']:
                self.r.stop_command()
                return dict(ok=False, status='SAFE_PARTIAL_PROGRESS', reason='CURRENT_COUPLED_ROUTE_UNAVAILABLE',
                    safely_stopped=True, physical_transmission_valid=False,
                    requested_target_reached=False, planning=proof, gain_native=0.,
                    joint_coordinate=self.r.actual())
            if self.articulation().joint_type != 'prismatic':
                return super().rotate_segment(*args, **kwargs)
            # Only two FIXED-base geometry preflights are replaced. The
            # original ABC monitor, startup/lead feedback, load forecast,
            # reference, centering, control, force and hard guards still run.
            import interaction_runtime.prismatic_v73 as legacy
            from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75 as Owner
            from interaction_runtime.prismatic_load_v75 import PrismaticLoadMarginStop
            def coupled_interval(r, articulation, target, action):
                return dict(valid=True, actual_start_q=float(r.actual()), feasible_target_q=target,
                    requested_target_q=target, stopping_distance_m=proof['endpoints'][0]['stopping_envelope_native'],
                    coupled_native_certificate=proof, fixed_base_proof_replaced=True)
            def segment(owner, *a, **kw):
                try: return super(Owner, owner).rotate_segment(*a, **kw)
                except PrismaticLoadMarginStop as error:
                    owner.r.stop_command(); owner.r.hold(.3)
                    result = dict(ok=False, status='SAFE_PARTIAL_PROGRESS', reason=error.args[0],
                        requested_target_reached=False, safely_stopped=True,
                        physical_transmission_valid=False, fit_prefix_transmission_valid=False,
                        interrupted_FIT_not_automatically_qualified=True, load_margin=error.evidence,
                        planning=dict(coordinate_unit='m', original_policy_unchanged=True),
                        joint_coordinate=owner.r.actual(), next='guarded release and fresh Agent decision')
                    owner.r.record.segments[-1]['result'] = result
                    return result
            with patch.object(legacy, 'reachable_interval', coupled_interval), patch.object(Owner, 'rotate_segment', segment):
                return super().rotate_segment(*args, **kwargs)
    return CoordinatedOperations
