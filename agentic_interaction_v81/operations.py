"""High-level station extension; frozen observation/contact operations retained."""
import json
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import patch


def operations_class(original):
    class StationOperations(original):
        worker_script='agentic_interaction_v81/update_model.py'

        def decision_packet(self,sequence,result=None):
            parent=getattr(super(),'decision_packet',None)
            if parent is None:
                from .decision_packet import revolute_packet
                packet=revolute_packet(self,sequence,result)
            else:packet=parent(sequence,result)
            from .replanning import packet as recovery_packet
            packet['planning_recovery']=recovery_packet(self)
            if self.r.job.get('v81_integrated_repair'):
                from .integration import packet as integrated_packet
                packet['integrated_task']=integrated_packet(self)
                if self.r.job['articulation_v7']['joint_type']=='prismatic':
                    packet['legal_preloads_n']=packet['integrated_task']['preload_menu']['requests_n']
            if self.r.job['articulation_v7']['joint_type']!='prismatic':return packet
            from .recovery_search import current_packet
            from .recovery_batch import measured_costs
            packet['measured_recovery_costs']=measured_costs(self)
            return current_packet(self,packet)

        def validate_execution(self):
            from .execution_return import direct_return
            with direct_return(self.r):return super().validate_execution()

        def prepare_grasp(self,command):
            from .boundary_operations import prepare_grasp
            return prepare_grasp(self,command)

        def observe_view(self,command):
            from .boundary_operations import observe_view
            return observe_view(self,command)

        def release_park(self):
            import time
            started=time.perf_counter()
            # Frozen wrist observation deliberately pauses ONLY at an unloaded
            # decision boundary. Its release owner steps directly before hold(),
            # whereas the old session explicitly played before calling it.
            # Restore that execution precondition here; never clear a hard stop
            # or substitute a synthetic dt for the native contact clock.
            from .physics_lifecycle import resume_for_exit
            resume_for_exit(self.r)
            from .exit_routes import release_park
            result=release_park(self.r,super().release_park)
            if self.r.job.get('v81_integrated_repair'):
                from .costs import record
                record(self,'release',started)
            return result

        def update_async(self,ready):
            if not self.r.job.get('v81_integrated_repair'):return super().update_async(ready)
            from .model_packets import submit
            return submit(self,ready)

        def acquire(self):
            import time
            started=time.perf_counter()
            result=super().acquire()
            self._v81_current_acquisition_verified=bool(result.get('acquired'))
            if self.r.job.get('v81_integrated_repair'):
                from .costs import record
                record(self,'acquire',started)
            return result

        def interact(self,target,max_sim_s=None,action=None):
            proposed=self.actions[0] if self.actions else action
            integrated=self.r.job.get('v81_integrated_repair')
            if integrated and getattr(self,'_v81_action_index',0)==0:
                self._v81_first_route_check=self.check_successor(proposed)
                if not self._v81_first_route_check.get('valid'):
                    if self.actions:self.actions.pop(0)
                    return dict(ok=False,status='PLANNING_REJECTED',
                        reason=self._v81_first_route_check.get('reason'),
                        native_check=self._v81_first_route_check,physical_motion_executed=False,
                        requested_target_reached=False,safely_stopped=True,
                        physical_transmission_valid=False)
            self._v81_action_index=getattr(self,'_v81_action_index',0)+1
            if proposed and proposed['role'].startswith('VALIDATION'):
                if integrated:
                    from .validation_queue import admit
                    admission=admit(self,proposed)
                    if not admission['ready']:
                        if self.actions:self.actions.pop(0)
                        return dict(ok=False,status='VALIDATION_DEFERRED',executed=False,
                            reason=admission['reason'],pending_request_preserved=True,
                            validation_admission=admission,requested_target_reached=False,
                            complete_independent_validation=False)
                import hashlib
                folder=self.r.output/'model';m=folder/'estimate.json';f=folder/'FIT_READINESS_V81.json'
                ready=json.loads(f.read_text()) if f.exists() else {}
                model_bytes=m.read_bytes() if m.exists() else None
                # Freshness covers every finished FIT action record, including
                # a safely interrupted one. Eligibility remains the frozen
                # production qualification, never the action's role/status.
                expected=max((s['id'] for s in self.r.record.segments if s['role'].startswith('FIT')),default=0)
                valid=(model_bytes is not None and ready.get('model_sha256')==hashlib.sha256(model_bytes).hexdigest()
                       and expected>0 and ready.get('episode_id')==self.r.job['episode_id']
                       and ready.get('latest_completed_FIT_id')==expected and ready.get('qualified_bidirectional')
                       and ready.get('positive_bins',0)>=8 and ready.get('negative_bins',0)>=8
                       and ready.get('distinct_eligible_trajectories',0)>=2)
                if not valid:
                    # No waiting for workers or Agent inference while loaded.
                    # End this finite block and use the original release/park.
                    if self.actions:self.actions.pop(0)
                    return dict(ok=False,status='VALIDATION_DEFERRED',executed=False,
                        reason='QUALIFIED_BIDIRECTIONAL_FIT_NOT_YET_READY',readiness=ready,
                        requested_target_reached=False,complete_independent_validation=False)
                model=json.loads(model_bytes);seal=self.r.output/'validation_model_seals'
                seal.mkdir(exist_ok=True)
                (seal/('%02d.json'%(len(self.r.record.segments)+1))).write_text(json.dumps(dict(
                    declared_action=proposed,model_sha256=ready['model_sha256'],
                    selected=model.get('selected'),configuration=model.get('configuration'),
                    readiness=ready,object_coordinate=self.r.actual()),indent=2))
            import time
            started=time.perf_counter()
            result=super().interact(target,max_sim_s,action)
            if integrated and result.get('status')=='TARGET_REACHED' and result.get('physical_transmission_valid'):
                from .costs import record
                record(self,'healthy_action',started,result.get('gain_native'))
            if integrated and proposed and proposed['role'].startswith('VALIDATION'):
                from .validation_queue import mark
                mark(self,proposed,'COMPLETE' if result.get('status')=='TARGET_REACHED'
                    and result.get('requested_target_reached') else 'PHYSICAL_FAILURE',result=result)
            return result

        def feedback(self):
            if self.r.job['articulation_v7']['joint_type']=='revolute' and self.g['contact_attempts']:
                from functools import partial
                import render_contact_feedback_v7 as display
                from .feedback import render
                with patch.object(display,'render',partial(render,attempt=self.g['contact_attempts'][-1],
                    attempt_id=len(self.g['contact_attempts']))):
                    result=super().feedback()
            else:result=super().feedback()
            p=self.r.output/'model/FIT_READINESS_V81.json'
            result['FIT_readiness_v81']=json.loads(p.read_text()) if p.exists() else dict(status='pending')
            return result

        def attempt(self,command):
            from copy import deepcopy
            from .replanning import admission,coverage,state
            command=deepcopy(command)  # replacements never erase the original Agent request
            if not self.current_plan_ready:
                return dict(executed=False,status='PLANNING_REJECTED',recoverable=True,
                    reason='CURRENT_FULL_TARGET_CHECKED_PLAN_REQUIRED')
            candidate=json.loads((self.r.output/'visual_plan.json').read_text())
            from .replanning import targets
            checked=coverage(candidate,targets(command.get('segments',[])))
            if not checked['full_requested_actions_feasible']:
                return dict(executed=False,status='PLANNING_REJECTED',recoverable=True,
                    reason='REQUESTED_TARGET_NOT_FULLY_CHECKED',coverage=checked)
            resources=admission(self,command.get('segments'))
            if not resources['admitted']:
                return dict(executed=False,status='RECOVERY_RESERVE_REQUIRED',recoverable=True,
                    reason='OBSERVED_ATTEMPT_AND_RECOVERY_COST_EXCEEDS_WORK_BUDGET',resources=resources)
            if command.get('contact_following',getattr(self.r,'v81_contact_following_mode','legacy_window'))!=getattr(self.r,'v81_contact_following_mode','legacy_window'):
                return dict(executed=False,recoverable=True,reason='REPLAN_CHANGED_CONTACT_FOLLOWING_MODE')
            if command.get('segments')!=getattr(self,'_v81_declared_chain',None):
                return dict(executed=False,recoverable=True,reason='DECLARE_AND_VALIDATE_THIS_CHAIN_BEFORE_ACQUISITION')
            generation=len(self.g['observation_generations'])
            integrated=self.r.job.get('v81_integrated_repair')
            if integrated:
                from .validation_queue import enqueue
                enqueue(self,command)
                self._v81_action_index=0;self._v81_first_route_check=None
                self._v81_block_first_record=self.r.record.count
                self._v81_current_acquisition_verified=False
            from .execution_return import direct_return
            with direct_return(self.r):result=super().attempt(command)
            result['observation_generation_v81']=generation
            self._last_contact_finished_world_s=float(self.r.world.current_time)
            result['station_generation']=self.r.base_generation
            result['contact_following_v81']=getattr(self.r,'v81_contact_following_mode','legacy_window')
            result['planning_recovery']=dict(state(self))
            rejected=(bool(getattr(self,'_v81_first_route_check',None))
                and not self._v81_first_route_check['valid']) or any(not c.get('valid') for c in result.get('continuations',[]))
            if rejected and self.r.supervisor.latched is None:
                safe=bool(result.get('safe_exit',{}).get('safe'))
                state(self)['state']='UNLOADED_REPLAN_REQUIRED' if safe else 'SAFE_EXIT_UNAVAILABLE'
                result.update(status='PLANNING_REJECTED',planning_status='PLANNING_REJECTED',recoverable=safe,
                    episode_termination_required=not safe)
                # Fresh post-contact observation may be collected automatically
                # at the verified unloaded boundary, never reasoning under load.
                costs=resources.get('measured_current_attempt_plus_one_recovery_wall_s')
                if integrated:costs=resources.get('recovery_reserve_wall_s')
                if safe and costs is not None:
                    remaining=self.r.episode_budget.remaining(self.r.tick*self.r.dt)
                    if remaining['wall_s']>=costs:
                        result['fresh_replanning_observation']=self.reobserve()
                        state(self)['state']='FRESH_EVIDENCE_REQUIRES_AGENT_INSPECTION'
            result['planning_recovery']=dict(state(self))
            if integrated:
                from .validation_queue import boundary
                result['pending_validation']=boundary(self)
                result['first_action_measured_native_check']=self._v81_first_route_check
            return result

        def check_successor(self,action):
            from .replanning import successor
            return successor(self,action,super().check_successor)

        def request_plan(self,command):
            from .declaration_preflight import check
            from .contact_following import selection
            actions=command.get('planned_segments')
            try:
                check(actions,self.articulation())
                following=selection(command,self.r.job['articulation_v7']['joint_type'])
                from .recovery_search import check as search_check
                search_round=search_check(self,command)
            except (ValueError,TypeError,KeyError) as error:
                return dict(plan=False,recoverable=True,
                    reason='DECLARATION_PREFLIGHT:'+str(error),
                    robot_action_executed=False,expensive_planning_started=False)
            from .replanning import admission
            resources=admission(self,actions)
            if not resources['admitted']:
                return dict(plan=False,recoverable=True,status='RECOVERY_RESERVE_REQUIRED',
                    reason='OBSERVED_ATTEMPT_AND_RECOVERY_COST_EXCEEDS_WORK_BUDGET',
                    resources=resources,expensive_planning_started=False)
            association=None
            if command.get('start_new_target_association',False):
                from .target_association import start_latest
                try:association=start_latest(self,command)
                except (ValueError,KeyError,FileNotFoundError) as error:
                    return dict(plan=False,recoverable=True,reason='TARGET_ASSOCIATION:'+str(error),
                        robot_action_executed=False,expensive_planning_started=False)
            if command.get('exploratory_task_reconsideration') and self.r.job['articulation_v7']['joint_type']!='prismatic':
                return dict(plan=False,reason='TASK_RECONSIDERATION_IS_PRISMATIC_ONLY',recoverable=True)
            self._v81_declared_chain=actions
            if self.r.job['articulation_v7']['joint_type']=='prismatic':
                from .recovery_search import retire_plan
                retire_plan(self)
            # A short first probe need not hide a later workspace limitation.
            # The Agent may request a separate known-kinematics planning horizon;
            # it is never an extra executable action or a physical success claim.
            horizon=float(command.get('planning_target_q',actions[0]['target_q']))
            self.articulation().validate_target(dict(target_q=horizon,
                coordinate_unit=actions[0]['coordinate_unit']))
            self.r.job['benchmark_goal_rad']=horizon
            import native_g1_execution_candidates as executable
            from interactive_twin_recovery.mobile import at_base
            original_export=executable.current_export
            def current(export,moving_link,moving_pose):
                # Planning always starts from the frozen native export. Only
                # the original support's measured station changes; the object
                # transform is the original current-state safety update.
                moved=at_base(export,self.r.initial_base,self.r.base)
                return original_export(moved,moving_link,moving_pose)
            from .performance import cached_planning_fk
            memory_scope=nullcontext()
            if self.r.job['articulation_v7']['joint_type']=='prismatic':
                from .recovery_memory import task_memory
                memory_scope=task_memory(self,command)
            from .recovery_search import scope as search_scope,complete as search_complete,skip_dominated_checks
            prismatic=self.r.job['articulation_v7']['joint_type']=='prismatic'
            search_context=search_scope(self,search_round) if prismatic else nullcontext()
            prune_context=skip_dominated_checks() if prismatic else nullcontext()
            from .replanning import candidate_gate
            collected=[];integrated=self.r.job.get('v81_integrated_repair')
            if prismatic and integrated:
                from .preload_extension import menu_scope
                menu=menu_scope(self)
            else:menu=nullcontext()
            with patch.object(executable,'current_export',current), cached_planning_fk(), memory_scope, search_context, prune_context, candidate_gate(actions,horizon,collected if integrated else None) as coverage_audit, menu:
                if command.get('reuse_current_candidates'):
                    if not integrated:return dict(plan=False,recoverable=True,reason='INTEGRATED_CANDIDATE_REPLAN_NOT_ACTIVE')
                    from .candidate_pool import select
                    result=select(self,command)
                else:result=super().request_plan(command)
            if integrated and collected and not command.get('reuse_current_candidates'):
                from .candidate_pool import keep
                keep(self,collected,horizon)
            if hasattr(self.r,'output'):
                from agent_command_protocol_v12 import atomic_json
                atomic_json(self.r.output/'TARGET_COVERAGE_CANDIDATE_AUDIT.json',dict(candidates=coverage_audit,
                    scores_unchanged=True,native_rejection_authoritative=True))
            if self.r.job['articulation_v7']['joint_type']=='prismatic':search_complete(self,search_round,result)
            result.update(planning_horizon_native=horizon,
                executable_first_target_native=actions[0]['target_q'],
                planning_horizon_is_not_motion=True)
            if result.get('plan') and hasattr(self.r,'output'):
                from .replanning import coverage,targets
                bounds=coverage(json.loads((self.r.output/'visual_plan.json').read_text()),targets(actions),horizon)
                result['target_coverage']=bounds
                if integrated:
                    final_goal=self.r.job['v81_task_goal_native']
                    result['original_final_goal_coverage']=coverage(
                        json.loads((self.r.output/'visual_plan.json').read_text()),[final_goal],final_goal)
                if not bounds['full_requested_actions_feasible']:
                    self.current_plan_ready=False
                    result.update(plan=False,status='PLANNING_REJECTED',recoverable=True,
                        reason='REQUESTED_TARGET_NOT_FULLY_CHECKED',episode_termination_required=False)
            if not result.get('plan'):
                from .replanning import state
                state(self)['state']='UNLOADED_REPLAN_REQUIRED'
                result.update(status='PLANNING_REJECTED',recoverable=True,episode_termination_required=False)
            if result.get('plan') and hasattr(self.r,'output'):
                from .action_declaration import ticket
                self._v81_plan_ticket=ticket(self)
            else:self._v81_plan_ticket=None
            if result.get('plan'):
                self.r.v81_contact_following_mode=following
                result['contact_following_v81']=following
            if association is not None:result['target_association']=association
            return result

        def redeclare_actions(self,command):
            from .action_declaration import redeclare
            return redeclare(self,command)

        def evaluate_station(self,command):
            if not command.get('reason') or not command.get('inspected_image_sha256'):
                return dict(valid=False,reason='FRESH_IMAGE_PROVENANCE_AND_AGENT_REASON_REQUIRED')
            import hashlib
            observations=self.g['agent_observations']
            if not observations or observations[-1]['world_sim_s']<getattr(self,'_last_contact_finished_world_s',0):
                return dict(valid=False,reason='FRESH_WRIST_OBSERVATION_AFTER_CONTACT_REQUIRED')
            folder=Path(observations[-1]['folder'])
            images=[folder/name for name in ['agent_inspection.png','rgb.png','selected_handle_overlay.png']]
            hashes={hashlib.sha256(p.read_bytes()).hexdigest() for p in images if p.is_file()}
            if command['inspected_image_sha256'] not in hashes:
                return dict(valid=False,reason='STALE_OR_NONWRIST_INSPECTED_IMAGE')
            target=command['target_xy_m']
            if len(target)!=2:raise ValueError('XY_TARGET_ONLY')
            result=self.r.evaluate_station(target)
            if result.get('valid') and 'task_horizon_q' in command:
                from .station_forecast import forecast
                result['task_forecast']=forecast(self,target,float(command['task_horizon_q']))
            result['Agent_reason']=command['reason']
            result['inspected_image_sha256']=command['inspected_image_sha256']
            return result

        def relocate_station(self,evaluated):
            if not evaluated or not evaluated.get('valid'):
                return dict(executed=False,reason='NO_VALIDATED_STATION_REQUEST')
            before=self.r.station_boundary()
            old_park=self.g.get('last_exit_plan')
            result=self.r.move_station(evaluated['target'][:2])
            if not result.get('reached'):return result
            self.current_plan_ready=False;self.cache.clear();self.panel_plan=None
            g=self.g;r=self.r
            # Preserve old observations, geometry and attempts as history, not
            # executable input. Always collect fresh images at the new station.
            g['observation_generations'].append(g['agent_observations'])
            g['agent_observations']=[];g['wrist_routes']=[];g['agent_moves']=0
            g.pop('recovery_candidate_catalog',None)
            old=Path(r.job['source'])/'report.json'
            source=json.loads(old.read_text());source['robot_base_pose']=r.base.tolist()
            location=r.output/'station_states'/('%02d'%r.base_generation)
            location.mkdir(parents=True,exist_ok=True)
            for name in ['visual_plan.json','AGENT_EXECUTION_TRANSITION_CHECK.json','MEASURED_STATE_APPROACH_CHECK.json']:
                path=r.output/name
                if path.exists():path.rename(location/('prior_station_'+name))
            for key in ['chosen','whole','T','visual_initial_folder']:
                g.pop(key,None)
            (location/'report.json').write_text(json.dumps(source,indent=2))
            r.job['source']=str(location);g['a'].source=location
            state=dict(station_generation=r.base_generation,base=r.base.tolist(),
                previous_source=str(old),source=str(location/'report.json'),
                before_boundary_sha256=before.commitment(),actual_object_coordinate=r.actual(),
                invalidated=['view_routes','visual_plan','approach','closing_path','execution_approval'],
                measured_base_used_for_camera_FK_and_IK=True,physical_regrasp_required=True)
            (location/'TRANSITION.json').write_text(json.dumps(state,indent=2))
            from native_g1_safe_park import verify_park
            import numpy as np
            # Only the parking JOINT CELL is reused. No historical retreat or
            # return waypoint is executed; the entire base route was checked.
            cell=dict(parking_q=(old_park['parking_q'] if old_park else r.model.home.tolist()),
                parking_region=(old_park['parking_region'] if old_park else 'native_rest'),
                incoming_motion='complete actuator-executed V81 base route',
                station_generation=r.base_generation)
            parked=verify_park(g,cell)
            g['at_safe_park']=bool(parked['reached']);g['last_exit_plan']=cell if parked['reached'] else None
            result['measured_destination_parking']=parked
            result['transition']=state
            result['next']='observe fresh wrist RGB-D; build and validate a new automatic plan'
            return result
    return StationOperations
