"""High-level station extension; frozen observation/contact operations retained."""
import json
from pathlib import Path
from unittest.mock import patch


def operations_class(original):
    class StationOperations(original):
        worker_script='agentic_interaction_v81/update_model.py'

        def interact(self,target,max_sim_s=None,action=None):
            proposed=self.actions[0] if self.actions else action
            if proposed and proposed['role'].startswith('VALIDATION'):
                import hashlib
                folder=self.r.output/'model';m=folder/'estimate.json';f=folder/'FIT_READINESS_V81.json'
                ready=json.loads(f.read_text()) if f.exists() else {}
                model_bytes=m.read_bytes() if m.exists() else None
                expected=max((s['id'] for s in self.r.record.segments if s['role'].startswith('FIT') and s['completed']),default=0)
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
            return super().interact(target,max_sim_s,action)

        def feedback(self):
            result=super().feedback();p=self.r.output/'model/FIT_READINESS_V81.json'
            result['FIT_readiness_v81']=json.loads(p.read_text()) if p.exists() else dict(status='pending')
            return result

        def attempt(self,command):
            if command.get('segments')!=getattr(self,'_v81_declared_chain',None):
                return dict(executed=False,recoverable=True,reason='DECLARE_AND_VALIDATE_THIS_CHAIN_BEFORE_ACQUISITION')
            result=super().attempt(command)
            self._last_contact_finished_world_s=float(self.r.world.current_time)
            result['station_generation']=self.r.base_generation
            return result

        def request_plan(self,command):
            from interaction_runtime.segments import validate_action
            actions=command.get('planned_segments')
            if not isinstance(actions,list) or not actions:
                return dict(plan=False,reason='PREDECLARE_FINITE_NATIVE_COORDINATE_CHAIN',recoverable=True)
            for action in actions:validate_action(action);self.articulation().validate_target(action)
            self._v81_declared_chain=actions
            # The old fixed planning target is an input alias, not a controller
            # limit. Plan the Agent's next declared native-coordinate action.
            # Subsequent transitions retain the original current-state checker.
            self.r.job['benchmark_goal_rad']=float(actions[0]['target_q'])
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
            with patch.object(executable,'current_export',current), cached_planning_fk():
                return super().request_plan(command)

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
