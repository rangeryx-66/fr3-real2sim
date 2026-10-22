"""High-level station extension; frozen observation/contact operations retained."""
import json
from pathlib import Path
from unittest.mock import patch


def operations_class(original):
    class StationOperations(original):
        def request_plan(self,command):
            import native_g1_execution_candidates as executable
            from interactive_twin_recovery.mobile import at_base
            original_export=executable.current_export
            def current(export,moving_link,moving_pose):
                # Planning always starts from the frozen native export. Only
                # the original support's measured station changes; the object
                # transform is the original current-state safety update.
                moved=at_base(export,self.r.initial_base,self.r.base)
                return original_export(moved,moving_link,moving_pose)
            with patch.object(executable,'current_export',current):
                return super().request_plan(command)

        def evaluate_station(self,command):
            if not command.get('reason') or not command.get('inspected_image_sha256'):
                return dict(valid=False,reason='FRESH_IMAGE_PROVENANCE_AND_AGENT_REASON_REQUIRED')
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
