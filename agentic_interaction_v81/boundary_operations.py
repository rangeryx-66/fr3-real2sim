"""Batch deterministic checks; Agent still reviews images and approves contact."""
import hashlib
import json
from pathlib import Path


def wrist_evidence(ops,command):
    rows=ops.g['agent_observations']
    if not rows or rows[-1]['world_sim_s']<getattr(ops,'_last_contact_finished_world_s',0):
        return False
    folder=Path(rows[-1]['folder'])
    return command.get('inspected_image_sha256') in {
        hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [folder/n for n in ('agent_inspection.png','rgb.png','selected_handle_overlay.png')]
        if p.is_file()}


def observe_view(ops,command):
    if not command.get('reason') or not wrist_evidence(ops,command):
        return dict(executed=False,reason='FRESH_INSPECTED_WRIST_EVIDENCE_REQUIRED')
    evaluated=ops.evaluate_view(command['target'])
    if not evaluated.get('feasible'):
        return dict(executed=False,evaluation=evaluated,reason='NO_VALIDATED_VIEW_ROUTE')
    return dict(evaluation=evaluated,observation=ops.move_and_observe(evaluated))


def prepare_grasp(ops,command):
    result=ops.request_plan(command)
    if not result.get('plan'):return result
    # No physical execution/approval here. Use the same existing native-finger
    # board with current observed contact-supported points, not a GT surface.
    from prismatic_inspection_v73 import plan_board
    from agentic_interaction_v8.resources import digest,dump
    board=Path(result['plan_board']) if result.get('plan_board') else Path(plan_board(ops.g))
    folder=ops.r.output
    dump(folder/'V8_PLAN_INSPECTION_PROVENANCE.json',dict(image=str(board),sha256=digest(board),
        plan_sha256=digest(folder/'visual_plan.json'),
        points_sha256=digest(folder/'agent_geometry/contact_supported_points_world.npy'),
        GT_geometry_used=False,read_only=True,
        rendered_contact_supported_points=True))
    execution=ops.validate_execution()
    if not execution.get('valid'):ops.current_plan_ready=False
    result.update(plan_board=str(board),plan_image_sha256=digest(board),
        execution_check=execution,ready_for_Agent_review=bool(execution.get('valid')),
        physical_contact_approved=False,physical_motion_executed=False)
    return result
