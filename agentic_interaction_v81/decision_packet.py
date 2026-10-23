"""Read-only SI decision packet for the frozen revolute owner.

The frozen prismatic owner already supplies this interface. Do not pretend it
exists on the revolute parent or change either parent's physical behavior.
"""
import json
from pathlib import Path


def revolute_packet(ops,sequence,result=None):
    r=ops.r
    if r.job['articulation_v7']['joint_type']!='revolute':
        raise ValueError('MISSING_PRISMATIC_PARENT_DECISION_PACKET')
    rows=ops.g['agent_observations'];folder=Path(rows[-1]['folder'])if rows else None
    feedback=ops.feedback();contact=feedback.get('contact_feedback',{})
    model=feedback.get('model',{});selected=model.get('selected')or{}
    board=r.output/'AGENT_GRASP_PLAN_INSPECTION.png'
    return dict(sequence=sequence,unloaded=True,joint_coordinate=r.actual(),
        units=ops.articulation().units,acquisitions=ops.acquisitions,
        remaining_resources=r.episode_budget.remaining(r.tick*r.dt),
        fresh_rgb=str(folder/'rgb.png')if folder else None,
        RGBD_board=str(folder/'agent_inspection.png')if folder else None,
        contact_board=contact.get('path'),contact_geometry=contact.get('contact_geometry',{}).get('path'),
        current_plan_ready=bool(ops.current_plan_ready),
        plan_board=str(board)if ops.current_plan_ready and board.exists()else None,
        current_observation_generation=len(ops.g['observation_generations']),
        model_summary=dict(status=model.get('status','pending'),parameters=selected.get('parameters'),
            conditional_std=selected.get('conditional_std'),condition=selected.get('scaled_condition'),
            validation_bins=model.get('validation_bins')),
        model_reference=str(r.output/'model/estimate.json'),
        FIT_readiness_v81=feedback.get('FIT_readiness_v81'),
        last_result=result,events=feedback.get('events',[]),hard_latch=r.supervisor.latched,
        instruction='Inspect fresh wrist evidence; current-state native checks and hard stops remain authoritative. Known hinge/state prior, no hidden dynamics.')
