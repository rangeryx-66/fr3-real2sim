"""Explicit Agent target switch; preserve images/history, never infer a target."""
import hashlib
import json
from pathlib import Path


def start_latest(ops, command):
    from .boundary_operations import wrist_evidence
    if (ops.r.job['articulation_v7']['joint_type'] not in ('prismatic','revolute')
            or ops.g['recovery_state']!='UNLOADED_OBSERVATION'
            or ops.r.supervisor.latched is not None
            or not command.get('reason') or not wrist_evidence(ops,command)):
        raise ValueError('INSPECTED_FRESH_UNLOADED_ARTICULATED_TARGET_REQUIRED')
    rows=ops.g['agent_observations'];latest=rows[-1];folder=Path(latest['folder'])
    choice=command.get('observed_handle_hypothesis',{})
    if not choice.get('reason'):raise ValueError('EXPLICIT_OBSERVED_TARGET_SELECTION_REQUIRED')
    if choice.get('source')=='SAM3':
        cues=json.loads((folder/'candidate_cues.json').read_text())
        candidates=[c for c in cues if c['name']==choice.get('id')
                    and not c['partial'] and c.get('eligible_for_grasp')]
        if len(candidates)!=1:raise ValueError('CURRENT_COMPLETE_SAM_TARGET_REQUIRED')
        source=folder/(candidates[0]['name']+'.npy')
        expected=hashlib.sha256(source.read_bytes()).hexdigest()
    elif choice.get('source')=='OBSERVED_DEPTH':
        data=json.loads((folder/'depth_handle_proposals.json').read_text())
        candidates=[c for c in data['proposals']if c['id']==choice.get('id')and not c['partial']]
        if len(candidates)!=1:raise ValueError('CURRENT_COMPLETE_DEPTH_TARGET_REQUIRED')
        expected=data['sha256']
    else:raise ValueError('OBSERVED_TARGET_SOURCE_REQUIRED')
    if choice.get('sha256')!=expected:raise ValueError('STALE_OBSERVED_TARGET')
    # The unchanged owner will apply/validate the chosen observed mask and
    # preserve its metric geometry. Do not combine it with a first mask the
    # Agent identified as another part. All old captures remain private history.
    evidence=dict(previous_folders=[r['folder']for r in rows],selected_folder=str(folder),
        observed_choice=choice,inspected_image_sha256=command['inspected_image_sha256'],
        reason=command['reason'],world_sim_s=float(ops.r.world.current_time),
        camera_or_robot_motion=False,observation_budget_reset=False,
        old_geometry_is_history_only=True,GT_geometry_used=False)
    from agent_command_protocol_v12 import atomic_json
    path=ops.r.output/'target_associations';path.mkdir(exist_ok=True)
    atomic_json(path/('%03d.json'%len(ops.g['observation_generations'])),evidence)
    ops.g['observation_generations'].append(list(rows))
    ops.g['agent_observations']=[dict(latest)]
    ops.g.pop('recovery_candidate_catalog',None)
    ops.current_plan_ready=False;ops._v81_plan_ticket=None;ops.cache.clear()
    return evidence


def accept_latest(ops,command):
    """Accept an inspected target before another view, without ranking grasps."""
    import numpy as np
    from agent_command_protocol_v12 import atomic_json
    evidence=start_latest(ops,command)
    request=command['observed_handle_hypothesis'];folder=Path(evidence['selected_folder'])
    if request['source']=='SAM3':mask=np.load(folder/(request['id']+'.npy'))
    else:
        with np.load(folder/'depth_handle_proposals.npz',allow_pickle=False)as data:
            mask=data['proposal_'+request['id']]
    np.save(folder/'handle_mask.npy',mask);np.save(folder/'sam3_mask_raw.npy',mask)
    atomic_json(folder/'AGENT_HANDLE_HYPOTHESIS_APPROVAL.json',dict(request=request,
        source='explicit current observed-target selection before further sensing',GT_used=False))
    # Reuse the existing observed-mask/depth owner for both articulation types.
    # Revolute OperationsV7 lacks this perception convenience method; it has
    # no dependence on rail dynamics or any physical contact permission.
    accept=getattr(ops,'accept_geometry',None)
    if accept is None:
        from interaction_runtime.prismatic_v73 import PrismaticOperationsV73
        accept=lambda folder,reason:PrismaticOperationsV73.accept_geometry(ops,folder,reason)
    feedback=accept(folder,'Agent-inspected current target, no GT or threshold change')
    return dict(accepted=True,target_association=evidence,incremental=feedback,
        robot_motion_executed=False,grasp_search_started=False)
