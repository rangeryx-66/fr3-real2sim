"""Summarize actual outputs, including failures, without turning exit codes into success."""
import argparse,csv,json
from pathlib import Path

def read(p):
    return json.loads(p.read_text()) if p.exists() else {}

def report(root):
    root=Path(root);rows=[];captures=[]
    for p in sorted(root.rglob('backend_status*.json')):
        d=read(p);prior='interaction_prior' in p.name;recon=p.parent/('reconstruction_interaction_prior' if prior else 'reconstruction')
        motion=read(recon/'motion_inferred.json') if d['state']=='COMPLETE' else {};source=Path(d['source']);meta=read(source/'input_provenance.json')
        name=source.name+('_interaction_prior' if prior else '')
        # Unified child runs and original runs both keep evaluation beside outputs.
        workflow=p.parents[5]
        ev=read(workflow/'value_check'/name/'heldout_state_evaluation.json') if d['state']=='COMPLETE' else {}
        rows.append({'capture_version':Path(meta.get('capture_root','unknown')).name,'attempt_file':p.name,'pair':name,'backend_status':d['state'],'joint_types':','.join(motion.get('joint_types',[])),
                     'type_source':motion.get('joint_type_source','unavailable'),'GT_mesh_axis_used':d.get('GT_mesh_input',False) or d.get('GT_axis_input',False),
                     'actual_iterations':str(d.get('actual_iterations',{})),'train_states':str(meta.get('states',[])),
                     'train_view_count':len(meta.get('train_views',[])),'independent_view_test':meta.get('independent_view_validation_available',False),
                     'heldout_state_RGB_RMSE':ev.get('mean_foreground_RGB_RMSE'),'heldout_state_silhouette_IoU':ev.get('mean_silhouette_IoU'),
                     'prediction_kind':'conditional geometry at measured estimated state phase; NOT autonomous physics',
                     'error':d.get('error',''),'source':str(source),'output':str(recon)})
    for p in sorted(root.glob('*/multistate_capture.json')):
        d=read(p);q=read(p.parent/'report.json');history=read(p.parent/'reposition_history.json') if (p.parent/'reposition_history.json').exists() else []
        if not isinstance(history,list):history=[]
        states=d.get('states',[]);completed=[h for h in history if h.get('regrasp_completed')]
        infrastructure=read(p.parent/'infrastructure_failure.json')
        supervised=read(p.parent/'supervised_stop.json')
        captures.append({'run':p.parent.name,'asset':d.get('object_id'),'capture_status':d.get('status'),'physical_status':supervised.get('status',q.get('status',infrastructure.get('status','RUNNING_OR_REPORT_MISSING'))),
                         'original_physical_report_status':q.get('status'),'controller_goal_reported_success':q.get('success',False),
                         'states':str([round(s['estimated_articulation_state'],5) for s in states]),'units':d.get('units'),
                         'actual_final_joint_displacement':q.get('evaluation',{}).get('actual_joint_displacement'),
                         'minimum_joint_margin_rad':q.get('minimum_joint_margin_rad'),'maximum_true_relative_slip_m':read(p.parent/'physical_collection_evaluation.json').get('maximum_true_relative_translation_drift_m'), 'final_relative_drift_m':q.get('evaluation',{}).get('final_true_relative_translation_slip_m'),
                         'repositions_completed':len(completed),'repositions_attempted':len(history),
                         'safe_releases_completed':sum(bool(h.get('released')) for h in history),
                         'base_routes_completed':sum(bool(h.get('base_reposition_completed')) for h in history),
                         'bilateral_grasp':q.get('bilateral_hold_established'),
                         'video':str(p.parent/'contact_baseline.mp4'),'effort_measured':read(p.parent/'effort_profile.json').get('measurement_completed',False)})
    def table(name,data):
        if not data:return
        with (root/name).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
    table('reconstruction_comparison.csv',rows);table('collection_status.csv',captures)
    lines=['# Articulated collection and ArtGS integration','',
           'Official ArtGS commit: `7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a`. Independent environment. Scene-specific optimization; official demonstration weights are never used as target meshes/axes.',
           'Successful physical grasp/contact baseline remains unchanged. New capture/orchestration outputs preserve failed attempts. Collection failure does not cancel reconstruction.',
           '', '## Actual reconstruction runs','', '| Pair | State | Type | Train views | Independent views | Held-out RGB RMSE | IoU |', '|---|---|---|---:|---|---:|---:|']
    for r in rows:lines.append(f"| {r['pair']} | {r['backend_status']} | {r['joint_types']} | {r['train_view_count']} | {r['independent_view_test']} | {r['heldout_state_RGB_RMSE']} | {r['heldout_state_silhouette_IoU']} |")
    lines+=['','Type-prior results use only the existing measured-EE identification of joint family. ArtGS still estimates axes and geometry. They are explicitly not blind ArtGS type prediction.',
            'Held-out state is not used in reconstruction. Its measured articulation label supplies rendering phase: conditional geometric prediction, not autonomous physics prediction. Old two-view train-view render checks are not independent validation.',
            '', '## Physical collection and recovery','', '| Run | Asset | Captured states | Physical stop | Completed release/reposition/regrasp | Actual final displacement |', '|---|---|---|---|---:|---:|']
    for r in captures:lines.append(f"| {r['run']} | {r['asset']} | {r['states']} {r['units']} | {r['physical_status']} | {r['repositions_completed']} | {r['actual_final_joint_displacement']} |")
    lines+=['','## Interpretation and provenance','',
            '- Mesh and joint outputs are inferred from actual sensor observations; dataset reference geometry/axes are not substituted. Poor geometry, wrong joint classification and empty meshes remain failures.',
            '- Meter scale and ROS-optical/OpenGL camera transforms are preserved in `input_provenance.json`. Different states remain separate; robot pixels are excluded with simulator instance masks, explicitly not real-world segmentation.',
            '- URDF limits are bounded preview/observed ranges, not recovered full joint limits. Preview mass/inertia are neutral priors, not measured physics.',
            '- `reconstructed_preview.mp4` drives the newly inferred twin joint in an independent Isaac scene. It demonstrates import/assembly only, never robot contact success.',
            '- Effort measurement requires verified normal+friction buffers. Unverified data remain command proxies. Onset windows are task-opening resistance, not exact minimum friction; two finger clamping loads are not added as pull force.',
            '- No friction model fitting, collision geometry changes or safety threshold changes are performed.',
            '', '## Files','', '`reconstruction_comparison.csv`, `collection_status.csv`, per-run logs/provenance and per-twin `reconstructed_parts/`, `reconstructed.urdf`, `effort_profile.json`, `state_observations/`, `twin_update.json`.',
            'Missing/failed components are not declared complete. State observation symlinks must be dereferenced when transferring twin packages.']
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path('results/articulated_system_20261005'));report(p.parse_args().root)
