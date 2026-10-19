"""Reporting only; never called by selection or native feedback control."""
import csv,json
from pathlib import Path
import numpy as np

def read(p):return json.loads(Path(p).read_text())
def summarize(e):
    rows=[];by_episode={}
    for eid in e.c['episodes']:
        out=e.out/eid;selection=read(out/'structure_selection.json') if (out/'structure_selection.json').exists() else {}
        evaluation=read(out/'structure_evaluation_only.json').get('rows',{}) if (out/'structure_evaluation_only.json').exists() else {}
        report=read(out/'refinement_once/report.json') if (out/'refinement_once/report.json').exists() else {}
        actual=read(out/'refinement_once/refinement_actual.json') if (out/'refinement_once/refinement_actual.json').exists() else {}
        physics=read(out/'physics_summary.json') if (out/'physics_summary.json').exists() else {}
        state={'structure_status':selection.get('status','NOT_RUN'),'refinement_interaction_status':report.get('status','NOT_RUN_DEV_GATE' if (e.out/'scientific_stop.json').exists() else 'NOT_RUN'),
               'actual_final_angle_deg':report.get('evaluation',{}).get('actual_door_displacement_deg'),
               'estimated_operation':actual,'minimum_joint_margin_rad':report.get('minimum_joint_margin_rad'),
               'max_observable_contact_drift_m':report.get('maximum_detected_contact_surface_drift_m'),
               'final_relative_slip_m':report.get('evaluation',{}).get('final_true_relative_translation_slip_m'),
               'video':str(out/'refinement_once/contact_baseline.mp4') if (out/'refinement_once/contact_baseline.mp4').exists() else None,
               'physics_status':physics.get('status','NOT_RUN_DEV_GATE' if (e.out/'scientific_stop.json').exists() else 'NOT_RUN'),
               'calibration_status':next((r.get('fit_status') for r in physics.get('rows',[]) if r['method']=='refined_fitted_physics'),'NOT_RUN'),
               'failure_reason':read(e.out/'scientific_stop.json') if (e.out/'scientific_stop.json').exists() else None}
        by_episode[eid]=state
        for row in physics.get('rows',[]) or [{'method':'not_evaluated','condition':'original','status':state['structure_status'] if selection else 'NOT_RUN_DEV_GATE'}]:
            method=row['method'];model='old' if method=='old_structure' else 'refined';error=evaluation.get(model,{})
            test=row.get('test',{});metrics=test.get('metrics',{});drift=test.get('post_drive_drift_prediction') or {}
            rows.append({'episode':eid,'condition':row['condition'],'method':method,'status':row['status'],
                'axis_error_deg':error.get('axis_error_deg') if method!='oracle' else 0.,
                'axis_line_error_mm':error.get('axis_line_error_m',float('nan'))*1000 if method!='oracle' else 0.,
                'heldout_RMSE_mm':metrics.get('ee_position_rmse_m',float('nan'))*1000,
                'velocity_RMSE_mm_s':metrics.get('ee_velocity_rmse_m_s',float('nan'))*1000,
                'start_delay_error_s':metrics.get('start_time_error_s'),
                'final_displacement_error_mm':metrics.get('final_displacement_error_m',float('nan'))*1000,
                'post_drive_drift_mm':drift.get('net_drift_m',float('nan'))*1000,
                'tau_c':row.get('parameters',{}).get('tau_c'),'b':row.get('parameters',{}).get('b'),
                'parameter_intervals':json.dumps(row.get('parameter_intervals')),'identifiability':row.get('fit_status'),
                'actual_final_angle_deg':state['actual_final_angle_deg'],'minimum_joint_margin_rad':state['minimum_joint_margin_rad'],
                'failure_reason':None if metrics else row.get('status')})
        if selection.get('refined'):
            import matplotlib;matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            val=selection['validation'];fig,ax=plt.subplots(1,2,figsize=(9,4))
            ax[0].bar(['old','joint SE(3)'],[val[k]['position_rmse_m']*1000 for k in ('old','refined')]);ax[0].set_ylabel('Validation position RMSE [mm]')
            ax[1].bar(['old','joint SE(3)'],[val[k]['rotation_rmse_rad']*1000 for k in ('old','refined')]);ax[1].set_ylabel('Validation rotation RMSE [mrad]')
            fig.suptitle(eid+' / complete validation action, not random frames');fig.tight_layout();fig.savefig(out/'structure_validation.png');plt.close(fig)
    with (e.out/'main_table.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    summary={'mode':'SIM_TO_SIM_BLIND_SYSID','overall_status':'STOPPED_BY_SCIENTIFIC_GATE' if (e.out/'scientific_stop.json').exists() else 'EVALUATED','complete_closed_loop_validated':False if (e.out/'scientific_stop.json').exists() else None,'episodes':by_episode,'main_table':'main_table.csv',
        'unseen_transfer_assets':1,'unseen_transfer_asset':'45621','old_unseen_denominator':12,
        'old_failures_45130_45166_7138_retained':True,'grasp_contact_proxy_base_recovery_modified':False,
        'real_robot_friction_measured':False,'full_slip_observability_claimed':False,'geometry_improvement_claimed':False}
    (e.out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    return summary
