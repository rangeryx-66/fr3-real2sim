"""Publish compact, explicitly post-episode diagnostics; never run by the actor.

Inputs are the independent evaluator JSON and its private MEASUREMENTS.npz.
No model is fitted here and no physical experiment is launched.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def summarize(evaluation, private, output):
    output.mkdir(parents=True, exist_ok=True)
    summaries = []
    for case in evaluation:
        i = case['case']
        result = case['physical_result']
        attempt = result['attempts'][0]
        model = case['posthoc'][0]['model']
        gt = case['authored_truth']
        maximum = attempt['acquisition']['angle_deg'] + attempt['opening_gain_deg']
        q = np.linspace(0, np.deg2rad(maximum), 201)
        law = gt['Coulomb']
        truth = law['base_nm'] + law['amplitude_nm'] * np.exp(-.5*((q-law['center_rad'])/law['width_rad'])**2)
        comparisons = {}
        for name, fit in model['models'].items():
            b = np.array(fit['parameters'])
            x = np.clip(q/np.deg2rad(20), 0, 1)
            predicted = np.full_like(q, b[2]) if name == 'constant' else b[2]*(1-x)**2 + 2*b[3]*x*(1-x) + b[4]*x*x
            comparisons[name] = dict(
                provisional_parameters=fit['parameters'], conditional_std=fit['conditional_std'],
                I_absolute_error_kg_m2=float(abs(b[0]-gt['I_eff_kg_m2'])),
                I_relative_error=float(abs(b[0]-gt['I_eff_kg_m2'])/gt['I_eff_kg_m2']),
                b_absolute_error_nm_s_rad=float(abs(b[1]-gt['b_nm_s_rad'])),
                b_relative_error=float(abs(b[1]-gt['b_nm_s_rad'])/gt['b_nm_s_rad']),
                c_observed_interval_rmse_nm=float(np.sqrt(np.mean((predicted-truth)**2))),
                c_observed_interval_mean_relative_absolute_error=float(np.mean(abs(predicted-truth)/truth)),
                training_rmse_nm=fit['training_rmse_nm'], heldout_torque_rmse_nm=fit['heldout_torque_rmse_nm'],
                scaled_regressor_singular_values=fit['scaled_regressor_singular_values'], scaled_condition=fit['scaled_condition'],
                scaled_fisher_information=fit['scaled_fisher_information'], conditional_covariance=fit['conditional_covariance'],
                conditional_supported_flags=fit['supported'],
                damping_inside_frozen_prior=bool(.1 <= b[1] <= .35),
                forward_prediction=fit['heldout_native_velocity_initial_state'],
                errors_are_posthoc_not_model_selection=True)
        summary = dict(case=i, seed=case['seed'], asset='46427', physics_family=law['family'],
            maximum_actual_angle_deg=maximum, post_acquisition_maximum_gain_deg=attempt['opening_gain_deg'],
            segments=model['executed_segment_outcomes'], actual_physical_grasps=result['physical_grasp_attempts'],
            actual_recovery_grasps=result['recovery_grasps'], recovery_planning_branches=1,
            recovery_selected_approach='rejected: gripper_link1 versus native original41; not executed',
            acquisition_pad_N=attempt['acquisition']['pad_loads_n'], peak_recorded_pad_N=attempt['peak_pad_load_n'],
            min_joint_margin_rad=attempt['minimum_joint_margin_rad'], termination=attempt['termination_reason'],
            guarded_physical_attempt_exit=attempt['safe_exit']['terminal_state'],
            final_scene_state='unloaded observation pose after recovery-plan rejection; no new final parking verification',
            simulation_s=result['simulation_s'], runner_wall_s=result['wall_s'], physical_attempt_wall_s=attempt['physical_attempt_wall_s'],
            wrist_observations=sum(map(len, result['observation_generations'])), Agent_adjustments=result['Agent_adjustments'],
            completed_information_blocks=len(case['updates']), in_episode_parameter_updates=len(case['updates']),
            selected_posthoc_model=model['selected_model'], hidden_authored_truth_for_evaluation_only=gt,
            native_Coulomb_readback_max_error_nm=case['max_native_Coulomb_readback_error_nm'],
            native_viscous_readback_range=case['native_viscous_readback_range'],
            native_viscous_vs_authored_max_error=case['native_viscous_vs_authored_b_max_error'],
            coverage=model['coverage'], training_coverage=model['training_coverage'], validation_coverage=model['validation_coverage'],
            training_bins=model['training_samples'], validation_bins=model['validation_samples'],
            data_quality_audit=model['data_quality_audit'], authored_reference=model['authored_inverse_dynamics_reference'],
            model_comparison=comparisons, identification_PASS=False, physical_twin_validated=False,
            uncertainty='conditional numerical covariance, not calibrated hardware confidence',
            no_initial_state_stitching_or_GT_grasp_fallback=True)
        root = private/('case_%02d'%i)/'archive'
        if not root.exists(): root = private/('case_%02d'%i)
        audit = json.loads((root/'EVIDENCE_AUDIT.json').read_text())
        summary['evidence_audit'] = {k: v for k, v in audit.items() if k != 'source'}
        quant = json.loads((root/'episode/posthoc_v1/attempt_00/RESOLUTION_SENSITIVITY.json').read_text())
        summary['quantization_only_sensitivity'] = quant
        for src, dst in [('ESTIMATED_PHYSICS_MODEL_V1.json', 'CASE_%02d_ESTIMATED_MODEL.json'%i),
                         ('MULTISTATE_RGBD_INDEX.json', 'CASE_%02d_RGBD_INDEX.json'%i)]:
            # References and numeric model only, never RGB/depth/raw telemetry.
            (output/dst).write_text((root/src).read_text())
        summaries.append(summary)

        d = dict(np.load(root/'episode/posthoc_v1/attempt_00/MEASUREMENTS.npz', allow_pickle=True))
        t = d['t'] - attempt['acquisition']['t']
        # Draw every native record; vectorized line rendering preserves brief peaks.
        with plt.rc_context({'path.simplify': False}):
            fig, axes = plt.subplots(6, 2, figsize=(14, 13), sharex=True)
            for j, axis in enumerate(axes[:, 0]):
                name = ('F' if j < 3 else 'T') + 'xyz'[j % 3]
                axis.plot(t, d['raw'][:, j], lw=.5, label='raw sensor '+name)
                axis.set_ylabel('N' if j < 3 else 'Nm'); axis.legend(fontsize=7)
            for j, axis in enumerate(axes[:, 1]):
                name = ('F' if j < 3 else 'T') + 'xyz'[j % 3]
                axis.plot(t, d['compensated_world'][:, j], lw=.5, label='compensated world '+name)
                axis.set_ylabel('N' if j < 3 else 'Nm'); axis.legend(fontsize=7)
            for ax in axes.flat: ax.grid(alpha=.2)
            for ax in axes[-1]: ax.set_xlabel('simulation seconds after GRASP_VERIFIED')
            fig.suptitle('Case %02d: all local-segment native samples; raw and compensated use different frames'%i)
            fig.tight_layout(); fig.savefig(output/('CASE_%02d_FT.png'%i), dpi=140); plt.close(fig)

            fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
            for j, name in enumerate(['left', 'right']): axes[0].plot(t, d['pad_loads'][:, j], lw=.6, label=name+' normal N')
            axes[1].plot(t, d['relative_translation_proxy_m']*1000, lw=.6, label='relative translation proxy mm')
            axes[2].plot(t, np.rad2deg(d['q']), lw=.7, label='actual angle deg')
            axes[3].plot(t, d['torque'], lw=.6, label='wrist-derived joint effort Nm')
            for ax in axes: ax.grid(alpha=.2); ax.legend(fontsize=8)
            for label in dict.fromkeys(d['segments']):
                ix = np.flatnonzero(d['segments'] == label)
                start, end = t[ix[0]], t[ix[-1]]
                for ax in axes: ax.axvspan(start, end, color='gold' if str(label).startswith('VALIDATION') else 'steelblue', alpha=.08)
                axes[0].text((start+end)/2, 1.02, str(label).replace('_INITIAL_', '\n'), fontsize=6, ha='center', transform=axes[0].get_xaxis_transform())
            axes[-1].set_xlabel('simulation seconds after GRASP_VERIFIED; yellow = whole held-out closing segment')
            fig.suptitle('Case %02d: actual bidirectional motion, followed by original transmission stop; no new contact veto'%i)
            fig.tight_layout(); fig.savefig(output/('CASE_%02d_TRANSMISSION.png'%i), dpi=140); plt.close(fig)
        for src in (root/'evaluation_figures').glob('*.png'):
            if src.name.startswith(('COVERAGE_', 'MODEL_COMPARISON_POSTHOC_')):
                (output/('CASE_%02d_'%i + src.name)).write_bytes(src.read_bytes())
    (output/'RESULTS_SUMMARY.json').write_text(json.dumps(dict(
        label='XARM7_NATIVE_G1 / KNOWN_HINGE_STATE / IDEALIZED_LOAD_CELL / APPROXIMATE_ACTUATOR_SIMULATION',
        frozen_runtime_commit='c360e38', cases=summaries, posthoc_analysis_not_online_updates=True,
        physical_trials_finished=True, further_physical_experiments_permitted_in_this_batch=False), indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('evaluation', type=Path); p.add_argument('private', type=Path); p.add_argument('output', type=Path)
    a = p.parse_args()
    summarize(json.loads(a.evaluation.read_text()), a.private, a.output)
