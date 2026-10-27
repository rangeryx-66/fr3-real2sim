"""Read-only previous-data sensitivity; no Isaac launch or physical success claim."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agentic_interaction_v88.physics_aware import DemandModel,Capability,choose
from update_probing_v7_model import load_segments


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--episode',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    p=args.episode; idx=json.loads((p/'telemetry/index.json').read_text())
    model=json.loads((p/'model/estimate.json').read_text())['selected']
    pieces,settings,quality=load_segments(p,False)
    fit=[d for d in pieces if str(d['block_id']).startswith('FIT')]
    X=np.concatenate([np.c_[d['qdd'][d['valid']],d['qd'][d['valid']],
        np.sign(d['qd'][d['valid']])] for d in fit])
    force=np.concatenate([d['torque'][d['valid']] for d in fit]);raw=np.concatenate([
        d['unfiltered_torque'][d['valid']] for d in fit])
    beta=np.asarray(model['parameters']);residual=raw-X@beta
    scale=np.maximum(np.linalg.norm(X,axis=0),1e-30)
    singular=np.linalg.svd(X/scale,compute_uv=False)
    phase=idx['metadata']['phases'].index('OPEN_CONTINUE');contact=[]
    for c in idx['chunks']:
        with np.load(p/'telemetry'/c['file']) as a:
            mask=(a['segment']==1)&(a['phase']==phase)&a['v88_coordination_active'].astype(bool)
            if mask.any():contact.append({k:a[k][mask] for k in [
                'normal_force_on_fingers_world_N','friction_force_on_fingers_world_N',
                'friction_api_valid','pad_load','relative_slip_m']})
    c={k:np.concatenate([v[k] for v in contact]) for k in contact[0]}
    ratio=np.linalg.norm(c['friction_force_on_fingers_world_N'],axis=-1)/np.maximum(
        np.linalg.norm(c['normal_force_on_fingers_world_N'],axis=-1),1e-9)
    residual95=float(np.percentile(abs(residual),95))
    report=dict(schema='physics-aware-offline-sensitivity/v1',physical_experiments_launched=False,
        prior_episode_used_for_measurement_audit_only=True,GT_used_for_formal_control=False,
        prior_FIT=dict(samples=len(X),rank=int(np.linalg.matrix_rank(X)),
            normalized_singular_values=singular.tolist(),condition=float(singular[0]/singular[-1]),
            parameters=beta.tolist(),conditional_std=model['conditional_std'],
            filtered_force_RMSE=float(np.sqrt(np.mean((force-X@beta)**2))),
            unfiltered_residual_abs95_N=residual95,
            measured_speed_minmax_m_s=[float(X[:,1].min()),float(X[:,1].max())],
            measured_acceleration_abs95_m_s2=float(np.percentile(abs(X[:,0]),95))),
        empirical_sensitivity={name:dict(force_RMS_N=float(np.sqrt(np.mean(delta**2))),
            force_abs95_N=float(np.percentile(abs(delta),95))) for name,delta in {
                'mass_plus_6kg':6*X[:,0], 'damping_plus_58_5Ns_m':58.5*X[:,1],
                'rail_Coulomb_plus_0_45N':.45*X[:,2]}.items()},
        prior_contact=dict(native_friction_API_valid_fraction=float(c['friction_api_valid'].mean()),
            used_friction_over_normal_ratio_percentiles=np.percentile(ratio,[5,50,95,99],axis=0).tolist(),
            pad_normal_median_N=np.median(c['pad_load'],axis=0).tolist(),
            relative_translation_max_m=float(c['relative_slip_m'].max()),
            no_slip_identifies_mu_point_value=False,
            interpretation='Observed used traction is only a friction/capacity lower bound; no forced slip inference'),
        controller_limits=json.loads((ROOT/'configs/interaction_runtime_v1.json').read_text())['phases']['LOADED'])
    # Optimistic oracle-only thought experiment. These normal forces are NOT
    # measured calibration and cannot be admitted to formal action execution.
    cases=[('nominal',2.2,2.,.35,.65),('damping',2.2,600.,.35,.65),
        ('mass',8.,2.,.35,.65),('rail',2.2,2.,1.1,.65),
        ('contact',2.2,2.,.35,.12),('combined',8.,60.,.7,.18)]
    menu=[.5,.75,1.,1.5,2.,2.5];speeds=[.002,.004,.006];accelerations=[.004,.008,.012]
    def oracle(m,b,f,mu):
        prediction=DemandModel((m,b,f),tuple(map(tuple,np.zeros((3,3)))),residual95,'OFFLINE_ORACLE_ONLY')
        capability=[Capability(v,2*v*mu,(v,v),(v,v),'UNVALIDATED_THEORETICAL_NORMAL_LOAD') for v in menu]
        return choose(prediction,capability,distance=.04,speeds=speeds,accelerations=accelerations,
            motion=report['controller_limits'],sustained_pad_n=4.,emergency_wrist_n=10.)
    stale=oracle(*cases[0][1:]);report['oracle_only_control_sensitivity']=[]
    control=lambda action:tuple(action[k] for k in ('requested_preload_n',
        'reference_speed_m_s','reference_acceleration_m_s2')) if action else None
    for name,m,b,f,mu in cases:
        updated=oracle(m,b,f,mu);report['oracle_only_control_sensitivity'].append(dict(
            condition=name,theoretical_parameters=dict(m=m,b=b,Fc=f,mu=mu),
            stale_action=stale['action'],oracle_updated_action=updated['action'],
            changed_action=control(stale['action'])!=control(updated['action']),
            real_slip_or_success_inferred=False))
    report['precheck_conclusion']=dict(
        actual_contact_sensitivity_not_yet_proven=True,
        actual_requested_preload_mapping_not_yet_calibrated=True,
        current_feedback_already_sufficient_not_yet_tested=True,
        model_to_control_theoretical_effect_demonstrated=any(
            x['changed_action'] for x in report['oracle_only_control_sensitivity']),
        formal_comparison_admitted=False,
        next='Short native contact/preload/braking qualification; theoretical policy differences are not operation benefits')
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),prior_FIT=report['prior_FIT'],
        empirical_sensitivity=report['empirical_sensitivity'],
        oracle_actions=[dict(condition=x['condition'],action=x['oracle_updated_action']) for x in report['oracle_only_control_sensitivity']],
        conclusion=report['precheck_conclusion']),indent=2))


if __name__=='__main__':main()
