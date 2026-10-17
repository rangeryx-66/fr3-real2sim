"""Independent post-episode evaluator. Never imported by the runtime or estimator."""
import argparse,json,datetime
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate_xarm_ft_identification import effective_inertia


def evaluate(root):
    root=Path(root);episode=root/'episode';pid=json.loads((root/'PROCESS.json').read_text())['pid']
    if Path('/proc/%d'%pid).exists() or not (episode/'EXPORT_COMPLETE.json').exists():
        raise RuntimeError('Hidden truth remains sealed until actor exits and evidence exports')
    # The boundary is recorded BEFORE any hidden dynamics file is opened.
    (root/'TRUTH_ACCESS_BOUNDARY.json').write_text(json.dumps(dict(UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),actor_alive=False,evidence_exported=True,online_actions_complete=True),indent=2))
    truth=json.loads((episode/'HIDDEN_AUTHORED_PHYSICS.json').read_text());job=json.loads((root/'job.json').read_text())
    from articulated_demo.kinematics import URDFChain
    import xml.etree.ElementTree as ET
    asset=Path(job['asset_root']);m=json.loads((asset/'manifest.json').read_text());urdf=asset/'urdf'/(m['asset_id']+'.urdf');chain=URDFChain(urdf)
    joint=next(j for j in ET.parse(urdf).getroot().findall('joint') if j.get('type')=='revolute');J=chain.root_to_link(joint.find('child').get('link'),{})
    source=json.loads((Path(job['source'])/'report.json').read_text());R=Rotation.from_euler('z',source['asset_installation']['yaw_deg'],degrees=True).as_matrix()@np.array([[0,0,1],[1,0,0],[0,1,0]])
    axis=R@J[:3,:3]@np.fromstring(joint.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis);gs=0.;gc=0.
    for b in truth['bodies']:
        T=chain.root_to_link(b['body'],{});r=R@(T[:3,:3]@b['com_local_m']+T[:3,3]-J[:3,3]);force=np.array([0.,0.,-9.81*b['mass_kg']])
        gs-=axis@np.cross(np.cross(axis,r),force);gc-=axis@np.cross(r,force)
    gt=np.array([effective_inertia(truth,asset,{}),truth['configuration']['b'],truth['configuration']['tau_c'],gs,gc])
    outputs=[]
    for folder in sorted((episode/'hybrid_identification').glob('*')):
        file=folder/'TORQUE_COMPONENTS.json'
        if not file.exists():continue
        model=json.loads(file.read_text());entry=dict(segment=folder.name,measurement_only_model=model)
        if model.get('fit'):
            p=np.array(model['fit']['parameters']);entry.update(authored_parameters=gt.tolist(),absolute_errors=abs(p-gt).tolist(),relative_errors=[float(abs(a-b)/abs(b)) if abs(b)>1e-12 else None for a,b in zip(p,gt)],
                errors_do_not_make_unsupported_parameters_identified=True)
        outputs.append(entry)
    result=dict(label='POSTHOC_AUTHORED_TRUTH_EVALUATION_NOT_ONLINE_INPUT',asset=m['asset_id'],truth_access_after_exit=True,authored_parameters=gt.tolist(),
        parameter_names=['I_eff_kg_m2','b_Nm_s_rad','Coulomb_Nm','gravity_sin_Nm','gravity_cos_Nm'],updates=outputs,
        truth_semantics='coherent authored moving-body density quadrature and native resistance; effective constraint/contact behavior may differ',
        no_complete_independent_GT_joint_effort_channel=True,no_contact_sum_substitution=True,
        gravity_reference='negative sum of m*g moments about hinge; sine/cosine projected first moments, not individually identified mass/COM')
    (root/'PRIVATE_COMPONENT_EVALUATION.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(updates=len(outputs),truth_evaluation_complete=True)))
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);evaluate(p.parse_args().root)
