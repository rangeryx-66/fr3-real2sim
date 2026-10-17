"""Hidden scene-generation/passive native-joint law only; estimator never imports this."""
import json
from pathlib import Path
import numpy as np
from xarm_ft_id_scene import author as baseline_mass_author

SEEDS=[17146427,17146428]
PRIORS=dict(mass_scale=[.75,1.25],positive_density_gradient=[-.30,.30],
    Coulomb_nm=[.015,.045],viscous_nm_s_rad=[.10,.35],
    localized_resistance_center_deg=[8.,14.],localized_width_deg=[3.,5.],
    fixed_axis_effective_inertia='constant; no internal moving masses',passive_restoring='not authored')


def register(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True);cases=[]
    for i,seed in enumerate(SEEDS):
        rng=np.random.default_rng(seed)
        mass=float(rng.uniform(.75,1.25));gradient=rng.uniform(-.3,.3,3).tolist()
        c=float(rng.uniform(.018,.026));b=float(rng.uniform(.1,.35))
        law=dict(family='constant' if i==0 else 'localized_passive_friction',base_nm=c,
            amplitude_nm=0. if i==0 else float(rng.uniform(.008,.016)),
            center_rad=float(np.deg2rad(rng.uniform(8,14))),width_rad=float(np.deg2rad(rng.uniform(3,5))))
        cases.append(dict(case=i,asset='46427',seed=seed,mass_scale=mass,density_gradient=gradient,tau_c=c,b=b,spring=False,law=law))
    (root/'HIDDEN_CONFIGURATIONS.json').write_text(json.dumps(cases,indent=2))


def law_value(theta,law):
    return float(law['base_nm']+law['amplitude_nm']*np.exp(-.5*((theta-law['center_rad'])/law['width_rad'])**2))


def author(stage,scene,case,output):
    body_audit=baseline_mass_author(stage,scene,case,output)
    p=Path(output)/'HIDDEN_AUTHORED_PHYSICS.json';d=json.loads(p.read_text())
    d.update(law=case['law'],native_realization='set_dof_friction_properties: static=dynamic=c(q); preserve native viscous coefficient',
        passivity='opposing native friction, dissipated power c(q)*abs(qd)+b*qd^2 >=0; no object drive or applied torque',
        law_registered_before_outcomes=True)
    p.write_text(json.dumps(d,indent=2));return body_audit


def install_native_law(g):
    """Physics owner closure, not an observation/estimator or force controller.

    Tensor API edits native passive joint friction, not joint effort/position.
    It keeps the imported native viscous value verbatim, avoiding a second
    USD-degree vs PhysX-radian conversion.
    """
    configuration=json.loads(Path(g['job']['private_physics_configuration']).read_text())[g['job']['identification_case']]
    law=configuration['law'];view=g['scene']['articulation']._articulation_view._physics_view
    dof=g['scene'].get('selected_asset_dof',0);initial=np.asarray(view.get_dof_friction_properties()).copy()
    if initial.shape[-1]!=3:raise RuntimeError('Native friction interface shape mismatch before contact')
    if not np.allclose(initial[0,dof,:2],configuration['tau_c'],atol=1e-6):raise RuntimeError('Native Coulomb mapping mismatch before contact')
    index=np.array([0],dtype=np.uint32);data=initial.copy();path=g['a'].output/'PRIVATE_NATIVE_RESISTANCE.jsonl';log=path.open('a',buffering=1)
    (g['a'].output/'PRIVATE_NATIVE_FRICTION_PREFLIGHT.json').write_text(json.dumps(dict(
        shape=list(initial.shape),initial_native_properties=initial.tolist(),selected_dof=int(dof),
        method='installed omni.physics.tensors.ArticulationView.set_dof_friction_properties',
        no_object_joint_target_or_effort=True,viscous_preserved_verbatim=True),indent=2))
    def update():
        angle=float(g['actual']());c=law_value(angle,law);data[0,dof,0]=c;data[0,dof,1]=c
        view.set_dof_friction_properties(data,index)
        # Periodic backend readback verifies actual native property application.
        if g['tick']%240==0:
            read=np.asarray(view.get_dof_friction_properties())
            if not np.allclose(read[0,dof,:2],c,atol=1e-6):raise RuntimeError('Native passive-friction update failed')
            log.write(json.dumps(dict(t=g['tick']*g['dt'],q=angle,native_properties=read[0,dof].tolist(),law_nm=c))+'\n')
    update();return update
