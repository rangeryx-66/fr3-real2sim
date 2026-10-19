"""Private scene-generation side only. Never imported by the numeric estimator."""
import json
from pathlib import Path
import numpy as np

SEEDS=[17046427,17046428,17046429]
PRIORS=dict(moving_mass_scale=[.75,1.25],density_gradient=[-.30,.30],
            coulomb_nm=[.015,.045],viscous_nm_s_rad=[.10,.35],spring='not authored')


def register(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    cases=[]
    for i,seed in enumerate(SEEDS):
        rng=np.random.default_rng(seed)
        cases.append(dict(case=i,asset='46427',seed=seed,mass_scale=float(rng.uniform(*PRIORS['moving_mass_scale'])),
            density_gradient=rng.uniform(*PRIORS['density_gradient'],3).tolist(),tau_c=float(rng.uniform(*PRIORS['coulomb_nm'])),
            b=float(rng.uniform(*PRIORS['viscous_nm_s_rad'])),spring=False))
    p=output/'HIDDEN_CONFIGURATIONS.json';p.write_text(json.dumps(cases,indent=2))
    return cases


def author(stage,scene,case,output):
    """Positive-density cuboid quadrature gives coherent COM/full inertia.

    This versioned approximation follows the baseline's uniform-box mass model.
    Density gradient perturbs COM and inertia together, rather than moving COM
    independently of mass distribution. No robot or surface physics changes.
    """
    from pxr import UsdPhysics,Gf
    from interactive_twin.plant import author_passive_resistance
    output=Path(output);size=np.asarray(scene['geometry_size']);center=np.asarray(scene['geometry_center'])
    nodes=(np.arange(7)+.5)/7-.5
    u=np.array(np.meshgrid(nodes,nodes,nodes,indexing='ij')).reshape(3,-1).T
    weights=1+2*u@np.asarray(case['density_gradient']);weights/=weights.sum()
    points=center+u*size;com=np.sum(points*weights[:,None],axis=0)
    # Include each quadrature voxel's uniform inertia, not only point masses.
    voxel=size/7;voxel_I=np.diag([(voxel[1]**2+voxel[2]**2)/12,(voxel[0]**2+voxel[2]**2)/12,(voxel[0]**2+voxel[1]**2)/12])
    offset=points-com;unit_I=voxel_I+sum(w*(np.dot(d,d)*np.eye(3)-np.outer(d,d)) for d,w in zip(offset,weights))
    audit=[]
    for name,path in [(scene['moving_link'],scene['moving_path']),(scene['door_link_name'],scene['contact_target_path'])]:
        prim=stage.GetPrimAtPath(path);api=UsdPhysics.MassAPI(prim)
        old_mass=float(scene['urdf_links'][name].find('inertial/mass').get('value'));mass=old_mass*case['mass_scale']
        X=scene['asset_chain'].root_to_link(name,{})
        local=X[:3,:3].T@(com-X[:3,3]);I=X[:3,:3].T@(mass*unit_I)@X[:3,:3]
        vals,axes=np.linalg.eigh(I)
        if np.linalg.det(axes)<0:axes[:,0]*=-1
        from scipy.spatial.transform import Rotation
        quat=np.roll(Rotation.from_matrix(axes).as_quat(),1)
        api.CreateMassAttr(mass);api.CreateCenterOfMassAttr(Gf.Vec3f(*local));api.CreateDiagonalInertiaAttr(Gf.Vec3f(*vals));api.CreatePrincipalAxesAttr(Gf.Quatf(*quat))
        audit.append(dict(body=name,path=path,mass_kg=mass,com_local_m=local.tolist(),inertia_local_kg_m2=I.tolist(),principal_values=vals.tolist()))
    resistance=author_passive_resistance(stage,scene['asset_path'],case)
    (output/'HIDDEN_AUTHORED_PHYSICS.json').write_text(json.dumps(dict(configuration=case,bodies=audit,resistance=resistance,
        density='positive affine-density cuboid quadrature; baseline geometry box, not CAD material identification',object_joint_commands=False),indent=2))
    return audit
