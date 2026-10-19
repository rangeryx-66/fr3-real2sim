"""Scene-generator-only V7 passive dynamics. Not imported by Agent/estimator.

Positive-density mass/inertia remains the existing V0 construction. The native
linear-axis schema uses SI metres, while the USD angular viscous coefficient
uses degrees. No object drive, applied effort, or external force is authored.
"""
import json
from pathlib import Path
import numpy as np


def law_value(q,law):
    return float(law['base']+law.get('amplitude',0.)*np.exp(-.5*((q-law.get('center',0.))/law.get('width',1.))**2))


def author(stage,scene,case,output):
    from pxr import UsdPhysics,PhysxSchema,Sdf
    from xarm_ft_id_scene import author as mass_author
    # Reuse mass/COM/inertia authoring, explicitly bypass its angular-only
    # resistance author. Exactly one selected-axis resistance law is authored.
    audit=mass_author(stage,scene,case,output,resistance_author=lambda *args:[])
    kind=scene['manifest'].get('joint_type','revolute')
    cls=UsdPhysics.RevoluteJoint if kind=='revolute' else UsdPhysics.PrismaticJoint
    prims=[p for p in stage.Traverse() if str(p.GetPath()).startswith(scene['asset_path']) and p.IsA(cls) and p.GetName()==scene['manifest']['joint_name']]
    if len(prims)!=1:raise RuntimeError('Selected passive joint mapping is ambiguous')
    prim=prims[0];axis='angular' if kind=='revolute' else 'linear'
    prim.AddAppliedSchema('PhysxJointAxisAPI:'+axis)
    attrs=dict(staticFrictionEffort=case['tau_c'],dynamicFrictionEffort=case['tau_c'],viscousFrictionCoefficient=case['b']*(np.pi/180 if kind=='revolute' else 1.))
    for k,v in attrs.items():prim.CreateAttribute('physxJointAxis:'+axis+':'+k,Sdf.ValueTypeNames.Float).Set(float(v))
    PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr().Set(0.)
    drive=UsdPhysics.DriveAPI.Get(prim,axis)
    if drive:
        drive.CreateStiffnessAttr().Set(0.);drive.CreateDampingAttr().Set(0.);drive.CreateMaxForceAttr().Set(0.)
    p=Path(output)/'HIDDEN_AUTHORED_PHYSICS.json';d=json.loads(p.read_text())
    d.update(joint_type=kind,selected_joint=str(prim.GetPath()),resistance_units='N m' if kind=='revolute' else 'N',native_passive_attributes=attrs,law=case['law'],passive_only=True)
    p.write_text(json.dumps(d,indent=2));return audit


def install_native_law(g):
    case=json.loads(Path(g['job']['private_physics_configuration']).read_text())[g['job']['identification_case']]
    view=g['scene']['articulation']._articulation_view._physics_view
    initial=np.asarray(view.get_dof_friction_properties()).copy();dof=g['scene'].get('selected_asset_dof',0)
    if initial.shape[-1]!=3:raise RuntimeError('Unsupported native friction interface')
    if not np.allclose(initial[0,dof,:2],case['tau_c'],atol=1e-6):raise RuntimeError('Passive Coulomb readback mismatch')
    # V7 linear viscous units must be verified, not silently inherited from an
    # angular degrees conversion. For revolute, the baseline USD conversion is
    # checked against its actual tensor SI readback too.
    if not np.isclose(initial[0,dof,2],case['b'],rtol=2e-4,atol=1e-6):raise RuntimeError('Passive viscous SI readback mismatch')
    data=initial.copy();index=np.array([0],dtype=np.uint32)
    report=g['a'].output/'PRIVATE_NATIVE_FRICTION_PREFLIGHT.json'
    report.write_text(json.dumps(dict(native_properties=initial.tolist(),selected_dof=int(dof),passive_only=True,units_checked=True),indent=2))
    # Independent plant-side readback, never an estimator/Agent packet. Preserve
    # unavailable API fields explicitly rather than equating authored inertia
    # with measured effective dynamics.
    native=dict(source='post-reset PhysX tensor readback; not an independent effective-inertia experiment',selected_dof=int(dof))
    for name in ['get_masses','get_inertias','get_coms']:
        try:native[name]=dict(available=True,values=np.asarray(getattr(view,name)()).tolist())
        except Exception as error:native[name]=dict(available=False,reason=repr(error))
    meta=getattr(view,'shared_metatype',None)
    for name in ['link_names','dof_names']:
        value=getattr(meta,name,None)
        native[name]=list(value) if value is not None else dict(available=False)
    (g['a'].output/'PRIVATE_NATIVE_OBJECT_TENSORS.json').write_text(json.dumps(native,indent=2))
    def update():
        data[0,dof,:2]=law_value(g['actual'](),case['law']);view.set_dof_friction_properties(data,index)
        if g['tick']%240==0:
            actual=np.asarray(view.get_dof_friction_properties())
            if not np.allclose(actual,data,atol=1e-6):raise RuntimeError('Native passive resistance readback failed')
    update();return update
