"""Opt-in constructor-only plant treatment and fresh-study matched observation.

No task controller reads plant coefficients. No contact outcome is scripted.
The original ABC dispatcher, actuators, collision checks and guards remain.
"""
from contextlib import contextmanager
import hashlib,json,math,re,shutil
from pathlib import Path
from unittest.mock import patch
import numpy as np


ROLES=('qualification','calibration','identify_m0','identify_m1','reactive','stale','updated')


def independent_heldout(job,action):
    """Formal A/B/C motion must not require fitting changed conditions first."""
    study=job.get('v88_physics_study') or {}
    return bool(job.get('v84_abc') and job.get('v88_coordination_policy')
        and study.get('schema')=='physics-aware-contact-v1'
        and study.get('role') in ('reactive','stale','updated')
        and action and action.get('role','').startswith('VALIDATION'))


def validate(config):
    if (not isinstance(config,dict) or set(config)-{'schema','role','reference'}
            or config.get('schema')!='physics-aware-contact-v1' or config.get('role') not in ROLES):
        raise ValueError('EXPLICIT_PHYSICS_STUDY_REGISTRATION_REQUIRED')
    if 'reference' in config:
        ref=config['reference']
        if not isinstance(ref,dict) or set(ref)!={'path','sha256'}:
            raise ValueError('COMMITTED_FRESH_STUDY_REFERENCE_REQUIRED')
        p=Path(ref['path'])
        if not p.is_absolute() or hashlib.sha256(p.read_bytes()).hexdigest()!=ref['sha256']:
            raise ValueError('STUDY_REFERENCE_COMMITMENT_MISMATCH')
        d=json.loads(p.read_text());f=Path(d['fresh_reference']['path'])
        if hashlib.sha256(f.read_bytes()).hexdigest()!=d['fresh_reference']['sha256']:
            raise ValueError('FRESH_REFERENCE_CHANGED')
        v=json.loads(f.read_text())
        if (v.get('kind')!='fresh_matched_observation_reference_v81'
                or v.get('physical_acquisitions_at_export')!=0
                or v.get('saved_successful_grasp') is not False or v.get('GT_grasp_used') is not False):
            raise ValueError('CURRENT_STUDY_UNEXECUTED_OBSERVED_REFERENCE_ONLY')
        for name,sha in d['observation_files'].items():
            if Path(name).is_absolute() or '..' in Path(name).parts:
                raise ValueError('REFERENCE_OBSERVATION_PATH_ESCAPE')
            if hashlib.sha256((Path(d['observation_folder'])/name).read_bytes()).hexdigest()!=sha:
                raise ValueError('REFERENCE_OBSERVATION_CHANGED')
    return dict(config)


def material_values(case):
    d=case.get('contact_material')
    inherited=case.get('inherit_contact_material') is True and 'contact_material' not in case
    if not inherited and (not isinstance(d,dict) or set(d)!={'static_friction','dynamic_friction','combine_mode'}):
        raise ValueError('EXPLICIT_NATIVE_CONTACT_MATERIAL_REQUIRED')
    if not inherited:
        s,f=d['static_friction'],d['dynamic_friction']
        if not all(type(x) in (int,float) and math.isfinite(x) for x in (s,f)) or not 0<f<=s<=2:
            raise ValueError('FINITE_POSITIVE_CONTACT_FRICTION_REQUIRED')
        if d['combine_mode']!='average':raise ValueError('EXPLICIT_PAIR_AVERAGE_REQUIRED')
    ratio=case.get('static_rail_multiplier',1.)
    if type(ratio) not in (int,float) or not math.isfinite(ratio) or not 1<=ratio<=3:
        raise ValueError('VALID_STATIC_DYNAMIC_RAIL_RATIO_REQUIRED')
    return d,float(ratio)


def author_contact(stage,scene,case,output):
    from pxr import Usd,UsdPhysics,UsdShade,PhysxSchema
    from xarm7_native_gripper import FINGER_BODIES
    material,ratio=material_values(case)
    from agent_command_protocol_v12 import atomic_json
    if material is None:
        atomic_json(Path(output)/'PRIVATE_NATIVE_CONTACT_MATERIAL.json',dict(
            inherited_native_baseline_material=True,contact_material_overrides=False,
            coefficient_not_inferred_from_engine_defaults=True,static_rail_multiplier=ratio))
        return
    mat=UsdShade.Material.Define(stage,'/World/PhysicsAwareContactMaterial')
    api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(material['static_friction']))
    api.CreateDynamicFrictionAttr().Set(float(material['dynamic_friction']))
    api.CreateRestitutionAttr().Set(0.)
    PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set('average')
    normalize=lambda s:re.sub('[^A-Za-z0-9]','',s)
    stems=[normalize(x) for x in scene['manifest']['grasp_mesh_stems']]
    pad_paths={name:[] for name in FINGER_BODIES};handle=[];binding_owners={}
    for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        path=str(prim.GetPath());finger=None;parent=prim
        while parent and not parent.IsPseudoRoot():
            if parent.GetName() in FINGER_BODIES:finger=parent.GetName();break
            parent=parent.GetParent()
        target=(path.startswith(scene['asset_path']+'/') and any(s in normalize(path) for s in stems))
        if not (finger and path.startswith('/World/Piper/') or target):continue
        # Imported collision meshes are instance proxies. Bind on the closest
        # writable instance root, then verify inherited physics material at
        # each real collider. Preserve instancing and every geometry byte.
        owner=prim
        while owner.IsInstanceProxy():owner=owner.GetParent()
        owner_path=str(owner.GetPath())
        if ((finger and not owner_path.startswith(str(parent.GetPath())))
                or (target and not any(s in normalize(owner_path) for s in stems))):
            raise RuntimeError('LOCAL_PAD_OR_HANDLE_BINDING_REQUIRED_NO_ASSEMBLY_OVERRIDE')
        binding=UsdShade.MaterialBindingAPI.Apply(owner)
        binding.Bind(mat,bindingStrength=UsdShade.Tokens.strongerThanDescendants,materialPurpose='physics')
        bound,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose='physics')
        if not bound or bound.GetPath()!=mat.GetPath():raise RuntimeError('NATIVE_MATERIAL_BINDING_FAILED')
        binding_owners[path]=owner_path
        if finger:pad_paths[finger].append(path)
        if target:handle.append(path)
    if not handle or not all(pad_paths.values()):raise RuntimeError('EXACT_PAD_AND_HANDLE_COLLIDERS_REQUIRED')
    atomic_json(Path(output)/'PRIVATE_NATIVE_CONTACT_MATERIAL.json',dict(
        material=material,handle_colliders=handle,finger_colliders=pad_paths,
        collider_binding_owners=binding_owners,instancing_preserved=True,
        both_sides_same_average_material=True,rigid_handle_damage_model=False,
        object_or_robot_geometry_changed=False,static_rail_multiplier=ratio))


def initialize_reference(r,ops,config):
    from agent_command_protocol_v12 import atomic_json
    d=json.loads(Path(config['reference']['path']).read_text())
    # A controlled initial-state association, never a fabricated fresh capture.
    # Subsequent contact invalidates it through its time-zero observation stamp.
    if abs(r.actual()-d['initial_object_coordinate_native'])>1e-6:
        raise RuntimeError('MATCHED_INITIAL_OBJECT_STATE_MISMATCH')
    r.read_base()
    if not np.allclose(r.base,d['initial_base_pose'],atol=1e-3,rtol=0):
        raise RuntimeError('MATCHED_INITIAL_STATION_MISMATCH')
    r.hold(r.g['wrist_config']['settle_s'])
    if r.supervisor.latched is not None or r.grasp_reference is not None or max(r.filtered())>.05:
        raise RuntimeError('VERIFIED_UNLOADED_REFERENCE_ASSOCIATION_REQUIRED')
    # Measure the actual sensing/refinement cost in this instance. Do not
    # fabricate a SAM timing entry to satisfy the unchanged cost admission.
    # Its image is retained but does not replace the matched geometry input.
    fresh_capture=ops.observe()
    r.g['observation_generations'].append(list(r.g['agent_observations']))
    folder=r.output/'rgbd'/'controlled_current_study_reference'
    shutil.copytree(d['observation_folder'],folder)
    r.g['agent_observations']=[dict(index=0,folder=str(folder),accepted_for_geometry=True,
        world_sim_s=0.,source='fixed current-study unexecuted observed reference',
        physical_capture_in_this_episode=False,original_capture_episode=d['source_episode'])]
    r.g['wrist_routes']=[];r.g['agent_moves']=0
    r.g['recovery_state']='UNLOADED_OBSERVATION'
    atomic_json(r.output/'AGENT_OBSERVATIONS.json',r.g['agent_observations'])
    result=dict(observation=r.g['agent_observations'][0],image_board=str(folder/'agent_inspection.png'),
        fresh_vision_generalization_claim=False,actual_native_admission_still_required=True,
        actual_instance_sensing=fresh_capture)
    atomic_json(r.output/'INITIAL_OBSERVATION_READY.json',result)
    atomic_json(r.output/'CONTROLLED_REFERENCE_ASSOCIATION.json',dict(
        reference=config['reference'],actual_object_coordinate_native=r.actual(),
        measured_base=r.base.tolist(),physical_robot_motion_executed=False,
        object_reset=False,saved_successful_pose_used=False,not_a_fresh_capture=True))
    return result


@contextmanager
def constructor_scope(job):
    config=validate(job['v88_physics_study'])
    import xarm_ft_v7_scene as plant
    import interaction_runtime.registered_observation_v71 as observation
    import interaction_runtime.recovery_candidates_v7 as candidates
    author,install,initialize=plant.author,plant.install_native_law,observation.initialize
    original_order=candidates.order;active_ops=[None];reference_family=None
    if 'reference' in config:
        descriptor=json.loads(Path(config['reference']['path']).read_text())
        fresh=json.loads(Path(descriptor['fresh_reference']['path']).read_text())['candidate']
        reference_family=tuple(float(fresh[k]) for k in
            ('section_fraction','insertion_m','roll_deg','pitch_deg'))
    def author_study(stage,scene,case,output):
        result=author(stage,scene,case,output);author_contact(stage,scene,case,output);return result
    def install_study(g):
        install(g)  # original SI/passive/mass readback and zero-drive checks
        case=json.loads(Path(g['job']['private_physics_configuration']).read_text())[g['job']['identification_case']]
        _,ratio=material_values(case)
        view=g['scene']['articulation']._articulation_view._physics_view
        data=np.asarray(view.get_dof_friction_properties()).copy();dof=g['scene'].get('selected_asset_dof',0)
        index=np.array([0],dtype=np.uint32)
        def update():
            dynamic=plant.law_value(g['actual'](),case['law'])
            data[0,dof,:2]=[dynamic*ratio,dynamic]
            view.set_dof_friction_properties(data,index)
            if g['tick']%240==0 and not np.allclose(view.get_dof_friction_properties(),data,atol=1e-6):
                raise RuntimeError('STUDY_NATIVE_PASSIVE_FRICTION_READBACK_FAILED')
        update()
        from agent_command_protocol_v12 import atomic_json
        atomic_json(Path(g['job']['output'])/'PRIVATE_STUDY_PASSIVE_READBACK.json',dict(
            native_properties=np.asarray(view.get_dof_friction_properties()).tolist(),
            selected_dof=int(dof),object_drive_or_external_effort_added=False))
        return update
    def initial(r,ops):
        active_ops[0]=ops
        return initialize_reference(r,ops,config) if 'reference' in config else initialize(r,ops)
    def matched_order(scored,attempted,preferred=None):
        ops=active_ops[0]
        if reference_family is not None and ops is not None and ops.acquisitions==0:
            # Recompute the same automatically observed family, not a saved T
            # or joint waypoint. Every current native path/closure check runs.
            return original_order(scored,attempted,preferred=reference_family)
        return original_order(scored,attempted,preferred=preferred)
    with patch.object(plant,'author',author_study),patch.object(plant,'install_native_law',install_study),\
            patch.object(observation,'initialize',initial),patch.object(candidates,'order',matched_order):
        yield
