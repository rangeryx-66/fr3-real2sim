"""Scene assembly only: isolated benchmark variants of the frozen Isaac plant.

Hidden reference parameters are consumed here, before simulation/control starts.
The controller receives neither these parameters nor object state. No per-step
object torque, joint-position command, or attachment is introduced.
"""
import hashlib
import inspect
import json
from pathlib import Path


def adapt_loader_source(source, job):
    # Isaac's OBJ converter drops valid source parts on invalid-UV errors.
    # Only the import copy's texture references change. The authoritative URDF,
    # FK, collision files, inertials and physical parameters stay unchanged.
    visual_marker = "asset_urdf=a.asset_root/'urdf'/f'{asset_id}.urdf'"
    if source.count(visual_marker) != 1:
        raise RuntimeError('FROZEN_ASSET_URDF_MARKER_CHANGED')
    source = source.replace(visual_marker, visual_marker + "\nfrom interactive_twin.visual_import import compatible_urdf\nasset_import_urdf,visual_import_audit=compatible_urdf(asset_urdf,a.asset_root/'visual_compatibility')")
    source = source.replace('urdf_path=str(asset_urdf)', 'urdf_path=str(asset_import_urdf)')
    old_version = "version=manifest.get('prepared_geometry_sha256',hashlib.sha256(asset_urdf.read_bytes()).hexdigest())[:12]"
    if source.count(old_version) != 1:
        raise RuntimeError('FROZEN_ASSET_CACHE_MARKER_CHANGED')
    source = source.replace(old_version, "version=hashlib.sha256(asset_import_urdf.read_bytes()).hexdigest()[:12]")
    old_cache = "asset_usd = a.asset_root / f'usd/{asset_id}/{asset_id}.usda/{asset_id}/{asset_id}.usda'"
    if source.count(old_cache) != 1:
        raise RuntimeError('FROZEN_ASSET_LEGACY_CACHE_MARKER_CHANGED')
    source = source.replace(old_cache, "asset_usd = a.asset_root/'visual_compatibility'/'no_legacy_visual_cache.usd'")
    initial = float(job.get('initial_articulation_rad', 0.))
    marker = 'world.reset(); camera.initialize()'
    if source.count(marker) != 1:
        raise RuntimeError('FROZEN_LOADER_RESET_MARKER_CHANGED')
    config = job.get('plant', {})
    # Applied once before world.reset. Native friction dissipates energy; unlike a
    # scripted opposing torque it needs no object velocity query in the controller.
    setup = '''\nfrom interactive_twin.plant import author_passive_resistance
plant_resistance_audit=author_passive_resistance(stage,asset_path,PLANT_CONFIGURATION)
from interactive_twin.initial_state import restore_baked_mass_properties
baked_mass_audit=restore_baked_mass_properties(stage,asset_path,asset_urdf,manifest)
'''.replace('PLANT_CONFIGURATION', repr(config))
    source = source.replace(marker, setup + '\n' + marker)
    fixture_statement="world.scene.add(FixedCuboid('/World/cabinet_fixture', name='cabinet_fixture',\n    position=fixture['center'],\n    scale=fixture_size,orientation=fixture_quat,physics_material=mat))"
    if source.count(fixture_statement)!=1:raise RuntimeError('FROZEN_FIXTURE_CREATION_CHANGED')
    source=source.replace(fixture_statement,"if a.fixture_height_m>0.:\n"+'\n'.join('    '+line for line in fixture_statement.splitlines()))
    source=source.replace("+ ['/World/table','/World/cabinet_fixture']","+ [path for path in ['/World/table','/World/cabinet_fixture'] if stage.GetPrimAtPath(path).IsValid()]")
    if job.get('fixed_fixture') is not None:
        old='fixture=fixture_box(manifest,asset_chain,asset_urdf,asset_rotation,asset_xyz,a.fixture_height_m)'
        if source.count(old)!=1:raise RuntimeError('FROZEN_FIXTURE_HOOK_CHANGED')
        source=source.replace(old,'fixture='+repr(job['fixed_fixture']))
    source = source.replace('articulation.set_joint_positions(np.array([0.]))',
                            f'articulation.set_joint_positions(np.array([{initial!r}]))')
    # Robot-only calibration starts at a prescribed safe robot posture. This is
    # initialization, not replay of observed q(t). The object is placed out of reach.
    if job.get('robot_start_q') is not None:
        source = source.replace('HOME = np.array(a.home_q,dtype=float)',
                                'HOME = np.array('+repr(job['robot_start_q'])+',dtype=float)')
    return source


def author_passive_resistance(stage, asset_path, configuration):
    from pxr import UsdPhysics, PhysxSchema, Sdf
    rows=[]
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(asset_path) or not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        if not configuration:
            rows.append({'joint':str(prim.GetPath()), 'mode':'unchanged source prior'})
            continue
        tau_c=float(configuration['tau_c']);b=float(configuration['b'])
        if tau_c<0 or b<0:raise ValueError('NEGATIVE_PASSIVE_RESISTANCE')
        # PhysxSchemaAddition is a multiple-apply schema with skipCodeGeneration.
        prim.AddAppliedSchema('PhysxJointAxisAPI:angular')
        attrs={'staticFrictionEffort':tau_c, 'dynamicFrictionEffort':tau_c,
               'viscousFrictionCoefficient':b*3.141592653589793/180.}
        for key,value in attrs.items():
            prim.CreateAttribute('physxJointAxis:angular:'+key,Sdf.ValueTypeNames.Float).Set(value)
        # Avoid adding a legacy load-dependent friction term to the minimal model.
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr().Set(0.)
        drive=UsdPhysics.DriveAPI.Get(prim,'angular')
        if drive:
            drive.CreateStiffnessAttr().Set(0.);drive.CreateDampingAttr().Set(0.);drive.CreateMaxForceAttr().Set(0.)
        rows.append({'joint':str(prim.GetPath()),'passive_native_schema':'PhysxJointAxisAPI:angular',
                     'tau_c_simulator_nm':tau_c,'b_simulator_nm_s_per_rad':b,'authored':attrs,
                     'static_equals_dynamic':True,'execution_time_object_commands':False})
    if configuration and len(rows)!=1:raise RuntimeError('ONE_PASSIVE_REVOLUTE_PLANT_REQUIRED')
    return rows


def bootstrap_job(args, base, job):
    import piper_mobile_execute as original
    code=inspect.getsource(original.bootstrap)
    # argparse misreads negative scientific notation as another option when it
    # is a separate argv token. Join only these scalar options; retain str(value)
    # exactly, without rounding near-zero placement or changing the legacy file.
    for option,field in (('asset-x','x_m'),('asset-y','y_m'),
                         ('asset-yaw-deg','yaw_deg'),('fixture-height-m','fixture_height_m')):
        separated=f"'--{option}',str(place['{field}'])"
        if code.count(separated)!=1:raise RuntimeError('FROZEN_BOOTSTRAP_SCALAR_ARG_CHANGED:'+option)
        code=code.replace(separated,f"'--{option}='+str(place['{field}'])")
    marker="    exec(compile(source,str(legacy),'exec'),scene);sys.argv=old;"
    if code.count(marker)!=1:raise RuntimeError('FROZEN_BOOTSTRAP_HOOK_CHANGED')
    code=code.replace(marker,"    source=adapt_loader_source(source,benchmark_job)\n"+marker)
    namespace=dict(original.__dict__, adapt_loader_source=adapt_loader_source, benchmark_job=job)
    exec(compile(code,'<interactive_twin_scene_assembly>','exec'),namespace)
    result=namespace['bootstrap'](args,base)
    (args.output/'baked_mass_setup_private.json').write_text(json.dumps(result['baked_mass_audit'],indent=2))
    (args.output/'visual_import_audit.json').write_text(json.dumps(result['visual_import_audit'],indent=2))
    # This private setup audit is not part of the observation/estimator interface.
    (args.output/'plant_setup_private.json').write_text(json.dumps(result['plant_resistance_audit'],indent=2))
    (args.output/'initial_scene_private.json').write_text(json.dumps({'fixture':result['fixture'],'asset_xyz':result['asset_xyz'].tolist(),'asset_rotation':result['asset_rotation'].tolist()},indent=2))
    return result
