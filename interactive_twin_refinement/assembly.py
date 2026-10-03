"""Setup-only physical frames; never exposed to controller/estimator."""
import json
import numpy as np
from scipy.spatial.transform import Rotation

def capture_assembly(scene,path):
    from pxr import UsdGeom,UsdPhysics
    cache=UsdGeom.XformCache();bodies={}
    targets={str(scene['moving_path']),str(scene['contact_target_path'])}
    for prim in scene['stage'].Traverse():
        name=str(prim.GetPath())
        if name not in targets or not prim.HasAPI(UsdPhysics.MassAPI):continue
        mass=UsdPhysics.MassAPI(prim);W=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        com=mass.GetCenterOfMassAttr().Get();inertia=mass.GetDiagonalInertiaAttr().Get();principal=mass.GetPrincipalAxesAttr().Get()
        if com is None or inertia is None or principal is None:continue
        R=Rotation.from_quat([*principal.GetImaginary(),principal.GetReal()]).as_matrix()
        I=W[:3,:3]@R@np.diag(inertia)@R.T@W[:3,:3].T
        bodies[prim.GetName()]={'mass_kg':mass.GetMassAttr().Get(),'world_com_m':(W@np.r_[list(com),1])[:3].tolist(),'world_inertia_kg_m2':I.tolist()}
    path.write_text(json.dumps({'setup_only':True,'controller_access':False,'bodies':bodies},indent=2))

def compare(a,b):
    if set(a['bodies'])!=set(b['bodies']) or not a['bodies']:return {'passed':False,'reason':'MISSING_OR_CHANGED_BODIES'}
    rows=[]
    for key,x in a['bodies'].items():
        y=b['bodies'][key]
        rows.append({'body':key,'mass_error_kg':abs(x['mass_kg']-y['mass_kg']),
          'world_com_error_m':float(np.linalg.norm(np.subtract(x['world_com_m'],y['world_com_m']))),
          'world_inertia_error':float(np.linalg.norm(np.subtract(x['world_inertia_kg_m2'],y['world_inertia_kg_m2'])))})
    return {'passed':all(r['mass_error_kg']<1e-6 and r['world_com_error_m']<2e-6 and r['world_inertia_error']<1e-6 for r in rows),'rows':rows,'tolerance_semantics':'serialization comparison only; no collision tolerance'}


def install_setup_capture():
    """Observe authored MassAPI BEFORE the loader's one-second passive warmup.

    No state write, extra physics step, clamp, or change to native joint dynamics.
    """
    import interactive_twin.plant as plant
    original=plant.adapt_loader_source
    def adapt(source,job):
        source=original(source,job)
        marker='world.reset(); camera.initialize()'
        if source.count(marker)!=1:raise RuntimeError('ASSEMBLY_CAPTURE_RESET_MARKER_CHANGED')
        code="from interactive_twin_refinement.assembly import capture_assembly\ncapture_assembly(globals(),Path(os.environ['R1A7_RUN_DIR'])/'native_authored_assembly_private.json')\n"
        return source.replace(marker,code+marker)
    plant.adapt_loader_source=adapt
