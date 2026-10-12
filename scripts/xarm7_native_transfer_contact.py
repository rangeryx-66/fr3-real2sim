"""One geometry-only change: rank direct normal opening support.

The archived PhysX point impulses are normal impulses, not friction-anchor
forces. The failed grasp relied almost entirely on tangential transmission.
This score prefers an available native pad-normal channel without assuming or
changing friction. It is a candidate utility, never a runtime acceptance gate.
"""
import inspect,types
import xarm7_native_planning as frozen

def contact_quality():
    source=inspect.getsource(frozen.common_quality)
    old="transmission=(normal_pull+np.sqrt(max(0.,1-(c@tangent)**2)))/np.sqrt(2)"
    new="transmission=float(abs(P[LOGICAL[0]][:3,2]@tangent))"
    if source.count(old)!=1:raise ValueError('PRELAUNCH_FROZEN_CONTACT_SCORE_SOURCE_MISMATCH')
    source=source.replace(old,new)
    source=source.replace("'sides':sides}","'sides':sides,'native_pad_normal_opening_component':transmission}")
    source=source.replace("'classification':'NATIVE_GEOMETRY_RANK_ONLY'", "'classification':'NATIVE_DIRECT_NORMAL_OPENING_RANK_ONLY'")
    namespace=dict(frozen.__dict__)
    exec(compile(source,__file__+'::unchanged_geometry_single_score_change','exec'),namespace)
    return namespace['common_quality']

QUALITY=contact_quality()

def plan(job,output,wall_s=300):
    # Reuse the original entire planner, candidate family, collision and Pink
    # branch search. Only its geometric utility callback differs.
    namespace=dict(frozen.plan.__globals__,common_quality=QUALITY)
    adapted=types.FunctionType(frozen.plan.__code__,namespace,frozen.plan.__name__,frozen.plan.__defaults__)
    return adapted(job,output,wall_s)
