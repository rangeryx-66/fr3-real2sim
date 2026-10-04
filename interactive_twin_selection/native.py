"""Optional native static friction setup. No execution-time object command."""
import math


def install():
    import interactive_twin.plant as plant
    original=plant.author_passive_resistance
    def author(stage,asset_path,configuration):
        rows=original(stage,asset_path,configuration)
        if 'tau_s' not in configuration:return rows
        c=float(configuration['tau_c']);s=float(configuration['tau_s'])
        if not math.isfinite(s) or s<c:raise ValueError('STATIC_FRICTION_MUST_BE_AT_LEAST_DYNAMIC')
        for row in rows:
            prim=stage.GetPrimAtPath(row['joint'])
            attr=prim.GetAttribute('physxJointAxis:angular:staticFrictionEffort')
            if not attr.IsValid():raise RuntimeError('NATIVE_STATIC_FRICTION_SCHEMA_MISSING')
            attr.Set(s)
            row.update(tau_s_simulator_nm=s,static_equals_dynamic=s==c,static_extension_native_only=True)
            row['authored']['staticFrictionEffort']=s
        return rows
    plant.author_passive_resistance=author
