"""Isolated thin ranking adapter; original approach/collision/Pink search stays intact."""
from pathlib import Path
import argparse,json,inspect,numpy as np
import xarm7_native_planning as native
import xarm7_native_transfer_contact as verified
from native_g1_robust_contact_ranking import rescore_geometries

def plan(job,output,wall_s=450):
    source=inspect.getsource(native.plan)
    source=source.replace("ns['Model']=NativeModel", "ns['rescore_geometries']=rescore_geometries;ns['Model']=NativeModel")
    injection=''' text=text.replace("    geometries=[]\\n    for section", "    geometries=[]\\n    cached_geometry=job.get('cached_closing_geometry_audit')\\n    for section")
 text=text.replace("for section in sorted(selection['sections'],key=lambda s:abs(s['fraction'])):", "for section in ([] if cached_geometry else sorted(selection['sections'],key=lambda s:abs(s['fraction']))):")
 anchor="    geometries.sort(key=lambda g:g['contact']['rank'],reverse=True)"
 robust="""    if cached_geometry:
        geometries=json.loads(Path(cached_geometry).read_text())
        for g in geometries:g['T']=np.asarray(g['T'])
    config=json.loads(Path(job['robust_uncertainty_config']).read_text())
    J=model.asset_T@model.asset.root_to_link(link,{joint:0.})
    hinge_axis=J[:3,:3]@model.asset.joints[link].axis
    geometries=rescore_geometries(geometries,model,surfaces,initial,J[:3,3],hinge_axis,config,output)
"""
 text=text.replace(anchor,robust+anchor)
'''
    source=source.replace(" ns=dict(shared.__dict__)",injection+" ns=dict(shared.__dict__)")
    namespace=dict(native.__dict__,common_quality=verified.QUALITY,rescore_geometries=rescore_geometries)
    exec(compile(source,__file__+'::shared_search_ranking_adapter','exec'),namespace)
    return namespace['plan'](job,output,wall_s)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wall-s',type=float,default=450);a=p.parse_args();plan(json.loads(a.job.read_text()),a.output,a.wall_s)
