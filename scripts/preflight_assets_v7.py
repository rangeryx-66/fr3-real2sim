"""Inexpensive asset inventory/preparation. No Isaac, dynamics or grasp outcomes.

The provisional list is not a final registration. Visual/kinematic/collision
checks must be completed and exclusions recorded before freezing the batch.
"""
import argparse,hashlib,json,subprocess,sys,zipfile,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def inventory(archive):
    rows=[]
    with zipfile.ZipFile(archive) as z:
        names=set(z.namelist())
        for name in sorted(names):
            if not name.endswith('.urdf'):continue
            aid=Path(name).stem;mp='PhysX_mobility/finaljson/'+aid+'.json'
            if mp not in names:continue
            r=ET.fromstring(z.read(name));meta=json.loads(z.read(mp));js=[j for j in r.findall('joint') if j.get('type')!='fixed']
            drawer=any('drawer' in p.get('name','').lower() for p in meta.get('parts',[]))
            ordinary=('cabinet' in meta.get('object_name','').lower() or 'wardrobe' in meta.get('object_name','').lower() or (drawer and any(k in meta.get('object_name','').lower() for k in ['table','desk','drawer'])))
            if not ordinary:continue
            reasons=[]
            if len(js)!=1:reasons.append('requires explicit selected-DOF/state mapping; other joints must not be silently locked')
            selected=js[0] if len(js)==1 else None
            if selected is not None:
                lim=selected.find('limit');kind=selected.get('type');axis=np.fromstring(selected.find('axis').get('xyz'),sep=' ')
                if kind not in ['revolute','prismatic']:reasons.append('unsupported articulation')
                if lim is None or not float(lim.get('lower'))<=0<=float(lim.get('upper')):reasons.append('normal closed zero outside authored limits')
                if kind=='prismatic' and (not drawer or abs(axis[1])>1e-3):reasons.append('not a horizontal drawer in source +Y-up coordinates')
                if kind=='revolute' and abs(abs(axis[1])-1)>1e-3:reasons.append('nonvertical hinge deferred')
            rows.append(dict(asset_id=aid,object_name=meta.get('object_name'),dimension_cm=meta.get('dimension'),joint_count=len(js),joint_types=[j.get('type') for j in js],raw_collision_elements=len(r.findall('link/collision')),metadata_exclusions=reasons,source_urdf_sha256=hashlib.sha256(z.read(name)).hexdigest()))
    return rows


def prepare_drawers(archive,out,ids):
    raw=out/'source_library';raw.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        names=z.namelist()
        for aid in ids:
            prefix='PhysX_mobility/'
            for name in names:
                if name.endswith('/'):continue
                if name in [prefix+'urdf/'+aid+'.urdf',prefix+'finaljson/'+aid+'.json'] or name.startswith(prefix+'partseg/'+aid+'/'):
                    p=raw/name[len(prefix):];p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
            meta=json.loads((raw/'finaljson'/f'{aid}.json').read_text());height=float(meta['dimension'].split('*')[-1])/100
            target=out/'geometry_assets'/aid
            with (out/(aid+'_preparation.log')).open('w') as log:
                tree=ET.parse(raw/'urdf'/f'{aid}.urdf').getroot();selected=next(j for j in tree.findall('joint') if j.get('type')=='prismatic')
                subprocess.run([sys.executable,str(ROOT/'scripts/prepare_physx_microwave.py'),'--source',str(raw),'--asset-id',aid,'--height-m',str(height),'--joint-type','prismatic','--joint-name',selected.get('name'),'--output',str(target)],stdout=log,stderr=subprocess.STDOUT,check=True)


def render_preflight(out,assets):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from piper_mobile_demo.model import geometry
    from articulated_demo.kinematics import URDFChain
    details=[]
    for asset in assets:
        m=json.loads((asset/'manifest.json').read_text());urdf=asset/'urdf'/(m['asset_id']+'.urdf');tree=ET.parse(urdf).getroot();chain=URDFChain(urdf)
        fig,axes=plt.subplots(1,2,figsize=(12,6));moving=set(m['moving_links']);meshes=[]
        for link in tree.findall('link'):
            for vis in link.findall('visual'):
                mesh=geometry(vis,urdf.parent);mesh.apply_transform(chain.root_to_link(link.get('name'),{}));meshes.append((link.get('name'),Path(vis.find('geometry/mesh').get('filename')).stem,mesh))
        for col,(az,elev) in enumerate([(0,0),(35,18)]):
            ax=axes[col];a=np.deg2rad(az);e=np.deg2rad(elev);R=np.array([[np.cos(a),0,-np.sin(a)],[np.sin(a)*np.sin(e),np.cos(e),np.cos(a)*np.sin(e)]])
            faces=[];colors=[];depth=[]
            for link,stem,mesh in meshes:
                p=mesh.vertices@R.T
                highlight=stem in m['grasp_mesh_stems'];faces.extend(p[mesh.faces]);colors.extend(['#e26731' if highlight else ('#65afd7' if link in moving else '#b5b8bb')]*len(mesh.faces));depth.extend(mesh.triangles_center@np.cross(R[0],R[1]))
                if highlight:ax.text(*np.median(p,axis=0),stem,color='darkred',fontsize=9)
            order=np.argsort(depth);ax.add_collection(PolyCollection(np.asarray(faces)[order],facecolors=np.asarray(colors)[order],edgecolors='none'));ax.autoscale_view()
            ax.set_aspect('equal');ax.set_title(f"{m['asset_id']} | {m.get('joint_type','revolute')} | source closed geometry");ax.set_xlabel('Source x');ax.set_ylabel('Source up / projected up')
        fig.tight_layout();fig.savefig(out/(m['asset_id']+'_geometry.png'),dpi=120);plt.close(fig)
        joints=[j for j in tree.findall('joint') if j.get('type')!='fixed'];joint=tree.find("joint[@name='%s']"%m['joint_name'])
        details.append(dict(asset_id=m['asset_id'],asset_root=str(asset),joint_type=joint.get('type'),joint_name=joint.get('name'),active_joint_count=len(joints),axis_source=joint.find('axis').get('xyz'),limits_native={k:float(joint.find('limit').get(k)) for k in ['lower','upper']},unit='m' if joint.get('type')=='prismatic' else 'rad',moving_link=m['moving_link'],contact_body=m['door_link'],provisional_handle_meshes=m['grasp_mesh_stems'],collision_count=len(tree.findall('link/collision')),geometry_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),status='VISUAL_AND_IK_REVIEW_PENDING'))
    return details


def main():
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rows=inventory(a.archive);(a.output/'LIBRARY_PREFLIGHT.json').write_text(json.dumps(rows,indent=2))
    # Geometry-diverse provisional drawers: compact desk through broad table.
    # No controller/planner/contact scores or new physics outcomes select them.
    ids=['19179','45290','27044','45746'];prepare_drawers(a.archive,a.output,ids)
    doors=[ROOT/'results/native_g1_agent_transfer_20261015'/aid/'asset' for aid in ['45936','46427']]+[ROOT/'results/native_g1_visual_reliability_20261015'/aid/'asset' for aid in ['45600','45606']]
    assets=doors+[a.output/'geometry_assets'/aid for aid in ids]
    excluded=[dict(asset_id=x,reason='protruding round knob; deferred because current frozen bar-handle candidate family does not represent this geometry') for x in ['20411','20555','22508','26525']]
    excluded += [dict(asset_id='45132',reason='closed-state visual preflight shows round knob rather than bar; deferred before any contact outcome'),dict(asset_id='20985',reason='prepared width/height proportion inconsistent with ordinary compact-table mounting; provisional most-front mesh is not the bar'),dict(asset_id='29525',reason='short rounded contact geometry deferred in favor of clearly observable standard bars; no physical outcome observed')]
    details=render_preflight(a.output,assets);(a.output/'PROVISIONAL_CASES.json').write_text(json.dumps(dict(status='NOT_FROZEN_NO_PHYSICS',cases=details,selection='existing standard door geometries and ordinary bar-handle drawers; no physical score selection',prephysical_exclusions=excluded,recovery='fresh wrist/contact Agent inspection and feasible-candidate regeneration within same scene; safe exit before slow reasoning'),indent=2))
    print(json.dumps(dict(prepared=len(details),metadata_candidates=len(rows),output=str(a.output))))


if __name__=='__main__':main()
