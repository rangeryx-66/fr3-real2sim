from pathlib import Path
import json,zipfile,subprocess,os,concurrent.futures,hashlib
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/benchmarkA_30deg_20261011';rows=json.loads((out/'GEOMETRY_ELIGIBLE_INVENTORY.json').read_text())
exclude={'7320','41529','45007','7138','35059','41004','45091','45166','45621','46744','47686','7236','7306','103082'}
# Discovery only: source type and shape. No IK or physical outcomes used.
pool=[x for x in rows if x['id'] not in exclude and x['name']!='Door Set'][:24]
source=out/'discovery_source';source.mkdir(exist_ok=True);assets=out/'discovery_assets';assets.mkdir(exist_ok=True)
z=zipfile.ZipFile('/data1/home/rangeryx/datasets/physx_mobility/PhysX-Mobility.zip');ids={x['id'] for x in pool}
for n in z.namelist():
 p=Path(n);extract=False
 if '/urdf/' in n and p.stem in ids:extract=True
 if '/finaljson/' in n and p.stem in ids:extract=True
 if '/partseg/' in n and any('/partseg/'+i+'/' in n for i in ids):extract=True
 if extract:z.extract(n,source)
raw=source/'PhysX_mobility';env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
def prepare(row):
 i=row['id'];height=float(row['metadata']['dimension'].split('*')[-1])/100
 log=out/('prepare_'+i+'.log')
 with log.open('w') as stream:
  code=subprocess.run(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/prepare_physx_microwave.py'),'--source',str(raw),'--asset-id',i,'--height-m',str(height),'--output',str(assets/i)],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT).returncode
 if code==0:
  f=assets/i/'manifest.json';m=json.loads(f.read_text());m['object_name']=row['metadata']['object_name'];m['category']=row['metadata']['category'];m['benchmark_preparation']={'method':'unchanged existing generic importer; no per-asset tuning','height_policy':'authored metadata centimetres converted to metres','source_raw_has_no_collision_tags':row['collision_count']==0,'collision_policy':'existing per-visual convex hull importer; no new handle proxy','inertia_policy':'existing importer uniform-box approximation, frozen before selection/planning','source_mass_and_joint_limits_preserved':True,'no_IK_used_for_discovery':True};f.write_text(json.dumps(m,indent=2))
 return {'asset_id':i,'status':code,'metadata_height_m':height,'asset_root':str(assets/i)}
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:result=list(executor.map(prepare,pool))
(out/'DISCOVERY_PREPARATION.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
