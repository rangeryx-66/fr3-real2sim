from pathlib import Path
import json,hashlib,datetime,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/benchmarkA_30deg_20261011';ranking=json.loads((out/'DISCOVERY_GEOMETRY_RANKING.json').read_text())['ranking']
selected=[('45087','vertical round bar','original-27'),('45176','vertical curved bow','original-46'),('45203','horizontal bow pull','original-53'),('45212','horizontal round bar','original-45'),('45244','vertical curved bow','original-38'),('45267','horizontal round bar','original-25'),('45297','vertical round bar','original-27'),('45384','vertical short bow','original-48'),('45415','bottom horizontal bow pull','original-40'),('45448','horizontal round bar','original-27')]
rows=[]
for i,kind,mesh in selected:
 asset=out/'discovery_assets'/i;m=json.loads((asset/'manifest.json').read_text());geom=next(x for x in ranking if x['asset_id']==i and x['mesh']==mesh);picture=out/'visual_inspection'/i/'oblique_closed.png'
 rows.append({'asset_id':i,'handle_type':kind,'eligibility_handle_mesh':mesh,'asset_root':str(asset),'asset_name':m['object_name'],'height_m':m['prepared_height_m'],'authored_range_rad':m['source_joint_limits_rad'],'source_joint_axis':m['source_joint_axis'],'joint_name':m['joint_name'],'moving_link':m['moving_link'],'eligibility_handle_dimensions_m':geom['dimensions_m'],'eligibility_handle_sections':geom['sections'],'visual_inspection_image':str(picture),'visual_inspection_sha256':hashlib.sha256(picture.read_bytes()).hexdigest(),'urdf_sha256':hashlib.sha256((asset/'urdf'/(i+'.urdf')).read_bytes()).hexdigest(),'manifest_sha256':hashlib.sha256((asset/'manifest.json').read_bytes()).hexdigest(),'original_source_urdf_sha256':m['source_urdf_sha256'],'eligibility_basis':'conventional vertical door hinge; protruding visible handle; authored opening >=30 deg; parallel-jaw size and rear space geometry; no IK or outcome consulted'})
reg={'benchmark':'Benchmark A: Standard hinged doors / 30 degrees','classification':'KNOWN_MODEL_DIAGNOSTIC','registered_at':datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'asset_ids':[x[0] for x in selected],'objects':rows,'success_actual_angle_deg':30.0,'success_requires_valid_physical_grasp':True,'unsafe_pregrasp_motion_not_credited':True,'base_fixed_after_grasp':True,'cabinet_fixed':True,'intentional_releases':0,'regrasps':0,'official_attempts_per_planning_feasible_asset':1,'planning_budget_s_per_object':300,'planning_failures_in_denominator':True,'selection_before_any_IK':True,'development_assets_excluded':['7320','41529','45007','7138'],'old_benchmark_assets_excluded':json.loads((ROOT/'results/revolute10_fixed_20261011/REGISTERED_OBJECTS.json').read_text())['asset_ids'],'selection_not_based_on_predicted_success':True,'preparation':'unchanged existing importer using authored metadata dimensions; existing uniform collision/inertia approximation; mass and joint limits preserved; no new handle proxy or parameter tuning'}
f=out/'REGISTERED_OBJECTS.json'
if f.exists():raise SystemExit('already registered; no overwrite')
f.write_text(json.dumps(reg,indent=2))
fig,axes=plt.subplots(5,2,figsize=(16,32),dpi=115)
for ax,row in zip(axes.flat,rows):
 ax.imshow(plt.imread(row['visual_inspection_image']));ax.axis('off');ax.set_title(row['asset_id']+' / '+row['handle_type'],fontsize=20,fontweight='bold',pad=10)
fig.subplots_adjust(left=.01,right=.99,top=.97,bottom=.01,wspace=.02,hspace=.11);fig.savefig(out/'REGISTERED_CONTACT_SHEET.png',facecolor='white');plt.close(fig)
sourcefiles=[ROOT/'scripts'/n for n in ['revolute10_fixed.py','run_revolute10_fixed.py','run_revolute10_batch.py','piper_pink_fixed.py','run_7320_pink_fixed.py','rank_prepared_handle_assets.py','prepare_physx_microwave.py']]
(out/'FROZEN_EXISTING_PIPELINE_HASHES.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sourcefiles},indent=2))
print('registered',reg['asset_ids']);print('contact sheet saved; no IK run')
