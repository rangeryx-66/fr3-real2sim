"""Post-episode T0/T1/T2 prediction, same issued input, frozen source physics.

These are autonomous full-start replays, not fitted-angle/q trajectory injection.
T0 is explicitly a dataset prior (near oracle), not a visual reconstruction claim.
"""
import copy
import concurrent.futures
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from run_interactive_twin_benchmark import read,write,sha


def prediction_context(job,source,out):
    """Adapt the existing workflow API, including its nonzero-start bake.

    This is post-episode twin setup, never an input to the online controller.
    """
    from interactive_twin_refinement.workflow import source_context
    experiment=SimpleNamespace(source=lambda _: (job,Path(source)),out=Path(out))
    _,_,scene,W,prior=source_context(experiment,'initial')
    return prior,W,scene


def final_object_error(reference_folder,prediction_folder):
    """Post-stop physical proxy transforms, never online simulator feedback."""
    files=[Path(folder)/'cooked_final.json' for folder in (reference_folder,prediction_folder)]
    if not all(f.exists() for f in files):return None
    groups=[]
    for file in files:
        groups.append({s['path'].rsplit('/',1)[-1]:np.asarray(s['world_transform'])
                       for s in read(file)['shapes'] if 'handlepiece' in s['path'].replace('_','').lower()})
    common=sorted(set(groups[0])&set(groups[1]))
    if len(common)!=3:return None
    delta=np.array([groups[0][name][:3,3]-groups[1][name][:3,3] for name in common])
    return {'scope':'final physical handle-proxy centers after complete held-out and final hold; not a full object trajectory',
            'matched_proxy_parts':len(common),'position_rmse_m':float(np.sqrt(np.mean(np.sum(delta*delta,axis=1))))}


def predict(b,summary,replay_gpus=None):
    from interactive_twin.twin import write_twins
    job=summary['selected_job'];source=Path(job['output']);out=source/'structural_prediction';out.mkdir(exist_ok=True)
    result_file=out/'results.json'
    if result_file.exists() and read(result_file).get('complete'):return read(result_file)
    prior,W,scene=prediction_context(job,source,out)
    records=[];versions={}
    for tag,file in [('T1','discovery_articulation.json'),('T2','refined_articulation.json')]:
        dest=out/('compiled_'+tag)
        if not (dest/'twin_versions.json').exists():
            write_twins(prior,dest,read(source/file),W,
                        ee_poses=read(source/'structure_selection.json')['support'])
        compiled=read(dest/'twin_versions.json')['versions']
        versions[tag]=compiled['T1']['asset_root']
        if tag=='T1':versions['T0']=compiled['T0']['asset_root']
    write(out/'T0_T1_T2.json',{'versions':versions,'T0':'dataset structure prior; oracle-like in SIM_TO_SIM',
          'T1':'EE discovery','T2':'robust SE3 refinement','physics_changed':False,'same_commands':sha(source/'command_tape.json')})
    reference=read(source/'observations.json')
    def one(item):
        index,tag=item
        j=copy.deepcopy(job)
        for key in ('structure_protocol','initial_estimate','initial_estimate_memory','mobile_route'):
            j.pop(key,None)
        asset=Path(versions[tag]);j.update(output=str(out/tag),mode='replay',role='prediction',
              asset_root=str(asset),initial_articulation_rad=0.,frozen_proxy_sha256=sha(asset/'manifest.json'),
              replay_commands=str(source/'command_tape.json'),safety_schedule=str(source/'safety_schedule.json'),
              fixed_fixture=scene['fixture'],continue_manipulation=False,refinement_once=False)
        if replay_gpus:j['gpu']=replay_gpus[index%len(replay_gpus)]
        r=b.run(j);row={'model':tag,'status':r['status'],'prediction_type':'same-command autonomous full-start native simulation',
                        'physics_updated':False,'state_replay':False,'GT_used_for_model_selection':False}
        file=Path(j['output'])/'observations.json'
        if file.exists():
            pred=read(file);ref=[(i,x) for i,x in enumerate(reference) if x['phase'] in ('HELDOUT_MANIPULATION','HELDOUT_DWELL')]
            matched=[(x,pred[i]) for i,x in ref if i<len(pred) and pred[i]['phase']==x['phase']]
            row['coverage']=len(matched)/max(1,len(ref));row['full_heldout_complete']=len(matched)==len(ref) and bool(ref)
            if matched:
                A=np.array([x['T_tcp'] for x,y in matched]);B=np.array([y['T_tcp'] for x,y in matched]);t=np.array([x['t'] for x,y in matched])
                dp=A[:,:3,3]-B[:,:3,3];dr=Rotation.from_matrix(A[:,:3,:3]@B[:,:3,:3].transpose(0,2,1)).magnitude()
                row.update(ee_position_rmse_m=float(np.sqrt(np.mean(np.sum(dp*dp,axis=1)))),
                           ee_rotation_rmse_rad=float(np.sqrt(np.mean(dr*dr))),
                           final_displacement_error_m=float(np.linalg.norm((A[-1,:3,3]-A[0,:3,3])-(B[-1,:3,3]-B[0,:3,3]))))
                if len(t)>2:
                    v=np.gradient(A[:,:3,3],t,axis=0)-np.gradient(B[:,:3,3],t,axis=0)
                    row['velocity_rmse_m_s']=float(np.sqrt(np.mean(np.sum(v*v,axis=1))))
                np.savez_compressed(out/(tag+'_heldout.npz'),time_s=t,reference_ee=A,predicted_ee=B)
                if row['full_heldout_complete']:row['post_episode_object_prediction']=final_object_error(source,j['output'])
        return row
    items=list(enumerate(('T0','T1','T2')))
    if replay_gpus:
        if len(set(replay_gpus))!=len(replay_gpus):raise ValueError('DUPLICATE_PREDICTION_GPU')
        # Resource allocation only. One autonomous world per allocated GPU;
        # all model variants receive the identical preregistered command tape.
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(3,len(replay_gpus))) as pool:
            for start in range(0,len(items),len(replay_gpus)):
                for row in pool.map(one,items[start:start+len(replay_gpus)]):
                    records.append(row);write(result_file,{'rows':records,'complete':len(records)==3,'heldout_used_for_selection':False})
    else:
        for item in items:
            records.append(one(item));write(result_file,{'rows':records,'complete':len(records)==3,'heldout_used_for_selection':False})
    return read(result_file)
