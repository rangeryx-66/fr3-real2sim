"""Rule-first fallback at the existing observed-geometry/native-planner seam."""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
from .grasp_candidates import geometry_candidates,novel_shortlist,pose_id


def physical_request(command):
    """Transport sequence/reworded reasoning cannot make a failed pose novel."""
    return {k:v for k,v in command.items()if k not in (
        'operation','protocol_version','sequence','reason','inspected_image_sha256')}


def snapshot(ops,directory):
    """Minimal native replay state; no authored physical parameters."""
    directory.mkdir(parents=True,exist_ok=True);r=ops.r
    data=dict(episode_id=r.job['episode_id'],instance_uuid=r.job.get('instance_uuid'),
        object_coordinate_native=float(r.actual()),moving_contact_body=r.model.manifest['door_link'],
        T_world_moving_contact_body=np.asarray(r.moving()).tolist(),base=r.base.tolist(),
        robot_joint_names=r.names,measured_robot_q=np.asarray(r.robot.get_joint_positions()).tolist(),
        retained_arm_equilibrium_q=r.qtarget[r.arm].tolist(),
        commanded_base_joint_goals=np.asarray(r.base_goal).tolist()if hasattr(r,'base_goal')else None,
        jaw_requested_opening_m=float(r.jaw_width),arm_indices=r.arm.tolist(),finger_indices=r.fingers.tolist(),
        simulation_s=r.tick*r.dt,world_simulation_s=float(r.world.current_time),original_safety_latch=r.supervisor.latched,
        source='native measured state and retained command at same unloaded planning boundary',
        authored_mass_or_resistance_read=False)
    path=directory/'NATIVE_CONTEXT.json'
    if not path.exists():path.write_text(json.dumps(data,indent=2)+'\n')
    import shutil
    for source,target in [('EXECUTION_CANDIDATE_AUDIT.json','RULE_NATIVE_AUDIT.json'),
            ('visual_planning/candidate_audit.json','RULE_ARC_AUDIT.json'),
            ('visual_planning/robust_geometry_audit.json','RULE_RANK_AUDIT.json'),
            ('planning_current_scene.json','NATIVE_COLLISION_SCENE.json')]:
        src=r.output/source;dst=directory/target
        if src.exists()and not dst.exists():shutil.copy2(src,dst)


def generate(ops,folder,directory):
    from native_g1_metric_rgbd_geometry import preserve
    from .graspgenx_geometry import derive
    directory.mkdir(parents=True,exist_ok=True)
    camera=json.loads((folder/'camera.json').read_text());C=np.asarray(camera['T_world_camera_optical'])
    source=ops.r.output/'agent_geometry/contact_supported_points_world.npy'
    if not source.exists():source=ops.r.output/'agent_geometry/points_world.npy'
    if source.exists():P=np.load(source)
    else:P=np.asarray(preserve(folder)['points'])
    points=(P-C[:3,3])@C[:3,:3]
    np.savez_compressed(directory/'input.npz',points=points,mask=np.ones(len(points),bool),T_B_C=C)
    provenance=dict(point_source=str(source)if source.exists()else str(folder),
        point_source_scope='measured current-generation wrist surfaces only; no rear completion or asset surface',
        points_available=len(points),camera_sha256=hashlib.sha256((folder/'camera.json').read_bytes()).hexdigest(),
        input_sha256=hashlib.sha256((directory/'input.npz').read_bytes()).hexdigest(),
        deterministic_inference_point_cap=8192)
    (directory/'INPUT_PROVENANCE.json').write_text(json.dumps(provenance,indent=2)+'\n')
    geometry=derive(Path(ops.r.job['v8_robot_io'])/'NATIVE_CONFIG.json')
    (directory/'G1_GEOMETRY.json').write_text(json.dumps(geometry,indent=2)+'\n')
    config=ops.r.job['v87_graspgenx'];root=Path(__file__).resolve().parents[1]
    command=[config['python'],'-u','-m','agentic_interaction_v87.infer_graspgenx',
        '--native-geometry',str(directory/'G1_GEOMETRY.json'),'--source',config['source'],
        '--input',str(directory/'input.npz'),'--output',str(directory/'candidates.json'),
        '--num-grasps','200','--top-k','200','--max-input-points','8192']
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'),
        CUDA_VISIBLE_DEVICES=str(config['gpu']),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
    with (directory/'inference.log').open('xb')as log:
        result=subprocess.run(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
    if result.returncode:raise RuntimeError('GRASPGENX_WORKER_FAILED:'+str(result.returncode))
    return json.loads((directory/'candidates.json').read_text()),camera


def operations_class(parent):
    class HybridOperations(parent):
        def request_plan(self,command):
            rows=[r for r in self.g['agent_observations']if r['accepted_for_geometry']]
            if not rows or command.get('reuse_current_candidates'):return super().request_plan(command)
            folder=Path(rows[-1]['folder'])
            if rows[-1]['world_sim_s']<getattr(self,'_last_contact_finished_world_s',0):return super().request_plan(command)
            # Exact same observation/state/declaration never reruns a completed
            # unsuccessful fallback. A new view, actual motion or declaration
            # is needed; failed local candidates do not end the episode.
            identity=dict(frame=hashlib.sha256((folder/'camera.json').read_bytes()).hexdigest(),
                mask=hashlib.sha256((folder/'handle_mask.npy').read_bytes()).hexdigest(),
                object_q=float(self.r.actual()),base=self.r.base.tolist(),
                equilibrium_command=self.r.qtarget.tolist(),resources=self.r.job['v87_graspgenx'],command=physical_request(command))
            key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
            cache=getattr(self,'_v87_grasp_searches',{})
            if key in cache:
                return dict(plan=False,recoverable=True,status='PLANNING_REJECTED',
                    reason='SAME_OBSERVATION_AND_DECLARATION_SEARCH_ALREADY_EXHAUSTED',graspgenx_fallback=cache[key],
                    next_search_requires_changed_observation_state_or_actions=True)
            result=super().request_plan(command)
            directory=self.r.output/'hybrid_grasp'/key
            try:snapshot(self,directory)
            except (OSError,TypeError)as error:
                result['optional_native_snapshot_error']=repr(error)
            if result.get('plan')or not result.get('geometry'):return result
            started=time.perf_counter()
            report=dict(identity=identity,rules_result=result,native_predicates_unchanged=True,
                physical_grasp_not_claimed=True)
            try:
                payload,camera=generate(self,folder,directory)
                import native_g1_multiview_input as multi
                import native_g1_robust_contact_ranking as ranking
                original=multi.rescore;generated=[]
                data=json.loads((self.r.output/'agent_geometry/observed_geometry.json').read_text())
                def rescore(geometries,model,surfaces,base,origin,axis,config,output):
                    candidates=geometry_candidates(payload,camera,data,model,surfaces,
                        multi.closing_scene(model,self.r.output/'agent_geometry'),base,np.asarray(origin),axis)
                    generated.extend(candidates)
                    with patch.object(ranking,'diverse_shortlist',novel_shortlist):
                        return original(candidates,model,surfaces,base,origin,axis,config,output)
                # The parent still installs the exact existing CandidateCheck,
                # whole-chain/closure/return certificates, score and ticket.
                with patch.object(multi,'rescore',rescore):fallback=super().request_plan(command)
                if not generated:raise ValueError('GRASPGENX_FALLBACK_SEAM_NOT_EXERCISED')
                report.update(generated_count=len(generated),fallback_result=fallback)
                if fallback.get('plan'):
                    p=self.r.output/'visual_plan.json';selected=json.loads(p.read_text())
                    proposal=next((g for g in generated if np.allclose(g['T'],selected['T'],atol=1e-8)),None)
                    if proposal is None:raise ValueError('SELECTED_GRASP_PROVENANCE_MISMATCH')
                    # Store provenance alongside the already ticketed plan;
                    # adding fields to that plan would invalidate its hash.
                    report['selected_pose_id']=proposal['pose_id']
                    fallback['candidate_provider']='GraspGenX_NATIVE_G1'
                result=fallback
            except (RuntimeError,ValueError,OSError,subprocess.TimeoutExpired)as error:
                report['worker_or_planning_error']=repr(error)
            report['wall_s']=time.perf_counter()-started
            directory.mkdir(parents=True,exist_ok=True)
            (directory/'SEARCH_REPORT.json').write_text(json.dumps(report,indent=2)+'\n')
            saved=dict(report=str(directory/'SEARCH_REPORT.json'),
                generated_count=report.get('generated_count',0),plan=bool(result.get('plan')))
            if not result.get('plan'):cache[key]=saved
            self._v87_grasp_searches=cache
            return dict(result,graspgenx_fallback=saved)
    return HybridOperations
