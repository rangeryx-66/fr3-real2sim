"""Reconstruct *issued* robot/base commands. Never restore object state."""
import json
from pathlib import Path
import numpy as np

def route_commands(route,dt):
    result=[]
    for a,b in zip(route['waypoints'][:-1],route['waypoints'][1:]):
        a=np.asarray(a,float);b=np.asarray(b,float);delta=b-a
        duration=max(float(np.linalg.norm(delta[:2]))/.01,abs(float(delta[3]))/5.,dt)
        result.extend((a+f*delta).tolist() for f in np.linspace(0,1,max(2,int(duration/dt)))[1:])
    return result

def prepare(source,output,dt=1/240,stop_t=None):
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    planfile=source/'mobile_wrist_planning.json';plans=json.loads(planfile.read_text()) if planfile.exists() else [];routes=[]
    for entry in plans:
        selected=[c['route'] for c in entry.get('candidates',[]) if c.get('status')=='MOBILE_VIEW_PREFLIGHT_PASSED' and c.get('route',{}).get('valid')]
        if len(selected)>1:raise RuntimeError('AMBIGUOUS_EXECUTED_BASE_ROUTE')
        routes.extend(selected)
    commands=[];route_index=0;active=None;offset=0;old_phase=None
    with (source/'command_tape.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if stop_t is not None and row['t']>=stop_t:break
            if row['phase']=='SYSTEM_BASE_ROUTE':
                if old_phase!='SYSTEM_BASE_ROUTE':
                    if route_index>=len(routes):raise RuntimeError('BASE_ROUTE_COMMAND_PROVENANCE_MISSING')
                    active=route_commands(routes[route_index],dt);route_index+=1;offset=0
                if offset>=len(active):raise RuntimeError('BASE_ROUTE_COMMAND_COUNT_MISMATCH')
                row['base_command']=active[offset];offset+=1
            elif old_phase=='SYSTEM_BASE_ROUTE':
                # The original route finishes with one-second lock hold, phase
                # remains BASE_ROUTE until its final driver statement.
                if offset<len(active):raise RuntimeError('INCOMPLETE_BASE_ROUTE_REPLAY_SOURCE')
            old_phase=row['phase'];commands.append(row)
    # Commands during final failed recovery are retained, not measured q.
    (output/'issued_robot_commands.json').write_text(json.dumps(commands))
    provenance={'source':str(source),'commands':len(commands),'executed_routes':route_index,'object_state_replay':False,'measured_q_replay':False,'contact_impulse_copy':False,'robot_base_commands':'original recorded route interpolation, 0.01m/s and 5deg/s','source_kind':'issued robot position/velocity/effort and Cartesian inputs'}
    (output/'continuation_provenance.json').write_text(json.dumps(provenance,indent=2))
    return provenance


def make_job(source,prepared,output,config,deadline,checkpoint_path=None):
    import copy,shutil
    source=Path(source);prepared=Path(prepared);output=Path(output)
    job=json.loads((source/'frozen_wrist_job.json').read_text());checkpoint_path=Path(checkpoint_path) if checkpoint_path else source/'max_range_checkpoint.json';checkpoint=json.loads(checkpoint_path.read_text())
    template=json.loads((source/'successful_grasp_template.json').read_text())
    H0=np.asarray(template['T_world_handle_at_success']);H1=np.asarray(checkpoint['observed_handle']['T_world_handle'])
    observed=copy.deepcopy(checkpoint['observed_handle'])
    origin={'D_world_initial_to_checkpoint':(H1@np.linalg.inv(H0)).tolist(),'visual_checkpoint':observed,'source':'saved calibrated RGB-D handle registration; not object joint state'}
    job.update(output=str(output.resolve()),deadline_shanghai=deadline,wall_clock_budget_s=config['capture']['wall_s'],wrist_experiment=config,system_capture=config['capture'],continuation_replay=True,replay_commands=str((prepared/'issued_robot_commands.json').resolve()),continuation_memory=str((source/'structured_memory.json').resolve()),continuation_checkpoint=str(checkpoint_path.resolve()),continuation_template=str((source/'successful_grasp_template.json').resolve()),continuation_observed_origin=origin,continuation_source=str(source.resolve()),episode_id='continued_'+job['episode_id'])
    memory=json.loads((source/'structured_memory.json').read_text())
    if len(memory.get('supporting_observations',[]))<12:job['continuation_memory']=job['operation_memory']
    job['skill'].update(maximum_segments=config['capture']['maximum_segments'],maximum_sim_s=config['capture']['maximum_sim_s'],maximum_path_m=config['capture']['task_path_m'])
    job['resume_closed_capture']=[str(source.resolve()),*job.get('resume_closed_capture',[])]
    output.mkdir(parents=True,exist_ok=True)
    # Reuse exact observations from the source command episode. Their original
    # timestamps, real robot poses and provenance remain unchanged.
    manifest=source/'multistate_capture.json'
    if manifest.exists():
        d=json.loads(manifest.read_text());states=[]
        for state in d.get('states',[]):
            if not state.get('clean_wrist_capture'):continue
            for v in state['views']:
                src=source/v['directory'];dst=output/v['directory'];dst.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(src,dst,dirs_exist_ok=True)
                v.update(acquired_in_current_execution=False,acquisition_reused_from=str(src))
            states.append(state)
        job['inherited_capture_states']=states
    (prepared/'continuation_job.json').write_text(json.dumps(job,indent=2));return job
