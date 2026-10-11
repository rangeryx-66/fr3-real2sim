"""One shared official attempt: unchanged fixed-base Pink contact execution.

Only object goal and original collider association are adapted. No retry, mobile
motion, release, regrasp, template fallback or new execution checks.
"""
import sys,json,time
from pathlib import Path
from piper_pink_fixed import PinkIK
from revolute10_fixed import authored_handle_colliders

spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
baseline=Path(spec['baseline_runner']);text=baseline.read_text()
text=text.replace("allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]",
                  "from revolute10_fixed import authored_handle_colliders\nallowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])")
text=text.replace("if len(allowed)!=len(model.manifest['interaction_geometry']['pieces']):raise RuntimeError('FROZEN_PROXY_ASSOCIATION_FAILED')",
                  "diagnostic_write(a.output/'authored_collider_association.json',json.dumps({'allowed':allowed,'geometry_changed':False}))")
text=text.replace("existing bar box plus two support primitives; approximate, not calibrated real geometry",
                  "unchanged authored dataset collision meshes; no proxy replacement")
setup,sequence=text.split("try:\n phase='SETTLE';hold(.5)",1)
exec(compile(setup,str(baseline)+'::shared_setup','exec'),globals())
solver=PinkIK(ROOT/'config/piper.urdf',model.home);model.ik=solver.solve
original_step=step;grasp_relative=None;ref=None;twist=np.zeros(6)
goal=float(whole['goal_state']);motion_scale=float(spec.get('pink_motion_scale',1.))
angle_reference=0.;bias=None;seed=None;opening=False

# Execute the previous Pink controller literally, from its immutable snapshot.
pink_source=Path(spec['pink_runner']).read_text()
controller_source=pink_source.split('def body_at(theta):',1)[1].split('initial_grasp=False',1)[0]
exec(compile('def body_at(theta):'+controller_source,spec['pink_runner']+'::shared_control','exec'),globals())
initial_grasp=False;initial_actual_angle=None
try:
    initial="if True:\n phase='SETTLE';hold(.5)"+sequence.split("  profile_mark('OPEN_1')",1)[0]
    exec(compile(initial,str(baseline)+'::unchanged_initial_grasp','exec'),globals());initial_grasp=legal
    initial_actual_angle=rows[0]['door_angle_deg']
    grasp_relative=np.linalg.inv(moving())@tcp();reference=grasp_relative.copy();ref=tcp().copy()
    seed=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-seed;angle_reference=actual();opening=True
    diagnostic_write(a.output/'acquired_grasp_reference.json',json.dumps({'T_body_tcp':grasp_relative.tolist(),'base':base,'q':seed.tolist(),'t':tick*dt}))
    while actual()<goal:
        phase='FIXED_BASE_PINK_OPEN';step()
    opening=False;qvelocity=np.zeros(6);phase='FINAL_HOLD';hold(.5)
    status='AUTHORED_TARGET_REACHED_SINGLE_GRASP';success=True
except BaseException as error:
    import traceback
    status=repr(error);print(traceback.format_exc(),flush=True);opening=False;qvelocity=np.zeros(6)
world.pause()
for label,data in [('observations',rows),('physics_steps',native.physics_steps)]:
    try:export_raw_records(label,data)
    except Exception as error:diagnostic_issue(label,error)
try:video.stdin.close();video.wait(timeout=30)
except Exception as error:diagnostic_issue('video',error)
result={'classification':'KNOWN_MODEL_DIAGNOSTIC','asset_id':model.manifest['asset_id'],'official_attempt':1,
        'target_deg':float(np.rad2deg(goal)),'success':success,'target_reached':success,'status':status,
        'max_actual_angle_deg':max_state,'final_actual_angle_deg':float(np.rad2deg(actual())),
        'initial_actual_angle_deg':rows[0]['door_angle_deg'] if rows else None,'initial_grasp':initial_grasp,
        'base_fixed':list(base),'base_moves_after_grasp':0,'regrasps':0,'intentional_releases':0,
        'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'physics_steps':len(native.physics_steps),
        'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=0),
        'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),
        'IK':'Pinocchio3.9.0/Pink3.3.0/DAQP','motion_scale':motion_scale,'solver_calls':solver.calls,
        'telemetry_issues':telemetry_issues,'first_divergence':first_divergence,'pid':os.getpid(),
        'grasp_generation':'automatic current-asset geometry; no successful-object GT grasp template',
        'GT_dependencies':['authored geometry and moving-body/articulation feedback','collision model'],
        'object_physics_or_geometry_changed':False,'shared_execution_logic':True}
diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))
diagnostic_write(a.output/'events.json',json.dumps(telemetry_events,indent=2))
print('OFFICIAL_RESULT',json.dumps(result),flush=True)
app.close()
