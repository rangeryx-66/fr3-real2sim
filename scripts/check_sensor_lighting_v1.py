"""One render-only A/B scene; no hidden benchmark dynamics, SAM or grasp actions."""
import argparse,json,sys,time
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import numpy as np
    job=json.loads(a.job.read_text());out=a.output.resolve();out.mkdir(exist_ok=False,parents=True)
    job.update(output=str(out),experiment='RENDER_ONLY_DIAGNOSTIC',identification_case=0)
    # Never open the registered private physics file. No randomized plant is
    # needed for a static illumination comparison; no physical interaction.
    dummy=out/'NO_HIDDEN_DYNAMICS.json';dummy.write_text('[{}]');job['private_physics_configuration']=str(dummy)
    for key in ('benchmark_registration','registered_episode_id','preload_fixture'):
        job.pop(key,None)
    (out/'DIAGNOSTIC_JOB.json').write_text(json.dumps(job,indent=2))
    from interaction_runtime.runtime import Runtime
    from interaction_runtime.operations import Operations
    from interaction_runtime.registered_observation_v71 import initial_route
    from interaction_runtime.sensor_lighting_v1 import install,lights
    from native_g1_wrist_input import save_rgbd
    began=time.perf_counter();r=None;result=dict(status='INITIALIZING',grasp_attempts=0,hidden_parameters_read=False)
    try:
        with patch('xarm_ft_v1_scene.author',lambda *args,**kwargs:[]):
            r=Runtime(job,'FAST_DEV')
        ops=Operations(r);route,audit=initial_route(r)
        if route is not None:r.move_path(route['q_path'][1:],'PREGRASP')
        r.hold(r.g['wrist_config']['settle_s'])
        import carb
        settings=carb.settings.get_settings()
        result['render_settings']={k:settings.get(k) for k in ['/rtx/rendermode','/rtx/post/tonemap/op','/rtx/post/histogram/enabled','/rtx/debugMaterialType']}
        result['lights_before']=lights(r.scene['stage'])
        before=save_rgbd(r.g,'lighting_before');q0=np.asarray(r.robot.get_joint_positions()).copy();object0=r.actual()
        result['lighting']=install(r.scene['stage'],ROOT/'configs/sensor_lighting_v1.json')
        repeated=install(r.scene['stage'],ROOT/'configs/sensor_lighting_v1.json')
        result['idempotent_light_count']=len(repeated['lights_after'])==len(result['lighting']['lights_after'])
        after=save_rgbd(r.g,'lighting_after');q1=np.asarray(r.robot.get_joint_positions()).copy()
        import cv2
        stats={}
        for name,folder in [('before',before),('after',after)]:
            rgb=cv2.imread(str(folder/'rgb.png'));gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
            # Image-wide diagnostics only, not GT masks or SAM gates.
            stats[name]=dict(mean_gray=float(gray.mean()),median_gray=float(np.median(gray)),dark_fraction_lt16=float((gray<16).mean()),saturated_fraction_ge250=float((gray>=250).mean()))
        d0=np.load(before/'depth_m.npy');d1=np.load(after/'depth_m.npy');valid=np.isfinite(d0)&np.isfinite(d1)
        result.update(status='RENDER_AB_COMPLETE',initial_route=audit,before=str(before),after=str(after),brightness=stats,
            joints_bit_identical=bool(np.array_equal(q0,q1)),object_coordinate_unchanged=object0==r.actual(),
            depth_finite_mask_equal=bool(np.array_equal(np.isfinite(d0),np.isfinite(d1))),
            max_depth_delta_m=float(np.max(abs(d0[valid]-d1[valid]))) if valid.any() else None)
        result['safe_exit']=ops.release_park();result['wall_s']=time.perf_counter()-began
    except BaseException as error:
        import traceback
        result.update(status='FAILED',error=repr(error),traceback=traceback.format_exc())
    finally:
        (out/'RENDER_RESULT.json').write_text(json.dumps(result,indent=2))
        if r is not None:r.close()
    return 0 if result['status']=='RENDER_AB_COMPLETE' else 1


if __name__=='__main__':sys.exit(main())
