"""Read-only selected-plan board using the episode's declared native robot IO."""
import argparse
import json
import runpy
import sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True)
    args=p.parse_args();episode=args.episode.resolve();root=Path(__file__).resolve().parents[1]
    sys.path[:0]=[str(root),str(root/'scripts')]
    job=json.loads((episode/'job.json').read_text())
    if job.get('package_version')!='8.0.0':raise ValueError('V8_EPISODE_REQUIRED')
    import xarm7_native_gripper
    xarm7_native_gripper.OUT=Path(job['v8_robot_io'])
    sys.argv=['render_agent_grasp_plan.py',str(episode)]
    runpy.run_path(str(root/'scripts/render_agent_grasp_plan.py'),run_name='__main__')
    from .resources import digest,dump
    board=episode/'AGENT_GRASP_PLAN_INSPECTION.png'
    dump(episode/'V8_PLAN_INSPECTION_PROVENANCE.json',dict(image=str(board),sha256=digest(board),
        plan_sha256=digest(episode/'visual_plan.json'),points_sha256=digest(episode/'agent_geometry/points_world.npy'),
        robot_io=str(xarm7_native_gripper.OUT),GT_geometry_used=False,read_only=True))
    print(board)


if __name__=='__main__':main()
