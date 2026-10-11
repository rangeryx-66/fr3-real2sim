"""Benchmark A task target adapter. All existing search/check logic is frozen."""
import argparse, inspect, json
from pathlib import Path
import revolute10_fixed as shared

source=inspect.getsource(shared.plan)
source=source.replace("goal = float(meta['source_joint_limits_rad']['upper'])",
                      "goal = float(job['benchmark_goal_rad'])",1)
namespace=dict(shared.__dict__)
exec(compile(source,str(Path(shared.__file__))+'::benchmark_30deg_target','exec'),namespace)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--job',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--wall-s',type=float,default=300)
    args=parser.parse_args();namespace['plan'](json.loads(args.job.read_text()),args.output,args.wall_s)
