"""Piper entry point for the shared AnyGrasp + MoveIt execution backend."""
import os
os.environ.setdefault('GRASP_ROBOT','piper')
from backend import Backend, Failure

class PiperBackend(Backend):
    """Official AgileX Piper model with Piper-specific frames and gripper."""
    pass

if __name__=='__main__':
    import argparse, json
    from collections import Counter
    from backend import RUN
    import rclpy
    p=argparse.ArgumentParser();p.add_argument('--trials',type=int,default=10);a=p.parse_args()
    rclpy.init();node=PiperBackend()
    results=[node.trial(i+1) for i in range(a.trials)]
    report={'robot':'piper','trials':len(results),'successes':sum(r['success'] for r in results),
            'categories':dict(Counter(r['category'] for r in results))}
    (RUN/'summary.json').write_text(json.dumps(report,indent=2));print(report)
    node.destroy_node();rclpy.shutdown()
