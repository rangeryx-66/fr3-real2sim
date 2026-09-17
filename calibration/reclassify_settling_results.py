"""Correct terminal labels from saved measurements without rerunning physics."""
import argparse,json
from pathlib import Path

def revise(result):
    if result.get('success'):return False
    peak=float(result.get('max_lift_m',0));floor=float(result.get('min_hold_lift_m',0))
    samples=result.get('hold_samples',[])
    contact_ok=bool(samples) and all(min(row.get('forces',[0,0]))>.1 for row in samples)
    if contact_ok and peak<.08:
        post=result.get('post_slow_lift',{});metric=post.get('metrics',{});window=metric.get('windows',{}).get('200ms',{})
        thresholds=result.get('thresholds',{})
        converged=(metric.get('terminal_bilateral',False) and
            window.get('relative_translation_velocity_m_s',float('inf'))<=thresholds.get('translation_velocity_m_s',.00025) and
            window.get('relative_angular_velocity_deg_s',float('inf'))<=thresholds.get('angular_velocity_deg_s',.5))
        result.setdefault('original_category',result.get('category'))
        result.update(category='INSUFFICIENT_LIFT' if converged else post.get('category','CONTINUOUS_SLIP'),
                      physical_failure='INSUFFICIENT_LIFT',
                      stability_outcome='SETTLING_THEN_STABLE' if converged else post.get('category'),
                      classification_revision='v3_support_aware_saved_measurements_no_physics_rerun')
        return True
    if contact_ok and peak>=.08 and floor<.08:
        result.update(physical_failure='DROP_AFTER_LIFT')
    return False

def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    changed=[]
    for path in sorted(Path(args.directory).glob('*_settling_r*.json')):
        result=json.loads(path.read_text())
        if revise(result):path.write_text(json.dumps(result,indent=2));changed.append(path.name)
    print(json.dumps({'changed':len(changed),'files':changed},indent=2))

if __name__=='__main__':main()
