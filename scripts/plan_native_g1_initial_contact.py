"""Thin isolated adapter: frozen native planner with one shared ranking change."""
from pathlib import Path
import argparse,json,inspect
import xarm7_native_planning as native
import xarm7_native_transfer_contact as verified
from native_g1_initial_contact_ranking import initial_contact_quality

def plan(job,output,wall_s=300):
    source=inspect.getsource(native.plan)
    source=source.replace("ns['Model']=NativeModel", "ns['initial_contact_quality']=initial_contact_quality;ns['Model']=NativeModel")
    anchor=" ns=dict(shared.__dict__)"
    injection=''' text=text.replace("quality['rank']+=quality['support_score']", "quality['initial_contact_accessibility']=initial_contact_quality(quality);quality['ideal_support_score']=quality['support_score'];quality['support_score']=quality['initial_contact_accessibility']['score'];quality['rank']+=quality['support_score']")
'''
    source=source.replace(anchor,injection+anchor)
    namespace=dict(native.__dict__,initial_contact_quality=initial_contact_quality,common_quality=verified.QUALITY)
    exec(compile(source,__file__+'::unchanged_native_search','exec'),namespace)
    return namespace['plan'](job,output,wall_s)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--job',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--wall-s',type=float,default=300);args=parser.parse_args();plan(json.loads(args.job.read_text()),args.output,args.wall_s)
