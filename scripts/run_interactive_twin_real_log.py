#!/usr/bin/env python3
"""Replay a verified hardware-log bundle in Isaac; never connect to hardware.

Only the frozen Cartesian constrained-drive command contract is supported.
Missing independent servo evidence returns BLOCKED rather than guessing gains.
"""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from interactive_twin.real_log import (validate_bundle,run_real_log_bundle,
    BundleBlocked,_write)
from interactive_twin.sysid import InvalidLog
from run_interactive_twin_benchmark import Benchmark,load_config

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--runtime-config',type=Path,default=ROOT/'configs/interactive_twin.yaml')
    parser.add_argument('--isaac-python');parser.add_argument('--gpu',type=int)
    parser.add_argument('--deadline-shanghai');parser.add_argument('--validate-only',action='store_true')
    args=parser.parse_args()
    if args.validate_only:
        try:
            context=validate_bundle(args.bundle)
            result={'mode':'REAL_LOG_TO_SIM','status':'VERIFIED_BUNDLE_READY_FOR_NATIVE_REPLAY',
                'simulation_launches':0,'hardware_commands_sent':False,
                'reference_source':'real_robot_log','input_sha256':context['identity'],
                'physics_sensitivity_evidence_present':context['sensitivity'] is not None,
                'real_to_sim_success_proven':False}
        except (BundleBlocked,InvalidLog,ValueError,TypeError,KeyError,OSError) as error:
            result={'mode':'REAL_LOG_TO_SIM','status':'BLOCKED_BUNDLE_EVIDENCE','reason':str(error),'simulation_launches':0,'real_to_sim_success_proven':False}
        _write(args.output/'real_log_status.json',result);print(json.dumps(result,indent=2));return 0 if result['status'].startswith('VERIFIED') else 2
    config=load_config(args.runtime_config);config['output']=str(args.output.resolve())
    config['mode']='REAL_LOG_TO_SIM'
    if args.isaac_python:config['runtime']['isaac_python']=args.isaac_python
    if args.gpu is not None:config['runtime']['gpus']=[args.gpu]
    if args.deadline_shanghai:config['runtime']['deadline_shanghai']=args.deadline_shanghai
    orchestrator=Benchmark(config)
    def native_replay(job):
        job={**job,'gpu':config['runtime']['gpus'][0],
             'deadline_shanghai':config['runtime']['deadline_shanghai']}
        return orchestrator.run(job)
    result=run_real_log_bundle(args.bundle,args.output,native_replay)
    print(json.dumps(result,indent=2))
    return 2 if result['status'].startswith('BLOCKED') else 0

if __name__=='__main__':raise SystemExit(main())
