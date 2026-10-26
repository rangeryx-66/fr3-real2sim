"""Measured split-GPU resource admission; no controller/safety limit changes."""
import hashlib,json,os,subprocess
from pathlib import Path

COMBINED_MIB=12000
RESERVE_MIB=1024
INITIALIZE_ONLY_MIB=4096


def evidence(registration,checkpoint,gpu):
    p=Path(registration['path']);raw=p.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=registration['sha256']:raise ValueError('GPU_CAPACITY_EVIDENCE_CHANGED')
    data=json.loads(raw);peak=float(data['peak_process_memory_MiB'])
    if (data['returncode']!=0 or not data['original_worker_unmodified']or data['gpu']!=gpu
            or data['checkpoint_sha256']!=checkpoint or data['image_resolution']!=[1280,960]
            or not 0<peak<COMBINED_MIB-RESERVE_MIB):
        raise ValueError('MATCHED_SUCCESSFUL_SAM_MEMORY_MEASUREMENT_REQUIRED')
    return data


def admission(free,physics,perception,measured):
    if physics==perception:raise ValueError('MEASURED_SPLIT_REQUIRES_DISTINCT_GPUS')
    peak=float(measured['peak_process_memory_MiB'])
    # Decompose the ORIGINAL combined 12GB reservation using the measured
    # unchanged SAM peak; do not invent a smaller combined requirement.
    physics_budget=COMBINED_MIB-peak
    # This grants ONLY initialization and the original no-load check. Actual
    # native allocation plus reserves must pass confirm_initialized() before
    # even the first wrist route or contact operation can start.
    if free[physics]<min(INITIALIZE_ONLY_MIB,physics_budget) or free[perception]<peak+RESERVE_MIB:
        raise ValueError('INSUFFICIENT_MEASURED_SPLIT_GPU_CAPACITY')
    return dict(physics_gpu=physics,perception_gpu=perception,
        original_combined_budget_MiB=COMBINED_MIB,physics_budget_MiB=physics_budget,
        measured_SAM_peak_MiB=peak,perception_reserve_MiB=RESERVE_MIB,
        initialization_only_reservation_MiB=INITIALIZE_ONLY_MIB,
        free_before_MiB=free,initialized_native_qualification_required=True,
        interaction_authorized_before_native_measurement=False)


def before_launch(job,profile,free):
    measured=evidence(job['v87_gpu_capacity_evidence'],profile['sam_checkpoint_sha256'],profile['perception_gpu'])
    return admission(free,int(profile['physics_gpu']),int(profile['perception_gpu']),measured)


def confirm_initialized(r):
    from agent_command_protocol_v12 import atomic_json
    from interaction_runtime.policy import SafetyStop
    proof=json.loads((r.output/'V87_GPU_CAPACITY_PRELAUNCH.json').read_text())
    initial=json.loads((r.output/'UNLOADED_ADMISSION.json').read_text())
    if initial.get('passed')is not True:raise SafetyStop('RESOURCE_CHECK_REQUIRES_ORIGINAL_NO_LOAD_PASS')
    before=float(r.world.current_time);r.world.render()
    if float(r.world.current_time)!=before:raise SafetyStop('RENDER_QUALIFICATION_ADVANCED_PHYSICS')
    free={int(a):int(b)for a,b in (line.split(',')for line in subprocess.check_output(
        ['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines())}
    import xml.etree.ElementTree as ET
    tree=ET.fromstring(subprocess.check_output(['nvidia-smi','-q','-x'],text=True))
    own=[float(p.findtext('used_memory').split()[0])for p in tree.findall('./gpu/processes/process_info')
        if p.findtext('pid')==str(os.getpid())and p.findtext('used_memory','N/A').split()[0]!='N/A']
    used=max(own)if own else None
    passed=(used is not None and used+RESERVE_MIB<=proof['physics_budget_MiB']
        and free[proof['physics_gpu']]>=RESERVE_MIB
        and free[proof['perception_gpu']]>=proof['measured_SAM_peak_MiB']+RESERVE_MIB)
    result=dict(passed=passed,native_actor_GPU_MiB=used,free_after_MiB=free,
        original_no_load_passed=True,world_time_unchanged=True,interaction_started=False,
        original_hard_safety_unchanged=True)
    atomic_json(r.output/'V87_GPU_CAPACITY_INITIALIZED.json',result)
    if not passed:raise SafetyStop('INITIALIZED_GPU_CAPACITY_NOT_QUALIFIED',result)
