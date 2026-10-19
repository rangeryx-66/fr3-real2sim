"""Reuse a previously accepted EE-only model as explicit operation memory."""
import copy,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def load_observed_model(path,kind):
    path=Path(path);doc=json.loads(path.read_text());report=json.loads((path.parent/'report.json').read_text())
    if doc.get('GT_inputs') is not False or not doc.get('source','').startswith('measured EE'):raise ValueError('OPERATION_MEMORY_NOT_EE_ONLY')
    if report.get('gt_control_inputs') is not False or report.get('gt_fit_inputs') is not False or report.get('object_actuation') or report.get('attachments'):raise ValueError('OPERATION_MEMORY_HAS_FORBIDDEN_PROVENANCE')
    estimate=copy.deepcopy(doc.get('estimated_articulation'))
    if not estimate or estimate['joint_type']!=kind:raise ValueError('OPERATION_MEMORY_WRONG_TYPE')
    observations=np.asarray(doc['supporting_observations'],float)
    if len(observations)<12 or np.max(np.linalg.norm(observations[:,:3,3]-observations[0,:3,3],axis=1))<.005:raise ValueError('OPERATION_MEMORY_NO_USEFUL_OBSERVATIONS')
    axis=np.asarray(estimate[kind]['axis'])
    motion=Rotation.from_matrix(observations[-1,:3,:3]@observations[0,:3,:3].T).as_rotvec() if kind=='revolute' else observations[-1,:3,3]-observations[0,:3,3]
    sign=1. if motion@axis>=0 else -1.
    return estimate,sign,{'source':str(path),'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'source_status':report['status'],'source_kind':'previous EE-only interaction estimate, NOT new discovery','GT_inputs':False,'supporting_observation_count':len(observations)}


def restore(r):
    path=r.capture.job.get('operation_memory')
    if not path or r.memory.estimate is not None:return
    estimate,sign,audit=load_observed_model(path,r.policy['joint_type'])
    r.memory.estimate=estimate;r.memory.follow_sign=sign;r.memory.initial=r.tcp().copy()
    if hasattr(r.memory,'monitor_anchor'):r.memory.monitor_anchor=r.tcp().copy()
    r.memory.save();(r.capture.output/'operation_memory_provenance.json').write_text(json.dumps(audit,indent=2))
