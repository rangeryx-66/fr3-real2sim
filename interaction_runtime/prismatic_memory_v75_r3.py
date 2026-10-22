"""Episode-local failure CONDITIONS; no pose, score, force or safety override."""
import hashlib,json
from pathlib import Path

LEGAL_REQUESTS=(.5,.75,1.,1.5)


def recovery_memory(attempts,output,requested):
    if isinstance(requested,bool)or requested not in LEGAL_REQUESTS:
        raise ValueError('TESTED_PRELOAD_MENU_ONLY')
    blocked=set();rows=[]
    for i,a in enumerate(attempts):
        if not a.get('physical_grasp_started') or not a.get('failed_contact_candidate',not a.get('short_success',False)):
            continue
        family=tuple(a['family']);path=Path(output)/'attempt_inputs'/('%02d.json'%i)
        old=None;digest=None
        try:
            raw=path.read_bytes();old=json.loads(raw)['requested_preload_n']
            if isinstance(old,bool)or old not in LEGAL_REQUESTS:old=None
            digest=hashlib.sha256(raw).hexdigest()
        except (OSError,ValueError,KeyError,TypeError):pass
        statuses=[s.get('status')for s in a.get('segments',[])]
        safe=a.get('safe_exit')or{};park=safe.get('park')or{}
        different=(old is not None and requested!=old)
        reconsider=bool(different and (a.get('acquisition')or{}).get('acquired')
            and 'TRANSMISSION_FAILURE'in statuses and 'SAFETY_STOP'not in statuses
            and not a.get('operation_status') and not a.get('error')
            and safe.get('safe') and park.get('reached') and park.get('unloaded'))
        if not reconsider:blocked.add(family)
        rows.append(dict(attempt_index=i,family=list(family),prior_requested_preload_n=old,
            requested_preload_n=requested,prior_input_sha256=digest,
            reconsideration_eligible=reconsider,prior_statuses=statuses,
            reason='DIFFERENT_LEGAL_REQUEST_AFTER_VERIFIED_SAFE_TRANSMISSION_FAILURE'if reconsider else'UNCHANGED_OR_UNVERIFIED_FAILURE_CONDITIONS'))
    result=dict(requested_preload_n=requested,failed_conditions=rows,
        blocked_families=sorted([list(f)for f in blocked]),
        scope='this episode; original failure retained; full native execution checks still required')
    result['conditions_sha256']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()
    return blocked,result


def stamp_catalog(catalog,memory):
    result=dict(catalog,recovery_conditions_sha256=memory['conditions_sha256'],
        requested_preload_n=memory['requested_preload_n'])
    result.pop('catalog_sha256',None)
    result['catalog_sha256']=hashlib.sha256(json.dumps(result,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return result


def resolve_candidate(request,catalog,observations,attempts,output,requested):
    from .recovery_candidates_v7 import resolve
    blocked,memory=recovery_memory(attempts,output,requested)
    if not catalog or catalog.get('recovery_conditions_sha256')!=memory['conditions_sha256']:
        raise ValueError('CHANGED_RECOVERY_CONDITIONS_REQUIRE_NEW_AUTOMATIC_CATALOG')
    return resolve(request,catalog,observations,blocked)
