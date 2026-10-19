"""Agent selects only existing measured-geometry families; no pose edits/scores."""
import hashlib,json


def family(c):return tuple(float(c[k]) for k in ['section','insertion_m','roll','pitch'])


def observation_key(observations):
    # Same-episode fresh captures are immutable; include their pixel/calibration
    # content hashes, not asset IDs or a saved historical grasp transform.
    from pathlib import Path
    rows=[]
    for view in observations:
        if not view['accepted_for_geometry']:continue
        p=Path(view['folder']);rows.append({name:hashlib.sha256((p/name).read_bytes()).hexdigest() for name in ['rgb.png','depth_m.npy','camera.json']})
    return hashlib.sha256(json.dumps(rows,sort_keys=True).encode()).hexdigest()


def catalog(scored,attempted,observations):
    rows=[]
    for i,c in enumerate(scored):
        r=c['contact']['robust_contact'];trial=min(r['aperture_trials'],key=lambda x:abs(x['width_m']-r['chosen_expected_aperture_m']))
        rows.append(dict(id=str(i),family=list(family(c)),eligible=family(c) not in attempted,
            acquisition_score=r['acquisition_score'],retention_score=r['retention_score'],task_score=r['task_score'],robust_score=r['robust_combined_score'],
            first_contacts=c['contact']['first_contacts'],expected_aperture_m=r['chosen_expected_aperture_m'],nominal_pad_support=trial['nominal']['sides']))
    data=dict(observation_sha256=observation_key(observations),candidates=rows,scores_changed=False,poses_are_automatic=True,native_execution_check_still_required=True)
    data['catalog_sha256']=hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return data


def resolve(request,previous,observations,attempted):
    if not previous or request.get('catalog_sha256')!=previous['catalog_sha256']:raise ValueError('STALE_OR_UNKNOWN_CANDIDATE_CATALOG')
    if previous['observation_sha256']!=observation_key(observations):raise ValueError('FRESH_OBSERVATION_REQUIRES_NEW_CATALOG')
    if not isinstance(request.get('reason'),str) or not request['reason'].strip():raise ValueError('OBSERVED_FEEDBACK_REASON_REQUIRED')
    matches=[r for r in previous['candidates'] if r['id']==request.get('candidate_id')]
    if len(matches)!=1 or tuple(matches[0]['family']) in attempted:raise ValueError('CANDIDATE_NOT_ELIGIBLE')
    return tuple(matches[0]['family'])


def order(scored,attempted,preferred=None):
    remaining=[c for c in scored if family(c) not in attempted]
    if preferred is not None:
        matches=[c for c in remaining if family(c)==preferred]
        if len(matches)!=1:raise ValueError('REQUESTED_FAMILY_NOT_IN_FRESH_AUTOMATIC_POOL')
        # The shared planner sorts again by unchanged utility after this seam.
        # Restrict this planning call, rather than pretending list order alone
        # survives that sort. The operation explicitly falls back to the full
        # eligible pool if this requested family has no executable path.
        remaining=matches
    return remaining
