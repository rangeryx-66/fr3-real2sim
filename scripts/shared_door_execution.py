"""Contact-aware orchestration around the frozen Pink/control primitives."""
from collections import deque
import numpy as np


def progress_checkpoint(checkpoint,maximum,theta,drift,t,window,count,radius=0.):
    maximum=max(maximum,theta)
    if checkpoint is None:return (t,maximum,drift),maximum,0
    if t-checkpoint[0]<window:return checkpoint,maximum,count
    progress_distance=max(0.,maximum-checkpoint[1])*radius
    drift_growth=max(0.,drift-checkpoint[2])
    count=count+1 if drift_growth>progress_distance and (radius>0 or maximum<=checkpoint[1]) else 0
    return (t,maximum,drift),maximum,count


def open_with_contact_feedback(g):
    original=g['step'];initial_reference=g['reference'].copy();g['drive_correction']=np.eye(4);attempts=0;records=[];window=.5;history=deque();checkpoint=None;stalled=0;maximum=g['actual']();start=maximum;last_log=-1.;status='OPENING';probe=min((p['state'] for p in g['whole']['path'] if p['state']>0),default=np.deg2rad(1.))
    def log(event,**data):
        item={'event':event,'t':g['tick']*g['dt'],'angle_deg':float(np.rad2deg(g['actual']())),'correction_attempts':attempts,**data};records.append(item);g['telemetry_events'].append(item)
    def sample():
        theta=g['actual']();relative=np.linalg.inv(g['moving']())@g['tcp']();drift=float(np.linalg.norm(relative[:3,3]-initial_reference[:3,3]));return theta,drift
    def center_once():
        nonlocal attempts
        attempts+=1;log('REFERENCE_ADVANCEMENT_HOLD');g['opening']=False;g['qvelocity']=np.zeros(6)
        for _ in range(int(.5/g['dt'])):original()
        origin=g['tcp']().copy();local=g['FixedClosure'](origin,g['policy']);local.opening=float(g['rows'][-1]['aperture_m']);local_bias=g['qtarget'][g['arm']]-np.asarray(g['robot'].get_joint_positions())[g['arm']];target=origin.copy();last=origin.copy();g['phase']='LOCAL_CENTERING';duration=g['policy']['max_centering_displacement_m']/min(g['policy']['max_centering_speed_m_s'],g['policy']['max_centering_step_m']/g['dt'])+.5
        acquired=False;feasible=True
        for _ in range(int(duration/g['dt'])):
            state=g['rows'][-1];P=g['poses'](np.asarray(state['q']));centers=[(P[n]@np.r_[g['model'].pads[n].mean(0),1])[:3] for n in g['FINGERS']];forces=g['filtered']();command=g['_fixed_update'](local,g['tcp'](),state['aperture_m'],forces,centers,g['dt']);target=command['T']
            # Reuse only the existing centering target. Jaw mode, effort and
            # aperture commands remain unchanged; there is no release.
            q=g['ik'](target,np.asarray(state['q'])[g['arm']])
            if q is None:feasible=False;log('LOCAL_CENTERING_NO_IK');break
            q=q+local_bias;g['qvelocity']=np.clip((q-g['qtarget'][g['arm']])/g['dt'],-g['vel']/3,g['vel']/3);g['qtarget'][g['arm']]=q;original();last=target.copy()
            if min(g['filtered']())>=.05:acquired=True;break # existing contact threshold; no symmetry/convergence requirement
        displacement=last[:3,3]-origin[:3,3];nominal=g['moving']()@g['grasp_relative'];g['drive_correction'][:3,3]+=nominal[:3,:3].T@displacement
        # Preserve the INITIAL grasp reference used by slip measurement.
        # Re-anchor only the drive clock to measured object progress and carry
        # the explicitly commanded centering displacement as a separate offset.
        g['angle_reference']=g['actual']();g['ref']=last.copy();g['seed']=np.asarray(g['robot'].get_joint_positions())[g['arm']];g['bias']=g['qtarget'][g['arm']]-g['seed'];g['twist']=np.zeros(6);g['qvelocity']=np.zeros(6)
        log('LOCAL_CENTERING_RESULT',existing_filtered_contact_reacquired=acquired,commanded_displacement_m=displacement.tolist(),drive_correction=g['drive_correction'].tolist(),initial_grasp_reference_unchanged=bool(np.array_equal(g['reference'],initial_reference)))
        g['opening']=feasible;return feasible # one-pad unloading never blocks continuation
    try:
        while g['actual']()<g['goal']:
            g['phase']='TRANSMISSION_PROBE' if g['actual']()-start<probe else 'OPEN_CONTINUE';original();theta,drift=sample();maximum=max(maximum,theta);t=g['tick']*g['dt']
            radius=float(np.linalg.norm(g['tcp']()[:3,3]-g['body_at'](theta)[:3,3]))
            checkpoint,maximum,stalled=progress_checkpoint(checkpoint,maximum,theta,drift,t,window,stalled,radius)
            if stalled>=g['job']['wrist_experiment']['force_policy']['sustained_windows']:
                log('CONTACT_TRANSMISSION_STALL',relative_drift_m=drift,pad_loads_n=list(map(float,g['filtered']())))
                if attempts>=1:status='CONTACT_TRANSMISSION_FAILURE_AFTER_ONE_LOCAL_CORRECTION';return False,status,records
                if not center_once():status='CONTACT_TRANSMISSION_FAILURE_LOCAL_CENTERING_EXHAUSTED';return False,status,records
                maximum=g['actual']();checkpoint=None;stalled=0
        status='SHARED_DOOR_30_DEG_SUCCESS';return True,status,records
    except BaseException as error:
        status='EXISTING_EXECUTION_EXIT:'+repr(error);raise
    finally:
        g['diagnostic_write'](g['a'].output/'CONTACT_ADAPTATION.json',g['json'].dumps({'status':status,'local_correction_attempts':attempts,'events':records,'initial_reference_preserved':bool(np.array_equal(g['reference'],initial_reference)),'initial_grasp_reference':initial_reference.tolist(),'drive_correction':g['drive_correction'].tolist(),'single_pad_unloading_not_failure':True,'releases':0,'regrasps':0},indent=2))
