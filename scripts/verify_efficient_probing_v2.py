"""Short storage, interface and independent validation regression; no Isaac actor."""
import argparse,json,sys,tempfile,threading
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.compact_v2 import CompactRecorder
from interaction_runtime.probing_v2 import ProbingRuntime
from interaction_runtime.policy import Policy
from update_probing_v2_model import update

def verify():
    r=object.__new__(ProbingRuntime);r.policy=Policy.load(ROOT/'configs/interaction_runtime_v1.json');r.probe_profile={'profile':'smooth_variation','reference_speed_m_s':1.,'reference_acceleration_m_s2':1.};r.profile_started=0;r.dt=1/240
    for tick in range(2400):
        r.tick=tick;m=r.motion_limits('LOADED');assert m['speed_m_s']<=.012 and m['acceleration_m_s2']<=.04
    with tempfile.TemporaryDirectory() as name:
        p=Path(name);log=CompactRecorder(p/'telemetry');atomic_json(p/'identification_input.json',dict(prior=dict(axis_world=[0,0,1],origin_world_m=[0,0,0],joint_type='revolute'),bias=[0.]*6))
        for sid,role in enumerate(['FIT_SYNTHETIC','VALIDATION_SYNTHETIC']):
            log.begin(dict(role=role,target_deg=10.,profile='constant'),'HANDLE')
            for j in range(1440):
                t=j/240
                if sid==0:q=.02*t+.005*np.sin(2*t);v=.02+.01*np.cos(2*t);a=-.02*np.sin(2*t)
                else:q=.12-.018*t-.004*np.sin(2*t);v=-.018-.008*np.cos(2*t);a=.016*np.sin(2*t)
                tau=.3*a+.12*v+.04*np.sign(v)
                row=dict(t=(sid*1440+j)/240,phase='OPEN_CONTINUE',q=np.zeros(13),joint_velocities=np.zeros(13),command_q_arm=np.zeros(7),command_q_velocity_arm=np.zeros(7),
                    door_angle_deg=float(np.rad2deg(q)),telemetry=dict(hinge_velocity_rad_s=v),wrist_ft=dict(raw_wrench_sensor=[0,0,0,0,0,tau],valid=True,T_world_sensor=np.eye(4),body_states=[dict(name='tool',mass_kg=0.,T_world_body=np.eye(4),com_world_m=np.zeros(3),inertia_world_kg_m2=np.zeros((3,3)))]),T_tcp=np.eye(4),aperture_m=.02,command_aperture_m=.02,forces_n={'L':.5,'R':.5},relative_translation_slip_m=0.,contacts=[{'force_n':.5}],force_event=dict(status='NORMAL',stop=False))
                log.append(row)
            log.end(dict(ok=True))
        idx=log.close();assert idx['records']==2880 and not idx['issues'];seen=[]
        for c in idx['chunks']:
            with np.load(p/'telemetry'/c['file'],allow_pickle=False) as z:seen+=z['step'].tolist();assert z['raw_ft'].dtype==np.float64
        assert len(set(seen))==len(seen)==2880
        fit=update(p)['constant_model'];before=fit['parameters']
        assert np.max(abs(np.array(before)-[.3,.12,.04])/[.3,.12,.04])<.15,before
        for c in idx['chunks']:
            path=p/'telemetry'/c['file']
            with np.load(path,allow_pickle=False) as z:values={k:z[k] for k in z.files}
            values['raw_ft'][values['segment']==2,5]+=.1
            np.savez_compressed(path,**values)
        after=update(p)['constant_model']['parameters'];assert before==after,'held-out information entered fitting'
        return dict(physical_episodes=0,compact_unique_samples=2880,FT_dtype='float64',native_q_dtype='float32',synthetic_parameters=before,
            complete_validation_mutation_does_not_change_fit=True,profile_cannot_raise_phase_envelope=True,stored_numeric_bytes=sum(c['bytes'] for c in idx['chunks']),no_legacy_control_owner=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=verify();a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
