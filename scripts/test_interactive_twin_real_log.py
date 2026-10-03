"""Synthetic contract fixtures only: these tests are NOT real-robot evidence."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));sys.path.insert(0,str(Path(__file__).resolve().parent))
from interactive_twin.real_log import (validate_bundle,run_real_log_bundle,BundleBlocked,
    current_servo_hashes,_write,_read,_sha,_tape_commands,SCHEMA)
from interactive_twin.sysid import command_hash
from test_interactive_twin_artifacts import URDF

class RealBundleContract(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        import interactive_twin.real_log as adapter
        fixture_root=self.root/'fixture_runtime'
        for name in adapter.SERVO_FILES:
            target=fixture_root/name;target.parent.mkdir(parents=True,exist_ok=True)
            if name=='config/piper.urdf':
                target.write_text('<robot name="synthetic_command_limits_only">'+''.join(f'<joint name="joint{i}" type="revolute"><limit lower="-3" upper="3" velocity="2" effort="20"/></joint>' for i in range(1,7))+'</robot>')
            else:target.write_bytes((adapter.ROOT/name).read_bytes())
        self.patch=patch.object(adapter,'ROOT',fixture_root);self.patch.start()
        root=self.root;asset=root/'asset';(asset/'urdf').mkdir(parents=True);(asset/'urdf/test.urdf').write_text(URDF)
        _write(asset/'manifest.json',{'asset_id':'test','joint_name':'hinge','moving_link':'door','door_link':'handle','moving_links':['door','handle','handle_tip']})
        _write(root/'source/report.json',{'robot_base_pose':[0,0,0,0]});_write(root/'plan.json',{'trial_candidates':[]})
        _write(root/'template.json',{'source':'source','asset_root':'asset','plan':'plan.json','wall_clock_budget_s':30})
        self.tape=[]
        # Values lie within the official PiPER ranges; no actual plant is run.
        q=[0.,.5,-.5,0.,.2,0.]
        def append(phase,compliant=False,retained=False):
            i=len(self.tape);self.tape.append({'t':i/240,'phase':phase,'compliant':compliant,'arm_position':q,'arm_velocity':[0.]*6,
                'finger_mode':'position','finger_position':[.015,-.015],'finger_effort':.5,'retention_armed':retained,
                'cartesian_input':{'reference_world_m':[.4,.1,.2],'direction_world':[1.,0.,0.],'active':True} if compliant else None})
        for _ in range(120):append('SETTLE')
        for _ in range(3):append('APPROACH')
        for _ in range(10):append('EXPLORATORY',True,True)
        self.protocol=[]
        for probe in ('P1','P2','P3','P4'):
            self.protocol.append({'probe_id':probe,'split':'heldout' if probe=='P4' else 'train','duration_s':5/240,'speed_m_s':.00025,'pattern':'opening'})
            for _ in range(5):append(probe,True,True)
        _write(root/'commands.json',self.tape)
        noise={'position_m':.00002,'rotation_rad':.00002,'joint_rad':.00002,'joint_velocity_rad_s':.001,'ee_velocity_m_s':.0001,'start_time_s':1/240,'tracking_m':.00002,'calibration_id':'test-cal'}
        cal={'mode':'REAL_LOG_TO_SIM','source':'real_robot_log','hardware_calibration':True,'repeats':2,'no_contact_supervisor_certified':True,'noise_scales':noise,'calibration_id':'test-cal','robot_model_id':'fixture-only-robot','controller_id':'fixture-only-controller'}
        _write(root/'calibration.json',cal)
        self.logs={}
        for p in ('P1','P2','P3','P4'):
            command=_tape_commands(self.tape,p);n=len(command['time_s']);pose=np.tile(np.eye(4),(n,1,1));pose[:,:3,3]=[.4,.1,.2]
            log={'schema_version':1,'mode':'REAL_LOG_TO_SIM','episode_id':'synthetic_contract_fixture','probe_id':p,'split':'heldout' if p=='P4' else 'train','time_s':command['time_s'],'commands':command,
                'signals':{'q_rad':[q]*n,'qdot_rad_s':[[0.]*6]*n,'ee_T_world_tcp':pose.tolist(),'gripper_opening_m':[.03]*n},
                'provenance':{'source':'real_robot_log','robot_model_id':cal['robot_model_id'],'controller_id':cal['controller_id'],'robot_calibration_id':cal['calibration_id'],'complete':True,'initialization_only':True,'robot_state_replayed':False,'object_state_replayed':False,'direct_object_actuation':False,'attachment':False,'command_applied_sha256':command_hash(command)}}
            self.logs[p]=log;_write(root/(p+'.json'),log)
        pairs=[]
        for i in range(2):
            real=deepcopy(self.logs['P1']);real.update(probe_id='ROBOT_CALIBRATION',split='calibration');real['provenance']['no_contact_supervisor_certified']=True
            sim=self.sim_log(real);_write(root/f'real_cal_{i}.json',real);_write(root/f'sim_cal_{i}.json',sim)
            pairs.append({'real_log':f'real_cal_{i}.json','sim_log':f'sim_cal_{i}.json','real_sha256':_sha(root/f'real_cal_{i}.json'),'sim_sha256':_sha(root/f'sim_cal_{i}.json')})
        mapping={'independently_calibrated':True,'frozen_before_object_data':True,'object_contact_data_used':False,'q_measurements_used_as_commands':False,'command_kind':'cartesian_constrained_drive','sample_rate_hz':240,'current_or_sdk_effort_to_external_torque':False,
            'native_servo_sha256':current_servo_hashes(),'robot_model_id':cal['robot_model_id'],'controller_id':cal['controller_id'],'hardware_firmware_version':'SYNTHETIC_UNIT_TEST_NOT_HARDWARE','predeclared_maximum_motion_loss':1.,'free_space_validation_pairs':pairs}
        _write(root/'mapping.json',mapping)
        fit={'joint_type':'revolute','confidence':.99,'revolute':{'axis':[0,0,1],'point_on_axis':[.2,.1,.3],'radius_m':.3}}
        poses=np.tile(np.eye(4),(6,1,1));poses[:,0,3]=np.linspace(0,.006,6)
        _write(root/'fit.json',fit);_write(root/'memory.json',{'GT_inputs':False,'fit_history':[{'accepted':True,'fit':fit,'observation_count':6}],'supporting_observations':poses.tolist(),'attempt_history':[{'result':'RELIABLE_REVOLUTE_ESTIMATE','end_s':132/240}]})
        _write(root/'initial.json',{'source':'measured_prepared_initial_scene','joint_coordinate_zero_is_initial':True,'robot_q_initial_verified':True,'robot_q_initial_rad':q,'T_world_asset':np.eye(4).tolist(),'fixture':{'frozen':True}})
        self.bundle={'schema':SCHEMA,'mode':'REAL_LOG_TO_SIM','command_tape':'commands.json','robot_calibration':'calibration.json','servo_mapping':'mapping.json','job_template':'template.json','estimated_articulation':'fit.json','structured_memory':'memory.json','initial_scene':'initial.json','reference_logs':{p:p+'.json' for p in self.logs},'physics_protocol':self.protocol,'estimate_support_scope':'pre_physics_EE_only','budget':{'simulation_launches':7,'maximum_candidates':4,'wall_clock_s':60},'grid':{'tau_c':[0.,.01],'b':[0.,.5]},'initial_physics_prior':{'tau_c':0.,'b':0.}}
        _write(root/'bundle.json',self.bundle)
    def tearDown(self):self.patch.stop();self.tmp.cleanup()
    @staticmethod
    def sim_log(log):
        result=deepcopy(log);result['provenance'].update(source='isaac_physics',simulator='isaac_physx');return result
    def test_valid_contract_and_immutable_sources(self):
        before=_sha(self.root/'commands.json');c=validate_bundle(self.root/'bundle.json')
        self.assertEqual(len(c['grid']),4);self.assertIsNone(c['sensitivity']);self.assertEqual(before,_sha(self.root/'commands.json'))
    def test_relabelled_sim_reference_blocked(self):
        _write(self.root/'P1.json',self.sim_log(self.logs['P1']))
        with self.assertRaisesRegex(BundleBlocked,'ORIGINAL_REAL'):validate_bundle(self.root/'bundle.json')
    def test_unverified_firmware_and_changed_servo_blocked(self):
        m=_read(self.root/'mapping.json');m['hardware_firmware_version']='unknown';_write(self.root/'mapping.json',m)
        with self.assertRaisesRegex(BundleBlocked,'FIRMWARE'):validate_bundle(self.root/'bundle.json')
        m['hardware_firmware_version']='fixture';m['native_servo_sha256']['config/piper.urdf']='0'*64;_write(self.root/'mapping.json',m)
        with self.assertRaisesRegex(BundleBlocked,'IMPLEMENTATION_CHANGED'):validate_bundle(self.root/'bundle.json')
    def test_measured_q_or_sdk_effort_is_not_replayed(self):
        self.tape[-1]['sdk_effort']=[1.]*6;_write(self.root/'commands.json',self.tape)
        with self.assertRaisesRegex(BundleBlocked,'NO_MEASURED_Q_OR_RAW_EFFORT'):validate_bundle(self.root/'bundle.json')
    def test_missing_evidence_blocks_without_native_callback(self):
        (self.root/'mapping.json').unlink();calls=[]
        result=run_real_log_bundle(self.root/'bundle.json',self.root/'out',lambda j:calls.append(j))
        self.assertEqual(calls,[]);self.assertEqual(result['status'],'BLOCKED_BUNDLE_EVIDENCE')
    def test_native_callback_grid_then_independent_heldout(self):
        calls=[];out=self.root/'out'
        def fake_native(job):
            # Contract-only fake; records no physical success claim.
            calls.append(job);self.assertEqual(job['observation_mode'],'REAL_LOG_TO_SIM');self.assertEqual(job['mode'],'replay')
            self.assertIn('reference_safety_memory',job);self.assertEqual(job['initial_articulation_rad'],0.)
            tape=_read(job['replay_commands']);isheld='/heldout/' in job['output']
            self.assertEqual(any(f['phase']=='P4' for f in tape),isheld)
            if isheld:self.assertTrue((out/'physics_fit.json').exists())
            for p in ('P1','P2','P3','P4') if isheld else ('P1','P2','P3'):
                _write(Path(job['output'])/'observable'/(p+'.json'),self.sim_log(self.logs[p]))
            return {'status':'REPLAY_COMPLETE','success':False,'test_fixture_only':True}
        result=run_real_log_bundle(self.root/'bundle.json',out,fake_native)
        self.assertEqual(len(calls),6);self.assertEqual(result['physics_identification'],'SENSITIVITY_NOT_VALIDATED')
        self.assertEqual(result['heldout_models_completed'],['B0','B1']);self.assertFalse(result['real_to_sim_success_proven'])
        self.assertFalse(result['real_robot_commands_sent']);self.assertFalse(_read(out/'twins_final/T2/twin.json')['physics_updated'])
        before=len(calls);run_real_log_bundle(self.root/'bundle.json',out,fake_native);self.assertEqual(len(calls),before)

if __name__=='__main__':unittest.main()
