import json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import validate,ProtocolError
from interaction_runtime.progress import ProgressClock
from interaction_runtime.segments import segment_plan,TransmissionMonitor,continuation_allowed
from interaction_runtime.policy import Policy
from interaction_runtime.probing_v7 import RecorderV7


class RuntimeTests(unittest.TestCase):
    def test_posthoc_reference_uses_registered_joint_not_first_hinge(self):
        from evaluate_cross_object_v7 import authored_inertial_reference
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'urdf').mkdir();(root/'manifest.json').write_text(json.dumps(dict(joint_name='selected')))
            (root/'urdf/object.urdf').write_text('<robot name="fixture"><link name="root"/><link name="other"/><link name="moving"/><joint name="first" type="revolute"><parent link="root"/><child link="other"/><axis xyz="0 0 1"/></joint><joint name="selected" type="revolute"><parent link="root"/><child link="moving"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint></robot>')
            truth=dict(bodies=[dict(body='moving',mass_kg=2.,com_local_m=[1,0,0],inertia_local_kg_m2=np.diag([.1,.2,.3]).tolist())])
            value,extra=authored_inertial_reference(truth,root,'revolute')
            self.assertAlmostEqual(value,2.3);self.assertEqual(extra,[])
            self.assertEqual(authored_inertial_reference(truth,root,'prismatic'),(2.,[]))

    def test_deferred_prismatic_case_cannot_launch_as_authorized_research(self):
        from launch_job_v7 import verify_registration
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);registration=root/'BENCHMARK.json'
            registration.write_text(json.dumps(dict(episodes=[dict(episode_id='drawer_0',asset_id='drawer',configuration_index=8,execution_authorized=False,deferral_reason='PRISMATIC_ENGINEERING_UNRESOLVED')],source_sha256={})))
            job=dict(experiment='V7_BENCHMARK',benchmark_registration=str(registration),registered_episode_id='drawer_0',asset_id='drawer',identification_case=8)
            with self.assertRaisesRegex(ValueError,'deferred: PRISMATIC_ENGINEERING_UNRESOLVED'):
                verify_registration(root,job)

    def test_failed_replanning_cannot_execute_a_previous_disk_plan(self):
        from unittest.mock import patch
        from interaction_runtime.probing_v7 import OperationsV7,OperationsV5
        ops=OperationsV7.__new__(OperationsV7);ops.current_plan_ready=True;ops.g={}
        with patch.object(OperationsV5,'build_geometry',return_value=dict(plan=False,reason='bounded candidates exhausted')):
            self.assertFalse(ops.build_geometry()['plan'])
        self.assertFalse(ops.validate_execution()['valid'])
        self.assertFalse(ops.attempt(dict(segments=[]))['executed'])
        with patch.object(OperationsV5,'build_geometry',return_value=dict(plan=True)):
            self.assertTrue(ops.build_geometry()['plan'])
        with patch.object(OperationsV5,'reobserve',return_value=dict(observation='fresh')):
            ops.reobserve()
        self.assertFalse(ops.current_plan_ready)

    def test_native_protocol_no_implicit_conversion(self):
        a=dict(target_q=.12,coordinate_unit='m',role='FIT_OPEN',profile='constant',hold=True)
        command=dict(protocol_version=1,sequence=0,operation='attempt_grasp_and_probe',segments=[a])
        validate(command,0)
        for change in [dict(target_deg=12),dict(coordinate_unit='mm'),dict(target_q=float('nan'))]:
            with self.assertRaises(ProtocolError):validate(dict(command,segments=[dict(a,**change)]),0)

    def test_linear_clock_nominal_reversal_and_stall(self):
        motion=Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json').motion('LOADED');motion=dict(motion)
        motion.update(angular_speed_rad_s=motion['speed_m_s'],angular_acceleration_rad_s2=motion['acceleration_m_s2'])
        dt=1/240;lead=motion['speed_m_s']**2/(2*motion['acceleration_m_s2'])+motion['speed_m_s']*dt
        action=dict(target_q=.12,coordinate_unit='m',role='FIT',profile='constant')
        plan=segment_plan(0,.12,action,motion,1.,dt,dict(wall_s=200,sim_s=100),lookahead=lead)
        self.assertTrue(plan['valid']);self.assertLess(plan['nominal_duration_s'],11)
        for start,target in [(0.,.12),(.12,.03)]:
            clock=ProgressClock(start,target,lookahead=lead);q=start;monitor=TransmissionMonitor(start,clock.direction,action,lookahead=lead)
            for i in range(5000):
                q,state=clock.advance(q,motion['speed_m_s'],1.,dt,motion['speed_m_s'],motion['acceleration_m_s2'],motion['acceleration_m_s2'])
                self.assertIsNone(monitor.update(i*dt,q,state,dt,clock.direction*(target-q)))
                if clock.direction*(q-target)>=0:break
            self.assertLess(i*dt,12)
        clock=ProgressClock(0,.12,lookahead=lead);monitor=TransmissionMonitor(0,1,action,lookahead=lead)
        for i in range(2400):
            ref,state=clock.advance(0,motion['speed_m_s'],1.,dt,motion['speed_m_s'],motion['acceleration_m_s2'],motion['acceleration_m_s2'])
            if monitor.update(i*dt,0,state,dt,.12):break
        self.assertIsNotNone(monitor.failure);self.assertLess(i*dt,8)
        self.assertLess(ref,.003)

    def test_compact_recorder_preserves_metres_without_angular_column(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=RecorderV7(tmp);r.begin(dict(target_q=.1,coordinate_unit='m',role='FIT',profile='constant'),'HANDLE')
            for i in range(300):
                row=dict(t=i/240,phase='OPEN_CONTINUE',q=np.zeros(13),joint_velocities=np.zeros(13),command_q_arm=np.zeros(7),command_q_velocity_arm=np.zeros(7),joint_coordinate=.0001*i,joint_coordinate_velocity=.024,
                    door_angle_deg=None,forces_n=dict(L=.1,R=.1),contacts=[],T_tcp=np.eye(4),aperture_m=.03,command_aperture_m=.03,relative_translation_slip_m=0,
                    wrist_ft=dict(raw_wrench_sensor=np.zeros(6),T_world_sensor=np.eye(4),body_states=[],valid=True))
                r.append(row)
            r.end(dict(ok=True,status='TARGET_REACHED',requested_target_reached=True));index=r.close()
            self.assertFalse(index['issues']);self.assertEqual(index['records'],300)
            with np.load(Path(tmp)/index['chunks'][0]['file']) as a:
                self.assertNotIn('door_q',a.files);np.testing.assert_allclose(a['joint_q'],np.arange(300)*.0001,rtol=0,atol=1e-17)
            metadata=json.loads((Path(tmp)/'index.json').read_text())
            self.assertEqual(metadata['segments'][0]['coordinate_unit'],'m')


if __name__=='__main__':unittest.main()
