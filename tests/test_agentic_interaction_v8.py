"""Packaging invariants; no mock claim of physical contact or hardware portability."""
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from agentic_interaction_v8.resources import digest,dump,inventory,materialize,verify_bundle
from agentic_interaction_v8.cli import audit,prepare


class V8Packaging(unittest.TestCase):
    def test_provenance_survives_runner_fast_exit_before_atexit(self):
        with tempfile.TemporaryDirectory()as t:
            path=Path(t)/'sources.json'
            code='from agentic_interaction_v8.entry import finish_before_shutdown\nfrom pathlib import Path\nimport os\nfinish_before_shutdown(None,lambda r:42,lambda:Path('+repr(str(path))+').write_text("recorded"))\nos._exit(0)\n'
            subprocess.run([sys.executable,'-c',code],check=True)
            self.assertEqual(path.read_text(),'recorded')
            from agentic_interaction_v8.entry import finish_before_shutdown
            calls=[]
            def failure(r):raise ValueError('original failure')
            with self.assertRaisesRegex(ValueError,'original failure'):
                finish_before_shutdown(None,failure,lambda:calls.append('finalized'))
            self.assertEqual(calls,['finalized'])
            def disk_failure():raise OSError('disk unavailable')
            with self.assertRaisesRegex(ValueError,'original failure')as c:
                finish_before_shutdown(None,failure,disk_failure)
            self.assertIn('disk unavailable',str(c.exception.__notes__))

    def test_frozen_actual_sources_and_historical_variants(self):
        a=audit();self.assertEqual(a['frozen_revolute_source_files'],71)
        self.assertFalse(a['failures'])
        profiles=a['historical_execution_profiles']
        self.assertEqual(len(profiles['REGISTRATION_A1']['differing_sources']),2)
        self.assertEqual(len(profiles['REGISTRATION_C1']['differing_sources']),0)

    def fixture(self,root):
        b=root/'bundle';b.mkdir();mesh=b/'mesh.stl';mesh.write_bytes(b'original mesh bytes')
        old='/machine/only/native.stl'
        xml=f'<robot name="test"><link name="finger"><inertial><mass value="0.12"/></inertial><collision><geometry><mesh filename="{old}" scale="1 1 1"/></geometry></collision></link><joint name="gripper_fix" type="fixed"><parent link="link_eef"/><child link="xarm_gripper_base_link"/></joint></robot>'
        cfg=b/'original/robot';cfg.mkdir(parents=True)
        dump(cfg/'NATIVE_CONFIG.json',{'sim_urdf':'/no/old_sim.urdf','dynamics_urdf':'/no/old_dyn.urdf','gripper_kp_nm_rad':10.,'gripper_max_clamp_n':30.})
        for k in ['sim_urdf','dynamics_urdf']:(cfg/(k+'.urdf')).write_text(xml)
        m={'robot':{k:'original/robot/'+k+'.urdf'for k in ['sim_urdf','dynamics_urdf']},
           'original_to_bundle':{old:'mesh.stl'},'original_sha256':{old:digest(mesh)},'files':inventory(b)}
        dump(b/'BUNDLE.json',m);return b

    def test_relocation_preserves_every_numeric_and_mesh_value(self):
        with tempfile.TemporaryDirectory()as t:
            p=Path(t);b=self.fixture(p);r=materialize(b,p/'robot')
            self.assertTrue(all(v['non_path_xml_and_mesh_bytes_equal']for v in r.values()))
            c=json.loads((p/'robot/NATIVE_CONFIG.json').read_text())
            self.assertEqual(c['gripper_max_clamp_n'],30.);self.assertEqual(c['gripper_kp_nm_rad'],10.)
            self.assertNotIn('/machine/only/',(p/'robot/sim_urdf.urdf').read_text())

    def test_resource_tamper_rejected_not_silently_regenerated(self):
        with tempfile.TemporaryDirectory()as t:
            p=Path(t);b=self.fixture(p);(b/'mesh.stl').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'RESOURCE_DRIFT'):verify_bundle(b)

    def test_path_escape_rejected(self):
        with tempfile.TemporaryDirectory()as t:
            p=Path(t);b=self.fixture(p);m=json.loads((b/'BUNDLE.json').read_text());m['files']['../outside']='x';dump(b/'BUNDLE.json',m)
            with self.assertRaisesRegex(ValueError,'RESOURCE_DRIFT'):verify_bundle(b)

    def test_no_source_algorithm_edits_and_no_legacy_import_fallback(self):
        root=Path(__file__).resolve().parents[1];entry=(root/'agentic_interaction_v8/entry.py').read_text()
        self.assertLess(entry.index('xarm7_native_gripper.OUT='),entry.index('from run_efficient_probing_v2'))
        self.assertIn('UNFROZEN_LEGACY_EXECUTION',entry)
        self.assertNotIn('exec(',entry);self.assertNotIn('monkeypatch',entry)

    def test_new_job_forbids_pose_safety_and_camera_overrides(self):
        with tempfile.TemporaryDirectory()as t:
            p=Path(t)/'spec.json';dump(p,{'schema':'agentic-interaction/v8','saved_grasp_poses':[]})
            with patch('agentic_interaction_v8.cli.environment',return_value={'bundle':'unused'}),patch('agentic_interaction_v8.cli.audit',return_value={}),self.assertRaisesRegex(ValueError,'UNKNOWN_JOB_FIELDS'):
                prepare(p,'unused')


if __name__=='__main__':unittest.main()
