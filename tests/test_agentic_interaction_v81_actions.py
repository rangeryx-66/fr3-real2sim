"""Revising a finite probe must not reuse stale geometry or bypass safety."""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from agentic_interaction_v81.action_declaration import redeclare, ticket
from agentic_interaction_v81.transport import validate, ProtocolError
from interaction_runtime.articulation_v7 import Articulation


class DeclarationTests(unittest.TestCase):
    def fixture(self, folder):
        p=Path(folder);view=p/'view';view.mkdir()
        for name in ('rgb.png','depth_m.npy','camera.json'):(view/name).write_bytes(name.encode())
        plan=p/'visual_plan.json';plan.write_text(json.dumps(dict(path=[dict(state=0.),dict(state=1.6)])))
        image=p/'AGENT_GRASP_PLAN_INSPECTION.png';image.write_bytes(b'unit-test image')
        sha=hashlib.sha256(image.read_bytes()).hexdigest()
        (p/'V8_PLAN_INSPECTION_PROVENANCE.json').write_text(json.dumps(dict(sha256=sha,
            plan_sha256=hashlib.sha256(plan.read_bytes()).hexdigest())))
        ops=SimpleNamespace(current_plan_ready=True,acquisitions=0,
            r=SimpleNamespace(output=p,base_generation=0,job=dict(articulation_v7=dict(joint_type='revolute'))),
            g=dict(agent_observations=[dict(accepted_for_geometry=True,folder=str(view))]),
            articulation=lambda:Articulation('revolute',(0,0,1),(0,0,0),(0,1.75)),
            validate_execution=lambda:dict(valid=True),_v81_declared_chain=['previous'])
        ops._v81_plan_ticket=ticket(ops)
        command=dict(operation='redeclare_actions',protocol_version=1,sequence=1,
            inspected_image_sha256=sha,reason='Current measured task arc now supports a larger finite probe.',
            segments=[dict(role='VALIDATION_NEW',target_q=1.57,coordinate_unit='rad',profile='constant',hold=True)])
        return ops,command

    def test_current_unexecuted_plan_revalidated_without_new_grasp(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);self.assertEqual(validate(c,1),c)
            result=redeclare(ops,c)
            self.assertTrue(result['declared']);self.assertFalse(result['robot_motion_executed'])
            self.assertEqual(ops._v81_declared_chain,c['segments'])

    def test_changed_plan_pixels_station_or_acquisition_is_rejected(self):
        for kind in ('plan','pixels','station','acquisition'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as d:
                ops,c=self.fixture(d)
                if kind=='plan':(Path(d)/'visual_plan.json').write_text('{}')
                if kind=='pixels':(Path(d)/'view/rgb.png').write_bytes(b'new frame')
                if kind=='station':ops.r.base_generation+=1
                if kind=='acquisition':ops.acquisitions+=1
                self.assertFalse(redeclare(ops,c)['declared'])
                self.assertEqual(ops._v81_declared_chain,['previous'])

    def test_targets_cannot_extrapolate_the_native_checked_arc(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);c['segments'][0]['target_q']=1.65
            self.assertEqual(redeclare(ops,c)['reason'],'ACTION_EXCEEDS_EXISTING_CHECKED_TASK_ARC')
            self.assertEqual(ops._v81_declared_chain,['previous'])

    def test_failed_current_state_route_does_not_change_declaration(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);ops.validate_execution=lambda:dict(valid=False,reason='native collision')
            self.assertFalse(redeclare(ops,c)['declared']);self.assertEqual(ops._v81_declared_chain,['previous'])

    def test_stale_inspection_and_wrong_units_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);c['inspected_image_sha256']='a'*64
            self.assertFalse(redeclare(ops,c)['declared'])
            c['inspected_image_sha256']=hashlib.sha256((Path(d)/'AGENT_GRASP_PLAN_INSPECTION.png').read_bytes()).hexdigest()
            c['segments'][0]['coordinate_unit']='m'
            with self.assertRaises(ValueError):redeclare(ops,c)

    def test_manual_pose_preload_and_incomplete_validation_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            _,c=self.fixture(d)
            for extra in (dict(grasp_pose=[]),dict(native_g1_preload_request_n=1.5)):
                with self.subTest(extra=extra),self.assertRaises(ProtocolError):validate(dict(c,**extra),1)
            c['segments'][0]['fit_intervals_s']=[[0,10]]
            with self.assertRaises(ProtocolError):validate(c,1)

    def test_prismatic_inherited_handshake_receives_same_revalidated_chain(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);ops.r.job['articulation_v7']['joint_type']='prismatic'
            ops.g['v75_declared_chain']=['previous']
            result=redeclare(ops,c)
            self.assertTrue(result['declared'])
            self.assertTrue(result['original_prismatic_declaration_synchronized'])
            self.assertEqual(ops._v81_declared_chain,ops.g['v75_declared_chain'])
            self.assertEqual(c['segments'],ops.g['v75_declared_chain'])

    def test_missing_prismatic_owner_state_fails_closed(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);ops.r.job['articulation_v7']['joint_type']='prismatic'
            self.assertEqual(redeclare(ops,c)['reason'],'ORIGINAL_PRISMATIC_DECLARATION_MISSING')
            self.assertEqual(ops._v81_declared_chain,['previous'])

    def test_revolute_does_not_author_or_change_prismatic_owner_state(self):
        with tempfile.TemporaryDirectory() as d:
            ops,c=self.fixture(d);ops.g['v75_declared_chain']=['untouched']
            self.assertTrue(redeclare(ops,c)['declared'])
            self.assertEqual(ops.g['v75_declared_chain'],['untouched'])


if __name__=='__main__':unittest.main()
