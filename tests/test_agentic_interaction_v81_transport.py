"""Atomic station extension without weakening original command validation."""
import json,tempfile,unittest
from pathlib import Path
from agentic_interaction_v81.transport import publish,receive,validate,ProtocolError,original_validate


class TransportTests(unittest.TestCase):
    def request(self):return dict(operation='evaluate_station',target_xy_m=[.2,-.6],
        inspected_image_sha256='a'*64,reason='Current wrist handle view and measured reachability justify a station alternative.')

    def test_station_publish_receive_ack(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);publish(root/'commands',0,self.request())
            value,ack=receive(root/'commands/000.json',0,root/'responses')
            self.assertEqual(value['target_xy_m'],[.2,-.6]);self.assertEqual(ack['status'],'ACCEPTED')
            with self.assertRaises(Exception):publish(root/'commands',0,self.request())

    def test_original_v8_protocol_is_still_frozen(self):
        command=dict(self.request(),protocol_version=1,sequence=0)
        with self.assertRaises(ProtocolError):original_validate(command,0)
        self.assertEqual(validate(command,0),command)

    def test_manual_pose_and_nonfinite_target_rejected_before_publication(self):
        for extra in [dict(grasp_pose=[1,2,3]),dict(target_xy_m=[float('nan'),0]),dict(target_xy_m=[True,0])]:
            command=dict(self.request(),**extra,protocol_version=1,sequence=0)
            with self.subTest(extra=extra),self.assertRaises(ProtocolError):validate(command,0)

    def test_invalid_payload_has_recoverable_no_motion_ack(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'000.json';p.write_text('{"operation":')
            value,ack=receive(p,0,root/'responses')
            self.assertIsNone(value);self.assertFalse(ack['robot_action_executed'])
            self.assertTrue(ack['recoverable']);self.assertEqual(ack['next_sequence'],1)

    def test_relocate_requires_preceding_evaluation(self):
        for n in [-1,1,2,True]:
            with self.subTest(n=n),self.assertRaises(ProtocolError):
                validate(dict(operation='relocate_station',protocol_version=1,sequence=1,evaluated_command=n,reason='move'),1)
        self.assertEqual(validate(dict(operation='relocate_station',protocol_version=1,sequence=1,
            evaluated_command=0,reason='move'),1)['evaluated_command'],0)


if __name__=='__main__':unittest.main()
