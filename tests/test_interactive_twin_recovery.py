"""Recovery triggers and visual-prior boundary; no changes to frozen contact tests."""
import unittest
import json
import tempfile
import sys
from pathlib import Path
import numpy as np
from interactive_twin_recovery.mobile import candidate_bases,eligible,at_base
from interactive_twin_recovery.oracle import visual_prior
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_interactive_twin_recovery_benchmark import summary_safe,RecoveryBenchmark


class RecoveryBoundary(unittest.TestCase):
 def test_missing_native_failure_metric_is_null_not_success(self):
  raw={'status':'PHYSICS_CLOCK_MISMATCH','success':False,'minimum_joint_margin_rad':float('nan')}
  safe=summary_safe(raw)
  self.assertIsNone(safe['minimum_joint_margin_rad'])
  self.assertFalse(safe['success'])
  json.dumps(safe,allow_nan=False)
  self.assertTrue(np.isnan(raw['minimum_joint_margin_rad']))

 def test_resume_only_accepts_explicit_implementation_hash_chain(self):
  with tempfile.TemporaryDirectory() as folder:
   bench=RecoveryBenchmark.__new__(RecoveryBenchmark);bench.output=Path(folder)
   name='interactive_twin_recovery/native.py'
   self.assertFalse(bench.verified_code_history({name:'a'},{name:'c'}))
   (bench.output/'implementation_amendments.json').write_text(json.dumps({'amendments':[
      {'file':name,'from':'a','to':'b'},{'file':name,'from':'b','to':'c'}]}))
   self.assertTrue(bench.verified_code_history({name:'a'},{name:'c'}))
   self.assertFalse(bench.verified_code_history({'config/piper.urdf':'a'},{'config/piper.urdf':'c'}))

 def test_contact_and_fit_failure_do_not_trigger_base_move(self):
  for reason in ('BAD_CONTACT','SUSTAINED_CONTACT_LOSS','SUSTAINED_RELATIVE_SLIP','UNOBSERVABLE'):
   self.assertFalse(eligible({'rows':[{'status':reason}],'trial_candidates':[]}))
  self.assertFalse(eligible({'rows':[{'status':'NO_IK'}],'trial_candidates':[{}]}))
  self.assertTrue(eligible({'rows':[{'status':'NO_PREGRASP_IK'}],'trial_candidates':[]}))

 def test_search_never_changes_height_or_handle(self):
  visual={'anchor_world_m':[.4,.2,.3],'outward_normal_world':[0.,-1.,0.]}
  before=dict(visual)
  poses=candidate_bases([.5,-.55,-.1,150.],visual,{'azimuth_offsets_deg':[-45,0,45],
             'standoff_m':[.35,.45,.55],'yaw_offsets_deg':[-30,0,30]})
  self.assertEqual(len(poses),27)
  self.assertTrue(all(p[2]==-.1 for p in poses))
  self.assertEqual(visual,before)

 def test_only_support_moves_in_scene_export(self):
  export={'shapes':[{'path':'/World/r1a7_pedestal','world_transform':np.eye(4).tolist()},
                    {'path':'/World/cabinet/handle','world_transform':np.eye(4).tolist()}]}
  result=at_base(export,[0,0,-.1,0],[.2,.1,-.1,30])
  self.assertEqual(result['shapes'][1],export['shapes'][1])
  self.assertEqual(export['shapes'][0]['world_transform'],np.eye(4).tolist())

 def test_visual_prior_does_not_need_joint_metadata(self):
  manifest={'moving_source_bounds':[[-.2,-.1,-.02],[.2,.1,.02]],'scale_source_to_meters':1.}
  visual={'outward_normal_world':[0.,-1.,0.],'anchor_world_m':[.18,-.12,0.]}
  prior=visual_prior(visual,manifest,np.eye(4),{'axis_line_offset_m':.02,'axis_perturbation_deg':5.})
  self.assertEqual(prior['joint_type'],'revolute')
  np.testing.assert_allclose(prior['revolute']['axis'],[-np.sin(np.deg2rad(5)),0,np.cos(np.deg2rad(5))])
  self.assertLess(prior['revolute']['point_on_axis'][0],-.19)


if __name__=='__main__':unittest.main()
