import ast
import importlib.util
import sys
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


class StructureBoundaryTests(unittest.TestCase):
    def test_absent_fixture_never_invokes_geometry_builder(self):
        from cross_object_structure.scene import adapt_no_fixture
        from types import SimpleNamespace
        import numpy as np
        statement='fixture=fixture_box(manifest,asset_chain,asset_urdf,asset_rotation,asset_xyz,a.fixture_height_m)'
        calls=[]
        def builder(*args):calls.append(args);return {'real_fixture':'unchanged'}
        namespace=dict(fixture_box=builder,manifest={},asset_chain=None,asset_urdf=None,asset_rotation=None,
                       asset_xyz=np.array([.1,.2,0.]),a=SimpleNamespace(fixture_height_m=0.))
        exec(adapt_no_fixture(statement,{}),namespace)
        self.assertEqual(calls,[]);self.assertTrue(namespace['fixture']['absent'])
        namespace['a'].fixture_height_m=.1
        exec(adapt_no_fixture(statement,{}),namespace)
        self.assertEqual(len(calls),1);self.assertEqual(namespace['fixture'],{'real_fixture':'unchanged'})

    def test_new_actions_cannot_bypass_existing_force_guard(self):
        spec=importlib.util.spec_from_file_location('structure_entry',ROOT/'scripts/run_cross_object_structure_episode.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        tree=ast.parse(module.source())
        checks=[n for n in ast.walk(tree) if isinstance(n,ast.If) and
                any(isinstance(c,ast.Constant) and c.value=='EXISTING_LOW_PRELOAD_FORCE_LIMIT' for c in ast.walk(n))]
        check=min(checks,key=lambda n:len(list(ast.walk(n))))
        condition=compile(ast.Expression(check.test),'<frozen_force_guard>','eval')
        for phase in ['SLOW_CLOSE','EXPLORATORY','ESTIMATED_FOLLOW','REFINEMENT_REVERSE',
                      'STRUCTURE_VALIDATION','HELDOUT_MANIPULATION','HELDOUT_DWELL']:
            scope={'phase':phase,'s':{'forces_n':{'left':3.,'right':.2}},'policy':{'max_pad_load_n':2.}}
            self.assertTrue(eval(condition,{},scope),phase)
            scope['s']['forces_n']['left']=1.
            self.assertFalse(eval(condition,{},scope),phase)

    def test_unreachable_not_counted_as_articulation_failure(self):
        from cross_object_structure.reporting import taxonomy
        self.assertEqual(taxonomy({'status':'NO_MOBILE_RECOVERY','fixed_plan_statuses':['NO_IK']},{}),'DEPLOYMENT_UNREACHABLE')
        self.assertEqual(taxonomy({'status':'UNOBSERVABLE','grasp_feasible':True,'grasp':True},
                                  {'failure_phase':'EXPLORATORY','estimate':{'joint_type':'UNOBSERVABLE'}}),'ARTICULATION_UNOBSERVABLE')
        self.assertEqual(taxonomy({'status':'SUCCESS','success':True},
                                  {'failure_phase':None,'first_failure_state':None}),'SUCCESS')
        self.assertEqual(taxonomy({'status':'DANGEROUS_LOADED_CONTACT:gripper_link2:panel','grasp_feasible':True},
                                  {'failure_phase':'PREGRASP'}),'APPROACH_COLLISION')

    def test_recovery_adapter_does_not_duplicate_chassis(self):
        from cross_object_structure.benchmark import recovery_export
        original={'shapes':[{'path':'/World/mobile_chassis'}, {'path':'/World/table'}, {'path':'/World/r1a7_pedestal'}]}
        normal=recovery_export(original)
        self.assertEqual(len(original['shapes']),3)
        self.assertEqual([s['path'] for s in normal['shapes']],['/World/table','/World/r1a7_pedestal'])
        # scene_at owns chassis authoring; environment and support must remain.
        normal['shapes'].append({'path':'/World/mobile_chassis'})
        self.assertEqual(sum(s['path']=='/World/mobile_chassis' for s in normal['shapes']),1)

    def test_heldout_not_used_for_refinement(self):
        import numpy as np
        from cross_object_structure.refinement import measured_action
        # A large held-out trajectory cannot make an unobservable training
        # segment observable. Filtering receives phase-separated actions only.
        rows=[]
        for i in range(600):
            T=np.eye(4);T[0,3]=i*.001
            rows.append({'phase':'HELDOUT_MANIPULATION','t':i/240.,'T_tcp':T.tolist(),
                         'forces_n':{'left':.2,'right':.2}})
        poses,_=measured_action(rows,'ESTIMATED_FOLLOW')
        self.assertEqual(len(poses),0)


if __name__=='__main__':unittest.main()
