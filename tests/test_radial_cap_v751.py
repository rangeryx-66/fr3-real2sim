import unittest
import numpy as np
import sys,tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from native_g1_radial_families_v751 import radial_transforms


class RadialTest(unittest.TestCase):
    def test_compact_center_depth_and_orthonormality_are_preserved(self):
        T=np.eye(4);T[:3,3]=[.1,-.3,.2];anchor=np.array([.1,-.2,.2]);normal=np.array([0,-1.,0])
        poses=list(radial_transforms(T,anchor,normal));self.assertEqual(len(poses),8)
        for angle,P in poses:
            np.testing.assert_allclose(P[:3,:3].T@P[:3,:3],np.eye(3),atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(P[:3,:3]),1.)
            self.assertAlmostEqual((P[:3,3]-anchor)@normal,(T[:3,3]-anchor)@normal)
            self.assertAlmostEqual(np.linalg.norm(P[:3,3]-anchor),np.linalg.norm(T[:3,3]-anchor))

    def test_actual_selected_scorer_receives_radial_candidates_in_both_modes(self):
        from native_g1_radial_families_v751 import build_radial_conditioned_geometry
        import native_g1_radial_families_v751 as radial
        for mode in ['frozen_visual','native_bidirectional']:
            with tempfile.TemporaryDirectory()as tmp:
                seen=[]
                def score(candidates,*args):seen.append((mode,candidates));return candidates
                multi=SimpleNamespace(rescore=score)
                recovery=SimpleNamespace(write=lambda *args:None)
                incremental=SimpleNamespace(build_geometry=lambda g:dict(plan=False,ranked=multi.rescore(['BAR'],None,None,None,None,None,{},Path(tmp))))
                def catalog(*args):return dict(candidates=[],observation_sha256='test')
                g=dict(contact_attempts=[],a=SimpleNamespace(output=Path(tmp)),policy=dict(preload_n=1.),agent_observations=[])
                with patch.dict(sys.modules,{'native_g1_contact_recovery':recovery,'native_g1_multiview_input':multi,'native_g1_agent_incremental_activation':incremental}),patch.object(radial,'generate',return_value=['RADIAL']),patch('interaction_runtime.recovery_candidates_v7.catalog',catalog),patch('interaction_runtime.recovery_candidates_v7.order',lambda s,*args:s):
                    result=build_radial_conditioned_geometry(g)
                self.assertEqual(seen,[(mode,['RADIAL'])]);self.assertEqual(result['ranked'],['RADIAL'])
                self.assertIs(multi.rescore,score)
                self.assertIn('recovery_conditions_sha256',g['recovery_candidate_catalog'])

    def test_missing_radial_input_never_scores_bar_fallback(self):
        from native_g1_radial_families_v751 import build_radial_conditioned_geometry
        import native_g1_radial_families_v751 as radial
        with tempfile.TemporaryDirectory()as tmp:
            score=SimpleNamespace(called=False)
            def scoring(*args):score.called=True;return []
            multi=SimpleNamespace(rescore=scoring)
            incremental=SimpleNamespace(build_geometry=lambda g:multi.rescore(['BAR'],None,None,None,None,None,{},Path(tmp)))
            g=dict(contact_attempts=[],a=SimpleNamespace(output=Path(tmp)),policy=dict(preload_n=1.),agent_observations=[])
            with patch.dict(sys.modules,{'native_g1_contact_recovery':SimpleNamespace(),'native_g1_multiview_input':multi,'native_g1_agent_incremental_activation':incremental}),patch.object(radial,'generate',side_effect=ValueError('unsupported cap evidence')):
                with self.assertRaises(ValueError):build_radial_conditioned_geometry(g)
            self.assertFalse(score.called);self.assertIs(multi.rescore,scoring)

    def test_inherited_selector_cannot_overwrite_the_generator(self):
        from native_g1_radial_families_v751 import radial_operations_class
        import native_g1_radial_families_v751 as radial
        from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75
        for mode in [False,True]:
            with tempfile.TemporaryDirectory()as tmp:
                seen=[]
                def selected(candidates,*args):seen.append(('bidirectional',candidates));return candidates
                def initial(candidates,*args):seen.append(('frozen_visual',candidates));return candidates
                multi=SimpleNamespace(rescore=initial);original=lambda g:dict(plan=False)
                recovery=SimpleNamespace(build_geometry=original,write=lambda *args:None)
                incremental=SimpleNamespace(build_geometry=lambda g:dict(plan=False,ranked=multi.rescore(['BAR'],None,None,None,None,None,{},Path(tmp))))
                cls=radial_operations_class();ops=cls.__new__(cls);ops.radial_selected=True
                ops.r=SimpleNamespace(output=Path(tmp));ops.g=dict(contact_attempts=[],a=SimpleNamespace(output=Path(tmp)),policy=dict(preload_n=1.),agent_observations=[])
                def inherited(ops):
                    # This is the real ordering that overwrote the old outer
                    # patch: select ranker first, enter recovery owner second.
                    if mode:
                        with patch.object(multi,'rescore',selected):return recovery.build_geometry(ops.g)
                    return recovery.build_geometry(ops.g)
                with patch.dict(sys.modules,{'native_g1_contact_recovery':recovery,'native_g1_multiview_input':multi,'native_g1_agent_incremental_activation':incremental}),patch.object(radial,'generate',return_value=['RADIAL']),patch('interaction_runtime.recovery_candidates_v7.catalog',lambda *args:dict(candidates=[],observation_sha256='test')),patch('interaction_runtime.recovery_candidates_v7.order',lambda s,*args:s),patch.object(PrismaticEvidenceOperationsV75,'build_geometry',inherited):
                    result=ops.build_geometry()
                self.assertEqual(seen,[('bidirectional'if mode else'frozen_visual',['RADIAL'])]);self.assertEqual(result['ranked'],['RADIAL'])
                self.assertIs(recovery.build_geometry,original);self.assertIs(multi.rescore,initial)

    def test_candidate_request_keeps_cap_family_until_explicit_change(self):
        from native_g1_radial_families_v751 import radial_operations_class
        from interaction_runtime.prismatic_v751 import PrismaticOperationsV751
        cls=radial_operations_class();ops=cls.__new__(cls);ops.radial_selected=True
        with patch.object(PrismaticOperationsV751,'request_plan',return_value=dict(plan=False)):
            ops.request_plan(dict(candidate_request={}))
            self.assertTrue(ops.radial_selected)
            ops.request_plan(dict(grasp_family='existing_bar'))
            self.assertFalse(ops.radial_selected)


if __name__=='__main__':unittest.main()
