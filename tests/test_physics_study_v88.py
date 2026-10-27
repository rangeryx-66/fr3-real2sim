import hashlib,json,tempfile,types,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from agentic_interaction_v88.physics_study import validate,material_values,constructor_scope,initialize_reference


class StudyTests(unittest.TestCase):
    def test_explicit_registration_only(self):
        with self.assertRaises(ValueError):validate({'role':'updated'})
        with self.assertRaises(ValueError):validate(dict(schema='physics-aware-contact-v1',role='oracle'))
        self.assertEqual(validate(dict(schema='physics-aware-contact-v1',role='updated'))['role'],'updated')

    def test_material_and_static_rail_values_are_physical_inputs_not_outcomes(self):
        case=dict(contact_material=dict(static_friction=.3,dynamic_friction=.2,combine_mode='average'),
            static_rail_multiplier=1.4)
        self.assertEqual(material_values(case)[1],1.4)
        case['contact_material']['combine_mode']='max'
        with self.assertRaises(ValueError):material_values(case)

    def test_baseline_inheritance_is_explicit_and_not_an_assumed_coefficient(self):
        self.assertEqual(material_values(dict(inherit_contact_material=True)),(None,1.))
        with self.assertRaises(ValueError):material_values({})

    def test_only_committed_current_unexecuted_reference_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);obs=p/'obs';obs.mkdir();(obs/'rgb.png').write_bytes(b'fresh observed pixels')
            fresh=p/'fresh.json';fresh.write_text(json.dumps(dict(kind='fresh_matched_observation_reference_v81',
                physical_acquisitions_at_export=0,saved_successful_grasp=False,GT_grasp_used=False)))
            digest=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
            manifest=p/'reference.json';manifest.write_text(json.dumps(dict(
                fresh_reference=dict(path=str(fresh),sha256=digest(fresh)),observation_folder=str(obs),
                observation_files={'rgb.png':digest(obs/'rgb.png')})))
            config=dict(schema='physics-aware-contact-v1',role='reactive',
                reference=dict(path=str(manifest),sha256=digest(manifest)))
            validate(config)
            (obs/'rgb.png').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'REFERENCE_OBSERVATION_CHANGED'):validate(config)

    def test_native_callback_sets_static_dynamic_resistance_and_preserves_viscous(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);case=dict(contact_material=dict(static_friction=.3,dynamic_friction=.2,
                combine_mode='average'),static_rail_multiplier=1.5,law=dict(base=.4))
            file=p/'case.json';file.write_text(json.dumps([case]))
            class View:
                def __init__(self):self.data=np.array([[[.4,.4,12.]]]);self.calls=0
                def get_dof_friction_properties(self):return self.data.copy()
                def set_dof_friction_properties(self,value,index):self.data=value.copy();self.calls+=1
            view=View();plant=types.ModuleType('xarm_ft_v7_scene')
            original=Mock();plant.author=Mock();plant.install_native_law=original
            plant.law_value=lambda q,law:law['base']
            job=dict(v88_physics_study=dict(schema='physics-aware-contact-v1',role='identify_m1'),
                private_physics_configuration=str(file),identification_case=0,output=str(p))
            g=dict(job=job,scene=dict(articulation=NS(_articulation_view=NS(_physics_view=view))),
                actual=lambda:0.,tick=0)
            with patch.dict('sys.modules',{'xarm_ft_v7_scene':plant}):
                with constructor_scope(job):
                    callback=plant.install_native_law(g);callback()
                    np.testing.assert_allclose(view.data,[[[.6,.4,12.]]])
                self.assertIs(plant.install_native_law,original)
            original.assert_called_once_with(g)
            self.assertTrue((p/'PRIVATE_STUDY_PASSIVE_READBACK.json').exists())

    def test_matched_input_keeps_real_sensing_evidence_and_never_claims_fresh_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);obs=p/'sealed';obs.mkdir();(obs/'rgb.png').write_bytes(b'pixels')
            out=p/'episode';out.mkdir()
            manifest=p/'reference.json';manifest.write_text(json.dumps(dict(
                initial_object_coordinate_native=0.,initial_base_pose=[0.,0.,0.,0.],
                observation_folder=str(obs),source_episode='current_study_before_first_contact')))
            r=NS(actual=lambda:0.,read_base=Mock(),base=np.zeros(4),hold=Mock(),
                supervisor=NS(latched=None),grasp_reference=None,filtered=lambda:[0.,0.],output=out,
                g=dict(wrist_config=dict(settle_s=.5),agent_observations=[],observation_generations=[]))
            def observe():
                r.g['agent_observations'].append(dict(folder='real_current_capture',world_sim_s=1.))
                return dict(actual_image='real_current_capture')
            ops=NS(observe=Mock(side_effect=observe))
            result=initialize_reference(r,ops,dict(reference=dict(path=str(manifest))))
            ops.observe.assert_called_once()
            self.assertEqual(r.g['observation_generations'][0][0]['folder'],'real_current_capture')
            self.assertEqual(result['actual_instance_sensing']['actual_image'],'real_current_capture')
            self.assertFalse(result['fresh_vision_generalization_claim'])
            self.assertFalse(r.g['agent_observations'][0]['physical_capture_in_this_episode'])
            self.assertEqual(r.g['agent_observations'][0]['world_sim_s'],0.)


if __name__=='__main__':unittest.main()
