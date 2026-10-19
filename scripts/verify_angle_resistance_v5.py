"""Bounded synthetic/native-property-interface checks, no Isaac episodes."""
import json,sys,tempfile,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from verify_estimator_v31 import synthetic_rows,PRIOR
from xarm_ft_v31_estimator import process,common_filter
from xarm_ft_v5_estimator import samples_from_piece,basis,fit,analyze,matrix,CONFIG

def run():
    start=time.perf_counter();checks=[]
    for controls in [[.04,.04,.04],[.025,.065,.025]]:
        samples=[]
        for k in range(3):
            rows,v,known=synthetic_rows(role='FIT_SYNTHETIC_%d'%k,mass=.75)
            q=np.deg2rad([r['door_angle_deg'] for r in rows]);new=basis(q,2)@controls
            for i,r in enumerate(rows):
                T=np.array(r['wrist_ft']['T_world_sensor']);p=T[:3,3];delta=(new[i]-.04)*np.sign(v[i]);force=delta*np.cross([0,0,1],p)/np.linalg.norm(p[:2])**2
                r['wrist_ft']['raw_wrench_sensor'][:3]+=T[:3,:3].T@force
            d=process(rows,PRIOR,[0]*6,v);d['block_id']='FIT_SYNTHETIC_%d'%k;ss=samples_from_piece(d);samples+=ss
            expected=common_filter(basis(q,2)*np.sign(v)[:,None]);assert np.max(abs(expected-d['angle_features'][2]))==0
            assert not np.allclose(expected,basis(common_filter(q),2)*np.sign(common_filter(v))[:,None])
        m=fit(samples,2,.0001);assert abs(m['parameters'][0]-.3)<.002
        assert np.max(abs(np.array(m['parameters'][2:])-controls))<.002,m
        a=analyze(samples);base=fit(samples,0,0)
        from update_probing_v31_model import conditional_model
        assert base['parameters']==conditional_model(samples)['parameters']
        if controls[0]==controls[1]:assert a['selected_model']=='constant'
        else:assert a['selected_model']=='angle_coulomb',a['selected_model']
        checks.append(dict(known_controls=controls,parameters=m['parameters'],selected=a['selected_model'],constant_exact_V31=True,common_product_filter_verified=True))
    # Immutable operation identities: only passive plant and numeric worker differ.
    from interaction_runtime.probing_v5 import OperationsV5
    from interaction_runtime.recovery_v44 import OperationsV44
    for name in ['observe','evaluate_view','move_and_observe','build_geometry','validate_execution','acquire','attempt','interact','release_park','push']:
        assert getattr(OperationsV5,name) is getattr(OperationsV44,name),name
    # Exercise the existing native-law owner with a readback-capable tensor mock.
    from xarm_ft_v1_scene import install_native_law,law_value
    from types import SimpleNamespace
    class View:
        def __init__(self):self.values=np.array([[[.02,.02,.27]]]);self.writes=0
        def get_dof_friction_properties(self):return self.values.copy()
        def set_dof_friction_properties(self,values,indices):self.values=values.copy();self.writes+=1
    with tempfile.TemporaryDirectory() as tmp:
        p=Path(tmp);law=dict(base_nm=.02,amplitude_nm=.01,center_rad=.17,width_rad=.05);(p/'hidden.json').write_text(json.dumps([dict(tau_c=.02,law=law)]));view=View();angle=[0.]
        g=dict(job=dict(private_physics_configuration=str(p/'hidden.json'),identification_case=0),scene=dict(articulation=SimpleNamespace(_articulation_view=SimpleNamespace(_physics_view=view))),a=SimpleNamespace(output=p),actual=lambda:angle[0],tick=0,dt=1/240)
        update=install_native_law(g)
        for i,q in enumerate(np.linspace(0,.35,100)):
            angle[0]=float(q);g['tick']=i*240;update();assert view.values[0,0,2]==.27;assert abs(view.values[0,0,0]-law_value(q,law))<1e-12
        assert view.writes==101
    return dict(status='PASS_OFFLINE',no_Isaac=True,synthetic=checks,native_property_readback_mock=True,passive_viscous_preserved=True,frozen_operations_identity=True,hidden_truth_not_imported_by_estimator=True,wall_s=time.perf_counter()-start,configuration=CONFIG)
if __name__=='__main__':print(json.dumps(run(),indent=2))
