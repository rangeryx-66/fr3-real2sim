"""Independent prismatic effort controller and explicit contact materials."""
import os
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdPhysics,UsdShade,PhysxSchema
from isaacsim.core.utils.types import ArticulationAction

def configure_stage_materials(stage,pad_paths,mu):
    for path,value in [('/World/calibration_pad',mu),('/World/calibration_target',1.),('/World/calibration_table',.7)]:
        m=UsdShade.Material.Define(stage,path);a=UsdPhysics.MaterialAPI.Apply(m.GetPrim());a.CreateStaticFrictionAttr(value);a.CreateDynamicFrictionAttr(value);a.CreateRestitutionAttr(0.)
        ph=PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim());ph.CreateFrictionCombineModeAttr('multiply');ph.CreateRestitutionCombineModeAttr('multiply')
    count=0
    for p in stage.Traverse():
        if not p.HasAPI(UsdPhysics.CollisionAPI):continue
        path=str(p.GetPath());material=None
        if path in pad_paths:material='/World/calibration_pad';count+=1
        elif path.startswith('/World/box/'):material='/World/calibration_target'
        elif path.startswith('/World/table/'):material='/World/calibration_table'
        if material:UsdShade.MaterialBindingAPI.Apply(p).Bind(UsdShade.Material(stage.GetPrimAtPath(material)),bindingStrength='strongerThanDescendants',materialPurpose='physics')
    assert count==2,(count,pad_paths)

class HandCalibration:
    def __init__(self,robot,contacts,stage,fingers,finger_paths,dt):
        self.robot=robot;self.contacts=contacts;self.stage=stage;self.fingers=fingers;self.dt=dt
        self.phase='IDLE';self.mode='position';self.goal=0.;self.integral=np.zeros(2);self.filtered=np.zeros(2);self.effort=np.zeros(2);self.contact_seen=np.zeros(2,dtype=bool);self.ramp=np.zeros(2)
        self.kp,self.kd=[np.array(x,copy=True) for x in robot.get_articulation_controller().get_gains()]
        self.pad_paths=[path+'/box_3' for path in finger_paths]
        from isaacsim.core.simulation_manager import SimulationManager
        self.audit_sim=SimulationManager.get_physics_sim_view()
        self.target_view=self.audit_sim.create_rigid_body_view('/World/box')
        self.original_materials=self.material_audit()
        props=np.asarray(self.contacts._physics_view.get_material_properties())
        self.mu=float(os.environ.get('CALIBRATION_MU','.7'))
        self.pad_indices=[[3],[3]]  # box_3, verified against cold binding and shape order
        assert np.allclose(props[:,3,:2],self.mu),('cold pad material mismatch',props.tolist())
        self.normal=np.zeros(2)
        self.relative=np.eye(4);self.gate_active=False;self.gate={}
    def observe(self,bp,bq,tp,tq,forces):
        H=Rotation.from_quat(np.roll(tq,-1)).as_matrix();O=Rotation.from_quat(np.roll(bq,-1)).as_matrix()
        self.relative[:3,:3]=H.T@O;self.relative[:3,3]=H.T@(np.asarray(bp)-tp)
        if self.gate_active:
            self.gate['max_translation_m']=max(self.gate['max_translation_m'],float(np.linalg.norm(self.relative[:3,3]-self.gate_ref[:3,3])))
            self.gate['max_rotation_deg']=max(self.gate['max_rotation_deg'],float(np.rad2deg(Rotation.from_matrix(self.gate_ref[:3,:3].T@self.relative[:3,:3]).magnitude())))
            self.gap=self.gap+self.dt if min(forces)<=.1 else 0.
            self.gate['max_contact_gap_s']=max(self.gate['max_contact_gap_s'],self.gap)
            self.gate['terminal_bilateral']=bool(min(forces)>.1)
    def gate_start(self):
        self.gate_ref=self.relative.copy();self.gate_active=True;self.gap=0.
        self.gate=dict(max_translation_m=0.,max_rotation_deg=0.,max_contact_gap_s=0.,terminal_bilateral=False,close_T_TCP_target=self.relative.tolist())
    def gate_finish(self):
        self.gate_active=False
        self.gate.update(passed=self.gate['max_translation_m']<=.002 and self.gate['max_rotation_deg']<=3. and self.gate['max_contact_gap_s']<=.05 and self.gate['terminal_bilateral'],micro_T_TCP_target=self.relative.tolist())
        return self.gate
    def material_audit(self):
        rows=[]
        for p in self.stage.Traverse():
            if p.HasAPI(UsdPhysics.CollisionAPI) and ('finger/' in str(p.GetPath()) or str(p.GetPath()).startswith('/World/box/')):
                m,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial(materialPurpose='physics')
                row=dict(collider=str(p.GetPath()),material=str(m.GetPath()) if m else None)
                if m:
                    a=UsdPhysics.MaterialAPI(m.GetPrim());ph=PhysxSchema.PhysxMaterialAPI(m.GetPrim())
                    row.update(static=a.GetStaticFrictionAttr().Get(),dynamic=a.GetDynamicFrictionAttr().Get(),combine=ph.GetFrictionCombineModeAttr().Get())
                rows.append(row)
        out=dict(bindings=rows)
        try:out['runtime_finger_material_properties']=np.asarray(self.contacts._physics_view.get_material_properties()).tolist()
        except Exception as e:out['runtime_query_error']=str(e)
        try:
            out['runtime_target_material_properties']=np.asarray(self.target_view.get_material_properties()).tolist()
            out['target_mass_kg']=np.asarray(self.target_view.get_masses()).tolist()
            out['target_COM_local']=np.asarray(self.target_view.get_coms()).tolist()
            out['target_inertia']=np.asarray(self.target_view.get_inertias()).tolist()
        except Exception as e:out['target_query_error']=str(e)
        return out
    def configure_material(self,mu):
        # Friction is fixed at stage initialization, never hot-switched.
        assert abs(mu-self.mu)<1e-8,('restart simulator for requested friction',mu,self.mu)
        audit=self.material_audit()
        props=np.asarray(audit['runtime_finger_material_properties'])
        target=np.asarray(audit['runtime_target_material_properties'])
        assert np.allclose(props[:,3,:2],mu,atol=1e-6),props[:,3,:].tolist()
        assert np.allclose(target[...,:2],1.,atol=1e-6)
        return audit
    def position(self):
        self.mode='position';self.goal=0.;self.gate_active=False;self.robot.get_articulation_controller().set_gains(kps=self.kp,kds=self.kd,save_to_usd=False)
        self.robot.apply_action(ArticulationAction(joint_efforts=np.zeros(2),joint_indices=self.fingers))
    def close_force(self,total_force):
        assert 0<total_force<=60
        kp,kd=[np.array(x,copy=True) for x in self.robot.get_articulation_controller().get_gains()];kp[self.fingers]=0.;kd[self.fingers]=0.
        self.robot.get_articulation_controller().set_gains(kps=kp,kds=kd,save_to_usd=False)
        self.mode='force';self.goal=total_force/2;self.integral[:]=0;self.filtered[:]=0;self.contact_seen[:]=False;self.ramp[:]=0
    def step(self):
        # Contact normal force magnitude (sum of manifold normal impulses / dt),
        # excluding tangential friction; separate values for each finger.
        values=self.contacts.get_contact_force_data(dt=self.dt)
        ff=np.asarray(values[0]).reshape(-1);count=np.asarray(values[4]);index=np.asarray(values[5])
        self.normal=np.array([ff[int(index[i,0]):int(index[i,0])+int(count[i,0])].sum() for i in range(2)])
        self.filtered+=(self.normal-self.filtered)*(self.dt/(.03+self.dt))
        if self.mode!='force':return
        velocity=self.robot.get_joint_velocities()[self.fingers]
        self.contact_seen|=self.filtered>.03
        if not self.contact_seen.any():
            self.effort[:]=np.clip(8*(-.015-float(velocity.mean())),-.5,.5)
        else:
            self.ramp[:]=min(self.goal,self.ramp[0]+self.goal*self.dt/.4)
            error=self.ramp[0]-float(self.filtered.mean())
            self.integral[:]=np.clip(self.integral[0]+error*self.dt,-self.goal*.5,self.goal*.5)
            self.effort[:]=-np.clip(self.ramp[0]+.1*error+self.integral[0]+2*float(velocity.mean()),0,min(35.,self.goal*1.5))
        self.robot.apply_action(ArticulationAction(joint_efforts=self.effort.copy(),joint_indices=self.fingers))
    def state(self):return dict(mode=self.mode,total_force_target_N=self.goal*2,normal_force_N=self.normal.tolist(),filtered_force_N=self.filtered.tolist(),effort_N=self.effort.tolist(),contact_seen=self.contact_seen.tolist())
