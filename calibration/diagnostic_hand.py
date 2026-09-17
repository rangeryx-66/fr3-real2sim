"""Measurement-only extension; frozen force controller is inherited unchanged."""
import os,json,numpy as np
from pathlib import Path
from pxr import UsdPhysics
from hand_force_control import HandCalibration
class DiagnosticHand(HandCalibration):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.extra={}
        audit={}
        for p in self.stage.Traverse():
            if p.GetTypeName()=='PhysicsScene':
                audit[str(p.GetPath())]={a.GetName():str(a.Get()) for a in p.GetAttributes()}
        audit['body']={a.GetName():str(a.Get()) for a in self.stage.GetPrimAtPath('/World/box').GetAttributes()}
        for key,view in [('target',self.target_view),('simulation',self.audit_sim)]:
            audit[key+'_gravity_methods']=[x for x in dir(view) if 'gravit' in x.lower()]
            for method in audit[key+'_gravity_methods']:
                if method.startswith('get'):
                    try:audit[key+'_'+method]=str(getattr(view,method)())
                    except Exception as e:audit[key+'_'+method]=str(e)
        out=Path(__file__).resolve().parents[1]/'results/hand_calibration'/os.environ.get('DIAGNOSTIC_DIR','physics_diagnostic');out.mkdir(exist_ok=True);(out/'stage_audit.json').write_text(json.dumps(audit,indent=2))
        self.stage.Export(str(out/'stage.usda'))
        self.report_out=out;self.contact_pairs=set()
        from omni.physx import get_physx_simulation_interface
        self.report_subscription=get_physx_simulation_interface().subscribe_contact_report_events(self.contact_report)
    def contact_report(self,headers,data):
        from pxr import PhysicsSchemaTools
        changed=False
        for h in headers:
            a=str(PhysicsSchemaTools.intToSdfPath(h.collider0));b=str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            if 'finger' not in a+b or '/World/box/' not in a+b:continue
            for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                c=data[k];pair=(a,b,str(PhysicsSchemaTools.intToSdfPath(c.material0)),str(PhysicsSchemaTools.intToSdfPath(c.material1)))
                if pair not in self.contact_pairs:self.contact_pairs.add(pair);changed=True
        if changed:(self.report_out/'actual_contact_material_pairs.json').write_text(json.dumps(sorted(self.contact_pairs),indent=2))
    def step(self):
        super().step()
        try:
            f,p,c,i=self.contacts.get_friction_data(dt=self.dt);f=np.asarray(f);c=np.asarray(c);i=np.asarray(i)
            friction=np.array([f[int(i[k,0]):int(i[k,0])+int(c[k,0])].sum(axis=0) for k in range(2)])
            fn,_,n,_,nc,ni=self.contacts.get_contact_force_data(dt=self.dt);fn=np.asarray(fn);n=np.asarray(n);nc=np.asarray(nc);ni=np.asarray(ni)
            normal=np.array([(fn[int(ni[k,0]):int(ni[k,0])+int(nc[k,0])]*n[int(ni[k,0]):int(ni[k,0])+int(nc[k,0])]).sum(axis=0) for k in range(2)])
            self.extra=dict(friction_world_N=friction.tolist(),normal_world_N=normal.tolist(),friction_counts=c.tolist(),normal_counts=nc.tolist())
        except Exception as e:self.extra=dict(friction_error=str(e))
    def state(self):return dict(super().state(),force_diagnostic=self.extra)
