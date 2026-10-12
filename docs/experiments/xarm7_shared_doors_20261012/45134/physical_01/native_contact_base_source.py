class NativeOwnershipReports:
    def __init__(self,stage,target_paths,physics_dt,allowed_pad_targets=None,world=None,sample_provider=None):
        from pxr import PhysicsSchemaTools,Usd
        import omni.physx
        self.dt=physics_dt;self.targets=set(target_paths);self.allowed_pad_targets=set(allowed_pad_targets or target_paths);self.rows=[];self.paths={};self.decode=PhysicsSchemaTools.intToSdfPath
        for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
            owner=p.GetAttribute('contact:owner').Get()
            if owner:self.paths[str(p.GetPath())]=(p.GetAttribute('contact:finger').Get(),owner)
        self.finger_body_paths={path.rsplit('/contact_owned/',1)[0]:finger for path,(finger,owner) in self.paths.items()}
        self.decoded_paths={};self.contact_header_count=0
        self.world=world;self.sample_provider=sample_provider;self.window_dt=0.;self.window_steps=0;self.physics_time=0.;self.physics_steps=[]
        if world is not None:world.add_physics_callback('piper_owned_contact_clock',self.on_physics_step)
        self.subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self.callback)

    def clear(self):self.rows=[];self.window_dt=0.;self.window_steps=0

    def on_physics_step(self,dt):
        if self.physics_steps:self.physics_steps[-1]['ownership']=self.summarize(self.physics_steps[-1]['contacts'],self.physics_steps[-1]['physics_dt_s'])
        dt=float(dt);self.window_dt+=dt;self.window_steps+=1;self.physics_time+=dt
        if self.sample_provider is not None:
            sample=self.sample_provider();sample.update(t=self.physics_time-dt,physics_dt_s=dt,geometry_pose_time='native rigid-body tensor pose immediately before physics integration',contacts=[])
            self.physics_steps.append(sample)

    def callback(self,headers,data):
        self.contact_header_count+=len(headers)
        def path(key):
            if key not in self.decoded_paths:self.decoded_paths[key]=str(self.decode(key))
            return self.decoded_paths[key]
        for h in headers:
            a,b=path(h.collider0),path(h.collider1)
            matches=lambda path:any(path==t or path.startswith(t+'/') for t in self.targets)
            if a in self.paths and matches(b):owned=a;target=b
            elif b in self.paths and matches(a):owned=b;target=a
            else:
                bodies=self.finger_body_paths
                unknown=next(((path,other,finger) for path,other in [(a,b),(b,a)] for body,finger in bodies.items() if path.startswith(body+'/') and matches(other)),None)
                if unknown is None:continue
                owned,target,finger=unknown;self.paths[owned]=(finger,'unknown')
            finger,owner=self.paths[owned]
            for d in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
                impulse=np.array([d.impulse.x,d.impulse.y,d.impulse.z]);force=float(np.linalg.norm(impulse)/self.dt)
                if owned==b:impulse=-impulse
                contact={'collider':owned,'target':target,'finger':finger,'owner':owner,'allowed_pad_target':any(target==t or target.startswith(t+'/') for t in self.allowed_pad_targets),'force_n':force,'impulse_world_ns':impulse.tolist(),'separation_m':float(d.separation),'face0':int(d.face_index0),'face1':int(d.face_index1)}
                contact.update(collider0=a,collider1=b,normal_reference='native report direction; original collider ordering preserved')
                try:
                    contact['contact_point_world_m']=[float(d.position.x),float(d.position.y),float(d.position.z)]
                    contact['native_normal_world']=[float(d.normal.x),float(d.normal.y),float(d.normal.z)]
                except Exception as error:
                    # Diagnostic metadata must never suppress a safety event.
                    contact['diagnostic_attribute_error']=repr(error)
                self.rows.append(contact)
                if self.physics_steps:self.physics_steps[-1]['contacts'].append(contact)

    def summarize(self,rows,dt):
        active=[x for x in rows if x['force_n']>0]
        if dt<=0:raise RuntimeError('native physics contact interval was not measured')
        impulses={n:np.sum([x['impulse_world_ns'] for x in active if x['finger']==n and x['owner']=='pad'],axis=0) if any(x['finger']==n and x['owner']=='pad' for x in active) else np.zeros(3) for n in FINGERS}
        return {'contacts':rows.copy(),'pad_forces_n':{n:float(np.linalg.norm(v)/dt) for n,v in impulses.items()},'pad_force_impulse_sum_over_nominal_dt_n':{n:sum(x['force_n'] for x in active if x['finger']==n and x['owner']=='pad') for n in FINGERS},'metal_contacts':sum(x['owner']!='pad' for x in active),'pad_target_violations':sum(x['owner']=='pad' and not x['allowed_pad_target'] for x in active),'unknown_allowed':False,'classification':'native collider ID; no position projection','force_rule':'net native impulse divided by measured physics interval, not by control-loop dt'}

    def state(self):
        dt=self.window_dt if self.world is not None else self.dt
        result=self.summarize(self.rows,dt);result.update(physics_window_dt_s=dt,physics_substeps=self.window_steps,physics_time_s=self.physics_time)
        if self.physics_steps:self.physics_steps[-1]['ownership']=self.summarize(self.physics_steps[-1]['contacts'],self.physics_steps[-1]['physics_dt_s'])
        return result
