class WholeFingerReports(NativeOwnershipReports):
    def __init__(self,stage,export,allowed,world,dt,provider):
        super().__init__(stage,['/World'],dt,allowed,world,provider)
        self.entries={e['path']:e for e in export['shapes']}
        for e in export['shapes']:
            if not e['path'].startswith('/World/Piper'):continue
            body=e.get('rigid_body_path','').split('/')[-1]
            self.paths[e['path']]=(body,'whole_finger' if body in FINGERS else 'robot_body')
            p=stage.GetPrimAtPath(e.get('rigid_body_path',''))
            if p:PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
        self.finger_body_paths={e['rigid_body_path']:e['rigid_body_path'].split('/')[-1] for e in export['shapes'] if e['path'] in self.paths}

    def summarize(self,rows,dt):
        impulses={n:np.sum([c['impulse_world_ns'] for c in rows if c['finger']==n and c['allowed_pad_target']],axis=0) if any(c['finger']==n and c['allowed_pad_target'] for c in rows) else np.zeros(3) for n in FINGERS}
        return {'contacts':rows.copy(),'finger_handle_forces_n':{n:float(np.linalg.norm(v)/dt) for n,v in impulses.items()},'physics_window_dt_s':dt,'classification':'whole official finger; task-region contacts, not pad triangle ownership'}
