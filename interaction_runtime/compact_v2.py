"""One columnar copy of essential measurements; no per-frame collider dictionaries."""
from collections import deque
import hashlib,json,os,queue,threading
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent_command_protocol_v12 import atomic_json

def pose7(T):
    T=np.asarray(T);return np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()]

class CompactRecorder:
    def __init__(self,folder,chunk_records=8192):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True);self.chunk_records=8192
        self.count=0;self.saved=0;self.pending=[];self.pre=deque();self.chunks=[];self.issues=[];self.failed=[];self.segment=0;self.segments=[];self.router=[]
        self.metadata=dict(format='lossless compressed numeric NPZ; one copy',native_hz=240,unloaded_hz=10,phases=[],body_properties=[],modes=['UNLOADED','HANDLE','PANEL'])
        self.context=lambda:{};self.completed=lambda ready:None;self.queue=queue.Queue();self.thread=threading.Thread(target=self._worker,daemon=True);self.thread.start()
    def _worker(self):
        while True:
            item=self.queue.get()
            try:
                if item is None:return
                if isinstance(item,threading.Event):self._index();item.set();continue
                rows=item;keys=rows[0].keys();arrays={k:np.asarray([r[k] for r in rows]) for k in keys};file=self.folder/('%08d.npz'%int(arrays['step'][0]))
                with file.with_suffix('.pending').open('wb') as f:np.savez_compressed(f,**arrays)
                os.replace(file.with_suffix('.pending'),file)
                self.chunks.append(dict(file=file.name,records=len(rows),first_step=int(arrays['step'][0]),last_step=int(arrays['step'][-1]),bytes=file.stat().st_size,sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
                self._index()
            except Exception as e:
                self.issues.append(repr(e));self.failed.append(item)
                if isinstance(item,threading.Event):item.set()
            finally:self.queue.task_done()
    def _index(self):
        atomic_json(self.folder/'index.json',dict(metadata=self.metadata,physics_steps_seen=self.count,records=sum(c['records'] for c in self.chunks),chunks=list(self.chunks),segments=self.segments,issues=self.issues,raw_private=True))
    def begin(self,action,mode):
        self.segment=len(self.segments)+1;self.segments.append(dict(id=self.segment,role=action['role'],mode=mode,target_deg=action['target_deg'],profile=action.get('profile','constant'),first_step=self.count,completed=False))
    def end(self,result=None):
        if self.segment:
            self.segments[self.segment-1].update(last_step=self.count-1,completed=True,result=result or {})
            self.segment=0
        self.flush();event=threading.Event();self.queue.put(event);self.completed(event)
    def append(self,row):
        phase=row['phase']
        if self.segment and self.segments[self.segment-1]['mode']=='PANEL' and phase!='PANEL_PUSH':self.end()
        if phase not in self.metadata['phases']:self.metadata['phases'].append(phase)
        ft=row['wrist_ft'];bodies=ft['body_states']
        if not self.metadata['body_properties']:
            for b in bodies:
                T=np.asarray(b['T_world_body']);R=T[:3,:3]
                self.metadata['body_properties'].append(dict(name=b['name'],mass_kg=b['mass_kg'],com_body_m=(R.T@(np.asarray(b['com_world_m'])-T[:3,3])).tolist(),inertia_body_kg_m2=(R.T@np.asarray(b['inertia_world_kg_m2'])@R).tolist()))
        if phase=='PANEL_PUSH' and self.router and not self.segment:self.begin(self.router.pop(0),'PANEL')
        ctx=self.context();mode=2 if phase.startswith('PANEL') else (1 if self.segment or max(row['forces_n'].values(),default=0)>0 else 0)
        force=row.get('force_event',{});positive=sum(c['force_n']>0 for c in row['contacts'])
        x=dict(step=np.uint32(self.count),t=np.float64(row['t']),q=np.asarray(row['q'],dtype=np.float32),dq=np.asarray(row['joint_velocities'],dtype=np.float32),
            command_q=np.asarray(row['command_q_arm'],dtype=np.float32),command_dq=np.asarray(row['command_q_velocity_arm'],dtype=np.float32),
            door_q=np.float64(np.deg2rad(row['door_angle_deg'])),door_dq=np.float64(row['telemetry']['hinge_velocity_rad_s']),
            raw_ft=np.asarray(ft['raw_wrench_sensor'],dtype=np.float64),sensor_pose=pose7(ft['T_world_sensor']),tool_poses=np.array([pose7(b['T_world_body']) for b in bodies]),tcp_pose=pose7(row['T_tcp']),
            reference_q=np.float64(ctx.get('current_articulation_reference',np.nan)),aperture=np.float32(row['aperture_m']),command_aperture=np.float32(row['command_aperture_m']),
            pad_load=np.array(list(row['forces_n'].values()),dtype=np.float32),relative_slip_m=np.float32(row['relative_translation_slip_m']),phase=np.uint8(self.metadata['phases'].index(phase)),
            mode=np.uint8(mode),segment=np.uint16(self.segment),ft_valid=np.uint8(ft['valid']),safety_stop=np.uint8(bool(force.get('stop') or row.get('stop'))),
            load_warning=np.uint8(force.get('status')=='INFORMATIONAL_LOAD_WARNING'),positive_contacts=np.uint16(positive))
        high=bool(self.segment or positive or mode==2 or force.get('status','NORMAL')!='NORMAL' or phase in ('LOW_PRELOAD','FORCE_HOLD','RELEASE'))
        self.pre.append([x,high or self.count%24==0]);self.count+=1
        if high:
            for item in self.pre:item[1]=True
        if len(self.pre)>60:
            old,keep=self.pre.popleft()
            if keep:self.pending.append(old)
        if len(self.pending)>=self.chunk_records:self._flush_pending()
        if phase=='PANEL_PUSH' and self.segment and self.router:
            target=self.segments[self.segment-1]['target_deg']
            if row['door_angle_deg']<=target:self.end(dict(boundary_crossed_deg=row['door_angle_deg']))
    def _flush_pending(self):
        if self.pending:self.queue.put(self.pending);self.pending=[]
    def flush(self):
        while self.pre:
            x,keep=self.pre.popleft()
            if keep:self.pending.append(x)
        self._flush_pending()
    def close(self):
        self.flush();self.queue.put(None);self.thread.join(timeout=30)
        if self.thread.is_alive():self.issues.append('COMPACT_EXPORT_INCOMPLETE')
        self._index()
        return dict(format=self.metadata['format'],records=sum(c['records'] for c in self.chunks),physics_steps_seen=self.count,native_rate_hz=240,chunks=self.chunks,issues=self.issues,raw_private=True)

def storage(folder):
    counts={}
    for f in Path(folder).rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(folder);category=('numeric' if rel.parts[0]=='telemetry' else 'observations' if rel.parts[0]=='rgbd' else 'clips' if f.suffix=='.mp4' else 'geometry' if rel.parts[0] in ('agent_geometry','incremental_feedback') else 'source_and_metadata')
        counts[category]=counts.get(category,0)+f.stat().st_size
    return dict(bytes_by_category=counts,total_bytes=sum(counts.values()),budget_bytes=1000000000)

def prune_optional(folder):
    """Space pressure affects optional views only, never numeric measurements."""
    import shutil
    root=Path(folder);s=storage(root)
    if s['total_bytes']>800000000:
        versions=sorted((root/'incremental_feedback').glob('*'))
        for p in versions[:-1]:shutil.rmtree(p,ignore_errors=True)
        for p in (root/'agent_geometry').glob('*.png'):p.unlink(missing_ok=True)
        s=storage(root)
    return s

def compact_checkpoint_files(folder):
    """After execution only: retain raw checkpoint pixels once, prune intermediates."""
    import shutil
    root=Path(folder)
    for view in (root/'rgbd').glob('*'):
        files=list(view.glob('*.npy'))
        if files:
            keep={'depth_m','handle_mask','sam3_seed_mask','robot_self_exclusion','partial_cue_mask'}
            arrays={f.stem:np.load(f,allow_pickle=False) for f in files if f.stem in keep or f.stem.startswith('cue_')}
            np.savez_compressed(view/'checkpoint_arrays.npz',**arrays)
            for f in files:f.unlink()
            atomic_json(view/'CHECKPOINT_STORAGE.json',dict(file='checkpoint_arrays.npz',keys=list(arrays),lossless=True,rgb='rgb.png',calibration='camera.json'))
        # Reconstructible point-cloud/mask renderings and code copies are not raw observations.
        for f in view.iterdir():
            if f.is_dir():shutil.rmtree(f,ignore_errors=True)
            elif f.name not in ['rgb.png','camera.json','checkpoint_arrays.npz','CHECKPOINT_STORAGE.json','INSPECTION.json','candidate_cues.json','visual_detection_attempts.json','visual_correction.json']:
                f.unlink()
    for name in ['incremental_feedback','attempts']:
        shutil.rmtree(root/name,ignore_errors=True)
    for name in ['cooked_initial.json','stage_prims.json','native_authored_assembly_private.json']:
        (root/name).unlink(missing_ok=True)
    # Keep final observed geometry and selected plan; discard re-creatable candidate diagnostics.
    for f in (root/'agent_geometry').glob('*'):
        if f.is_file() and f.suffix in ['.png','.npy'] :f.unlink()
    atomic_json(root/'STORAGE_SUMMARY.json',storage(root))
