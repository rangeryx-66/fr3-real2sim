"""Buffered lossless binary evidence. No per-step JSON/image/file creation."""
from collections import defaultdict, deque
from contextlib import contextmanager
import gzip, hashlib, json, os, pickle, queue, threading, time
from pathlib import Path
import numpy as np

class Profile:
    def __init__(self):
        self.cost=defaultdict(float);self.inclusive=defaultdict(float);self.calls=defaultdict(int);self.started=time.perf_counter();self.phase='INITIALIZE';self.phases=defaultdict(float);self.stack=[]
    @contextmanager
    def measure(self,name):
        item=[name,time.perf_counter(),0.,self.phase];self.stack.append(item)
        try:yield
        finally:
            elapsed=time.perf_counter()-item[1];self.stack.pop();exclusive=elapsed-item[2]
            self.cost[name]+=exclusive;self.inclusive[name]+=elapsed;self.calls[name]+=1;self.phases[item[3]]+=exclusive
            if self.stack:self.stack[-1][2]+=elapsed
    def report(self,simulation_s):
        wall=time.perf_counter()-self.started
        return dict(wall_s=wall,simulation_s=simulation_s,simulation_per_wall_s=simulation_s/max(wall,1e-9),cost_wall_s=dict(self.cost),inclusive_cost_wall_s=dict(self.inclusive),calls=dict(self.calls),phase_accounted_wall_s=dict(self.phases),unaccounted_wall_s=wall-sum(self.cost.values()),cost_semantics='exclusive nested timers; inclusive totals separately, no double counting')

class Recorder:
    def __init__(self,folder,chunk_records=2048):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True);self.chunk_records=chunk_records
        self.pending=[];self.chunks=[];self.count=0;self.issues=[];self.failed=[];self.queue=queue.Queue()
        self.worker=threading.Thread(target=self._worker,name='interaction-evidence',daemon=True);self.worker.start()
    def _worker(self):
        while True:
            item=self.queue.get()
            try:
                if item is None:return
                first,rows=item;target=self.folder/('%08d.pkl.gz'%first)
                # Pickle5 keeps native numeric arrays binary, including nonfinite
                # optional diagnostics. No JSON serialization reaches control.
                temporary=target.with_suffix('.pending')
                with gzip.open(temporary,'wb',compresslevel=1) as f:pickle.dump(rows,f,protocol=5)
                os.replace(temporary,target)
                self.chunks.append(dict(file=target.name,first_record=first,records=len(rows),bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
            except Exception as e:
                self.issues.append(repr(e))
                # Keep failed chunks in memory; diagnostic failure is never a
                # control exception. Owner close can retry/export the buffer.
                if item is not None:self.failed.append(item)
            finally:self.queue.task_done()
    def append(self,row):
        self.pending.append(row);self.count+=1
        if len(self.pending)>=self.chunk_records:self.flush()
    def flush(self):
        if not self.pending:return
        rows=self.pending;self.pending=[];self.queue.put((self.count-len(rows),rows))
    def close(self):
        self.flush();self.queue.put(None);self.worker.join(timeout=15)
        if self.worker.is_alive():self.issues.append("EXPORT_WORKER_TIMEOUT_EVIDENCE_INCOMPLETE")
        index=dict(format='gzip1 / pickle5 / native numpy arrays',native_rate_hz=240,records=self.count,chunks=sorted(self.chunks,key=lambda x:x['first_record']),issues=self.issues,raw_private=True)
        (self.folder/'index.json').write_text(json.dumps(index,indent=2));return index

class RenderSchedule:
    def __init__(self,mode,configuration):
        self.mode=mode;self.c=configuration;self.until=-1.;self.next=-1.;self.requests=0
    def event_clip(self,t):self.until=max(self.until,t+self.c['event_clip_s'])
    def due(self,t):
        active=self.c['continuous_video'] or t<self.until
        if active and t>=self.next:
            self.next=t+1/self.c['video_hz'];self.requests+=1;return True
        return False
