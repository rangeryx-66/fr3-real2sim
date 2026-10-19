"""Bounded control history with lossless disk journal for long-lived actors."""
import json
from pathlib import Path

class JournalList(list):
    def __init__(self,path,retain=4096):
        super().__init__();self.stream=Path(path).open('a');self.retain=retain;self.total=0;self.minimum_margin=None;self.peak_force=0.
    def append(self,row):
        self.stream.write(json.dumps(row)+'\n');self.total+=1
        if self.total%240==0:self.stream.flush()
        margin=row.get('margin_rad')
        if margin is not None:self.minimum_margin=margin if self.minimum_margin is None else min(self.minimum_margin,margin)
        self.peak_force=max(self.peak_force,max(row.get('forces_n',{}).values(),default=0))
        super().append(row)
        if len(self)>self.retain:del self[:len(self)-self.retain]
    def close(self):self.stream.flush();self.stream.close()

class EvaluationJournal:
    """Write-only evaluator; never returns object observations to the actor."""
    def __init__(self,scene,output):
        from paper_structure.evaluation_logger import EvaluationLogger
        self.logger=EvaluationLogger(scene,output);self.output=Path(output);self.output.mkdir(exist_ok=True)
        self.logger._rows=JournalList(self.output/'object_trajectory.jsonl',retain=2)
    def append(self,*args):self.logger.append(*args)
    def close(self):
        self.logger._rows.close()
        (self.output/'scope.json').write_text(json.dumps({'evaluation_only':True,'controller_readback':False,'GT_inputs_to_fitter':False,'format':'object_trajectory.jsonl'}))
