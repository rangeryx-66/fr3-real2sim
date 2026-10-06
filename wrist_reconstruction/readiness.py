"""Both-object backend gate and honest independent task result tables."""
import json,csv,subprocess,sys
from pathlib import Path

def both_ready(out,c):
    rows=[]
    for o in c['objects']:
        p=Path(out)/('capture_'+o['id'])/'multistate_capture.json'
        states=json.loads(p.read_text())['states'] if p.exists() else []
        clean=[s for s in states if s.get('clean_wrist_capture') and len(s.get('views',[]))>=8]
        span=max([s['estimated_articulation_state'] for s in clean] or [0])-min([s['estimated_articulation_state'] for s in clean] or [0])
        required=c['backend']['minimum_pair_span']['revolute' if o['id']=='7320' else 'prismatic']
        evaluation=Path(out)/('capture_'+o['id'])/'maximum_range_evaluation.json'
        actual=json.loads(evaluation.read_text()).get('maximum_actual_state') if evaluation.exists() else None
        # Label span alone cannot stand in for actual physical object motion.
        ready=len(clean)>=c['backend']['minimum_clean_states'] and span>=required and actual is not None and actual>=required
        rows.append({'object':o['id'],'clean_states':len(clean),'label_span':span,'actual_maximum':actual,'ready':ready})
    return all(r['ready'] for r in rows),rows

def summarize(out):
    lines=['','## Maximum-range physical results (not reconstruction acceptance)','']
    for obj in ['7320','45746']:
        root=Path(out)/('capture_'+obj)
        def read(name,default):
            p=root/name;return json.loads(p.read_text()) if p.exists() else default
        if (root/'effort_segments.json').exists() and (root/'effort_samples.jsonl').exists():
            interpreter=Path(__file__).resolve().parents[1]/'environments/artgs/bin/python'
            command=[str(interpreter) if interpreter.exists() else sys.executable,'-c','from wrist_reconstruction.effort_summary import summarize;import sys;summarize(sys.argv[1])',str(root)]
            subprocess.run(command,cwd=Path(__file__).resolve().parents[1],check=True,timeout=300)
        actual=read('maximum_range_evaluation.json',{});history=read('reposition_history.json',[])
        capture=read('multistate_capture.json',{});effort=read('effort_segments.json',[]);moves=read('mobile_progress.json',{}).get('repositions',0)
        lines += [f'### {obj}','',f"Actual maximum: {actual.get('maximum_actual_state','PENDING independent post-run evaluation')}; units: {'degrees' if obj=='7320' else 'meters'}. Mobile moves: {moves}; regrasp successes: {sum(bool(x.get('regrasp_completed')) for x in history)}; effort-valid segments: {sum(x.get('effort_validity')=='VALID_EFFORT_SEGMENT' for x in effort)}.",'','| State | Release | Scan | Wrist reobserve | Regrasp | Subsequent pull |','|---|---|---|---|---|---|']
        rows=[]
        for h in history:
            row={'state_estimate':h.get('start_state_estimated'),'release':h.get('released',False),'scan':h.get('wrist_scan_completed','not requested'),'reobserve':any('current_wrist_observation' in a for a in h.get('regrasp_attempts',[])),'regrasp':h.get('regrasp_completed',False),'subsequent_pull':'see observed_progress.jsonl and evaluation trajectory','status':h.get('status')}
            rows.append(row);lines.append('| '+' | '.join(str(row[k]) for k in ['state_estimate','release','scan','reobserve','regrasp','subsequent_pull'])+' |')
        if root.exists():
            with (root/'state_table.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=['state_estimate','release','scan','reobserve','regrasp','subsequent_pull','status']);w.writeheader();w.writerows(rows)
        lines+=['',f"Clean captures: {[(s.get('estimated_articulation_state'),len(s.get('views',[]))) for s in capture.get('states',[]) if s.get('clean_wrist_capture')]}."]
    return lines
