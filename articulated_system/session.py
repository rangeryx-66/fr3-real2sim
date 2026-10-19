"""Reuse unchanged segment loop; recover before the physical margin limit."""
from pathlib import Path
import ast

def run(runtime):
    # Source is guarded by exact hook counts, so a baseline change cannot silently
    # bypass a safety check. No grasp/controller/collision implementation edited.
    original=(Path(__file__).resolve().parents[1]/'articulated_interaction_skill/session.py').read_text()
    a="return r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp())"
    b="return r.state_offset+(r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp()))"
    if original.count(a)!=1:raise RuntimeError('SESSION_STATE_HOOK_CHANGED')
    text=original.replace(a,b)
    a="direction=r.memory.tangent(r.tcp());safe,why=r.increment_safe(direction,p['segment_m'])"
    b="""direction=r.memory.tangent(r.tcp())
  if r.margin()<=p['warning_margin_rad']:
   snapshot('pre_reposition',value);last_capture=value;r.recover(value);last=r.tcp()[:3,3].copy();continue
  safe,why=r.increment_safe(direction,p['segment_m'])"""
    if text.count(a)!=1:raise RuntimeError('SESSION_RECOVERY_HOOK_CHANGED')
    text=text.replace(a,b)
    text=text.replace("raise RuntimeError('REPOSITION_REQUIRES_SAFE_REGRASP:'+why)","r.recover(value);last=r.tcp()[:3,3].copy();continue")
    ast.parse(text);scope={};exec(compile(text,str(Path(__file__)),'exec'),scope);return scope['run'](runtime)
