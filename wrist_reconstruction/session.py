"""Wrist orchestration bookkeeping; contact and segment control stay unchanged."""
import ast
from pathlib import Path


def run(runtime):
    original=(Path(__file__).resolve().parents[1]/'articulated_interaction_skill/session.py').read_text()
    replacements={
        "return r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp())":
        "return r.state_offset+(r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp()))",
        " def snapshot(label,value):":
        " def snapshot(label,value):\n  nonlocal last",
        "  if not ready:raise RuntimeError('CAPTURE_HOLD_GRASP_LOST')":
        "  if not ready:raise RuntimeError('CAPTURE_HOLD_GRASP_LOST')\n  # Released scan motion is not manipulation travel.\n  last=r.tcp()[:3,3].copy()",
        "direction=r.memory.tangent(r.tcp());safe,why=r.increment_safe(direction,p['segment_m'])":
        """direction=r.memory.tangent(r.tcp())
  if r.margin()<=p['warning_margin_rad']:
   # capture already performs release/retreat/route/regrasp once.
   snapshot('pre_reposition',value);last_capture=value;continue
  safe,why=r.increment_safe(direction,p['segment_m'])""",
        "raise RuntimeError('REPOSITION_REQUIRES_SAFE_REGRASP:'+why)":
        "r.recover(value);last=r.tcp()[:3,3].copy();continue",
    }
    text=original
    for before,after in replacements.items():
        if text.count(before)!=1:raise RuntimeError('WRIST_SESSION_HOOK_CHANGED:'+before)
        text=text.replace(before,after)
    ast.parse(text);scope={};exec(compile(text,str(Path(__file__)),'exec'),scope)
    return scope['run'](runtime)
