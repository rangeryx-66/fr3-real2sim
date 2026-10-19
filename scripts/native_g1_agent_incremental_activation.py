"""Activate semantic uncertainty inputs in the existing bounded Agent interface."""
import types
from pathlib import Path
import native_g1_agent_incremental as previous
import native_g1_incremental_semantics as semantic

_feedback=types.FunctionType(previous.feedback.__code__,dict(previous.__dict__,incremental=semantic))

def feedback(g):
    result=_feedback(g)
    if result.get('geometry'):
        result['semantic_image_board']=str(Path(result['geometry_folder'])/'semantic_surfaces.png')
    return result

observe=types.FunctionType(previous.observe.__code__,dict(previous.__dict__,feedback=feedback))
build_geometry=types.FunctionType(previous.build_geometry.__code__,dict(previous.__dict__,incremental=semantic))
# The previous callback was itself bound to the observation module's globals.
# Preserve that namespace (including actuate), not the adapter module's dict.
move_and_observe=types.FunctionType(previous.move_and_observe.__code__,dict(previous.move_and_observe.__globals__,observe=observe))
prepare=types.FunctionType(previous.prepare.__code__,dict(previous.__dict__,
    observe=observe,build_geometry=build_geometry,move_and_observe=move_and_observe))
