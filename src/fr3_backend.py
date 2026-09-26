"""Explicit FR3 entry point; kept for downstream skill integrations."""
import os
os.environ.setdefault('GRASP_ROBOT','fr3')
from backend import Backend
class FR3Backend(Backend):
    pass
