"""Isolated V7.5 engineering entry; never activates a registered benchmark."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceRuntimeV75
from interaction_runtime.prismatic_session_v75_r3 import serve_v75_r3
from interaction_runtime.prismatic_v73 import audit_frozen
from run_angle_resistance_v5 import SOURCES


class PrismaticRuntimeV75R3(PrismaticEvidenceRuntimeV75):
    def __init__(self,job):
        if job.get('engineering_revision')!='V7.5-R3':raise ValueError('Explicit separately registered R3 engineering only')
        super().__init__(job)

if __name__=='__main__':
    audit_frozen(ROOT)
    sys.exit(main(PrismaticRuntimeV75R3,serve_v75_r3,extra_sources=SOURCES+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/native_g1_prismatic_ranking.py','scripts/prismatic_visual_v73.py',
        'scripts/prismatic_quality_v73.py','scripts/prismatic_inspection_v73.py','scripts/update_prismatic_v73_model.py',
        'scripts/render_prismatic_feedback_v75.py','scripts/run_prismatic_v75_r3.py',
        'configs/prismatic_preload_v71.json','configs/sensor_lighting_v1.json','configs/prismatic_execution_v75.json')))
