"""Explicit registered-prismatic entry; activate only after engineering admission.

No revolute runtime, controller, estimator or physical parameter is replaced.
The launcher verifies the original blind commitment and versioned activation.
"""
from .prismatic_preload_v71 import PrismaticPreloadRuntime


class RegisteredPrismaticRuntimeV71(PrismaticPreloadRuntime):
    ALLOW_REGISTERED_PRISMATIC=True

    def __init__(self,job):
        if (job.get('experiment')!='V7_BENCHMARK' or not job.get('benchmark_registration')
                or not job.get('registered_episode_id') or job.get('preload_fixture')
                or job.get('engineering_microtracking') or job.get('engineering_pregrasp_reference')):
            raise ValueError('Registered fresh prismatic benchmark job required; no engineering fixture')
        super().__init__(job)
