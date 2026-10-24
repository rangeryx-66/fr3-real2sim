"""Explicit V8.1 lifecycle precondition; frozen step/clock guard unchanged."""


def resume_for_exit(runtime):
    from interaction_runtime.policy import SafetyStop
    if runtime.supervisor.latched is not None:
        raise SafetyStop('LATCHED_BEFORE_EXIT', runtime.supervisor.latched)
    runtime.world.play()
