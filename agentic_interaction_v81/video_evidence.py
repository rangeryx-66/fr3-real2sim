"""Explicit continuous evidence mode; physical policy and owners unchanged."""
from agent_command_protocol_v12 import atomic_json


def configure(runtime):
    hz = runtime.job.get('v83_continuous_video_hz')
    if not hz or getattr(runtime, '_v83_video_configured', False):
        return
    if hz != 5:
        raise ValueError('REGISTERED_V83_VIDEO_RATE_REQUIRED')
    previous = dict(runtime.policy.data['modes'][runtime.mode])
    current = dict(previous, continuous_video=True, video_hz=hz)
    runtime.policy.data['modes'][runtime.mode] = current
    runtime.render.c = current
    runtime._v83_video_configured = True
    atomic_json(runtime.output / 'V83_VIDEO_EVIDENCE.json', dict(
        original_policy_sha256=runtime.policy.sha256, previous_mode=previous,
        recording_mode=current, physical_policy_changed=False,
        event_only_frame_cap_bypassed=True, private_evaluation_only=True,
        coverage='All advancing simulation steps at 5 Hz; paused planning has no new physical state',
        camera='Existing diagnostic overview, excluded from online decisions'))
