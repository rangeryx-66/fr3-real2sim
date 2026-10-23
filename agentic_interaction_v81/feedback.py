"""Measured feedback with separate servo and articulation-relative errors.

The frozen generic renderer's drawer/rail error panel subtracts radians from
metres on revolute runs. This versioned display fixes that dimensional error;
it never changes telemetry, estimation, contact decisions or safety channels.
"""
import hashlib
import json
from pathlib import Path


def motion_errors(command, measured, coordinate, prior, anchor=0):
    import numpy as np
    from scipy.spatial.transform import Rotation
    command, measured = np.asarray(command), np.asarray(measured)
    coordinate = np.asarray(coordinate)
    axis = np.asarray(prior['axis_world'], float)
    if (command.shape != measured.shape or measured.shape != (len(coordinate), 3)
            or not 0 <= anchor < len(coordinate) or axis.shape != (3,)
            or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12
            or not np.isfinite(measured[anchor]).all() or not np.isfinite(coordinate[anchor])):
        raise ValueError('MEASURED_XYZ_NATIVE_COORDINATE_AND_NONZERO_AXIS_REQUIRED')
    axis /= np.linalg.norm(axis)
    delta = coordinate - coordinate[anchor]
    if prior['joint_type'] == 'revolute':
        origin = np.asarray(prior['origin_world_m'])
        rotation = Rotation.from_rotvec(delta[:, None] * axis).as_matrix()
        expected = origin + np.einsum('nij,j->ni', rotation, measured[anchor] - origin)
    elif prior['joint_type'] == 'prismatic':
        expected = measured[anchor] + delta[:, None] * axis
    else:
        raise ValueError('KNOWN_REVOLUTE_OR_PRISMATIC_PRIOR_REQUIRED')
    servo = command - measured
    return dict(command_minus_measured_world_m=servo,
                incremental_servo_error_world_m=servo-servo[anchor],
                measured_minus_rigid_articulation_world_m=measured-expected)


def render(folder, *, attempt=None, attempt_id=None):
    import numpy as np
    from diagnose_probing_v3 import columns, rows_at
    from xarm_wrist_ft import wrench_at
    from agent_command_protocol_v12 import atomic_json
    folder = Path(folder)
    if attempt is None:
        trace = folder/'AGENT_ACTION_TRACE.json'
        entries = json.loads(trace.read_text()) if trace.exists() else []
        physical = [e['result'] for e in entries if e['command']['operation'] == 'attempt_grasp_and_probe'
                    and e['result'].get('physical_grasp_started')]
        if not physical:
            return dict(available=False, reason='NO_CURRENT_PHYSICAL_ATTEMPT')
        attempt, attempt_id = physical[-1], len(physical)
    index = json.loads((folder/'telemetry/index.json').read_text())
    settings = json.loads((folder/'identification_input.json').read_text())
    if settings['prior']['joint_type'] != 'revolute':
        return dict(available=False, reason='USE_EXISTING_PRISMATIC_ATTEMPT_RENDERER')
    if type(attempt_id) is not int or attempt_id < 1:
        raise ValueError('EXPLICIT_CURRENT_ATTEMPT_ID_REQUIRED')
    first, last = attempt['first_record'], attempt['last_record']
    lineage = dict(attempt_id=attempt_id, first_step=first, last_step_exclusive=last,
                   observation_generation=attempt.get('observation_generation_v81'),
                   station_generation=attempt.get('station_generation'))
    chunks = [c['sha256'] for c in index['chunks'] if c['first_step'] < last and c['last_step'] >= first]
    signature = hashlib.sha256(json.dumps(dict(lineage=lineage, chunks=chunks, settings=settings,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), sort_keys=True).encode()).hexdigest()
    out = folder/'contact_feedback_v81'/('attempt_%02d'%attempt_id)
    manifest, image = out/'INSPECTION.json', out/'CONTACT_FEEDBACK.png'
    if manifest.exists() and image.exists():
        previous = json.loads(manifest.read_text())
        if previous.get('signature') == signature:
            return dict(previous, cached_same_measurement=True)
    a, index = columns(folder)
    eligible = (a['step'] >= first) & (a['step'] < last)
    active = np.flatnonzero(eligible & (a['segment'] > 0))
    if len(active):
        sid = int(a['segment'][active[-1]])
        active = np.flatnonzero(eligible & (a['segment'] == sid))
        start, end = active[0], active[-1]
    else:
        active = np.flatnonzero(eligible & (a['positive_contacts'] > 0))
        if not len(active):
            return dict(available=False, reason='NO_CONTACT_SAMPLES_IN_THIS_ATTEMPT', **lineage)
        sid = 0
        start, end = active[0], active[-1]
    ix = np.flatnonzero(eligible & (a['t'] >= a['t'][start]-2) & (a['t'] <= a['t'][end]+.5))
    anchor = int(np.flatnonzero(ix == start)[0])
    errors = motion_errors(a['command_TCP_pose'][ix, :3], a['tcp_pose'][ix, :3],
                           a['joint_q'][ix], settings['prior'], anchor)
    t = a['t'][ix]-a['t'][start]
    rows = rows_at(a, ix, index['metadata'], 'AGENT_DIAGNOSTIC_ONLY')
    wrench, effort = [], []
    axis = np.asarray(settings['prior']['axis_world']); axis /= np.linalg.norm(axis)
    origin = np.asarray(settings['prior']['origin_world_m'])
    for row in rows:
        ft = row['wrist_ft']; T = np.asarray(ft['T_world_sensor'])
        w = wrench_at(np.asarray(ft['raw_wrench_sensor'])-settings['bias'], T, T[:3, 3]) + ft['tool_gravity_wrench_world']
        wrench.append(w)
        effort.append(float(axis @ (w[3:]+np.cross(T[:3, 3]-origin, w[:3]))))
    wrench = np.asarray(wrench)
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
    for j, name in enumerate(['L raw normal', 'R raw normal']): ax[0].plot(t, a['pad_load'][ix, j], label=name, lw=.7)
    ax[0].set_ylabel('Pad normal (N)')
    for field, label in [('aperture', 'measured'), ('command_aperture', 'command')]: ax[1].plot(t, a[field][ix]*1000, label=label)
    ax[1].set_ylabel('Aperture (mm)')
    factor = 180/np.pi
    ax[2].plot(t, a['joint_q'][ix]*factor, label='measured'); ax[2].plot(t, a['reference_q'][ix]*factor, label='reference')
    ax[2].set_ylabel('Hinge (deg)')
    for field, label in [('command_minus_measured_world_m', 'absolute servo error'), ('incremental_servo_error_world_m', 'change from segment start')]:
        ax[3].plot(t, np.linalg.norm(errors[field], axis=1)*1000, label=label)
    ax[3].set_ylabel('TCP tracking (mm)')
    relative = np.linalg.norm(errors['measured_minus_rigid_articulation_world_m'], axis=1)
    ax[4].plot(t, relative*1000, label='measured TCP minus rigid hinge arc')
    ax[4].set_ylabel('Relative motion (mm)')
    for j, name in enumerate(['Fx', 'Fy', 'Fz']): ax[5].plot(t, wrench[:, j], label=name, lw=.7)
    ax[5].set_ylabel('Sensor-origin force (N)')
    for j, name in enumerate(['Tx sensor origin', 'Ty sensor origin', 'Tz sensor origin']): ax[6].plot(t, wrench[:, j+3], label=name, lw=.7)
    ax[6].plot(t, effort, label='full wrench shifted/projected at hinge', lw=1)
    ax[6].set_ylabel('Moment (Nm)'); ax[6].set_xlabel('Seconds from current attempt interaction start')
    for plot in ax: plot.axvline(0, color='gray', ls=':'); plot.grid(alpha=.2); plot.legend(fontsize=7)
    fig.suptitle('Current-attempt revolute feedback | bias + gravity compensation; tool inertia retained\nSeparate Cartesian servo error and rigid-arc residual; no radians/metres subtraction or inferred pad separation')
    fig.tight_layout(); out.mkdir(parents=True, exist_ok=True); fig.savefig(image, dpi=120); plt.close(fig)
    def maximum(values):
        values = np.asarray(values)
        finite = values[np.isfinite(values)]
        return float(finite.max()) if len(finite) else None
    geometry=dict(available=False,reason='No current-attempt native point record; no prior-attempt fallback')
    if (folder/'contact_evidence/index.json').exists():
        from render_prismatic_feedback_v75 import contact_geometry
        geometry=contact_geometry(folder,out,dict(attempt_id=attempt_id,native_point_attempt_id=attempt_id,
            first_step=first,last_step_exclusive=last))
    result = dict(available=True, path=str(image), signature=signature, **lineage,
        segment=sid, plot_samples=len(ix), measured_only=True, previous_attempt_fallback=False,
        legacy_revolute_rail_panel_replaced=True,
        force_processing='bias/gravity compensation only, dynamic tool inertia retained; diagnostic not protection/estimation channel',
        raw_normal_peaks_n=a['pad_load'][ix].max(0).tolist(),
        max_absolute_tracking_m=maximum(np.linalg.norm(errors['command_minus_measured_world_m'][anchor:], axis=1)),
        max_incremental_tracking_m=maximum(np.linalg.norm(errors['incremental_servo_error_world_m'][anchor:], axis=1)),
        max_rigid_arc_residual_m=maximum(relative[anchor:]),
        max_relative_drift_m=maximum(relative[anchor:]),
        joint_travel_native=float(np.ptp(a['joint_q'][active])),
        contact_geometry=geometry)
    atomic_json(manifest, result)
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('episode', type=Path)
    args = parser.parse_args()
    print(json.dumps(render(args.episode), indent=2))
